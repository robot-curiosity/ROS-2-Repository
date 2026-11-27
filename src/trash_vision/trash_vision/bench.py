"""Bench test: ros2 run trash_vision bench <drive|arm|sonar|camera|all>.

Opens the hardware directly (stop the ROS nodes first) using the values in
config/robot.yaml. The space bar is a software E-stop; keep a hand on the
power switch anyway. ``drive`` needs ``--wheels-off``.
"""
import argparse
import contextlib
import os
import select
import signal
import sys
import time

import yaml

from trash_vision import arm_logic as al
from trash_vision import bench_logic as bl
from trash_vision import config_logic as cl
from trash_vision import drive_logic as dl
from trash_vision import ground_projection as gp
from trash_vision import hardware_check_logic as hl
from trash_vision import sonar_scheduler as ss
from trash_vision import ultrasonic_logic as ul
from trash_vision.hardware_check import default_config_path

SUBSYSTEMS = ['sonar', 'drive', 'arm', 'camera']


class TerminalKeys:
    """Non-blocking single key reads from a terminal in cbreak mode."""

    def __init__(self, fd):
        """Read from file descriptor ``fd``."""
        self.fd = fd

    def read(self, timeout=0.0):
        """Return every pending character, waiting up to ``timeout``."""
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return ''
        data = os.read(self.fd, 1024)
        if not data:
            # Terminal gone: treat like the E-stop.
            return bl.ESTOP_KEY
        return data.decode(errors='ignore')


@contextlib.contextmanager
def terminal():
    """Cbreak mode on stdin; signals become one KeyboardInterrupt."""
    import termios
    import tty
    signals = [signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT]

    def stop(signum, frame):
        # Ignore repeats so a second Ctrl-C cannot interrupt the cleanup.
        for s in signals:
            signal.signal(s, signal.SIG_IGN)
        raise KeyboardInterrupt

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    old = {s: signal.signal(s, stop) for s in signals}
    try:
        tty.setcbreak(fd)
        yield TerminalKeys(fd)
    finally:
        with contextlib.suppress(Exception):
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        for s, handler in old.items():
            signal.signal(s, handler)


def out(text):
    """Print and flush; a closed terminal must not stop the cleanup."""
    with contextlib.suppress(OSError):
        print(text, flush=True)


# ---------------------------------------------------------------- drive

class GpioMotors:
    """The two MDD10A channels (and encoders if set) through gpiozero."""

    def __init__(self, params):
        """Open the pins from the drive section."""
        from gpiozero import DigitalInputDevice, DigitalOutputDevice
        from gpiozero import PWMOutputDevice
        self._devices = []
        self._pwm = {}
        self._dir = {}
        self._dec = {}
        try:
            for i, side in enumerate(bl.SIDES):
                self._pwm[side] = self._keep(PWMOutputDevice(
                    params['pwm_pins'][i], frequency=1000))
                self._dir[side] = self._keep(DigitalOutputDevice(
                    params['dir_pins'][i]))
                self._open_encoder(i, side, params, DigitalInputDevice)
        except Exception:
            self.close()
            raise

    def _keep(self, device):
        self._devices.append(device)
        return device

    def _open_encoder(self, i, side, params, input_device):
        a_pins = params.get('enc_a_pins') or [-1, -1]
        b_pins = params.get('enc_b_pins') or [-1, -1]
        if a_pins[i] < 0 or b_pins[i] < 0:
            return
        a = self._keep(input_device(a_pins[i], pull_up=True))
        b = self._keep(input_device(b_pins[i], pull_up=True))
        dec = dl.QuadratureDecoder(a.value, b.value)
        sign = -1 if params.get('invert_enc_' + side) else 1
        self._dec[side] = (dec, sign)

        def changed():
            dec.update(a.value, b.value)

        a.when_activated = a.when_deactivated = changed
        b.when_activated = b.when_deactivated = changed

    def set(self, side, duty, forward):
        """Set direction, then duty, for one side."""
        self._dir[side].value = forward
        self._pwm[side].value = duty

    def counts(self, side):
        """Signed encoder count, or None without an encoder."""
        if side not in self._dec:
            return None
        dec, sign = self._dec[side]
        return sign * dec.count

    def off(self):
        """Zero every PWM output, even if one of them fails."""
        error = None
        for device in self._pwm.values():
            try:
                device.value = 0.0
            except Exception as exc:
                error = exc
        if error is not None:
            raise error

    def close(self):
        """Zero the outputs and release the pins."""
        with contextlib.suppress(Exception):
            self.off()
        for device in self._devices:
            with contextlib.suppress(Exception):
                device.close()


def drive_pin_errors(config):
    """Problems with the pins the motor test would use."""
    params = cl.node_params(config, 'drive')
    errors = []
    for name in ('pwm_pins', 'dir_pins'):
        value = params.get(name)
        if (not isinstance(value, list) or len(value) != 2
                or not all(isinstance(v, int) and v >= 0 for v in value)):
            errors.append('unset or wrong length: drive.' + name)
    pins = hl.collect_pins(config)
    mine = [p for p in pins if p[1].startswith('drive.')]
    errors += hl.check_pin_range(mine)
    errors += hl.check_conflicts(pins, {'drive'})
    return errors


def bench_drive(config, args, keys):
    """Run each motor channel alone at low duty."""
    if not args.wheels_off:
        out('Refused: put the robot on a stand and pass --wheels-off.')
        return bl.FAIL
    params = cl.node_params(config, 'drive')
    errors = drive_pin_errors(config)
    if errors:
        out('\n'.join(errors))
        return bl.SKIP if params.get('pwm_pins') == [-1, -1] else bl.FAIL
    duty = bl.bench_duty(args.duty, params.get('min_duty', 0.0))
    out('Pi GPIO backend only (not the Pico). Wheels off the ground.')
    inverted = {s: bool(params.get('invert_' + s)) for s in bl.SIDES}
    motors = GpioMotors(params)
    try:
        return bl.motor_test(motors, duty, inverted, keys, out,
                             time.monotonic, time.sleep)
    finally:
        motors.close()


# ------------------------------------------------------------------ arm

def bench_arm(config, args, keys):
    """Jog the arm joints from the keyboard."""
    params = cl.node_params(config, 'arm')
    if cl.is_unset(params.get('port')):
        out('arm.port not set')
        return bl.SKIP
    limits = list(zip(params.get('joint_min') or [],
                      params.get('joint_max') or []))
    if len(limits) != len(al.JOINT_NAMES):
        out('arm.joint_min and joint_max need %d values'
            % len(al.JOINT_NAMES))
        return bl.FAIL
    if not bl.confirm(keys, out, 'Connect to the arm? Keep clear of it.'):
        return bl.SKIP
    from trash_vision.arm_node import LeRobotBackend
    backend = LeRobotBackend(
        params['port'], params.get('robot_id', 'curiosity_follower'),
        bool(params.get('use_degrees', True)))
    try:
        return bl.arm_jog(backend, al.JOINT_NAMES, limits, args.step,
                          keys, out)
    finally:
        # Disconnecting releases servo torque on LeRobot's default settings.
        out('Hold the arm: disconnecting may let it drop. Press Enter.')
        with contextlib.suppress(Exception, KeyboardInterrupt):
            while not any(c in keys.read(0.1) for c in '\r\n '):
                pass
        backend.close()


# ---------------------------------------------------------------- sonar

def bench_sonar(config, args, keys):
    """Live readout of every configured sonar."""
    from gpiozero import DigitalInputDevice, DigitalOutputDevice
    params = cl.node_params(config, 'ultrasonic')
    configured, problems = ul.build_sensor_config(
        ul.SENSOR_NAMES, params.get('trig_pins') or [],
        params.get('echo_pins') or [])
    if problems:
        out('\n'.join(problems))
        return bl.FAIL
    if not configured:
        out('no sonar pins filled in')
        return bl.SKIP
    timeout = float(params.get('echo_timeout_s', 0.025))
    sensors = {}
    try:
        for cfg in configured:
            sensors[cfg['name']] = (DigitalOutputDevice(cfg['trig']),
                                    DigitalInputDevice(cfg['echo']))

        def ping(name):
            trig, echo = sensors[name]
            trig.on()
            time.sleep(1e-5)
            trig.off()
            return ss.validate_range(ss.measure_echo(
                lambda: echo.value, time.perf_counter, timeout))

        return bl.sonar_readout(
            list(sensors), ping, keys, out, time.monotonic, time.sleep,
            float(params.get('settle_gap_s', 0.06)), args.seconds)
    finally:
        for pair in sensors.values():
            for device in pair:
                with contextlib.suppress(Exception):
                    device.close()


# --------------------------------------------------------------- camera

def bench_camera(config, args, keys):
    """Show a few frames and their detections as text."""
    import cv2
    from ultralytics import YOLO

    from trash_vision.frame_grabber import FrameGrabber
    p = cl.node_params(config, 'detector')
    cam = gp.camera_from_degrees(
        p.get('camera_x_m', 0.248), p.get('camera_y_m', -0.1575),
        p.get('camera_height_m', 0.642), p.get('camera_pitch_deg', 35.0),
        p.get('camera_hfov_deg', 66.0), p.get('camera_vfov_deg', 41.0))
    model = YOLO(p['model_path'])
    capture = cv2.VideoCapture(p['camera_source'])
    grabber = FrameGrabber(capture.read)
    try:
        if not capture.isOpened():
            out('cannot open camera_source %s' % p['camera_source'])
            return bl.FAIL
        grabber.start()
        out('q = done, space = E-stop.')
        shown = 0
        last = time.monotonic()
        while shown < args.frames:
            if bl.QUIT_KEY in bl.read_keys(keys, 0.05):
                break
            item = grabber.latest.take()
            if item is None:
                if time.monotonic() - last > 5.0:
                    out('no frame for 5 s')
                    return bl.FAIL
                continue
            frame = item[0]
            height, width = frame.shape[:2]
            result = model(source=frame, imgsz=640, verbose=False,
                           conf=float(p.get('confidence', 0.4)))[0]
            found = []
            for box in result.boxes or []:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                found.append((
                    result.names[int(box.cls[0])], float(box.conf[0]),
                    (x1 / width, y1 / height, x2 / width, y2 / height)))
            small = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
                               (64, 20))
            out(bl.ascii_preview(small.tolist(), [d[2] for d in found]))
            out('frame %dx%d, mean brightness %.0f' % (
                width, height, small.mean()))
            out(bl.describe_detections(
                found, lambda u, v: gp.floor_point_to_base(u, v, cam)))
            shown += 1
            last = time.monotonic()
            if shown < args.frames:
                if bl.QUIT_KEY in bl.read_keys(keys, 1.0):
                    break
        return bl.DONE if shown else bl.SKIP
    finally:
        grabber.stop()
        capture.release()


RUNNERS = {
    'sonar': bench_sonar,
    'drive': bench_drive,
    'arm': bench_arm,
    'camera': bench_camera,
}


def build_parser():
    """Command line of the bench tool."""
    p = argparse.ArgumentParser(
        description='Interactive bench test. Space bar = software E-stop. '
        'Stop the ROS nodes first.')
    p.add_argument('subsystem', choices=SUBSYSTEMS + ['all'])
    p.add_argument('--wheels-off', action='store_true',
                   help='confirm the wheels are off the ground (drive)')
    p.add_argument('--duty', type=float, default=bl.DEFAULT_DUTY,
                   help='motor test PWM duty 0..%g' % bl.MAX_DUTY)
    p.add_argument('--step', type=float, default=bl.DEFAULT_JOG_STEP,
                   help='arm jog step per key press, joint units, max %g'
                   % bl.MAX_JOG_STEP)
    p.add_argument('--seconds', type=float, default=None,
                   help='stop the sonar readout after this long')
    p.add_argument('--frames', type=int, default=5,
                   help='camera frames to show')
    p.add_argument('--config', default=None, help='path to robot.yaml')
    return p


def main(argv=None):
    """Run the chosen bench tests."""
    args = build_parser().parse_args(argv)
    names = SUBSYSTEMS if args.subsystem == 'all' else [args.subsystem]
    if args.subsystem == 'drive' and not args.wheels_off:
        print('Refused: put the robot on a stand and pass --wheels-off.')
        return 2
    if not sys.stdin.isatty():
        print('Refused: needs a terminal, otherwise the E-stop key is dead.')
        return 2
    with open(args.config or default_config_path()) as f:
        config = yaml.safe_load(f)
    with terminal() as keys:
        steps = [(n, lambda n=n: RUNNERS[n](config, args, keys))
                 for n in names]
        try:
            return bl.run_steps(steps, out)
        except KeyboardInterrupt:
            return 130


def run():
    """Console script entry point."""
    sys.exit(main())


if __name__ == '__main__':
    sys.exit(main())
