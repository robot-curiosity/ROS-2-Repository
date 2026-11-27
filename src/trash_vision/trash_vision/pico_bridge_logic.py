"""Pure-Python logic of the Pico wheel bridge (no ROS, no serial port).

`BridgeSession` turns velocity targets into protocol lines and Pico
telemetry into odometry. It talks through a `send(line)` callable, so tests
can connect it to a fake Pico.
"""
import math

from trash_vision import drive_logic as dl
from trash_vision import pico_protocol as pp


def build_setup_lines(params):
    """PINS and CFG lines made from the pico_bridge parameter dict."""
    pins = [int(p) for p in params['pico_pins']]
    cfg = [
        float(params['ticks_per_rev']),
        float(params['wheel_radius_m']),
        float(params['max_wheel_speed_mps']),
        float(params['max_accel_mps2']),
        float(params['min_duty']),
        float(params['kp']),
        float(params['ki']),
        int(round(float(params['pico_cmd_timeout_s']) * 1000.0)),
        bool(params['invert_left']),
        bool(params['invert_right']),
        bool(params['invert_enc_left']),
        bool(params['invert_enc_right']),
    ]
    return [pp.cmd_pins(pins), pp.cmd_cfg(*cfg)]


def describe_flags(flags):
    """Human-readable names of the set telemetry flags."""
    names = []
    if flags & pp.FLAG_WATCHDOG:
        names.append('watchdog')
    if flags & pp.FLAG_UNCONFIGURED:
        names.append('unconfigured')
    if flags & pp.FLAG_FAULT:
        names.append('fault')
    return names


class BridgeSession:
    """Protocol state of the Pi side."""

    def __init__(self, send, params):
        """Keep the send function and build odometry from the parameters."""
        self._send = send
        self._params = dict(params)
        self.odom = dl.DiffDriveOdometry(
            float(params['ticks_per_rev']), float(params['wheel_radius_m']),
            float(params['track_width_m']))
        self.events = []
        self.flags = 0
        self.speeds = (0.0, 0.0)
        self.pico_ready = False
        self._last_ticks = None
        self._last_ms = None

    def start(self):
        """Send PINS and CFG."""
        for line in build_setup_lines(self._params):
            self._send(line)

    def send_velocity(self, linear, angular):
        """Send wheel targets (m/s) from a Twist; returns the pair sent."""
        linear, angular = dl.sanitize_twist(linear, angular)
        left, right = dl.twist_to_wheels(
            linear, angular, float(self._params['track_width_m']))
        left, right = dl.limit_wheel_speeds(
            left, right, float(self._params['max_wheel_speed_mps']))
        self._send(pp.cmd_velocity(left, right))
        return left, right

    def stop(self):
        """Tell the Pico to stop."""
        self._send(pp.cmd_stop())

    def on_line(self, line):
        """Handle one line from the Pico; returns True if odom changed."""
        try:
            name, args = pp.decode(line)
        except ValueError:
            self.events.append('bad line from pico')
            return False
        if name == 'READY':
            # The Pico (re)booted and forgot its setup.
            self.pico_ready = True
            self._last_ticks = None
            self.start()
            return False
        if name == 'ERR':
            self.events.append('pico error: %s' % ' '.join(
                str(a) for a in args))
            return False
        if name == 'T' and len(args) == 6:
            return self._on_telemetry(args)
        return False

    def _on_telemetry(self, args):
        ms, ticks_left, ticks_right, speed_left, speed_right, flags = args
        flags = int(flags)
        if flags != self.flags:
            self.events.append('pico flags: %s' % (
                ','.join(describe_flags(flags)) or 'ok'))
            self.flags = flags
        self.speeds = (float(speed_left), float(speed_right))
        ticks = (int(ticks_left), int(ticks_right))
        if self._last_ticks is None:
            self._last_ticks = ticks
            self._last_ms = int(ms)
            return False
        dt = ((int(ms) - self._last_ms) % pp.MS_MODULO) / 1000.0
        d_left = ticks[0] - self._last_ticks[0]
        d_right = ticks[1] - self._last_ticks[1]
        self._last_ticks = ticks
        self._last_ms = int(ms)
        if dt <= 0.0:
            return False
        self.odom.update(d_left, d_right, dt)
        return True

    def pose(self):
        """Return (x, y, theta, v, w)."""
        o = self.odom
        return o.x, o.y, o.theta, o.v, o.w


def link_lost(now_s, last_rx_s, timeout_s):
    """True if nothing arrived from the Pico for longer than timeout_s."""
    if last_rx_s is None:
        return True
    return (now_s - last_rx_s) > timeout_s


def yaw_quaternion(theta):
    """Yaw to quaternion (z, w)."""
    return math.sin(theta / 2.0), math.cos(theta / 2.0)
