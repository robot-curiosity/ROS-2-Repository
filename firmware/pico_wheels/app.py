"""Pico application logic: protocol handling around the wheel controller.

Hardware sits behind a small `hw` object, so the same code runs on the Pico
(main.py) and in PC tests with a fake motor plant. `hw` provides:

    configure_pins(pins)      set up GPIO from the 8 pin numbers
    read_counts()             (left, right) raw encoder counts
    set_motor(side, duty, forward)   side 0 = left, 1 = right
    stop_all()                PWM to zero on both sides
"""
import math

import control
import protocol

CONTROL_PERIOD_MS = 20      # 50 Hz control loop
TELEMETRY_PERIOD_MS = 20    # one T line per control step

# RP2040 GPIOs on the Pico headers (GPIO 23-25 are internal on the board).
VALID_PINS = tuple(range(0, 23)) + (26, 27, 28)


def pins_error(pins):
    """Return None if pins is a usable list of 8 GPIO numbers, else a word."""
    if len(pins) != protocol.N_PINS:
        return 'BAD_PINS'
    for p in pins:
        if not isinstance(p, int) or p not in VALID_PINS:
            return 'BAD_PINS'
    if len(set(pins)) != len(pins):
        return 'PIN_CONFLICT'
    return None


class PicoApp:
    """Handles protocol lines and runs the wheel loop."""

    def __init__(self, hw):
        """Create the app around a hardware object."""
        self.hw = hw
        self.ctrl = control.WheelController()
        self.pins_ok = False
        self.fault = False
        self._next_step = None

    def hello(self):
        """Line to send after boot."""
        return [protocol.msg_ready()]

    def flags(self):
        """Current telemetry flag bits."""
        bits = 0
        if self.ctrl.watchdog:
            bits |= protocol.FLAG_WATCHDOG
        if not (self.pins_ok and self.ctrl.configured):
            bits |= protocol.FLAG_UNCONFIGURED
        if self.fault:
            bits |= protocol.FLAG_FAULT
        return bits

    def handle_line(self, line, now_ms):
        """Process one received line; return a list of reply lines."""
        try:
            name, args = protocol.decode(line)
        except ValueError:
            return [protocol.msg_error('BAD_LINE')]
        if name == 'PING':
            return [protocol.msg_pong()]
        if name == 'STOP':
            self.ctrl.stop()
            return []
        if name == 'V':
            return self._on_velocity(args, now_ms)
        if name == 'PINS':
            return self._on_pins(args)
        if name == 'CFG':
            return self._on_cfg(args)
        return [protocol.msg_error('UNKNOWN_CMD')]

    def _on_velocity(self, args, now_ms):
        if len(args) != 2 or not all(
                isinstance(a, (int, float)) for a in args):
            return [protocol.msg_error('BAD_ARGS')]
        if not (self.pins_ok and self.ctrl.configured):
            return [protocol.msg_error('NOT_CONFIGURED')]
        if not all(math.isfinite(a) for a in args):
            self.ctrl.stop()    # NaN or inf must never become a speed target
            return [protocol.msg_error('BAD_ARGS')]
        self.ctrl.set_target(float(args[0]), float(args[1]), now_ms)
        return []

    def _on_pins(self, args):
        problem = pins_error(args)
        if problem:
            return [protocol.msg_error(problem)]
        try:
            self.hw.configure_pins(list(args))
        except Exception:
            self.fault = True
            return [protocol.msg_error('PIN_SETUP_FAILED')]
        self.fault = False
        self.pins_ok = True
        self.ctrl.reset_encoders()    # new decoders start at 0
        return []

    def _on_cfg(self, args):
        names = protocol.CFG_FIELDS
        if len(args) != len(names) or not all(
                isinstance(a, (int, float)) for a in args):
            return [protocol.msg_error('BAD_ARGS')]
        if not all(math.isfinite(a) for a in args):
            return [protocol.msg_error('BAD_CFG')]
        cfg = dict(zip(names, args))
        for key in ('ticks_per_rev', 'wheel_radius_m', 'max_speed_mps',
                    'cmd_timeout_ms'):
            if cfg[key] <= 0:
                return [protocol.msg_error('BAD_CFG')]
        self.ctrl.configure(cfg)
        return []

    def poll(self, now_ms):
        """Run the control step when due; return telemetry lines."""
        if self._next_step is not None and \
                (now_ms - self._next_step) % (1 << 30) > (1 << 29):
            return []
        self._next_step = (now_ms + CONTROL_PERIOD_MS) % (1 << 30)
        if self.pins_ok and self.ctrl.configured:
            left, right = self.hw.read_counts()
            outputs = self.ctrl.update(left, right, now_ms)
            for side in (0, 1):
                self.hw.set_motor(side, outputs[side][0], outputs[side][1])
        else:
            self.hw.stop_all()
        return [protocol.msg_telemetry(
            now_ms, self.ctrl.ticks[0], self.ctrl.ticks[1],
            self.ctrl.speeds[0], self.ctrl.speeds[1], self.flags())]
