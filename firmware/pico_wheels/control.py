"""Wheel speed control for the Pico: decoder, ramp, PI loop, watchdog.

Plain Python (no hardware imports) so it runs on a PC for unit tests and on
MicroPython on the Pico. Same control law as the Pi drive node
(`drive_logic.py`): feed-forward plus PI per side, acceleration limit.
"""

# Quadrature transition table: index = (prev_state << 2) | new_state,
# state = (A << 1) | B. +1 forward, -1 reverse, 0 no change / invalid.
_QUAD_TABLE = (
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
)


class QuadratureDecoder:
    """Counts encoder edges from the A and B levels."""

    def __init__(self, a=0, b=0):
        """Start from the current levels."""
        self._state = ((1 if a else 0) << 1) | (1 if b else 0)
        self.count = 0

    def update(self, a, b):
        """Feed the current levels; return the count change."""
        new = ((1 if a else 0) << 1) | (1 if b else 0)
        delta = _QUAD_TABLE[(self._state << 2) | new]
        self._state = new
        self.count += delta
        return delta


def slew(current, target, max_rate, dt):
    """Move current toward target by at most max_rate * dt."""
    if max_rate <= 0.0:
        return target
    step = max_rate * dt
    return current + max(-step, min(step, target - current))


def limit_wheel_speeds(left, right, max_speed):
    """Scale both wheels together so neither exceeds max_speed."""
    peak = max(abs(left), abs(right))
    if max_speed <= 0.0 or peak <= max_speed:
        return left, right
    k = max_speed / peak
    return left * k, right * k


def command_to_pwm(command, min_duty=0.0):
    """Signed command -1..1 to (duty 0..1, forward), with a dead band."""
    command = max(-1.0, min(1.0, command))
    if abs(command) < 1e-3:
        return 0.0, True
    duty = min_duty + (1.0 - min_duty) * abs(command)
    return min(duty, 1.0), command > 0.0


class PIController:
    """PI with feed-forward and anti-windup."""

    def __init__(self, kp, ki, limit=1.0):
        """Set the gains and the output limit."""
        self.kp = kp
        self.ki = ki
        self.limit = limit
        self._integral = 0.0

    def reset(self):
        """Clear the integral."""
        self._integral = 0.0

    def update(self, target, measured, dt, feedforward=0.0):
        """Return the output in -limit..limit."""
        error = target - measured
        self._integral += error * dt
        if self.ki > 0.0:
            cap = self.limit / self.ki
            self._integral = max(-cap, min(cap, self._integral))
        out = feedforward + self.kp * error + self.ki * self._integral
        return max(-self.limit, min(self.limit, out))


class WheelController:
    """Closed-loop speed control of the left and right side."""

    def __init__(self):
        """Start unconfigured, targets zero."""
        self.configured = False
        self.cfg = {}
        self.target = [0.0, 0.0]
        self.applied = [0.0, 0.0]
        self.speeds = [0.0, 0.0]
        self.ticks = [0, 0]
        self.watchdog = False
        self._pi = [PIController(0.0, 0.0), PIController(0.0, 0.0)]
        self._cmd_ms = None
        self._last_raw = None
        self._last_ms = None

    def configure(self, cfg):
        """Apply a dict with the keys of protocol.CFG_FIELDS."""
        self.cfg = dict(cfg)
        self._pi = [PIController(cfg['kp'], cfg['ki']) for _ in range(2)]
        self.configured = True

    def set_target(self, left, right, now_ms):
        """Set wheel speed targets (m/s) and feed the watchdog."""
        if not self.configured:
            return
        left, right = limit_wheel_speeds(
            left, right, self.cfg['max_speed_mps'])
        self.target = [left, right]
        self._cmd_ms = now_ms
        self.watchdog = False

    def stop(self):
        """Zero both targets."""
        self.target = [0.0, 0.0]

    def _elapsed(self, now_ms, since_ms):
        return (now_ms - since_ms) % (1 << 30)

    def update(self, raw_left, raw_right, now_ms):
        """Run one step with raw encoder counts; return motor commands.

        Returns [(duty, forward), (duty, forward)] for left and right.
        """
        if not self.configured:
            return [(0.0, True), (0.0, True)]
        cfg = self.cfg
        raw = [raw_left, raw_right]
        sign = [-1 if cfg['invert_enc_left'] else 1,
                -1 if cfg['invert_enc_right'] else 1]
        if self._last_raw is None:
            self._last_raw = raw
            self._last_ms = now_ms
        dt = self._elapsed(now_ms, self._last_ms) / 1000.0
        d = [(raw[i] - self._last_raw[i]) * sign[i] for i in range(2)]
        self._last_raw = raw
        self._last_ms = now_ms
        for i in range(2):
            self.ticks[i] += d[i]
        per_tick = 2.0 * 3.141592653589793 * cfg['wheel_radius_m'] / \
            cfg['ticks_per_rev']
        if dt > 0.0:
            self.speeds = [d[i] * per_tick / dt for i in range(2)]

        target = self.target
        if self._cmd_ms is not None and not self.watchdog and \
                self._elapsed(now_ms, self._cmd_ms) > cfg['cmd_timeout_ms']:
            self.watchdog = True
        if self.watchdog:
            target = [0.0, 0.0]

        inv = [bool(cfg['invert_left']), bool(cfg['invert_right'])]
        out = []
        for i in range(2):
            self.applied[i] = slew(
                self.applied[i], target[i], cfg['max_accel_mps2'], dt)
            if self.applied[i] == 0.0:
                self._pi[i].reset()
                cmd = 0.0
            else:
                ff = self.applied[i] / cfg['max_speed_mps']
                cmd = self._pi[i].update(
                    self.applied[i], self.speeds[i], dt, ff)
            duty, fwd = command_to_pwm(cmd, cfg['min_duty'])
            out.append((duty, fwd != inv[i]))
        return out
