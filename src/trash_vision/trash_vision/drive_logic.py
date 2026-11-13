"""Pure-Python drive logic (no ROS, no GPIO) so CI can test it."""
import math

# Quadrature transition table: index = (prev_state << 2) | new_state,
# state = (A << 1) | B. +1 forward, -1 reverse, 0 no change / invalid.
_QUAD_TABLE = [
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
]


# Quadrature decoder:
class QuadratureDecoder:
    def __init__(self, a: int = 0, b: int = 0) -> None:
        self._state = ((1 if a else 0) << 1) | (1 if b else 0)
        self.count = 0

    # Feed the current A/B levels; returns the count change (-1, 0, +1):
    def update(self, a: int, b: int) -> int:
        new = ((1 if a else 0) << 1) | (1 if b else 0)
        delta = _QUAD_TABLE[(self._state << 2) | new]
        self._state = new
        self.count += delta
        return delta


# Twist -> wheel linear speeds (m/s):
def twist_to_wheels(linear: float, angular: float, track_width: float):
    left = linear - angular * track_width / 2.0
    right = linear + angular * track_width / 2.0
    return left, right


# Scale both wheels down together so neither exceeds max_speed (keeps the arc):
def limit_wheel_speeds(left: float, right: float, max_speed: float):
    peak = max(abs(left), abs(right))
    if max_speed <= 0.0 or peak <= max_speed:
        return left, right
    k = max_speed / peak
    return left * k, right * k


# Signed command in -1..1 -> (duty 0..1, forward bool), with a dead band:
def command_to_pwm(command: float, min_duty: float = 0.0):
    command = max(-1.0, min(1.0, command))
    if abs(command) < 1e-3:
        return 0.0, True
    duty = min_duty + (1.0 - min_duty) * abs(command)
    return min(duty, 1.0), command > 0.0


# Limits how fast a value may change per second:
def slew(current: float, target: float, max_rate: float, dt: float) -> float:
    if max_rate <= 0.0:
        return target
    step = max_rate * dt
    return current + max(-step, min(step, target - current))


# Simple PI controller with feed-forward and anti-windup:
class PIController:
    def __init__(self, kp: float, ki: float, limit: float = 1.0) -> None:
        self.kp = kp
        self.ki = ki
        self.limit = limit
        self._integral = 0.0

    def reset(self) -> None:
        self._integral = 0.0

    def update(self, target: float, measured: float, dt: float,
               feedforward: float = 0.0) -> float:
        error = target - measured
        self._integral += error * dt
        if self.ki > 0.0:
            cap = self.limit / self.ki
            self._integral = max(-cap, min(cap, self._integral))
        out = feedforward + self.kp * error + self.ki * self._integral
        return max(-self.limit, min(self.limit, out))


# Differential-drive odometry from encoder tick deltas:
class DiffDriveOdometry:
    def __init__(self, ticks_per_rev: float, wheel_radius: float,
                 track_width: float) -> None:
        self.metres_per_tick = 2.0 * math.pi * wheel_radius / ticks_per_rev
        self.track_width = track_width
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0

    def update(self, d_left_ticks: int, d_right_ticks: int, dt: float):
        dl = d_left_ticks * self.metres_per_tick
        dr = d_right_ticks * self.metres_per_tick
        ds = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.track_width
        mid = self.theta + dtheta / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.theta = math.atan2(math.sin(self.theta + dtheta),
                                math.cos(self.theta + dtheta))
        if dt > 0.0:
            self.v = ds / dt
            self.w = dtheta / dt
        return self.x, self.y, self.theta

    def wheel_speeds(self, d_left_ticks: int, d_right_ticks: int, dt: float):
        if dt <= 0.0:
            return 0.0, 0.0
        return (d_left_ticks * self.metres_per_tick / dt,
                d_right_ticks * self.metres_per_tick / dt)


# Yaw -> quaternion (z, w):
def yaw_to_quaternion(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)
