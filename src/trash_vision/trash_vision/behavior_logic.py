"""Pure-Python behavior state machine (no ROS): search/approach/pick/deposit.

All distances in metres, speeds in m/s and rad/s, times in seconds.
"""
import math

SEARCH = "SEARCH"
APPROACH = "APPROACH"
PICK = "PICK"
DEPOSIT = "DEPOSIT"
STOW = "STOW"
RECOVER = "RECOVER"

DEFAULTS = {
    "search_turn_speed": 0.5,
    "approach_speed": 0.2,
    "max_turn_speed": 0.8,
    "turn_gain": 2.0,
    "min_confidence": 0.3,
    # Target floor point (normalized image y, 0=top, 1=bottom) at which the
    # item is within arm reach. TODO: Bryan to calibrate on the robot.
    "pick_floor_y": 0.85,
    "pick_center_tolerance": 0.12,
    "target_lost_timeout": 1.5,
    "stop_distance": 0.25,
    "slow_distance": 0.6,
    "avoid_turn_speed": 0.6,
    "recover_backup_speed": 0.1,
    "recover_backup_time": 1.5,
    "rear_stop_distance": 0.25,
    "pick_failures_before_recover": 2,
    "arm_timeout": 30.0,
}


class Command:
    """Velocity command plus an optional arm action request."""

    def __init__(self, linear=0.0, angular=0.0, arm_action=None):
        self.linear = linear
        self.angular = angular
        self.arm_action = arm_action


def min_range(values):
    """Smallest finite-or-inf range; -inf (too close) counts as 0."""
    best = math.inf
    for v in values:
        if v is None or math.isnan(v):
            continue
        best = min(best, 0.0 if v == -math.inf else v)
    return best


def split_halves(values):
    """Return (left_min, right_min), sensors ordered left to right."""
    half = len(values) // 2
    return min_range(values[:half]), min_range(values[half:])


def clamp(value, limit):
    return max(-limit, min(limit, value))


def valid_target(target, min_confidence):
    """True if a /vision/target dict is a usable detection."""
    if not target or not target.get("detected"):
        return False
    if float(target.get("confidence", 0.0)) < min_confidence:
        return False
    return "horizontal_error" in target and "floor_point_normalized" in target


class BehaviorMachine:
    """search -> approach -> pick -> deposit -> stow, with avoidance."""

    def __init__(self, params=None):
        self.p = dict(DEFAULTS)
        if params:
            self.p.update(params)
        self.state = SEARCH
        self.items_collected = 0
        self._last_seen = -math.inf
        self._last_target = None
        self._state_start = 0.0
        self._pick_failures = 0
        self._requested = False
        self._avoid_dir = 1.0

    def _enter(self, state, now):
        self.state = state
        self._state_start = now
        self._requested = False

    def arm_result(self, success, now):
        """Report the result of the arm service requested earlier."""
        if self.state == PICK:
            if success:
                self._pick_failures = 0
                self._enter(DEPOSIT, now)
            else:
                self._pick_failures += 1
                if self._pick_failures >= self.p["pick_failures_before_recover"]:
                    self._pick_failures = 0
                    self._enter(RECOVER, now)
                else:
                    self._enter(SEARCH, now)
        elif self.state == DEPOSIT:
            self.items_collected += 1 if success else 0
            self._enter(STOW, now)
        elif self.state == STOW:
            self._enter(SEARCH, now)

    def _obstacle_scale(self, front):
        """Return (speed scale 0..1, avoid turn rate) for front ranges."""
        nearest = min_range(front)
        if nearest <= self.p["stop_distance"]:
            left, right = split_halves(front)
            # Turn away from the closer side (sensors ordered left->right).
            self._avoid_dir = -1.0 if left < right else 1.0
            return 0.0, self._avoid_dir * self.p["avoid_turn_speed"]
        slow, stop = self.p["slow_distance"], self.p["stop_distance"]
        if nearest < slow:
            return (nearest - stop) / (slow - stop), 0.0
        return 1.0, 0.0

    def step(self, now, target, front, rear):
        """Advance one tick and return a Command."""
        p = self.p
        if self.state in (PICK, DEPOSIT, STOW):
            return self._arm_step(now)
        if self.state == RECOVER:
            return self._recover_step(now, rear)

        if valid_target(target, p["min_confidence"]):
            self._last_seen = now
            self._last_target = target
        seen = now - self._last_seen <= p["target_lost_timeout"]

        scale, avoid_turn = self._obstacle_scale(front)
        if self.state == SEARCH and seen:
            self._enter(APPROACH, now)
        if self.state == APPROACH and not seen:
            self._enter(SEARCH, now)

        if self.state == SEARCH:
            if scale == 0.0:
                return Command(0.0, avoid_turn)
            # Spin to look around; obstacle ahead slows nothing (no motion).
            return Command(0.0, p["search_turn_speed"] * self._avoid_dir)

        # APPROACH
        tgt = self._last_target
        err = float(tgt["horizontal_error"])
        floor_y = float(tgt["floor_point_normalized"]["y"])
        if (floor_y >= p["pick_floor_y"]
                and abs(err) <= p["pick_center_tolerance"]
                and now - self._last_seen < 0.5):
            self._enter(PICK, now)
            return self._arm_step(now)
        if scale == 0.0:
            return Command(0.0, avoid_turn)
        angular = clamp(-err * p["turn_gain"], p["max_turn_speed"])
        # Turn first when the target is far off-center.
        align = max(0.0, 1.0 - abs(err) / 0.5)
        return Command(p["approach_speed"] * scale * align, angular)

    def _arm_step(self, now):
        action = {PICK: "pick", DEPOSIT: "deposit", STOW: "stow"}[self.state]
        if not self._requested:
            self._requested = True
            return Command(0.0, 0.0, action)
        if now - self._state_start > self.p["arm_timeout"]:
            self.arm_result(False, now)
        return Command()

    def _recover_step(self, now, rear):
        p = self.p
        if now - self._state_start >= p["recover_backup_time"]:
            self._enter(SEARCH, now)
            return Command()
        if min_range(rear) <= p["rear_stop_distance"]:
            self._enter(SEARCH, now)
            return Command()
        return Command(-p["recover_backup_speed"], 0.0)
