"""Safety output stage: the last filter between behavior and cmd_vel.

Pure Python (no ROS). Clamps speed and acceleration and forces zero velocity
when the robot cannot trust its inputs. Forced stops are immediate (no ramp).
"""
import math

DEFAULTS = {
    # Speed limits (working values, tune on the robot).
    'safety_max_linear': 0.4,          # m/s
    'safety_max_angular': 1.0,         # rad/s
    # Acceleration limits; they also limit deceleration of normal commands.
    'safety_max_linear_accel': 0.5,    # m/s^2
    'safety_max_angular_accel': 2.0,   # rad/s^2
    # Front sonar minimum below which all motion is forced to zero. Keep it
    # below behavior stop_distance so normal avoidance acts first.
    'safety_hard_stop_distance': 0.12,  # m
    # True: below the hard stop distance only FORWARD motion is cut to zero;
    # reversing and turning still pass (ramped), so the robot can back or
    # turn away from what it is touching instead of wedging there forever.
    # False: everything is forced to zero (old behavior).
    'safety_hard_stop_escape': True,
    # No behavior update for this long -> zero velocity.
    'safety_behavior_timeout': 0.5,    # s
    # True: stop if no sonar has ever reported. False: allow a robot without
    # sonars (bring-up only; never for field use).
    'safety_require_sonars': True,
}


def sonars_healthy(names, stamps, now, stale_after_s, require_any=True):
    """Return False if a sonar that reported earlier is stale, or none ever reported.

    ``stamps`` maps sensor name to local receive time. A sensor that never
    reported is treated as not installed (same rule as
    ``sonar_scheduler.apply_staleness``).
    """
    seen = [stamps[n] for n in names if stamps.get(n) is not None]
    if not seen:
        return not require_any
    return all(now - s <= stale_after_s for s in seen)


def front_minimum(values):
    """Smallest front range; a NaN reading counts as 0 (blocked).

    The builtin ``min`` is order dependent with NaN, so a bad reading could
    hide behind a good one. An empty list reads NaN (unknown -> stop).
    """
    best = math.inf
    for v in values:
        if v is None or math.isnan(v):
            return 0.0
        best = min(best, v)
    return best if values else math.nan


def _clamp(value, limit):
    limit = abs(limit)
    return max(-limit, min(limit, value))


def _ramp(prev, target, max_step):
    return prev + max(-max_step, min(max_step, target - prev))


class SafetyStage:
    """Stateful output filter. Call ``note_update`` per behavior output."""

    def __init__(self, params=None):
        """Create the stage from optional parameter overrides."""
        cfg = dict(DEFAULTS)
        cfg.update(params or {})
        self.cfg = cfg
        self.linear = 0.0
        self.angular = 0.0
        self.reason = ''
        self._last_out = None
        self._last_update = None

    def note_update(self, now):
        """Record that behavior produced a fresh command at ``now``."""
        self._last_update = now

    def timed_out(self, now):
        """Return True if behavior has not updated within the timeout."""
        if self._last_update is None:
            return True
        return now - self._last_update > self.cfg['safety_behavior_timeout']

    def _stop(self, now, reason):
        self.linear = 0.0
        self.angular = 0.0
        self.reason = reason
        self._last_out = now
        return 0.0, 0.0

    def apply(self, now, linear, angular, front_min, sonars_ok=True):
        """Return the safe (linear, angular) for a requested command.

        ``front_min`` is the smallest front sonar range in metres (stale
        sensors already read 0 via ``apply_staleness``).
        """
        if not (math.isfinite(linear) and math.isfinite(angular)):
            return self._stop(now, 'invalid_command')
        if self.timed_out(now):
            return self._stop(now, 'behavior_timeout')
        if not sonars_ok:
            return self._stop(now, 'sonars_stale')
        if front_min is None or math.isnan(front_min):
            return self._stop(now, 'front_hard_stop')
        blocked = front_min < self.cfg['safety_hard_stop_distance']
        if blocked and not self.cfg['safety_hard_stop_escape']:
            return self._stop(now, 'front_hard_stop')
        if blocked:
            # Cut forward motion at once; reverse and turning stay allowed.
            linear = min(linear, 0.0)
            self.linear = min(self.linear, 0.0)
        lin = _clamp(linear, self.cfg['safety_max_linear'])
        ang = _clamp(angular, self.cfg['safety_max_angular'])
        reason = 'clamped' if (lin != linear or ang != angular) else ''
        if blocked:
            reason = 'front_hard_stop'
        if self._last_out is not None:
            dt = max(0.0, now - self._last_out)
            la = self.cfg['safety_max_linear_accel'] * dt
            aa = self.cfg['safety_max_angular_accel'] * dt
            ramped_l = _ramp(self.linear, lin, la)
            ramped_a = _ramp(self.angular, ang, aa)
            if (ramped_l != lin or ramped_a != ang) and not blocked:
                reason = 'accel_limited'
            lin, ang = ramped_l, ramped_a
        else:
            # First output after start or a stop: start from rest.
            la = self.cfg['safety_max_linear_accel'] * 0.1
            aa = self.cfg['safety_max_angular_accel'] * 0.1
            lin, ang = _ramp(0.0, lin, la), _ramp(0.0, ang, aa)
        self.linear, self.angular = lin, ang
        self.reason = reason
        self._last_out = now
        return lin, ang
