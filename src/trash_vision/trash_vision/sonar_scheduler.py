"""Pure-Python sonar scheduling and validity rules (no ROS, no GPIO).

HC-SR04 modules hear each other's pings, so the array is fired round-robin,
one sensor at a time, with a settle gap between pings. A reading is always a
real number: no echo or beyond max range means "no obstacle" and is reported
at max range (never 0). Sensors that stop updating are treated as blocked by
the behavior node via ``apply_staleness``.
"""

import math

from trash_vision import ultrasonic_logic as ul

OK = "ok"
NO_ECHO = "no_echo"        # nothing heard: no obstacle within range
TOO_CLOSE = "too_close"    # inside the blind zone: obstacle, report min range
OUT_OF_RANGE = "out_of_range"  # echo from beyond max range: no obstacle


class SonarScheduler:
    """Round-robin firing order with a minimum gap between pings."""

    def __init__(self, names, settle_gap_s=0.06):
        self.names = list(names)
        self.settle_gap_s = max(0.0, float(settle_gap_s))
        self._index = 0
        self._last_fire = None

    def next_due(self, now):
        """Name of the sensor to fire now, or None if the gap has not passed.

        Call ``fired(now)`` after the ping (and its echo wait) has finished;
        the settle gap counts from that moment.
        """
        if not self.names:
            return None
        if (self._last_fire is not None
                and now - self._last_fire < self.settle_gap_s):
            return None
        return self.names[self._index]

    def fired(self, now):
        """Record that the current sensor was fired and move to the next."""
        self._last_fire = now
        self._index = (self._index + 1) % max(1, len(self.names))

    def cycle_time_s(self, echo_timeout_s):
        """Worst-case time for one full round of the array."""
        return len(self.names) * (self.settle_gap_s + echo_timeout_s)


def measure_echo(read_pin, clock, timeout_s):
    """Time the echo pulse; return its width in seconds or None.

    ``read_pin()`` returns the echo level (truthy = high); ``clock()`` returns
    seconds. Gives up (None) if the line never goes high or never falls
    within ``timeout_s`` of the call. Trigger the sensor before calling.
    """
    deadline = clock() + timeout_s
    while not read_pin():
        if clock() > deadline:
            return None
    start = clock()
    while read_pin():
        if clock() > deadline:
            return None
    return clock() - start


def validate_range(pulse_s, min_range=ul.HC_SR04_MIN_RANGE_M,
                   max_range=ul.HC_SR04_MAX_RANGE_M):
    """Turn an echo pulse width into (range_m, status). Never returns 0/inf.

    No echo and out of range both give ``max_range`` (no obstacle). A reading
    inside the blind zone gives ``min_range`` (obstacle very close).
    """
    distance = ul.echo_to_distance(pulse_s)
    if distance is None or math.isnan(distance):
        return max_range, NO_ECHO
    if distance < min_range:
        return min_range, TOO_CLOSE
    if distance > max_range:
        return max_range, OUT_OF_RANGE
    return float(distance), OK


def sonar_age(now, last_update):
    """Seconds since a sensor last updated; inf if it never has."""
    if last_update is None:
        return math.inf
    return max(0.0, now - last_update)


def apply_staleness(names, ranges, stamps, now, stale_after_s,
                    blocked_value=0.0):
    """List of ranges (in ``names`` order) with stale sensors set to blocked.

    A sensor that has reported before but not within ``stale_after_s`` reads
    ``blocked_value`` (0.0 = obstacle touching). A sensor that has never
    reported is treated as not installed and reads "nothing seen" (inf), so
    a partly assembled robot still runs; ``sonar_test`` finds dead sensors.
    """
    out = []
    for name in names:
        stamp = stamps.get(name)
        if stamp is None or name not in ranges:
            out.append(math.inf)
        elif sonar_age(now, stamp) > stale_after_s:
            out.append(blocked_value)
        else:
            out.append(ranges[name])
    return out
