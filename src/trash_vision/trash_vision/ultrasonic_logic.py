"""Pure-Python helpers for the HC-SR04 ultrasonic node (no ROS needed)."""

import math
from collections import deque

SPEED_OF_SOUND_M_S = 343.0

# Sensor order. Names become topic suffixes: /ultrasonic/<name>.
SENSOR_NAMES = [
    "front_1", "front_2", "front_3", "front_4",
    "rear_1", "rear_2", "rear_3", "rear_4",
]

# HC-SR04 datasheet limits.
HC_SR04_MIN_RANGE_M = 0.02
HC_SR04_MAX_RANGE_M = 4.0


def echo_to_distance(pulse_seconds, speed=SPEED_OF_SOUND_M_S):
    """Convert an echo pulse width (round trip) to a distance in metres."""
    if pulse_seconds is None or pulse_seconds <= 0:
        return None
    return pulse_seconds * speed / 2.0


def classify_range(distance, min_range=HC_SR04_MIN_RANGE_M,
                   max_range=HC_SR04_MAX_RANGE_M):
    """Return a clamped range in metres.

    No reading or beyond max gives +inf (nothing seen), below min gives -inf
    (too close), as in the sensor_msgs/Range convention.
    """
    if distance is None or math.isnan(distance):
        return math.inf
    if distance < min_range:
        return -math.inf
    if distance > max_range:
        return math.inf
    return float(distance)


class MedianFilter:
    """Sliding-window median to reject single-sample ultrasonic spikes."""

    def __init__(self, window=3):
        self._values = deque(maxlen=max(1, int(window)))

    def update(self, value):
        """Add a finite reading (non-finite readings are kept as-is)."""
        self._values.append(value)
        ordered = sorted(self._values)
        return ordered[len(ordered) // 2]


def build_sensor_config(names, trig_pins, echo_pins):
    """Pair names with GPIO pins; return (configured, problems).

    Pins < 0 mean "not set yet" (TODO: Bryan to fill in) and that sensor is
    skipped. Duplicate pins or length mismatches are reported as problems.
    """
    problems = []
    if len(trig_pins) != len(names) or len(echo_pins) != len(names):
        problems.append(
            "trig_pins/echo_pins must each have %d entries" % len(names))
        return [], problems
    configured = []
    used = {}
    for name, trig, echo in zip(names, trig_pins, echo_pins):
        if trig < 0 or echo < 0:
            continue
        for pin in (trig, echo):
            if pin in used:
                problems.append("pin %d used by both %s and %s"
                                % (pin, used[pin], name))
            used[pin] = name
        configured.append({"name": name, "trig": int(trig), "echo": int(echo)})
    return configured, problems


def nearest_obstacle(ranges):
    """Smallest finite positive range in a {name: range} dict, else None."""
    finite = [r for r in ranges.values() if math.isfinite(r) and r > 0]
    return min(finite) if finite else None
