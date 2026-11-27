import math

from trash_vision import sonar_scheduler as ss
from trash_vision import ultrasonic_logic as ul


def test_round_robin_one_at_a_time_with_gap():
    s = ss.SonarScheduler(['a', 'b', 'c'], settle_gap_s=0.06)
    order = []
    t = 0.0
    while len(order) < 7:
        name = s.next_due(t)
        if name is not None:
            order.append(name)
            s.fired(t + 0.01)  # ping + echo wait took 10 ms
        t += 0.005
    assert order == ['a', 'b', 'c', 'a', 'b', 'c', 'a']


def test_gap_is_enforced_after_fire():
    s = ss.SonarScheduler(['a', 'b'], settle_gap_s=0.06)
    assert s.next_due(0.0) == 'a'
    s.fired(0.0)
    assert s.next_due(0.059) is None
    assert s.next_due(0.061) == 'b'


def test_two_sensors_never_due_together():
    s = ss.SonarScheduler(ul.SENSOR_NAMES, 0.06)
    assert s.next_due(0.0) == 'front_1'
    # Without fired() the same sensor stays due; no second one appears.
    assert s.next_due(0.0) == 'front_1'


def test_empty_scheduler_never_due():
    assert ss.SonarScheduler([], 0.06).next_due(1.0) is None


def test_cycle_time():
    s = ss.SonarScheduler(ul.SENSOR_NAMES, 0.06)
    assert abs(s.cycle_time_s(0.025) - 0.68) < 1e-9


class FakePin:
    """Echo line that is low until rise, high until fall (clock seconds)."""

    def __init__(self, clock, rise, fall):
        self.clock, self.rise, self.fall = clock, rise, fall

    def __call__(self):
        t = self.clock()
        return self.rise <= t < self.fall


class FakeClock:
    def __init__(self, step=0.0001):
        self.t = 0.0
        self.step = step

    def __call__(self):
        self.t += self.step
        return self.t


def test_measure_echo_returns_pulse_width():
    clock = FakeClock()
    pin = FakePin(clock, 0.002, 0.0125)
    width = ss.measure_echo(pin, clock, 0.025)
    assert abs(width - 0.0105) < 0.0005


def test_measure_echo_no_echo_times_out():
    clock = FakeClock()
    assert ss.measure_echo(lambda: False, clock, 0.025) is None


def test_measure_echo_stuck_high_times_out():
    clock = FakeClock()
    assert ss.measure_echo(lambda: True, clock, 0.025) is None


def test_validate_normal_reading():
    rng, status = ss.validate_range(0.00583)  # ~1.0 m
    assert status == ss.OK
    assert abs(rng - 1.0) < 0.01


def test_validate_no_echo_is_max_range_not_zero():
    rng, status = ss.validate_range(None)
    assert (rng, status) == (ul.HC_SR04_MAX_RANGE_M, ss.NO_ECHO)
    assert ss.validate_range(0.0)[1] == ss.NO_ECHO


def test_validate_out_of_range_is_max_range():
    rng, status = ss.validate_range(0.04)  # ~6.9 m
    assert (rng, status) == (ul.HC_SR04_MAX_RANGE_M, ss.OUT_OF_RANGE)


def test_validate_too_close_is_min_range_not_zero():
    rng, status = ss.validate_range(0.00002)  # ~3 mm
    assert status == ss.TOO_CLOSE
    assert rng == ul.HC_SR04_MIN_RANGE_M and rng > 0


def test_validate_always_finite_and_positive():
    for pulse in (None, 0, -1, 1e-6, 0.001, 0.01, 0.05, 1.0, float('nan')):
        rng, _ = ss.validate_range(pulse)
        assert math.isfinite(rng) and rng > 0


def test_sonar_age():
    assert ss.sonar_age(10.0, 9.5) == 0.5
    assert ss.sonar_age(10.0, None) == math.inf
    assert ss.sonar_age(10.0, 11.0) == 0.0


NAMES = ['front_1', 'front_2']


def test_fresh_sensor_keeps_range():
    out = ss.apply_staleness(NAMES, {'front_1': 1.2, 'front_2': 0.8},
                             {'front_1': 9.5, 'front_2': 9.9}, 10.0, 2.0)
    assert out == [1.2, 0.8]


def test_stale_sensor_is_blocked():
    out = ss.apply_staleness(NAMES, {'front_1': 1.2, 'front_2': 0.8},
                             {'front_1': 5.0, 'front_2': 9.9}, 10.0, 2.0)
    assert out == [0.0, 0.8]


def test_never_heard_sensor_is_ignored_not_blocked():
    out = ss.apply_staleness(NAMES, {'front_2': 0.8}, {'front_2': 9.9},
                             10.0, 2.0)
    assert out == [math.inf, 0.8]
