import math

from trash_vision import ultrasonic_logic as ul


def test_echo_to_distance():
    assert abs(ul.echo_to_distance(0.002915) - 0.5) < 0.001
    assert ul.echo_to_distance(0) is None
    assert ul.echo_to_distance(None) is None


def test_classify_range():
    assert ul.classify_range(1.0) == 1.0
    assert ul.classify_range(None) == math.inf
    assert ul.classify_range(5.0) == math.inf
    assert ul.classify_range(0.005) == -math.inf


def test_median_filter_rejects_spike():
    f = ul.MedianFilter(3)
    f.update(1.0)
    f.update(1.0)
    assert f.update(3.0) == 1.0


def test_config_skips_unset_pins():
    names = ["a", "b", "c"]
    cfg, problems = ul.build_sensor_config(names, [4, -1, 6], [5, -1, 7])
    assert problems == []
    assert [c["name"] for c in cfg] == ["a", "c"]


def test_config_detects_problems():
    _, p = ul.build_sensor_config(["a", "b"], [4], [5, 6])
    assert p
    _, p = ul.build_sensor_config(["a", "b"], [4, 4], [5, 6])
    assert any("pin 4" in x for x in p)


def test_nearest_obstacle():
    assert ul.nearest_obstacle({"a": math.inf, "b": 0.8, "c": 0.4}) == 0.4
    assert ul.nearest_obstacle({"a": math.inf, "b": -math.inf}) is None
    assert len(ul.SENSOR_NAMES) == 8
