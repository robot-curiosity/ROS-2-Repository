import math

import pytest

from trash_vision import calibration_logic as cl
from trash_vision import ground_projection as gp


def test_ticks_per_rev():
    assert cl.ticks_per_rev(8800, 10) == 880.0
    assert cl.ticks_per_rev(-8800, 10) == 880.0
    assert cl.spin_direction(-5) == -1
    assert cl.spin_direction(5) == 1


def test_ticks_per_rev_rejects_bad_input():
    with pytest.raises(ValueError):
        cl.ticks_per_rev(0, 10)
    with pytest.raises(ValueError):
        cl.ticks_per_rev(100, 0)


def test_wheel_radius_correction():
    # odometry says 1.00 m but tape says 1.10 m: wheel is bigger
    assert cl.wheel_radius_correction(0.06, 1.0, 1.1) == pytest.approx(0.066)
    with pytest.raises(ValueError):
        cl.wheel_radius_correction(0.06, 0.0, 1.0)


def test_track_width_correction():
    # robot really turned 10% more than odometry: track is 10% too long
    new = cl.track_width_correction(0.5, math.pi, math.pi * 1.1)
    assert new == pytest.approx(0.5 / 1.1)
    with pytest.raises(ValueError):
        cl.track_width_correction(0.5, 0.0, 1.0)


def test_unwrap_yaw_full_turn():
    yaws = [math.atan2(math.sin(a), math.cos(a))
            for a in [i * 0.5 for i in range(14)]]
    assert cl.unwrap_yaw(yaws) == pytest.approx(6.5)


def test_pick_zone():
    samples = [(0.5 + d, 0.84 + e) for d, e in
               [(0.0, 0.0), (0.01, 0.01), (-0.01, 0.02), (0.02, -0.01),
                (0.0, 0.0), (-0.02, 0.01)]]
    out = cl.pick_zone(samples)
    assert out["pick_floor_y"] == pytest.approx(0.84, abs=0.01)
    assert out["pick_center_tolerance"] >= 0.05
    with pytest.raises(ValueError):
        cl.pick_zone(samples[:3])


def test_pick_zone_off_centre_widens_tolerance():
    samples = [(0.65, 0.8)] * 6
    assert cl.pick_zone(samples)["pick_center_tolerance"] >= 0.15


def test_camera_pitch_recovers_true_pitch():
    true_cam = gp.camera_from_degrees(0.248, -0.1575, 0.642, 30.0, 66.0, 41.0)
    # marker 1.0 m ahead of base_link: find where it appears in the image
    # (search v), then solve with a camera configured at 35 deg.
    v = None
    for i in range(1, 1000):
        cand = i / 1000.0
        pt = gp.floor_point_to_base(0.5, cand, true_cam)
        if pt and abs(pt[0] - 1.0) < 0.002:
            v = cand
            break
    assert v is not None
    wrong = gp.camera_from_degrees(0.248, -0.1575, 0.642, 35.0, 66.0, 41.0)
    solved = cl.camera_pitch_from_marker(0.5, v, 1.0, wrong)
    assert solved == pytest.approx(30.0, abs=0.3)
    assert cl.pitch_error(35.0, solved) == pytest.approx(-5.0, abs=0.3)


def test_camera_pitch_unsolvable():
    cam = gp.camera_from_degrees(0.248, 0.0, 0.642, 35.0, 66.0, 41.0)
    assert cl.camera_pitch_from_marker(0.5, 0.5, 50.0, cam) is None


def test_analyze_sonar():
    inf = math.inf
    series = {
        "front_1": [0.5 + 0.01 * i for i in range(25)],
        "front_2": [],
        "rear_1": [inf] * 12,
        "rear_2": [0.321] * 25,
        "left_1": [None, None],
    }
    out = cl.analyze_sonar(series)
    assert out["front_1"][0] == "OK"
    assert out["front_2"][0] == "NO_DATA"
    assert out["rear_1"][0] == "NO_ECHO"
    assert out["rear_2"][0] == "STUCK"
    assert out["left_1"][0] == "NO_DATA"


def test_hand_test_reading_picks_closest():
    series = {"a": [1.0, 1.1, 1.0], "b": [0.1, 0.12, 0.11],
              "c": [math.inf] * 3}
    assert cl.hand_test_reading(series)[0] == "b"
    assert cl.hand_test_reading({"a": [math.inf]}) is None


def test_find_swaps():
    seen = {"front_1": "front_2", "front_2": "front_1", "rear_1": "rear_1",
            "left_1": None, "left_2": "right_1"}
    problems = cl.find_swaps(seen)
    assert len(problems) == 3
    assert any("front_1 and front_2 are swapped" in p for p in problems)
    assert any("left_1: nothing" in p for p in problems)
    assert any("left_2" in p and "right_1" in p for p in problems)
    assert cl.find_swaps({"a": "a"}) == []


def test_yaml_line():
    assert cl.yaml_line("ticks_per_rev", 880.0) == "ticks_per_rev: 880.0"
    assert cl.yaml_line("wheel_radius_m", 0.06612345) == \
        "wheel_radius_m: 0.0661"
    assert cl.yaml_line("n", 3) == "n: 3"
