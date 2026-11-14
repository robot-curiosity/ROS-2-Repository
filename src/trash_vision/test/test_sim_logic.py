import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from trash_vision import sim_logic as sl  # noqa: E402
from trash_vision.behavior_logic import DEFAULTS  # noqa: E402


def test_step_moves_forward_and_walls_stop():
    w = sl.SimWorld(4, 4, x=2, y=2, yaw=0)
    w.step(1.0, 0.0, 1.0)
    assert abs(w.x - 3.0) < 1e-9
    w.step(5.0, 0.0, 10.0)
    assert w.x <= 3.96


def test_ranges_to_walls():
    w = sl.SimWorld(4, 4, x=1, y=2, yaw=0)
    r = w.ranges()
    assert abs(w.range_at(math.pi) - 1.0) < 1e-9
    assert r["rear_2"] < 1.1
    assert r["front_2"] == float("inf")  # 3 m away > max range


def test_no_litter_no_target():
    w = sl.SimWorld(litter=[])
    assert w.target(1)["detected"] is False


def test_litter_left_appears_left_of_center():
    w = sl.SimWorld(4, 4, [(3.0, 2.5)], x=2, y=2, yaw=0)
    t = w.target(1)
    assert t["detected"] is True
    assert t["center_normalized"]["x"] < 0.5


def test_litter_behind_not_visible():
    w = sl.SimWorld(4, 4, [(1.0, 2.0)], x=2, y=2, yaw=0)
    assert w.target(1)["detected"] is False


def test_floor_y_reaches_pick_threshold_when_close():
    w = sl.SimWorld(4, 4, [(2.3, 2.0)], x=2, y=2, yaw=0)
    t = w.target(1)
    assert t["floor_point_normalized"]["y"] >= DEFAULTS["pick_floor_y"]
    far = sl.SimWorld(4, 4, [(3.5, 2.0)], x=2, y=2, yaw=0).target(1)
    assert far["floor_point_normalized"]["y"] < DEFAULTS["pick_floor_y"]


def test_pick_and_deposit_cycle():
    w = sl.SimWorld(4, 4, [(2.3, 2.0)], x=2, y=2, yaw=0)
    ok, _ = w.pick()
    assert ok and w.held and not w.litter
    assert w.pick()[0] is False
    assert w.deposit()[0] is True
    assert w.collected == 1
    assert w.deposit()[0] is False


def test_pick_out_of_reach_fails():
    w = sl.SimWorld(4, 4, [(3.5, 2.0)], x=2, y=2, yaw=0)
    assert w.pick()[0] is False
    assert w.litter
