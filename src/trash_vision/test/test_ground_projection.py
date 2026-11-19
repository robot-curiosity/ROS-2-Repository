import json
from math import radians

from trash_vision import detection_logic as dl
from trash_vision import ground_projection as gp


def test_bottom_edge_matches_review():
    x, y = gp.floor_point_to_base(0.5, 1.0)
    assert abs(x - 0.689) < 0.002
    assert abs(y - (-0.1575)) < 1e-9


def test_top_edge_matches_review():
    x, _ = gp.floor_point_to_base(0.5, 0.0)
    assert abs(x - 2.730) < 0.005


def test_horizon_returns_none():
    # pitch 0, axis level: anything at or above centre is not on the floor
    cam = gp.CameraModel(pitch_rad=0.0)
    assert gp.floor_point_to_base(0.5, 0.5, cam) is None
    assert gp.floor_point_to_base(0.5, 0.0, cam) is None
    assert gp.floor_point_to_base(0.5, 0.9, cam) is not None


def test_right_in_image_is_right_of_robot():
    _, y_left = gp.floor_point_to_base(0.2, 0.8)
    _, y_right = gp.floor_point_to_base(0.8, 0.8)
    assert y_right < y_left          # base_link y is left-positive


def test_lower_in_image_is_closer():
    near, _ = gp.floor_point_to_base(0.5, 0.9)
    far, _ = gp.floor_point_to_base(0.5, 0.3)
    assert near < far


def test_item_on_arm_centerline_appears_left_of_image_centre():
    # Review finding 1: centerline (y=0) item at 1.0 m ahead is at u~0.38.
    assert gp.floor_point_to_base(0.38, 0.5)[1] > -0.1575
    lo, hi = 0.30, 0.46
    for _ in range(40):
        mid = (lo + hi) / 2
        if gp.floor_point_to_base(mid, 0.6)[1] > 0:
            lo = mid
        else:
            hi = mid
    assert 0.2 < lo < 0.5


def test_invalid_height_returns_none():
    assert gp.floor_point_to_base(0.5, 0.9, gp.CameraModel(height=0.0)) is None


def test_camera_from_degrees():
    cam = gp.camera_from_degrees(0.1, 0.0, 0.5, 30, 60, 40)
    assert abs(cam.pitch_rad - radians(30)) < 1e-12
    assert cam.height == 0.5


def test_build_target_has_base_xy_and_old_fields():
    t = dl.build_target(300, 300, 340, 470, 640, 480, 0.9, 1, 0.12)
    for key in ('floor_point_normalized', 'direction', 'horizontal_error'):
        assert key in t
    assert t['base_x'] is not None and t['base_y'] is not None
    assert 0.6 < t['base_x'] < 0.8
    json.dumps(t)


def test_build_target_above_horizon_gives_null():
    cam = gp.CameraModel(pitch_rad=0.0)
    t = dl.build_target(300, 10, 340, 100, 640, 480, 0.9, 1, 0.12, camera=cam)
    assert t['base_x'] is None and t['base_y'] is None
    json.dumps(t)
