"""Regression tests for review pass 3 (vision and arm)."""
import json
import math

import pytest

from trash_vision import arm_logic as al
from trash_vision import detection_filter as df
from trash_vision import detection_logic as dl
from trash_vision import frame_grabber as fg
from trash_vision import ground_projection as gp


def test_projection_rejects_non_finite_input():
    assert gp.floor_point_to_base(float('nan'), 0.8) is None
    assert gp.floor_point_to_base(0.5, float('inf')) is None


def test_projection_rejects_non_finite_camera():
    cam = gp.CameraModel(height=float('nan'))
    assert gp.floor_point_to_base(0.5, 0.8, cam) is None


def test_projection_defaults_still_work():
    xy = gp.floor_point_to_base(0.5, 0.9)
    assert xy is not None and xy[0] > 0.248


def test_gpt_nan_confidence_is_not_passed_on():
    text = ('{"found": true, "x1": 0.1, "y1": 0.1, "x2": 0.3, "y2": 0.3,'
            ' "confidence": NaN}')
    box = dl.parse_gpt_response(text)
    assert box is not None and math.isfinite(box['confidence'])


def test_filter_rejects_nan_confidence():
    f = df.DetectionFilter(min_confidence=0.4, confirm_frames=1,
                           window_frames=1)
    c = df.Candidate(0.4, 0.4, 0.5, 0.5, float('nan'))
    assert f.update([c]) is None


def test_timer_period_ignores_nan_rate():
    assert fg.timer_period_s(float('nan')) == pytest.approx(0.1)
    assert fg.timer_period_s(float('inf')) == pytest.approx(0.1)


def test_clamp_pose_refuses_nan():
    limits = [(-100.0, 100.0)] * 6
    with pytest.raises(ValueError):
        al.clamp_pose([0, 0, float('nan'), 0, 0, 0], limits)
    assert al.clamp_pose([0, 200, 0, 0, 0, -200], limits)[1] == 100.0


def test_parse_target_xy_good_and_bad():
    good = json.dumps({'detected': True, 'base_x': 0.4, 'base_y': -0.1})
    assert al.parse_target_xy(good) == (0.4, -0.1)
    assert al.parse_target_xy('{"base_x": "abc", "base_y": 0}') is None
    assert al.parse_target_xy('{"base_x": null, "base_y": 0}') is None
    assert al.parse_target_xy('{"base_x": NaN, "base_y": 0}') is None
    assert al.parse_target_xy('{"base_x": true, "base_y": 0}') is None
    assert al.parse_target_xy('[1, 2]') is None
    assert al.parse_target_xy('not json') is None
    assert al.parse_target_xy(None) is None
    assert al.parse_target_xy('{"detected": false}') is None
