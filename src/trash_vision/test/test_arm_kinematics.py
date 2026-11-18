import math

import pytest

from trash_vision import arm_kinematics as ak

ARM = ak.SO101


def _reachable():
    out = []
    for q1, q2, q3 in [(0.2, -0.3, 0.1), (-0.3, 0.5, -0.2),
                       (0.6, -0.8, 0.4)]:
        r_terms = [
            ARM.l1 * math.cos(ARM.a1 + q1),
            ARM.l2 * math.cos(ARM.a2 + q1 + q2),
            ARM.l3 * math.cos(ARM.a3 + q1 + q2 + q3)]
        r = ARM.origin_r + sum(r_terms)
        z_terms = [
            ARM.l1 * math.sin(ARM.a1 + q1),
            ARM.l2 * math.sin(ARM.a2 + q1 + q2),
            ARM.l3 * math.sin(ARM.a3 + q1 + q2 + q3)]
        z = ARM.origin_z + sum(z_terms)
        out.append((r, z, ARM.a3 + q1 + q2 + q3))
    return out


def test_zero_pose_tip_matches_urdf_fk():
    # URDF FK at zero pose: tip x 0.39136, z 0.22647 (pan axis x 0.03884).
    r, z = ARM.tip(0, 0, 0)
    assert r + ARM.axis_xy[0] == pytest.approx(0.39136, abs=1e-4)
    assert z == pytest.approx(0.22647, abs=1e-4)


def test_planar_roundtrip_and_bearing():
    for pan in (0.0, 0.4, -0.7):
        pt = ARM.from_planar(0.2, -0.1, pan)
        r, z = ARM.to_planar(pt, pan)
        assert r == pytest.approx(0.2)
        assert z == pytest.approx(-0.1)
        assert ARM.bearing(pt) == pytest.approx(pan, abs=1e-3)


def test_solution_round_trips_through_forward_kinematics():
    for r, z, approach in _reachable():
        for pan in (0.0, 0.4, -0.7):
            target = ARM.from_planar(r, z, pan)
            s = ak.solve(ARM, target, approach)
            assert s is not None
            tr, tz = ARM.tip(s.shoulder_lift, s.elbow_flex, s.wrist_flex)
            assert tr == pytest.approx(r, abs=1e-6)
            assert tz == pytest.approx(z, abs=1e-6)
            assert s.shoulder_pan == pytest.approx(pan, abs=1e-3)


def test_solution_respects_limits():
    for r, z, approach in _reachable():
        s = ak.solve(ARM, ARM.from_planar(r, z), approach)
        for name in ('shoulder_lift', 'elbow_flex', 'wrist_flex'):
            low, high = ak.JOINT_LIMITS[name]
            assert low <= getattr(s, name) <= high


def test_unreachable_returns_none_never_clamps():
    assert ak.solve(ARM, (1.5, 0.0, 0.0), -math.pi / 2) is None
    assert ak.solve(ARM, (0.0, 0.0, 0.0), -math.pi / 2) is None


def test_pan_limit_rejects_behind_the_arm():
    assert ak.solve(ARM, (-0.3, 0.0, -0.1), -math.pi / 2) is None


def test_limit_violation_returns_none():
    tight = dict(ak.JOINT_LIMITS)
    tight['elbow_flex'] = (-0.01, 0.01)
    r, z, approach = _reachable()[0]
    assert ak.solve(ARM, ARM.from_planar(r, z), approach, tight) is None


def test_reachable_radius_band_is_contiguous_and_solvable():
    band = ak.reachable_radius_band(
        ARM, -0.185, -math.pi / 2, clearance_height=0.06)
    assert band is not None
    low, high = band
    assert 0.0 < low < high < 0.4
    mid = (low + high) / 2
    assert ak.graspable(ARM, mid, -0.185, -math.pi / 2, 0.06)
    assert not ak.graspable(ARM, high + 0.05, -0.185, -math.pi / 2, 0.06)


def test_band_none_when_nothing_reachable():
    assert ak.reachable_radius_band(ARM, 2.0, -math.pi / 2) is None
