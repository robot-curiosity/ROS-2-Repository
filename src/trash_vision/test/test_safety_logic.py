"""Tests for safety logic."""

import math

from trash_vision import safety_logic as sl


def fresh_stage(**over):
    """Build a SafetyStage with defaults overridden by ``over``."""
    s = sl.SafetyStage(over)
    s.note_update(0.0)
    return s


def test_passes_small_command_after_ramp():
    """Check passes small command after ramp."""
    s = fresh_stage()
    out = (0.0, 0.0)
    for i in range(1, 30):
        s.note_update(i * 0.1)
        out = s.apply(i * 0.1, 0.2, 0.3, 2.0)
    assert out == (0.2, 0.3)
    assert s.reason == ''


def test_speed_clamped():
    """Check speed clamped."""
    s = fresh_stage(safety_max_linear=0.3, safety_max_angular=0.5)
    out = (0, 0)
    for i in range(1, 60):
        s.note_update(i * 0.1)
        out = s.apply(i * 0.1, 5.0, -5.0, 2.0)
    assert out == (0.3, -0.5)


def test_acceleration_limited():
    """Check acceleration limited."""
    s = fresh_stage(safety_max_linear_accel=0.5)
    s.note_update(0.1)
    s.apply(0.1, 0.0, 0.0, 2.0)
    s.note_update(0.2)
    lin, _ = s.apply(0.2, 0.4, 0.0, 2.0)
    assert math.isclose(lin, 0.05)
    assert s.reason == 'accel_limited'


def test_first_command_starts_from_rest():
    """Check first command starts from rest."""
    s = fresh_stage()
    lin, ang = s.apply(0.1, 0.4, 1.0, 2.0)
    assert 0 < lin < 0.4 and 0 < ang < 1.0


def test_decel_limited_for_normal_command():
    """Check decel limited for normal command."""
    s = fresh_stage()
    for i in range(1, 40):
        s.note_update(i * 0.1)
        s.apply(i * 0.1, 0.4, 0.0, 2.0)
    s.note_update(4.1)
    lin, _ = s.apply(4.1, 0.0, 0.0, 2.0)
    assert 0.3 < lin < 0.4


def test_hard_stop_forces_zero_immediately():
    """Check hard stop forces zero immediately."""
    s = fresh_stage(safety_hard_stop_distance=0.15,
                    safety_hard_stop_escape=False)
    for i in range(1, 40):
        s.note_update(i * 0.1)
        s.apply(i * 0.1, 0.4, 0.5, 2.0)
    s.note_update(4.1)
    assert s.apply(4.1, 0.4, 0.5, 0.10) == (0.0, 0.0)
    assert s.reason == 'front_hard_stop'
    assert (s.linear, s.angular) == (0.0, 0.0)


def test_front_exactly_at_hard_stop_is_allowed():
    """Check front exactly at hard stop is allowed."""
    s = fresh_stage(safety_hard_stop_distance=0.15)
    s.apply(0.1, 0.1, 0.0, 0.15)
    assert s.reason != 'front_hard_stop'


def test_stale_sonars_force_zero():
    """Check stale sonars force zero."""
    s = fresh_stage()
    assert s.apply(0.1, 0.2, 0.0, 2.0, sonars_ok=False) == (0.0, 0.0)
    assert s.reason == 'sonars_stale'


def test_behavior_timeout_forces_zero():
    """Check behavior timeout forces zero."""
    s = fresh_stage(safety_behavior_timeout=0.5)
    s.note_update(1.0)
    assert not s.timed_out(1.4)
    assert s.timed_out(1.6)
    assert s.apply(1.6, 0.2, 0.0, 2.0) == (0.0, 0.0)
    assert s.reason == 'behavior_timeout'


def test_never_updated_is_timed_out():
    """Check never updated is timed out."""
    s = sl.SafetyStage()
    assert s.timed_out(0.0)
    assert s.apply(0.0, 0.2, 0.0, 2.0) == (0.0, 0.0)


def test_invalid_commands_force_zero():
    """Check invalid commands force zero."""
    s = fresh_stage()
    assert s.apply(0.1, math.nan, 0.0, 2.0) == (0.0, 0.0)
    assert s.apply(0.1, 0.0, math.inf, 2.0) == (0.0, 0.0)
    assert s.reason == 'invalid_command'


def test_unknown_front_min_forces_zero():
    """Check unknown front min forces zero."""
    s = fresh_stage()
    assert s.apply(0.1, 0.2, 0.0, None) == (0.0, 0.0)
    assert s.apply(0.1, 0.2, 0.0, math.nan) == (0.0, 0.0)


def test_restart_after_stop_ramps_from_rest():
    """Check restart after stop ramps from rest."""
    s = fresh_stage()
    for i in range(1, 40):
        s.note_update(i * 0.1)
        s.apply(i * 0.1, 0.4, 0.0, 2.0)
    s.note_update(4.1)
    s.apply(4.1, 0.4, 0.0, 0.05)
    s.note_update(4.2)
    lin, _ = s.apply(4.2, 0.4, 0.0, 2.0)
    assert 0 < lin <= 0.05 + 1e-9


def test_sonars_healthy_rules():
    """Check sonars healthy rules."""
    stamps = {'a': 9.5, 'b': 9.9}
    assert sl.sonars_healthy(['a', 'b', 'c'], stamps, 10.0, 2.0)
    assert not sl.sonars_healthy(['a', 'b'], stamps, 12.0, 2.0)
    assert not sl.sonars_healthy(['a'], {}, 10.0, 2.0)
    assert sl.sonars_healthy(['a'], {}, 10.0, 2.0, require_any=False)
    # One stale sensor among fresh ones is unhealthy.
    assert not sl.sonars_healthy(['a', 'b'], {'a': 1.0, 'b': 9.9}, 10.0, 2.0)


def test_review_reset_forgets_motion_and_requires_new_update():
    s = sl.SafetyStage()
    s.note_update(0.0)
    s.apply(0.1, 0.3, 0.0, 2.0)
    s.reset()
    assert (s.linear, s.angular) == (0.0, 0.0)
    assert s.timed_out(0.2)
    assert s.apply(0.2, 0.3, 0.0, 2.0) == (0.0, 0.0)
