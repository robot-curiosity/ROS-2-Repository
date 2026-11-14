import math

from trash_vision import behavior_logic as bl

CLEAR = [math.inf] * 4


def tgt(err=0.0, y=0.5, conf=0.9):
    return {"detected": True, "confidence": conf, "horizontal_error": err,
            "floor_point_normalized": {"x": 0.5, "y": y}}


def test_min_range_handles_inf_and_too_close():
    assert bl.min_range([math.inf, 1.0, 2.0]) == 1.0
    assert bl.min_range([-math.inf, 1.0]) == 0.0
    assert bl.min_range([None, math.nan]) == math.inf


def test_search_spins_without_target():
    m = bl.BehaviorMachine()
    c = m.step(0.0, None, CLEAR, CLEAR)
    assert m.state == bl.SEARCH
    assert c.linear == 0.0 and c.angular > 0


def test_target_starts_approach_and_steers():
    m = bl.BehaviorMachine()
    c = m.step(0.0, tgt(err=0.2), CLEAR, CLEAR)
    assert m.state == bl.APPROACH
    assert c.angular < 0 and c.linear > 0
    c = m.step(0.1, tgt(err=-0.2), CLEAR, CLEAR)
    assert c.angular > 0


def test_low_confidence_ignored():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(conf=0.1), CLEAR, CLEAR)
    assert m.state == bl.SEARCH


def test_target_lost_returns_to_search():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(), CLEAR, CLEAR)
    m.step(1.0, {"detected": False}, CLEAR, CLEAR)
    assert m.state == bl.APPROACH
    m.step(2.0, {"detected": False}, CLEAR, CLEAR)
    assert m.state == bl.SEARCH


def test_obstacle_stops_and_turns_away():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(), CLEAR, CLEAR)
    c = m.step(0.1, tgt(), [0.1, 1.0, 1.0, 1.0], CLEAR)
    assert c.linear == 0.0 and c.angular < 0
    c = m.step(0.2, tgt(), [1.0, 1.0, 1.0, 0.1], CLEAR)
    assert c.linear == 0.0 and c.angular > 0


def test_slow_zone_scales_speed():
    m = bl.BehaviorMachine()
    fast = m.step(0.0, tgt(), CLEAR, CLEAR).linear
    slow = m.step(0.1, tgt(), [0.4] * 4, CLEAR).linear
    assert 0 < slow < fast


def test_full_cycle():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(), CLEAR, CLEAR)
    c = m.step(0.1, tgt(err=0.0, y=0.95), CLEAR, CLEAR)
    assert m.state == bl.PICK and c.arm_action == "pick"
    assert m.step(0.2, None, CLEAR, CLEAR).arm_action is None
    m.arm_result(True, 1.0)
    assert m.state == bl.DEPOSIT
    assert m.step(1.1, None, CLEAR, CLEAR).arm_action == "deposit"
    m.arm_result(True, 2.0)
    assert m.state == bl.STOW and m.items_collected == 1
    assert m.step(2.1, None, CLEAR, CLEAR).arm_action == "stow"
    m.arm_result(True, 3.0)
    assert m.state == bl.SEARCH


def test_not_centered_does_not_pick():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(err=0.3, y=0.95), CLEAR, CLEAR)
    assert m.state == bl.APPROACH


def test_pick_failures_lead_to_recover_backup():
    m = bl.BehaviorMachine()
    for i in range(2):
        t = i * 10.0
        m.step(t, tgt(y=0.95), CLEAR, CLEAR)
        m.step(t + 0.1, tgt(y=0.95), CLEAR, CLEAR)
        assert m.state == bl.PICK
        m.arm_result(False, t + 1)
    assert m.state == bl.RECOVER
    c = m.step(11.5, None, CLEAR, CLEAR)
    assert c.linear < 0
    c = m.step(11.6, None, CLEAR, [0.1] * 4)
    assert m.state == bl.SEARCH and c.linear == 0.0


def test_arm_timeout_counts_as_failure():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(y=0.95), CLEAR, CLEAR)
    m.step(0.1, tgt(y=0.95), CLEAR, CLEAR)
    m.step(100.0, None, CLEAR, CLEAR)
    assert m.state == bl.SEARCH
