import math

from trash_vision import behavior_logic as bl

CLEAR = [math.inf] * 2


def tgt(err=0.0, y=0.5, conf=0.9):
    return {'detected': True, 'confidence': conf, 'horizontal_error': err,
            'floor_point_normalized': {'x': 0.5, 'y': y}}


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
    m.step(1.0, {'detected': False}, CLEAR, CLEAR)
    assert m.state == bl.APPROACH
    m.step(2.0, {'detected': False}, CLEAR, CLEAR)
    assert m.state == bl.SEARCH


def test_obstacle_stops_and_turns_away():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(), CLEAR, CLEAR)
    c = m.step(0.1, tgt(), [0.1, 1.0], CLEAR)
    assert c.linear == 0.0 and c.angular < 0
    c = m.step(0.2, tgt(), [1.0, 0.1], CLEAR)
    assert c.linear == 0.0 and c.angular > 0


def test_slow_zone_scales_speed():
    m = bl.BehaviorMachine()
    fast = m.step(0.0, tgt(), CLEAR, CLEAR).linear
    slow = m.step(0.1, tgt(), [0.4] * 2, CLEAR).linear
    assert 0 < slow < fast


def test_full_cycle():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(), CLEAR, CLEAR)
    c = m.step(0.1, tgt(err=0.0, y=0.95), CLEAR, CLEAR)
    assert m.state == bl.PICK and c.arm_action == 'pick'
    assert m.step(0.2, None, CLEAR, CLEAR).arm_action is None
    m.arm_result(True, 1.0)
    assert m.state == bl.DEPOSIT
    assert m.step(1.1, None, CLEAR, CLEAR).arm_action == 'deposit'
    m.arm_result(True, 2.0)
    assert m.state == bl.STOW and m.items_collected == 1
    assert m.step(2.1, None, CLEAR, CLEAR).arm_action == 'stow'
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
    c = m.step(11.6, None, CLEAR, [0.1] * 2)
    assert m.state == bl.SEARCH and c.linear == 0.0


def test_arm_timeout_counts_as_failure():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(y=0.95), CLEAR, CLEAR)
    m.step(0.1, tgt(y=0.95), CLEAR, CLEAR)
    m.step(100.0, None, CLEAR, CLEAR)
    assert m.state == bl.SEARCH


# --- Wedge-proofing: every state must return to SEARCH on its own ---------
BLOCKED = [0.1] * 2
# Never reaches the pick zone: target visible but floor y stays small.
FAR = {'err': 0.0, 'y': 0.3}


def _drive_until_search(m, start, end, target, front=CLEAR, rear=CLEAR,
                        dt=0.5):
    t = start
    while t <= end:
        m.step(t, target() if callable(target) else target, front, rear)
        if m.state == bl.SEARCH:
            return t
        t += dt
    return None


def test_search_is_home_state_and_never_requests_arm():
    m = bl.BehaviorMachine()
    for i in range(200):
        c = m.step(i * 0.5, None, CLEAR, CLEAR)
        assert m.state == bl.SEARCH and c.arm_action is None


def test_approach_blocked_by_obstacle_times_out_to_search():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(**FAR), CLEAR, CLEAR)
    assert m.state == bl.APPROACH
    t = _drive_until_search(
        m, 0.5, 60.0, lambda: tgt(**FAR), front=BLOCKED)
    assert t is not None and t <= m.p['approach_timeout'] + 1.0


def test_approach_never_reaching_pick_zone_times_out():
    m = bl.BehaviorMachine()
    t = _drive_until_search(m, 0.0, 60.0, lambda: tgt(**FAR))
    assert t is not None


def test_approach_cooldown_ignores_same_target_then_resumes():
    m = bl.BehaviorMachine()
    t = _drive_until_search(m, 0.0, 60.0, lambda: tgt(**FAR))
    m.step(t + 0.5, tgt(**FAR), CLEAR, CLEAR)
    assert m.state == bl.SEARCH
    m.step(t + m.p['approach_cooldown'] + 0.5, tgt(**FAR), CLEAR, CLEAR)
    assert m.state == bl.APPROACH


def test_pick_deposit_stow_each_time_out_to_search():
    for state, setup in ((bl.PICK, 0), (bl.DEPOSIT, 1), (bl.STOW, 2)):
        m = bl.BehaviorMachine()
        m.step(0.0, tgt(y=0.95), CLEAR, CLEAR)
        m.step(0.1, tgt(y=0.95), CLEAR, CLEAR)
        if setup >= 1:
            m.arm_result(True, 1.0)
        if setup >= 2:
            m.arm_result(True, 2.0)
        assert m.state == state
        t = 3.0
        # Arm service never answers: walk the clock forward.
        while m.state == state and t < 200.0:
            m.step(t, None, CLEAR, CLEAR)
            t += 1.0
        assert m.state != state, state
        # Whatever the path (RECOVER included) it must reach SEARCH.
        while m.state != bl.SEARCH and t < 400.0:
            m.step(t, None, CLEAR, CLEAR)
            t += 1.0
        assert m.state == bl.SEARCH, state


def test_recover_returns_to_search_by_time_and_by_rear_sensor():
    for rear, expect_t in ((CLEAR, 1.5), ([0.1] * 2, 0.0)):
        m = bl.BehaviorMachine()
        m._enter(bl.RECOVER, 0.0)
        t = 0.0
        while m.state == bl.RECOVER and t < 10.0:
            m.step(t, None, CLEAR, rear)
            t += 0.5
        assert m.state == bl.SEARCH and t - 0.5 <= expect_t + 0.5


def test_stale_arm_result_after_timeout_is_ignored():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(y=0.95), CLEAR, CLEAR)
    m.step(0.1, tgt(y=0.95), CLEAR, CLEAR)
    m.step(100.0, None, CLEAR, CLEAR)  # pick timed out -> SEARCH
    assert m.state == bl.SEARCH
    m.arm_result(True, 101.0, 'pick')  # late reply
    assert m.state == bl.SEARCH and m.items_collected == 0


def test_stale_result_for_other_phase_does_not_advance():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(y=0.95), CLEAR, CLEAR)
    m.step(0.1, tgt(y=0.95), CLEAR, CLEAR)
    m.arm_result(True, 1.0, 'deposit')
    assert m.state == bl.PICK


def test_fresh_target_drops_stale_detection():
    t = tgt()
    assert bl.fresh_target(t, 10.0, 10.5, 1.0) is t
    assert bl.fresh_target(t, 10.0, 12.0, 1.0) is None
    assert bl.fresh_target(t, None, 12.0, 1.0) is None
    assert bl.fresh_target(None, 10.0, 10.1, 1.0) is None


# --- Side clearance for rotation in place (sonar layout item 10) -----------
NEAR = [0.1, 0.1]


def test_search_spin_refused_toward_close_left_side_flips_to_right():
    m = bl.BehaviorMachine()
    c = m.step(0.0, None, CLEAR, CLEAR, NEAR, CLEAR)
    assert c.linear == 0.0 and c.angular < 0  # CCW refused, spins CW instead
    c = m.step(0.1, None, CLEAR, CLEAR, CLEAR, CLEAR)
    assert c.angular < 0  # direction stays flipped once clear


def test_search_spin_refused_toward_close_right_side():
    m = bl.BehaviorMachine()
    m._avoid_dir = -1.0  # currently spinning CW
    c = m.step(0.0, None, CLEAR, CLEAR, CLEAR, NEAR)
    assert c.angular > 0


def test_search_spin_zero_when_both_sides_close():
    m = bl.BehaviorMachine()
    c = m.step(0.0, None, CLEAR, CLEAR, NEAR, NEAR)
    assert c.linear == 0.0 and c.angular == 0.0


def test_side_clearance_is_a_parameter():
    m = bl.BehaviorMachine({'side_clearance': 0.05})
    c = m.step(0.0, None, CLEAR, CLEAR, NEAR, NEAR)
    assert c.angular > 0
    m = bl.BehaviorMachine({'side_clearance': 0.5})
    c = m.step(0.0, None, CLEAR, CLEAR, [0.4, 2.0], CLEAR)
    assert c.angular < 0


def test_side_sensor_on_far_side_does_not_limit_spin():
    m = bl.BehaviorMachine()
    c = m.step(0.0, None, CLEAR, CLEAR, CLEAR, NEAR)
    assert c.angular > 0  # CCW swings left; right is close but irrelevant


def test_avoidance_turn_refused_not_flipped_into_front_obstacle():
    m = bl.BehaviorMachine()
    m.step(0.0, tgt(), CLEAR, CLEAR)
    # Front-left blocked -> would turn CW (right); right side is close.
    c = m.step(0.1, tgt(), [0.1, 1.0], CLEAR, CLEAR, NEAR)
    assert c.linear == 0.0 and c.angular == 0.0


def test_moving_arc_is_not_limited_by_side_sensors():
    m = bl.BehaviorMachine()
    c = m.step(0.0, tgt(err=0.1), CLEAR, CLEAR, NEAR, NEAR)
    assert c.linear > 0.02 and c.angular != 0.0


def test_missing_side_lists_mean_no_restriction():
    m = bl.BehaviorMachine()
    assert m.step(0.0, None, CLEAR, CLEAR).angular > 0


def test_recover_backs_up_using_rear_not_front():
    m = bl.BehaviorMachine()
    m._enter(bl.RECOVER, 0.0)
    c = m.step(0.1, None, [0.05, 0.05], CLEAR, NEAR, NEAR)
    assert c.linear < 0 and m.state == bl.RECOVER


def test_stale_front_sonar_stops_forward_motion():
    from trash_vision import sonar_scheduler as ss
    names = ['front_1', 'front_2']
    ranges = {'front_1': 2.0, 'front_2': 2.0}

    def drive(stamps):
        m = bl.BehaviorMachine()
        front = ss.apply_staleness(names, ranges, stamps, 10.0, 2.0)
        return m.step(10.0, tgt(), front, CLEAR)

    assert drive({'front_1': 9.9, 'front_2': 9.9}).linear > 0.0
    assert drive({'front_1': 5.0, 'front_2': 9.9}).linear <= 0.0


def _target(**over):
    tgt = {'detected': True, 'confidence': 0.9, 'horizontal_error': 0.1,
           'floor_point_normalized': {'x': 0.5, 'y': 0.5}}
    tgt.update(over)
    return tgt


def test_review_malformed_targets_are_rejected():
    bad = [
        _target(horizontal_error=math.nan),
        _target(horizontal_error=None),
        _target(confidence=None),
        _target(confidence='high'),
        _target(floor_point_normalized={'x': 0.5}),
        _target(floor_point_normalized={'x': 0.5, 'y': math.nan}),
        _target(floor_point_normalized=None),
        ['not', 'a', 'dict'],
    ]
    assert bl.valid_target(_target(), 0.3)
    for tgt in bad:
        assert not bl.valid_target(tgt, 0.3), tgt


def test_review_nan_target_never_steers_the_robot():
    m = bl.BehaviorMachine({'search_pattern': 'spin'})
    cmd = m.step(1.0, _target(horizontal_error=math.nan), [2.0, 2.0],
                 [2.0, 2.0])
    assert m.state == bl.SEARCH
    assert cmd.linear == 0.0 and abs(cmd.angular) == 0.5


def test_review_nan_pose_falls_back_to_spin():
    m = bl.BehaviorMachine()
    cmd = m.step(1.0, None, [2.0, 2.0], [2.0, 2.0],
                 pose=(math.nan, 0.0, 0.0))
    assert cmd.linear == 0.0
    assert abs(cmd.angular) == m.p['search_turn_speed']
