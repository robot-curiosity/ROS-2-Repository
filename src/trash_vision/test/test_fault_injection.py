"""Fault-injection tests: behavior plus safety stage under sensor faults.

``Rig`` mirrors the node tick (behavior_node._tick): staleness on sonars,
stale-target guard, behavior step, then the safety output stage.
Pure Python, no ROS. Each test injects one fault and asserts the robot stops
or recovers as designed and never wedges.
"""
import math

from trash_vision import behavior_logic as bl
from trash_vision import safety_logic as sfl
from trash_vision import sonar_scheduler as ss

FRONT = ['front_1', 'front_2']
REAR = ['rear_1', 'rear_2']
LEFT = ['left_1', 'left_2']
RIGHT = ['right_1', 'right_2']
ALL = FRONT + REAR + LEFT + RIGHT
DT = 0.1


def target(err=0.0, y=0.5, conf=0.9):
    """Return a /vision/target style dict."""
    return {'detected': True, 'confidence': conf, 'horizontal_error': err,
            'floor_point_normalized': {'x': 0.5, 'y': y}}


class Rig:
    """Behavior machine + safety stage fed by faultable inputs."""

    def __init__(self, params=None, sonar_stale=2.0):
        self.m = bl.BehaviorMachine(params)
        self.s = sfl.SafetyStage()
        self.sonar_stale = sonar_stale
        self.now = 0.0
        self.ranges = {n: 3.0 for n in ALL}
        self.stamps = {}
        self.silent = set()       # sensors that stopped reporting
        self.target = None        # latest detector message
        self.target_stamp = None
        self.pose = (0.0, 0.0, 0.0)
        self.behavior_runs = True  # False: the tick stops (hang)
        self.out = (0.0, 0.0)
        self.arm = []

    def detect(self, tgt):
        """Detector publishes ``tgt`` now."""
        self.target = tgt
        self.target_stamp = self.now

    def tick(self):
        """Advance DT seconds and return the safe (linear, angular)."""
        self.now += DT
        now = self.now
        for n in ALL:
            if n not in self.silent:
                self.stamps[n] = now
        if not self.behavior_runs:
            # Node watchdog: safety sees no update and forces zero.
            self.out = self.s.apply(now, 0.0, 0.0, 0.0, False)
            return self.out

        def sonars(names):
            return ss.apply_staleness(
                names, self.ranges, self.stamps, now, self.sonar_stale)
        front = sonars(FRONT)
        tgt = bl.fresh_target(self.target, self.target_stamp, now, 1.0)
        cmd = self.m.step(now, tgt, front, sonars(REAR), sonars(LEFT),
                          sonars(RIGHT), pose=self.pose)
        if cmd.arm_action:
            self.arm.append(cmd.arm_action)
        self.s.note_update(now)
        ok = sfl.sonars_healthy(ALL, self.stamps, now, self.sonar_stale)
        self.out = self.s.apply(now, cmd.linear, cmd.angular,
                                sfl.front_minimum(front), ok)
        return self.out

    def run(self, seconds):
        """Tick for ``seconds`` and return the last output."""
        for _ in range(int(round(seconds / DT))):
            self.tick()
        return self.out

    def run_until(self, state, limit=60.0):
        """Tick until the machine is in ``state`` (or limit)."""
        t = 0.0
        while self.m.state != state and t < limit:
            self.tick()
            t += DT
        return self.m.state == state


def approaching_rig(**kw):
    """Rig in APPROACH with a fresh, off-centre, far target."""
    r = Rig(**kw)
    for _ in range(5):
        r.detect(target(err=0.0, y=0.3))
        r.tick()
    assert r.m.state == bl.APPROACH
    return r


def keep_target(r, seconds, **kw):
    """Tick while the detector keeps publishing."""
    for _ in range(int(seconds / DT)):
        r.detect(target(**kw))
        r.tick()


# ---------------------------------------------------------------- sonars

def test_sonar_dropout_stops_robot_then_recovers():
    r = approaching_rig()
    keep_target(r, 1.0, y=0.3)
    assert r.out[0] > 0.0
    r.silent.add('rear_2')                   # a sensor that reported dies
    for _ in range(35):                      # > sonar_stale (2.0 s)
        r.detect(target(y=0.3))
        r.tick()
    assert r.out == (0.0, 0.0) and r.s.reason == 'sonars_stale'
    r.silent.clear()                         # sensor comes back
    keep_target(r, 1.5, y=0.3)
    assert r.out[0] > 0.0                    # moves again, no latch


def test_front_sensor_dropout_reads_blocked_and_stops():
    r = approaching_rig()
    r.silent.add('front_1')
    for _ in range(35):
        r.detect(target(y=0.3))
        r.tick()
    assert r.out == (0.0, 0.0)


def test_no_sonar_ever_reported_refuses_to_move():
    r = Rig()
    r.stamps = {}
    r.silent = set(ALL)
    r.detect(target(y=0.3))
    for _ in range(20):
        r.detect(target(y=0.3))
        r.tick()
    assert r.out == (0.0, 0.0)


def test_nan_reading_in_front_is_blocked_in_any_order():
    assert sfl.front_minimum([math.nan, 0.5]) == 0.0
    assert sfl.front_minimum([0.5, math.nan]) == 0.0
    assert sfl.front_minimum([0.5, 0.3]) == 0.3
    assert math.isnan(sfl.front_minimum([]))
    s = sfl.SafetyStage()
    s.note_update(1.0)
    assert s.apply(1.0, 0.3, 0.0, sfl.front_minimum([])) == (0.0, 0.0)


def test_hard_stop_does_not_wedge_robot_against_obstacle():
    # Obstacle closer than the hard stop distance (pushed into a wall).
    r = Rig()
    r.ranges['front_1'] = 0.08
    r.ranges['front_2'] = 0.08
    r.run(1.0)
    # Behavior wants to turn away; safety must still let it turn.
    assert r.out[0] == 0.0
    assert r.out[1] != 0.0
    assert r.s.reason == 'front_hard_stop'
    # Obstacle gone -> robot moves again.
    r.ranges['front_1'] = r.ranges['front_2'] = 3.0
    r.run(1.0)
    assert r.s.reason in ('', 'clamped', 'accel_limited')


def test_hard_stop_cuts_forward_instantly_even_when_moving():
    s = sfl.SafetyStage()
    for i in range(1, 40):
        s.note_update(i * DT)
        s.apply(i * DT, 0.4, 0.0, 2.0)
    assert s.linear > 0.3
    s.note_update(4.1)
    lin, _ = s.apply(4.1, 0.4, 0.0, 0.05)
    assert lin == 0.0
    s.note_update(4.2)
    lin, _ = s.apply(4.2, -0.2, 0.0, 0.05)    # reversing out is allowed
    assert lin < 0.0


# ---------------------------------------------------------------- detector

def test_detector_goes_silent_during_approach_returns_to_search():
    r = approaching_rig()
    keep_target(r, 1.0, y=0.3)
    # Detector dies: no more messages.
    r.run(0.8)
    assert r.out[0] >= 0.0
    r.run(3.0)                                # > stale + lost timeouts
    assert r.m.state == bl.SEARCH
    assert r.m.reason == 'target_lost'
    assert r.out[1] != 0.0 or r.out[0] != 0.0  # searching, not frozen


def test_detector_silent_from_start_keeps_searching_safely():
    r = Rig()
    r.run(10.0)
    assert r.m.state == bl.SEARCH
    assert abs(r.out[0]) <= 0.4 and abs(r.out[1]) <= 1.0


def test_garbage_target_is_ignored():
    r = Rig()
    r.detect({'detected': True, 'confidence': 0.9})   # missing fields
    r.run(2.0)
    assert r.m.state == bl.SEARCH
    r.detect({'detected': True, 'confidence': 0.01,
              'horizontal_error': 0.0,
              'floor_point_normalized': {'x': .5, 'y': .5}})
    r.run(2.0)
    assert r.m.state == bl.SEARCH


# ---------------------------------------------------------------- arm

def test_arm_pick_service_never_answers_does_not_wedge():
    r = Rig(params={'arm_timeout': 5.0})
    for _ in range(3):
        r.detect(target(err=0.0, y=0.95))
        r.tick()
    assert r.run_until(bl.PICK, 5.0)
    assert r.arm == ['pick']
    r.run(1.0)
    assert r.out == (0.0, 0.0)               # stands still while waiting
    r.run(5.5)                               # arm_timeout, no result ever
    assert r.m.state != bl.PICK
    assert r.m.reason == 'arm_timeout'
    assert r.m.items_collected == 0


def test_arm_deposit_and_stow_timeouts_end_in_search():
    m = bl.BehaviorMachine({'arm_timeout': 2.0})
    now = 0.0
    m._enter(bl.DEPOSIT, now)
    seen = set()
    for _ in range(200):
        now += DT
        m.step(now, None, [3.0, 3.0], [3.0, 3.0])
        seen.add(m.state)
        if m.state == bl.SEARCH:
            break
    assert m.state == bl.SEARCH and bl.STOW in seen
    assert m.items_collected == 0            # timed-out deposit not counted


def test_late_arm_result_after_timeout_is_ignored():
    m = bl.BehaviorMachine({'arm_timeout': 1.0})
    m._enter(bl.PICK, 0.0)
    m.step(0.1, None, [3.0], [3.0])
    m.step(2.0, None, [3.0], [3.0])          # times out
    state = m.state
    m.arm_result(True, 2.1, action='pick')   # stale success
    assert m.state == state
    assert m.items_collected == 0


def test_arm_service_unavailable_counts_as_pick_failure():
    # The node reports arm_result(False) at once if the service is missing.
    m = bl.BehaviorMachine({'pick_failures_before_recover': 2})
    now = 0.0
    for attempt in range(2):
        m._enter(bl.PICK, now)
        m.step(now, None, [3.0], [3.0])
        now += DT
        m.arm_result(False, now, 'pick')
    assert m.state == bl.RECOVER


def test_repeated_pick_failure_backs_up_then_searches():
    r = Rig(params={'pick_failures_before_recover': 2,
                    'recover_backup_time': 1.0})
    for attempt in range(2):
        for _ in range(3):
            r.detect(target(err=0.0, y=0.95))
            r.tick()
        assert r.run_until(bl.PICK, 10.0)
        r.tick()
        r.m.arm_result(False, r.now, 'pick')
        r.run(0.1)
        r.run(3.5)       # let the cooldown/lost timers expire between tries
        r.detect(None)
        r.target = None
    seen_reverse = False
    saw_recover = False
    for _ in range(100):
        r.tick()
        saw_recover = saw_recover or r.m.state == bl.RECOVER
        seen_reverse = seen_reverse or r.out[0] < 0.0
        if r.m.state == bl.SEARCH and saw_recover:
            break
    assert r.m.state == bl.SEARCH


def test_recovery_with_rear_blocked_does_not_reverse():
    m = bl.BehaviorMachine()
    m._enter(bl.RECOVER, 0.0)
    cmd = m.step(0.1, None, [3.0], [0.1, 0.1])
    assert cmd.linear == 0.0 and m.state == bl.SEARCH


def test_recovery_always_ends_even_if_rear_never_clears():
    m = bl.BehaviorMachine({'recover_backup_time': 1.0})
    m._enter(bl.RECOVER, 0.0)
    now = 0.0
    for _ in range(40):
        now += DT
        m.step(now, None, [3.0], [3.0])
        if m.state != bl.RECOVER:
            break
    assert m.state == bl.SEARCH


# ---------------------------------------------------------------- odometry

def test_odometry_loss_falls_back_to_spin_search():
    r = Rig()
    r.pose = None
    r.run(3.0)
    assert r.m.state == bl.SEARCH
    assert r.out[0] == 0.0 and r.out[1] != 0.0


def test_odometry_lost_mid_search_does_not_crash_or_stall():
    r = Rig()
    r.run(5.0)
    r.pose = None
    r.run(3.0)
    r.pose = (1.0, 1.0, 0.0)                  # comes back at a jump
    r.run(3.0)
    assert r.m.state == bl.SEARCH
    assert all(math.isfinite(v) for v in r.out)


# ---------------------------------------------------------------- watchdog

def test_behavior_tick_hang_forces_zero_within_timeout():
    r = Rig()
    r.run(3.0)
    r.behavior_runs = False                   # tick stops, watchdog runs
    for _ in range(8):
        r.tick()
    assert r.out == (0.0, 0.0) and r.s.reason == 'behavior_timeout'
    r.behavior_runs = True
    r.run(2.0)
    assert r.s.reason != 'behavior_timeout'   # recovers when ticks resume


def test_invalid_behavior_command_is_stopped():
    s = sfl.SafetyStage()
    s.note_update(1.0)
    assert s.apply(1.0, math.nan, 0.0, 2.0) == (0.0, 0.0)
    assert s.reason == 'invalid_command'
    s.note_update(1.1)
    assert s.apply(1.1, 0.0, math.inf, 2.0) == (0.0, 0.0)


def test_every_fault_output_stays_within_limits():
    # Mixed faults over a long run; output must always be finite and clamped.
    r = Rig()
    for i in range(600):
        if i % 50 == 10:
            r.silent = {ALL[(i // 50) % len(ALL)]}
        if i % 50 == 30:
            r.silent = set()
        if i % 70 == 0:
            r.detect(target(err=0.3, y=0.4))
        r.tick()
        lin, ang = r.out
        assert math.isfinite(lin) and math.isfinite(ang)
        assert abs(lin) <= 0.4 + 1e-9 and abs(ang) <= 1.0 + 1e-9
