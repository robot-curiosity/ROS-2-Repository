"""Run the behavior logic against scripted litter in the pure-Python sim.

No ROS needed. Gives a repeatable "collected fraction" for CI and for
tracking the 80% pickup target in software (not a real-robot number).
"""
from trash_vision import behavior_logic as bl
from trash_vision import safety_logic as sfl
from trash_vision import search_pattern as sp
from trash_vision import sim_logic as sl
from trash_vision import trial_log as tl

# Seconds the simulated arm takes per action (sim only, not measured).
ARM_DURATION = {'pick': 2.0, 'deposit': 1.5, 'stow': 1.0}

# Scripted scenario: 8 items in the open plus 4 near walls (0.3 m) so the
# side and front sonars matter. World is 4 m x 4 m, robot starts at centre.
OPEN_LITTER = [(1.0, 1.0), (3.0, 2.5), (1.5, 3.2), (2.8, 1.2),
               (2.0, 3.0), (1.2, 2.2), (3.0, 3.2), (2.5, 0.9)]
WALL_LITTER = [(0.3, 2.0), (3.7, 1.5), (2.0, 0.3), (1.0, 3.7)]
# Mixed litter types for the noisy, arm-limited scenario (types are sim labels).
TYPED_LITTER = [(x, y, t) for (x, y), t in zip(
    OPEN_LITTER + WALL_LITTER,
    ['can', 'bottle', 'wrapper', 'can', 'bottle', 'wrapper',
     'can', 'bottle', 'wrapper', 'can', 'bottle', 'wrapper'])]
# Hard case: corners and 0.2 m from a wall. Known weak spot (the front
# stop distance blocks the approach), tracked as a baseline, not a goal.
CORNER_LITTER = [(0.15, 0.15), (3.85, 3.85), (0.2, 3.0), (3.8, 0.8)]


class CoverageGrid:
    """Tracks how much of the search area the simulated camera has swept."""

    def __init__(self, world, p):
        half_l = p['search_area_length'] / 2.0
        half_w = p['search_area_width'] / 2.0
        self._cx, self._cy = world.x, world.y
        self.grid = sp.CoverageGrid(-half_l, half_l, -half_w, half_w)
        self._range = p['search_detect_range']
        self._hfov = p['search_camera_hfov_deg']

    def update(self, world):
        self.grid.mark((world.x - self._cx, world.y - self._cy, world.yaw),
                       self._range, self._hfov)

    def fraction(self):
        return self.grid.fraction()


def run_search_only(duration=300.0, **kwargs):
    """Run with no litter and report how much area the search sweeps."""
    return run_mission([], duration=duration, search_only=True, **kwargs)


# With the arm model on, the robot must stop where IK can reach. With the
# placeholder arm mount the reachable band is about 0.35-0.43 m straight
# ahead (narrower off-centre), so aim for 0.38 m. This is what
# calibrate_pick_zone does on the real robot; sim only, not measured.
ARM_PICK_DISTANCE = 0.38
ARM_PICK_PARAMS = {
    'pick_floor_y': 0.5 + sl.FLOOR_K / ARM_PICK_DISTANCE,
    'pick_center_tolerance': 0.08,
}


def type_counts(litter):
    """Count litter items per type (items are (x, y[, type]))."""
    out = {}
    for item in litter:
        kind = sl.litter_type(item)
        out[kind] = out.get(kind, 0) + 1
    return out


def pickup_rate_by_type(result):
    """Return {type: collected / total} from a run_mission result."""
    return {k: result['collected_by_type'].get(k, 0) / n
            for k, n in result['total_by_type'].items()}


def run_mission(litter, duration=600.0, dt=0.05, width=4.0, height=4.0,
                params=None, recorder_sink=None, search_only=False,
                noise=None, arm=None, safety=None):
    """Simulate one mission; return a dict of results.

    Keys: total, collected, fraction, duration, states (visit counts),
    coverage (fraction of the search area the camera swept).
    Also total_by_type and collected_by_type (litter type -> count).
    noise is a sim_logic.SimNoise (seeded) or None; arm is an
    arm_logic.PickConfig or None (see SimWorld).
    safety is a fresh safety_logic.SafetyStage that filters the drive
    commands as the behavior node does, or None to drive them unfiltered.
    safety_stops counts the times that stage began a front hard stop (always
    0 without one).
    The odometry pose is the world pose relative to the start (centre).
    """
    if arm is not None:
        params = dict(ARM_PICK_PARAMS, **(params or {}))
    world = sl.SimWorld(width, height, litter, x=width / 2, y=height / 2,
                        noise=noise, arm=arm)
    total_by_type = type_counts(world.litter)
    machine = bl.BehaviorMachine(params)
    recorder = (tl.TrialRecorder(machine, recorder_sink, 'sim')
                if recorder_sink else None)
    total = len(world.litter)
    cov_params = dict(sp.DEFAULTS)
    cov_params.update(params or {})
    coverage = CoverageGrid(world, cov_params)
    pending = None  # (action, finish_time)
    visits = {}
    last_state = None
    frame = 0
    now = 0.0
    safety_stops = 0
    hard_stopped = False
    while now < duration and (search_only or world.litter or world.held):
        frame += 1
        if pending and now >= pending[1]:
            action = pending[0]
            pending = None
            if action == 'pick':
                ok, _ = world.pick()
            elif action == 'deposit':
                ok, _ = world.deposit()
            else:
                ok = True
            machine.arm_result(ok, now, action)
            if recorder:
                recorder.update(now)
        r = world.ranges()
        target = world.target(frame)
        cmd = machine.step(
            now, target,
            [r['front_1'], r['front_2']], [r['rear_1'], r['rear_2']],
            [r['left_1'], r['left_2']], [r['right_1'], r['right_2']],
            pose=(world.x - width / 2, world.y - height / 2, world.yaw))
        if recorder:
            recorder.update(now, target)
        if cmd.arm_action and pending is None:
            pending = (cmd.arm_action,
                       now + ARM_DURATION[cmd.arm_action])
        linear, angular = cmd.linear, cmd.angular
        if safety is not None:
            safety.note_update(now)
            linear, angular = safety.apply(
                now, linear, angular,
                sfl.front_minimum([r['front_1'], r['front_2']]))
            blocked = safety.reason == 'front_hard_stop'
            safety_stops += 1 if blocked and not hard_stopped else 0
            hard_stopped = blocked
        busy = pending is not None
        world.step(0.0 if busy else linear, 0.0 if busy else angular, dt)
        if machine.state != last_state:
            visits[machine.state] = visits.get(machine.state, 0) + 1
            last_state = machine.state
        coverage.update(world)
        now += dt
    collected = world.collected
    return {'total': total, 'collected': collected,
            'fraction': collected / total if total else 1.0,
            'duration': now, 'states': visits,
            'safety_stops': safety_stops,
            'coverage': coverage.fraction(),
            'total_by_type': total_by_type,
            'collected_by_type': dict(world.collected_by_type),
            'grasp_stats': {k: list(v) for k, v in world.grasp_stats.items()}}
