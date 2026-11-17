"""Run the behavior logic against scripted litter in the pure-Python sim.

No ROS needed. Gives a repeatable "collected fraction" for CI and for
tracking the 80% pickup target in software (not a real-robot number).
"""
from trash_vision import behavior_logic as bl
from trash_vision import search_pattern as sp
from trash_vision import sim_logic as sl
from trash_vision import trial_log as tl

# Seconds the simulated arm takes per action (sim only, not measured).
ARM_DURATION = {"pick": 2.0, "deposit": 1.5, "stow": 1.0}

# Scripted scenario: 8 items in the open plus 4 near walls (0.3 m) so the
# side and front sonars matter. World is 4 m x 4 m, robot starts at centre.
OPEN_LITTER = [(1.0, 1.0), (3.0, 2.5), (1.5, 3.2), (2.8, 1.2),
               (2.0, 3.0), (1.2, 2.2), (3.0, 3.2), (2.5, 0.9)]
WALL_LITTER = [(0.3, 2.0), (3.7, 1.5), (2.0, 0.3), (1.0, 3.7)]
# Hard case: corners and 0.2 m from a wall. Known weak spot (the front
# stop distance blocks the approach), tracked as a baseline, not a goal.
CORNER_LITTER = [(0.15, 0.15), (3.85, 3.85), (0.2, 3.0), (3.8, 0.8)]


class CoverageGrid:
    """Tracks how much of the search area the simulated camera has swept."""

    def __init__(self, world, p):
        half_l = p["search_area_length"] / 2.0
        half_w = p["search_area_width"] / 2.0
        self._cx, self._cy = world.x, world.y
        self.grid = sp.CoverageGrid(-half_l, half_l, -half_w, half_w)
        self._range = p["search_detect_range"]
        self._hfov = p["search_camera_hfov_deg"]

    def update(self, world):
        self.grid.mark((world.x - self._cx, world.y - self._cy, world.yaw),
                       self._range, self._hfov)

    def fraction(self):
        return self.grid.fraction()


def run_search_only(duration=300.0, **kwargs):
    """No litter: how much area does the search pattern sweep?"""
    return run_mission([], duration=duration, search_only=True, **kwargs)


def run_mission(litter, duration=600.0, dt=0.05, width=4.0, height=4.0,
                params=None, recorder_sink=None, search_only=False):
    """Simulate one mission; return a dict of results.

    Keys: total, collected, fraction, duration, states (visit counts),
    coverage (fraction of the search area the camera swept).
    The odometry pose is the world pose relative to the start (centre).
    """
    world = sl.SimWorld(width, height, litter, x=width / 2, y=height / 2)
    machine = bl.BehaviorMachine(params)
    recorder = (tl.TrialRecorder(machine, recorder_sink, "sim")
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
    while now < duration and (search_only or world.litter or world.held):
        frame += 1
        if pending and now >= pending[1]:
            action = pending[0]
            pending = None
            if action == "pick":
                ok, _ = world.pick()
            elif action == "deposit":
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
            [r["front_1"], r["front_2"]], [r["rear_1"], r["rear_2"]],
            [r["left_1"], r["left_2"]], [r["right_1"], r["right_2"]],
            pose=(world.x - width / 2, world.y - height / 2, world.yaw))
        if recorder:
            recorder.update(now, target)
        if cmd.arm_action and pending is None:
            pending = (cmd.arm_action,
                       now + ARM_DURATION[cmd.arm_action])
        busy = pending is not None
        world.step(0.0 if busy else cmd.linear,
                   0.0 if busy else cmd.angular, dt)
        if machine.state != last_state:
            visits[machine.state] = visits.get(machine.state, 0) + 1
            last_state = machine.state
        coverage.update(world)
        now += dt
    collected = world.collected
    return {"total": total, "collected": collected,
            "fraction": collected / total if total else 1.0,
            "duration": now, "states": visits,
            "coverage": coverage.fraction()}
