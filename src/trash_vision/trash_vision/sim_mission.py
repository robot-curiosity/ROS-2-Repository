"""Run the behavior logic against scripted litter in the pure-Python sim.

No ROS needed. Gives a repeatable "collected fraction" for CI and for
tracking the 80% pickup target in software (not a real-robot number).
"""
from trash_vision import behavior_logic as bl
from trash_vision import sim_logic as sl

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


def run_mission(litter, duration=600.0, dt=0.05, width=4.0, height=4.0,
                params=None):
    """Simulate one mission; return a dict of results.

    Keys: total, collected, fraction, duration, states (visit counts).
    """
    world = sl.SimWorld(width, height, litter, x=width / 2, y=height / 2)
    machine = bl.BehaviorMachine(params)
    total = len(world.litter)
    pending = None  # (action, finish_time)
    visits = {}
    last_state = None
    frame = 0
    now = 0.0
    while now < duration and (world.litter or world.held):
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
        r = world.ranges()
        cmd = machine.step(
            now, world.target(frame),
            [r["front_1"], r["front_2"]], [r["rear_1"], r["rear_2"]],
            [r["left_1"], r["left_2"]], [r["right_1"], r["right_2"]])
        if cmd.arm_action and pending is None:
            pending = (cmd.arm_action,
                       now + ARM_DURATION[cmd.arm_action])
        busy = pending is not None
        world.step(0.0 if busy else cmd.linear,
                   0.0 if busy else cmd.angular, dt)
        if machine.state != last_state:
            visits[machine.state] = visits.get(machine.state, 0) + 1
            last_state = machine.state
        now += dt
    collected = world.collected
    return {"total": total, "collected": collected,
            "fraction": collected / total if total else 1.0,
            "duration": now, "states": visits}
