#!/usr/bin/env python3
"""Try a grasp many times, varying one thing, in a single simulator session.

    bash scripts/dev.sh bash -c 'ros2 launch drivebase_sim sim.launch.py \
        world:=grasp_lab headless:=true litter_detector:=false \
        > /tmp/sim.log 2>&1 & sleep 28; python3 /ws/scripts/grasp_lab.py steps'

`verify_pickup.py` is the regression test: fixed parameters, a verdict, an
exit code. This is the bench beside it. A trial here costs ~25 s instead of a
~90 s simulator start, which is the difference between asking one question and
sweeping a parameter.

WHY ONE PROBE, RE-POSED. Spawning a probe per trial is what the earlier sweeps
did and it corrupts them: `/world/<w>/remove` reports success and leaves the
model, so trial N+1 spawns inside the corpse of trial N and both are flung
metres. Reproducibly, at exactly the sizes worth measuring. So there is one
probe for the whole session and `set_pose` moves it - and the arm is lifted
clear and opened first, because posing a body the jaws are touching is
silently refused by the solver and leaves it where it was.

Sweeps, as the first argument:

    steps     descent waypoints: 1 (one-shot) .. 8
    offset    radial offset of the object from the aim frame
    open      gripper_open
    yaw       object rotation, 0..90 deg - only interesting for a box probe
    repeat    the same trial many times, for how repeatable any of this is

Then the probe, and optionally its friction:

    grasp_lab.py yaw wrapper        the test field's 80 x 50 x 24 mm wrapper
    grasp_lab.py repeat can 3.0     the 66 mm can, at triple rubber friction

Size and shape are per SESSION, not per trial, because respawning is what
corrupts a sweep. A sweep varies only what `set_pose` and the joint commands
can vary.
"""

import math
import subprocess
import sys

import rclpy
import tf2_ros
from rclpy.node import Node
from std_msgs.msg import Float64

sys.path.insert(0, "/ws/src/drivebase_behaviour")

from drivebase_behaviour.ik import (  # noqa: E402
    JOINT_LIMITS,
    reachable_radius_band,
    solve,
)
from drivebase_behaviour.kinematics import (  # noqa: E402
    extract_planar_arm,
    forward_kinematics,
    load_chain,
)

WORLD = "grasp_lab"
PROBE = "grasp_probe"
URDF_XACRO = "/ws/src/drivebase_description/urdf/drivebase.urdf.xacro"
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
          "wrist_roll", "gripper"]

# coordinator.yaml.
GRASP_HEIGHT = 0.020
GRASP_APPROACH_HEIGHT = 0.060
GRIPPER_OPEN = 1.0
GRIPPER_CLOSED = JOINT_LIMITS["gripper"][0]
APPROACH_ANGLE = -math.pi / 2.0

# Defaults every sweep varies one of.
STEPS = 6
OFFSET = 0.004          # beyond one half-width, from the outermost swept x
MASS = 0.015

# 4.0 is NOT enough and the symptom is subtle: the jaws end 11.2 mm above the
# grasp plane on the first trial and 17.7 mm by the sixth, because the arm is
# still crawling in on its integrator when the trial reads its position and
# starts the descent. Every trial then measures a moving arm, and none of them
# lift. See docs/STATUS.md on joints holding short of their command.
SETTLE = 6.0
LIFTED = 0.020
# Just enough to lift the jaws clear of the tallest probe so it can be posed.
# 0.25 was a long trip that arrived unsettled and cost more than it bought.
PARK = 0.15


def gz(args: list[str]) -> str:
    return subprocess.run(["gz"] + args, capture_output=True, text=True,
                          timeout=20).stdout


def probe_pose() -> tuple[float, float, float]:
    text = gz(["model", "-m", PROBE, "-p"])
    for line in text.splitlines():
        if line.strip().startswith("["):
            numbers = [float(v) for v in line.strip().strip("[]").split()]
            if len(numbers) >= 3:
                return tuple(numbers[:3])
    sys.exit(f"could not read {PROBE} pose:\n{text}")


def geometry(radius: float, height: float, box: tuple | None) -> str:
    if box is None:
        return (f'<cylinder><radius>{radius}</radius>'
                f'<length>{height}</length></cylinder>')
    return f'<box><size>{box[0]} {box[1]} {height}</size></box>'


def spawn(radius: float, height: float, x: float, y: float,
          box: tuple | None = None, mu: float = 1.0) -> None:
    shape = geometry(radius, height, box)
    sdf = (f'<?xml version="1.0"?><sdf version="1.9"><model name="{PROBE}">'
           f'<pose>{x} {y} {height/2} 0 0 0</pose><link name="link">'
           f'<inertial><mass>{MASS}</mass><inertia>'
           f'<ixx>7e-6</ixx><iyy>7e-6</iyy><izz>4e-6</izz>'
           f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
           f'<collision name="c"><geometry>{shape}</geometry>'
           f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2>'
           f'</ode></friction></surface></collision>'
           f'<visual name="v"><geometry>{shape}</geometry></visual>'
           f'</link></model></sdf>')
    gz(["service", "-s", f"/world/{WORLD}/create",
        "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000", "--req", f"sdf: '{sdf}'"])


def place(x: float, y: float, height: float, yaw: float = 0.0) -> None:
    gz(["service", "-s", f"/world/{WORLD}/set_pose",
        "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000",
        "--req", f'name: "{PROBE}", position: {{x: {x}, y: {y}, '
                 f'z: {height/2}}}, orientation: {{z: {math.sin(yaw/2)}, '
                 f'w: {math.cos(yaw/2)}}}'])


class Rig:
    """The arm, the probe, and one trial of the grasp."""

    def __init__(self, node: Node, arm, grasp) -> None:
        self.node = node
        self.arm = arm
        self.grasp = grasp
        self.publishers = {
            j: node.create_publisher(Float64, f"/arm/{j}/position", 10)
            for j in JOINTS}
        self.buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buffer, node)
        self.held: dict[str, float] = {}

    def spin(self, seconds: float) -> None:
        # Held and re-published, not fired once: a volatile publisher whose
        # subscriber has not appeared yet discards into nothing, and the gz
        # bridge is regularly not there for the first command.
        end = self.node.get_clock().now().nanoseconds + seconds * 1e9
        while rclpy.ok() and self.node.get_clock().now().nanoseconds < end:
            for joint, value in self.held.items():
                self.publishers[joint].publish(Float64(data=value))
            for _ in range(5):
                rclpy.spin_once(self.node, timeout_sec=0.04)

    def at(self, height: float, gripper: float):
        """Command the tip `height` above the grasp plane. Returns the pose."""
        point = (self.grasp[0], self.grasp[1], self.grasp[2] + height)
        solution = solve(self.arm, point, APPROACH_ANGLE, JOINT_LIMITS)
        if solution is None:
            # Loudly. Returning None here would leave the previous command
            # held, so the arm would simply not move - and every trial after
            # it would be measuring a probe the jaws are still standing on.
            sys.exit(f"no IK {height*1000:.0f} mm above the grasp plane")
        self.held.update({
            "shoulder_pan": solution.shoulder_pan,
            "shoulder_lift": solution.shoulder_lift,
            "elbow_flex": solution.elbow_flex,
            "wrist_flex": solution.wrist_flex,
            "wrist_roll": 0.0,
            "gripper": gripper,
        })
        return solution

    def jaws(self) -> tuple[float, float, float]:
        t = self.buffer.lookup_transform("base_footprint",
                                         "arm_gripper_frame_link",
                                         rclpy.time.Time()).transform
        return t.translation.x, t.translation.y, t.translation.z

    def descend(self, steps: int, gripper: float) -> None:
        """Down the column, then STOP before anything closes on anything.

        The dwell at each waypoint is a floor, not a share of a budget - at
        eight steps a proportional dwell is under a second and the arm never
        arrives. The bottom then gets settled properly, because that is where
        the grip happens and the jaws must not still be moving when they shut.
        Measured without it: the descent bottom sat 12.6 mm above the plane at
        one waypoint and 17.3 mm at three, purely because the per-waypoint
        dwell shrank - the jaws were closing on the way down.
        """
        for step in range(1, steps + 1):
            self.at(GRASP_APPROACH_HEIGHT * (1.0 - step / steps), gripper)
            self.spin(max(1.5, SETTLE / steps))
        self.settled_at(0.0, gripper)

    def settled_at(self, height: float, gripper: float) -> tuple:
        """Command a height and wait until the jaws stop moving.

        Polling rather than a fixed sleep because the arm converges at wildly
        different rates depending on how far it came - and a trial that starts
        its descent early is measuring the tail of the previous move.
        """
        self.at(height, gripper)
        previous = None
        for _ in range(6):
            self.spin(SETTLE / 2.0)
            current = self.jaws()
            if previous is not None and \
                    math.dist(current, previous) < 0.0005:
                return current
            previous = current
        return current

    def trial(self, probe, steps: int, offset: float, opening: float,
              yaw: float) -> dict:
        # Park clear and open, so the probe can be posed without the jaws
        # standing on it - set_pose is silently refused if they are.
        self.settled_at(PARK, opening)

        # A DRY RUN OF THIS EXACT DESCENT, to find where the jaws end up.
        # Commanding the bottom pose directly and reading it there does NOT
        # give the same answer: these joints stall short of their setpoint by
        # an amount that depends on where they came from, so a reference taken
        # by dropping in from the park pose read 13.1 mm on the first trial
        # and 15.9 on the second while the descent itself lands at ~10.6.
        # Placing the probe against a reference the descent will not honour is
        # a rig defect that reads exactly like a grasp failure.
        hx, hy, _ = self.settled_at(GRASP_APPROACH_HEIGHT, opening)
        self.descend(steps, opening)
        jx, jy, jz = self.jaws()

        # Where the face is at its OUTERMOST over that descent, not where it
        # ends up: droop leaves it several mm further out at the top, so
        # clearing only the final position puts the probe inside the column
        # the face sweeps through.

        self.settled_at(PARK, opening)
        # The half-width the jaws must reach past, which for a rotated box is
        # not a constant: the aim point has to back off by the object's own
        # extent along the closing direction, whatever that is at this yaw.
        place(max(jx, hx) + probe.half_width(yaw) + offset, jy,
              probe.height, yaw)
        self.spin(SETTLE)

        self.settled_at(GRASP_APPROACH_HEIGHT, opening)
        start = probe_pose()

        self.descend(steps, opening)
        settled = probe_pose()

        self.at(0.0, GRIPPER_CLOSED)
        self.spin(SETTLE)
        self.at(GRASP_APPROACH_HEIGHT, GRIPPER_CLOSED)
        self.spin(SETTLE + 2.0)
        after = probe_pose()

        rise = after[2] - settled[2]
        return {
            "jaw_z": jz,
            "nudge": math.hypot(settled[0] - start[0], settled[1] - start[1]),
            "rise": rise,
            "moved": math.hypot(after[0] - settled[0], after[1] - settled[1]),
            "lifted": rise >= LIFTED,
        }


class Probe:
    """The thing being grasped. ONE per session, because respawning corrupts.

    A sweep may therefore vary only what `set_pose` and the joint commands can
    vary - position, yaw, the descent, the gripper. Size and shape are chosen
    per session on the command line.
    """

    def __init__(self, radius: float, height: float,
                 box: tuple | None = None, mu: float = 1.0) -> None:
        self.radius = radius
        self.height = height
        self.box = box
        self.mu = mu
        self.label = (f"{radius*2000:.0f}x{height*1000:.0f} cyl" if box is None
                      else f"{box[0]*1000:.0f}x{box[1]*1000:.0f}"
                           f"x{height*1000:.0f} box")

    def half_width(self, yaw: float) -> float:
        """Half the extent along the jaws' closing direction, at this yaw.

        A cylinder does not care. A box does, and that is the whole of the
        orientation question: the same wrapper is 25 mm across at one yaw and
        40 mm at another, and the jaws only open so far.
        """
        if self.box is None:
            return self.radius
        length, width = self.box
        return 0.5 * (abs(length * math.cos(yaw)) + abs(width * math.sin(yaw)))

    def spawn_at(self, x: float, y: float) -> None:
        spawn(self.radius, self.height, x, y, self.box, self.mu)


PROBES = {
    "can": Probe(0.033, 0.070),
    "bottle": Probe(0.020, 0.070),
    "tube": Probe(0.012, 0.070),
    "disc": Probe(0.025, 0.024),
    # test_field.sdf's litter_wrapper, actual size.
    "wrapper": Probe(0.025, 0.024, box=(0.080, 0.050)),
}


def sweep(name: str):
    """(label, steps, offset, opening, yaw) per trial."""
    if name == "steps":
        return [(f"{n} step{'s' if n > 1 else ''}", n, OFFSET, GRIPPER_OPEN,
                 0.0) for n in (1, 2, 3, 4, 6, 8)]
    if name == "offset":
        return [(f"{o*1000:+.0f} mm", STEPS, o, GRIPPER_OPEN, 0.0)
                for o in (-0.004, 0.0, 0.004, 0.008, 0.012, 0.016)]
    if name == "open":
        return [(f"open {g:.2f}", STEPS, OFFSET, g, 0.0)
                for g in (0.6, 0.8, 0.9, 1.0, 1.1)]
    if name == "yaw":
        return [(f"yaw {round(math.degrees(a)):d} deg", STEPS, OFFSET,
                 GRIPPER_OPEN, a)
                for a in (0.0, math.pi/8, math.pi/4, 3*math.pi/8, math.pi/2)]
    if name == "repeat":
        return [(f"run {i+1}", STEPS, OFFSET, GRIPPER_OPEN, 0.0)
                for i in range(6)]
    sys.exit(f"unknown sweep {name!r}; see the module docstring")


def main() -> int:
    trials = sweep(sys.argv[1] if len(sys.argv) > 1 else "steps")
    name = sys.argv[2] if len(sys.argv) > 2 else "disc"
    if name not in PROBES:
        sys.exit(f"unknown probe {name!r}; one of {sorted(PROBES)}")
    probe = PROBES[name]
    if len(sys.argv) > 3:
        probe.mu = float(sys.argv[3])

    urdf = subprocess.run(["xacro", URDF_XACRO, "use_sim:=true"],
                          check=True, capture_output=True, text=True).stdout
    arm = extract_planar_arm(
        load_chain(urdf, "arm_base_link", "arm_gripper_frame_link"))
    mount_z = float(forward_kinematics(
        load_chain(urdf, "base_footprint", "arm_base_link"), {})[2, 3])

    plane = -mount_z + GRASP_HEIGHT
    band = reachable_radius_band(arm, plane, APPROACH_ANGLE, JOINT_LIMITS,
                                 clearance_height=GRASP_APPROACH_HEIGHT)
    if band is None:
        sys.exit("no reachable radius at the grasp plane")
    grasp = arm.from_planar(0.5 * (band[0] + band[1]), plane, 0.0)

    rclpy.init()
    node = Node("grasp_lab")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    rig = Rig(node, arm, grasp)

    rig.at(PARK, GRIPPER_OPEN)
    rig.spin(8.0)
    probe.spawn_at(grasp[0] + 0.5, grasp[1])
    rig.spin(2.0)

    print(f"\nprobe {probe.label}, mu {probe.mu}", flush=True)
    print(f"\n{'trial':>14s} {'jaw z':>8s} {'nudge':>8s} {'rise':>8s} "
          f"{'moved':>8s}  verdict", flush=True)
    lifted = 0
    for label, steps, offset, opening, yaw in trials:
        r = rig.trial(probe, steps, offset, opening, yaw)
        verdict = ("LIFTED" if r["lifted"] else
                   "unreliable" if r["moved"] > 0.5 else
                   "shoved" if r["moved"] > 0.02 else "not touched")
        lifted += bool(r["lifted"])
        print(f"{label:>14s} {r['jaw_z']*1000:7.1f}m {r['nudge']*1000:7.1f}m "
              f"{r['rise']*1000:7.1f}m {r['moved']*1000:7.1f}m  {verdict}",
              flush=True)

    node.destroy_node()
    rclpy.shutdown()
    print(f"\n{lifted}/{len(trials)} lifted.")
    return 0 if lifted else 1


if __name__ == "__main__":
    sys.exit(main())
