#!/usr/bin/env python3
"""Does the gripper actually lift a can? Nothing else in this repo asks.

Needs the simulator, so it is one dev.sh invocation:

    bash scripts/dev.sh bash -c 'ros2 launch drivebase_sim sim.launch.py \
        headless:=true litter_detector:=false > /tmp/sim.log 2>&1 & \
        sleep 28; python3 /ws/scripts/verify_pickup.py'

WHY THIS EXISTS. Three mission runs reached PICKING with the range closed, the
bearing closed and the IK reporting the target reachable, and picked up
nothing. Every part of the aim had a test; the grasp itself had none, because
the only way to test it is to put a can somewhere the arm can reach and watch
what the can does. So: place one, run the sequence the coordinator runs, and
read the can's height back out of the simulator.

It does not drive the base and it does not use the detector. Those are tested
elsewhere and they were never the thing that was broken.
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

WORLD = "test_field"

# (radius, height) in metres. The test field's cans are 0.033 x 0.070; the
# smaller radii are here because the jaws that have to hold them are 12 mm
# across at the tip and a cylinder much wider than that rolls out of them.
# The 0.024 heights are the wrapper's - the case the 224 mm mount exists for,
# and the one that was under the jaws rather than between them at 260.
PROBES = [(0.033, 0.070), (0.025, 0.070), (0.020, 0.070), (0.015, 0.070),
          (0.010, 0.070), (0.025, 0.024), (0.015, 0.024)]

# One radius per simulator session, because probes interact. Parking a probe
# the jaws are still touching is silently refused by the solver - it stays put
# - and the next probe is then placed inside it and both are flung metres.
# Reproducible, and it corrupted exactly the sizes worth measuring.
OBJECT_MASS = 0.015

URDF_XACRO = "/ws/src/drivebase_description/urdf/drivebase.urdf.xacro"
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
          "wrist_roll", "gripper"]

# coordinator.yaml. Kept here rather than parsed so this script states what it
# assumed and fails loudly if the file drifts away from it.
GRASP_HEIGHT = 0.020
GRASP_APPROACH_HEIGHT = 0.060
GRIPPER_OPEN = 1.0
GRIPPER_CLOSED = JOINT_LIMITS["gripper"][0]
APPROACH_ANGLE = -math.pi / 2.0

# Gap between the fixed jaw's face and the object at placement. Not zero,
# because droop carries the tip 8.3 mm outward over the 60 mm descent, so the
# "vertical" approach rakes anything touching that face. 0.012 was tried too
# and changed nothing: with more clearance the jaws simply close short.
GRASP_CLEARANCE = 0.004
SETTLE = 6.0
# Waypoints down the 60 mm pre-grasp column, 10 mm apart.
DESCENT_STEPS = 6
LIFTED = 0.020          # metres above its resting height to count as picked


def gz(args: list[str]) -> str:
    return subprocess.run(["gz"] + args, capture_output=True, text=True,
                          timeout=20).stdout


def can_pose(name: str) -> tuple[float, float, float]:
    """Position of the object, from the simulator itself."""
    text = gz(["model", "-m", name, "-p"])
    numbers = []
    for line in text.splitlines():
        if line.strip().startswith("["):
            numbers = [float(v) for v in line.strip().strip("[]").split()]
            break
    if len(numbers) < 3:
        sys.exit(f"could not read {name} pose from gz:\n{text}")
    return tuple(numbers[:3])


def spawn(name: str, radius: float, height: float, x: float, y: float) -> None:
    """An upright cylinder of the given size, standing on the ground.

    Spawned rather than moved: the world's own cans are all 33 x 70 mm, and
    the point of this sweep is the sizes it does not have.
    """
    sdf = (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}">'
           f'<pose>{x} {y} {height/2} 0 0 0</pose><link name="link">'
           f'<inertial><mass>{OBJECT_MASS}</mass><inertia>'
           f'<ixx>7e-6</ixx><iyy>7e-6</iyy><izz>4e-6</izz>'
           f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
           f'<collision name="c"><geometry><cylinder>'
           f'<radius>{radius}</radius><length>{height}</length>'
           f'</cylinder></geometry></collision>'
           f'<visual name="v"><geometry><cylinder>'
           f'<radius>{radius}</radius><length>{height}</length>'
           f'</cylinder></geometry></visual>'
           f'</link></model></sdf>')
    gz(["service", "-s", f"/world/{WORLD}/create",
        "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
        "--timeout", "5000", "--req", f'sdf: \'{sdf}\''])


def main(probes) -> int:
    urdf = subprocess.run(["xacro", URDF_XACRO, "use_sim:=true"],
                          check=True, capture_output=True, text=True).stdout
    arm = extract_planar_arm(
        load_chain(urdf, "arm_base_link", "arm_gripper_frame_link"))
    mount_z = float(forward_kinematics(
        load_chain(urdf, "base_footprint", "arm_base_link"), {})[2, 3])

    grasp_plane = -mount_z + GRASP_HEIGHT
    band = reachable_radius_band(arm, grasp_plane, APPROACH_ANGLE,
                                 JOINT_LIMITS,
                                 clearance_height=GRASP_APPROACH_HEIGHT)
    if band is None:
        sys.exit("no reachable radius at the grasp plane")
    radius = 0.5 * (band[0] + band[1])

    grasp = arm.from_planar(radius, grasp_plane, 0.0)
    above = (grasp[0], grasp[1], grasp[2] + GRASP_APPROACH_HEIGHT)
    at_grasp = solve(arm, grasp, APPROACH_ANGLE, JOINT_LIMITS)
    at_above = solve(arm, above, APPROACH_ANGLE, JOINT_LIMITS)
    if at_grasp is None or at_above is None:
        sys.exit("the band reported a radius the IK will not solve")

    rclpy.init()
    node = Node("verify_pickup")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    publishers = {j: node.create_publisher(Float64, f"/arm/{j}/position", 10)
                  for j in JOINTS}
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)

    held: dict[str, float] = {}

    def spin(seconds: float) -> None:
        """Spin, re-publishing the held command as we go.

        Publishing once loses the first command outright: the gz bridge has
        not subscribed yet when a node that just started fires, and a
        volatile publisher discards into nothing. Every arm reading in this
        repo taken with a single publish is suspect for the same reason.
        """
        end = node.get_clock().now().nanoseconds + seconds * 1e9
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            for joint, value in held.items():
                publishers[joint].publish(Float64(data=value))
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.04)

    def command(solution, gripper: float) -> None:
        held.update({
            "shoulder_pan": solution.shoulder_pan,
            "shoulder_lift": solution.shoulder_lift,
            "elbow_flex": solution.elbow_flex,
            "wrist_flex": solution.wrist_flex,
            "wrist_roll": 0.0,
            "gripper": gripper,
        })

    def descend(gripper: float) -> None:
        """Step the tip down the column instead of commanding the bottom.

        Six P-controlled joints handed a single new setpoint do NOT move the
        tip along the line between the two poses - each converges at its own
        rate, so the tip bulges out of the column on the way. Measured: a
        one-shot descent knocked a 50 mm disc 33 mm sideways and a 66 mm can
        19 mm outward, both before the jaws closed at all.
        """
        for step in range(1, DESCENT_STEPS + 1):
            height = GRASP_APPROACH_HEIGHT * (1.0 - step / DESCENT_STEPS)
            at = solve(arm, (grasp[0], grasp[1], grasp[2] + height),
                       APPROACH_ANGLE, JOINT_LIMITS)
            if at is None:
                sys.exit(f"no IK {height*1000:.0f} mm up the descent column")
            command(at, gripper)
            spin(SETTLE / DESCENT_STEPS + 0.5)

    spin(8.0)

    # Open, and go to the pre-grasp height. Where the jaws end up is read from
    # tf rather than predicted, because the arm droops and the can has to be
    # placed against where it actually goes.
    command(at_grasp, GRIPPER_OPEN)
    spin(SETTLE)
    try:
        jaws = buffer.lookup_transform("base_footprint",
                                       "arm_gripper_frame_link",
                                       rclpy.time.Time())
    except tf2_ros.TransformException as error:
        sys.exit(f"no tf to the jaws: {error}")
    jx = jaws.transform.translation.x
    jy = jaws.transform.translation.y
    jz = jaws.transform.translation.z

    print(f"jaws at x {jx:+.4f} y {jy:+.4f} z {jz:+.4f} "
          f"(commanded {GRASP_HEIGHT:.3f})")

    # Lift clear BEFORE placing the can, so the descent is the only movement
    # anywhere near it. Placing it first and then travelling knocks it over,
    # and a can that has already been shoved cannot show whether the grasp
    # works.
    command(at_above, GRIPPER_OPEN)
    spin(SETTLE)
    high = buffer.lookup_transform("base_footprint", "arm_gripper_frame_link",
                                   rclpy.time.Time())
    hx = high.transform.translation.x
    print(f"pre-grasp jaws at x {high.transform.translation.x:+.4f} "
          f"z {high.transform.translation.z:+.4f} "
          f"(x differs by {(high.transform.translation.x - jx)*1000:+.1f} mm)")

    print(f"\n{'object':>13s} {'descent nudge':>14s} {'rise':>9s} "
          f"{'shove':>9s}  verdict")
    results = []
    for index, (radius, height) in enumerate(probes):
        name = f"pickup_probe_{index}"
        # ONE RADIUS BEYOND THE FRAME, plus GRASP_CLEARANCE - and beyond the
        # OUTERMOST point of the descent, not the endpoint. Droop leaves the
        # face several mm further out at pre-grasp than at the grasp, so
        # clearing only the final position puts the object inside the column
        # the face sweeps and the drop rakes it away before anything closes.
        spawn(name, radius, height,
              max(jx, hx) + radius + GRASP_CLEARANCE, jy)
        command(at_above, GRIPPER_OPEN)
        spin(SETTLE)
        placed = can_pose(name)

        descend(GRIPPER_OPEN)
        settled = can_pose(name)
        nudge = math.hypot(settled[0] - placed[0], settled[1] - placed[1])

        command(at_grasp, GRIPPER_CLOSED)
        spin(SETTLE)
        command(at_above, GRIPPER_CLOSED)
        spin(SETTLE + 2.0)
        after = can_pose(name)

        rise = after[2] - settled[2]
        moved = math.hypot(after[0] - settled[0], after[1] - settled[1])
        lifted_it = rise >= LIFTED
        # Half a metre is not a grasp outcome, it is the probe having been
        # spawned into something. Reported as unreliable rather than shoved.
        verdict = ("LIFTED" if lifted_it else
                   "unreliable" if moved > 0.5 else
                   "shoved" if moved > 0.02 else "not touched")
        print(f"{radius*2000:5.0f}x{height*1000:<7.0f} "
              f"{nudge*1000:13.1f}m {rise*1000:8.1f}m "
              f"{moved*1000:8.1f}m  {verdict}")
        # Which way it went, in the jaws' own frame: a push straight out
        # along +x is the face raking it, a sideways one is the moving jaw.
        print(f"          placed {(placed[0]-jx)*1000:+.1f},"
              f"{(placed[1]-jy)*1000:+.1f} -> settled "
              f"{(settled[0]-jx)*1000:+.1f},{(settled[1]-jy)*1000:+.1f}"
              f" -> after {(after[0]-jx)*1000:+.1f},"
              f"{(after[1]-jy)*1000:+.1f} mm from the jaws")
        results.append((radius, height, lifted_it))

        command(at_above, GRIPPER_OPEN)
        spin(SETTLE)

    node.destroy_node()
    rclpy.shutdown()

    lifted = [(r, h) for r, h, ok in results if ok]
    if not lifted:
        print("\nFAIL: nothing was lifted at any size.")
        return 1
    print("\nOK, lifted: " + ", ".join(
        f"{r*2000:.0f}x{h*1000:.0f} mm" for r, h in lifted))
    return 0


if __name__ == "__main__":
    chosen = [tuple(float(v) for v in a.split(","))
              for a in sys.argv[1:]] or PROBES
    sys.exit(main(chosen))
