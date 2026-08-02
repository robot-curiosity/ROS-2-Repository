#!/usr/bin/env python3
"""Commanded vs achieved gripper angle, swept, with the rest of the arm still.

    bash scripts/dev.sh bash -c \
      'ros2 launch drivebase_sim sim.launch.py headless:=true arm:=true & \
       sleep 30; python3 /ws/scripts/measure_gripper_sweep.py [pose] [--step]'

    pose: confirm (default) | stow | search
    --step: hold one command for a long time and trace the approach, instead
            of sweeping. Answers "how long would it take" rather than "where
            does it stop".

WHY THIS EXISTS. PROGRESS.md records that the gripper never opens as
commanded - 1.200 asked for, 0.937-1.088 reached, across every pose, against a
URDF limit of 1.74533 that is nowhere near. Three causes are consistent with a
single-point reading and they want different fixes:

  fixed offset  -> the joint is being held short by something constant
  fixed ratio   -> proportional steady-state error, i.e. the p_gain 4.0 that
                   arm_joint_hold applies uniformly to all six joints, sized
                   for a shoulder holding the arm against gravity rather than
                   for the smallest, lightest joint on it
  neither, and
  path-dependent-> contact. The jaws are closing on something, and the
                   controller holds short and reports nothing.

One command cannot tell them apart; a sweep can. The sweep runs up and then
back down over the same values, because contact is the one cause that makes the
achieved angle depend on which way it was approached.

The arm holds one pose throughout so nothing else is moving while the gripper
is read - a mission run steps the pose underneath the reading.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# [pan, lift, elbow, wrist_flex, wrist_roll] - gripper is swept, not held.
POSES = {
    "confirm": [0.0, 0.9, 0.46, -1.40, 0.0],
    "stow": [0.0, -1.20, 1.55, 1.10, 0.0],
    "search": [0.0, -1.4, 0.0, 0.4, 0.0],
}
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
              "wrist_roll"]

# Spans the mission's 0.0 closed to its 1.2 open and out to 1.6, which is still
# inside the 1.74533 limit. A limit clamp shows up as the top of the sweep
# flattening; a gain shows up as every step falling short in proportion.
SWEEP = [0.0, 0.4, 0.8, 1.2, 1.6, 1.2, 0.8, 0.4, 0.0]
SETTLE_SECONDS = 6.0
CREEP_WINDOW = 1.5


class GripperSweep(Node):
    def __init__(self, pose: list[float]) -> None:
        super().__init__("gripper_sweep")
        self.pose = pose
        self.command = SWEEP[0]
        self.publishers_by_joint = {
            j: self.create_publisher(Float64, f"/arm/{j}/position", 10)
            for j in ARM_JOINTS + ["gripper"]
        }
        self.latest: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        # Re-sent every tick: a volatile publisher with no subscriber yet drops
        # silently, and discovery is not instant. See measure_arm_droop.py.
        self.create_timer(0.2, self._publish)

    def _on_state(self, message: JointState) -> None:
        self.latest = message

    def _publish(self) -> None:
        for joint, value in zip(ARM_JOINTS, self.pose):
            message = Float64()
            message.data = float(value)
            self.publishers_by_joint[joint].publish(message)
        message = Float64()
        message.data = float(self.command)
        self.publishers_by_joint["gripper"].publish(message)

    def spin_for(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def gripper_angle(self) -> float | None:
        state = self.latest
        if state is None or "arm_gripper" not in state.name:
            return None
        return state.position[state.name.index("arm_gripper")]


STEP_COMMAND = 1.2
STEP_SECONDS = 45.0


def run_step(node: GripperSweep) -> None:
    """Trace one command to wherever it ends up, sampling as it goes."""
    node.command = STEP_COMMAND
    start = time.time()
    print(f"{'t':>6s} {'achieved':>10s} {'remaining':>10s}")
    samples = []
    while time.time() - start < STEP_SECONDS and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = time.time() - start
        if samples and elapsed - samples[-1][0] < 1.0:
            continue
        achieved = node.gripper_angle()
        samples.append((elapsed, achieved))
        print(f"{elapsed:6.1f} {achieved:+10.4f} "
              f"{STEP_COMMAND - achieved:+10.4f}")

    final = samples[-1][1]
    # The mission holds each grasp step for grasp_hold_seconds = 1.5.
    at_hold = next(a for t, a in samples if t >= 1.5)
    print(f"\nat 1.5 s (one grasp_hold_seconds): {at_hold:+.4f}")
    print(f"at {STEP_SECONDS:.0f} s: {final:+.4f}, "
          f"still {STEP_COMMAND - final:+.4f} from {STEP_COMMAND}")


def main() -> int:
    step_mode = "--step" in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    name = args[0] if args else "confirm"
    if name not in POSES:
        print(f"unknown pose {name!r}; choose from {', '.join(POSES)}")
        return 2

    rclpy.init()
    node = GripperSweep(POSES[name])
    node.spin_for(SETTLE_SECONDS)
    if node.gripper_angle() is None:
        print("no /joint_states carrying arm_gripper received")
        return 1

    print(f"pose: {name} {POSES[name]}")
    if step_mode:
        run_step(node)
        node.destroy_node()
        rclpy.shutdown()
        return 0

    print(f"{'commanded':>10s} {'achieved':>10s} {'error':>9s} "
          f"{'ratio':>7s} {'creep':>8s}")

    rows = []
    for command in SWEEP:
        node.command = command
        node.spin_for(SETTLE_SECONDS - CREEP_WINDOW)
        early = node.gripper_angle()
        node.spin_for(CREEP_WINDOW)
        achieved = node.gripper_angle()
        creep = achieved - early
        ratio = achieved / command if command else float("nan")
        rows.append((command, achieved))
        print(f"{command:10.3f} {achieved:+10.3f} {achieved - command:+9.3f} "
              f"{ratio:7.3f} {creep:+8.4f}")

    # Ascending and descending halves share their endpoints; if they disagree
    # at the same command, the joint is being stopped by something it ran into.
    up = dict(rows[:SWEEP.index(max(SWEEP)) + 1])
    down = dict(rows[SWEEP.index(max(SWEEP)):])
    hysteresis = max(abs(up[c] - down[c]) for c in up if c in down)

    moving = [(c, a) for c, a in rows if c > 0.0]
    offsets = [a - c for c, a in moving]
    ratios = [a / c for c, a in moving]
    offset_spread = max(offsets) - min(offsets)
    ratio_spread = max(ratios) - min(ratios)

    print(f"\nhysteresis (up vs down, same command): {hysteresis:.4f} rad")
    print(f"offset  {sum(offsets) / len(offsets):+.4f} rad, "
          f"spread {offset_spread:.4f}")
    print(f"ratio   {sum(ratios) / len(ratios):.4f}, "
          f"spread {ratio_spread:.4f}")
    print("\nA constant offset means the offset spread is the small one; a gain "
          "problem means the ratio spread is. Hysteresis above either means "
          "contact, and neither fit is the story.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
