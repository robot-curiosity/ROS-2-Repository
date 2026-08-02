#!/usr/bin/env python3
"""Reports the radius span the arm can actually grasp at, off the URDF.

Pure geometry - no simulator, no ROS:

    bash scripts/dev-native.sh python scripts/verify_grasp_band.py

WHAT IT IS FOR. The coordinator's range loop creeps the base until the target
sits in the middle of this span, and the confirmation gate refuses anything
outside it. Both read the span from `reachable_radius_band` at runtime, so this
script does not feed them a number - it shows what that number currently is,
and fails loudly if the span has collapsed or vanished.

Run it after touching grasp_height, grasp_approach_height, approach_angle, the
arm's mount, or anything in so101_macro.xacro. A band that has gone narrow is
the failure mode worth catching: docs/arm_workspace.md found the window is a
cliff rather than a gradient, 19 mm wide at the arm's lowest reachable height
and 123 mm only 15 mm above it.
"""

import math
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src" / "drivebase_behaviour"))

from drivebase_behaviour.ik import (  # noqa: E402
    JOINT_LIMITS,
    reachable_radius_band,
)
from drivebase_behaviour.kinematics import (  # noqa: E402
    extract_planar_arm,
    forward_kinematics,
    load_chain,
)

# Defaults from config/coordinator.yaml. Passed explicitly rather than parsed
# so this script says what it assumed.
GRASP_HEIGHT = 0.050
GRASP_APPROACH_HEIGHT = 0.060
APPROACH_ANGLE = -math.pi / 2.0
# Narrower than this and the creep deadband has nowhere to sit.
MIN_USEFUL_WIDTH = 0.050


def main() -> int:
    urdf = subprocess.run(
        ["xacro", str(REPO / "src/drivebase_description/urdf/drivebase.urdf.xacro"),
         "use_sim:=true"],
        check=True, capture_output=True, text=True,
    ).stdout

    arm = extract_planar_arm(
        load_chain(urdf, "arm_base_link", "arm_gripper_frame_link"))

    # The same datum the coordinator uses: the ground, found by measuring down
    # from the arm's mount rather than by trusting the ToF, which at the confirm
    # pose lands beside the litter and reads the ground anyway.
    mount = forward_kinematics(
        load_chain(urdf, "base_footprint", "arm_base_link"), {})
    mount_z = float(mount[2, 3])
    grasp_plane = -mount_z + GRASP_HEIGHT

    print(f"arm mount height      {mount_z:.4f} m above base_footprint")
    print(f"grasp plane           {grasp_plane:.4f} m in arm_base_link "
          f"({GRASP_HEIGHT:.3f} m above ground)")
    print(f"approach angle        {APPROACH_ANGLE:.4f} rad")
    print(f"vertical pre-grasp    {GRASP_APPROACH_HEIGHT:.3f} m")
    print()

    band = reachable_radius_band(
        arm, grasp_plane, APPROACH_ANGLE, JOINT_LIMITS,
        clearance_height=GRASP_APPROACH_HEIGHT,
    )
    if band is None:
        print("FAIL: no radius solves both the grasp and the descent onto it.")
        return 1

    low, high = band
    width = high - low
    print(f"graspable radius band {low:.4f} .. {high:.4f} m")
    print(f"width                 {width * 1000:.1f} mm")
    print(f"creep setpoint        {0.5 * (low + high):.4f} m")

    # Descent-only, to show what the vertical approach costs. A band that is
    # much wider here means the pre-grasp height, not the reach, is binding.
    flat = reachable_radius_band(
        arm, grasp_plane, APPROACH_ANGLE, JOINT_LIMITS, clearance_height=0.0)
    if flat is not None:
        print(f"  ({(flat[1] - flat[0]) * 1000:.1f} mm of it without "
              f"requiring the vertical descent)")

    if width < MIN_USEFUL_WIDTH:
        print(f"\nFAIL: {width * 1000:.1f} mm is too narrow to place the base "
              f"in; expected at least {MIN_USEFUL_WIDTH * 1000:.0f} mm.")
        return 1

    print("\nOK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
