#!/usr/bin/env python3
"""Is there an opening between the jaws, in the geometry physics actually uses?

Pure geometry - no simulator, no ROS:

    bash scripts/dev-native.sh python scripts/verify_gripper_clearance.py

WHY THIS EXISTS. A mission run reached PICKING twice, with range and bearing
both closed and the IK reporting the target reachable, and picked up nothing -
the cans were shoved across the field at a constant z = 0.035, never lifted.
Every part of the aim was verified; the thing that had never been checked is
whether the jaws have a gap for the can to be in.

`docs/STATUS.md` records that arm collision geometry is boxes rather than
meshes, so 0.5-2.6 MB STLs do not get mesh-mesh contacts every step. That is
the right call for the five joints that only have to not hit things. For the
gripper it is different in kind: a jaw's whole job is the space between it and
the other jaw, and a bounding box is exactly the shape that fills that space
in. Gazebo grips the boxes, not the meshes, so the boxes are what decides
whether a grasp is possible at all.

What this reports, in arm_gripper_link's frame:

  - where the grasp point (gripper_frame_link) sits relative to each jaw box
  - the largest upright cylinder that fits at the grasp point without
    intersecting either box, swept over the gripper's travel

The world's cans are radius 0.033. If the largest fitting cylinder is smaller
than that at every angle, no gripper gain, IK fix or approach change can pick
one up, and the numbers below say so before another 5-minute run does.
"""

import math
import subprocess
import sys
from pathlib import Path

import numpy as np
from urdf_parser_py.urdf import URDF

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src" / "drivebase_behaviour"))

from drivebase_behaviour.kinematics import _rpy_to_matrix  # noqa: E402

GRIPPER_LINK = "arm_gripper_link"
MOVING_JAW_LINK = "arm_moving_jaw_so101_v1_link"
GRASP_FRAME_LINK = "arm_gripper_frame_link"
GRIPPER_JOINT = "arm_gripper"

# The can the world spawns, and the height the coordinator grasps at.
CAN_RADIUS = 0.033
CAN_HALF_HEIGHT = 0.035

ANGLES = [0.0, 0.4, 0.8, 1.2, 1.6, 1.74533]


def load_urdf() -> URDF:
    xacro = subprocess.run(
        ["xacro", str(REPO / "src" / "drivebase_description" / "urdf"
                      / "drivebase.urdf.xacro"), "use_sim:=true",
         "use_arm:=true"],
        capture_output=True, text=True, check=True,
    )
    return URDF.from_xml_string(xacro.stdout)


def boxes_of(link) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """Every box collision on a link, as (centre, rotation, half-extents)."""
    out = []
    for collision in link.collisions:
        geometry = collision.geometry
        if not hasattr(geometry, "size"):
            continue
        origin = collision.origin
        xyz = np.array(origin.xyz if origin else [0.0, 0.0, 0.0])
        rpy = origin.rpy if origin else [0.0, 0.0, 0.0]
        out.append((xyz, _rpy_to_matrix(*rpy),
                    np.array(geometry.size) / 2.0))
    return out


def inside(point: np.ndarray, box) -> bool:
    centre, rotation, half = box
    local = rotation.T @ (point - centre)
    return bool(np.all(np.abs(local) <= half))


def fits(radius: float, grasp: np.ndarray, axis: np.ndarray,
         boxes, samples: int = 72) -> bool:
    """Does an upright cylinder at the grasp point clear every box?"""
    # Two perpendiculars to the can's axis, to sweep its surface.
    helper = np.array([1.0, 0.0, 0.0])
    if abs(axis @ helper) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)

    for k in range(samples):
        theta = 2.0 * math.pi * k / samples
        radial = radius * (math.cos(theta) * u + math.sin(theta) * v)
        for h in np.linspace(-CAN_HALF_HEIGHT, CAN_HALF_HEIGHT, 9):
            point = grasp + radial + h * axis
            if any(inside(point, b) for b in boxes):
                return False
    return True


def largest_fitting(grasp, axis, boxes) -> float:
    if not fits(1e-4, grasp, axis, boxes):
        return 0.0
    low, high = 1e-4, 0.10
    for _ in range(40):
        mid = 0.5 * (low + high)
        if fits(mid, grasp, axis, boxes):
            low = mid
        else:
            high = mid
    return low


def main() -> int:
    robot = load_urdf()
    links = {link.name: link for link in robot.links}
    joints = {joint.name: joint for joint in robot.joints}

    fixed_boxes = boxes_of(links[GRIPPER_LINK])

    grasp_joint = joints["arm_gripper_frame_joint"]
    grasp = np.array(grasp_joint.origin.xyz)
    # The can stands upright and the descent is vertical, so its axis is the
    # gripper's approach direction - gripper_link's z, whichever way it points.
    axis = np.array([0.0, 0.0, 1.0])

    jaw = joints[GRIPPER_JOINT]
    jaw_origin = np.array(jaw.origin.xyz)
    jaw_rotation = _rpy_to_matrix(*jaw.origin.rpy)
    jaw_axis = np.array(jaw.axis)
    moving_boxes = boxes_of(links[MOVING_JAW_LINK])

    print(f"grasp point (gripper_frame_link) in {GRIPPER_LINK}: "
          f"{grasp[0]:+.4f} {grasp[1]:+.4f} {grasp[2]:+.4f}")
    print(f"\nfixed jaw boxes on {GRIPPER_LINK}:")
    for centre, _, half in fixed_boxes:
        print(f"  centre {centre[0]:+.4f} {centre[1]:+.4f} {centre[2]:+.4f}"
              f"   size {2*half[0]:.4f} x {2*half[1]:.4f} x {2*half[2]:.4f}")
    swallowed = [i for i, b in enumerate(fixed_boxes) if inside(grasp, b)]
    if swallowed:
        print(f"  *** the grasp point is INSIDE fixed box {swallowed} ***")

    print(f"\n{'gripper':>8s} {'largest cylinder':>17s} {'vs can 0.033':>13s}")
    for angle in ANGLES:
        turn = _axis_rotation(jaw_axis, angle)
        boxes = list(fixed_boxes)
        for centre, rotation, half in moving_boxes:
            world_rotation = jaw_rotation @ turn @ rotation
            world_centre = jaw_origin + jaw_rotation @ turn @ centre
            boxes.append((world_centre, world_rotation, half))
        radius = largest_fitting(grasp, axis, boxes)
        verdict = "fits" if radius >= CAN_RADIUS else "TOO SMALL"
        print(f"{angle:8.3f} {radius:17.4f} {verdict:>13s}")

    print("\nThe can is radius 0.033. A row reading TOO SMALL at every angle "
          "means the sim's gripper has no opening to put it in, whatever the "
          "aim does.")
    return 0


def _axis_rotation(axis, angle: float) -> np.ndarray:
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [c + x*x*(1-c), x*y*(1-c) - z*s, x*z*(1-c) + y*s],
        [y*x*(1-c) + z*s, c + y*y*(1-c), y*z*(1-c) - x*s],
        [z*x*(1-c) - y*s, z*y*(1-c) + x*s, c + z*z*(1-c)],
    ])


if __name__ == "__main__":
    sys.exit(main())
