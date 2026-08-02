#!/usr/bin/env python3
"""Is there an opening between the jaws, in the geometry physics actually uses?

Pure geometry - no simulator, no ROS:

    bash scripts/dev-native.sh python scripts/verify_gripper_clearance.py

WHY THIS EXISTS. A mission run reached PICKING twice, with range and bearing
both closed and the IK reporting the target reachable, and picked up nothing -
the cans were shoved across the field at a constant z = 0.035, never lifted.
Every part of the aim was verified; the thing that had never been checked is
whether the jaws have a gap for the can to be in.

The cause was the collision geometry. `docs/STATUS.md` records that the arm's
collisions are boxes rather than meshes, so 0.5-2.6 MB STLs do not get
mesh-mesh contacts every step - the right call for five joints that only have
to not hit things, and wrong in kind for the gripper. A jaw's whole job is the
space beside it, and a bounding box is exactly the shape that fills that space
in: the fixed jaw's box was 65 x 52 x 105 mm and contained the grasp point, so
the simulated gripper was a solid brick. `vendor_so101.py` now keeps mesh
collisions for the two jaws (`KEEP_MESH_COLLISION`), which measured 0.979
real-time factor against 0.977 for the boxes.

Gazebo grips the collision geometry, not the visual meshes, so this script
runs on whatever the URDF currently declares and reports, in
arm_gripper_link's frame:

  - where the grasp point (gripper_frame_link) sits relative to each jaw
  - the largest upright can that fits standing on the ground with the grasp
    point 50 mm up its side, swept over the gripper's travel
  - the nearest spot that is actually graspable - free with the jaws open and
    trapped once they close, which free space alone does not imply

Run it after touching the jaw geometry, grasp_height, or the gripper limits.
It exits non-zero when the can does not fit where the arm aims, which is the
state that produces a converging, confident, empty pick.
"""

import math
import struct
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

# The can test_field.sdf spawns, and coordinator.yaml's grasp_height. The can
# stands on the ground and the grasp point is 50 mm up its 70 mm side, so it
# reaches 50 mm below that point and 20 mm above it - it is NOT centred there,
# and a cylinder centred on the grasp point would model a can floating 15 mm
# off the ground.
CAN_RADIUS = 0.033
CAN_HEIGHT = 0.070
GRASP_HEIGHT = 0.050

ANGLES = [0.0, 0.4, 0.8, 1.2, 1.6, 1.74533]

RAY_DIRECTIONS = [
    np.array(d) / np.linalg.norm(d)
    for d in ([0.5773, 0.5774, 0.5772], [-0.7071, 0.4082, 0.5774],
              [0.3015, -0.9045, 0.3015])
]


def load_urdf() -> URDF:
    xacro = subprocess.run(
        ["xacro", str(REPO / "src" / "drivebase_description" / "urdf"
                      / "drivebase.urdf.xacro"), "use_sim:=true",
         "use_arm:=true"],
        capture_output=True, text=True, check=True,
    )
    return URDF.from_xml_string(xacro.stdout)


def load_stl(path: Path) -> np.ndarray:
    """Triangles as an (n, 3, 3) array, binary or ASCII."""
    raw = path.read_bytes()
    if raw[:5].lower() == b"solid" and b"facet" in raw[:2048]:
        vertices = [
            [float(p) for p in line.split()[1:]]
            for line in raw.decode("utf-8", "replace").splitlines()
            if line.split()[:1] == ["vertex"]
        ]
        return np.array(vertices).reshape(-1, 3, 3)

    count = struct.unpack("<I", raw[80:84])[0]
    data = np.frombuffer(raw, dtype=np.uint8, count=count * 50, offset=84)
    floats = data.reshape(count, 50)[:, :48].copy().view("<f4")
    return floats[:, 3:12].reshape(count, 3, 3).astype(float)


class Solid:
    """A collision shape placed in some frame, that can be asked what it holds."""

    def __init__(self, rotation: np.ndarray, offset: np.ndarray) -> None:
        self.rotation = rotation
        self.offset = offset

    def placed(self, rotation: np.ndarray, offset: np.ndarray) -> "Solid":
        clone = self.__class__.__new__(self.__class__)
        clone.__dict__.update(self.__dict__)
        clone.rotation = rotation @ self.rotation
        clone.offset = offset + rotation @ self.offset
        clone._placed_points = None
        return clone

    def contains(self, point: np.ndarray) -> bool:
        return self.local_contains(self.rotation.T @ (point - self.offset))

    def points(self) -> np.ndarray:
        """Points of this solid, in the frame it is placed in.

        Used the other way round from `contains`: a can is solid too, so it
        does not fit merely because its wall misses the jaw - the jaw must
        also not be standing inside it.
        """
        if getattr(self, "_placed_points", None) is None:
            self._placed_points = (
                (self.rotation @ self.local_points().T).T + self.offset)
        return self._placed_points


class Box(Solid):
    def __init__(self, rotation, offset, half) -> None:
        super().__init__(rotation, offset)
        self.half = half
        self.description = f"box {2*half[0]:.4f} x {2*half[1]:.4f} x {2*half[2]:.4f}"

    def local_contains(self, local: np.ndarray) -> bool:
        return bool(np.all(np.abs(local) <= self.half))

    def local_points(self) -> np.ndarray:
        # The eight corners: a box has no detail between them.
        signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1)
                          for k in (-1, 1)])
        return signs * self.half


class Mesh(Solid):
    def __init__(self, rotation, offset, triangles, name) -> None:
        super().__init__(rotation, offset)
        self.triangles = triangles
        self.vertices = np.unique(triangles.reshape(-1, 3), axis=0)
        self.low = triangles.reshape(-1, 3).min(axis=0)
        self.high = triangles.reshape(-1, 3).max(axis=0)
        self.description = f"mesh {name} ({len(triangles)} triangles)"

    def local_points(self) -> np.ndarray:
        return self.vertices

    def local_contains(self, local: np.ndarray) -> bool:
        # A bounding-box reject first: nearly every sample point is nowhere
        # near the jaw, and the ray cast below is the expensive part.
        if np.any(local < self.low) or np.any(local > self.high):
            return False
        # Odd crossings of an outbound ray means inside, for a closed surface.
        # Three deliberately skew directions, majority vote: these STLs are
        # full of axis-aligned facets, and an axis-aligned ray grazes their
        # shared edges and double-counts. A +x ray reported the empty channel
        # between the jaws as solid for exactly that reason.
        votes = sum(self._crossings_odd(local, d) for d in RAY_DIRECTIONS)
        return votes >= 2

    def _crossings_odd(self, local: np.ndarray,
                       direction: np.ndarray) -> bool:
        # Moller-Trumbore, vectorised over every triangle at once.
        v0, v1, v2 = (self.triangles[:, i, :] for i in range(3))
        edge1, edge2 = v1 - v0, v2 - v0
        h = np.cross(direction, edge2)
        a = np.einsum("ij,ij->i", edge1, h)
        parallel = np.abs(a) < 1e-12
        f = np.where(parallel, 0.0, 1.0 / np.where(parallel, 1.0, a))
        s = local - v0
        u = f * np.einsum("ij,ij->i", s, h)
        q = np.cross(s, edge1)
        v = f * (q @ direction)
        t = f * np.einsum("ij,ij->i", edge2, q)
        hit = ~parallel & (u >= 0.0) & (u <= 1.0) & (v >= 0.0) \
            & (u + v <= 1.0) & (t > 1e-9)
        return bool(np.count_nonzero(hit) % 2 == 1)


def solids_of(link, meshes_dir: Path) -> list[Solid]:
    """Every collision on a link, whatever geometry it is expressed in."""
    out: list[Solid] = []
    for collision in link.collisions:
        geometry = collision.geometry
        origin = collision.origin
        offset = np.array(origin.xyz if origin else [0.0, 0.0, 0.0])
        rotation = _rpy_to_matrix(*(origin.rpy if origin else [0.0, 0.0, 0.0]))
        if hasattr(geometry, "size"):
            out.append(Box(rotation, offset, np.array(geometry.size) / 2.0))
        elif hasattr(geometry, "filename"):
            name = Path(geometry.filename).name
            out.append(Mesh(rotation, offset,
                            load_stl(meshes_dir / name), name))
        else:
            sys.exit(f"unhandled collision geometry on {link.name}")
    return out


def fits(radius: float, grasp: np.ndarray, axis: np.ndarray,
         solids, samples: int = 72, height: float = CAN_HEIGHT,
         grasp_height: float = GRASP_HEIGHT) -> bool:
    """Does an upright object standing on the ground clear every solid?

    `grasp` is the point the arm drives to; the object stands on the ground
    with that point `grasp_height` up its side, so it reaches from
    -grasp_height to height-grasp_height along the axis. A short object is
    entirely below the grasp point and clears everything trivially - which is
    not the same as being grippable, see `graspable`.

    Both directions matter, and they catch different failures. A jaw standing
    inside the object is caught by the jaw's own points; a jaw face the
    object's wall passes through is caught by sampling the wall. The first is
    nearly free and rejects almost everything, so it goes first.
    """
    low, high = -grasp_height, height - grasp_height
    for solid in solids:
        points = solid.points() - grasp
        along = points @ axis
        radial = np.linalg.norm(points - np.outer(along, axis), axis=1)
        if np.any((along >= low) & (along <= high) & (radial <= radius)):
            return False

    # Two perpendiculars to the can's axis, to sweep its wall.
    helper = np.array([1.0, 0.0, 0.0])
    if abs(axis @ helper) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)

    for k in range(samples):
        theta = 2.0 * math.pi * k / samples
        radial_offset = radius * (math.cos(theta) * u + math.sin(theta) * v)
        for h in np.linspace(low, high, 15) if high > low else [low]:
            point = grasp + radial_offset + h * axis
            if any(s.contains(point) for s in solids):
                return False
    return True


def largest_fitting(grasp, axis, solids) -> float:
    if not fits(1e-4, grasp, axis, solids):
        return 0.0
    low, high = 1e-4, 0.10
    for _ in range(24):
        mid = 0.5 * (low + high)
        if fits(mid, grasp, axis, solids):
            low = mid
        else:
            high = mid
    return low


def main() -> int:
    robot = load_urdf()
    links = {link.name: link for link in robot.links}
    joints = {joint.name: joint for joint in robot.joints}

    meshes = REPO / "src" / "so101_description" / "meshes"
    fixed_solids = solids_of(links[GRIPPER_LINK], meshes)

    grasp_joint = joints["arm_gripper_frame_joint"]
    grasp = np.array(grasp_joint.origin.xyz)
    # The can stands upright and the descent is vertical, so its axis is the
    # gripper's approach direction - gripper_link's z, whichever way it points.
    axis = np.array([0.0, 0.0, 1.0])

    jaw = joints[GRIPPER_JOINT]
    jaw_origin = np.array(jaw.origin.xyz)
    jaw_rotation = _rpy_to_matrix(*jaw.origin.rpy)
    jaw_axis = np.array(jaw.axis)
    moving_solids = solids_of(links[MOVING_JAW_LINK], meshes)

    print(f"grasp point (gripper_frame_link) in {GRIPPER_LINK}: "
          f"{grasp[0]:+.4f} {grasp[1]:+.4f} {grasp[2]:+.4f}")
    print(f"\ncollisions on {GRIPPER_LINK}:")
    for solid in fixed_solids:
        holds = " *** CONTAINS THE GRASP POINT ***" \
            if solid.contains(grasp) else ""
        print(f"  {solid.description}{holds}")
    print(f"collisions on {MOVING_JAW_LINK}:")
    for solid in moving_solids:
        print(f"  {solid.description}")

    print(f"\n{'gripper':>8s} {'largest cylinder':>17s} {'vs can 0.033':>13s}")
    widest = 0.0
    for angle in ANGLES:
        turn = _axis_rotation(jaw_axis, angle)
        solids = fixed_solids + [
            s.placed(jaw_rotation @ turn, jaw_origin) for s in moving_solids
        ]
        radius = largest_fitting(grasp, axis, solids)
        widest = max(widest, radius)
        verdict = "fits" if radius >= CAN_RADIUS else "too small"
        print(f"{angle:8.3f} {radius:17.4f} {verdict:>13s}")

    print(f"\nThe can is radius {CAN_RADIUS}. A column of 'too small' means "
          "nothing of that size can be at the grasp point with the jaws where "
          "they are.")

    def placed_at(angle):
        turn = _axis_rotation(jaw_axis, angle)
        return fixed_solids + [
            s.placed(jaw_rotation @ turn, jaw_origin) for s in moving_solids
        ]

    sweep_object_sizes(grasp, axis, placed_at(OPEN_ANGLE),
                       placed_at(CLOSED_ANGLE))

    # Where the gap actually is, if it is not where the arm is aiming. The
    # coordinator drives gripper_frame_link to the target, so an opening
    # offset from that frame is a bias the grasp pose has to carry.
    best = search_opening(grasp, axis, fixed_solids, moving_solids,
                          jaw_rotation, jaw_origin, jaw_axis)
    if best is None:
        print("\nNo spot near the jaws holds the can with the jaws open and "
              "traps it when they close - the gripper cannot grasp it "
              "anywhere, not just at the grasp point.")
    else:
        _, offset = best
        print(f"\nNearest spot that IS graspable (fits at gripper "
              f"{OPEN_ANGLE}, trapped at {CLOSED_ANGLE}):")
        print(f"  dx {offset[0]:+.4f}  dy {offset[1]:+.4f} from the grasp "
              f"point ({np.linalg.norm(offset)*1000:.1f} mm away)")

    if widest < CAN_RADIUS:
        print("\nFAIL: the can does not fit at the point the arm aims at.")
        return 1
    print("\nOK: the jaws admit the can where the arm aims.")
    return 0


# coordinator.yaml's gripper_open and gripper_closed.
OPEN_ANGLE = 1.2
CLOSED_ANGLE = 0.0
SEARCH_HALF_WIDTH = 0.06
SEARCH_STEP = 0.005


OBJECT_RADII = [0.005, 0.010, 0.015, 0.020, 0.025, 0.033]
OBJECT_HEIGHTS = [0.024, 0.040, 0.070]


def graspable(radius, height, grasp, axis, open_solids, closed_solids,
              grasp_height=GRASP_HEIGHT) -> bool:
    """Free with the jaws open, and caught once they close.

    The second half is what separates a grasp from a near miss. An object the
    jaws close straight past is not held, and an object entirely below them
    clears everything precisely because they never touch it.
    """
    if not fits(radius, grasp, axis, open_solids, samples=36,
                height=height, grasp_height=grasp_height):
        return False
    return not fits(radius, grasp, axis, closed_solids, samples=36,
                    height=height, grasp_height=grasp_height)


def sweep_object_sizes(grasp, axis, open_solids, closed_solids) -> None:
    """Whether a smaller object would be gripped at the point the arm aims."""
    print(f"\nAt the grasp point, by object size "
          f"(grasp_height {GRASP_HEIGHT}):")
    print(f"{'radius':>8s}" + "".join(f"{h:>10.3f}" for h in OBJECT_HEIGHTS))
    for radius in OBJECT_RADII:
        cells = []
        for height in OBJECT_HEIGHTS:
            held = graspable(radius, height, grasp, axis, open_solids,
                             closed_solids)
            reaches = height > GRASP_HEIGHT
            cells.append("held" if held else
                         ("under" if not reaches else "no"))
        print(f"{radius:8.3f}" + "".join(f"{c:>10s}" for c in cells))
    print("  held  = fits open, trapped closed\n"
          "  no    = reaches the jaws but is not held\n"
          "  under = shorter than grasp_height, so the jaws close above it")


def search_opening(grasp, axis, fixed_solids, moving_solids,
                   jaw_rotation, jaw_origin, jaw_axis):
    """The nearest spot the jaws could actually grasp a can at.

    Free space alone is not a grasp - the air beside the gripper is free too.
    A graspable spot is one the can fits in with the jaws open and does NOT
    fit in once they close, which is what "the jaws end up on it" means.
    """
    def placed(angle):
        turn = _axis_rotation(jaw_axis, angle)
        return fixed_solids + [
            s.placed(jaw_rotation @ turn, jaw_origin) for s in moving_solids
        ]

    open_solids, closed_solids = placed(OPEN_ANGLE), placed(CLOSED_ANGLE)
    steps = np.arange(-SEARCH_HALF_WIDTH, SEARCH_HALF_WIDTH + 1e-9,
                      SEARCH_STEP)
    best = None
    for dx in steps:
        for dy in steps:
            offset = np.array([dx, dy, 0.0])
            if best is not None \
                    and np.linalg.norm(offset) >= np.linalg.norm(best):
                continue
            centre = grasp + offset
            if not fits(CAN_RADIUS, centre, axis, open_solids, samples=36):
                continue
            if fits(CAN_RADIUS, centre, axis, closed_solids, samples=36):
                continue        # the jaws close past it - nothing is gripped
            best = offset
    if best is None:
        return None
    return CAN_RADIUS, best


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
