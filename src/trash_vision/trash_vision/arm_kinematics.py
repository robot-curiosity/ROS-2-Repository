"""
Planar kinematics and closed-form inverse kinematics for the SO-101.

Pure Python (math only, no numpy, no URDF parser, no ROS). Ported from
``drivebase_behaviour/kinematics.py`` and ``ik.py`` on branch
``experimental/behaviour-coordinator``. Instead of parsing the URDF at run
time, the planar-arm constants below were derived once with that branch's
``extract_planar_arm`` from ``so101_upstream.urdf`` (chain ``base_link`` to
``gripper_frame_link``), so they match the model that code used.

Frame: ``arm_base_link`` (x forward, y left, z up, origin at the arm base).
Joint values are radians. Planar angles (q) are positive from r toward z;
joint commands are ``q = sense * t`` and ``sense`` is -1 on this arm.
``r`` is the signed projection onto the radial direction measured from the
pan axis (not hypot from the base origin), see ``PlanarArm``.
"""
from dataclasses import dataclass
import math

# From so101_macro.xacro on the experimental branch (radians).
JOINT_LIMITS = {
    'shoulder_pan': (-1.91986, 1.91986),
    'shoulder_lift': (-1.74533, 1.74533),
    'elbow_flex': (-1.69, 1.69),
    'wrist_flex': (-1.65806, 1.65806),
    'wrist_roll': (-2.74385, 2.84121),
    'gripper': (-0.174533, 1.74533),
}


def _wrap(angle):
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class PlanarArm:
    """Three pitch joints reduced to a planar 3R arm (see module docs)."""

    l1: float
    l2: float
    l3: float
    a1: float
    a2: float
    a3: float
    origin_r: float
    origin_z: float
    axis_xy: tuple
    radial_xy: tuple
    sense: float
    pan_sense: float

    def _radial(self, pan):
        angle = self.pan_sense * pan
        c, s = math.cos(angle), math.sin(angle)
        return (self.radial_xy[0] * c - self.radial_xy[1] * s,
                self.radial_xy[0] * s + self.radial_xy[1] * c)

    def to_planar(self, point, pan=0.0):
        """Base-link (x, y, z) to planar (r, z) for the arm panned to pan."""
        ux, uy = self._radial(pan)
        dx = point[0] - self.axis_xy[0]
        dy = point[1] - self.axis_xy[1]
        return (dx * ux + dy * uy, point[2])

    def from_planar(self, r, z, pan=0.0):
        """Planar (r, z) back to an arm-base-link point at pan."""
        ux, uy = self._radial(pan)
        return (self.axis_xy[0] + r * ux, self.axis_xy[1] + r * uy, z)

    def bearing(self, point):
        """Pan command that turns the arm plane to face point."""
        dx = point[0] - self.axis_xy[0]
        dy = point[1] - self.axis_xy[1]
        zero = math.atan2(self.radial_xy[1], self.radial_xy[0])
        return self.pan_sense * _wrap(math.atan2(dy, dx) - zero)

    def tip(self, t1, t2, t3):
        """Planar (r, z) of the tip for the three pitch joint values."""
        q1, q2, q3 = self.sense * t1, self.sense * t2, self.sense * t3
        r_terms = [
            self.l1 * math.cos(self.a1 + q1),
            self.l2 * math.cos(self.a2 + q1 + q2),
            self.l3 * math.cos(self.a3 + q1 + q2 + q3)]
        r = self.origin_r + sum(r_terms)
        z_terms = [
            self.l1 * math.sin(self.a1 + q1),
            self.l2 * math.sin(self.a2 + q1 + q2),
            self.l3 * math.sin(self.a3 + q1 + q2 + q3)]
        z = self.origin_z + sum(z_terms)
        return (r, z)


# Derived from so101_upstream.urdf (see module docstring). Not measured on
# the real arm.
SO101 = PlanarArm(
    l1=0.11600002111908701, l2=0.1350001851146071, l3=0.15942273309208735,
    a1=1.327013106870974, a2=0.038527996871866026,
    a3=-0.049576250093597375,
    origin_r=0.0303997256555827, origin_z=0.11659995149802566,
    axis_xy=(0.0388353, -8.97657e-09),
    radial_xy=(0.9999999996592355, -2.610610876270341e-05),
    sense=-1.0, pan_sense=-1.0)


@dataclass(frozen=True)
class IkSolution:
    """Joint values in radians for the four solved joints."""

    shoulder_pan: float
    shoulder_lift: float
    elbow_flex: float
    wrist_flex: float


def _within(name, value, limits):
    low, high = limits[name]
    return low <= value <= high


def solve(arm, target_xyz, approach_angle, limits=None):
    """
    Closed-form IK. Returns an IkSolution or None if out of reach/limits.

    approach_angle is the planar direction of the last link (straight down is
    -pi/2). Never clamps: an unreachable target returns None.
    """
    limits = JOINT_LIMITS if limits is None else limits
    pan = arm.bearing(target_xyz)
    if not _within('shoulder_pan', pan, limits):
        return None
    r, z = arm.to_planar(target_xyz, pan)
    wrist_r = r - arm.origin_r - arm.l3 * math.cos(approach_angle)
    wrist_z = z - arm.origin_z - arm.l3 * math.sin(approach_angle)
    distance = math.hypot(wrist_r, wrist_z)
    if distance > arm.l1 + arm.l2 or distance < abs(arm.l1 - arm.l2):
        return None
    cos_elbow = (distance * distance - arm.l1 * arm.l1 - arm.l2 * arm.l2
                 ) / (2.0 * arm.l1 * arm.l2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    # Elbow-down first; the other branch only if that breaks a limit.
    for interior in (-math.acos(cos_elbow), math.acos(cos_elbow)):
        solution = _finish(arm, wrist_r, wrist_z, interior,
                           approach_angle, pan, limits)
        if solution is not None:
            return solution
    return None


def _finish(arm, wrist_r, wrist_z, interior, approach_angle, pan, limits):
    beta = math.atan2(arm.l2 * math.sin(interior),
                      arm.l1 + arm.l2 * math.cos(interior))
    link1 = math.atan2(wrist_z, wrist_r) - beta
    q1 = link1 - arm.a1
    q2 = (link1 + interior) - arm.a2 - q1
    q3 = approach_angle - arm.a3 - q1 - q2
    lift, elbow, flex = arm.sense * q1, arm.sense * q2, arm.sense * q3
    for name, value in (('shoulder_lift', lift), ('elbow_flex', elbow),
                        ('wrist_flex', flex)):
        if not _within(name, value, limits):
            return None
    return IkSolution(pan, lift, elbow, flex)


def graspable(arm, r, z, approach_angle, clearance_height=0.0, limits=None):
    """Return True if both the grasp point and the point above it solve."""
    grasp = arm.from_planar(r, z)
    if solve(arm, grasp, approach_angle, limits) is None:
        return False
    above = arm.from_planar(r, z + clearance_height)
    return solve(arm, above, approach_angle, limits) is not None


def reachable_radius_band(arm, z, approach_angle, clearance_height=0.0,
                          r_min=0.0, r_max=0.6, step=0.002, limits=None):
    """
    Widest contiguous (low, high) span of planar r graspable at height z.

    Returns None if nothing in [r_min, r_max] solves. Scanned, not solved.
    """
    best = None
    run_start = None
    previous = r_min
    steps = int((r_max - r_min) / step) + 1
    for index in range(steps + 1):
        r = r_min + index * step
        ok = index < steps and graspable(
            arm, r, z, approach_angle, clearance_height, limits)
        if ok and run_start is None:
            run_start = r
        elif not ok and run_start is not None:
            if best is None or (previous - run_start) > (best[1] - best[0]):
                best = (run_start, previous)
            run_start = None
        if ok:
            previous = r
    return best
