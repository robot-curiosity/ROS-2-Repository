"""Pure-Python helpers for the SO101 arm node (no ROS or LeRobot needed)."""
from dataclasses import dataclass
import math

from trash_vision import arm_kinematics as ak

JOINT_NAMES = [
    'shoulder_pan', 'shoulder_lift', 'elbow_flex',
    'wrist_flex', 'wrist_roll', 'gripper',
]

# Placeholder poses in the units of the backend (degrees if use_degrees).
# TODO: Bryan to calibrate these on the real arm.
DEFAULT_POSES = {
    'stow': [0.0, -90.0, 90.0, 60.0, 0.0, 0.0],
    'pre_grasp': [0.0, -30.0, 60.0, 30.0, 0.0, 50.0],
    'grasp': [0.0, 0.0, 45.0, 20.0, 0.0, 50.0],
    'lift': [0.0, -40.0, 60.0, 30.0, 0.0, 0.0],
    'deposit': [90.0, -30.0, 60.0, 30.0, 0.0, 50.0],
}

# Sequences of (pose name, gripper override or None).
SEQUENCES = {
    'pick': [('pre_grasp', None), ('grasp', None), ('grasp', 'close'),
             ('lift', 'close'), ('stow', 'close')],
    'deposit': [('deposit', 'close'), ('deposit', 'open'),
                ('stow', 'close')],
    'stow': [('stow', None)],
}


def clamp(value, low, high):
    """Clamp value to [low, high]."""
    return max(low, min(high, value))


def clamp_pose(pose, limits):
    """Clamp each joint to its (low, high) limit."""
    return [clamp(p, lo, hi) for p, (lo, hi) in zip(pose, limits)]


def validate_pose(pose, n=len(JOINT_NAMES)):
    """Return an error string for a bad pose, or None if it is fine."""
    if len(pose) != n:
        return 'pose needs %d values, got %d' % (n, len(pose))
    return None


def build_poses(overrides):
    """Merge flat parameter overrides ({name: list}) over the defaults.

    Returns (poses, problems). Invalid overrides are ignored and reported.
    """
    poses = {k: list(v) for k, v in DEFAULT_POSES.items()}
    problems = []
    for name, value in overrides.items():
        if name not in poses:
            problems.append("unknown pose '%s'" % name)
            continue
        value = list(value)
        err = validate_pose(value)
        if err:
            problems.append("pose '%s': %s" % (name, err))
            continue
        poses[name] = value
    return poses, problems


def interpolate(start, end, max_step):
    """Yield intermediate poses so no joint moves more than max_step per step.

    The last yielded pose is always `end`.
    """
    if max_step <= 0:
        yield list(end)
        return
    biggest = max(abs(e - s) for s, e in zip(start, end))
    steps = max(1, int(-(-biggest // max_step)))
    for i in range(1, steps + 1):
        f = i / steps
        yield [s + (e - s) * f for s, e in zip(start, end)]


def plan_sequence(name, poses, gripper_open, gripper_closed):
    """Return the list of target poses for a named sequence.

    Raises KeyError for an unknown sequence.
    """
    targets = []
    for pose_name, grip in SEQUENCES[name]:
        pose = list(poses[pose_name])
        if grip == 'open':
            pose[-1] = gripper_open
        elif grip == 'close':
            pose[-1] = gripper_closed
        targets.append(pose)
    return targets


def grasp_held(gripper_pos, gripper_closed, min_gap):
    """Return True if the gripper stopped open by at least min_gap (holding item)."""
    return gripper_pos - gripper_closed >= min_gap


@dataclass(frozen=True)
class PickConfig:
    """Geometry and joint mapping for IK pick waypoints.

    Defaults are placeholders: TODO: Bryan to fill in / measure. The mount
    height is the arm shelf top per STATE.md (215.5 mm, from CAD, not
    measured). The mount x/y default is the experimental branch's drivebase
    value, NOT this robot's CAD. Zone limits are the pick zone from STATE.md.
    """

    mount_x: float = 0.115     # arm base ahead of base_link (m) TODO: Bryan
    mount_y: float = 0.0       # arm base left of base_link (m) TODO: Bryan
    mount_height: float = 0.2155   # arm base above ground (m), CAD
    grasp_height: float = 0.03     # gripper tip height at grasp (m) TODO
    clearance: float = 0.06    # lift above the grasp point for pre-grasp (m)
    zone_min_x: float = 0.35   # pick zone ahead of base_link (m), STATE.md
    zone_max_x: float = 0.56
    zone_max_abs_y: float = 0.30   # sideways limit (m) TODO: Bryan
    approach_deg: tuple = (-90.0, -75.0, -60.0)  # steepest first
    use_degrees: bool = True
    # Model radians -> backend units: value = sign * model + offset.
    # TODO: Bryan to calibrate against the real arm (LeRobot calibration).
    joint_sign: tuple = (1.0, 1.0, 1.0, 1.0)
    joint_offset: tuple = (0.0, 0.0, 0.0, 0.0)
    gripper_open: float = 50.0


def _to_backend(rad, sign, offset, use_degrees):
    value = math.degrees(rad) if use_degrees else rad
    return sign * value + offset


def _pose(sol, cfg, gripper):
    joints = (sol.shoulder_pan, sol.shoulder_lift, sol.elbow_flex,
              sol.wrist_flex)
    out = [_to_backend(j, cfg.joint_sign[i], cfg.joint_offset[i],
                       cfg.use_degrees) for i, j in enumerate(joints)]
    return out + [0.0, gripper]


def plan_pick_poses(base_x, base_y, cfg=None):
    """IK pick waypoints for a floor target at (base_x, base_y) in base_link.

    Returns (poses, reason). poses is {"pre_grasp", "grasp", "lift"} (6-value
    poses in backend units) or None, with reason saying why (outside the
    pick zone, or unreachable by the arm). Never clamps or guesses.
    """
    cfg = PickConfig() if cfg is None else cfg
    if not (cfg.zone_min_x <= base_x <= cfg.zone_max_x):
        return None, 'outside pick zone (x %.2f m)' % base_x
    if abs(base_y) > cfg.zone_max_abs_y:
        return None, 'outside pick zone (y %.2f m)' % base_y
    dx = base_x - cfg.mount_x
    dy = base_y - cfg.mount_y
    z_grasp = cfg.grasp_height - cfg.mount_height
    grasp = (dx, dy, z_grasp)
    above = (dx, dy, z_grasp + cfg.clearance)
    for deg in cfg.approach_deg:
        angle = math.radians(deg)
        s_grasp = ak.solve(ak.SO101, grasp, angle)
        s_above = ak.solve(ak.SO101, above, angle)
        if s_grasp is None or s_above is None:
            continue
        return {
            'pre_grasp': _pose(s_above, cfg, cfg.gripper_open),
            'grasp': _pose(s_grasp, cfg, cfg.gripper_open),
            'lift': _pose(s_above, cfg, 0.0),
        }, 'ok'
    return None, 'target not reachable by the arm'
