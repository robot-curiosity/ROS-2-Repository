"""Pure-Python helpers for the SO101 arm node (no ROS or LeRobot needed)."""

JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
]

# Placeholder poses in the units of the backend (degrees if use_degrees).
# TODO: Bryan to calibrate these on the real arm.
DEFAULT_POSES = {
    "stow": [0.0, -90.0, 90.0, 60.0, 0.0, 0.0],
    "pre_grasp": [0.0, -30.0, 60.0, 30.0, 0.0, 50.0],
    "grasp": [0.0, 0.0, 45.0, 20.0, 0.0, 50.0],
    "lift": [0.0, -40.0, 60.0, 30.0, 0.0, 0.0],
    "deposit": [90.0, -30.0, 60.0, 30.0, 0.0, 50.0],
}

# Sequences of (pose name, gripper override or None).
SEQUENCES = {
    "pick": [("pre_grasp", None), ("grasp", None), ("grasp", "close"),
             ("lift", "close"), ("stow", "close")],
    "deposit": [("deposit", "close"), ("deposit", "open"),
                ("stow", "close")],
    "stow": [("stow", None)],
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
        return "pose needs %d values, got %d" % (n, len(pose))
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
        if grip == "open":
            pose[-1] = gripper_open
        elif grip == "close":
            pose[-1] = gripper_closed
        targets.append(pose)
    return targets


def grasp_held(gripper_pos, gripper_closed, min_gap):
    """True if the gripper stopped open by at least min_gap (holding item)."""
    return gripper_pos - gripper_closed >= min_gap
