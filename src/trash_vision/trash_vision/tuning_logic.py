"""Which behavior parameters may be changed while running, and the limits.

Pure Python, no ROS. Used by the behavior node (to accept or reject a live
parameter change) and by the dashboard. The limits are working values that
keep a typo from doing harm; they are not measurements of the robot.
"""
import math

# name -> (minimum, maximum, step, unit, label)
TUNABLES = {
    'stop_distance': (0.15, 1.0, 0.01, 'm', 'Obstacle stop distance'),
    'slow_distance': (0.2, 2.0, 0.05, 'm', 'Obstacle slow distance'),
    'rear_stop_distance': (0.1, 1.0, 0.01, 'm', 'Rear stop distance'),
    'side_clearance': (0.1, 1.0, 0.01, 'm', 'Side clearance for spinning'),
    'search_turn_speed': (0.1, 1.0, 0.05, 'rad/s', 'Search spin speed'),
    'search_speed': (0.05, 0.4, 0.01, 'm/s', 'Search row speed'),
    'approach_speed': (0.05, 0.4, 0.01, 'm/s', 'Approach speed'),
    'max_turn_speed': (0.1, 1.0, 0.05, 'rad/s', 'Approach turn limit'),
    'avoid_turn_speed': (0.1, 1.0, 0.05, 'rad/s', 'Avoidance turn speed'),
    'recover_backup_speed': (0.0, 0.2, 0.01, 'm/s', 'Backup speed'),
    'safety_max_linear': (0.05, 0.5, 0.01, 'm/s', 'Safety speed limit'),
    'safety_max_angular': (0.1, 1.5, 0.05, 'rad/s', 'Safety turn limit'),
    'safety_max_linear_accel': (0.1, 1.0, 0.05, 'm/s^2', 'Ramp limit'),
    'safety_max_angular_accel': (0.2, 4.0, 0.1, 'rad/s^2', 'Turn ramp limit'),
    'pick_floor_y': (0.5, 1.0, 0.01, 'image y', 'Pick zone floor y'),
    'pick_center_tolerance': (0.02, 0.4, 0.01, 'image x', 'Pick tolerance'),
    'search_row_spacing': (0.0, 3.0, 0.1, 'm', 'Search row spacing (0=auto)'),
}


def validate_changes(changes, current):
    """Check a batch of live changes against limits and each other.

    changes maps parameter name to new value; current maps name to the value
    in use now (used by the rules that compare two parameters). Returns a list
    of error strings, empty when the whole batch is acceptable. Names that are
    not in TUNABLES are reported as errors.
    """
    errors = []
    for name, value in changes.items():
        if name not in TUNABLES:
            errors.append('%s is not tunable' % name)
            continue
        low, high = TUNABLES[name][:2]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value)):
            errors.append('%s must be a number' % name)
        elif not low <= value <= high:
            errors.append('%s must be between %g and %g' % (name, low, high))
    if errors:
        return errors
    merged = dict(current)
    merged.update(changes)
    if {'stop_distance', 'slow_distance'} & set(changes):
        stop, slow = merged.get('stop_distance'), merged.get('slow_distance')
        if stop is not None and slow is not None and slow - stop < 0.049:
            errors.append(
                'slow_distance must be at least 0.05 above stop_distance')
    if 'stop_distance' in changes:
        hard = merged.get('safety_hard_stop_distance')
        if hard is not None and merged['stop_distance'] <= hard:
            errors.append(
                'stop_distance must be above safety_hard_stop_distance '
                '(%g)' % hard)
    return errors
