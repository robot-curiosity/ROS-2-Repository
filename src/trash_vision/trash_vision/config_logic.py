"""Pure-Python helpers for config/robot.yaml (no ROS needed).

robot.yaml is the single place for hardware values. A value that still has
its "not set" marker (-1 pin, 0 number, empty string) is reported by name.
"""

# Launch subsystem -> node (also the top-level key in robot.yaml).
SUBSYSTEMS = {
    'drive': ['drive'],
    'sonar': ['ultrasonic'],
    'arm': ['arm'],
    'camera': ['detector'],
    'imu_gps': ['gps_imu'],
}

# Parameters a node cannot work without; the "not set" marker is -1 (pins),
# 0 / 0.0 (numbers) or "" (strings). A list is unset if any entry is.
REQUIRED = {
    'drive': [
        'pwm_pins', 'dir_pins', 'enc_a_pins', 'enc_b_pins',
        'ticks_per_rev', 'wheel_radius_m', 'track_width_m'],
    'pico_bridge': [
        'serial_port', 'pico_pins', 'ticks_per_rev', 'wheel_radius_m',
        'track_width_m'],
    'ultrasonic': ['trig_pins', 'echo_pins'],
    'arm': ['port'],
    'gps_imu': ['gps_port', 'imu_model'],
}


def is_unset(value):
    """Return True if value is a "not filled in yet" marker."""
    if isinstance(value, (list, tuple)):
        return len(value) == 0 or any(is_unset(v) for v in value)
    if isinstance(value, str):
        return value.strip() == ''
    if isinstance(value, bool):
        return False
    if isinstance(value, (int, float)):
        return value <= 0
    return value is None


def find_unset(node, params):
    """Names of required parameters of `node` that are missing or unset."""
    return [name for name in REQUIRED.get(node, [])
            if is_unset(params.get(name))]


def unset_message(node, params):
    """One warning line naming every unset parameter, or None if ready."""
    names = find_unset(node, params)
    if not names:
        return None
    return ('%s idle: set %s in config/robot.yaml. '
            'TODO: Bryan to fill in.' % (node, ', '.join(names)))


def node_params(config, node):
    """Return the ros__parameters dict of a node in a loaded robot.yaml."""
    section = (config or {}).get(node) or {}
    return dict(section.get('ros__parameters') or {})
