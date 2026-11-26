"""Feed recorded inputs back through the behavior logic (pure Python).

One replay tick per recorded /behavior/state message, doing what
behavior_node._tick does. Arm service replies are not topics, so they are
inferred from the recorded state changes. Live parameter changes made during
the run are not replayed, and a recording that starts mid-mission differs
until the states line up.
"""
import json
import math

from trash_vision import behavior_logic as bl
from trash_vision import safety_logic as sfl
from trash_vision import sonar_scheduler as ss
from trash_vision import ultrasonic_logic as ul

_ARM_OK = {bl.PICK: bl.DEPOSIT, bl.DEPOSIT: bl.STOW, bl.STOW: bl.SEARCH}


def _parse_target(text):
    try:
        target = json.loads(text)
    except (TypeError, ValueError):
        return None
    return target if isinstance(target, dict) else None


def replay(events, params=None):
    """Return one record per tick with replayed and recorded outputs."""
    params = params or {}
    machine = bl.BehaviorMachine(
        {k: params[k] for k in bl.DEFAULTS if k in params})
    safety = sfl.SafetyStage(
        {k: params[k] for k in sfl.DEFAULTS if k in params})
    cfg = machine.p
    ranges, stamps = {}, {}
    target = target_stamp = pose = pose_stamp = None
    recorded_cmd = None
    out = []
    for event in sorted(events, key=lambda e: e['t']):
        t, topic, data = event['t'], event['topic'], event['data']
        if topic.startswith('/ultrasonic/'):
            name = topic.rsplit('/', 1)[1]
            ranges[name] = data['range']
            stamps[name] = t
        elif topic == '/vision/target':
            target, target_stamp = _parse_target(data['data']), t
        elif topic == '/odom':
            pose, pose_stamp = (data['x'], data['y'], data['yaw']), t
        elif topic == '/cmd_vel':
            recorded_cmd = (data['linear'], data['angular'])
        if topic != '/behavior/state':
            continue
        state = data['data']
        if machine.state in _ARM_OK and state != machine.state:
            action = {bl.PICK: 'pick', bl.DEPOSIT: 'deposit',
                      bl.STOW: 'stow'}[machine.state]
            machine.arm_result(state == _ARM_OK[machine.state], t, action)
        stale = float(cfg['sonar_stale_timeout'])
        front, rear, left, right = (
            ss.apply_staleness(names, ranges, stamps, t, stale)
            for names in (ul.FRONT_NAMES, ul.REAR_NAMES,
                          ul.LEFT_NAMES, ul.RIGHT_NAMES))
        fresh = bl.fresh_target(
            target, target_stamp, t, float(cfg['target_stale_timeout']))
        use_pose = pose
        if pose_stamp is None or t - pose_stamp > 1.0:
            use_pose = None
        cmd = machine.step(t, fresh, front, rear, left, right, pose=use_pose)
        safety.note_update(t)
        healthy = sfl.sonars_healthy(
            ul.SENSOR_NAMES, stamps, t, stale,
            bool(safety.cfg['safety_require_sonars']))
        linear, angular = safety.apply(
            t, float(cmd.linear), float(cmd.angular),
            sfl.front_minimum(front), healthy)
        out.append({
            't': t, 'state': machine.state, 'recorded_state': state,
            'linear': linear, 'angular': angular, 'recorded': recorded_cmd,
            'safety': safety.reason, 'front': sfl.front_minimum(front),
            'rear': bl.min_range(rear), 'left': bl.min_range(left),
            'right': bl.min_range(right),
            'target': bool(fresh and fresh.get('detected'))})
    return out


def mismatches(ticks, tolerance=0.1):
    """Ticks where the state or a command differs from the recording."""
    bad = []
    for k in ticks:
        rec = k['recorded']
        differs = k['state'] != k['recorded_state']
        if rec is not None:
            differs = differs or (
                abs(k['linear'] - rec[0]) > tolerance
                or abs(k['angular'] - rec[1]) > 4 * tolerance)
        if differs:
            bad.append(k)
    return bad


def _metres(value):
    return 'inf' if value is None or math.isinf(value) else '%.2f' % value


def format_tick(k):
    """One line describing the inputs and decision of a tick."""
    rec = k['recorded']
    was = '-' if rec is None else '%+.2f/%+.2f' % rec
    return ('%.2f %-8s (was %-8s) cmd %+.2f/%+.2f (was %s) F %s B %s L %s '
            'R %s target %s %s' % (
                k['t'], k['state'], k['recorded_state'], k['linear'],
                k['angular'], was, _metres(k['front']), _metres(k['rear']),
                _metres(k['left']), _metres(k['right']),
                'yes' if k['target'] else 'no', k['safety']))
