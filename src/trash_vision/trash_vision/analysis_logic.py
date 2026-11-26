"""Offline analysis of a recorded run (pure Python, no ROS).

Numbers behind review findings #2 (sonar detection delay) and #6 (stopping
distance). Distances come from wheel odometry, so wheel slip is not seen:
check the important ones with a tape measure.
"""
import json
import math

from trash_vision import ultrasonic_logic as ul

MOVING_CMD = 0.02     # m/s: commanded speed that counts as driving
STOPPED_SPEED = 0.01  # m/s: measured speed that counts as standing still
ZERO_CMD = 1e-3


def by_topic(events):
    """Group events into {topic: [(t, data), ...]} keeping time order."""
    out = {}
    for e in events:
        out.setdefault(e['topic'], []).append((e['t'], e['data']))
    return out


def _percentile(values, fraction):
    ordered = sorted(values)
    index = int(math.ceil(fraction * len(ordered))) - 1
    return ordered[max(0, min(index, len(ordered) - 1))]


def loop_rates(topics):
    """Rate and gap statistics for every topic with at least two messages.

    'missed' estimates lost or late cycles: every gap longer than 1.5 times
    the median interval counts round(gap / median) - 1.
    """
    out = {}
    for name, items in sorted(topics.items()):
        times = [t for t, _ in items]
        gaps = [b - a for a, b in zip(times, times[1:])]
        if not gaps or times[-1] <= times[0]:
            continue
        median = _percentile(gaps, 0.5)
        missed = 0
        if median > 0:
            missed = sum(int(round(g / median)) - 1
                         for g in gaps if g > 1.5 * median)
        out[name] = {'count': len(times),
                     'hz': (len(times) - 1) / (times[-1] - times[0]),
                     'median_s': median, 'p95_s': _percentile(gaps, 0.95),
                     'max_s': max(gaps), 'missed': missed}
    return out


def dropped_frames(topics):
    """Gaps in the detector's frame_number on /vision/target.

    These are processed frames whose result never reached the recording. Frames
    the camera produced but the detector skipped on purpose are not numbered.
    """
    numbers = []
    for _, data in topics.get('/vision/target', []):
        try:
            number = json.loads(data['data'])['frame_number']
        except (KeyError, TypeError, ValueError):
            continue
        if isinstance(number, int):
            numbers.append(number)
    dropped = sum(b - a - 1 for a, b in zip(numbers, numbers[1:]) if b > a + 1)
    return {'frames': len(numbers), 'dropped': dropped}


def stop_events(topics):
    """Distance and time to standstill after each stop command.

    A stop command is /cmd_vel linear going from driving to zero. 'start' is
    the beginning of the final slow-down (equal to 'zero' for an immediate
    stop). Stops where driving resumes before standstill are skipped.
    """
    cmds = [(t, abs(d['linear'])) for t, d in topics.get('/cmd_vel', [])]
    odom = topics.get('/odom', [])
    reasons = topics.get('/behavior/safety', [])
    out = []
    for i in range(1, len(cmds)):
        if not (cmds[i][1] <= ZERO_CMD < cmds[i - 1][1]):
            continue
        # Walk back over a ramp: samples that keep getting slower.
        first = i
        while first >= 2 and cmds[first - 1][1] < cmds[first - 2][1] - ZERO_CMD:
            first -= 1
        peak = max(c for _, c in cmds[max(0, first - 1):i])
        if peak <= MOVING_CMD:
            continue
        start, zero = cmds[first][0], cmds[i][0]
        resume = next((t for t, c in cmds[i:] if c > MOVING_CMD), math.inf)
        distance, last, stopped = 0.0, None, None
        for t, d in odom:
            if t < start:
                last = (d['x'], d['y'])
                continue
            if t >= resume:
                break
            if last is not None:
                distance += math.hypot(d['x'] - last[0], d['y'] - last[1])
            last = (d['x'], d['y'])
            if t >= zero and abs(d['v']) < STOPPED_SPEED:
                stopped = t
                break
        if stopped is None:
            continue
        # The safety reason is published just before or after cmd_vel.
        reason = [d['data'] for t, d in reasons
                  if start - 0.05 <= t <= zero + 0.05 and d['data']]
        out.append({'start': start, 'zero': zero, 'speed': peak,
                    'distance_m': distance, 'time_s': stopped - start,
                    'reason': reason[-1] if reason else ''})
    return out


def sonar_delay(topics, marks, stop_distance, median_window=3,
                timeout_s=5.0):
    """Sonar detection delay: measured from marks, and bounded from rates.

    For each mark, the delay is the time until the first front reading at or
    under stop_distance (only if the front was clear at the mark), and until
    the next zero linear /cmd_vel. The bound assumes the node's median filter
    needs window // 2 + 1 new pings of one sensor to follow a step.
    """
    needed = max(1, int(median_window)) // 2 + 1
    front = []
    periods = {}
    for name in ul.SENSOR_NAMES:
        items = topics.get('/ultrasonic/' + name, [])
        gaps = [b[0] - a[0] for a, b in zip(items, items[1:])]
        if gaps:
            period = _percentile(gaps, 0.5)
            periods[name] = {'period_s': period,
                             'min_delay_s': (needed - 1) * period,
                             'max_delay_s': needed * period}
        if name in ul.FRONT_NAMES:
            front += [(t, name, d['range']) for t, d in items]
    front.sort()
    cmds = topics.get('/cmd_vel', [])
    measured = []
    for mark in marks:
        t0 = mark['t']
        latest = {}
        for t, name, rng in front:
            if t <= t0:
                latest[name] = rng
        entry = {'t': t0, 'label': mark.get('label', ''),
                 'detect_s': None, 'stop_s': None}
        if any(r <= stop_distance for r in latest.values()):
            entry['label'] += ' (skipped: front already blocked)'
        else:
            seen = next((t for t, _, r in front
                         if t0 < t <= t0 + timeout_s and r <= stop_distance),
                        None)
            if seen is not None:
                entry['detect_s'] = seen - t0
                zero = next((t for t, d in cmds
                             if t >= seen and t <= t0 + timeout_s
                             and abs(d['linear']) <= ZERO_CMD), None)
                if zero is not None:
                    entry['stop_s'] = zero - t0
        measured.append(entry)
    return {'periods': periods, 'marks': measured}


def analyze(events, marks=(), stop_distance=0.25, median_window=3):
    """All metrics for one run."""
    topics = by_topic(events)
    duration = events[-1]['t'] - events[0]['t'] if events else 0.0
    return {'duration_s': duration,
            'rates': loop_rates(topics),
            'frames': dropped_frames(topics),
            'stops': stop_events(topics),
            'sonar': sonar_delay(topics, marks, stop_distance, median_window)}


def _num(value):
    return '-' if value is None else '%.2f' % value


def format_report(result):
    """Plain text report."""
    lines = ['Run length: %.1f s' % result['duration_s'], '',
             'Loop rates (missed = estimated from long gaps):',
             '%-22s %6s %7s %7s %7s %6s' % (
                 'topic', 'count', 'Hz', 'p95 s', 'max s', 'missed')]
    for name, r in result['rates'].items():
        if name == '/rosout':
            continue
        lines.append('%-22s %6d %7.1f %7.3f %7.3f %6d' % (
            name, r['count'], r['hz'], r['p95_s'], r['max_s'], r['missed']))
    frames = result['frames']
    lines += ['', 'Detector frames recorded: %d, dropped: %d'
              % (frames['frames'], frames['dropped']),
              '', 'Stops (odometry distance, wheel slip not included):']
    for s in result['stops']:
        lines.append('  at %.2f: from %.2f m/s, %.3f m in %.2f s  %s' % (
            s['zero'], s['speed'], s['distance_m'], s['time_s'], s['reason']))
    if result['stops']:
        worst = max(s['distance_m'] for s in result['stops'])
        lines.append('  worst: %.3f m over %d stop(s)'
                     % (worst, len(result['stops'])))
    else:
        lines.append('  none found (needs /cmd_vel and /odom)')
    lines += ['', 'Sonar update period and filter delay bound:']
    for name, p in result['sonar']['periods'].items():
        lines.append('  %-8s period %.2f s, delay %.2f to %.2f s' % (
            name, p['period_s'], p['min_delay_s'], p['max_delay_s']))
    lines.append('Measured from marks (includes your reaction time):')
    for m in result['sonar']['marks']:
        lines.append('  at %.2f: seen after %s s, stop command after %s s %s'
                     % (m['t'], _num(m['detect_s']), _num(m['stop_s']),
                        m['label']))
    if not result['sonar']['marks']:
        lines.append('  no marks (press Enter during record_trial)')
    return '\n'.join(lines)
