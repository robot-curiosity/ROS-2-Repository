"""Pure-Python helpers for record_trial (no ROS)."""
import json
import os
import re

from trash_vision import ultrasonic_logic as ul

DEFAULT_ROOT = os.path.join('~', 'curiosity_runs')

SONAR_TOPICS = ['/ultrasonic/' + n for n in ul.SENSOR_NAMES]

# Everything the analyzer and the replay need. No camera images: the detector
# does not publish any.
TOPICS = ['/cmd_vel', '/odom'] + SONAR_TOPICS + [
    '/vision/target', '/vision/direction', '/vision/fps',
    '/behavior/state', '/behavior/safety',
    '/imu/data', '/gps/fix', '/rosout']


def next_run_name(date, existing):
    """Return '<date>_runNN', one above the highest NN already used today."""
    pattern = re.compile(re.escape(date) + r'_run(\d+)$')
    used = [int(m.group(1)) for m in map(pattern.match, existing) if m]
    return '%s_run%02d' % (date, max(used, default=0) + 1)


def bag_command(bag_dir, topics=None):
    """Return the ros2 bag record command (MCAP with zstd chunks)."""
    return ['ros2', 'bag', 'record', '--storage', 'mcap',
            '--storage-preset-profile', 'zstd_fast',
            '--output', bag_dir, '--topics'] + list(topics or TOPICS)


def snapshot_logs(folder):
    """Map each .jsonl file in folder to its current size in bytes."""
    if not os.path.isdir(folder):
        return {}
    return {name: os.path.getsize(os.path.join(folder, name))
            for name in os.listdir(folder) if name.endswith('.jsonl')}


def new_log_lines(folder, before):
    """Return the complete lines added to folder since snapshot_logs."""
    lines = []
    for name in sorted(snapshot_logs(folder)):
        with open(os.path.join(folder, name), 'rb') as fh:
            fh.seek(before.get(name, 0))
            data = fh.read()
        # A line still being written has no newline yet: leave it out.
        for raw in data.split(b'\n')[:-1]:
            if raw.strip():
                lines.append(raw.decode('utf-8', 'replace'))
    return lines


def mark_line(t, label):
    """One marks.jsonl line: an operator mark at wall-clock time t."""
    return json.dumps({'t': round(t, 3), 'label': label.strip()})
