"""Console script: record one run (ros2 bag, parameters, trial log, marks).

  ros2 run trash_vision record_trial [--out ~/curiosity_runs]

Start the robot nodes first. While recording, press Enter at the moment you
put an obstacle in front of the robot (used to measure sonar detection
delay); text typed before Enter is saved as the mark label. Ctrl-C stops.
"""
import argparse
import datetime
import os
import select
import signal
import subprocess
import sys
import time

import yaml

from trash_vision import record_logic as rl

DEFAULT_TRIAL_DIR = '~/curiosity_trials'


def _dump_params(node, path):
    """Save `ros2 param dump <node>` to path; return the parsed dict or None."""
    try:
        out = subprocess.run(['ros2', 'param', 'dump', node], timeout=15,
                             capture_output=True, text=True, check=True).stdout
        data = yaml.safe_load(out)
    except (OSError, subprocess.SubprocessError, yaml.YAMLError):
        data = None
        out = ''
    if not isinstance(data, dict):
        print('warning: could not read parameters of %s (not running?)' % node)
        return None
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(out)
    return data


def _trial_dir(params, override):
    if override:
        return os.path.expanduser(override)
    try:
        value = params['/behavior']['ros__parameters']['trial_log_dir']
    except (TypeError, KeyError):
        value = DEFAULT_TRIAL_DIR
    return os.path.expanduser(str(value)) if value else ''


def _wait(proc, marks_path):
    """Collect marks from the keyboard until Ctrl-C or the recorder exits."""
    read_keys = True
    while proc.poll() is None:
        if not read_keys:
            time.sleep(0.2)
            continue
        if not select.select([sys.stdin], [], [], 0.2)[0]:
            continue
        text = sys.stdin.readline()
        if not text:
            read_keys = False
            continue
        with open(marks_path, 'a', encoding='utf-8') as fh:
            fh.write(rl.mark_line(time.time(), text) + '\n')
        print('mark saved')


def main(argv=None):
    """Record until Ctrl-C and write the run folder."""
    parser = argparse.ArgumentParser(
        description='Record one run: ros2 bag, parameters, trial log, marks.')
    parser.add_argument('--out', default=rl.DEFAULT_ROOT,
                        help='folder for run folders (default %(default)s)')
    parser.add_argument('--trial-log-dir', default='',
                        help="behavior node's trial_log_dir (default: ask it)")
    args = parser.parse_args(argv)
    root = os.path.expanduser(args.out)
    os.makedirs(root, exist_ok=True)
    name = rl.next_run_name(datetime.date.today().isoformat(),
                            os.listdir(root))
    run = os.path.join(root, name)
    os.makedirs(run)
    params = _dump_params('/behavior', os.path.join(run, 'params_behavior.yaml'))
    _dump_params('/ultrasonic', os.path.join(run, 'params_ultrasonic.yaml'))
    trial_dir = _trial_dir(params, args.trial_log_dir)
    before = rl.snapshot_logs(trial_dir) if trial_dir else {}
    try:
        proc = subprocess.Popen(rl.bag_command(os.path.join(run, 'bag')),
                                stdin=subprocess.DEVNULL,
                                start_new_session=True)
    except OSError as exc:
        print('cannot start ros2 bag (is ROS sourced?): %s' % exc)
        return 1
    print('recording to %s. Enter = mark, Ctrl-C = stop.' % run)
    try:
        _wait(proc, os.path.join(run, 'marks.jsonl'))
        failed = proc.returncode not in (None, 0)
    except KeyboardInterrupt:
        failed = False
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                proc.kill()
        lines = rl.new_log_lines(trial_dir, before) if trial_dir else []
        with open(os.path.join(run, 'trials.jsonl'), 'w',
                  encoding='utf-8') as fh:
            fh.writelines(line + '\n' for line in lines)
    if failed:
        print('ros2 bag record failed (exit %s)' % proc.returncode)
        return 1
    print('saved %s (%d pickup attempts)' % (run, len(lines)))
    print('next: ros2 run trash_vision analyze_run %s' % run)
    return 0


def run():
    """Entry point for the console script."""
    sys.exit(main())


if __name__ == '__main__':
    run()
