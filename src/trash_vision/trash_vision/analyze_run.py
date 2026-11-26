"""Console script: analyze a run recorded with record_trial.

  ros2 run trash_vision analyze_run ~/curiosity_runs/<run>
  ros2 run trash_vision analyze_run <run> --export   # also write events.jsonl

--export needs ROS (to read the bag); the exported file works on any PC.
"""
import argparse
import json
import os
import sys

from trash_vision import analysis_logic as al
from trash_vision import behavior_logic as bl
from trash_vision import run_data as rd


def main(argv=None):
    """Print the report for one run."""
    parser = argparse.ArgumentParser(
        description='Stopping distance, sonar delay, loop rates and dropped '
        'frames from a recorded run.')
    parser.add_argument('run', help='run folder or events.jsonl')
    parser.add_argument('--export', action='store_true',
                        help='write events.jsonl into the run folder')
    parser.add_argument('--json', action='store_true',
                        help='print the numbers as JSON')
    parser.add_argument('--stop-distance', type=float,
                        help='default: the value recorded with the run')
    args = parser.parse_args(argv)
    path = os.path.expanduser(args.run)
    try:
        events = rd.load_events(path)
    except (OSError, ImportError, RuntimeError) as exc:
        print('cannot read run: %s' % exc)
        return 1
    if not events:
        print('the recording is empty')
        return 1
    folder = rd.run_folder(path)
    if args.export:
        rd.write_events(os.path.join(folder, rd.EVENTS_FILE), events)
    behavior = rd.load_params(
        os.path.join(folder, 'params_behavior.yaml'), 'behavior')
    sonar = rd.load_params(
        os.path.join(folder, 'params_ultrasonic.yaml'), 'ultrasonic')
    stop = args.stop_distance
    if stop is None:
        stop = behavior.get('stop_distance', bl.DEFAULTS['stop_distance'])
    result = al.analyze(
        events, rd.read_jsonl(os.path.join(folder, 'marks.jsonl')),
        stop_distance=float(stop),
        median_window=sonar.get('median_window', 3))
    print(json.dumps(result, indent=2) if args.json
          else al.format_report(result))
    return 0


def run():
    """Entry point for the console script."""
    sys.exit(main())


if __name__ == '__main__':
    run()
