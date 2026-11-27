"""Console script: replay a recorded run through the behavior logic.

  ros2 run trash_vision replay_run <run>                 # summary
  ros2 run trash_vision replay_run <run> --at 12.5       # ticks near 12.5 s
  ros2 run trash_vision replay_run <run> --set stop_distance=0.35

Times for --at are seconds from the start of the recording. Nothing is sent
to the robot.
"""
import argparse
import os
import sys

import yaml

from trash_vision import behavior_logic as bl
from trash_vision import replay_logic as rp
from trash_vision import run_data as rd
from trash_vision import safety_logic as sfl


def parse_overrides(items):
    """Turn ['name=value'] into a dict, refusing unknown names."""
    out = {}
    for item in items:
        name, sep, value = item.partition('=')
        if not sep or (name not in bl.DEFAULTS and name not in sfl.DEFAULTS):
            raise ValueError('unknown parameter setting: %s' % item)
        out[name] = yaml.safe_load(value)
    return out


def main(argv=None):
    """Replay one run and print where it differs."""
    parser = argparse.ArgumentParser(
        description='Replay recorded sonar, detection and odometry data '
        'through the behavior logic.')
    parser.add_argument('run', help='run folder or events.jsonl')
    parser.add_argument('--params', help='yaml with behavior parameters '
                        '(default: params_behavior.yaml of the run)')
    parser.add_argument('--set', action='append', default=[],
                        metavar='NAME=VALUE', help='override a parameter')
    parser.add_argument('--at', type=float, help='show ticks around this time')
    parser.add_argument('--window', type=float, default=1.0,
                        help='seconds either side of --at')
    args = parser.parse_args(argv)
    path = os.path.expanduser(args.run)
    try:
        overrides = parse_overrides(args.set)
        events = rd.load_events(path)
    except (OSError, ImportError, RuntimeError, ValueError) as exc:
        print('cannot replay: %s' % exc)
        return 1
    params_file = args.params or os.path.join(
        rd.run_folder(path), 'params_behavior.yaml')
    params = rd.load_params(params_file, 'behavior')
    if not params:
        print('warning: no recorded parameters, using code defaults')
    params.update(overrides)
    ticks = rp.replay(events, params)
    if not ticks:
        print('no /behavior/state in the recording (was behavior enabled?)')
        return 1
    start = events[0]['t']
    shown = [dict(k, t=k['t'] - start) for k in ticks]
    if args.at is not None:
        for k in shown:
            if abs(k['t'] - args.at) <= args.window:
                print(rp.format_tick(k))
        return 0
    bad = rp.mismatches(shown)
    print('%d ticks replayed, %d differ from the recording'
          % (len(shown), len(bad)))
    previous = None
    for k in shown:
        if k['state'] != previous:
            print(rp.format_tick(k))
            previous = k['state']
    if bad:
        print('first differences:')
        for k in bad[:5]:
            print(rp.format_tick(k))
    return 0


def run():
    """Entry point for the console script."""
    sys.exit(main())


if __name__ == '__main__':
    run()
