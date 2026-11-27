"""Console script: ros2 run trash_vision hardware_check [subsystem ...].

Reads config/robot.yaml, prints PASS/FAIL/SKIP per subsystem, lists unset
values and pin problems. Needs no hardware.
"""
import argparse
import os
import sys

import yaml

from trash_vision import hardware_check_logic as hl


def default_config_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('trash_vision')
        return os.path.join(share, 'config', 'robot.yaml')
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, '..', 'config', 'robot.yaml')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('subsystems', nargs='*',
                        help='one or more of: ' + ', '.join(hl.CHECKS))
    parser.add_argument('--config', default=None,
                        help='path to robot.yaml (default: installed one)')
    args = parser.parse_args(argv)
    path = args.config or default_config_path()
    with open(path) as f:
        config = yaml.safe_load(f)
    try:
        results = hl.run_checks(config, args.subsystems)
    except ValueError as exc:
        print(exc)
        return 2
    print('config: %s' % path)
    print(hl.format_report(results))
    return hl.exit_code(results)


if __name__ == '__main__':
    sys.exit(main())
