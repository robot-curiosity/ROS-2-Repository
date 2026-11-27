#!/usr/bin/env python3
"""Print a short progress summary: python3 tools/status.py [trial logs].

Needs only the repo (PyYAML for robot.yaml). Trial logs may be .jsonl files
or directories of them.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'trash_vision'))

from trash_vision import status_logic as sl  # noqa: E402
from trash_vision import trial_log as tl  # noqa: E402


def main(argv=None):
    """Print the status; extra args are trial log files or directories."""
    args = sys.argv[1:] if argv is None else argv
    with open(os.path.join(ROOT, 'docs', 'HARDWARE_TESTS.md'),
              encoding='utf-8') as fh:
        tests_text = fh.read()
    import yaml
    with open(os.path.join(ROOT, 'src', 'trash_vision', 'config',
                           'robot.yaml'), encoding='utf-8') as fh:
        config = yaml.safe_load(fh)
    records = None
    if args:
        paths = tl.find_logs(args)
        records = tl.read_records(paths) if paths else []
    print(sl.build_status(tests_text, config, records))
    return 0


if __name__ == '__main__':
    sys.exit(main())
