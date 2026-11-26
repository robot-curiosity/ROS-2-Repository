#!/usr/bin/env python3
"""Record hardware test results without editing markdown by hand.

python3 tools/record_result.py next
python3 tools/record_result.py pass <n> "note"
python3 tools/record_result.py fail <n> "note"

Only the Python standard library is needed.
"""
import argparse
import datetime
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'trash_vision'))

from trash_vision import record_result_logic as rr  # noqa: E402
from trash_vision import status_logic as sl  # noqa: E402

DEFAULT_FILE = os.path.join(ROOT, 'docs', 'HARDWARE_TESTS.md')


def _write(path, text):
    """Replace the file in one step so a crash cannot truncate it."""
    folder = os.path.dirname(os.path.abspath(path))
    handle, temp = tempfile.mkstemp(dir=folder, suffix='.tmp')
    with os.fdopen(handle, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)
    shutil.copymode(path, temp)
    os.replace(temp, path)


def main(argv=None):
    """Run one command; returns the process exit code."""
    parser = argparse.ArgumentParser(
        description='Show or record hardware test results.')
    parser.add_argument('--file', default=DEFAULT_FILE,
                        help='test table (default docs/HARDWARE_TESTS.md)')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('next', help='print the next untested row')
    for name in ('pass', 'fail'):
        cmd = sub.add_parser(name, help='mark test n as %s' % name.upper())
        cmd.add_argument('number', type=int)
        cmd.add_argument('note', nargs='?', default='')
        cmd.add_argument('--force', action='store_true',
                         help='replace an existing result')
    args = parser.parse_args(argv)

    try:
        with open(args.file, encoding='utf-8', newline='') as fh:
            text = fh.read()
        if args.command == 'next':
            tests = rr.list_tests(text)
            test = rr.next_untested(tests)
            if test is None:
                print('No untested rows (%d tests).' % len(tests))
            else:
                counts = sl.count_tests(sl.parse_hardware_tests(text))
                print(rr.format_test(test, len(tests), counts))
            return 0
        date = datetime.date.today().isoformat()
        new_text = rr.set_result(text, args.number, args.command, date,
                                 args.note, args.force)
        _write(args.file, new_text)
        test = rr.list_tests(new_text)[args.number - 1]
        print('Test %d (%s): %s' % (args.number, test['change'],
                                    test['result']))
        return 0
    except (OSError, ValueError) as error:
        print('Error: %s' % error, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
