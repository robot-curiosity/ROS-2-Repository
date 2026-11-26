"""Pure-Python editing of the docs/HARDWARE_TESTS.md Result column.

Tests are numbered 1, 2, 3... in table order (rows starting with "| 20").
New tests are appended at the bottom, so a number stays valid. Only the
Result cell of the chosen row is rewritten; all other text is kept as is.
"""
import re

from trash_vision import status_logic as sl

# A pipe that separates cells (markdown writes a literal pipe as "\|").
_PIPE = re.compile(r'(?<!\\)\|')


def _split_row(line):
    """Split a table row into (text before the result, result, line end)."""
    body = line.rstrip()
    end = line[len(body):]
    pipes = [m.start() for m in _PIPE.finditer(body)]
    if len(pipes) < 5 or pipes[-1] != len(body) - 1:
        raise ValueError('table row is not "| date | change | steps | '
                         'result |": %s' % body[:80])
    return body[:pipes[-2] + 1], body[pipes[-2] + 1:pipes[-1]], end


def list_tests(text):
    """Return every test row as a dict, numbered from 1."""
    tests = []
    for index, line in enumerate(text.splitlines(keepends=True)):
        if not line.startswith('| 20'):
            continue
        head, result, _ = _split_row(line.rstrip('\r\n'))
        cells = _PIPE.split(head.strip()[1:-1], maxsplit=2)
        added, change, steps = [c.strip().replace('\\|', '|') for c in cells]
        tests.append({'number': len(tests) + 1, 'line': index,
                      'added': added, 'change': change, 'steps': steps,
                      'result': result.strip()})
    return tests


def next_untested(tests):
    """First test with an empty Result, or None."""
    for test in tests:
        if not test['result']:
            return test
    return None


def format_result(verdict, date, note=''):
    """Build the Result cell text; newlines and pipes are made table-safe."""
    verdict = verdict.upper()
    if verdict not in ('PASS', 'FAIL'):
        raise ValueError('verdict must be PASS or FAIL')
    # status_logic splits on every pipe, so replace rather than escape.
    note = ' '.join((note or '').split()).replace('|', '/')
    text = '%s %s' % (verdict, date)
    return '%s: %s' % (text, note) if note else text


def set_result(text, number, verdict, date, note='', force=False):
    """Return text with the Result of test `number` filled in.

    Raises ValueError for an unknown number, or if the test already has a
    result and force is False.
    """
    tests = list_tests(text)
    if not 1 <= number <= len(tests):
        raise ValueError('no test %d (there are %d)' % (number, len(tests)))
    test = tests[number - 1]
    if test['result'] and not force:
        raise ValueError('test %d already has a result "%s"; add --force to '
                         'replace it' % (number, test['result']))
    lines = text.splitlines(keepends=True)
    head, _, end = _split_row(lines[test['line']])
    lines[test['line']] = '%s %s |%s' % (
        head, format_result(verdict, date, note), end)
    return ''.join(lines)


def format_test(test, total, counts=None):
    """Plain-text description of one test for the terminal."""
    steps = re.sub(r'\s+(?=\(\d\) )', '\n', test['steps'])
    lines = ['Test %d of %d (added %s)' % (test['number'], total,
                                           test['added']),
             test['change'], '', steps, '']
    if counts:
        lines.append('%d not yet run' % counts[sl.NOT_RUN])
    lines.append('Report: python3 tools/record_result.py pass %d "note"  '
                 '(or fail)' % test['number'])
    return '\n'.join(lines)
