"""Tests for the hardware test result helper."""
import os
import subprocess
import sys

import pytest
from trash_vision import record_result_logic as rr
from trash_vision import status_logic as sl

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    '..', '..', '..'))
TOOL = os.path.join(ROOT, 'tools', 'record_result.py')

SAMPLE = (
    '# Hardware Test Queue\n'
    '\n'
    '| Added | Change (PR) | What to test | Result |\n'
    '|-------|-------------|--------------|--------|\n'
    '| 2026-11-13 | first | run `a` | PASS 2026-11-14 |\n'
    '| 2026-11-14 | second | `topic list | grep x` (1) look (2) wait | |\n'
    '| 2026-11-15 | third | spin wheel |  |\n'
)


def _run(path, *args):
    return subprocess.run([sys.executable, TOOL, '--file', str(path)]
                          + list(args), capture_output=True, text=True)


def test_list_and_next_untested():
    tests = rr.list_tests(SAMPLE)
    assert [t['number'] for t in tests] == [1, 2, 3]
    test = rr.next_untested(tests)
    assert test['number'] == 2
    assert test['steps'] == '`topic list | grep x` (1) look (2) wait'
    assert '\n(2) wait' in rr.format_test(test, 3)


def test_set_result_changes_only_that_cell():
    out = rr.set_result(SAMPLE, 2, 'fail', '2026-11-25', 'left motor | dead')
    before = SAMPLE.splitlines()
    after = out.splitlines()
    assert after[5] == (before[5][:-len(' |')]
                        + ' FAIL 2026-11-25: left motor / dead |')
    assert after[:5] == before[:5]
    assert after[6:] == before[6:]
    statuses = [r['status'] for r in sl.parse_hardware_tests(out)]
    assert statuses == [sl.PASS, sl.FAIL, sl.NOT_RUN]


def test_multiline_note_and_crlf_keep_table_intact():
    crlf = SAMPLE.replace('\n', '\r\n')
    out = rr.set_result(crlf, 3, 'pass', '2026-11-25', 'ok\nboth\r\nsides\\')
    assert out.count('\r\n') == crlf.count('\r\n')
    assert out.endswith('| PASS 2026-11-25: ok both sides\\ |\r\n')
    assert rr.list_tests(out)[2]['result'].startswith('PASS')


def test_existing_result_needs_force():
    with pytest.raises(ValueError, match='--force'):
        rr.set_result(SAMPLE, 1, 'fail', '2026-11-25')
    out = rr.set_result(SAMPLE, 1, 'fail', '2026-11-25', force=True)
    assert rr.list_tests(out)[0]['result'] == 'FAIL 2026-11-25'


@pytest.mark.parametrize('number', [0, -1, 4])
def test_unknown_number_rejected(number):
    with pytest.raises(ValueError, match='no test'):
        rr.set_result(SAMPLE, number, 'pass', '2026-11-25')


def test_broken_row_rejected():
    with pytest.raises(ValueError, match='table row'):
        rr.list_tests('| 2026-11-13 | no result cell |\n')


def test_command_line(tmp_path):
    path = tmp_path / 'tests.md'
    path.write_text(SAMPLE, encoding='utf-8')
    shown = _run(path, 'next')
    assert shown.returncode == 0
    assert 'Test 2 of 3' in shown.stdout
    assert _run(path, 'pass', '2', 'worked').returncode == 0
    assert _run(path, 'fail', '3').returncode == 0
    assert 'No untested rows' in _run(path, 'next').stdout
    saved = path.read_text(encoding='utf-8')
    bad = _run(path, 'pass', '9', 'x')
    assert bad.returncode == 1
    assert 'no test 9' in bad.stderr
    assert path.read_text(encoding='utf-8') == saved
    assert os.listdir(tmp_path) == ['tests.md']


def test_real_table_parses_and_matches_status_count():
    with open(os.path.join(ROOT, 'docs', 'HARDWARE_TESTS.md'),
              encoding='utf-8') as fh:
        text = fh.read()
    assert len(rr.list_tests(text)) == len(sl.parse_hardware_tests(text))
