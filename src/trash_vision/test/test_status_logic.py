import json
import os
import subprocess
import sys

from trash_vision import status_logic as sl

TABLE = """| Added | Change | Test | Result |
|---|---|---|---|
| 2026-11-13 | [agent] A | do x | PASS fine |
| 2026-11-14 | [agent] B | do y | FAIL motor stuck |
| 2026-11-15 | [agent] C | do z |  |
| 2026-11-16 | [agent] D | do w | **pass** |
| 2026-11-17 | [agent] E | do v | looked odd |
"""

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


def test_parse_and_count():
    rows = sl.parse_hardware_tests(TABLE)
    assert [r['status'] for r in rows] == [
        sl.PASS, sl.FAIL, sl.NOT_RUN, sl.PASS, sl.OTHER]
    counts = sl.count_tests(rows)
    assert counts['total'] == 5 and counts[sl.PASS] == 2
    assert counts[sl.FAIL] == 1 and counts[sl.NOT_RUN] == 1


def test_parse_empty():
    assert sl.parse_hardware_tests('') == []
    assert sl.parse_hardware_tests(None) == []


def test_unset_values():
    config = {'drive': {'ros__parameters': {
        'pwm_pins': [5, 6], 'dir_pins': [7, 8], 'enc_a_pins': [9, 10],
        'enc_b_pins': [11, 12], 'ticks_per_rev': 0.0,
        'wheel_radius_m': 0.06, 'track_width_m': 0.48}}}
    unset = sl.unset_values(config)
    assert unset['drive'] == ['drive.ticks_per_rev']
    assert 'ultrasonic.trig_pins' in unset['sonar']


def test_trial_status():
    recs = [{'success': True}] * 4 + [{'success': False}]
    t = sl.trial_status(recs)
    assert t['attempts'] == 5 and t['successes'] == 4
    assert t['meets_target']
    assert not sl.trial_status(recs[:2] + [{'success': False}] * 3)[
        'meets_target']
    assert not sl.trial_status([])['meets_target']


def test_build_status_text():
    text = sl.build_status(TABLE, {}, [{'success': True}])
    assert '5 total, 2 pass, 1 fail, 1 not yet run' in text
    assert '1 with notes only' in text
    assert 'FAIL: [agent] B' in text
    assert 'Trials: 1/1 picked up (100%)' in text
    assert 'Trials: no log given' in sl.build_status(TABLE, {})


def test_script_runs_on_repo(tmp_path):
    log = tmp_path / 'run_1.jsonl'
    log.write_text(json.dumps({'success': True, 'run': 'r'}) + '\n')
    out = subprocess.run(
        [sys.executable, os.path.join(ROOT, 'tools', 'status.py'),
         str(tmp_path)], capture_output=True, text=True, check=True).stdout
    assert 'Hardware tests:' in out
    assert 'robot.yaml unset required values' in out
    assert 'Trials: 1/1' in out
