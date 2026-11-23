"""Check docs/TRIAL_PLAN.md stays in step with the trial log fields."""
import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
FIELDS = ('run', 't_start', 't_end', 'detected', 'approached', 'picked',
          'success', 'litter_type', 'confidence', 'failure_reason')


def _plan():
    with open(os.path.join(ROOT, 'docs', 'TRIAL_PLAN.md')) as f:
        return f.read()


def test_plan_lists_every_trial_log_field():
    text = _plan()
    for field in FIELDS:
        assert field in text, field


def test_plan_keeps_todo_markers_and_target():
    text = _plan()
    assert 'TODO: Bryan to fill in' in text
    assert '80 percent' in text


def test_plan_commands_exist():
    text = _plan()
    assert 'trial_summary' in text and 'tools/status.py' in text
    assert os.path.exists(os.path.join(ROOT, 'tools', 'status.py'))
