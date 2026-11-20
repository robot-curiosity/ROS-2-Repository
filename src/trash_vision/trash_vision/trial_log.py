"""Pickup trial logging and summary (pure Python, no ROS).

TrialRecorder watches the BehaviorMachine state after every step and writes
one JSON object per pickup attempt (JSON Lines). An attempt starts when the
machine enters APPROACH from SEARCH and ends when it falls back to SEARCH or
RECOVER, or leaves DEPOSIT.

Record fields: run, t_start, t_end, detected (always true), approached (the
robot reached the pick position), picked (the arm grabbed it), success
(deposited in the bag), litter_type, confidence, failure_reason.
"""
import glob
import json
import os

APPROACH = 'APPROACH'
SEARCH = 'SEARCH'
PICK = 'PICK'
DEPOSIT = 'DEPOSIT'
STOW = 'STOW'
RECOVER = 'RECOVER'

UNKNOWN = 'unknown'


def litter_type(target):
    """Litter type from a /vision/target dict (detector gives none yet)."""
    if not isinstance(target, dict):
        return UNKNOWN
    for key in ('litter_type', 'class_name', 'label'):
        if target.get(key):
            return str(target[key])
    return UNKNOWN


class TrialRecorder:
    """Turn machine state transitions into attempt records."""

    def __init__(self, machine, writer, run_id='run'):
        self._machine = machine
        self._write = writer
        self._run = run_id
        self._prev = machine.state
        self._attempt = None
        self._items = machine.items_collected

    def _finish(self, now, success, reason):
        a = self._attempt
        self._attempt = None
        if a is None:
            return
        a['t_end'] = round(now, 3)
        a['success'] = bool(success)
        a['failure_reason'] = None if success else (reason or 'unknown')
        self._write(a)

    def update(self, now, target=None):
        """Call after every machine step; target is the latest detection."""
        m = self._machine
        new = m.state
        old = self._prev
        self._prev = new
        if new == old:
            return
        if new == APPROACH and old == SEARCH:
            conf = target.get('confidence') if isinstance(target, dict) \
                else None
            self._attempt = {
                'run': self._run, 't_start': round(now, 3), 't_end': None,
                'detected': True, 'approached': False, 'picked': False,
                'success': False, 'litter_type': litter_type(target),
                'confidence': conf, 'failure_reason': None}
        elif self._attempt is None:
            return
        elif old == APPROACH and new == PICK:
            self._attempt['approached'] = True
        elif old == APPROACH:
            self._finish(now, False, m.reason or 'target_lost')
        elif old == PICK and new == DEPOSIT:
            self._attempt['picked'] = True
        elif old == PICK:
            self._finish(now, False, m.reason or 'pick_failed')
        elif old == DEPOSIT:
            ok = m.items_collected > self._items
            self._finish(now, ok, m.reason or 'deposit_failed')
        self._items = m.items_collected


class JsonlWriter:
    """Append one JSON object per line, flushing every record."""

    def __init__(self, path):
        folder = os.path.dirname(path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        self.path = path
        self._fh = open(path, 'a', encoding='utf-8')

    def __call__(self, record):
        self._fh.write(json.dumps(record, sort_keys=True) + '\n')
        self._fh.flush()

    def close(self):
        self._fh.close()


def read_records(paths):
    """Read records from files, skipping blank or damaged lines."""
    records = []
    for path in paths:
        with open(path, encoding='utf-8') as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except ValueError:
                    continue
                if isinstance(rec, dict):
                    records.append(rec)
    return records


def _stats(records):
    n = len(records)
    ok = sum(1 for r in records if r.get('success'))
    reasons = {}
    for r in records:
        if not r.get('success'):
            key = r.get('failure_reason') or 'unknown'
            reasons[key] = reasons.get(key, 0) + 1
    appr = sum(1 for r in records if r.get('approached'))
    picked = sum(1 for r in records if r.get('picked'))
    return {'attempts': n, 'approached': appr, 'picked': picked,
            'successes': ok, 'rate': ok / n if n else 0.0,
            'failures': reasons}


def summarize(records):
    """Pickup rate overall, by litter type and by run."""
    by_type = {}
    by_run = {}
    for r in records:
        by_type.setdefault(r.get('litter_type') or UNKNOWN, []).append(r)
        by_run.setdefault(r.get('run') or 'run', []).append(r)
    return {'overall': _stats(records),
            'by_type': {k: _stats(v) for k, v in sorted(by_type.items())},
            'by_run': {k: _stats(v) for k, v in sorted(by_run.items())}}


def _row(name, s):
    fails = ', '.join('%s %d' % kv for kv in sorted(s['failures'].items()))
    return '%-20s %5d %5d %5d %5d %6.0f%%  %s' % (
        name[:20], s['attempts'], s['approached'], s['picked'],
        s['successes'], 100 * s['rate'], fails)


def format_summary(summary):
    head = '%-20s %5s %5s %5s %5s %7s  %s' % (
        '', 'tries', 'appr', 'pick', 'ok', 'rate', 'failures')
    lines = [head, _row('ALL', summary['overall']), '', 'By litter type:']
    lines += [_row(k, v) for k, v in summary['by_type'].items()]
    lines += ['', 'By run:']
    lines += [_row(k, v) for k, v in summary['by_run'].items()]
    return '\n'.join(lines)


def find_logs(args):
    """Expand files and directories into a sorted list of .jsonl files."""
    paths = []
    for a in args:
        if os.path.isdir(a):
            paths += glob.glob(os.path.join(a, '*.jsonl'))
        else:
            paths.append(a)
    return sorted(paths)
