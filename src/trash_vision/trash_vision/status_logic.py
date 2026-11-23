"""Pure-Python progress summary for Bryan (no ROS needed).

Reads docs/HARDWARE_TESTS.md, config/robot.yaml and optional trial logs and
formats a short plain-text status.
"""
from trash_vision import config_logic as cl
from trash_vision import trial_log as tl

PASS = 'pass'
FAIL = 'fail'
NOT_RUN = 'not run'
OTHER = 'other'

# Pickup target from the project notes (labeled default, not measured).
DEFAULT_TARGET_RATE = 0.80


def parse_hardware_tests(text):
    """Return a list of {'added', 'change', 'status'} for each table row.

    A row is a markdown line starting with "| 20" (a date). The Result is the
    last cell: PASS or FAIL (any case) at its start, empty means not run,
    anything else is counted as 'other' (notes without a verdict).
    """
    rows = []
    for line in (text or '').splitlines():
        line = line.strip()
        if not line.startswith('|'):
            continue
        body = line.strip('|')
        cells = [c.strip() for c in body.split('|')]
        if len(cells) < 4 or not cells[0][:2] == '20':
            continue
        result = cells[-1].strip('*_ ').upper()
        if result.startswith('PASS'):
            status = PASS
        elif result.startswith('FAIL'):
            status = FAIL
        elif result == '':
            status = NOT_RUN
        else:
            status = OTHER
        rows.append({'added': cells[0], 'change': cells[1],
                     'status': status})
    return rows


def count_tests(rows):
    """Counts per status plus the total."""
    counts = {PASS: 0, FAIL: 0, NOT_RUN: 0, OTHER: 0}
    for row in rows:
        counts[row['status']] += 1
    counts['total'] = len(rows)
    return counts


def unset_values(config):
    """{subsystem: [unset 'node.param', ...]} using the required params."""
    out = {}
    for subsystem, nodes in cl.SUBSYSTEMS.items():
        names = []
        for node in nodes:
            params = cl.node_params(config, node)
            names += ['%s.%s' % (node, n) for n in cl.find_unset(node, params)]
        out[subsystem] = names
    return out


def trial_status(records, target=DEFAULT_TARGET_RATE):
    """Overall trial stats and whether the rate meets the target."""
    stats = tl.summarize(records)['overall']
    return {'attempts': stats['attempts'], 'successes': stats['successes'],
            'rate': stats['rate'], 'target': target,
            'meets_target': stats['attempts'] > 0
            and stats['rate'] >= target}


def format_status(counts, unset, trials=None, failing=None):
    """Short plain-text status."""
    lines = ['Hardware tests: %d total, %d pass, %d fail, %d not yet run'
             % (counts['total'], counts[PASS], counts[FAIL],
                counts[NOT_RUN])]
    if counts[OTHER]:
        lines[0] += ', %d with notes only' % counts[OTHER]
    for name in failing or []:
        lines.append('  FAIL: %s' % name)
    total_unset = sum(len(v) for v in unset.values())
    lines.append('robot.yaml unset required values: %d' % total_unset)
    for subsystem, names in unset.items():
        if names:
            lines.append('  %s: %s' % (subsystem, ', '.join(names)))
        else:
            lines.append('  %s: all set' % subsystem)
    if trials is None:
        lines.append('Trials: no log given')
    elif trials['attempts'] == 0:
        lines.append('Trials: no attempts recorded')
    else:
        lines.append('Trials: %d/%d picked up (%.0f%%), target %.0f%% (%s)'
                     % (trials['successes'], trials['attempts'],
                        100 * trials['rate'], 100 * trials['target'],
                        'met' if trials['meets_target'] else 'not met'))
    return '\n'.join(lines)


def build_status(tests_text, config, records=None):
    """Combine everything into the status text."""
    rows = parse_hardware_tests(tests_text)
    failing = [r['change'] for r in rows if r['status'] == FAIL]
    trials = trial_status(records) if records is not None else None
    return format_status(count_tests(rows), unset_values(config), trials,
                         failing)
