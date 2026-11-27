"""Parameter sweep of the behavior over the pure-Python sim (no ROS).

Every number produced here is from the simulation only. The sim robot has no
body, the only obstacles are the walls and the noise values are guesses, so
the results are suggestions that need confirmation on the real robot.
"""
import itertools
import math
import random

from trash_vision import arm_logic as al
from trash_vision import behavior_logic as bl
from trash_vision import safety_logic as sfl
from trash_vision import sim_logic as sl
from trash_vision import sim_mission as sm
from trash_vision import tuning_logic as tl

SIM_ONLY = ('SIM ONLY: suggestions from the simulation, not measurements. '
            'Confirm on the real robot before changing config/robot.yaml.')

# name -> candidate values tried by the random search (and the names that may
# be swept at all). Safety limits and calibration values are left out on
# purpose. The values are working choices around the defaults.
SPACE = {
    'approach_speed': (0.1, 0.15, 0.2, 0.25, 0.3),
    'turn_gain': (1.0, 1.5, 2.0, 2.5, 3.0),
    'max_turn_speed': (0.5, 0.65, 0.8, 1.0),
    'pick_center_tolerance': (0.05, 0.08, 0.12, 0.16),
    'target_lost_timeout': (0.8, 1.5, 2.5),
    'stop_distance': (0.2, 0.25, 0.3, 0.35),
    'slow_distance': (0.4, 0.5, 0.6, 0.8),
    'avoid_turn_speed': (0.4, 0.6, 0.8),
    'search_speed': (0.15, 0.2, 0.25, 0.3),
    'search_turn_speed': (0.3, 0.5, 0.7),
    'search_row_overlap': (0.1, 0.2, 0.4),
}


def baseline_params(config_params=None):
    """Current values of the sweepable parameters.

    config_params is the behavior section of robot.yaml (or None); anything
    missing there comes from the code defaults.
    """
    config_params = config_params or {}
    return {n: float(config_params.get(n, bl.DEFAULTS[n])) for n in SPACE}


def safety_params(config_params=None):
    """Safety stage settings from the behavior section (never swept)."""
    config_params = config_params or {}
    return {n: config_params.get(n, v) for n, v in sfl.DEFAULTS.items()}


def setting_errors(overrides, base, safety=None):
    """List why a setting (changes on top of base) must not be tried."""
    errors = []
    for name, value in overrides.items():
        if name not in SPACE:
            errors.append('%s cannot be swept' % name)
        elif not math.isfinite(value) or value <= 0:
            errors.append('%s must be a positive number' % name)
    if errors:
        return errors
    current = dict(safety or sfl.DEFAULTS)
    current.update(base)
    # Same limits as live tuning on the robot, where a limit exists.
    return tl.validate_changes(
        {n: v for n, v in overrides.items() if n in tl.TUNABLES}, current)


def grid_settings(axes, base, safety=None, max_settings=200):
    """Every combination of axes {name: [values]}; invalid ones are dropped.

    Raises ValueError for a name or value that is never allowed, a grid larger
    than max_settings, or a grid with no valid combination.
    """
    for name, values in axes.items():
        for value in values:
            # Check one value at a time; pair rules are checked below.
            if name not in SPACE or not math.isfinite(value) or value <= 0:
                raise ValueError(setting_errors({name: value}, base)[0])
    size = math.prod(len(v) for v in axes.values())
    if size > max_settings:
        raise ValueError('grid has %d settings, more than %d; use fewer '
                         'values or --random' % (size, max_settings))
    names = list(axes)
    out = []
    for combo in itertools.product(*(axes[n] for n in names)):
        overrides = dict(zip(names, combo))
        if not setting_errors(overrides, base, safety):
            out.append(overrides)
    if not out:
        raise ValueError('no valid setting in the grid (check limits, and '
                         'slow_distance above stop_distance)')
    return out


def random_settings(count, seed, base, safety=None, names=None):
    """Draw up to count different valid settings from SPACE (seeded)."""
    rng = random.Random(seed)
    names = list(names or SPACE)
    out, seen = [], set()
    for _ in range(count * 50):
        if len(out) >= count:
            break
        overrides = {n: rng.choice(SPACE[n]) for n in names}
        key = tuple(overrides[n] for n in names)
        if key in seen or setting_errors(overrides, base, safety):
            continue
        seen.add(key)
        out.append(overrides)
    return out


def evaluate(params, seeds, duration=300.0, litter=None, safety=None):
    """Run one setting once per seed and pool the results.

    pickup_rate: items deposited / items placed, over all seeds.
    time_per_item: simulated seconds of all runs / items deposited (a missed
    item costs the whole run duration); inf when nothing was collected.
    safety_stops: mean front hard stops of the safety stage per run.
    """
    litter = sm.TYPED_LITTER if litter is None else litter
    if not seeds:
        raise ValueError('need at least one seed')
    total = collected = stops = 0
    seconds = 0.0
    for seed in seeds:
        result = sm.run_mission(
            litter, duration=duration, params=dict(params),
            noise=sl.SimNoise(seed=seed), arm=al.PickConfig(),
            safety=sfl.SafetyStage(safety))
        total += result['total']
        collected += result['collected']
        stops += result['safety_stops']
        seconds += result['duration']
    return {
        'pickup_rate': collected / total if total else 0.0,
        'time_per_item': seconds / collected if collected else math.inf,
        'safety_stops': stops / len(seeds),
    }


def rank(rows):
    """Best first: pickup rate, then fewer safety stops, then less time."""
    return sorted(rows, key=lambda r: (
        -r['pickup_rate'], r['safety_stops'], r['time_per_item']))


def sweep(settings, base, seeds, duration=300.0, litter=None, safety=None,
          progress=None):
    """Evaluate the baseline and each setting on the same seeds; ranked.

    Each row has 'changes' (only values that differ from base; empty for the
    baseline), 'baseline' and the numbers from evaluate().
    """
    todo = [{}]
    for overrides in settings:
        changes = {n: v for n, v in overrides.items() if v != base[n]}
        if changes not in todo:
            todo.append(changes)
    rows = []
    for index, changes in enumerate(todo):
        row = evaluate(dict(base, **changes), seeds, duration, litter, safety)
        row.update(changes=changes, baseline=not changes)
        rows.append(row)
        if progress:
            progress(index + 1, len(todo))
    return rank(rows)


def describe(row):
    """Short text of what a row changes."""
    if row['baseline']:
        return 'current values (baseline)'
    return ' '.join('%s=%g' % item for item in sorted(row['changes'].items()))


def format_table(rows, top=10):
    """Ranked text table; the baseline row is always shown."""
    lines = [SIM_ONLY, '',
             'rank  pickup  s/item  stops/run  change from current values']
    for number, row in enumerate(rows, 1):
        if number > top and not row['baseline']:
            continue
        per_item = ('%6.1f' % row['time_per_item']
                    if math.isfinite(row['time_per_item']) else '     -')
        lines.append('%4d  %5.1f%%  %s  %9.2f  %s' % (
            number, 100.0 * row['pickup_rate'], per_item,
            row['safety_stops'], describe(row)))
    lines += ['', SIM_ONLY]
    return '\n'.join(lines)


def to_csv(rows, base):
    """All rows as CSV text with the full values of every setting."""
    names = list(SPACE)
    lines = ['# ' + SIM_ONLY, ','.join(
        ['rank', 'baseline', 'pickup_rate', 'time_per_item_s',
         'safety_stops_per_run'] + names)]
    for number, row in enumerate(rows, 1):
        values = dict(base, **row['changes'])
        lines.append(','.join(
            [str(number), str(int(row['baseline'])),
             '%.4f' % row['pickup_rate'], '%.2f' % row['time_per_item'],
             '%.2f' % row['safety_stops']]
            + ['%g' % values[n] for n in names]))
    return '\n'.join(lines) + '\n'
