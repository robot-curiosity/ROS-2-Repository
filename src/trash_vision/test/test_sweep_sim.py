import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from trash_vision import safety_logic as sfl  # noqa: E402
from trash_vision import sim_mission as sm  # noqa: E402
from trash_vision import sweep_logic as sw  # noqa: E402
from trash_vision import sweep_sim  # noqa: E402

LITTER = sm.TYPED_LITTER[:3]


def _sweep(seeds):
    base = sw.baseline_params()
    settings = sw.random_settings(2, 7, base)
    return sw.sweep(settings, base, seeds, duration=40.0, litter=LITTER)


def test_sweep_is_deterministic_for_a_seed():
    assert _sweep([3, 4]) == _sweep([3, 4])


def test_random_settings_repeat_for_a_seed_and_are_valid():
    base = sw.baseline_params()
    first = sw.random_settings(30, 1, base)
    assert first == sw.random_settings(30, 1, base)
    assert first != sw.random_settings(30, 2, base)
    assert len(first) == 30
    for setting in first:
        assert sw.setting_errors(setting, base) == []
        assert setting['slow_distance'] - setting['stop_distance'] > 0.04


def test_space_never_includes_safety_or_calibration_values():
    assert not [n for n in sw.SPACE if n.startswith('safety_')]
    assert 'pick_floor_y' not in sw.SPACE


def test_baseline_reads_config_and_falls_back_to_defaults():
    base = sw.baseline_params({'approach_speed': 0.15, 'enabled': True})
    assert base['approach_speed'] == 0.15
    assert base['turn_gain'] == 2.0
    assert set(base) == set(sw.SPACE)


def test_grid_builds_combinations_and_drops_invalid_pairs():
    base = sw.baseline_params()
    out = sw.grid_settings(
        {'stop_distance': [0.25, 0.5], 'slow_distance': [0.4, 0.6]}, base)
    assert {(s['stop_distance'], s['slow_distance']) for s in out} == {
        (0.25, 0.4), (0.25, 0.6), (0.5, 0.6)}


@pytest.mark.parametrize('axes', [
    {'safety_max_linear': [0.5]},
    {'turn_gain': [-1.0]},
    {'turn_gain': [math.nan]},
    {'approach_speed': [5.0]},          # outside the live tuning limits
    {'stop_distance': [0.7]},           # above slow_distance
    {'turn_gain': [float(i + 1) for i in range(201)]},
])
def test_grid_rejects_bad_input(axes):
    with pytest.raises(ValueError):
        sw.grid_settings(axes, sw.baseline_params())


def test_stop_distance_must_stay_above_hard_stop():
    base = sw.baseline_params()
    safety = sw.safety_params({'safety_hard_stop_distance': 0.2})
    assert sw.setting_errors({'stop_distance': 0.2}, base, safety)


def test_rank_orders_by_rate_then_stops_then_time():
    def row(rate, stops, time):
        return {'pickup_rate': rate, 'safety_stops': stops,
                'time_per_item': time}
    best, second, third, last = (row(0.9, 0, 30), row(0.9, 0, 40),
                                 row(0.9, 1, 10), row(0.5, 0, 5))
    assert sw.rank([last, third, second, best]) == [best, second, third, last]


def test_evaluate_pools_runs_and_handles_nothing_collected():
    base = sw.baseline_params()
    good = sw.evaluate(base, [1], duration=120.0, litter=LITTER)
    assert 0.0 < good['pickup_rate'] <= 1.0
    assert 0.0 < good['time_per_item'] <= 120.0
    none = sw.evaluate(base, [1], duration=1.0, litter=LITTER)
    assert none['pickup_rate'] == 0.0
    assert none['time_per_item'] == math.inf
    with pytest.raises(ValueError):
        sw.evaluate(base, [], litter=LITTER)


def test_mission_counts_each_hard_stop_episode_once():
    stage = sfl.SafetyStage()
    ticks = []

    def apply(now, linear, angular, front_min, sonars_ok=True):
        # Blocked on ticks 5-9 and 20-24: two episodes.
        ticks.append(now)
        blocked = 5 <= len(ticks) < 10 or 20 <= len(ticks) < 25
        stage.reason = 'front_hard_stop' if blocked else ''
        return 0.0, 0.0

    stage.apply = apply
    r = sm.run_mission([], duration=2.0, search_only=True, safety=stage)
    assert r['safety_stops'] == 2
    plain = sm.run_mission([], duration=1.0, search_only=True)
    assert plain['safety_stops'] == 0


def test_table_and_csv_are_labelled_sim_only_and_keep_baseline():
    base = sw.baseline_params()
    rows = []
    for i in range(4):
        rows.append({'pickup_rate': 1.0 - i / 10, 'time_per_item': 10.0,
                     'safety_stops': 0.0, 'baseline': i == 3,
                     'changes': {} if i == 3 else {'turn_gain': 1.0 + i / 2}})
    rows[2]['time_per_item'] = math.inf
    table = sw.format_table(rows, top=1)
    assert table.startswith('SIM ONLY') and table.count(sw.SIM_ONLY) == 2
    assert 'baseline' in table
    assert 'turn_gain=1.5' not in table
    csv = sw.to_csv(rows, base).splitlines()
    assert 'SIM ONLY' in csv[0]
    assert len(csv) == 2 + len(rows)
    assert len(csv[1].split(',')) == len(csv[2].split(','))


def test_cli_grid_writes_csv(tmp_path, capsys):
    config = tmp_path / 'robot.yaml'
    config.write_text('behavior:\n  ros__parameters:\n    turn_gain: 1.5\n')
    out = tmp_path / 'rows.csv'
    code = sweep_sim.main([
        '--param', 'turn_gain=1.5,2.5', '--seeds', '1', '--duration', '5',
        '--config', str(config), '--csv', str(out)])
    assert code == 0
    printed = capsys.readouterr().out
    assert 'SIM ONLY' in printed and 'turn_gain=2.5' in printed
    # 1.5 equals the current value, so only the baseline and 2.5 are run.
    assert len(out.read_text().splitlines()) == 4


@pytest.mark.parametrize('argv', [
    ['--param', 'turn_gain'], ['--param', 'nope=1'], ['--seeds', '0'],
    ['--duration', '0'], ['--config', '/does/not/exist.yaml'],
])
def test_cli_rejects_bad_arguments(argv):
    with pytest.raises(SystemExit) as err:
        sweep_sim.main(argv)
    assert err.value.code == 2
