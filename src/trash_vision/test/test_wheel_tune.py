"""Tests of the wheel tuner on synthetic responses with known answers."""
import math
import os
import random
import subprocess
import sys

import pytest
from trash_vision import wheel_tune_logic as wl

PARAMS = {'kp': 0.0, 'ki': 0.0, 'max_wheel_speed_mps': 0.5,
          'max_accel_mps2': 1.0, 'track_width_m': 0.4}


def _first_order(final, tau, n=200, dt=0.01):
    return [(i * dt, final * (1.0 - math.exp(-i * dt / tau)))
            for i in range(n)]


def _open_loop(gain, tau, theta, target=0.2, noise=0.0, seed=1):
    model = {'K': gain, 'tau': tau, 'theta': theta}
    rng = random.Random(seed)
    u_final = target / 0.5
    return [(0.02 * i, model['K'] * wl.ramp_response(
        0.02 * i - theta, tau, u_final, 2.0) + rng.gauss(0.0, noise))
        for i in range(100)]


def test_first_order_metrics_match_theory():
    m = wl.step_metrics(_first_order(0.18, 0.2), 0.2)
    assert m['final'] == pytest.approx(0.18, abs=1e-3)
    assert m['rise_time_s'] == pytest.approx(0.2 * math.log(9), abs=0.01)
    assert m['settling_time_s'] == pytest.approx(0.2 * math.log(20), abs=0.02)
    assert m['overshoot_pct'] == pytest.approx(0.0, abs=0.1)
    assert m['ss_error'] == pytest.approx(0.02, abs=1e-3)
    assert m['ss_error_pct'] == pytest.approx(10.0, abs=0.5)


def test_second_order_overshoot_matches_theory():
    zeta, wn = 0.5, 10.0
    wd = wn * math.sqrt(1 - zeta ** 2)
    data = [(t, 0.2 * (1 - math.exp(-zeta * wn * t) * (
        math.cos(wd * t) + zeta / math.sqrt(1 - zeta ** 2) *
        math.sin(wd * t)))) for t in (0.01 * i for i in range(300))]
    m = wl.step_metrics(data, 0.2)
    expected = 100 * math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2))
    assert m['overshoot_pct'] == pytest.approx(expected, abs=0.3)


def test_unsettled_and_dead_records_give_none():
    ramp = [(0.01 * i, 0.001 * i) for i in range(200)]
    assert wl.step_metrics(ramp, 2.0)['settling_time_s'] is None
    flat = wl.step_metrics([(0.01 * i, 0.0) for i in range(50)], 0.2)
    assert flat['rise_time_s'] is None
    assert flat['ss_error_pct'] == pytest.approx(100.0)
    with pytest.raises(ValueError):
        wl.step_metrics([(0.0, 0.0)], 0.2)


def test_fit_recovers_known_model_with_noise():
    fit = wl.fit_model(_open_loop(0.45, 0.15, 0.06, noise=0.002), 0.2,
                       0.5, 1.0)
    assert fit['K'] == pytest.approx(0.45, rel=0.03)
    assert fit['tau'] == pytest.approx(0.15, rel=0.15)
    assert fit['theta'] == pytest.approx(0.06, abs=0.02)
    assert fit['r2'] > 0.98


def test_simc_formula_and_prediction():
    model = {'K': 0.5, 'tau': 0.2, 'theta': 0.04}
    g = wl.simc_pi(model)
    assert g['tau_c'] == pytest.approx(0.1)
    assert g['kp'] == pytest.approx(0.2 / (0.5 * 0.14))
    assert g['ki'] == pytest.approx(g['kp'] / 0.2)
    pred = wl.step_metrics(wl.simulate(
        model, g['kp'], g['ki'], 0.2, 0.5, 1.0, 3.0), 0.2)
    assert pred['ss_error_pct'] == pytest.approx(0.0, abs=1.0)
    assert pred['overshoot_pct'] > 15.0  # feed-forward adds to SIMC
    safer = wl.suggest_gains(model, 0.2, 0.5, 1.0)
    assert safer['tau_c'] > g['tau_c'] and safer['kp'] < g['kp']
    run = wl.simulate(model, safer['kp'], safer['ki'], 0.2, 0.5, 1.0, 3.0)
    assert max(y for _, y in run) <= 0.22


def test_suggestions_only_for_open_loop_and_good_fit():
    data = _open_loop(0.45, 0.15, 0.04)
    assert wl.analyze_side(data, 0.2, PARAMS)['gains']
    closed = wl.analyze_side(data, 0.2, dict(PARAMS, kp=0.5))
    assert closed['gains'] is None and 'kp: 0.0' in closed['note']
    dead = wl.analyze_side([(0.02 * i, 0.0) for i in range(100)], 0.2, PARAMS)
    assert dead['gains'] is None and 'no suggestion' in dead['note']


def test_side_twist_round_trip():
    for side in wl.SIDES:
        v, w = wl.side_twist(side, 0.2, 0.4)
        other = [s for s in wl.SIDES if s != side][0]
        assert wl.side_speed(side, v, w, 0.4) == pytest.approx(0.2)
        assert wl.side_speed(other, v, w, 0.4) == pytest.approx(0.0)
    assert wl.step_speed(2.0, 0.5) == wl.SPEED_CAP_MPS
    assert wl.step_speed(0.2, 0.1) == 0.1
    with pytest.raises(ValueError):
        wl.step_speed(float('nan'), 0.5)


class _Bench:
    """Fake robot: odometry every 20 ms, response chosen by `wheel`."""

    def __init__(self, wheel):
        self.wheel = wheel
        self.t = 100.0
        self.start = None
        self.sent = []
        self.pending = []

    def send(self, linear, angular):
        self.sent.append((linear, angular))
        if self.start is None:
            self.start = self.t

    def sleep(self, seconds):
        for _ in range(int(round(seconds / 0.02))):
            self.t += 0.02
            y = self.wheel(self.t - (self.start or self.t))
            if y is not None:
                # left wheel only: v = y/2, w = -y/track
                self.pending.append((self.t, y / 2.0, -y / 0.4))

    def read(self):
        out, self.pending = self.pending, []
        return out

    def run(self):
        return wl.run_step('left', 0.2, 0.4, self.send, self.read,
                           lambda: self.t, self.sleep)


def test_run_step_records_and_stops():
    bench = _Bench(lambda t: 0.2 * (1 - math.exp(-t / 0.1)))
    samples, reason = bench.run()
    assert reason is None
    assert len(samples) > 90
    assert samples[-1][1] == pytest.approx(0.2, abs=0.001)
    assert bench.sent[-1] == (0.0, 0.0)


@pytest.mark.parametrize('wheel, word', [
    (lambda t: -0.2, 'BACKWARDS'),
    (lambda t: 0.6, 'runaway'),
    (lambda t: 0.0, 'no motion'),
    (lambda t: None, 'no odometry'),
    (lambda t: float('nan'), 'not a number'),
])
def test_run_step_aborts_on_faults(wheel, word):
    bench = _Bench(wheel)
    _, reason = bench.run()
    assert word in reason
    assert bench.t - bench.start < 1.3
    assert bench.sent[-1] == (0.0, 0.0)


def test_stop_is_sent_when_interrupted():
    bench = _Bench(lambda t: 0.2)

    def boom(seconds):
        raise KeyboardInterrupt

    bench.sleep = boom
    with pytest.raises(KeyboardInterrupt):
        bench.run()
    assert bench.sent[-1] == (0.0, 0.0)


def test_csv_round_trip_report_and_svg():
    records = {'left': (0.2, _open_loop(0.45, 0.15, 0.04)),
               'right': (0.2, _open_loop(0.40, 0.2, 0.04))}
    back, params = wl.from_csv(wl.to_csv(records, PARAMS))
    assert params == PARAMS
    assert len(back['right'][1]) == 100
    results = {s: wl.analyze_side(d, t, params) for s, (t, d) in back.items()}
    report = wl.format_report(results, params)
    assert report.count('NOT VERIFIED') == 2
    assert 'lower of each' in report
    svg = wl.to_svg(back)
    assert svg.count('<polyline') == 2 and svg.endswith('</svg>\n')
    with pytest.raises(ValueError):
        wl.from_csv('side,t_s\n')


def test_tool_analyzes_a_csv_without_ros(tmp_path):
    root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    tool = os.path.join(root, 'tools', 'tune_wheels.py')
    if not os.path.exists(tool):
        pytest.skip('repo tools folder not present')
    path = tmp_path / 'run.csv'
    path.write_text(wl.to_csv(
        {'left': (0.2, _open_loop(0.45, 0.15, 0.04))}, PARAMS))
    done = subprocess.run(
        [sys.executable, tool, '--analyze', str(path)],
        capture_output=True, text=True)
    assert done.returncode == 0, done.stderr
    assert 'SIMC PI SUGGESTION' in done.stdout
