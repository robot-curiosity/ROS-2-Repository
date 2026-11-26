"""Tests for record_trial helpers, run analysis and replay on synthetic data."""
import json
import math
import os
from types import SimpleNamespace as NS

import pytest

from trash_vision import analysis_logic as al
from trash_vision import analyze_run
from trash_vision import behavior_logic as bl
from trash_vision import record_logic as rl
from trash_vision import record_trial
from trash_vision import replay_logic as rp
from trash_vision import replay_run
from trash_vision import run_data as rd
from trash_vision import safety_logic as sfl
from trash_vision import ultrasonic_logic as ul


def ev(t, topic, **data):
    return {'t': round(t, 4), 'topic': topic, 'data': data}


def odom(t, x, v):
    return ev(t, '/odom', x=x, y=0.0, yaw=0.0, v=v, w=0.0)


def stop_recording():
    """0.3 m/s, forced stop at 1.0 s, wheels ramp down at 0.5 m/s^2."""
    events, x = [], 0.0
    for i in range(200):
        t = i * 0.01
        v = 0.3 if t < 1.0 else max(0.0, 0.3 - 0.5 * (t - 1.0))
        x += v * 0.01
        events.append(odom(t, x, v))
        if i % 10 == 0:
            events.append(ev(t, '/cmd_vel', linear=0.3 if t < 1.0 else 0.0,
                             angular=0.0))
    events.append(ev(1.0, '/behavior/safety', data='front_hard_stop'))
    return events


def test_run_names_and_bag_command():
    assert rl.next_run_name('2026-12-30', []) == '2026-12-30_run01'
    used = ['2026-12-30_run01', '2026-12-30_run07', '2026-12-29_run09', 'x']
    assert rl.next_run_name('2026-12-30', used) == '2026-12-30_run08'
    cmd = rl.bag_command('/tmp/bag')
    assert cmd[:3] == ['ros2', 'bag', 'record'] and 'zstd_fast' in cmd
    assert len(rl.SONAR_TOPICS) == 8 and set(rl.SONAR_TOPICS) <= set(cmd)


def test_only_new_complete_trial_lines_are_copied(tmp_path):
    log = tmp_path / 'run_a.jsonl'
    log.write_text('{"old": 1}\n')
    before = rl.snapshot_logs(str(tmp_path))
    with log.open('a') as fh:
        fh.write('{"new": 2}\n{"half')
    (tmp_path / 'run_b.jsonl').write_text('{"new": 3}\n')
    assert rl.new_log_lines(str(tmp_path), before) == [
        '{"new": 2}', '{"new": 3}']


def test_record_trial_writes_run_folder(tmp_path, monkeypatch):
    trials = tmp_path / 'trials'
    trials.mkdir()

    class FakeProc:
        returncode = 0

        def __init__(self, cmd, **kwargs):
            (trials / 'run_x.jsonl').write_text('{"success": true}\n')

        def poll(self):
            return 0

    monkeypatch.setattr(record_trial.subprocess, 'Popen', FakeProc)
    monkeypatch.setattr(record_trial, '_dump_params', lambda *a: None)
    out = tmp_path / 'runs'
    assert record_trial.main(
        ['--out', str(out), '--trial-log-dir', str(trials)]) == 0
    run = next(out.iterdir())
    assert (run / 'trials.jsonl').read_text() == '{"success": true}\n'


def test_messages_reduce_to_event_data():
    q = NS(w=math.cos(0.5), x=0.0, y=0.0, z=math.sin(0.5))
    msg = NS(pose=NS(pose=NS(position=NS(x=1.0, y=2.0), orientation=q)),
             twist=NS(twist=NS(linear=NS(x=0.2), angular=NS(z=0.1))))
    data = rd.message_to_data('/odom', msg)
    assert data['yaw'] == pytest.approx(1.0) and data['v'] == 0.2
    assert rd.message_to_data('/ultrasonic/left_1', NS(range=0.5)) == {
        'range': 0.5}
    assert rd.message_to_data('/rosout', NS()) == {}


def test_stopping_distance_of_a_forced_stop():
    stops = al.stop_events(al.by_topic(
        sorted(stop_recording(), key=lambda e: e['t'])))
    assert len(stops) == 1
    # v^2 / (2 a) = 0.09 m, 0.6 s.
    assert stops[0]['distance_m'] == pytest.approx(0.09, abs=0.006)
    assert stops[0]['time_s'] == pytest.approx(0.6, abs=0.03)
    assert stops[0]['reason'] == 'front_hard_stop'
    assert stops[0]['start'] == stops[0]['zero']


def test_ramped_stop_starts_at_the_slow_down():
    speeds = [0.3, 0.3, 0.3, 0.2, 0.1, 0.0, 0.0]
    events = [ev(i * 0.1, '/cmd_vel', linear=v, angular=0.0)
              for i, v in enumerate(speeds)]
    events += [odom(i * 0.1, 0.0, v) for i, v in enumerate(speeds)]
    stops = al.stop_events(al.by_topic(sorted(events, key=lambda e: e['t'])))
    assert stops[0]['start'] == pytest.approx(0.3)
    assert stops[0]['zero'] == pytest.approx(0.5)


def test_stop_interrupted_by_driving_again_is_skipped():
    events = [ev(0.0, '/cmd_vel', linear=0.3, angular=0.0),
              ev(0.1, '/cmd_vel', linear=0.0, angular=0.0),
              ev(0.2, '/cmd_vel', linear=0.3, angular=0.0),
              odom(0.15, 0.0, 0.2), odom(0.3, 0.1, 0.0)]
    assert al.stop_events(al.by_topic(events)) == []


def test_loop_rate_and_dropped_frames():
    events = []
    for n in range(1, 31):
        if n in (10, 11):
            continue
        text = json.dumps({'detected': False, 'frame_number': n})
        events.append(ev(n * 0.1, '/vision/target', data=text))
    result = al.analyze(events)
    rate = result['rates']['/vision/target']
    assert rate['median_s'] == pytest.approx(0.1)
    assert rate['missed'] == 2 and rate['max_s'] == pytest.approx(0.3)
    assert result['frames'] == {'frames': 28, 'dropped': 2}


def test_sonar_delay_from_marks_and_period():
    events = [ev(i * 0.48, '/ultrasonic/front_1',
                 range=4.0 if i * 0.48 < 2.9 else 0.2) for i in range(12)]
    events.append(ev(3.4, '/cmd_vel', linear=0.0, angular=0.0))
    events.sort(key=lambda e: e['t'])
    res = al.sonar_delay(al.by_topic(events),
                         [{'t': 2.0, 'label': 'box'}, {'t': 4.0}], 0.25)
    first, second = res['marks']
    assert first['detect_s'] == pytest.approx(1.36)
    assert first['stop_s'] == pytest.approx(1.4)
    assert 'already blocked' in second['label']
    assert res['periods']['front_1']['max_delay_s'] == pytest.approx(0.96)


def synthetic_mission():
    """Record what the logic itself does for a can that comes into reach."""
    machine = bl.BehaviorMachine()
    safety = sfl.SafetyStage()
    events = []
    for i in range(60):
        t = 100.0 + i * 0.1
        for name in ul.SENSOR_NAMES:
            events.append(ev(t - 0.01, '/ultrasonic/' + name, range=4.0))
        target = {'detected': False}
        if i >= 10:
            target = {'detected': True, 'confidence': 0.9,
                      'horizontal_error': 0.0,
                      'floor_point_normalized': {
                          'x': 0.5, 'y': 0.5 if i < 30 else 0.9}}
        events.append(ev(t - 0.005, '/vision/target', data=json.dumps(target)))
        if machine.state == bl.PICK and i == 40:
            machine.arm_result(True, t, 'pick')
        cmd = machine.step(t, target, [4.0, 4.0], [4.0, 4.0],
                           [4.0, 4.0], [4.0, 4.0])
        safety.note_update(t)
        lin, ang = safety.apply(t, cmd.linear, cmd.angular, 4.0, True)
        events.append(ev(t, '/cmd_vel', linear=lin, angular=ang))
        events.append(ev(t + 0.001, '/behavior/state', data=machine.state))
    return events


def test_replay_reproduces_the_recorded_decisions():
    ticks = rp.replay(synthetic_mission())
    states = {k['state'] for k in ticks}
    assert {bl.SEARCH, bl.APPROACH, bl.PICK, bl.DEPOSIT} <= states
    assert rp.mismatches(ticks) == []


def test_replay_with_a_changed_parameter_differs():
    ticks = rp.replay(synthetic_mission(), {'approach_speed': 0.0})
    assert rp.mismatches(ticks, tolerance=0.05)


def test_overrides_refuse_unknown_names():
    assert replay_run.parse_overrides(['stop_distance=0.3']) == {
        'stop_distance': 0.3}
    with pytest.raises(ValueError):
        replay_run.parse_overrides(['stop_distanse=0.3'])


def test_command_line_tools_on_an_exported_run(tmp_path, capsys):
    rd.write_events(str(tmp_path / rd.EVENTS_FILE), synthetic_mission())
    (tmp_path / 'params_behavior.yaml').write_text(
        '/behavior:\n  ros__parameters:\n    stop_distance: 0.25\n')
    assert analyze_run.main([str(tmp_path)]) == 0
    assert 'Loop rates' in capsys.readouterr().out
    assert replay_run.main([str(tmp_path)]) == 0
    assert '0 differ' in capsys.readouterr().out
    assert replay_run.main([str(tmp_path), '--at', '3.0']) == 0
    assert 'cmd' in capsys.readouterr().out
    assert analyze_run.main([os.path.join(str(tmp_path), 'nothing')]) == 1
