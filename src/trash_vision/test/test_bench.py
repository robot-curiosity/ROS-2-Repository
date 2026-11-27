"""Tests for the bench test CLI logic and its safety rules."""
import copy
import os
import subprocess
import sys

import pytest
import yaml

from trash_vision import bench
from trash_vision import bench_logic as bl

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


class Keys:
    """Scripted key source: one entry per read, then ''."""

    def __init__(self, *script):
        self.script = list(script)

    def read(self, timeout=0.0):
        item = self.script.pop(0) if self.script else ''
        if isinstance(item, BaseException):
            raise item
        return item


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Motors:
    def __init__(self):
        self.duty = {'left': 0.0, 'right': 0.0}
        self.history = []

    def set(self, side, duty, forward):
        self.history.append((side, duty, forward))
        self.duty[side] = duty

    def off(self):
        self.duty = {'left': 0.0, 'right': 0.0}

    def counts(self, side):
        return None


class Arm:
    def __init__(self, pose):
        self.pose = list(pose)
        self.writes = []

    def read(self):
        return list(self.pose)

    def write(self, pose):
        self.writes.append(list(pose))
        self.pose = list(pose)


LIMITS = [(-10.0, 10.0)] * 6
NAMES = ['a', 'b', 'c', 'd', 'e', 'f']


def silent(text):
    pass


def run_motors(keys, motors=None):
    motors = motors or Motors()
    clock = Clock()
    result = bl.motor_test(motors, 0.2, {'right': True}, keys, silent,
                           clock, clock.sleep)
    return result, motors, clock


def test_motors_run_one_at_a_time_after_each_confirmation():
    # drain, answer, then the polls of a 2 s run return ''.
    keys = Keys('', 'y', *[''] * 101, '', 'y')
    result, motors, clock = run_motors(keys)
    assert result == bl.DONE
    assert motors.history == [('left', 0.2, True), ('right', 0.2, False)]
    assert motors.duty == {'left': 0.0, 'right': 0.0}
    assert clock.now == pytest.approx(4.0, abs=0.1)


def test_motor_not_run_without_yes():
    keys = Keys('', 'n', '', '\n')
    result, motors, _ = run_motors(keys)
    assert result == bl.SKIP
    assert motors.history == []


def test_stale_yes_typed_before_the_question_is_discarded():
    keys = Keys('y', 'n', 'y', 'n')
    result, motors, _ = run_motors(keys)
    assert motors.history == []


@pytest.mark.parametrize('stop', [' ', 'x ', KeyboardInterrupt(),
                                  RuntimeError('boom')])
def test_motors_zero_on_estop_interrupt_and_error(stop):
    motors = Motors()
    with pytest.raises((bl.EStop, KeyboardInterrupt, RuntimeError)):
        run_motors(Keys('', 'y', '', '', '', stop), motors)
    assert motors.history == [('left', 0.2, True)]
    assert motors.duty == {'left': 0.0, 'right': 0.0}


def test_bench_duty_is_capped_and_validated():
    assert bl.bench_duty(0.2) == 0.2
    assert bl.bench_duty(0.9) == bl.MAX_DUTY
    assert bl.bench_duty(0.1, 0.25) == 0.25
    assert bl.bench_duty(0.1, 0.8) == bl.MAX_DUTY
    for bad in (0.0, -0.2, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            bl.bench_duty(bad)


def test_jog_moves_one_joint_and_respects_limits():
    assert bl.jog_pose([0.0] * 6, 1, 2.0, LIMITS) == [0, 2, 0, 0, 0, 0]
    assert bl.jog_pose([9.0] * 6, 0, 2.0, LIMITS)[0] == 10.0
    # Outside the limits: no jump to the limit, and never further out.
    assert bl.jog_pose([30.0] * 6, 0, -2.0, LIMITS)[0] == 28.0
    assert bl.jog_pose([30.0] * 6, 0, 2.0, LIMITS)[0] == 30.0
    assert bl.jog_pose([30.0] * 6, 1, -2.0, LIMITS)[0] == 30.0
    with pytest.raises(ValueError):
        bl.jog_pose([float('nan')] * 6, 0, 2.0, LIMITS)


def test_arm_jog_uses_newest_key_only_and_quits():
    arm = Arm([0.0] * 6)
    result = bl.arm_jog(arm, NAMES, LIMITS, 50.0,
                        Keys('2', '++++', '-', 'q'), silent)
    assert result == bl.DONE
    assert arm.pose == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert arm.writes[0][1] == bl.MAX_JOG_STEP  # one step for four '+'


def test_arm_estop_holds_and_does_not_move():
    arm = Arm([1.0] * 6)
    with pytest.raises(bl.EStop):
        bl.arm_jog(arm, NAMES, LIMITS, 2.0, Keys('+ '), silent)
    assert arm.writes == [[1.0] * 6]


def test_sonar_readout_reports_silent_and_failed_sensors():
    clock = Clock()
    lines = []

    def ping(name):
        if name == 'b':
            raise OSError('gpio')
        return (0.5, 'ok') if name == 'a' else (4.0, 'no_echo')

    result = bl.sonar_readout(['a', 'b', 'c'], ping, Keys(), lines.append,
                              clock, clock.sleep, seconds=0.1)
    text = '\n'.join(lines)
    assert result == bl.FAIL
    assert '0.50 m' in text and 'ERROR' in text and 'no echo' in text
    assert 'Never heard an echo: b, c' in text


def test_sonar_readout_quit_and_estop():
    clock = Clock()
    ok = bl.sonar_readout(['a'], lambda n: (1.0, 'ok'), Keys('', 'q'),
                          silent, clock, clock.sleep)
    assert ok == bl.DONE
    with pytest.raises(bl.EStop):
        bl.sonar_readout(['a'], lambda n: (1.0, 'ok'), Keys(' '),
                         silent, clock, clock.sleep)


def test_preview_and_detection_text():
    art = bl.ascii_preview([[0] * 8 for _ in range(4)],
                           [(0.25, 0.25, 0.75, 0.99)])
    rows = art.split('\n')
    assert len(rows) == 4 and rows[1] == '  OOOOO '
    assert bl.ascii_preview([[255]]) == '@'
    text = bl.describe_detections(
        [('can', 0.9, (0.4, 0.5, 0.6, 0.8))], lambda u, v: (0.45, -0.02))
    assert 'floor (0.50, 0.80)' in text and 'base_x 0.45' in text
    assert bl.describe_detections([], None) == 'no detections'


def test_estop_ends_the_whole_run():
    ran = []

    def stop():
        raise bl.EStop()

    code = bl.run_steps(
        [('one', stop), ('two', lambda: ran.append(1))], silent)
    assert code == 130 and ran == []


def test_error_in_one_step_does_not_stop_the_others():
    ran = []

    def broken():
        raise OSError('busy')

    def fine():
        ran.append(1)
        return bl.DONE

    assert bl.run_steps([('a', broken), ('b', fine)], silent) == 1
    assert ran == [1]


def _config():
    with open(os.path.join(PKG, 'config', 'robot.yaml')) as f:
        return yaml.safe_load(f)


def test_drive_refused_without_wheels_off():
    assert bench.main(['drive']) == 2
    args = bench.build_parser().parse_args(['all'])
    assert bench.bench_drive(_config(), args, Keys()) == bl.FAIL


def test_unset_hardware_is_skipped_without_opening_anything():
    config = _config()
    args = bench.build_parser().parse_args(['all', '--wheels-off'])
    assert bench.bench_drive(config, args, Keys()) == bl.SKIP
    assert bench.bench_arm(config, args, Keys()) == bl.SKIP


def test_drive_pin_errors():
    config = copy.deepcopy(_config())
    params = config['drive']['ros__parameters']
    params.update(pwm_pins=[12, 13], dir_pins=[5, 6])
    assert bench.drive_pin_errors(config) == []
    params['dir_pins'] = [12, 6]
    assert 'used more than once' in bench.drive_pin_errors(config)[0]
    params['dir_pins'] = [-1, 6]
    assert bench.drive_pin_errors(config)


def test_help_works_without_ros():
    r = subprocess.run(
        [sys.executable, '-m', 'trash_vision.bench', '--help'],
        cwd=PKG, capture_output=True, text=True, timeout=60)
    assert r.returncode == 0 and 'E-stop' in r.stdout, r.stderr[-300:]
