"""Regression tests for review pass 2 (drive, sonar, Pico)."""
import math

import pico_fakes  # noqa: F401  (adds the firmware folder to sys.path)

from trash_vision import drive_logic as dl
from trash_vision import pico_bridge_logic as pbl
from trash_vision import pico_protocol as pp
from trash_vision import sonar_scheduler as ss

import app  # firmware
import control


class _Hw:
    def __init__(self):
        self.counts = [0, 0]
        self.motors = {}

    def configure_pins(self, pins):
        self.counts = [0, 0]

    def read_counts(self):
        return tuple(self.counts)

    def set_motor(self, side, duty, forward):
        self.motors[side] = (duty, forward)

    def stop_all(self):
        pass


def _ready_app():
    pico = app.PicoApp(_Hw())
    pico.handle_line(pp.cmd_pins([0, 1, 2, 3, 4, 5, 6, 7]), 0)
    cfg = [1000.0, 0.06, 0.5, 1.0, 0.0, 0.5, 1.0, 500,
           False, False, False, False]
    pico.handle_line(pp.cmd_cfg(*cfg), 0)
    return pico


def test_review_nan_twist_becomes_a_stop():
    """Check: NaN or inf in a twist never reaches the wheel targets."""
    assert dl.sanitize_twist(float('nan'), 0.0) == (0.0, 0.0)
    assert dl.sanitize_twist(0.3, math.inf) == (0.0, 0.0)
    assert dl.sanitize_twist(None, 0.0) == (0.0, 0.0)
    assert dl.sanitize_twist(0.3, 0.1) == (0.3, 0.1)


def test_review_nan_target_does_not_ramp_up():
    """Check: slew and command_to_pwm treat NaN as zero, not full speed."""
    assert dl.slew(0.0, float('nan'), 1.0, 0.02) == 0.0
    assert dl.command_to_pwm(float('nan')) == (0.0, True)


def test_review_bridge_sends_zero_for_nan_cmd_vel():
    """Check: the Pico bridge sends a zero V line for a NaN cmd_vel."""
    sent = []
    params = dict(pico_fakes.PARAMS)
    session = pbl.BridgeSession(sent.append, params)
    session.send_velocity(float('nan'), 0.0)
    name, args = pp.decode(sent[-1])
    assert name == 'V' and args == [0, 0]


def test_review_pico_rejects_nan_velocity():
    """Check: a V line with nan stops the wheels and is answered BAD_ARGS."""
    pico = _ready_app()
    assert pico.handle_line(pp.cmd_velocity(0.2, 0.2), 10) == []
    assert pico.ctrl.target == [0.2, 0.2]
    reply = pico.handle_line(pp.cmd_velocity(float('nan'), 0.2), 20)
    assert pp.decode(reply[0]) == ('ERR', ['BAD_ARGS'])
    assert pico.ctrl.target == [0.0, 0.0]
    for t in range(20, 400, 20):
        pico.poll(t)
    assert all(abs(x) < 1e-9 for x in pico.ctrl.applied)


def test_review_pico_rejects_nan_config():
    """Check: a CFG with nan is refused and the controller stays unconfigured."""
    pico = app.PicoApp(_Hw())
    pico.handle_line(pp.cmd_pins([0, 1, 2, 3, 4, 5, 6, 7]), 0)
    cfg = [1000.0, 0.06, float('nan'), 1.0, 0.0, 0.5, 1.0, 500,
           False, False, False, False]
    reply = pico.handle_line(pp.cmd_cfg(*cfg), 0)
    assert pp.decode(reply[0]) == ('ERR', ['BAD_CFG'])
    assert not pico.ctrl.configured


def test_review_new_pins_do_not_cause_a_tick_jump():
    """Check: re-sending PINS (decoders restart at 0) adds no phantom ticks."""
    pico = _ready_app()
    pico.hw.counts = [5000, 5000]
    pico.poll(20)
    pico.poll(40)
    before = list(pico.ctrl.ticks)
    pico.handle_line(pp.cmd_pins([0, 1, 2, 3, 4, 5, 6, 7]), 50)
    pico.poll(60)
    pico.poll(80)
    assert pico.ctrl.ticks == before
    assert all(abs(s) < 1e-9 for s in pico.ctrl.speeds)


def test_review_reset_encoders_exists_on_controller():
    """Check: the controller forgets raw counts on reset_encoders."""
    ctrl = control.WheelController()
    ctrl.reset_encoders()
    assert ctrl._last_raw is None


def test_review_gpio_error_is_not_a_clear_reading():
    """Check: a ping that raises is reported as failed, not as no echo."""
    def boom():
        raise OSError('gpio gone')
    assert ss.try_ping(boom) == (False, None)
    assert ss.try_ping(lambda: 0.001) == (True, 0.001)
    assert ss.try_ping(lambda: None) == (True, None)
