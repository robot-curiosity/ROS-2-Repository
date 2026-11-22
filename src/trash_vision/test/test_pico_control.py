"""Tests for the Pico wheel controller and the firmware app."""
import pico_fakes
from pico_fakes import FakeLink, PARAMS

from trash_vision import pico_bridge_logic as pbl
from trash_vision import pico_protocol as pp

import app  # firmware (path added by pico_fakes)
import control


def _cfg(**over):
    cfg = dict(zip(pp.CFG_FIELDS, [
        1000.0, 0.06, 0.5, 1.0, 0.0, 0.5, 1.0, 500, False, False, False,
        False]))
    cfg.update(over)
    return cfg


def test_decoder_counts_both_directions():
    """Check: quadrature decoder counts up and down."""
    dec = control.QuadratureDecoder(0, 0)
    for a, b in ((1, 0), (1, 1), (0, 1), (0, 0)):
        dec.update(a, b)
    assert dec.count == 4
    for a, b in ((0, 1), (1, 1), (1, 0), (0, 0)):
        dec.update(a, b)
    assert dec.count == 0


def test_slew_limits_step():
    """Check: slew moves at most max_rate * dt."""
    assert control.slew(0.0, 1.0, 1.0, 0.1) == 0.1
    assert control.slew(0.0, -1.0, 1.0, 0.1) == -0.1
    assert control.slew(0.95, 1.0, 1.0, 0.1) == 1.0


def test_wheel_limit_keeps_the_arc():
    """Check: both wheels scale together."""
    left, right = control.limit_wheel_speeds(0.2, 1.0, 0.5)
    assert right == 0.5 and abs(left - 0.1) < 1e-9


def test_unconfigured_controller_outputs_zero():
    """Check: nothing moves before CFG arrives."""
    ctrl = control.WheelController()
    ctrl.set_target(0.3, 0.3, 0)
    assert ctrl.update(0, 0, 20) == [(0.0, True), (0.0, True)]


def test_controller_reaches_target_speed_on_plant():
    """Check: closed loop settles near the commanded speed."""
    hw = pico_fakes.FakeHardware()
    ctrl = control.WheelController()
    ctrl.configure(_cfg())
    now = 0
    ctrl.set_target(0.3, 0.3, now)
    for _ in range(300):
        now += 20
        ctrl.set_target(0.3, 0.3, now)
        out = ctrl.update(*hw.read_counts(), now)
        for side in (0, 1):
            hw.set_motor(side, *out[side])
        hw.advance(0.02)
    assert abs(hw.speed[0] - 0.3) < 0.03
    assert abs(ctrl.speeds[1] - 0.3) < 0.05


def test_watchdog_stops_wheels_when_commands_stop():
    """Check: no command for the timeout zeroes the target and flags it."""
    link = FakeLink()
    for line in pbl.build_setup_lines(PARAMS):
        link.write(line)
    link.write(pp.cmd_velocity(0.3, 0.3))
    link.run(1.0, on_step=lambda: None)
    assert link.pico.ctrl.watchdog
    assert abs(link.hw.speed[0]) < 0.02
    flags = pp.decode(link.drain()[-1])[1][5]
    assert flags & pp.FLAG_WATCHDOG


def test_new_command_clears_watchdog():
    """Check: a fresh V line re-arms the wheels."""
    link = FakeLink()
    for line in pbl.build_setup_lines(PARAMS):
        link.write(line)
    link.run(1.0)
    link.write(pp.cmd_velocity(0.2, 0.2))
    assert not link.pico.ctrl.watchdog


def test_velocity_before_setup_is_refused():
    """Check: V without PINS/CFG gives NOT_CONFIGURED and no motion."""
    link = FakeLink()
    link.drain()
    link.write(pp.cmd_velocity(0.3, 0.3))
    replies = [pp.decode(x) for x in link.drain()]
    assert ('ERR', ['NOT_CONFIGURED']) in replies
    link.run(0.2)
    assert link.hw.duty == [0.0, 0.0]
    flags = pp.decode(link.drain()[-1])[1][5]
    assert flags & pp.FLAG_UNCONFIGURED


def test_pin_checks():
    """Check: bad or duplicate pins are refused with a clear code."""
    assert app.pins_error([2, 3, 4, 5, 6, 7, 8, 9]) is None
    assert app.pins_error([2, 3, 4]) == 'BAD_PINS'
    assert app.pins_error([-1] * 8) == 'BAD_PINS'
    assert app.pins_error([2, 3, 4, 5, 6, 7, 8, 23]) == 'BAD_PINS'
    assert app.pins_error([2, 2, 4, 5, 6, 7, 8, 9]) == 'PIN_CONFLICT'


def test_pin_setup_failure_sets_fault_flag():
    """Check: a GPIO error is reported and flagged."""
    link = FakeLink()
    link.hw.fail_pins = True
    link.drain()
    link.write(pp.cmd_pins([2, 3, 4, 5, 6, 7, 8, 9]))
    assert ('ERR', ['PIN_SETUP_FAILED']) in [
        pp.decode(x) for x in link.drain()]
    assert link.pico.flags() & pp.FLAG_FAULT


def test_corrupted_and_unknown_lines_get_errors():
    """Check: the app answers bad input and keeps running."""
    link = FakeLink()
    link.drain()
    link.write('V 0.1 0.1*00')
    link.write(pp.encode('FOO', 1))
    codes = [pp.decode(x)[1][0] for x in link.drain()]
    assert codes == ['BAD_LINE', 'UNKNOWN_CMD']


def test_bad_cfg_values_refused():
    """Check: zero ticks_per_rev is not accepted."""
    link = FakeLink()
    link.drain()
    values = [0.0, 0.06, 0.5, 1.0, 0.0, 0.5, 1.0, 500, 0, 0, 0, 0]
    link.write(pp.cmd_cfg(*values))
    assert ('ERR', ['BAD_CFG']) in [pp.decode(x) for x in link.drain()]
    assert not link.pico.ctrl.configured


def test_stop_command_zeroes_target():
    """Check: STOP takes effect on the next step."""
    link = FakeLink()
    for line in pbl.build_setup_lines(PARAMS):
        link.write(line)
    link.write(pp.cmd_velocity(0.3, 0.3))
    link.write(pp.cmd_stop())
    assert link.pico.ctrl.target == [0.0, 0.0]


def test_encoder_inversion_flips_ticks_and_motor_direction():
    """Check: invert flags change the reported ticks and DIR output."""
    ctrl = control.WheelController()
    ctrl.configure(_cfg(invert_enc_left=True, invert_left=True))
    ctrl.set_target(0.2, 0.2, 0)
    ctrl.update(0, 0, 20)
    out = ctrl.update(1, 1, 40)
    assert ctrl.ticks == [-1, 1]
    assert out[0][1] is False and out[1][1] is True
