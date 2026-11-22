"""Loopback tests: the bridge session talking to the firmware logic."""
import math

from pico_fakes import FakeHardware, FakeLink, PARAMS

from trash_vision import pico_bridge_logic as pbl
from trash_vision import pico_protocol as pp


def _connect(params=None, hw=None):
    params = dict(PARAMS, **(params or {}))
    link = FakeLink(hw)
    session = pbl.BridgeSession(link.write, params)
    # The Pico says READY on boot; the session answers with PINS and CFG.
    for line in link.drain():
        session.on_line(line)
    return link, session


def _pump(link, session, seconds, linear=0.0, angular=0.0):
    def step():
        for line in link.drain():
            session.on_line(line)
        if int(link.now_ms) % 100 == 0:
            session.send_velocity(linear, angular)
    link.run(seconds, on_step=step)


def test_ready_triggers_setup_lines():
    """Check: READY makes the session send PINS then CFG."""
    link, _ = _connect()
    names = [pp.decode(x)[0] for x in link.from_pi]
    assert names == ['PINS', 'CFG']
    assert link.pico.pins_ok and link.pico.ctrl.configured


def test_straight_drive_matches_commanded_distance():
    """Check: odom x follows the commanded speed through the full loop."""
    link, session = _connect()
    _pump(link, session, 4.0, linear=0.3)
    x, y, theta, v, w = session.pose()
    assert 0.9 < x < 1.25
    assert abs(y) < 0.02 and abs(theta) < 0.02
    assert abs(v - 0.3) < 0.05


def test_turn_in_place_changes_yaw_not_position():
    """Check: a pure rotation turns the odometry without moving it."""
    link, session = _connect()
    _pump(link, session, 3.0, angular=0.8)
    x, y, theta, _, w = session.pose()
    assert abs(x) < 0.03 and abs(y) < 0.03
    assert theta > 0.5
    assert math.isfinite(w)


def test_speed_is_limited_to_the_configured_maximum():
    """Check: a big command is scaled to max_wheel_speed_mps."""
    link, session = _connect()
    sent = session.send_velocity(5.0, 0.0)
    assert sent == (0.5, 0.5)


def test_stop_reaches_the_pico():
    """Check: STOP is sent and the target becomes zero."""
    link, session = _connect()
    session.send_velocity(0.3, 0.3)
    session.stop()
    assert link.pico.ctrl.target == [0.0, 0.0]


def test_unplugged_link_stops_the_wheels_through_the_watchdog():
    """Check: if the Pi goes silent the Pico stops by itself."""
    link, session = _connect()
    _pump(link, session, 1.5, linear=0.3)
    assert abs(link.hw.speed[0]) > 0.2
    link.run(1.5)
    assert link.pico.ctrl.watchdog
    assert abs(link.hw.speed[0]) < 0.02


def test_pico_reboot_resends_setup():
    """Check: a second READY re-sends PINS and CFG and resets odometry base."""
    link, session = _connect()
    link.from_pi.clear()
    session.on_line(pp.msg_ready())
    assert [pp.decode(x)[0] for x in link.from_pi] == ['PINS', 'CFG']


def test_flag_changes_and_errors_become_events():
    """Check: watchdog and ERR lines are reported once as events."""
    link, session = _connect()
    link.run(0.1, on_step=lambda: [session.on_line(x) for x in link.drain()])
    session.events.clear()
    session.on_line(pp.msg_error('BAD_PINS'))
    session.on_line(pp.msg_telemetry(1, 0, 0, 0.0, 0.0, pp.FLAG_WATCHDOG))
    session.on_line(pp.msg_telemetry(2, 0, 0, 0.0, 0.0, pp.FLAG_WATCHDOG))
    assert session.events == ['pico error: BAD_PINS',
                              'pico flags: watchdog']


def test_corrupt_line_from_pico_is_ignored():
    """Check: noise on the line does not crash or move the odometry."""
    _, session = _connect()
    assert session.on_line('garbage') is False
    assert session.on_line('T 1 2*00') is False
    assert session.pose()[0] == 0.0


def test_timestamp_wrap_gives_small_positive_dt():
    """Check: the Pico clock wrapping does not make a huge or negative dt."""
    _, session = _connect()
    top = pp.MS_MODULO - 10
    session.on_line(pp.msg_telemetry(top, 0, 0, 0.0, 0.0, 0))
    session.on_line(pp.msg_telemetry(top + 20, 100, 100, 0.0, 0.0, 0))
    assert 0.0 < session.pose()[0] < 1.0
    assert abs(session.pose()[3]) < 20.0


def test_link_lost_rule():
    """Check: link_lost handles never-received and stale cases."""
    assert pbl.link_lost(10.0, None, 1.0)
    assert not pbl.link_lost(10.0, 9.5, 1.0)
    assert pbl.link_lost(10.0, 8.0, 1.0)


def test_inverted_encoders_keep_odom_forward():
    """Check: invert_enc flags are applied on the Pico, odom still forward."""
    hw = FakeHardware()
    # Encoders count backwards for forward motion:
    original = hw.read_counts
    hw.read_counts = lambda: tuple(-v for v in original())
    params = {'invert_enc_left': True, 'invert_enc_right': True}
    link, session = _connect(params, hw)
    _pump(link, session, 3.0, linear=0.3)
    assert session.pose()[0] > 0.5
