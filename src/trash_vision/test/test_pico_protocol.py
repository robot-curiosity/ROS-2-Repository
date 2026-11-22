"""Tests for the Pico line protocol and the file copy in the package."""
import os

import pytest

from trash_vision import pico_protocol as pp

import pico_fakes  # noqa: F401  (adds the firmware folder to sys.path)

HERE = os.path.dirname(__file__)


def test_encode_decode_round_trip():
    """Check: a line decodes back to its name and numbers."""
    line = pp.cmd_velocity(0.25, -0.1)
    assert line.startswith('V 0.25 -0.1*')
    name, args = pp.decode(line)
    assert name == 'V' and args == [0.25, -0.1]


def test_ints_stay_ints_floats_stay_floats():
    """Check: number types survive the round trip."""
    name, args = pp.decode(pp.msg_telemetry(5, -3, 7, 0.5, -0.5, 3))
    assert name == 'T'
    assert args[:3] == [5, -3, 7] and args[5] == 3
    assert isinstance(args[0], int) and isinstance(args[3], float)


def test_bad_checksum_is_rejected():
    """Check: a changed character fails the checksum."""
    line = pp.cmd_velocity(0.2, 0.2)
    bad = line.replace('0.2', '0.3', 1)
    with pytest.raises(ValueError):
        pp.decode(bad)


def test_missing_or_garbled_checksum_is_rejected():
    """Check: lines without a valid checksum are refused."""
    for bad in ('V 0.1 0.1', 'V 0.1 0.1*ZZ', '*00', ''):
        with pytest.raises(ValueError):
            pp.decode(bad)


def test_trailing_newline_is_tolerated():
    """Check: CR LF at the end is ignored."""
    assert pp.decode(pp.cmd_ping() + '\r\n') == ('PING', [])


def test_timestamp_wraps_modulo():
    """Check: the millisecond stamp is sent modulo MS_MODULO."""
    _, args = pp.decode(pp.msg_telemetry(pp.MS_MODULO + 5, 0, 0, 0.0, 0.0, 0))
    assert args[0] == 5


def test_cfg_has_one_value_per_field():
    """Check: the CFG field list matches what the bridge sends."""
    from trash_vision import pico_bridge_logic as pbl
    from pico_fakes import PARAMS
    lines = pbl.build_setup_lines(PARAMS)
    name, args = pp.decode(lines[1])
    assert name == 'CFG' and len(args) == len(pp.CFG_FIELDS)
    name, args = pp.decode(lines[0])
    assert name == 'PINS' and args == PARAMS['pico_pins']


def test_firmware_copy_matches_package_copy():
    """Check: the ROS side protocol file is a copy of the firmware one."""
    firmware = os.path.join(pico_fakes.FIRMWARE, 'protocol.py')
    package = os.path.join(HERE, '..', 'trash_vision', 'pico_protocol.py')
    with open(firmware) as f1, open(package) as f2:
        assert f1.read() == f2.read()
