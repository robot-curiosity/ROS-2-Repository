import math
import struct

from trash_vision import gps_imu_logic as gl


def _sentence(body):
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    return "$%s*%02X" % (body, calc)


GGA = _sentence("GPGGA,123519,3404.7040,N,11815.0000,W,1,08,0.9,100.0,M,,M,,")


def test_checksum():
    assert gl.nmea_checksum_ok(GGA)
    assert not gl.nmea_checksum_ok(GGA[:-2] + "00")
    assert not gl.nmea_checksum_ok("garbage")


def test_parse_gga():
    fix = gl.parse_gga(GGA)
    assert abs(fix["lat"] - (34 + 4.704 / 60)) < 1e-9
    assert abs(fix["lon"] + (118 + 15.0 / 60)) < 1e-9
    assert fix["alt"] == 100.0
    assert fix["satellites"] == 8
    assert fix["hdop"] == 0.9


def test_parse_gga_no_fix_and_bad():
    nofix = _sentence("GPGGA,123519,,,,,0,00,99.9,,M,,M,,")
    assert gl.parse_gga(nofix) is None
    assert gl.parse_gga(_sentence("GPRMC,1,2,3")) is None
    assert gl.parse_gga("$GPGGA,bad*00") is None


def test_south_hemisphere_negative():
    s = _sentence("GPGGA,1,3000.0,S,01000.0,E,1,05,1.0,5.0,M,,M,,")
    fix = gl.parse_gga(s)
    assert fix["lat"] == -30.0
    assert fix["lon"] == 10.0


def test_fix_status():
    assert gl.fix_status(1) == gl.STATUS_FIX
    assert gl.fix_status(2) == gl.STATUS_SBAS_FIX
    assert gl.fix_status(0) == gl.STATUS_NO_FIX


def test_covariance_grows_with_hdop():
    low = gl.position_covariance(1.0)
    high = gl.position_covariance(4.0)
    assert high[0] > low[0] and high[8] > high[0]
    assert len(low) == 9


def test_euler_quaternion_yaw_90():
    x, y, z, w = gl.euler_to_quaternion(0, 0, math.pi / 2)
    assert abs(z - math.sin(math.pi / 4)) < 1e-9
    assert abs(w - math.cos(math.pi / 4)) < 1e-9
    assert abs(x) < 1e-9 and abs(y) < 1e-9


def test_normalize_quaternion():
    assert gl.normalize_quaternion((0, 0, 0, 0)) is None
    q = gl.normalize_quaternion((0, 0, 0, 2))
    assert q == (0.0, 0.0, 0.0, 1.0)


def test_bno055_decode():
    data = struct.pack("<hhhh", 16384, 0, 0, 0)
    assert gl.bno055_decode_quaternion(data) == (0.0, 0.0, 0.0, 1.0)
    assert gl.bno055_decode_quaternion(bytes(8)) is None
    gyr = gl.bno055_decode_vec3(
        struct.pack("<hhh", 16, 0, -16), gl.BNO055_GYR_SCALE)
    assert abs(gyr[0] - math.radians(1.0)) < 1e-9
    assert abs(gyr[2] + math.radians(1.0)) < 1e-9
    acc = gl.bno055_decode_vec3(
        struct.pack("<hhh", 0, 0, 981), gl.BNO055_ACC_SCALE)
    assert abs(acc[2] - 9.81) < 1e-9
