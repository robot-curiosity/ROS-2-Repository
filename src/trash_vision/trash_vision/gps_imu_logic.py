"""Pure-Python helpers for the GPS/IMU node (no ROS needed)."""

import math
import struct

# NavSatFix status values (sensor_msgs/NavSatStatus).
STATUS_NO_FIX = -1
STATUS_FIX = 0
STATUS_SBAS_FIX = 1

# Typical horizontal error (m) per NMEA fix quality, used for covariance.
_HDOP_BASE_ERROR_M = 2.5

# BNO055 register map and scales (datasheet BST-BNO055-DS000).
BNO055_ADDR = 0x28
BNO055_REG_OPR_MODE = 0x3D
BNO055_MODE_CONFIG = 0x00
BNO055_MODE_NDOF = 0x0C
BNO055_REG_ACC = 0x08
BNO055_REG_GYR = 0x14
BNO055_REG_QUAT = 0x20
BNO055_QUAT_SCALE = 1.0 / 16384.0
BNO055_GYR_SCALE = math.pi / 180.0 / 16.0
BNO055_ACC_SCALE = 1.0 / 100.0


def nmea_checksum_ok(sentence):
    """Return True if an NMEA sentence ('$....*HH') has a valid checksum."""
    sentence = sentence.strip()
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, _, given = sentence[1:].partition("*")
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(given[:2], 16)
    except ValueError:
        return False


def _dm_to_degrees(value, hemisphere):
    """Convert NMEA (d)ddmm.mmmm plus N/S/E/W to signed decimal degrees."""
    if not value:
        return None
    try:
        raw = float(value)
    except ValueError:
        return None
    degrees = int(raw // 100)
    minutes = raw - degrees * 100
    result = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        result = -result
    return result


def _to_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def parse_gga(sentence):
    """Parse a $xxGGA sentence into a dict, or None if invalid/no fix.

    Keys: lat, lon, alt (m), quality (int), satellites (int), hdop (float).
    """
    if not nmea_checksum_ok(sentence):
        return None
    fields = sentence.strip()[1:].split("*")[0].split(",")
    if len(fields) < 10 or not fields[0].endswith("GGA"):
        return None
    try:
        quality = int(fields[6] or 0)
    except ValueError:
        return None
    if quality == 0:
        return None
    lat = _dm_to_degrees(fields[2], fields[3])
    lon = _dm_to_degrees(fields[4], fields[5])
    alt = _to_float(fields[9])
    if lat is None or lon is None:
        return None
    try:
        sats = int(fields[7] or 0)
    except ValueError:
        sats = 0
    hdop = _to_float(fields[8])
    return {
        "lat": lat, "lon": lon,
        "alt": alt if alt is not None else 0.0,
        "quality": quality, "satellites": sats,
        "hdop": hdop if hdop is not None else 99.0,
    }


def fix_status(quality):
    """Map NMEA GGA quality to a NavSatStatus status value."""
    if quality in (2, 5):
        return STATUS_SBAS_FIX
    if quality >= 1:
        return STATUS_FIX
    return STATUS_NO_FIX


def position_covariance(hdop, base_error=_HDOP_BASE_ERROR_M):
    """Row-major 3x3 ENU covariance (m^2) estimated from HDOP."""
    horiz = (max(hdop, 0.5) * base_error) ** 2
    vert = (2.0 * max(hdop, 0.5) * base_error) ** 2
    return [horiz, 0.0, 0.0, 0.0, horiz, 0.0, 0.0, 0.0, vert]


def euler_to_quaternion(roll, pitch, yaw):
    """Return (x, y, z, w) for roll/pitch/yaw in radians."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def normalize_quaternion(q):
    """Return a unit quaternion, or None if the input has ~zero length."""
    norm = math.sqrt(sum(c * c for c in q))
    if norm < 1e-6:
        return None
    return tuple(c / norm for c in q)


def bno055_decode_quaternion(data):
    """Decode 8 bytes (w, x, y, z int16 LE) to a unit (x, y, z, w) or None."""
    w, x, y, z = struct.unpack("<hhhh", bytes(data[:8]))
    return normalize_quaternion(
        (x * BNO055_QUAT_SCALE, y * BNO055_QUAT_SCALE,
         z * BNO055_QUAT_SCALE, w * BNO055_QUAT_SCALE))


def bno055_decode_vec3(data, scale):
    """Decode 6 bytes (x, y, z int16 LE) times scale."""
    x, y, z = struct.unpack("<hhh", bytes(data[:6]))
    return (x * scale, y * scale, z * scale)


def diagonal_covariance(variance):
    """Row-major 3x3 diagonal covariance with the same variance."""
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]
