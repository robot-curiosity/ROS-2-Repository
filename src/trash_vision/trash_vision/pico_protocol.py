"""Line protocol between the Pi and the Pico wheel controller.

Plain Python that runs on a PC and on MicroPython. Every line is ASCII:
`NAME arg arg ...*HH` ending in a newline, where HH is the XOR of all body
characters in two hex digits. See docs/PICO_PROTOCOL.md.

KEEP IN SYNC: src/trash_vision/trash_vision/pico_protocol.py is a byte for
byte copy of this file (a unit test fails if they differ).
"""

VERSION = 1

# Telemetry flag bits:
FLAG_WATCHDOG = 1        # no velocity command arrived in time, wheels stopped
FLAG_UNCONFIGURED = 2    # PINS and/or CFG not received yet, wheels stopped
FLAG_FAULT = 4           # hardware setup failed (see the ERR line)

# The Pico's time counter wraps; timestamps are sent modulo this value.
MS_MODULO = 1 << 30

N_PINS = 8
CFG_FIELDS = (
    'ticks_per_rev', 'wheel_radius_m', 'max_speed_mps', 'max_accel_mps2',
    'min_duty', 'kp', 'ki', 'cmd_timeout_ms',
    'invert_left', 'invert_right', 'invert_enc_left', 'invert_enc_right')


def checksum(body):
    """XOR of all characters of body."""
    value = 0
    for ch in body:
        value ^= ord(ch)
    return value


def _fmt(value):
    if isinstance(value, bool):
        return '1' if value else '0'
    if isinstance(value, int):
        return '%d' % value
    if isinstance(value, str):
        return value
    return '%.6g' % value


def encode(name, *args):
    """Build one protocol line (without the newline)."""
    body = ' '.join([name] + [_fmt(a) for a in args])
    return '%s*%02X' % (body, checksum(body))


def _convert(token):
    try:
        return int(token)
    except ValueError:
        pass
    try:
        return float(token)
    except ValueError:
        return token


def decode(line):
    """Return (name, [args]) for a line; raise ValueError if it is bad."""
    line = line.strip()
    if '*' not in line:
        raise ValueError('missing checksum')
    body, _, tail = line.rpartition('*')
    try:
        given = int(tail, 16)
    except ValueError:
        raise ValueError('bad checksum field')
    if given != checksum(body):
        raise ValueError('checksum mismatch')
    tokens = body.split()
    if not tokens:
        raise ValueError('empty line')
    return tokens[0], [_convert(t) for t in tokens[1:]]


# Commands, Pi to Pico:
def cmd_pins(pins):
    """PINS pwm_l pwm_r dir_l dir_r enc_a_l enc_b_l enc_a_r enc_b_r."""
    return encode('PINS', *pins)


def cmd_cfg(*values):
    """CFG followed by the CFG_FIELDS values in order."""
    return encode('CFG', *values)


def cmd_velocity(left_mps, right_mps):
    """V left right: wheel speed targets in m/s."""
    return encode('V', left_mps, right_mps)


def cmd_stop():
    """STOP: zero the targets at once."""
    return encode('STOP')


def cmd_ping():
    """PING: the Pico answers PONG."""
    return encode('PING')


# Messages, Pico to Pi:
def msg_ready():
    """READY version: sent after boot."""
    return encode('READY', VERSION)


def msg_pong():
    """PONG: answer to PING."""
    return encode('PONG')


def msg_error(code):
    """ERR code: code is one word, e.g. BAD_PINS."""
    return encode('ERR', code)


def msg_telemetry(ms, ticks_left, ticks_right, speed_left, speed_right,
                  flags):
    """T ms ticks_l ticks_r speed_l speed_r flags."""
    return encode('T', int(ms) % MS_MODULO, int(ticks_left),
                  int(ticks_right), speed_left, speed_right, int(flags))
