"""Pure-Python logic of the bench test CLI (no ROS, GPIO, arm or camera).

Hardware, keyboard, clock and output are passed in, so CI can test the safety
rules: every motor is asked for separately, outputs go to zero on every exit
path, and the space bar (software E-stop) stops everything. It is software
only and does not replace a hand on the power switch.
"""
import math

ESTOP_KEY = ' '
QUIT_KEY = 'q'

DEFAULT_DUTY = 0.2
MAX_DUTY = 0.35
MOTOR_RUN_S = 2.0
POLL_S = 0.02
DEFAULT_JOG_STEP = 2.0
MAX_JOG_STEP = 5.0

DONE = 'DONE'
SKIP = 'SKIP'
FAIL = 'FAIL'

SIDES = ['left', 'right']


class EStop(Exception):
    """The operator pressed the software E-stop key."""


def read_keys(keys, timeout=0.0):
    """Return the pending key presses; raise EStop if space is among them.

    ``keys.read(timeout)`` returns the typed characters ('' if none).
    """
    typed = keys.read(timeout)
    if ESTOP_KEY in typed:
        raise EStop()
    return typed


def confirm(keys, out, question):
    """Ask a y/N question. Keys typed before the question are discarded."""
    read_keys(keys)
    out(question + ' [y/N, space = E-stop]')
    while True:
        typed = read_keys(keys, 0.1)
        if typed:
            return typed[0] in ('y', 'Y')


def bench_duty(requested, min_duty=0.0):
    """PWM duty for the motor test: at least min_duty, never above MAX_DUTY.

    Raises ValueError for a request that is not a positive finite number.
    """
    requested = float(requested)
    if not math.isfinite(requested) or requested <= 0.0:
        raise ValueError('duty must be a number above 0')
    floor = float(min_duty)
    if not math.isfinite(floor) or floor < 0.0:
        floor = 0.0
    return min(max(requested, floor), MAX_DUTY)


# ---------------------------------------------------------------- motors

def run_motor(motors, side, duty, forward, keys, clock, sleep,
              run_s=MOTOR_RUN_S):
    """Drive one side for ``run_s`` seconds; the motors are off on return."""
    try:
        read_keys(keys)
        motors.set(side, duty, forward)
        end = clock() + run_s
        while clock() < end:
            read_keys(keys)
            sleep(POLL_S)
    finally:
        motors.off()


def motor_test(motors, duty, inverted, keys, out, clock, sleep):
    """Run each motor channel alone, forward, asking before each one.

    ``inverted`` maps side -> bool (drive.invert_*). ``motors`` has
    ``set(side, duty, forward)``, ``off()`` and ``counts(side)`` (encoder
    count, or None without an encoder).
    """
    ran = 0
    try:
        for side in SIDES:
            if not confirm(keys, out,
                           'Run the %s motor forward at %.0f%% duty for %g s?'
                           % (side.upper(), duty * 100.0, MOTOR_RUN_S)):
                out('%s skipped' % side)
                continue
            before = motors.counts(side)
            run_motor(motors, side, duty, not inverted.get(side, False),
                      keys, clock, sleep)
            ran += 1
            after = motors.counts(side)
            if before is None or after is None:
                out('%s done (no encoder pins set)' % side)
            else:
                out('%s done, encoder count change %+d'
                    % (side, after - before))
            out('Check: only the %s wheel(s) turned, forward.' % side)
    finally:
        motors.off()
    return DONE if ran else SKIP


# ------------------------------------------------------------------- arm

def jog_pose(pose, joint, delta, limits):
    """Return a copy of ``pose`` with one joint moved by at most ``delta``.

    Only the selected joint changes. The result is clamped to the joint
    limits, but a joint that starts outside them still moves no further than
    ``delta`` per press (no jump to the limit), and never further out.
    """
    if not all(math.isfinite(p) for p in pose) or not math.isfinite(delta):
        raise ValueError('non-finite joint value: %r' % (list(pose),))
    low, high = limits[joint]
    current = pose[joint]
    target = min(max(current + delta, low), high)
    step = target - current
    if step * delta <= 0.0:
        step = 0.0
    step = max(-abs(delta), min(abs(delta), step))
    new = list(pose)
    new[joint] = current + step
    return new


def arm_jog(backend, names, limits, step, keys, out):
    """Jog the arm one joint at a time until q (done) or space (E-stop).

    Each loop uses only the newest key, so a held key stops when released.
    On every exit the arm is told to hold the pose it is in now.
    """
    step = min(abs(float(step)), MAX_JOG_STEP)
    pose = list(backend.read())
    joint = 0
    out('Joints: ' + ', '.join(
        '%d=%s' % (i + 1, n) for i, n in enumerate(names)))
    out('number = pick joint, +/- = move %g, space = E-stop, q = done' % step)
    try:
        while True:
            typed = read_keys(keys, 0.1)
            if not typed:
                continue
            key = typed[-1]
            if key == QUIT_KEY:
                return DONE
            if key.isdigit() and 1 <= int(key) <= len(names):
                joint = int(key) - 1
                out('selected %s = %.1f' % (names[joint], pose[joint]))
            elif key in ('+', '=', '-'):
                delta = -step if key == '-' else step
                pose = jog_pose(pose, joint, delta, limits)
                backend.write(pose)
                out('%s -> %.1f' % (names[joint], pose[joint]))
    finally:
        backend.write(list(backend.read()))


# ----------------------------------------------------------------- sonar

def format_sonar(readings):
    """One line per sensor from {name: (range_m, status) or None (error)}."""
    lines = []
    for name, reading in readings.items():
        if reading is None:
            lines.append('%-8s ERROR (GPIO)' % name)
            continue
        rng, status = reading
        if status == 'no_echo':
            lines.append('%-8s no echo (nothing in range, or not wired)'
                         % name)
        elif status == 'out_of_range':
            lines.append('%-8s out of range (> %.2f m)' % (name, rng))
        elif status == 'too_close':
            lines.append('%-8s too close (< %.2f m)' % (name, rng))
        else:
            bar = '#' * int(min(rng, 2.0) * 10)
            lines.append('%-8s %5.2f m %s' % (name, rng, bar))
    return '\n'.join(lines)


def sonar_readout(names, ping, keys, out, clock, sleep, gap_s=0.06,
                  seconds=None):
    """Ping the sensors one at a time and print each round until q.

    ``ping(name)`` returns (range_m, status) and may raise on a GPIO error.
    FAIL if a sensor never gave an echo or error free reading.
    """
    if not names:
        return SKIP
    out('Live sonar. q = done, space = E-stop.')
    heard = set()
    start = clock()
    while True:
        readings = {}
        for name in names:
            if QUIT_KEY in read_keys(keys):
                return _sonar_result(names, heard, out)
            try:
                readings[name] = ping(name)
            except Exception:
                readings[name] = None
            else:
                if readings[name][1] != 'no_echo':
                    heard.add(name)
            sleep(max(gap_s, 0.06))
        out(format_sonar(readings) + '\n')
        if seconds is not None and clock() - start >= seconds:
            return _sonar_result(names, heard, out)


def _sonar_result(names, heard, out):
    silent = [n for n in names if n not in heard]
    if silent:
        out('Never heard an echo: ' + ', '.join(silent))
        return FAIL
    return DONE


# ---------------------------------------------------------------- camera

SHADES = ' .:-=+*%@'


def ascii_preview(gray_rows, boxes=()):
    """Text picture of a small grayscale grid with detection boxes.

    ``gray_rows`` is a list of rows of 0..255 values; ``boxes`` is a list of
    (x1, y1, x2, y2) normalized to 0..1. Box outlines are drawn as 'O'.
    """
    top = len(SHADES) - 1
    grid = [[SHADES[min(top, max(0, int(v) * len(SHADES) // 256))]
             for v in row] for row in gray_rows]
    height = len(grid)
    width = len(grid[0]) if height else 0
    if not width:
        return ''

    def cell(value, size):
        return min(size - 1, max(0, int(value * size)))

    for x1, y1, x2, y2 in boxes:
        left, right = cell(x1, width), cell(x2, width)
        upper, lower = cell(y1, height), cell(y2, height)
        for col in range(left, right + 1):
            grid[upper][col] = grid[lower][col] = 'O'
        for row in range(upper, lower + 1):
            grid[row][left] = grid[row][right] = 'O'
    return '\n'.join(''.join(row) for row in grid)


def describe_detections(detections, project):
    """Text lines for (name, confidence, (x1, y1, x2, y2)) detections.

    The floor point is the bottom centre of the box, as in the detector.
    ``project(u, v)`` returns base_link (x, y) metres or None.
    """
    if not detections:
        return 'no detections'
    lines = []
    for name, conf, (x1, y1, x2, y2) in detections:
        u, v = (x1 + x2) / 2.0, y2
        xy = project(u, v)
        where = ('above horizon' if xy is None
                 else 'base_x %.2f m, base_y %+.2f m' % xy)
        lines.append('%s %.2f floor (%.2f, %.2f) %s'
                     % (name, conf, u, v, where))
    return '\n'.join(lines)


# ----------------------------------------------------------------- runner

def run_steps(steps, out):
    """Run (name, function) steps in order and return the exit code.

    Each function returns DONE, SKIP or FAIL and leaves its outputs at zero
    itself. An E-stop or Ctrl-C ends the whole run (code 130); an error in
    one subsystem is a FAIL and the run goes on (code 1).
    """
    results = []
    for name, step in steps:
        out('--- %s ---' % name)
        try:
            result = step()
        except EStop:
            out('E-STOP: everything stopped.')
            return 130
        except KeyboardInterrupt:
            out('Interrupted: everything stopped.')
            return 130
        except Exception as exc:
            out('%s error: %s' % (name, exc))
            result = FAIL
        results.append((name, result))
    out('Summary: ' + ', '.join('%s %s' % r for r in results))
    return 1 if any(r == FAIL for _, r in results) else 0
