"""Pure-Python calibration maths (no ROS or hardware needed).

Each function turns one physical action into a number for config/robot.yaml.
The thin ROS/GPIO wrappers live in calibrate.py.
"""
import math
import statistics

from trash_vision import ground_projection as gp


def ticks_per_rev(total_ticks, turns):
    """Return encoder counts per wheel revolution from a hand-spun test.

    ``total_ticks`` may be negative (wheel spun backwards); the sign is
    reported separately by ``spin_direction``.
    """
    if turns <= 0:
        raise ValueError('turns must be > 0')
    if total_ticks == 0:
        raise ValueError('no encoder ticks counted: check wiring and pins')
    return abs(total_ticks) / float(turns)


def spin_direction(total_ticks):
    """+1 if counts rose, -1 if they fell (-1 means invert_enc_* is needed)."""
    return 1 if total_ticks >= 0 else -1


def odom_distance(x0, y0, x1, y1):
    """Return the straight-line distance travelled between two odometry poses."""
    return math.hypot(x1 - x0, y1 - y0)


def wheel_radius_correction(radius, odom_dist, measured_dist):
    """Return the new wheel radius so odometry matches a tape-measured distance.

    Odometry distance is proportional to the radius, so scale by the ratio.
    """
    if radius <= 0 or odom_dist <= 0 or measured_dist <= 0:
        raise ValueError('radius, odom and measured distance must be > 0')
    return radius * measured_dist / odom_dist


def angle_diff(a, b):
    """Smallest signed difference a - b in radians."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def track_width_correction(track, odom_yaw_rad, measured_yaw_rad):
    """Return the new track width so odometry yaw matches the measured turn angle.

    yaw_odom = (right - left) / track, so a wheel path that really turned
    more than odometry says means the track is shorter than configured.
    Angles may exceed one turn (unwrapped); use magnitudes.
    """
    if track <= 0:
        raise ValueError('track must be > 0')
    odom = abs(odom_yaw_rad)
    meas = abs(measured_yaw_rad)
    if odom < 1e-6 or meas < 1e-6:
        raise ValueError('odom and measured angle must be non-zero')
    return track * odom / meas


def unwrap_yaw(samples):
    """Total signed rotation from a sequence of wrapped yaw angles."""
    total = 0.0
    for prev, cur in zip(samples, samples[1:]):
        total += angle_diff(cur, prev)
    return total


def pick_zone(samples, min_tolerance=0.05, spread_factor=3.0):
    """Derive pick_floor_y and pick_center_tolerance from detector samples.

    ``samples`` are (x, y) normalized floor points of a can placed at the
    pick position. pick_floor_y is the median y (the behavior picks once y
    is at least that large). The tolerance covers detector jitter in x plus
    the worst offset seen, never below ``min_tolerance``.
    """
    if len(samples) < 5:
        raise ValueError('need at least 5 detections, got %d' % len(samples))
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    y_med = statistics.median(ys)
    x_med = statistics.median(xs)
    spread = statistics.pstdev(xs)
    worst = max(abs(x - 0.5) for x in xs)
    tol = max(min_tolerance, spread_factor * spread, worst)
    return {
        'pick_floor_y': round(y_med, 3),
        'pick_center_tolerance': round(tol, 3),
        'x_median': round(x_med, 3),
        'y_spread': round(statistics.pstdev(ys), 4),
        'n': len(samples),
    }


def camera_pitch_from_marker(u, v, true_distance, cam):
    """Pitch (degrees) that makes a marker at (u, v) land at ``true_distance``.

    The marker lies on the floor on the centreline, ``true_distance`` metres
    ahead of base_link. Solved by bisection over the pitch; returns None if
    no pitch in 5..85 degrees matches.
    """
    def err(pitch_deg):
        c = gp.camera_from_degrees(
            cam.x, cam.y, cam.height, pitch_deg,
            math.degrees(cam.hfov_rad), math.degrees(cam.vfov_rad))
        pt = gp.floor_point_to_base(u, v, c)
        # above horizon: treat as very far
        return (1e6 if pt is None else pt[0]) - true_distance

    lo, hi = 5.0, 85.0
    e_lo, e_hi = err(lo), err(hi)
    if e_lo * e_hi > 0:
        return None
    for _ in range(60):
        mid = (lo + hi) / 2.0
        e_mid = err(mid)
        if e_lo * e_mid <= 0:
            hi = mid
        else:
            lo, e_lo = mid, e_mid
    return round((lo + hi) / 2.0, 2)


def pitch_error(configured_deg, solved_deg):
    """Return solved minus configured camera pitch in degrees."""
    return round(solved_deg - configured_deg, 2)


# ---------------------------------------------------------------- sonar

def analyze_sonar(series, min_samples=10, stuck_samples=20):
    """Classify each sensor from a list of readings per name.

    ``series`` maps sensor name -> list of ranges in metres (inf = no echo,
    -inf = too close, None entries are ignored). Returns name -> (status,
    note) with status OK, NO_DATA, STUCK or NO_ECHO.
    """
    result = {}
    for name, values in series.items():
        vals = [v for v in values if v is not None]
        if not vals:
            result[name] = ('NO_DATA', 'never reported: check pins and node')
            continue
        finite = [v for v in vals if math.isfinite(v)]
        if len(vals) >= min_samples and not finite:
            result[name] = ('NO_ECHO',
                            'never heard an echo: check echo wiring/divider '
                            'and that something is in range')
        elif len(finite) >= stuck_samples and len(set(finite)) == 1:
            result[name] = ('STUCK', 'identical reading %.3f m every time'
                            % finite[0])
        else:
            result[name] = ('OK', '')
    return result


def hand_test_reading(series):
    """Sensor with the smallest median finite range (the one a hand covers).

    Returns None when no sensor has a finite reading.
    """
    best = None
    for name, values in series.items():
        finite = [v for v in values if v is not None and math.isfinite(v)
                  and v > 0]
        if not finite:
            continue
        med = statistics.median(finite)
        if best is None or med < best[1]:
            best = (name, med)
    return best


def find_swaps(expected_to_seen):
    """From {sensor asked about: sensor that reacted} list the mismatches.

    A value of None means nothing reacted. Pairs that exchange names are
    reported as a swap, others as a wrong/missing wire.
    """
    problems = []
    done = set()
    for asked, seen in expected_to_seen.items():
        if asked == seen or asked in done:
            continue
        if seen is not None and expected_to_seen.get(seen) == asked:
            problems.append('%s and %s are swapped: exchange their pins '
                            '(trig and echo) in robot.yaml' % (asked, seen))
            done.update((asked, seen))
        elif seen is None:
            problems.append('%s: nothing reacted when covered' % asked)
        else:
            problems.append('%s: covering it reacted on %s' % (asked, seen))
    return problems


def yaml_line(key, value):
    """Format a line to paste into robot.yaml."""
    if isinstance(value, float):
        value = ('%.4f' % value).rstrip('0').rstrip('.')
        if '.' not in value:
            value += '.0'
    return '%s: %s' % (key, value)
