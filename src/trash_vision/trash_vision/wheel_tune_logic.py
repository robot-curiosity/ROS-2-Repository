"""Wheel PID step-response tuner: all the logic (no ROS, no GPIO).

One side at a time gets a speed step through /cmd_vel and the measured side
speed is read back from /odom, so the Pi drive node and the Pico bridge are
handled the same way. Gain suggestions need an open-loop run (kp = ki = 0,
feed-forward only): a first-order-plus-dead-time model is fitted to the
response of the ramped command and PI gains follow the SIMC rules
(S. Skogestad, 2003). The drivers add feed-forward to the PI output, which
SIMC does not model, so its closed-loop constant tau_c is raised until the
fitted model, run with the real control law, overshoots at most 10 %.
They are suggestions to try, not verified values.
"""
import math

from trash_vision import drive_logic as dl

SIDES = ('left', 'right')
SPEED_CAP_MPS = 0.35        # largest step the tool will command
SETTLE_BAND = 0.05          # +-5 % of the final value
MIN_FIT_R2 = 0.9
MIN_TAU_C_S = 0.1           # lower limit of the SIMC closed-loop constant
MAX_TAU_C_S = 2.0
MAX_PREDICTED_OVERSHOOT = 0.10
CONTROL_DT_S = 0.02         # drive node and Pico loop period (50 Hz)
PARAM_NAMES = ('kp', 'ki', 'max_wheel_speed_mps', 'max_accel_mps2',
               'track_width_m')


# ------------------------------------------------------------ commands

def side_twist(side, speed, track_width):
    """(linear, angular) that gives `speed` on one side and 0 on the other."""
    if side not in SIDES:
        raise ValueError('side must be left or right')
    angular = (-speed if side == 'left' else speed) / track_width
    return speed / 2.0, angular


def side_speed(side, linear, angular, track_width):
    """Measured speed of one side from the odometry twist."""
    left, right = dl.twist_to_wheels(linear, angular, track_width)
    return left if side == 'left' else right


def step_speed(requested, max_wheel_speed):
    """The step size actually used: positive and capped."""
    if not math.isfinite(requested) or requested <= 0.0:
        raise ValueError('step speed must be a positive number')
    return min(requested, SPEED_CAP_MPS, max_wheel_speed)


class StepGuard:
    """Abort a step before a wiring or sign fault runs a motor away."""

    def __init__(self, target, odom_timeout_s=0.5, no_motion_s=1.0):
        """Watch a step toward `target` m/s."""
        self.target = target
        self.odom_timeout_s = odom_timeout_s
        self.no_motion_s = no_motion_s
        self._reverse = 0
        self._fast = 0
        self._moved = False
        self._last_t = 0.0

    def sample(self, t, measured):
        """Feed one measurement; return a reason to abort or None."""
        self._last_t = t
        if not math.isfinite(measured):
            return 'odometry is not a number'
        if abs(measured) >= 0.02:
            self._moved = True
        low = -max(0.05, 0.3 * self.target)
        self._reverse = self._reverse + 1 if measured < low else 0
        if self._reverse >= 3:
            return ('wheel reads BACKWARDS: check invert_*/invert_enc_* for '
                    'this side')
        high = 1.8 * self.target + 0.05
        self._fast = self._fast + 1 if measured > high else 0
        if self._fast >= 5:
            return 'wheel far above the target (runaway)'
        return None

    def check(self, t):
        """Time-based checks; return a reason to abort or None."""
        if t - self._last_t > self.odom_timeout_s:
            return 'no odometry from the wheel driver'
        if t > self.no_motion_s and not self._moved:
            return ('no motion measured: check motor power, encoder wiring '
                    'and min_duty')
        return None


def run_step(side, target, track_width, send, read, clock, sleep,
             duration_s=2.0, rate_hz=20.0):
    """Step one side and record it; always sends zero at the end.

    send(linear, angular) publishes a command, read() returns the new
    (time, linear, angular) odometry samples since the last call, clock()
    is in the same time base as those samples, sleep(s) waits.
    Returns ([(t, measured)], abort reason or None).
    """
    guard = StepGuard(target)
    samples = []
    reason = None
    linear, angular = side_twist(side, target, track_width)
    read()  # drop what arrived before the step
    start = clock()
    try:
        while reason is None and clock() - start < duration_s:
            send(linear, angular)
            sleep(1.0 / rate_hz)
            for stamp, v, w in read():
                t = stamp - start
                if t < 0.0:
                    continue
                measured = side_speed(side, v, w, track_width)
                samples.append((t, measured))
                reason = reason or guard.sample(t, measured)
            reason = reason or guard.check(clock() - start)
    finally:
        send(0.0, 0.0)
    return samples, reason


# ------------------------------------------------------------ metrics

def median3(values):
    """3-point median filter (removes single encoder-quantisation spikes)."""
    if len(values) < 3:
        return list(values)
    out = [values[0]]
    for i in range(1, len(values) - 1):
        out.append(sorted(values[i - 1:i + 2])[1])
    out.append(values[-1])
    return out


def _crossing(times, values, level):
    """First time the signal reaches level (linear interpolation)."""
    for i, v in enumerate(values):
        if v >= level:
            if i == 0 or values[i - 1] >= level:
                return times[i]
            frac = (level - values[i - 1]) / (v - values[i - 1])
            return times[i - 1] + frac * (times[i] - times[i - 1])
    return None


def step_metrics(samples, target):
    """Rise time, overshoot, settling time and steady-state error.

    Final value = mean of the last quarter of the record. Rise time is 10 to
    90 % of it, overshoot is relative to it, settling time is when the signal
    stays within +-5 % of it. A value that cannot be computed is None.
    """
    if len(samples) < 8 or target <= 0.0:
        raise ValueError('need at least 8 samples and a positive target')
    times = [t for t, _ in samples]
    raw = [y for _, y in samples]
    tail = raw[-max(len(raw) // 4, 2):]
    final = sum(tail) / len(tail)
    result = {
        'target': target, 'final': final,
        'ss_error': target - final,
        'ss_error_pct': 100.0 * (target - final) / target,
        'rise_time_s': None, 'overshoot_pct': None, 'settling_time_s': None,
    }
    if final < 0.05 * target:
        return result
    smooth = median3(raw)
    t10 = _crossing(times, smooth, 0.1 * final)
    t90 = _crossing(times, smooth, 0.9 * final)
    if t10 is not None and t90 is not None:
        result['rise_time_s'] = t90 - t10
    result['overshoot_pct'] = max(0.0, 100.0 * (max(smooth) - final) / final)
    outside = [i for i, y in enumerate(smooth)
               if abs(y - final) > SETTLE_BAND * final]
    if not outside:
        result['settling_time_s'] = times[0]
    elif outside[-1] < len(times) - max(len(times) // 10, 2):
        result['settling_time_s'] = times[outside[-1] + 1]
    return result


# ------------------------------------------------------- model and gains

def ramp_response(t, tau, final_u, accel_u):
    """Unit-gain first-order response to a ramp from 0 that holds final_u."""
    if t <= 0.0:
        return 0.0
    if accel_u <= 0.0:
        return final_u * (1.0 - math.exp(-t / tau))
    ramp_s = final_u / accel_u
    tr = min(t, ramp_s)
    x = accel_u * (tr - tau * (1.0 - math.exp(-tr / tau)))
    if t <= ramp_s:
        return x
    return final_u - (final_u - x) * math.exp(-(t - ramp_s) / tau)


def fit_model(samples, target, max_speed, max_accel):
    """Fit gain K (m/s per unit command), tau and dead time theta (s).

    Open-loop data only: the command is the ramped target / max_speed.
    Returns a dict with K, tau, theta and r2, or None.
    """
    if len(samples) < 8 or target <= 0.0 or max_speed <= 0.0:
        return None
    ys = [y for _, y in samples]
    mean = sum(ys) / len(ys)
    total = sum((y - mean) ** 2 for y in ys)
    if total <= 0.0:
        return None
    final_u = target / max_speed
    accel_u = max_accel / max_speed

    def score(tau, theta):
        xs = [ramp_response(t - theta, tau, final_u, accel_u)
              for t, _ in samples]
        denom = sum(x * x for x in xs)
        if denom <= 0.0:
            return float('inf'), 0.0
        gain = sum(x * y for x, y in zip(xs, ys)) / denom
        return sum((y - gain * x) ** 2 for x, y in zip(xs, ys)), gain

    taus = [0.01 * (200.0 ** (i / 39.0)) for i in range(40)]   # 0.01..2 s
    thetas = [0.01 * i for i in range(41)]                      # 0..0.4 s
    err, gain, tau, theta = min(
        score(a, b) + (a, b) for a in taus for b in thetas)
    # Refine around the best grid point:
    err, gain, tau, theta = min(
        score(a, b) + (a, b)
        for a in [tau * (1.0 + 0.02 * i) for i in range(-7, 8)]
        for b in [max(0.0, theta + 0.001 * j) for j in range(-10, 11)])
    if gain <= 0.0 or not math.isfinite(err):
        return None
    return {'K': gain, 'tau': tau, 'theta': theta, 'r2': 1.0 - err / total}


def simc_pi(model, tau_c=None):
    """SIMC PI rules: kp = tau / (K (tau_c + theta)), Ti = min(tau, ...)."""
    if tau_c is None:
        tau_c = max(model['theta'], MIN_TAU_C_S)
    horizon = tau_c + model['theta']
    kp = model['tau'] / (model['K'] * horizon)
    ti = min(model['tau'], 4.0 * horizon)
    return {'kp': kp, 'ki': kp / ti, 'tau_c': tau_c}


def simulate(model, kp, ki, target, max_speed, max_accel, duration_s=2.0):
    """Closed loop of the real control law on the fitted model."""
    pi = dl.PIController(kp, ki)
    alpha = math.exp(-CONTROL_DT_S / model['tau'])
    delayed = [0.0] * int(round(model['theta'] / CONTROL_DT_S))
    applied = 0.0
    speed = 0.0
    out = []
    for i in range(int(round(duration_s / CONTROL_DT_S))):
        applied = dl.slew(applied, target, max_accel, CONTROL_DT_S)
        cmd = pi.update(applied, speed, CONTROL_DT_S, applied / max_speed)
        delayed.append(cmd)
        u = delayed.pop(0)
        speed = alpha * speed + (1.0 - alpha) * model['K'] * u
        out.append(((i + 1) * CONTROL_DT_S, speed))
    return out


def suggest_gains(model, target, max_speed, max_accel):
    """SIMC gains with tau_c raised until the predicted overshoot is small."""
    # Feed-forward alone may already end above the target (fast motor):
    limit = (1.0 + MAX_PREDICTED_OVERSHOOT) * max(
        target, model['K'] * target / max_speed)
    gains = simc_pi(model)
    while gains['tau_c'] < MAX_TAU_C_S:
        run = simulate(model, gains['kp'], gains['ki'], target, max_speed,
                       max_accel, 3.0)
        if max(y for _, y in run) <= limit:
            break
        gains = simc_pi(model, min(gains['tau_c'] * 1.25, MAX_TAU_C_S))
    return gains


# ------------------------------------------------------------ analysis

def analyze_side(samples, target, params):
    """Metrics and, for open-loop data, model and suggested gains."""
    result = {'metrics': step_metrics(samples, target), 'model': None,
              'gains': None, 'predicted': None, 'note': ''}
    if params['kp'] != 0.0 or params['ki'] != 0.0:
        result['note'] = ('closed-loop run: for gain suggestions set kp: 0.0 '
                          'and ki: 0.0, restart the driver and run again')
        return result
    max_speed = params['max_wheel_speed_mps']
    accel = params['max_accel_mps2']
    model = fit_model(samples, target, max_speed, accel)
    if model is None or model['r2'] < MIN_FIT_R2:
        result['model'] = model
        result['note'] = ('no suggestion: a first-order model does not fit '
                          'this record (no motion, noise or too short)')
        return result
    result['model'] = model
    result['gains'] = suggest_gains(model, target, max_speed, accel)
    if abs(model['K'] / max_speed - 1.0) > 0.2:
        result['note'] = (
            'feed-forward assumes full command = max_wheel_speed_mps (%g) '
            'but the model says %.2f m/s: consider correcting it, then run '
            'again' % (max_speed, model['K']))
    result['predicted'] = step_metrics(
        simulate(model, result['gains']['kp'], result['gains']['ki'],
                 target, max_speed, accel), target)
    return result


def _fmt(value, spec, unit=''):
    return 'n/a' if value is None else format(value, spec) + unit


def _metric_line(m):
    return ('rise %s, overshoot %s, settling %s, steady-state error %s '
            '(%s)' % (_fmt(m['rise_time_s'], '.3f', ' s'),
                      _fmt(m['overshoot_pct'], '.1f', ' %'),
                      _fmt(m['settling_time_s'], '.3f', ' s'),
                      _fmt(m['ss_error'], '+.3f', ' m/s'),
                      _fmt(m['ss_error_pct'], '+.1f', ' %')))


def format_report(results, params):
    """Text report for {side: analyze_side() result}."""
    lines = ['kp=%g ki=%g max_wheel_speed=%g m/s max_accel=%g m/s^2' % (
        params['kp'], params['ki'], params['max_wheel_speed_mps'],
        params['max_accel_mps2'])]
    gains = []
    for side, r in results.items():
        m = r['metrics']
        lines.append('%s: step %.2f m/s, final %.3f m/s' % (
            side, m['target'], m['final']))
        lines.append('  ' + _metric_line(m))
        accel = params['max_accel_mps2']
        if accel > 0.0 and m['rise_time_s'] is not None and \
                m['rise_time_s'] < 1.2 * 0.8 * m['target'] / accel:
            lines.append('  rise time is set by the max_accel ramp, not by '
                         'the gains')
        if r['gains']:
            fit, g = r['model'], r['gains']
            gains.append(g)
            lines.append('  model: K=%.3f m/s, tau=%.3f s, dead time=%.3f s, '
                         'R2=%.3f' % (fit['K'], fit['tau'], fit['theta'],
                                      fit['r2']))
            lines.append('  SIMC PI SUGGESTION (tau_c=%.2f s), NOT VERIFIED: '
                         'kp: %.2f  ki: %.2f' % (g['tau_c'], g['kp'], g['ki']))
            lines.append('  model prediction with these gains: ' +
                         _metric_line(r['predicted']))
        if r['note']:
            lines.append('  ' + r['note'])
    if len(gains) == 2:
        # Both sides share one kp/ki, so offer the more cautious pair:
        lines.append('To try (both sides share the gains; lower of each, not '
                     'verified): kp: %.2f  ki: %.2f' % (
                         min(g['kp'] for g in gains),
                         min(g['ki'] for g in gains)))
    lines.append('Wheels were unloaded: confirm any gains on the ground.')
    return '\n'.join(lines)


# ------------------------------------------------------------ files

def to_csv(records, params):
    """CSV text of {side: (target, samples)} with the parameters on top."""
    lines = ['# %s=%.9g' % (k, params[k]) for k in PARAM_NAMES]
    lines.append('side,t_s,target_mps,measured_mps')
    for side, (target, samples) in records.items():
        lines.extend('%s,%.4f,%.4f,%.5f' % (side, t, target, y)
                     for t, y in samples)
    return '\n'.join(lines) + '\n'


def from_csv(text):
    """Inverse of to_csv: (records, params). Raises ValueError."""
    params = {}
    records = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith('side,'):
            continue
        if line.startswith('#'):
            key, sep, value = line[1:].partition('=')
            if sep:
                params[key.strip()] = float(value)
            continue
        side, t, target, y = line.split(',')
        if side not in SIDES:
            raise ValueError('unknown side: %s' % side)
        records.setdefault(side, (float(target), []))[1].append(
            (float(t), float(y)))
    missing = [k for k in PARAM_NAMES if k not in params]
    if missing or not records:
        raise ValueError('not a tune_wheels CSV (missing %s)' % (
            ', '.join(missing) or 'data'))
    return records, params


def to_svg(records, width=640, height=240):
    """One plot per side: target (dashed) and measured speed."""
    pad = 40
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
             'font-family="sans-serif" font-size="12">' % (
                 width, height * max(len(records), 1)),
             '<rect width="100%" height="100%" fill="white"/>']
    for row, (side, (target, samples)) in enumerate(records.items()):
        top = row * height
        t_max = max([t for t, _ in samples] + [0.1])
        ys = [y for _, y in samples] + [target, 0.0]
        lo, hi = min(ys), max(ys) * 1.1 or 1.0

        def x(t):
            return pad + (width - 2 * pad) * t / t_max

        def y(v):
            return top + height - pad - (height - 2 * pad) * (
                (v - lo) / (hi - lo))

        parts.append('<text x="%d" y="%d">%s wheel: speed (m/s) over time, '
                     'dashed = target %.2f</text>' % (
                         pad, top + 20, side, target))
        parts.append('<path d="M%d %.1fV%.1fH%d" fill="none" stroke="#888"/>'
                     % (pad, y(hi), y(lo), width - pad))
        parts.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#c00" '
                     'stroke-dasharray="6 4"/>' % (
                         pad, width - pad, y(target), y(target)))
        parts.append('<polyline fill="none" stroke="#06c" stroke-width="1.5" '
                     'points="%s"/>' % ' '.join(
                         '%.1f,%.1f' % (x(t), y(v)) for t, v in samples))
        for value in (lo, target, hi):
            parts.append('<text x="2" y="%.1f">%.2f</text>' % (
                y(value) + 4, value))
        parts.append('<text x="%d" y="%.1f">%.2f s</text>' % (
            width - pad - 20, y(lo) + 16, t_max))
    parts.append('</svg>')
    return '\n'.join(parts) + '\n'
