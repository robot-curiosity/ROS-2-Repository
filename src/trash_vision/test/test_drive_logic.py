import math

from trash_vision import drive_logic as dl


def test_quadrature_forward_and_reverse():
    d = dl.QuadratureDecoder(0, 0)
    for a, b in [(1, 0), (1, 1), (0, 1), (0, 0)]:
        d.update(a, b)
    assert d.count == 4
    for a, b in [(0, 1), (1, 1), (1, 0), (0, 0)]:
        d.update(a, b)
    assert d.count == 0


def test_quadrature_ignores_repeat_and_skip():
    d = dl.QuadratureDecoder(0, 0)
    assert d.update(0, 0) == 0
    assert d.update(1, 1) == 0  # illegal two-bit jump


def test_twist_and_limit():
    left, right = dl.twist_to_wheels(0.2, 1.0, 0.4)
    assert abs(left - 0.0) < 1e-9 and abs(right - 0.4) < 1e-9
    left, right = dl.limit_wheel_speeds(1.0, 0.5, 0.5)
    assert abs(left - 0.5) < 1e-9 and abs(right - 0.25) < 1e-9
    assert dl.limit_wheel_speeds(0.1, -0.1, 0.5) == (0.1, -0.1)


def test_command_to_pwm():
    assert dl.command_to_pwm(0.0) == (0.0, True)
    assert dl.command_to_pwm(-0.5) == (0.5, False)
    assert dl.command_to_pwm(2.0) == (1.0, True)
    duty, _ = dl.command_to_pwm(0.5, min_duty=0.2)
    assert abs(duty - 0.6) < 1e-9


def test_slew():
    assert abs(dl.slew(0.0, 1.0, 1.0, 0.1) - 0.1) < 1e-9
    assert dl.slew(0.95, 1.0, 1.0, 0.1) == 1.0
    assert dl.slew(0.0, 1.0, 0.0, 0.1) == 1.0


def test_pi_converges_and_clamps():
    pi = dl.PIController(0.5, 1.0, limit=1.0)
    out = 0.0
    for _ in range(100):
        out = pi.update(1.0, 0.0, 0.1)
    assert out == 1.0
    assert abs(pi._integral) <= 1.0 / 1.0 + 1e-9


def test_odometry_straight_and_turn():
    odo = dl.DiffDriveOdometry(ticks_per_rev=100, wheel_radius=0.05,
                               track_width=0.3)
    x, y, th = odo.update(100, 100, 1.0)
    assert abs(x - 2 * math.pi * 0.05) < 1e-9 and abs(y) < 1e-9
    assert abs(odo.v - 2 * math.pi * 0.05) < 1e-9
    odo2 = dl.DiffDriveOdometry(100, 0.05, 0.3)
    odo2.update(-10, 10, 1.0)
    assert odo2.theta > 0.0 and abs(odo2.x) < 1e-9


def test_yaw_to_quaternion():
    z, w = dl.yaw_to_quaternion(math.pi)
    assert abs(z - 1.0) < 1e-9 and abs(w) < 1e-9
