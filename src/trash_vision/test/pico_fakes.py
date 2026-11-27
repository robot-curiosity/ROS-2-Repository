"""Fake Pico hardware and serial link shared by the Pico tests."""
import math
import os
import sys

FIRMWARE = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                        'firmware', 'pico_wheels')
if FIRMWARE not in sys.path:
    sys.path.insert(0, FIRMWARE)

import app  # noqa: E402


class FakeHardware:
    """Two motors with a first-order response and ideal encoders."""

    def __init__(self, ticks_per_rev=1000.0, wheel_radius=0.06,
                 top_speed=0.6, lag_s=0.1):
        """Create the plant; speeds in m/s."""
        self.ticks_per_rev = ticks_per_rev
        self.radius = wheel_radius
        self.top_speed = top_speed
        self.lag_s = lag_s
        self.duty = [0.0, 0.0]
        self.speed = [0.0, 0.0]
        self.counts = [0.0, 0.0]
        self.pins = None
        self.fail_pins = False
        self.stopped = 0

    def configure_pins(self, pins):
        """Record the pins (or fail on request)."""
        if self.fail_pins:
            raise RuntimeError('no such pin')
        self.pins = pins

    def read_counts(self):
        """Return integer counts."""
        return int(self.counts[0]), int(self.counts[1])

    def set_motor(self, side, duty, forward):
        """Record the signed duty."""
        self.duty[side] = duty if forward else -duty

    def stop_all(self):
        """Zero both outputs."""
        self.duty = [0.0, 0.0]
        self.stopped += 1

    def advance(self, dt):
        """Move the plant forward by dt seconds."""
        per_tick = 2.0 * math.pi * self.radius / self.ticks_per_rev
        for i in range(2):
            goal = self.duty[i] * self.top_speed
            self.speed[i] += (goal - self.speed[i]) * min(1.0, dt / self.lag_s)
            self.counts[i] += self.speed[i] * dt / per_tick


class FakeLink:
    """A Pico firmware instance on a fake serial line, driven by time."""

    def __init__(self, hw=None):
        """Create the firmware app and an empty outbox for the Pi."""
        self.hw = hw or FakeHardware()
        self.pico = app.PicoApp(self.hw)
        self.now_ms = 1000
        self.to_pi = list(self.pico.hello())
        self.from_pi = []
        self.connected = True

    def write(self, line):
        """Pi writes a line to the Pico."""
        self.from_pi.append(line)
        if self.connected:
            self.to_pi.extend(self.pico.handle_line(line, self.now_ms))

    def run(self, seconds, on_step=None, dt_ms=20):
        """Advance time; collect telemetry into to_pi."""
        steps = int(seconds * 1000 / dt_ms)
        for _ in range(steps):
            self.now_ms += dt_ms
            self.hw.advance(dt_ms / 1000.0)
            if self.connected:
                self.to_pi.extend(self.pico.poll(self.now_ms))
            if on_step is not None:
                on_step()

    def drain(self):
        """Return and clear the lines waiting for the Pi."""
        lines, self.to_pi = self.to_pi, []
        return lines


PARAMS = {
    'serial_port': '/dev/null',
    'pico_pins': [2, 3, 4, 5, 6, 7, 8, 9],
    'ticks_per_rev': 1000.0,
    'wheel_radius_m': 0.06,
    'track_width_m': 0.48,
    'max_wheel_speed_mps': 0.5,
    'max_accel_mps2': 1.0,
    'min_duty': 0.0,
    'kp': 0.5,
    'ki': 1.0,
    'invert_left': False,
    'invert_right': False,
    'invert_enc_left': False,
    'invert_enc_right': False,
    'pico_cmd_timeout_s': 0.5,
}

