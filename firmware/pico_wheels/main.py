"""MicroPython entry point for the Pico wheel controller.

Copy protocol.py, control.py, app.py and this file to the Pico (see
docs/PICO_PROTOCOL.md). Talks to the Pi over the USB serial port (stdin and
stdout). All pins arrive from the Pi in the PINS command, which come from
config/robot.yaml (pico_bridge.pico_pins). TODO: Bryan to fill in the pins.

Not testable in CI (needs the board): see docs/HARDWARE_TESTS.md.
"""
import select
import sys
import time

from machine import Pin, PWM

import app
import control

PWM_FREQ_HZ = 1000


class PicoHardware:
    """Real GPIO: two PWM+DIR motor outputs and two quadrature encoders."""

    def __init__(self):
        """Nothing is claimed until configure_pins."""
        self.pwm = [None, None]
        self.direction = [None, None]
        self.decoders = [control.QuadratureDecoder(),
                         control.QuadratureDecoder()]
        self.enc = []

    def configure_pins(self, pins):
        """pins = pwm_l, pwm_r, dir_l, dir_r, a_l, b_l, a_r, b_r."""
        self.stop_all()
        for dev in self.pwm:
            if dev is not None:
                dev.deinit()
        self.pwm = [PWM(Pin(pins[0])), PWM(Pin(pins[1]))]
        for dev in self.pwm:
            dev.freq(PWM_FREQ_HZ)
            dev.duty_u16(0)
        self.direction = [Pin(pins[2], Pin.OUT), Pin(pins[3], Pin.OUT)]
        self.enc = []
        for side in (0, 1):
            a = Pin(pins[4 + 2 * side], Pin.IN, Pin.PULL_UP)
            b = Pin(pins[5 + 2 * side], Pin.IN, Pin.PULL_UP)
            self.decoders[side] = control.QuadratureDecoder(a.value(), b.value())
            handler = self._make_handler(side, a, b)
            a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=handler)
            b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=handler)
            self.enc.append((a, b))

    def _make_handler(self, side, a, b):
        dec = self.decoders

        def handler(_pin):
            dec[side].update(a.value(), b.value())
        return handler

    def read_counts(self):
        """Return (left, right) raw counts."""
        return self.decoders[0].count, self.decoders[1].count

    def set_motor(self, side, duty, forward):
        """Set PWM duty 0..1 and the direction pin."""
        self.direction[side].value(1 if forward else 0)
        self.pwm[side].duty_u16(int(max(0.0, min(1.0, duty)) * 65535))

    def stop_all(self):
        """PWM to zero on both sides."""
        for dev in self.pwm:
            if dev is not None:
                dev.duty_u16(0)


def run():
    """Read lines from the USB serial port and run the wheel loop forever."""
    hw = PicoHardware()
    pico = app.PicoApp(hw)
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    buf = ''
    for line in pico.hello():
        print(line)
    while True:
        now = time.ticks_ms()
        while poller.poll(0):
            ch = sys.stdin.read(1)
            if ch == '\n':
                for reply in pico.handle_line(buf, now):
                    print(reply)
                buf = ''
            elif len(buf) < 200:
                buf += ch
            else:
                buf = ''
        for line in pico.poll(now):
            print(line)
        time.sleep_ms(2)


run()
