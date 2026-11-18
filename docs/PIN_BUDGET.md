# Pin budget (options only, the choice is Bryan's)

Written by the agent for TODO item 23. No pins in `robot.yaml` were changed.
Counts are from `robot.yaml` and `docs/TODO.md` (sensor layout). Header facts
(Pi 5 40-pin header) are standard but check them with `pinctrl` on your Pi.

## Pins the Pi needs today

| Signal | GPIO pins | Notes |
|---|---|---|
| Sonar trig (8) | 8 | front_1/2, rear_1/2, left_1/2, right_1/2 |
| Sonar echo (8) | 8 | 5 V echo needs a divider each (wiring, not a pin) |
| Drive PWM (2) | 2 | MDD10A PWM left/right |
| Drive DIR (2) | 2 | MDD10A DIR left/right |
| Encoders A/B (2 wheels x 2) | 4 | per `robot.yaml` (`enc_a_pins`, `enc_b_pins`) |
| IMU (BNO055) | 2 | I2C1 = GPIO2 (SDA), GPIO3 (SCL); fixed pins |
| GPS | 2 | UART TX/RX, e.g. GPIO14/15 on UART0; fixed-function pins |
| Arm (SO101) | 0 | USB serial (`/dev/ttyACM*`), no GPIO |
| **Total** | **28** | |

## Pins available

The Pi 5 header has 28 GPIO (BCM 0 to 27). GPIO0 and GPIO1 are the HAT ID
EEPROM I2C bus and should be left alone, so **26 are usable**. Two of those
(GPIO2/3) are already the I2C bus and two more (GPIO14/15) are the GPS UART,
so only **22 are free for sonar, PWM, DIR and encoders**, which need
**24**. Shortfall: **2 pins** by this count (28 needed vs 26 usable).

`RISKS.md` on `docs/consolidated-state` says about 32 needed vs 26. This
count is lower because the arm is on USB, not GPIO; if you add anything else
(status LED, e-stop input, bumper switch, spare I2C device INT lines, a second
UART) each costs one more pin and the shortfall grows. PWM note (RISKS #10):
only some pins have hardware PWM on the Pi 5, so PWM, DIR and encoder pins must
be chosen with `pinctrl` in mind.

## Options

Pin counts are the Pi GPIOs used for the affected signals only.

| Option | Sonar pins | Wheel pins | Total GPIO (incl. IMU 2, GPS 2) | Shortfall vs 26 | Cost / risk |
|---|---|---|---|---|---|
| 0. Today (no change) | 16 | 8 | 28 | 2 | Does not fit |
| A. Sonar trig via 3-to-8 decoder (e.g. 74HC138), echoes OR-ed into one input | 3 trig + 1 echo = 4 | 8 | 16 | 0 (10 spare) | Needs 2 small ICs, a diode/OR stage and level shifting; works because sonars fire one at a time anyway; echo OR means the node must know which sensor it fired (it does, round-robin); node code change needed |
| B. Shared trigger line, 8 echoes | 1 trig + 8 echo = 9 | 8 | 21 | 0 (5 spare) | Not compatible with the "one at a time" rule: all sensors would ping together and hear each other. Only usable with a per-sensor enable, which costs pins again. Not recommended |
| C. I2C GPIO expander (e.g. MCP23017) for the 8 trig lines, echo stays on Pi GPIO | 8 echo (trig on expander, 0 GPIO) | 8 | 20 | 0 (6 spare) | One board on the I2C bus the IMU already uses. Putting echo on the expander too would save 8 more pins but I2C timing (about 100 us steps, ~1.7 cm, plus jitter) is too coarse, so echo stays on the Pi |

Note on C: with the hybrid, the sonar needs 8 echo GPIOs on the Pi plus the
existing I2C bus. Total = 8 (echo) + 8 (wheels) + 2 (I2C, shared with IMU)
+ 2 (GPS) = 20.

## Quick comparison

Fewest parts and no new firmware: Option C hybrid (one MCP23017 board on the
I2C bus already used by the IMU). Fewest pins and best timing: Option D or E.
Option A is the cheapest in parts but needs a small circuit to be built and
tested. This is a summary for the decision, not a recommendation.

## What the code would need

- A, C: the `ultrasonic` node pin driver changes (decoder select lines or
  expander writes). Config keys would be added then, not now.
- D: drive node and `robot.yaml` drive section change. Blocked item.
- Any option: Bryan fills `robot.yaml` pins and runs `hardware_check`.
