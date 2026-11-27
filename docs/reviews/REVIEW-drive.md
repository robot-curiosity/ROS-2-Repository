# Review pass 2: drive, sonar and Pico (2026-11-24)

Scope: `drive_logic.py`, `drive_node.py`, `ultrasonic_logic.py`, `ultrasonic_node.py`,
`sonar_scheduler.py`, `firmware/pico_wheels/*`, `pico_protocol.py`, `pico_bridge_logic.py`,
`pico_bridge_node.py`. Method: read as a skeptical reviewer, designs unchanged.
The robot-side behavior could not be run here; every fix below has a pure-Python
regression test in `test_review_drive.py`.

## Fixed in this pass

| File | Finding | Severity |
|---|---|---|
| `drive_node._on_cmd`, `drive_logic.limit_wheel_speeds`/`slew`/`command_to_pwm` | A `cmd_vel` with NaN made the wheel target NaN. `slew` did `min(step, nan)`, which returns `step`, so the applied speed ramped UP every tick (and the command stayed fresh), i.e. a NaN twist drove both wheels forward at full duty. Now `sanitize_twist` turns non-finite input into a stop, and `slew` / `command_to_pwm` treat NaN as 0. | high |
| `pico_bridge_logic.send_velocity` | Same NaN went into `V nan nan` (the encoder prints `nan`). Now sanitized to a zero `V`. | high |
| `firmware app._on_velocity`, `_on_cfg` | The Pico accepted `nan`/`inf` in `V` and `CFG` (`float('nan')` parses and passes `isinstance`), same ramp-up as above. Now both are refused (`BAD_ARGS`/`BAD_CFG`) and a bad `V` also zeroes the targets. | high |
| `firmware main.run` | An exception in the loop (or Ctrl-C) ended the script with the RP2040 PWM still running at its last duty, so the wheels kept turning at the last speed. Now `finally: hw.stop_all()`. Needs the board, see HARDWARE_TESTS. | high |
| `firmware app._on_pins` / `control` | Re-sending `PINS` created new decoders at 0 but the controller kept the old raw counts, giving one huge negative tick delta (odometry jump and a speed spike). Now `reset_encoders()` is called. | medium |
| `ultrasonic_node._tick` | A GPIO exception during a ping was turned into "no echo", which is published as max range (clear path) and counts as a fresh reading. Now `try_ping` reports failure and nothing is published, so the sensor goes stale and the behavior treats it as blocked. | medium |

## Open for Bryan (not changed)

| File | Finding | Severity |
|---|---|---|
| `drive_node._tick`, `control.WheelController.update` | Wrong `invert_enc_*` sign means positive feedback: measured speed is the opposite sign of the target, the PI output saturates and that wheel runs away at full duty until the command times out. Nothing detects it. A guard (cut the wheel and warn if duty is high and measured speed has the wrong sign for 0.3 s) is a design addition, so not done. Check the encoder signs first with wheels off the ground (BRINGUP step). | high at first bring-up |
| both drive paths | Stopping is not instant: `cmd_timeout` / Pico watchdog / `STOP` only zero the TARGET and the slew limit (`max_accel_mps2`, 1.0) ramps down. From 0.5 m/s that is 0.5 s after the 0.5 s timeout, so about 0.25 to 0.4 m of travel after the Pi dies or the safety stage says stop. Consider letting stop and watchdog bypass the slew. | medium |
| `sonar` timing | Median-of-3 per sensor plus a round robin of 8 x (60 ms gap + up to 25 ms echo wait) = up to 0.68 s per sensor. A new obstacle needs 2 of 3 readings, so it can take 1 to 1.4 s to show on that sensor; at 0.3 to 0.5 m/s the robot moves 0.4 to 0.7 m first. Options: median window 1 or 2, fire the front pair more often, shorter gap (HC-SR04 can use 30 ms). Choice is Bryan's. | high |
| `measure_echo` / `validate_range` | A sensor with a broken echo wire, echo stuck high or stuck low reads "no echo" = max range = clear and keeps publishing, so it looks healthy. There is no per-sensor health count. `sonar_test` finds it by hand, but nothing flags it at runtime. | high once sonars are fitted |
| `measure_echo` | Python polls the pin value (gpiozero `.value`, tens of microseconds per read) and can be pre-empted, so pulse widths are quantized (about 1 cm) and can only read too long (too far), never too short. The blocking 25 ms wait inside a 5 ms timer callback delays other callbacks of that node only. | low |
| `drive_node` encoders | gpiozero callbacks read the pin level at call time, not the edge level. At high wheel speed, bounce or callback delay makes transitions look like 2-state jumps, which the table counts as 0, so ticks are lost (odometry short, never long). The Pico option avoids this. Compare ticks to the expected count at full speed. | medium |
| `drive_node` startup and shutdown | `stop_motors` sets PWM to 0, then the devices are closed and the pins float. Whether the MDD10A inputs have pull-downs is not stated anywhere in the repo, and a SIGKILL or a Pi crash leaves the pins floating too. Verify on the robot (HARDWARE_TESTS) and add external pull-downs on PWM and DIR if the motors twitch. | medium |
| `drive_node` `max_wheel_speed_mps` | Setting it to 0 disables the speed limit but then `ff = applied / max_speed` divides by zero and the node crashes (safe: `finally` stops the motors). Pico rejects it with `BAD_CFG`. | low |
| `pico_bridge_node._reader` | After a serial read error the thread exits and is never restarted. The node keeps sending (write errors every tick), odometry stops, the Pico watchdog stops the wheels. Safe, but needs a restart of the node after re-plugging the cable. | medium |
| `pico_bridge_node._write_line` | `serial.write` has no `write_timeout`, so a stalled port can block the 20 Hz tick (and the executor). | low |
| `firmware main.run` | `print` on USB CDC can block or drop when the Pi is not reading. The watchdog is only evaluated inside `poll`, so a loop blocked in `print` keeps the last PWM. Needs the cable-pull test on the board. | medium, unverified |
| `firmware main.run` buffer | A line longer than 200 characters drops one character and the rest is parsed as a new line; it fails the checksum (`ERR BAD_LINE`). Fine for the real protocol (all lines are short). The XOR checksum does not catch swapped characters. | low |
| `pico_bridge_node._tick` | `send_rate_hz` below 2 with `pico_cmd_timeout_s` 0.5 makes the Pico watchdog flap. | low |
| `drive_node` PWM | gpiozero software PWM at 1 kHz on a Pi 5 is jittery; MDD10A accepts up to 20 kHz. Hardware PWM or the Pico path would be cleaner. | low |

## Checked and fine

- Quadrature table: A leading B counts +1, all 4 edges counted (x4), invalid two-state jumps count 0; Pi and Pico tables are identical. `ticks_per_rev` is documented as x4 in `robot.yaml`.
- Python ints do not wrap, so encoder counts never overflow. The Pico time counter wraps at 2^30 and `_elapsed`, `poll` and `_on_telemetry` all use modulo arithmetic.
- Differential drive: positive angular (CCW) makes the right wheel faster; odometry uses mid-point heading and normalized theta; units are m, s, rad everywhere.
- `invert_left/right` flips only the direction pin, `invert_enc_*` flips only the counted sign, in both implementations.
- Command timeout: Pi drive (0.5 s) and Pico watchdog (`cmd_timeout_ms`) both zero the target when commands stop; the Pico watchdog clears on the next valid `V`. A PINS/CFG-less Pico holds PWM at 0 and reports `FLAG_UNCONFIGURED`.
- Bridge: `READY` resets the tick baseline and re-sends setup, so a Pico reboot does not jump the odometry. Bad checksums are dropped and logged.
- Sonars are fired one at a time with a settle gap counted after the echo wait; no echo is never reported as 0.
