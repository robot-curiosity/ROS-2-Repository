# Pico wheel controller (option, not the default)

The default drive path is the Pi `drive` node (`wheel_backend:=pi`). This is an
OPTION: the 4-wheel speed loop runs on a Raspberry Pi Pico, which reads the
encoders and drives the two MDD10A channels. The Pi only sends speed targets
and reads ticks. Nothing here changes the Pi drive node.

Each side (left, right) has one PWM, one DIR and one encoder (A and B), the same
as the Pi node. Wheels on the same side are driven together by one channel.

## Files

| File | Where it runs |
|---|---|
| `firmware/pico_wheels/main.py` | Pico (MicroPython): GPIO, PWM, encoder interrupts, serial loop |
| `firmware/pico_wheels/app.py` | Pico and PC tests: protocol handling, setup, telemetry |
| `firmware/pico_wheels/control.py` | Pico and PC tests: quadrature decoder, ramp, PI, watchdog |
| `firmware/pico_wheels/protocol.py` | Pico and PC tests: line format |
| `src/trash_vision/trash_vision/pico_protocol.py` | Pi: exact copy of `protocol.py` (a test fails if they differ; edit the firmware one, then copy it) |
| `src/trash_vision/trash_vision/pico_bridge_logic.py`, `pico_bridge_node.py` | Pi: the `pico_bridge` ROS node |

## Install on the Pico (TODO: Bryan to do on the real board)

1. Flash MicroPython on the Pico.
2. Copy `protocol.py`, `control.py`, `app.py` and `main.py` to the Pico (for example with `mpremote cp firmware/pico_wheels/*.py :`).
3. Plug the Pico into the Pi by USB. Find its port with `ls /dev/ttyACM*` and put it in `config/robot.yaml` under `pico_bridge.serial_port`.
4. Fill in `pico_bridge.pico_pins` (Pico GPIO numbers, see below) and the wheel values.
5. `ros2 launch trash_vision bringup_drive.launch.py wheel_backend:=pico`.

Echo/encoder lines on the Pico are 3.3 V inputs; check the encoder supply voltage before wiring (wiring is Bryan's job).

## Serial protocol (version 1)

ASCII, one line per message, ending in `\n`. Format: `NAME arg arg ...*HH`
where `HH` is the XOR of all characters before the `*`, two hex digits. A line
with a missing or wrong checksum is refused (`ERR BAD_LINE`). Numbers are
decimal. Units: m/s, metres, milliseconds.

### Pi to Pico

| Line | Meaning |
|---|---|
| `PINS pwm_l pwm_r dir_l dir_r enc_a_l enc_b_l enc_a_r enc_b_r` | Set up GPIO. 8 distinct Pico GPIO numbers (0-22, 26-28). |
| `CFG ticks_per_rev wheel_radius_m max_speed_mps max_accel_mps2 min_duty kp ki cmd_timeout_ms invert_left invert_right invert_enc_left invert_enc_right` | Tuning and geometry (booleans as 0/1). |
| `V left right` | Wheel speed targets in m/s. Must repeat faster than `cmd_timeout_ms`. |
| `STOP` | Zero the targets now. |
| `PING` | Pico answers `PONG`. |

### Pico to Pi

| Line | Meaning |
|---|---|
| `READY 1` | Sent after boot. The bridge answers by sending `PINS` and `CFG` again. |
| `T ms ticks_l ticks_r speed_l speed_r flags` | Telemetry every 20 ms. `ms` is the Pico clock modulo 2^30. Ticks are cumulative and already corrected by `invert_enc_*`. Speeds are measured wheel speeds in m/s. |
| `ERR code` | One word: `BAD_LINE`, `UNKNOWN_CMD`, `BAD_ARGS`, `BAD_PINS`, `PIN_CONFLICT`, `BAD_CFG`, `PIN_SETUP_FAILED`, `NOT_CONFIGURED`. |
| `PONG` | Answer to `PING`. |

`flags` bits: 1 = watchdog (no `V` within the timeout, wheels stopped), 2 =
unconfigured (PINS or CFG missing, wheels stopped), 4 = fault (GPIO setup
failed).

### Safety behaviour

- Before `PINS` and `CFG` arrive, or after a GPIO failure, PWM stays at zero.
- If no `V` line arrives within `cmd_timeout_ms` (`pico_cmd_timeout_s`), the Pico stops the wheels by itself and sets flag 1. The next `V` clears it. This covers a dead Pi or an unplugged cable.
- The control loop is the same as the Pi node: feed-forward plus PI per side, acceleration limit, both wheels scaled together to the speed limit.
- The bridge also sends zero speed when `/cmd_vel` is older than `cmd_timeout_s`.
- Not covered by the Pico: the behavior safety stage still runs on the Pi and still publishes `/cmd_vel`.

## ROS interface (same as the drive node)

`/cmd_vel` (Twist) in, `/odom` (Odometry) out. Parameters are the `pico_bridge`
section of `config/robot.yaml`. Do not run `pico_bridge` and `drive` together
(both would publish `/odom`); the launch argument `wheel_backend` starts one.
