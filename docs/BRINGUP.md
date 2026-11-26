# Curiosity bring-up guide

Do the steps in order. Each step: wire, fill in `src/trash_vision/config/robot.yaml`
(the only file with hardware values), run, check PASS. After editing the yaml
run `colcon build --packages-select trash_vision` and `source install/setup.bash`
so the Pi uses the new values. Pins are BCM numbers 2..27. A pin plan will be in
`docs/PIN_BUDGET.md` once it exists (item 23); until then pick pins yourself and
let `hardware_check` catch duplicates.

## Safety (every step)

- Wheels OFF the ground (robot on a stand) until step 5 says otherwise.
- Fuse in the 12 V battery line. Check polarity and fuse before connecting the pack.
- Arm clear of people, bag hoop and wiring. Keep the arm at stow.
- A hand on the power switch / battery connector whenever motors or arm can move.
- `behavior.enabled` stays `false` until step 9.

## Bench test (any step, nodes stopped)

`ros2 run trash_vision bench <sonar|drive|arm|camera|all>` opens the hardware directly from
`robot.yaml`. Space bar = software E-stop (not a replacement for the power switch).

- `drive --wheels-off`: asks before each side, runs it forward for 2 s at 20 % duty (Pi GPIO backend only).
- `arm`: number picks a joint, `+`/`-` moves it 2 degrees, `q` ends. Hold the arm before it disconnects.
- `sonar`: live distance per sensor, `q` ends.
- `camera`: 5 frames as a text picture plus detections with `base_x`, `base_y`.

## 0. Power and fuse check

- Wire: 12 V pack -> fuse -> switch -> motor drivers; Pi on its own regulated 5 V.
- Run: multimeter on the fused rail with the switch on.
- PASS: about 12 V at the drivers, 5.0 V (+-0.25) at the Pi, no heat or smell.
- Common failure: reversed polarity or a missing common ground between Pi and drivers. Switch off and fix before going on.

## 1. Sonars

- Wire: 8 HC-SR04, trig and echo each to a GPIO. Echo is 5 V: use a divider or level shifter on every echo pin (Pi pins are 3.3 V only). Sensor order: front_1, front_2, rear_1, rear_2, left_1, left_2, right_1, right_2.
- Fill in: `ultrasonic.trig_pins`, `ultrasonic.echo_pins` (-1 = not fitted).
- Run: `ros2 run trash_vision hardware_check sonar`, then `ros2 run trash_vision sonar_test`, then `ros2 launch trash_vision bringup_sonar.launch.py`.
- PASS: `[PASS] sonar`; `sonar_test` shows each named sensor changing when you put a hand in front of it, none stuck or swapped.
- Common failure: no echo or stuck at max range (divider wrong, trig/echo swapped), or two sensors swapped in the list.

## 2. Motors (wheels off the ground)

- Wire: 2x MDD10A, one PWM and one DIR pin per side; motors on the driver outputs.
- Fill in: `drive.pwm_pins`, `drive.dir_pins`, `drive.invert_left`, `drive.invert_right`.
- Run: `ros2 launch trash_vision bringup_drive.launch.py`, then in a second terminal `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"`.
- PASS: both wheels turn forward slowly; Ctrl-C or stopping the publisher stops them within 0.5 s.
- Common failure: a wheel turns backwards (flip its `invert_*`); a wheel does not start (raise `min_duty`).

## 3. Encoders

- Wire: A and B channel per wheel to GPIO (encoders usually need 3.3 V supply, not 5 V).
- Fill in: `drive.enc_a_pins`, `drive.enc_b_pins`, then `drive.ticks_per_rev` from the tool below.
- Run: `ros2 run trash_vision calibrate_encoders` (spin each wheel exactly 10 turns by hand; paste the printed line into the yaml). Then `ros2 run trash_vision hardware_check drive`.
- PASS: `[PASS] drive`; with the wheels spinning forward (step 2 command) `ros2 topic echo /odom` shows x increasing.
- Common failure: odom x decreases when the wheel rolls forward (set `invert_enc_left` / `invert_enc_right`); ticks look 4x off (count after the gearbox, per wheel rev).

## 4. Drive calibration

- Fill in: `drive.wheel_radius_m`, `drive.track_width_m` (CAD 0.060 and 0.481 are not measured), `kp`, `ki` if the speed oscillates.
- Run: `ros2 run trash_vision calibrate_drive`, put the robot on the floor only when the tool asks, clear 2 m ahead, hand on the power. Paste the printed corrections.
- PASS: commanded distance matches the tape measure within about 5 %, and a commanded 360 degree turn comes back within about 10 degrees.
- Common failure: drives a curve (one wheel radius or one invert flag is wrong); turns too little or too much (track width).

## 5. Camera and detector

- Wire: Camera Module 3 on the Pi, stream running at `detector.camera_source`.
- Fill in: `detector.model_path`, `detector.camera_source`, then measure `camera_x_m`, `camera_y_m`, `camera_height_m`, `camera_pitch_deg` (CAD values are not measured).
- Run: `ros2 run trash_vision hardware_check camera`, `ros2 launch trash_vision bringup_camera.launch.py`, `ros2 topic echo /vision/target`. Put a marker at a known distance and run `ros2 run trash_vision calibrate_camera`; then a can at the arm pick spot and `ros2 run trash_vision calibrate_pick_zone` (sets `behavior.pick_floor_y` and `pick_center_tolerance`).
- PASS: `[PASS] camera`; a can in view gives a target with `base_x`, `base_y` close to the tape measure (within about 3 cm at the pick spot).
- Common failure: no frames (wrong `camera_source`, stream not running); `base_x` consistently off (pitch or height wrong).

## 6. Arm

- Wire: SO101 on USB, own supply. Arm clear, hand on the arm power.
- Fill in: `arm.port` (`ls /dev/ttyACM*`), then joint limits, gripper open/closed and the five poses. They are placeholders: tune each one at low speed.
- Run: `ros2 launch trash_vision bringup_arm.launch.py`; command stow, pre_grasp, grasp, lift, deposit one at a time via the arm node's pick service/topics.
- PASS: arm reaches each pose without hitting the frame, gripper closes on a can and the can stays held.
- Common failure: wrong port or permission denied (add user to `dialout`); a pose that collides, lower `max_step` and re-tune.

## 7. IMU and GPS

- Wire: BNO055 on I2C (GPIO2/3), GPS on a UART (GPIO14/15) or USB. Keep sonar/motor pins off those GPIOs.
- Fill in: `gps_imu.gps_port`, `gps_imu.imu_model` (`bno055`), `imu_i2c_bus`, `imu_i2c_addr`.
- Run: `i2cdetect -y 1`, `ros2 run trash_vision hardware_check imu_gps`, `ros2 launch trash_vision bringup_imu_gps.launch.py`, `ros2 topic echo /imu/data` and `ros2 topic echo /gps/fix`.
- PASS: `[PASS] imu_gps`; IMU orientation changes when you rotate the robot; GPS fix outdoors.
- Common failure: address not in `i2cdetect` (wiring or address); no GPS fix indoors (take it outside).

## 8. Behavior, disabled

- Run: `ros2 launch trash_vision bringup.launch.py` (wheels off the ground), `ros2 topic echo /behavior/state`.
- PASS: all nodes start, no crash, robot does not move, state is published.
- Common failure: a node idles with a warning naming the unset parameter: fill it in, or start without it (`use_arm:=false`).

## 9. Behavior, enabled

- Run: `ros2 launch trash_vision bringup.launch.py enable_behavior:=true`, wheels off the ground first, then on the floor in an open area with a can and a hand on the power.
- PASS: SEARCH spins slowly, APPROACH turns to the can and drives to it, PICK/DEPOSIT/STOW cycle, a hand in front of a front sensor stops or turns the robot, Ctrl-C stops the motors.
- Common failure: spins away from the can (camera x sign or `invert_*`); never reaches PICK (re-run `calibrate_pick_zone`).
