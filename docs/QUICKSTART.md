# Curiosity quickstart

One page for the Pi. Details: [README.md](../README.md), [BRINGUP.md](BRINGUP.md).

**Safety: wheels off the ground, fuse in the 12 V line, hand on the power, E-stop ready
(Space in `bench` is software only, not a replacement for the power switch).**

## 1. Setup

Install the ROS 2 Jazzy apt source first ([README.md](../README.md), step 1), then:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep \
    python3-pip python3-pytest python3-gpiozero python3-lgpio python3-serial \
    python3-yaml python3-opencv python3-requests python3-numpy python3-smbus2
pip install --break-system-packages ultralytics lerobot
mkdir -p ~/curiosity_ws/src && cd ~/curiosity_ws/src
git clone -b dev https://github.com/robot-curiosity/ROS-2-Repository.git
cd ~/curiosity_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select trash_vision
source install/setup.bash
```

In every new terminal run the two `source` lines from `~/curiosity_ws`.

## 2. I want to... / run this

Add `--help` to a tool (not a node) for its options. `python3 tools/...` commands run
from `~/curiosity_ws/src/ROS-2-Repository`.

| I want to... | Run this | Needs first |
|---|---|---|
| check `robot.yaml` | `ros2 run trash_vision hardware_check` (or add `sonar`, `drive`, `arm`, `camera`, `imu_gps`) | nothing |
| test hardware directly | `ros2 run trash_vision bench sonar` (or `arm`, `camera`, `all`) | nodes stopped |
| test motors | `ros2 run trash_vision bench drive --wheels-off` | nodes stopped, Pi backend |
| check sonar names and swaps | `ros2 run trash_vision sonar_test` | `bringup_sonar.launch.py` running |
| get encoder ticks per rev | `ros2 run trash_vision calibrate_encoders --side left` (then `right`) | drive node stopped |
| check driven distance and turn | `ros2 run trash_vision calibrate_drive` | wheel driver running |
| tune wheel speed gains | `ros2 run trash_vision tune_wheels` | wheels off, driver running, no other `/cmd_vel` |
| fit camera pitch | `ros2 run trash_vision calibrate_camera --distance 1.0` | detector running |
| set the pick zone | `ros2 run trash_vision calibrate_pick_zone` | detector running |
| start everything (behavior off) | `ros2 launch trash_vision bringup.launch.py` | filled values |
| let it move | `ros2 launch trash_vision bringup.launch.py enable_behavior:=true` | BRINGUP steps 0-8 pass |
| leave a part out | add `use_arm:=false` (also `use_drive`, `use_ultrasonic`, `use_detector`, `use_gps_imu`) | |
| start one subsystem | `ros2 launch trash_vision bringup_drive.launch.py` (also `bringup_sonar.launch.py`, `bringup_arm.launch.py`, `bringup_camera.launch.py`, `bringup_imu_gps.launch.py`) | filled values |
| use the Pico wheel loop | add `wheel_backend:=pico` to `bringup.launch.py` or `bringup_drive.launch.py` | `pico_bridge:` values, [PICO_PROTOCOL.md](PICO_PROTOCOL.md) |
| run the sim (no hardware) | `ros2 launch trash_vision sim.launch.py` | nothing |
| watch it live | `ros2 run trash_vision dashboard`, open `http://<pi-address>:8080` | nodes running |
| change settings from the page | `ros2 run trash_vision dashboard --allow-write` (no password, trusted network only) | nodes running |
| record a run | `ros2 run trash_vision record_trial` (Enter = mark, Ctrl-C = stop) | nodes running |
| measure a recorded run | `ros2 run trash_vision analyze_run <run folder>` | a recording |
| replay its decisions | `ros2 run trash_vision replay_run <run folder>` (`--at`, `--set name=value`) | a recording |
| see pickup rate | `ros2 run trash_vision trial_summary` | trial logs |
| try behavior settings in sim | `ros2 run trash_vision sweep_sim` or `python3 tools/sweep_sim.py` | nothing (sim only) |
| tune from the repo | `python3 tools/tune_wheels.py` (`--analyze <file.csv>` needs no robot) | as `tune_wheels` |
| see progress and unset values | `python3 tools/status.py` | nothing |
| record a hardware test | `python3 tools/record_result.py next`, then `python3 tools/record_result.py pass <n> "note"` or `fail` | |

Single nodes (the launch files start these for you): `ros2 run trash_vision drive`,
`ros2 run trash_vision pico_bridge`, `ros2 run trash_vision ultrasonic`,
`ros2 run trash_vision detector`, `ros2 run trash_vision arm`,
`ros2 run trash_vision gps_imu`, `ros2 run trash_vision behavior`,
`ros2 run trash_vision sim`. Run this way they do not read `robot.yaml`.

## 3. First day (full steps in [BRINGUP.md](BRINGUP.md))

0. Power and fuse: multimeter shows about 12 V at the drivers, 5 V at the Pi.
1. Sonars: fill pins, `hardware_check`, start `bringup_sonar.launch.py`, `sonar_test`.
2. Motors, wheels off: fill PWM and DIR pins, both sides turn forward and stop.
3. Encoders: fill pins, `calibrate_encoders`, odometry x increases going forward.
4. Drive calibration: wheel radius and track width, `calibrate_drive` on a clear floor.
5. Camera: model and stream, `calibrate_camera`, `calibrate_pick_zone`.
6. Arm: port, limits and poses, tuned at low speed with a hand on the arm power.
7. IMU and GPS: `i2cdetect -y 1`, topics publish.
8. Everything with behavior off: nothing moves, `/behavior/state` publishes.
9. Behavior on: wheels off first, then the floor in an open area.

## 4. Where values go and where results go

- Values: only `~/curiosity_ws/src/ROS-2-Repository/src/trash_vision/config/robot.yaml`.
  Pins are BCM numbers. After editing, from `~/curiosity_ws`:
  `colcon build --packages-select trash_vision && source install/setup.bash`, then restart nodes.
- Calibration tools only print lines; paste them into `robot.yaml` yourself.
  With the Pico, put `ticks_per_rev`, radius and track in `pico_bridge:`. `calibrate_drive`
  still checks radius and track in `drive:`, and `calibrate_encoders` only reads encoders
  wired to the Pi (`drive:` pins).
- Pickup logs: `~/curiosity_trials`. Recordings: `~/curiosity_runs`.
  Wheel tuning CSV and plot: `~/curiosity_logs/wheel_tune`.
- Test results: [HARDWARE_TESTS.md](HARDWARE_TESTS.md). Commit and push it so the agent sees it.

## 5. Something is wrong

| Symptom | Likely cause | Run / do |
|---|---|---|
| node warns it is idle | a value it needs is unset | `ros2 run trash_vision hardware_check`, fill it |
| `[FAIL]` from the check | wrong or duplicate pin or value | read the reason, see [PIN_BUDGET.md](PIN_BUDGET.md) |
| edits have no effect | not rebuilt | rebuild and source (section 4), restart |
| GPIO busy | a node still holds the pins | stop the launch, then `bench` or `calibrate_encoders` |
| `sonar_test` all FAIL | sonar node not running, or echo wiring | `ros2 launch trash_vision bringup_sonar.launch.py`, `ros2 run trash_vision bench sonar` |
| no `/odom` | wheel driver not running or idle | `ros2 launch trash_vision bringup_drive.launch.py`, read its warnings |
| wheel turns backwards | invert flag | `invert_left` / `invert_right` |
| no detections | stream down or wrong `model_path` | `ros2 run trash_vision bench camera` |
| serial port permission denied | user not in `dialout` | `sudo usermod -aG dialout $USER`, log in again |
| robot will not move | behavior off, or safety stop (no sonars, obstacle) | `ros2 topic echo /behavior/state`, `ros2 topic echo /behavior/safety` |
| never reaches PICK | pick zone not calibrated | `ros2 run trash_vision calibrate_pick_zone` |
