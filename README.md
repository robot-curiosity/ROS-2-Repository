# Curiosity

Autonomous litter-collection robot (LACSEF/ISEF 2026): Raspberry Pi 5, ROS 2
Jazzy, SO101 arm, YOLOv8 detection, 8 HC-SR04 sonars, GPS/IMU, 12 V drive motors.

Branches: `master` is code tested on the real robot, `dev` is where new work lands.
Clone `dev` to try new work, `master` for the tested version.

## Quick start on the Pi

Run each block on the Pi 5 (Ubuntu 24.04). Pins and hardware values are not set yet,
so every node idles with a warning until you fill in `config/robot.yaml`.

1. Install ROS 2 Jazzy (see https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then:

       sudo apt update
       sudo apt install ros-jazzy-ros-base python3-colcon-common-extensions \
           python3-rosdep python3-pip python3-pytest

2. Python dependencies (apt for the GPIO and serial parts, pip for the rest):

       sudo apt install python3-gpiozero python3-lgpio python3-serial python3-yaml \
           python3-opencv python3-requests python3-numpy python3-smbus2
       pip install --break-system-packages ultralytics lerobot

   `ultralytics` (YOLOv8), `lerobot` (SO101 arm) and `smbus2` (IMU) have no rosdep
   key; versions are not confirmed on the robot (TODO: Bryan to confirm).

3. Clone and build:

       mkdir -p ~/curiosity_ws/src && cd ~/curiosity_ws/src
       git clone -b dev https://github.com/robot-curiosity/ROS-2-Repository.git
       cd ~/curiosity_ws
       source /opt/ros/jazzy/setup.bash
       colcon build --packages-select trash_vision
       source install/setup.bash

   Add the two `source` lines to `~/.bashrc` if you want them in every shell.

4. Check the setup (reads `src/trash_vision/config/robot.yaml`):

       ros2 run trash_vision hardware_check

   Unfilled subsystems print `[SKIP]`; a wrong or duplicate pin prints `[FAIL]`.
   Check one part with `ros2 run trash_vision hardware_check sonar`
   (also `drive`, `arm`, `camera`, `imu_gps`).

5. Try it without hardware:

       ros2 launch trash_vision sim.launch.py

After editing `robot.yaml`, run `colcon build --packages-select trash_vision` and
`source install/setup.bash` again.

## Where to find things

- `docs/BRINGUP.md`: the order to bring the robot up, step by step. Start here.
- `src/trash_vision/config/robot.yaml`: the only file with hardware values (pins, ports, dimensions).
- Calibration tools (each prints a line to paste into `robot.yaml`):
  `calibrate_encoders`, `calibrate_drive`, `calibrate_pick_zone`, `calibrate_camera`,
  `sonar_test`. Run with `ros2 run trash_vision <name>`.
- Launch files: `bringup.launch.py` (everything), `bringup_drive`, `bringup_sonar`,
  `bringup_arm`, `bringup_camera`, `bringup_imu_gps`, and `sim.launch.py`.
  Use `ros2 launch trash_vision <file>`.
- `ros2 run trash_vision trial_summary`: pickup rate from trial logs; plan in `docs/TRIAL_PLAN.md`.
- `ros2 run trash_vision dashboard`: live web page (sonars, wheels, state, detections, safety stops) on port 8080 of the Pi; read-only unless started with `--allow-write` (no password, trusted networks only).
- `python3 tools/status.py`: hardware test progress and unset config values, from the repo only.
- `docs/HARDWARE_TESTS.md`: tests to run on the robot and their results.
- `tools/record_result.py`: `next` shows the next untested row; `pass <n> "note"` or
  `fail <n> "note"` records it (commit and push the file so the agent sees it).
- `docs/PIN_BUDGET.md`, `docs/BOM.md`, `docs/PICO_PROTOCOL.md`, `docs/DECISIONS.md`, `docs/TODO.md`.

## Tests on a PC

    sudo apt install python3-pytest
    PYTHONPATH=src/trash_vision python3 -m pytest src/trash_vision/test \
        -k "not flake8 and not pep257 and not copyright"
