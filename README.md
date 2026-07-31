# Autonomous Cleaning Robot — ROS 2 Workspace

ROS 2 workspace for an autonomous cleaning robot: a mobile drivebase, a
camera-based trash-detection pipeline, and an **SO-101 follower arm** for
picking up detected litter.

- **ROS 2 Jazzy** on **Ubuntu 24.04**, running on a Raspberry Pi
- Planned hardware: mobile drivebase, SO-101 arm, Raspberry Pi camera,
  ultrasonic sensors, GPS, IMU

---

## Packages

| Package | Type | What it does | Status |
|---|---|---|---|
| [`trash_vision`](src/trash_vision) | `ament_python` | YOLO/Ultralytics trash detector node; publishes detections as `std_msgs/String` | Working, pre-existing |
| [`so101_description`](src/so101_description) | `ament_cmake` | SO-101 arm URDF/Xacro, meshes, `ros2_control` config, RViz + Gazebo launch files | Added for the Week 1 arm milestone |

Each package has its own README with the detail. Start with
[`src/so101_description/README.md`](src/so101_description/README.md) for
anything arm-related.

Not yet written: drivebase description/control, sensor drivers (ultrasonics,
GPS, IMU), and the pick-and-place behaviour tying vision to the arm.

---

## Quick start

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Visualize and drive the arm in simulation:

```bash
# Model only, drag joints with sliders (needs a display)
ros2 launch so101_description display.launch.py

# Controllers on mock hardware — no physics, runs at real time
ros2 launch so101_description bringup.launch.py

# Full Gazebo Sim physics (headless; add gui:=true for the GUI)
ros2 launch so101_description gazebo.launch.py
```

Then, in another terminal, nudge one joint and return it:

```bash
source /ros2_ws/install/setup.bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [shoulder_pan],
  points: [
    {positions: [0.2], time_from_start: {sec: 2}},
    {positions: [0.0], time_from_start: {sec: 4}}
  ]
}"
```

Run the detector:

```bash
ros2 run trash_vision detector
```

---

## Environment notes specific to this Pi

These have bitten us already — read before debugging a build or a slow sim.

**`python3` is `/opt/vision-venv/bin/python3`.** That venv belongs to the
Ultralytics/YOLO install, and colcon/ament use it too. ROS build dependencies
must be pip-installed *into that venv*:

```bash
/opt/vision-venv/bin/pip install catkin_pkg empy==3.3.4 lark
```

Without them `colcon build` fails with `ModuleNotFoundError: No module named
'catkin_pkg'`.

**Gazebo runs at roughly 0.2× real time here.** A 2-second trajectory takes
about 10 wall-clock seconds. Scripted checks should be driven off `/clock` with
`use_sim_time: true` rather than `sleep`. See the `so101_description` README §8
for how this once looked like a controller bug and was not one.

**Headless over SSH.** No `DISPLAY`, so RViz, the Gazebo GUI, and
`joint_state_publisher_gui` cannot run from a terminal session — use a desktop
session for anything visual.

### ROS packages installed beyond the base Jazzy image

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
                 ros-jazzy-ros2controlcli ros-jazzy-rviz2 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-ros-gz-sim ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-bridge
```

---

## Repository hygiene

`build/`, `install/`, `log/`, `runs/` and a stray `core.158` crash dump
currently sit in the working tree, and parts of `log/` appear to be tracked by
git. A `.gitignore` covering those would be worth adding — it has not been done
so as to keep the arm work reviewable in isolation.
