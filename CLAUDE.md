# CLAUDE.md

Guidance for Claude Code when working in this repository.

## What this is

ROS 2 Jazzy workspace for an autonomous cleaning robot (mobile drivebase +
camera trash detection + SO-101 arm), running on a Raspberry Pi / Ubuntu 24.04.

- `src/trash_vision` — YOLO detector node (`ament_python`), pre-existing.
- `src/so101_description` — SO-101 arm description, `ros2_control` config and
  launch files (`ament_cmake`).

Read `README.md` first, then `src/so101_description/README.md` for arm work.

## Environment gotchas

- **`python3` is `/opt/vision-venv/bin/python3`** (the YOLO venv), and colcon
  uses it. ROS build deps (`catkin_pkg`, `empy==3.3.4`, `lark`) must be
  pip-installed into *that venv* or `colcon build` fails with
  `ModuleNotFoundError: No module named 'catkin_pkg'`.
- **No `DISPLAY`** — headless over SSH. RViz, the Gazebo GUI and
  `joint_state_publisher_gui` cannot be verified here. Launch them headless
  (`gui:=false`, `rviz:=false`) and report GUI checks as untested rather than
  claiming they pass.
- **Gazebo runs at ~0.2× real time.** A 2 s trajectory needs ~10 s of wall
  clock. Do not conclude a joint is stuck or clamped from a short `sleep` —
  that already produced one false alarm. Verify against `/clock` or command a
  larger target and check the joint keeps moving.

## Build and test

```bash
cd /ros2_ws
colcon build --symlink-install --packages-select <pkg>
source install/setup.bash
```

Arm validation:

```bash
xacro $(ros2 pkg prefix so101_description)/share/so101_description/urdf/so101.urdf.xacro > /tmp/so101.urdf
check_urdf /tmp/so101.urdf
ros2 control list_controllers          # expect joint_state_broadcaster + arm_controller active
ros2 control list_hardware_interfaces  # expect 6 command interfaces [available] [claimed]
ros2 topic echo /joint_states
```

`/joint_states` orders joints **alphabetically**, not as listed in the YAML:
`elbow_flex, gripper, shoulder_lift, shoulder_pan, wrist_flex, wrist_roll`.

## Conventions and constraints

- **Do not modify `trash_vision`** (camera / trash detection) unless explicitly
  asked.
- **Do not hand-edit `urdf/so101_arm_macro.xacro`.** It is generated from
  upstream `TheRobotStudio/SO-ARM100`. To update, re-import and re-apply the
  three documented transformations (see that package's README §1).
- **Never invent physical parameters** — masses, inertias, joint origins, axes,
  limits, mesh dimensions. Import them from a trustworthy source or leave them
  out. Do not fabricate values to silence a Gazebo warning.
- **The chassis mounting transform is unknown.** It is deliberately zeroed and
  exposed as Xacro args (`mount_x/y/z/roll/pitch/yaw`). Do not guess a real one.
- **Simulation only.** Never send commands to real arm hardware.
- Exactly one `robot_state_publisher` and one controller manager at a time —
  never run two of the three launch files simultaneously.
- Keep changes small and reviewable. Do not commit, push or rewrite git history
  unless explicitly authorized.

## Layout of the arm description

Three Xacro files with separate jobs — preserve this split:

| File | Job |
|---|---|
| `so101_arm_macro.xacro` | geometry/kinematics only; generated from upstream |
| `so101.ros2_control.xacro` | hardware plugin + interfaces; `use_gazebo` switch |
| `so101.urdf.xacro` | composition: `world` link, launch args, calls both macros |

Simulation backends share one controller config, so a trajectory that works on
mock hardware works in Gazebo:

- `use_gazebo:=false` → `mock_components/GenericSystem`, no physics, real time.
- `use_gazebo:=true` → `gz_ros2_control/GazeboSimSystem`, controller manager
  runs inside the Gazebo process.

A real Feetech STS3215 interface should slot into the same `<hardware>` block
without changes elsewhere.
