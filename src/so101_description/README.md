# so101_description

URDF/Xacro description, meshes, `ros2_control` configuration and launch files for
the **SO-101 follower arm** used on the autonomous cleaning robot.

This package covers the arm only. It does not contain the mobile drivebase, the
camera, or the trash-detection pipeline (see `trash_vision`).

---

## 1. Upstream source and license

The kinematic model and all meshes are imported from the official SO-ARM100
repository — **nothing in the kinematics was authored by hand**.

| | |
|---|---|
| Upstream repo | <https://github.com/TheRobotStudio/SO-ARM100> |
| File used | `Simulation/SO101/so101_new_calib.urdf` |
| Meshes | `Simulation/SO101/assets/*.stl` (13 files, copied verbatim to `meshes/`) |
| License | Apache-2.0 — full text in [`LICENSE`](LICENSE) |
| Provenance | Generated upstream by `onshape-to-robot` from the official Onshape CAD |

**Why the "new calib" variant:** upstream ships two calibrations. `new_calib`
places each joint's zero at the *middle of its range*, which matches the current
LeRobot convention. `old_calib` places zero at the fully-extended horizontal
pose. If you ever switch, re-import — do not hand-edit offsets.

### Local modifications to the upstream URDF

Only three, all mechanical and reviewable:

1. Mesh paths rewritten from relative `assets/…` to
   `package://so101_description/meshes/…` (upstream deliberately uses relative
   paths; ROS needs package-relative ones).
2. Legacy `<transmission>` blocks removed — they are ROS 1 / `transmission_interface`
   era and are superseded by the `<ros2_control>` block in
   `urdf/so101.ros2_control.xacro`.
3. The whole model wrapped in a Xacro macro with a configurable mounting joint.

**Link names, joint names, origins, axes, joint limits, inertials and collision
geometry are byte-for-byte upstream.** No physical parameter has been invented.
If Gazebo ever warns about inertia or collisions, fix it upstream or with a
documented, sourced value — do not fabricate numbers to silence the warning.

The conversion was scripted rather than done by hand. To re-import a future
upstream revision, re-run that transformation rather than patching the generated
file; the three rules above are the entire delta.

---

## 2. Package layout

```
so101_description/
├── CMakeLists.txt                    # ament_cmake; installs all data dirs
├── package.xml
├── LICENSE                           # Apache-2.0, from upstream
├── meshes/                           # 13 STLs, verbatim upstream
├── urdf/
│   ├── so101_arm_macro.xacro         # the arm itself (generated from upstream)
│   ├── so101.ros2_control.xacro      # ros2_control block + Gazebo plugin tag
│   └── so101.urdf.xacro              # top-level entry point; world link + args
├── config/
│   └── so101_controllers.yaml        # controller_manager + arm_controller params
├── launch/
│   ├── display.launch.py             # RViz + slider GUI, no controllers
│   ├── bringup.launch.py             # mock hardware + controllers (fast)
│   └── gazebo.launch.py              # Gazebo Sim physics + controllers
└── rviz/
    └── so101.rviz
```

### The three URDF files and why they are separate

| File | Responsibility |
|---|---|
| `so101_arm_macro.xacro` | Pure geometry/kinematics. Defines `xacro:macro so101_arm`. Regenerated from upstream; **avoid hand-editing**. |
| `so101.ros2_control.xacro` | Everything control-related: which hardware plugin, which joints expose which interfaces. Defines `xacro:macro so101_ros2_control`. |
| `so101.urdf.xacro` | Composition layer. Creates a `world` link, calls both macros, exposes launch-time args. This is the file the launch files pass to `xacro`. |

This split is deliberate: mounting the arm on the future chassis means the
chassis description calls `so101_arm` directly and `so101.urdf.xacro` becomes
unnecessary — without touching the model or the control config.

---

## 3. Joints

Six actuated revolute joints, upstream names preserved. Limits are upstream
values, in radians.

| Joint | Parent → child | Axis | Lower | Upper | Effort | Velocity |
|---|---|---|---|---|---|---|
| `shoulder_pan`  | `base_link` → `shoulder_link` | Z | −1.91986 | 1.91986 | 10 | 10 |
| `shoulder_lift` | `shoulder_link` → `upper_arm_link` | Z | −1.74533 | 1.74533 | 10 | 10 |
| `elbow_flex`    | `upper_arm_link` → `lower_arm_link` | Z | −1.69 | 1.69 | 10 | 10 |
| `wrist_flex`    | `lower_arm_link` → `wrist_link` | Z | −1.65806 | 1.65806 | 10 | 10 |
| `wrist_roll`    | `wrist_link` → `gripper_link` | Z | −2.74385 | 2.84121 | 10 | 10 |
| `gripper`       | `gripper_link` → `moving_jaw_so101_v1_link` | Z | −0.174533 | 1.74533 | 10 | 10 |

Plus two fixed joints: `so101_mount_joint` (parent → `base_link`, added by us)
and `gripper_frame_joint` (`gripper_link` → `gripper_frame_link`, upstream TCP
frame — the natural target frame for future grasp planning).

> The effort/velocity limits of `10`/`10` are uniform upstream placeholders, not
> measured STS3215 values. Fine for simulation; **re-derive from the motor
> datasheet before driving real hardware.**

Kinematic chain:

```
world → base_link → shoulder_link → upper_arm_link → lower_arm_link
      → wrist_link → gripper_link → ├─ moving_jaw_so101_v1_link
                                    └─ gripper_frame_link
```

---

## 4. Mounting the arm on the chassis

The mounting transform is **not guessed anywhere in this package** — the
defaults are all zero and are placeholders.

Quick override at launch time:

```bash
ros2 launch so101_description bringup.launch.py   # defaults, arm at world origin
xacro urdf/so101.urdf.xacro mount_x:=0.12 mount_z:=0.30 mount_yaw:=1.5708
```

Available args: `mount_x`, `mount_y`, `mount_z`, `mount_roll`, `mount_pitch`,
`mount_yaw`, plus `use_ros2_control` and `use_gazebo`.

**Preferred approach once the chassis exists** — call the macro from the chassis
description with the real transform, and stop using `so101.urdf.xacro`:

```xml
<xacro:include filename="$(find so101_description)/urdf/so101_arm_macro.xacro"/>
<xacro:include filename="$(find so101_description)/urdf/so101.ros2_control.xacro"/>

<xacro:so101_arm parent="chassis_link">
  <origin xyz="0.12 0.0 0.30" rpy="0 0 1.5708"/>   <!-- measured, not guessed -->
</xacro:so101_arm>

<xacro:so101_ros2_control name="so101_arm" use_gazebo="true"/>
```

---

## 5. Simulation backends

Two backends, selected by the `use_gazebo` Xacro arg. Both use the identical
controller configuration, so a trajectory that works in one works in the other.

| | `use_gazebo:=false` (default) | `use_gazebo:=true` |
|---|---|---|
| Plugin | `mock_components/GenericSystem` | `gz_ros2_control/GazeboSimSystem` |
| Physics | none — commands echoed back as states | full Gazebo Sim (Harmonic) |
| Controller manager | standalone `ros2_control_node` | runs **inside** the Gazebo process |
| Speed on this Pi | real time | ~0.2× real time (see §8) |
| Use it for | controller/trajectory logic, RViz, CI | contact, gravity, gripping |

A third backend — a real Feetech STS3215 hardware interface — slots into the
same `<hardware>` block later. Nothing outside `so101.ros2_control.xacro`
should need to change.

### Controllers (`config/so101_controllers.yaml`)

- `joint_state_broadcaster` → publishes `/joint_states` at 50 Hz.
- `arm_controller` (`JointTrajectoryController`) → all six joints, position
  command interface, position+velocity state interfaces.

`allow_partial_joints_goal: true` is set so a trajectory may name a subset of
joints (used by the single-joint smoke test in §7). `update_rate` is 100 Hz.

All six joints including `gripper` are in one controller — simplest thing that
works. If the gripper later needs independent action-based control, split it
into a second controller; the joint would move out of `arm_controller`'s
`joints:` list.

---

## 6. Build and launch

```bash
cd /ros2_ws
colcon build --symlink-install --packages-select so101_description
source install/setup.bash
```

| Goal | Command |
|---|---|
| Look at the model, drag joints by hand | `ros2 launch so101_description display.launch.py` |
| Controllers, no physics (fast) | `ros2 launch so101_description bringup.launch.py` |
| …with RViz | `ros2 launch so101_description bringup.launch.py rviz:=true` |
| Gazebo physics, headless | `ros2 launch so101_description gazebo.launch.py` |
| …with the Gazebo GUI | `ros2 launch so101_description gazebo.launch.py gui:=true` |

`display.launch.py` and `bringup.launch.py` each start their own
`robot_state_publisher`; `gazebo.launch.py` starts one too. **Never run two of
these at once** — you get duplicate state publishers and, worse, two controller
managers fighting over the same joints.

`gazebo.launch.py` appends the package's parent directory to
`GZ_SIM_RESOURCE_PATH`. This is required: sdformat rewrites `package://` mesh
URIs to `model://`, and without that path Gazebo cannot find the STLs (the arm
spawns invisible but physically present — a confusing failure mode).

---

## 7. Verification

The `ros2 control` checks need `bringup.launch.py` or `gazebo.launch.py`
running; `display.launch.py` starts no controllers, so only the model and TF
checks apply there.

```bash
# Model parses, and the TF tree has no disconnected links (no launch needed)
xacro $(ros2 pkg prefix so101_description)/share/so101_description/urdf/so101.urdf.xacro > /tmp/so101.urdf
check_urdf /tmp/so101.urdf

# Controllers are loaded and active
ros2 control list_controllers
ros2 control list_hardware_interfaces      # all 6 should be [available] [claimed]

# State is flowing
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo world gripper_frame_link

# Trajectory action endpoint
ros2 action list | grep follow_joint_trajectory
```

### Safe single-joint smoke test (simulation only)

Moves `shoulder_pan` by 0.2 rad (≈11°, well inside its ±1.92 rad limit) and
returns it to zero. **Never point this at real hardware.**

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [shoulder_pan],
  points: [
    {positions: [0.2], time_from_start: {sec: 2}},
    {positions: [0.0], time_from_start: {sec: 4}}
  ]
}"
```

Watch it with `ros2 topic echo /joint_states --field position` in another
terminal. Joint order in that array is **alphabetical**, not the order in the
YAML: `elbow_flex, gripper, shoulder_lift, shoulder_pan, wrist_flex, wrist_roll`
— so `shoulder_pan` is index **3**.

Under Gazebo, allow ~5× the wall-clock time for this to finish (see below).

---

## 8. Known quirks on this machine

**Gazebo runs at ~0.2× real time on the Raspberry Pi.** Measured by comparing
`/clock` against wall clock. A trajectory with `time_from_start: {sec: 2}`
therefore takes ~10 s of wall time. This caused an early false alarm: sampling
`/joint_states` a few wall-seconds after publishing showed the joint apparently
"stuck" partway, which looked like a position clamp but was just mid-trajectory
in sim time. Commanding a larger target and seeing it continue past the previous
"stuck" value confirmed correct tracking.

If you script assertions against Gazebo, drive them off `/clock` (with
`use_sim_time: true`), not `sleep`.

The main cost is the 13 STL **collision** meshes. If simulation speed becomes a
problem, replace collision geometry with primitives — but keep the visual meshes,
and note that upstream already removed the base collision mesh due to solver
problems.

**`python3` is `/opt/vision-venv/bin/python3`** on this Pi (it belongs to the
YOLO/Ultralytics setup), and colcon uses it. It needs `catkin_pkg`, `empy` and
`lark` installed *into that venv* or `colcon build` fails with
`ModuleNotFoundError: No module named 'catkin_pkg'`.

**Headless.** There is no `DISPLAY` over SSH, so RViz and the Gazebo GUI cannot
be verified from a terminal session; run them from a desktop session.

---

## 9. Not done yet

- RViz visual appearance has never been eyeballed (headless machine). Everything
  underneath it — model parsing, TF chain, mesh resolution — is verified.
- Effort/velocity limits are upstream placeholders, not STS3215 datasheet values.
- No real-hardware interface yet; simulation only.
- The mounting transform to the chassis is unknown and deliberately zeroed.
- The gripper shares `arm_controller` with the arm joints.
