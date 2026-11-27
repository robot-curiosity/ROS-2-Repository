# Review pass 4: wiring of the whole package (2026-11-24)

Scope: `setup.py`, `package.xml`, launch files, `config/robot.yaml`, `hardware_check`,
calibration tools, `docs/BRINGUP.md`. Method: static cross-check (AST, no ROS needed),
guarded by `test/test_wiring.py`. Nothing here was run on the robot.

## Fixed in this pass

| File | Finding | Severity |
|---|---|---|
| `detector_node.py`, `ultrasonic_node.py` | Internal node names were `trash_detector` and `ultrasonic_array`, but the yaml sections are `detector` and `ultrasonic`. Launch files rename the node so they worked, but `ros2 run trash_vision detector --ros-args --params-file robot.yaml` silently ignored the whole section. Now named `detector` and `ultrasonic`. Topics are unchanged. | medium |
| `trial_summary.py` | `--help` was treated as a log path and printed "No trial logs found in: --help". Now uses argparse. | low |

## Checked and fine

- Every key in `robot.yaml` is declared by the node of its section (behavior also counts `behavior_logic.DEFAULTS` and `safety_logic.DEFAULTS`; `pose_*` counts via the `pose_` prefix).
- Every hardware parameter the drive, pico_bridge, ultrasonic, arm and gps_imu nodes declare is present in the yaml.
- All 15 `setup.py` entry points resolve to a real function; every `ros2 run` / `ros2 launch` name in `docs/BRINGUP.md` exists.
- `hardware_check` and the five calibration tools print `--help` without ROS or GPIO (imports are lazy).
- `package.xml` covers the imports used by the nodes (`std_srvs` for Trigger, `nav_msgs`, `sensor_msgs`, `geometry_msgs`).

## Topic table

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/cmd_vel` | Twist | behavior (also `calibrate_drive`) | drive or pico_bridge, sim |
| `/odom` | Odometry | drive or pico_bridge, sim | behavior, `calibrate_drive` |
| `/vision/target` | String (JSON) | detector, sim | behavior, arm, `calibrate_pick_zone`, `calibrate_camera` |
| `/vision/fps`, `/vision/direction` | Float32, String | detector | none (diagnostics) |
| `/ultrasonic/<8 names>` | Range | ultrasonic, sim | behavior, `sonar_test` |
| `/arm/joint_states` | JointState | arm | none (diagnostics) |
| `/arm/pick`, `/arm/deposit`, `/arm/stow` | Trigger service | arm, sim | behavior |
| `/behavior/state` | String | behavior | none |
| `/gps/fix`, `/imu/data` | NavSatFix, Imu | gps_imu | none yet |

All publishers and subscribers use the default reliable QoS with depth 10, so none are incompatible.

## Open for Bryan (not changed)

- Behavior `bringup.launch.py` always starts `behavior` (disabled by default). Fine, but `use_behavior:=false` does not exist.
- `/gps/fix` and `/imu/data` have no consumer, so GPS/IMU wiring is not exercised by the behavior yet.
- `package.xml` licence and maintainer are still `TODO` (kept on purpose, `test_package_xml` expects it).
- `calibrate_pick_zone` and `calibrate_camera` read `/vision/target` as JSON with the `String` type; this changes if you ever decide on a custom detector message (blocked item).

## Hardware tests

See the 2026-11-24 "wiring review" row in `HARDWARE_TESTS.md` (renamed nodes with `--params-file`).
