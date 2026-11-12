# Repo review: this repo against the simulation results

Reviewed 2026-11-11 at commit `1152e38` (one commit, 2026-07-29, "Initial commit!"). The ROS code that runs on the robot lives on the Pi; this review covers only what is in git.

## What is in the repo

| Path | What it is |
|---|---|
| `src/trash_vision/` | The only package. One node, `detector` (`trash_vision.detector_node:main`) |
| `build/`, `install/`, `log/` | colcon build output, committed by mistake |
| `core.158` | A 24.8 MB crash dump of `turtlesim_node` from ROS 2 Jazzy on aarch64, committed by mistake |
| `runs/detect/predict/camera-test.jpg` | One test image from a YOLO run |
| `src/trash_vision/trash_vision/__pycache__/` | Compiled Python, committed by mistake |

The committed files show Python 3.12 and `/opt/ros/jazzy`, which confirms Ubuntu 24.04 with ROS 2 Jazzy. The simulation package in the parameters doc targets Humble and Gazebo Classic, so it does not match.

Not in the repo: the simulation package `curiosity_description`, any launch file, any URDF, any parameter YAML, the YOLO model weights, the drive, sonar, IMU, GPS or arm code, and a README.

## What the `detector` node does

1. Opens a camera stream (`tcp://127.0.0.1:8888` by default) with OpenCV.
2. Runs a YOLO model (an NCNN export loaded from a hard-coded path on one person's home directory) on every frame at 640 px, confidence 0.40.
3. Takes the highest-confidence box and publishes JSON text on `/vision/target` and `LEFT`, `RIGHT`, `CENTER` or `NO_TARGET` on `/vision/direction`, both as `std_msgs/String`. "CENTER" means the box center is within 0.12 of the image center.
4. Also publishes the bottom-center of the box as a normalized "floor point", which is the right feature for ground-plane projection.
5. Every 2 seconds, uploads a frame over HTTP to a server on the local network (`192.168.0.102:5000`, called "LingBot" in the code). On by default.

Nothing in the repo subscribes to these topics.

## Findings

### 1. The camera and the arm are not aligned

This follows from the CAD pose of the camera (x 0.248, y -0.1575, z 0.582 m from base_link, pitched 35 degrees down, yaw 0) and the arm base on the centerline (y = 0). It applies only if the real camera is mounted as in the CAD, so check that.

The image center is the camera's own axis, which is 15.75 cm to the right of the arm's line. With the node's 0.12 deadband (a half-angle of 8.9 degrees at a 66 degree horizontal field of view):

| Item on the arm's centerline at | Where it appears in the image (0 = left, 1 = right) | Node says |
|---|---|---|
| 0.69 m ahead | 0.33 | LEFT |
| 1.0 m ahead | 0.38 | LEFT (just) |
| 1.5 m ahead | 0.41 | CENTER |

At 0.69 m, "CENTER" covers items from 4 cm to 27 cm to the right of the arm's line. The arm's collision-free pick study used a pan range of only plus or minus 9 degrees. Pan swings about the arm's own base axis (x = 0.28 m in base_link), not about base_link, so across the pick zone that is only about plus or minus 1 to 4 cm sideways (2.7 cm at the 0.448 m tip pose). So steering until the node says "CENTER" would stop the robot with the item well outside the arm's studied reach. (An earlier version of this file said plus or minus 7 cm; that measured from the wrong point.)

Fix: do not steer on pixel direction. Project the floor point to base_link coordinates (below), then drive until the item is at y near 0 and x between 0.35 and 0.56 m.

### 2. Blind zone before the pick zone

The mast camera sees the ground from 0.69 m ahead of base_link. The pick zone is 0.35 to 0.56 m ahead. The last 0.13 to 0.34 m of approach is out of view by geometry, and the study (FABLE-STUDY-REPORT.md) argues 0.20 to 0.45 m in practice, because a box touching the bottom edge gives a bad floor point. Options: drive that distance by wheel odometry after the last detection (simple, a few centimeters of error), or add the wrist camera the SO-101 camera URDF describes.

### 3. Detections are text with no timestamp or frame

`std_msgs/String` with JSON carries no header, so nothing can be synchronized with the TF tree, odometry or the arm, and Nav2 cannot use it. Recommended: `vision_msgs/Detection2DArray` with a header stamped when the frame was captured, `frame_id` set to the camera frame, plus `sensor_msgs/CameraInfo`. Also publish the class, not just the box.

### 4. Latency risk

The 1 ms timer calls a blocking camera read and a model prediction in one callback. If inference is slower than the stream, a TCP/OpenCV capture serves old frames and the robot reacts to the past. Read in a separate thread and keep only the newest frame, or lower the stream frame rate. The node logs `processing_rate`: ask the partner for the real number on the Pi.

### 5. Hygiene

- Weights path, camera source and upload URL are hard-coded defaults; move them to a parameter YAML and a launch file.
- `mapping_enabled` is on by default and posts to a LAN IP; default it to off.
- `package.xml` declares only `rclpy` and `std_msgs`; OpenCV, `requests` and Ultralytics are used but not declared. Maintainer, license and description are still TODO.
- Model weights are not versioned. Use a release asset or Git LFS, with a note of the training data and metrics.

## Comparison with the simulation results

| Simulation result (parameters doc) | What the repo does | Gap |
|---|---|---|
| Plan for 0.40 m/s; camera sees 0.69 to 2.73 m ahead | Detector reports pixels only | Convert to meters (below) so speed and stopping distance can be planned |
| Sonar is a close-range bumper with blind sectors | No sonar code | Implement Pico sonar publisher (`sensor_msgs/Range`, 4 firing slots) |
| `diff_drive_controller`, `/cmd_vel`, `/odom` | No drive code | Needs the wheel-loop decision (DECISIONS.md) |
| Pick zone x 0.35 to 0.56 m, drop pose over the bag | Arm lives in arm-sim, outside ROS | Decide how the arm joins ROS |
| IMU and GPS in the sim | Not in code | Hardware owned; add to TF and wiring |
| Nav2 with range layer, footprint 0.72 x 0.52 m | Nothing | Later; start with local behaviors |

The simulation covers drive, sensors and navigation. The repo covers only perception. The two meet at the detector's output, so that interface is the one to design first.

## Ground-plane projection

Use the camera pose from the CAD (h is the camera height above ground, 0.582 + 0.060 = 0.642 m). Normalized pixel (u, v) runs 0 to 1 from the top-left; HFOV 66 degrees and VFOV 41 degrees for the standard Camera Module 3 lens (check against the stream's actual crop):

```python
from math import tan, cos, sin, radians

H, CAM_X, CAM_Y = 0.642, 0.248, -0.1575   # meters, base_link frame, y is left
PITCH, HFOV, VFOV = radians(35), radians(66), radians(41)

def floor_point_to_base(u: float, v: float):
    """Normalized image point on the ground -> (x, y) in base_link, or None if above the horizon."""
    dx = (u - 0.5) * 2 * tan(HFOV / 2)       # right of the optical axis
    dy = (v - 0.5) * 2 * tan(VFOV / 2)       # below the optical axis
    forward = cos(PITCH) - dy * sin(PITCH)
    down = sin(PITCH) + dy * cos(PITCH)
    if down <= 0:
        return None
    s = H / down
    return CAM_X + s * forward, CAM_Y - s * dx
```

Checked by hand: the bottom edge of the image (v = 1) maps to x = 0.689 m and the top edge (v = 0) to x = 2.730 m, matching the parameters doc. Verify against the real camera by placing a marker at a measured spot.

## Proposed cleanup (separate commit, review before merging)

Untrack `build/`, `install/`, `log/`, `core.158` and `__pycache__`, and add a `.gitignore`. Warning: when someone pulls that commit, git deletes those tracked files from their working copy. On the Pi that removes the built workspace, so rebuild afterwards with `colcon build`. The files stay in git history.

## Proposed layout, since the repo is still small

```
src/
  curiosity_description/   URDF/xacro, meshes, Gazebo Harmonic plugins (port of the existing package)
  curiosity_bringup/       launch files, parameter YAML, one launch per mode (sim, robot)
  curiosity_perception/    the detector, renamed and cleaned up (trash_vision can stay as the name)
  curiosity_drive/         Pico bridge: encoders, sonar, wheel commands
  curiosity_arm/           ROS interface to the SO-101 (see FABLE-STUDY-BRIEF.md)
docs/
```
