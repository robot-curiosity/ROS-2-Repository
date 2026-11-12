# Decisions

Decided by Bryan on 2026-11-11 unless noted.

## Decided

| Decision | Detail |
|---|---|
| ROS 2 distribution | The Pi 5 runs ROS 2 Jazzy on Ubuntu 24.04. Simulate with Gazebo Harmonic, the pairing Jazzy supports. The older Humble and Gazebo Classic package must be ported |
| Arm servos | 12 V STS3215, powered from the 12 V rail through the E-stop's second contact |
| Terrain | Stay off grass for now. Pavement and flat hard surfaces only. Reason: sonar reads grass badly and a monocular camera gives no depth. Revisit once the base drives reliably |
| Gripper torque | LeRobot limits the gripper to 50% torque on every connect, and the arm-sim README says so. Bryan's judgment is that 50% is enough. Revisit if grips slip. The arm-sim "record trace" feature shows slips |
| Hardware values | For now, the tire diameter and the arm base position come from the CAD |
| Camera | Working on the partner's side |

## Deferred on purpose

| Item | Why it waits | Revisit when |
|---|---|---|
| GPT-4o (or any cloud model) as a second opinion on ambiguous detections | Needs Wi-Fi at school, adds latency, and the on-device pipeline has to work first. Pinned for later | Pick-up works reliably on-device. Design the offline fallback first |
| Grass | See above | Base validated on pavement |

## Still open

| Question | Options | Notes |
|---|---|---|
| Who runs the wheel control loop? | Pi 5 (hardware PWM, `gpiod` or `lgpio`) or the Pico (PWM, encoders and speed loop, talking to the Pi over USB) | The Pico already reads the encoders. Running the loop there avoids Linux timing jitter. Pi 5 PWM channel sharing on pins 12/13/18/19 is unverified |
| How do arm-sim and ROS connect? | A ROS node wrapping LeRobot, or ros2_control with a servo hardware interface | See FABLE-STUDY-BRIEF.md |
| Detector output format | Keep JSON strings or move to `vision_msgs` with headers | Recommended: `vision_msgs`. See REPO-REVIEW.md |
| Where IMU and GPS connect | Directly to the Pi or through the Pico | Depends on the wheel-loop decision |
| Fuse layout | One shared 15 A fuse or separate drive and arm branches | RISKS.md item 4 |
