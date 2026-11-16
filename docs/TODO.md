# Curiosity Robot TODO (prioritized)

Software-only work the agent can do and CI can test. Hardware work, ordering and
the open design decisions below are Bryan's, not the agent's.

## Where the other work lives (read before starting an item)

Bryan's real project history is on other branches of this repo. They are NOT
fetched by default. Fetch one (shallow) with an explicit refspec, then read it
with `git show`:

    git fetch --depth=1 origin "+refs/heads/docs/consolidated-state:refs/remotes/origin/docs/consolidated-state"
    git show origin/docs/consolidated-state:docs/STATE.md

| Branch | What it has |
|---|---|
| `docs/consolidated-state` | `STATE.md` (deadlines, hardware status, CAD dimensions), `DECISIONS.md`, `RISKS.md`, `REPO-REVIEW.md` (review of the detector, with the ground-plane projection formula), `FABLE-STUDY-REPORT.md` |
| `experimental/behaviour-coordinator` | Gazebo Harmonic sim, `drivebase_behaviour` (pure-Python mission logic, kinematics, closed-form IK, 67 tests), `STATUS.md`, QA review `docs/qa/2026-08-01-behaviour-coordinator-review.md`, measured grasp findings |
| `experimental/so101-ros2-control` | Same, plus `ros2_control` for the SO-101 |
| `URDF-arm` | SO-101 URDF and Gazebo/ros2_control integration |

Rules for using them:
- Port or adapt what is there instead of rewriting it. Say in the PR which file you ported from.
- Never copy `build/`, `install/`, `log/`, `core.*`, `__pycache__/` or mesh files.
- The experimental branches were written for a different power and motor design (24 V motors, power-tool battery pack). Ignore their parts-ordering and power items. The real robot is 12 V (JGB37-520 motors, 12 V pack).
- Dimensions documented in `STATE.md` (wheel radius 0.060 m, track 0.481 m, wheelbase 0.335 m, camera at x 0.248, y -0.1575, z 0.582 m from base_link pitched 35 degrees down, about 66 degree horizontal field of view, pick zone 0.35 to 0.56 m ahead) may be used as parameter defaults. Label them "from CAD V16-18 per STATE.md, not measured".
- Target: a drive-and-pick loop working end to end, then field trials. Submission is Jan 19-20, 2027; first milestone is one can bagged 3 of 10 by Dec 1.

## Open items (do the first unchecked one)

- [ ] 3. Sim pickup-rate test. Extend the pure-Python sim (`sim_logic.py`) and add a CI test that runs the behavior logic against N scripted litter items and asserts a minimum collected fraction. Print the rate. This gives a repeatable software number toward the 80% target and a regression guard.
- [ ] 4. Trial logging and summary. Log every pickup attempt (time, detected, approached, picked, success, litter type, failure reason) to JSON Lines from the behavior node, and add a script that prints pickup rate by litter type and per run. Needed for the field trials and the research question.
- [ ] 5. Detector latency. Read camera frames in a separate thread and keep only the newest frame, make the processing loop rate a parameter instead of a 1 ms timer, and publish the measured processing FPS (RISKS.md #11).
- [ ] 6. Arm kinematics. Port the pure-Python SO-101 forward/inverse kinematics from `drivebase_behaviour` on `experimental/behaviour-coordinator`, with its tests, into `trash_vision` as `arm_kinematics.py`. Use it to compute pick waypoints in `arm_logic.py` instead of fixed placeholder poses. Arm mount height is a parameter (the arm shelf top is 215.5 mm above ground per STATE.md; the branch assumed 224 mm).
- [ ] 7. Package hygiene. Declare the missing dependencies in `package.xml` (OpenCV, requests, ultralytics where a rosdep key exists, otherwise note it), set a real package description, and leave the license as "TODO: Bryan to choose".

## Blocked on Bryan's decisions (do NOT work on these)

- Wheel control loop on the Pi or on a Pico (the existing drive/ultrasonic nodes stay as they are; do not extend them).
- Arm interface: LeRobot wrapper or `ros2_control`.
- Detector message type (stay with JSON in `std_msgs/String` for now).
- Cloud vision (GPT-4o) stays off and is not to be extended until on-device pickup works (see `DECISIONS.md` on `docs/consolidated-state`).
- Porting the Gazebo sim to this package.
- Anything that needs the real robot or purchasing: counts per revolution, fuse sizing, IMU/GPS wiring, field trials.

## Done

- [x] 1. Detection node (`trash_vision`): YOLOv8 + GPT-4o Vision fallback; publish detections topic (message type TODO: Bryan to decide)
- [x] 2. Ultrasonic node: 4 front + 4 rear HC-SR04, publish ranges (GPIO pins TODO: Bryan to fill in)
- [x] 3. Drive/encoder node: 2x Cytron MDD10A + JGB37-520 encoders, cmd_vel -> motors, odometry
- [x] 4. Arm control node: SO101 via LeRobot (Feetech STS3215), pick service/action
- [x] 5. GPS/IMU node: publish NavSatFix and Imu (device model TODO: Bryan to fill in)
- [x] 6. Behavior/state machine: search -> approach -> pick -> deposit, with obstacle avoidance
- [x] 7. Simulation launch (Gazebo) and bringup launch file for the real robot
- [x] 8. Ground-plane projection. Convert the detector's normalized floor point to base_link x, y metres using the camera pose and field of view above (see `REPO-REVIEW.md` on `docs/consolidated-state`; also look at the pixel-to-point code in `drivebase_behaviour` on `experimental/behaviour-coordinator`). Pure-Python module with tests. Add `base_x`, `base_y` to the `/vision/target` JSON without removing existing fields. Camera pose and field of view are parameters in `config/robot.yaml`. This fixes the camera/arm misalignment risk (RISKS.md #3). (Ported from the formula in REPO-REVIEW.md; `ground_projection.py`.)
- [x] 9. Behavior robustness. Apply the review in `docs/qa/2026-08-01-behaviour-coordinator-review.md` to our `behavior_logic.py`: every state has a timeout that returns to SEARCH, nothing busy-loops or blocks in a timer callback, no unused outputs. Add a test per state proving it cannot wedge the mission. (Added approach timeout + cooldown, stale-detection and stale-arm-result guards; per-state tests in `test_behavior_logic.py`. Ported intent from the QA review F1/F4/F5: blocking waits/unused outputs were not present in our node.)
