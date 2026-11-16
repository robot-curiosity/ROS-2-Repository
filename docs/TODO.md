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

## Sensor layout (decided by Bryan, Nov 15)

The CAD (Onshape "Curiosity") has 8 printed sonar holders: 4 "forward/back
facing" and 4 "side facing". Bryan chose the CAD layout over 4 front + 4 rear
(a skid-steer robot turning in place swings its corners sideways, so sides
matter more than a second rear pair). Working assumption until Bryan sends
exact positions, label it as such in code comments:

| Name | Facing | Heading from forward (CCW +) |
|---|---|---|
| front_1, front_2 | forward, left to right | +0.20 rad, -0.20 rad |
| rear_1, rear_2 | backward, left to right when facing the rear | pi-0.20, pi+0.20 |
| left_1, left_2 | left side, front to back | +pi/2 (staggered fore/aft) |
| right_1, right_2 | right side, front to back | -pi/2 (staggered fore/aft) |

Sonars must be fired ONE AT A TIME (never together; HC-SR04 modules hear each
other's pings). Echo pins are 5 V on the sensor, so the Pi pins need a divider
or level shifter. That is wiring and is Bryan's job, not the agent's.

## Open items (do the first unchecked one)

Order matters; later items assume earlier ones. One item per run, one PR each.

The end goal Bryan set: as each part of the robot is assembled he only puts values into `config/robot.yaml`, runs one command and tests it. Items 10-14 build that path first; everything after improves behavior and software quality.

- [ ] 10. Sonar layout migration. Change the ultrasonic names, config and logic from `front_1..4`/`rear_1..4` to the layout above (`front_1..2`, `rear_1..2`, `left_1..2`, `right_1..2`). Update `ultrasonic_logic.py`, `ultrasonic_node.py`, `config/robot.yaml` (8 trig and 8 echo pins in that order, still unset), `sim_logic.py` sensor angles and positions, `docs/BOM.md` note, and all tests. In `behavior_logic.py`, replace the front/rear split: obstacle slowdown and avoidance use front_*, recovery backing uses rear_*, and pure rotation in place is limited or refused when the side sensor on the side being swung into reads closer than a `side_clearance` parameter. Tests for each of these. Also update the affected rows in `docs/DECISIONS.md` (mark the old front/rear row superseded) and `docs/HARDWARE_TESTS.md` (ultrasonic and behavior rows mention front_1..4/rear_1..4).
- [ ] 11. Single bring-up config. Goal: when a part is assembled, Bryan fills in numbers in ONE place and tests it. Make `config/robot.yaml` the only file with hardware values, grouped by subsystem (drive, sonar, arm, camera, imu_gps, behavior calibration), each value with a one-line comment saying what it is, its unit, and how to measure it. Add per-subsystem launch files (`bringup_drive.launch.py`, `bringup_sonar.launch.py`, `bringup_arm.launch.py`, `bringup_camera.launch.py`, `bringup_imu_gps.launch.py`) that load that file and start only that subsystem, plus `bringup.launch.py` for everything. A node with unset values idles with one clear warning naming the missing parameter. Tests that every parameter a node declares appears in the yaml.
- [ ] 12. Bring-up check script. A `hardware_check` console script: `ros2 run trash_vision hardware_check [subsystem]` reads `robot.yaml`, prints each unset value, checks pin conflicts (same pin used twice), checks the Pi 5 pin numbers are valid, and prints PASS/FAIL/SKIP per subsystem. Add one row per subsystem to `docs/HARDWARE_TESTS.md` with the exact command and expected output, Result column empty. Pure-Python checks with tests.
- [ ] 13. Calibration helpers (small CLI tools, pure logic plus thin ROS wrappers) so Bryan gets each number by doing one physical action: (a) `calibrate_encoders`: spin each wheel exactly 10 turns by hand, print `ticks_per_rev`; (b) `calibrate_drive`: command a known speed for 3 s, ask for measured distance, print wheel radius and track-width corrections; (c) `calibrate_pick_zone`: place a can at the pick position, print `pick_floor_y` and tolerance from the detector; (d) `calibrate_camera`: place a marker at a known distance, print the pitch error; (e) `sonar_test`: print live distances per named sensor and flag a sensor that is swapped, stuck or never reports. Each prints the line to paste into `robot.yaml`.
- [ ] 14. Bring-up guide. `docs/BRINGUP.md`: the order to bring the robot up (power and fuse check, sonars, motors with wheels off the ground, encoders, drive calibration, camera and detector, arm, IMU/GPS, behavior with `enabled:=false` then true), and for each step: what to wire (refer to `PIN_BUDGET.md` once it exists), what to fill in, the command to run, what PASS looks like, and what the common failure is. Keep it short and exact, no theory. Safety first: wheels off ground, fuse in, arm clear, a hand on the power.
- [ ] 15. Sonar scheduling and validity. Fire the sonars round-robin one at a time with a settle gap parameter (default 60 ms between pings), a per-sensor timeout, and a validity rule (no echo or out of range is reported as "no obstacle" at max range, never as 0). Pure-Python scheduler with tests, then use it in the node. Add a `sonar_age` staleness check so the behavior treats a sensor not updated recently as blocked.
- [ ] 16. Sim pickup-rate test. Extend the pure-Python sim (`sim_logic.py`) and add a CI test that runs the behavior logic against N scripted litter items (including some near walls so the side sensors matter) and asserts a minimum collected fraction. Print the rate. This gives a repeatable software number toward the 80% target and a regression guard.
- [ ] 17. Trial logging and summary. Log every pickup attempt (time, detected, approached, picked, success, litter type, failure reason) to JSON Lines from the behavior node, and add a script that prints pickup rate by litter type and per run. Needed for the field trials and the research question.
- [ ] 18. Search pattern. Replace any random wander/spin in the SEARCH state with a coverage pattern driven by odometry (for example expanding lawnmower rows with a parameterized row spacing from the camera field of view), with obstacle and boundary handling. Pure logic, tests, and show the coverage fraction in the sim test.
- [ ] 19. Detector latency. Read camera frames in a separate thread and keep only the newest frame, make the processing loop rate a parameter instead of a 1 ms timer, and publish the measured processing FPS (RISKS.md #11).
- [ ] 20. Detection filtering. Pure-Python module: require a target to be seen in N of the last M frames before reporting it, ignore a configurable image region where the robot's own bag hoop or arm can appear, pick the best target by distance and confidence, with a minimum-confidence parameter. Tests with scripted frame sequences.
- [ ] 21. Arm kinematics. Port the pure-Python SO-101 forward/inverse kinematics from `drivebase_behaviour` on `experimental/behaviour-coordinator`, with its tests, into `trash_vision` as `arm_kinematics.py`. Use it to compute pick waypoints in `arm_logic.py` instead of fixed placeholder poses. Arm mount height is a parameter (the arm shelf top is 215.5 mm above ground per STATE.md; the branch assumed 224 mm). Reject targets outside the pick zone.
- [ ] 22. Safety output stage. A pure-Python last stage in the behavior node: clamp speed and acceleration (parameters), force zero velocity if sonars are stale, if the front minimum is below a hard stop distance, or if no behavior update has arrived within a timeout. Tests for every case. Do not change the drive node.
- [ ] 23. Pin budget document. Write `docs/PIN_BUDGET.md`: list every signal the Pi needs (8 sonar trig, 8 echo, 2 PWM, 2 dir, 4 encoder channels per the config, plus I2C/UART for IMU, GPS and the arm), count the GPIO available on a Pi 5 header, and show the shortfall (RISKS.md says about 32 needed vs 26). List the options (shared trigger line, I2C GPIO expander, moving wheel control to a Pico) with pin counts for each. Present options only. The choice is Bryan's. Do not edit `robot.yaml` pins.
- [ ] 25. Style cleanup (one or two modules per run). The ament flake8 and pep257 tests are skipped in CI because of 247 legacy complaints. Fix them module by module and, once a module is clean, say so in the PR. When every module is clean, remove the skip from `.github/workflows/ci.yml`.
- [ ] 26. Package hygiene. Declare the missing dependencies in `package.xml` (OpenCV, requests, ultralytics where a rosdep key exists, otherwise note it), set a real package description, and leave the license as "TODO: Bryan to choose".

If every item is done or blocked, stop and say so in the run summary. Do not invent new work.

## Blocked on Bryan's decisions (do NOT work on these)

- Wheel control loop on the Pi or on a Pico (the existing drive node stays as it is; do not extend it. The ultrasonic node is changed only as item 10 and 11 say).
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
