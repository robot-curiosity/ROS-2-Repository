# Study brief: software and algorithms for Curiosity

Paste this whole file to the model doing the study. It is written to stand alone.

## Your task

Study the software and algorithm problems below for a student-built litter-collection robot and tell us what to build, in what order, and what to cut. We have about 10 weeks until a competition submission (2027-01-20), so a design that works by then beats an optimal one. Be concrete: name algorithms, libraries, parameters, expected failure modes, and the cheapest experiment that would settle each doubt. Say plainly when you are unsure.

## The robot

- Chassis: 4-wheel skid-steer, 0.72 x 0.52 m, about 6.4 kg empty. Wheel radius 0.060 m. Planned max speed 0.40 m/s. Surfaces: flat pavement only for now (grass is out of scope).
- Arm: SO-101, 5 degrees of freedom plus a parallel gripper, 12 V Feetech STS3215 servos, mounted upright at the front on the centerline. The gripper is limited to 50% torque by the LeRobot driver. The arm is controlled today through LeRobot in a separate repo (a pose and macro studio with collision checking and input-shaped motion).
- Compute: Raspberry Pi 5, ROS 2 Jazzy on Ubuntu 24.04. A Raspberry Pi Pico is available for real-time I/O. No GPU. No lidar. No depth camera.
- Sensors: one forward RGB camera (Pi Camera Module 3 class; 66 x 41 degree field of view) at x 0.248, y -0.1575 (right of the arm line), z 0.582 m, pitched 35 degrees down; eight HC-SR04 ultrasonic sensors (4 front, 4 rear, 15 degree beams, about 4 Hz); wheel encoders (JGB37-520 Hall, gear ratio unverified); an IMU and a GPS module that are owned but not yet integrated.
- Detection today: a YOLO model (NCNN export on the Pi) on camera frames, publishing the box as JSON text with a normalized "floor point". It has no distance, class or timestamp. Nothing consumes it yet.
- Optional later: a cloud vision model (GPT-4o) as a second opinion. Deliberately deferred. Design the on-device pipeline first, and say how a cloud step would slot in without being required.

## Facts from our own analysis (treat as estimates, not measurements)

- The camera sees ground from 0.69 m to 2.73 m ahead of base_link. The arm's pick zone is 0.35 to 0.56 m ahead, so the last 0.13 to 0.34 m of an approach is out of view.
- The camera is 15.75 cm right of the arm line. Steering until the detection is at the image center leaves the item about 16 cm right of where the arm can reach. The detector's current "CENTER" band excludes the arm line for items closer than about 1 m. A ground-plane projection to base_link fixes this and is available (see REPO-REVIEW.md).
- The arm's collision-free pick study sampled a pan range of only +/- 9 degrees (about 7 cm sideways at the 0.45 m reach). 603 collision-free pick poses exist at the current shelf height. The wider pan range was not studied.
- Sonar covers 24 to 42% of directions at 1 m, with blind sectors up to 62 degrees. We treat it as a bumper.
- Simulation: 15 degree slope with a full bin is within motor torque; runtime about 45 minutes on a 12 V 3 Ah pack; the bin holds a trash bag.

## Constraints

- Free or cheap tools only. Python and C++ both fine.
- Everything must run on the Pi 5 within its power budget, or be explicitly off-board and optional.
- Student team of two, with school and exams: favor simple, debuggable, testable pieces.
- The project is judged on a research question: how reliably can a low-cost robot find and retrieve litter outdoors, and which design parameters matter most. We need experiments we can run and report, not only a working robot.

## Questions

Answer each with a recommendation, the main alternative, the reason, the failure modes to expect, and the cheapest test.

1. **Pick-up.** How should the robot get the gripper onto an item from a ground-plane position estimate with no depth camera? Cover: approach strategy and final alignment (visual servoing versus look-then-move with odometry), the camera-versus-arm offset, handling the blind zone, grasp pose choice (top-down versus angled), what to do about flat or small items, detecting a failed grasp using servo load and gripper angle (our arm studio records both), retry policy, and the drop motion into the bag. Estimate achievable pickup rate per litter type.
2. **Obstacle detection.** Using only sonar, a monocular camera, the IMU and wheel odometry, what is the minimum reliable obstacle strategy for pavement? Cover: sonar filtering and firing order, camera-based free-space or obstacle detection that runs on a Pi 5, curbs and drop-offs, and what safety stop behavior is enough for a demonstration.
3. **Object recognition.** How should we train and evaluate the detector? Cover: model size and export for the Pi 5 (frames per second to expect), datasets (public litter datasets versus our own), synthetic cut-and-paste data versus real versus mixed (we want this as an experiment), false-positive handling on pavement (leaves, shadows, painted lines), confidence and temporal filtering across frames, and which classes to include.
4. **Object selection.** When several items are visible, which one next? Cover: scoring (distance, reachability, confidence, class, cluster), commitment to a target to avoid dithering, handling a target that disappears into the blind zone, and when to give up on an item.
5. **Navigation.** Which navigation stack is appropriate for a first version? Cover: whether Nav2 is worth its complexity versus a simple behavior state machine (search, approach, pick, deposit), the search pattern for a known campus area, using GPS and IMU with `robot_localization` and how much to trust them, how to drive an accurate approach with skid-steer scrubbing, and how to return or stop when the bag is full.
6. **System design.** Message types and node boundaries, how the SO-101 joins ROS (a node wrapping LeRobot versus ros2_control), where the wheel control loop should run (Pi versus Pico), logging for experiments, and a state machine with explicit failure states.
7. **Schedule.** Given 10 weeks, produce a dated build order that has a working end-to-end loop first, field trials last, and a list of what to cut if we slip.
8. **Experiments.** Propose three to five experiments that would make a strong research contribution at a regional or international science fair, with variables, metrics, number of trials, and what would count as a surprising result.

## What to return

A short ranked plan first (what to do in the next two weeks), then a section per question, then the schedule and experiments. Keep each section short enough to read in a few minutes. Mark claims you are confident about versus guessing.

## Files to read first if you have repo access

`docs/REPO-REVIEW.md`, `docs/STATE.md`, `docs/RISKS.md`, `docs/DECISIONS.md`.
