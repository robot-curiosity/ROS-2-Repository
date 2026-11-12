# Current state (2026-11-11)

Sources: Bryan's statements on 2026-11-11; the Onshape document; the ROS 2 robot parameters doc (dated 2026-09-20); the code in this repo; the LACSEF website. Anything not confirmed by one of these says so.

## Deadlines

| Date | What | Source |
|---|---|---|
| 2027-01-20 | LACSEF final project submission | https://lacsef.github.io/ |
| 2027-03-08 | LACSEF fair day, Shrine Expo Hall | https://lacsef.github.io/ |
| 2027-05-08 to 05-14 | Regeneron ISEF 2027, Los Angeles | a regional fair's page; confirm on the official ISEF site |

LACSEF revised its competition categories for 2027. Confirm the project is still entered in the right one. Safety and pre-approval form dates are in the official 2026-27 dates and deadlines document and have not been checked yet.

Earlier plans targeted LACSEF in March 2026 with a January 2026 demo. That timeline is obsolete.

## Project goal

Research question: can a low-cost autonomous robot reliably detect and retrieve litter in outdoor school environments, and which design parameters most affect pickup accuracy and efficiency?

Targets from the original plan: at least 80% pickup rate in field trials, at least 99% uptime in a known environment, BOM under $1,000, battery life of 30+ minutes. Nothing has been measured against these yet.

Scope for now: flat, hard surfaces (pavement, sidewalks). Grass is out of scope until the base is validated. See DECISIONS.md.

## Hardware

| Item | Status | Notes |
|---|---|---|
| SO-101 arm, 12 V Feetech STS3215 servos | Built, partly tested | Runs from the 12 V rail. Poses and macros live in the arm-sim repo |
| Raspberry Pi 5 with a commercial power bank | In use | Check the bank supplies 5 V / 5 A over USB-PD (see RISKS.md) |
| Camera | Working (partner) | CAD assumes Pi Camera Module 3. Confirm the actual module and mounting |
| Raspberry Pi Pico | Available | Planned for wheel encoders and sonar |
| GPS module | Owned | Not yet in the CAD, TF tree or wiring |
| IMU module | Owned | Not yet in the CAD, TF tree or wiring |
| Fuse (15 A planned) | Not confirmed purchased | See RISKS.md on sizing |
| 4x JGB37-520 12 V 110 rpm motors, 2x Cytron MDD10A, E-stop, 12 V battery | Status not yet confirmed | Parts photo to follow |
| Printed frame parts, wheels, pods | Status not yet confirmed | Parts photo to follow |

## CAD

Onshape document "Curiosity" is at version V21 (2026-09-27), the latest. Nothing has changed since. The parameters doc was written from V16 to V18, so check it against V21.

| Version | Change |
|---|---|
| V17 | Tire retaining beads |
| V18 | Onshape interference check: zero interferences involving project parts |
| V19 | Keyhole mounts for one-person assembly |
| V20 | Exploded View assembly tab |
| V21 | Corner-joint legs thickened from 5 mm to 7 mm after a hand structural check |

Key dimensions (from the parameters doc, V16 to V18):

| Quantity | Value |
|---|---|
| Frame | Box of 420 mm REV 1515 extrusions that doubles as the trash bin |
| Footprint, arm stowed | 0.72 m x 0.52 m |
| Wheel radius, track, wheelbase | 0.060 m, 0.481 m, 0.335 m |
| Ground clearance | 40 mm at the motor bodies |
| Arm shelf top above ground | 215.5 mm |
| Camera | x 0.248, y -0.1575, z 0.582 m from base_link, pitched 35 degrees down |
| Mass | About 6.1 to 6.4 kg empty (1.57 kg printed, the rest estimated); weigh the finished robot |

The tire diameter (120 mm) and the so101/base_link position are taken from the CAD, not measured.

## Software

| Piece | State |
|---|---|
| ROS 2 on the robot | Jazzy on Ubuntu 24.04 (Python 3.12, `/opt/ros/jazzy` visible in the committed files) |
| `trash_vision` package (this repo) | One node: YOLO detector on a camera stream, publishes target as JSON text. Nothing consumes it yet |
| Simulation package `curiosity_description` | Generated for Humble and Gazebo Classic 11. Not in this repo. Needs porting to Jazzy and Gazebo Harmonic |
| Arm control | Separate arm-sim repo (LeRobot, not ROS). Not connected to ROS yet |
| Drive, sonar, encoders, IMU, GPS, Nav2 | Parameters exist in the doc. No code in this repo |
| Cloud vision (GPT-4o) | Pinned for later. See DECISIONS.md |

The ROS code currently exists only on the Pi and in this repo's single commit. See REPO-REVIEW.md.

## Simulation and analysis results

From the parameters doc. All are simulation or hand estimates, not measurements.

| Question | Result |
|---|---|
| Top speed | 0.69 m/s no-load; plan for 0.40 m/s |
| Slope | Tips at 31 degrees or more; 15 degrees with a full bin is the planning limit for motor torque |
| Runtime | About 29 W driving, 53 W picking; about 45 min on a 12 V 3 Ah pack |
| Sonar coverage | Only 24 to 42% of directions seen at 1 m, with blind sectors up to 62 degrees. Treat sonar as a close-range bumper |
| Camera view | Sees ground from 0.69 m to 2.73 m ahead of base_link |
| Arm reach | Pick zone x 0.35 to 0.56 m ahead; 603 collision-free pick poses at the current shelf height |

## Open items

- Measure counts per wheel revolution (the ~3,960 figure is an unverified estimate).
- Add the IMU and GPS to the CAD, TF tree and wiring.
- Confirm the fuse is bought and sized (RISKS.md).
- Update the parameters doc to CAD V21.
- Port the simulation to Jazzy and Gazebo Harmonic.
- Get the ROS code from the Pi into git.
