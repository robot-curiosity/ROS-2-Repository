# Risks

Ranked by how much each could hurt the January 20, 2027 submission. "Unverified" means it comes from reasoning or a datasheet recollection, not a measurement.

| # | Risk | Why it matters | Mitigation | Status |
|---|---|---|---|---|
| 1 | Schedule: about 10 weeks to submission, and no field-trial data yet | The research question needs a measured pickup rate | Get a drive-and-pick loop working end to end first, then run trials. Cut scope before cutting trials | Open |
| 2 | ROS code exists only on the Pi and in one commit | One SD card failure or a partner's laptop loss loses the work | Push the Pi workspace to git regularly. See REPO-REVIEW.md | Open |
| 3 | Camera and arm are not aligned (found in the repo review) | If the camera is mounted as in the CAD, the detector's "CENTER" puts the item 4 to 27 cm right of the arm's line at under 1 m, outside the arm's measured pan range | Convert detections to base-frame coordinates by ground-plane projection and servo on those. Formula in REPO-REVIEW.md | Open |
| 4 | Shared 15 A fuse may be too small | Four motors stalling can pull more than 15 A before adding the arm. Motor stall current is unverified | Check the datasheet; fuse the drive and arm branches separately | Fuse not confirmed bought |
| 5 | Pi 5 power from a commercial power bank | If the bank only offers 5 V / 3 A, the Pi limits USB current and can drop the Pico or the camera | Use a bank that supplies 5 V / 5 A over USB-PD, or a 12 V to 5 V converter sized for the Pi 5 | Check the bank |
| 6 | No localization beyond wheel odometry | Skid-steer odometry drifts. GPS and IMU are owned but not wired or modeled | Add them to the TF tree and wiring; fuse with `robot_localization`. Keep early runs short and local | Open |
| 7 | Picking flat or small litter with a parallel gripper | The pick study sampled only a narrow pan range, and flat items are hard to grip | Pick the first target litter types (cans, bottles) and measure pickup rate per type | Open |
| 8 | Blind zone in front of the robot | The mast camera sees ground from 0.69 m ahead; the pick zone is 0.35 to 0.56 m | Drive the last 0.13 to 0.34 m by odometry, or add a wrist camera | Open |
| 9 | Obstacle detection is thin | Sonar leaves large blind sectors and works poorly on grass; a single camera has no depth | Stay on pavement, treat sonar as a bumper, and test the camera for obstacles explicitly | Open, scoped by the off-grass decision |
| 10 | Motor PWM on the Pi 5 | The older GPIO libraries do not work on the Pi 5, and four independent PWM channels on the chosen pins are unverified | Move the wheel loop to the Pico, or verify with `pinctrl` first | Open |
| 11 | Detector latency | The node reads a camera stream and runs the model in one loop, so stale frames pile up if inference is slower than the stream | Read the latest frame only and report the real FPS. See REPO-REVIEW.md | Open |
| 12 | Cloud vision depends on school Wi-Fi | Not needed until GPT-4o is revived | Keep it pinned. Build the offline path first | Deferred |

## Closed

| Risk | Closed because |
|---|---|
| Camera ribbon cable for the Pi 5's smaller connector | The partner already has the camera working |
| Arm servo voltage | Confirmed 12 V |
| Gripper torque cap | Accepted at 50% for now |
