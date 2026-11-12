> Model-written study (Fable 5.2), 2026-11-11, from the docs in this folder and sources it lists at the end. Recommendations are not decisions: see DECISIONS.md for what the team has actually decided. One correction applies: the pan tolerance is about 1 to 4 cm across the pick zone, measured from the arm base axis, not 7 cm (see REPO-REVIEW.md).

# Curiosity design study: software and algorithms

Written 2026-11-11 against repo commit `e0636f6`. Submission is 2027-01-20, which is 70 days (10 weeks) away.

Markers: **[C]** = confident (checked by calculation, by reading the code, or against a source opened today). **[G]** = guess or engineering judgment that the listed test should settle. Source numbers like [S3] refer to the Sources section at the end.

The core recommendation is: **no Nav2, no GPS fusion, no Gazebo port, no cloud model before submission.** Build a plain state machine that does stop-and-measure, rotate, straight blind drive, and a lookup-table pick, and spend the saved weeks on logged trials.

---

## 0. Ranked plan for the next two weeks (Nov 11 to Nov 24)

1. **Get the Pi workspace into git and confirm fair paperwork (day 1 to 2).** RISKS item 2 is still open. STATE.md says the LACSEF safety and pre-approval dates are unchecked. ISEF-affiliated fairs normally want the research plan approved before data collection starts [G], so check this before running anything you intend to report.
2. **Static pick test (the most informative experiment available today).** Robot parked, arm only. Place items by hand on a taped grid in the pick zone and run the pick macro 10 times per item type. This decides which litter classes are in scope and how much sideways error the arm tolerates. Everything else depends on these two answers. Before this, extend the collision-free pose study from pan +/- 9 degrees to +/- 25 degrees.
3. **Calibrate the camera to the ground with a taped marker grid** and replace the analytic projection with a fitted homography (section 1). 20 minutes of tape work removes five unmeasured parameters.
4. **Fix the detector node**: newest-frame-only capture thread, upload off by default, `vision_msgs` output with capture timestamp and class, reject boxes that touch the image edge, rectangular export. Record the real frames per second on the Pi.
5. **Pico firmware v0**: motor PWM, encoder counting, speed loop, 300 ms command watchdog, serial link to a `pico_bridge` node. Drive by gamepad. Measure counts per wheel revolution and straight-line odometry error over 5 m.
6. **Arm server v0**: one ROS action `PickAt(x, y, class)` and one `RunMacro(name)` wrapping the existing LeRobot code.
7. **Record dataset v0**: push the robot by hand over 5 pavement surfaces with the camera at its real pose, 300 frames, label, train. Keep one location out for testing.
8. **One-hour hardware test**: rotate the camera to portrait at 45 to 50 degrees pitch and check whether the pick zone becomes visible without the arm blocking it (section 1).

Milestone for Dec 1: one can, starting 1.5 m away, picked and bagged autonomously at least 3 times in 10.

---

## Numbers in the docs that I checked

| Claim in docs | Result | Status |
|---|---|---|
| Camera sees ground 0.69 to 2.73 m | Reproduced: 0.689 and 2.730 m | [C], but only if base_link is at axle height (camera 0.642 m above ground). If base_link is on the ground the span is 0.65 to 2.50 m. Measure the real lens height |
| Arm-line item appears at u = 0.33 / 0.38 / 0.41 at 0.69 / 1.0 / 1.5 m | Reproduced: 0.334 / 0.377 / 0.413 | [C] |
| "CENTER" at 0.69 m covers 4 to 27 cm right of the arm line | Reproduced: 4.4 to 27.1 cm | [C] |
| Deadband 0.12 is 8.9 degrees | Reproduced: 8.86 degrees | [C] |
| CENTER excludes the arm line under about 1 m | Reproduced: crossover near 1.03 m | [C] |
| Blind distance 0.13 to 0.34 m | Arithmetic is right, but **optimistic in practice**. A box that touches the bottom edge gives a wrong floor point, so the last usable measurement is at about 0.75 to 0.80 m. Plan for 0.20 to 0.45 m of blind travel | [C] on the reasoning, [G] on the exact figure |
| Pan +/- 9 degrees is about +/- 7 cm at 0.45 m | **Could not reconcile.** 0.45 m is measured from base_link, but pan sweeps about the arm's own base axis, which is at the front of the chassis. If the pan axis is 0.20 m ahead of base_link, the radius is 0.25 m and +/- 9 degrees is only +/- 4 cm. Need the so101 base x from the CAD | [C] that the figure depends on an unstated value |
| HFOV 66, VFOV 41 degrees | Matches the Raspberry Pi spec for the standard module [S5]. The committed test image is 1280x720 (16:9), consistent with the full-width modes. A 4:3 stream would crop to about 52 degrees. The two angles imply focal lengths of 986 and 963 px, a 2% mismatch, so they are rounded and should not be trusted for centimeter work | [C] |
| Top speed 0.69 m/s | 110 rpm x 0.060 m radius = 0.691 m/s no-load | [C] |
| About 3,960 counts per wheel rev | Equals 11 pulses x 4 edges x 90:1. Plausible for this motor, still measure | [G] |
| Runtime about 45 min on 12 V 3 Ah | 36 Wh gives 74 min at 29 W and 41 min at 53 W at full depth of discharge; 33 to 60 min at 80% usable. 45 min is plausible for a pick-heavy run, not conservative | [C] arithmetic |
| Sonar sees 24 to 42% of directions, blind sectors up to 62 degrees | 8 beams x 15 degrees = 33% of the circle, so the magnitude is consistent. Could not check the 62 degree figure without the mounting angles; with all four front sensors facing straight ahead, each side would be blind over more than 90 degrees | Not reconciled |
| Targets: 99% uptime, 80% pickup | "Uptime" is not measurable as written; use interventions per run. "80%" needs a denominator (per attempt, or per item present). See section 8 | [C] |

Two more things from the repo itself:

- `runs/detect/predict/camera-test.jpg` shows an out-of-focus frame with a single class `trash` at 0.52 on a blurred cable cut off by the left edge. That is one image, but it shows three issues at once: autofocus hunting, a one-class model, and an edge-truncated false positive above the 0.40 threshold. [C]
- At the current mount, a 6.6 cm can is about 44 px wide at 0.7 m, 23 px at 1.5 m and 14 px at 2.7 m in a 640 px wide image. A cigarette butt is 2 to 5 px everywhere. So useful detection range for cans is about 2 m, and butts and bottle caps are not detectable with this camera. [C] geometry, [G] on the 2 m cutoff

---

## 1. Pick-up

**Recommendation: look, stop, rotate, drive straight blind, pick from a lookup table.**

1. **Coarse approach.** Turn until the item's bearing in base_link is within 5 degrees, drive at 0.25 m/s until it is 0.85 to 0.95 m ahead.
2. **Stop and measure.** Stand still, take 5 to 10 frames, use the median projected (x, y). Store it in the odom frame.
3. **Rotate in place until y = 0.** Because the arm is on the centerline and the robot turns about its own center, a pure rotation by atan2(y, x) puts the item on the arm line. The camera offset stops mattering once you steer on base_link coordinates. Close the loop with the camera (the item is still in view at this range, at u about 0.36) until |y| is under 1.5 cm or 4 tries.
4. **Blind straight drive** of (x minus 0.45 m), at 0.10 to 0.15 m/s, holding heading with the gyro. Keep correcting from the camera until the box nears the bottom edge, then dead-reckon. Over 0.3 to 0.45 m, encoder distance error is about 1 cm and 1 degree of heading error is 0.5 to 0.8 cm sideways. [C] arithmetic, [G] that the wheels hold 1 degree
5. **Pick.** Transform the stored odom point to base_link and index a precomputed table of collision-free poses (from the 603 you have, extended to wider pan). No online inverse kinematics.

**Main alternative:** remove the blind zone in hardware. Three options, with my computed ground coverage for the same camera position:

| Mount | Ground seen | Half-width at 0.45 m / 1.5 m | Note |
|---|---|---|---|
| Now: landscape, 35 degrees | 0.69 to 2.73 m | 0.35 / 0.91 m | Blind zone |
| Portrait, 45 degrees | 0.38 to 3.27 m | 0.22 / 0.50 m | Covers most of the pick zone; narrow search swath; arm line is only 6 cm inside the edge, so yaw the camera about 8 degrees left |
| Portrait, 50 degrees | 0.33 to 2.35 m | 0.23 / 0.48 m | Covers the whole pick zone |
| Camera Module 3 Wide (102 x 67 [S5]), landscape, 50 degrees | 0.32 to 2.42 m | 0.77 / 1.60 m | Best coverage; items are about 40% fewer pixels across; needs lens undistortion |

These are [C] as geometry and [G] as a design, because the arm may block the view. A wrist camera is the other alternative; I would not add a second camera stream and a second servo loop in 10 weeks.

**Reason.** Projection error grows fast with range, so one early measurement is not good enough, but a close one is. With the current mount, 1 degree of pitch error moves the estimate 1.6 cm at 0.69 m, 3.3 cm at 1.16 m and 10.7 cm at 2.2 m [C]. The arm has about 10 cm of slack along x (pick zone 0.35 to 0.56 m) and only a few centimeters across y. Rotate-then-straight turns the hard lateral problem into a rotation the camera can close the loop on, and leaves the forgiving axis to odometry. It also avoids arcs, which is where skid-steer odometry is worst.

**Calibration: use a homography, not the formula.** The formula in REPO-REVIEW.md is correct [C], but it needs height, pitch, roll, yaw and two field-of-view angles, none measured. Tape 12 to 16 markers at measured base_link positions, click them in one image, and fit `cv2.findHomography`. It absorbs all of those at once. Re-check four markers at the start of each trial day.

**Floor point bias.** The box bottom is the item's near edge, not its center. For a can lying side-on the center is about 3 cm further; for a bottle lying end-on up to 10 cm. Add a per-class forward offset, fitted in experiment 1. [C] on direction, [G] on size

**Grasp pose.** Top-down, jaws across the item's short axis, wrist roll sets jaw yaw. Version 1: choose jaw yaw from the box aspect ratio; on failure, retry rotated 90 degrees. Version 2 if needed: threshold the crop at the stop pose and take the principal axis of the mask with OpenCV. Standing cans and bottles need no orientation. [G]

**Flat and small items.** Declare tiers and report them separately:

- Tier A (attempt, target class): empty cans, empty bottles up to 500 ml, cups, crumpled paper or bags.
- Tier B (attempt, expect worse): crushed cans, cartons, loosely crumpled chip bags.
- Tier C (detect and log, do not attempt): flat wrappers, sheets, lids, caps, butts.

Skipping Tier C is a result, not a failure. Rubber fingertip pads are the one cheap change worth testing for flat items. [G]

**Failed-grasp detection.** LeRobot's follower code sets the gripper to `Max_Torque_Limit` 500, `Protection_Current` 250 and `Overload_Torque` 25 on configure, which matches the 50% figure in DECISIONS.md [C, S9]. Use two checks:

- After closing: if the gripper reaches within about 3% of fully closed, nothing rigid is held. If it stalls short of closed with load, something is held. Check again at the top of the lift and before release; a change to fully closed means dropped in transit.
- Thin items close the jaws fully even when held, so also look: back up 0.35 m (or just look, with a portrait mount) and check whether the track at that odom position is gone.

**Retry policy.** Maximum 2 attempts per item. Attempt 2 backs up 0.35 m, re-measures, and uses the other jaw yaw. Then blacklist that spot (section 4).

**Drop motion.** One recorded macro: lift to a carry pose above the frame rim, pan over the bag, open, wait 0.5 s, one wrist wiggle for sticky items, confirm the gripper reads open, return to a stow pose that does not block the camera. The base does not move while the arm is out of stow.

**Expected pickup rate per attempt** (all [G]; experiment 2 replaces these):

| Item | First attempt | With one retry |
|---|---|---|
| Can, standing | 75 to 90% | 85 to 95% |
| Can, lying | 60 to 80% | 75 to 90% |
| Empty 500 ml bottle, lying | 50 to 70% | 65 to 80% |
| Cup or crumpled paper | 60 to 80% | 70 to 85% |
| Crumpled chip bag | 40 to 60% | 50 to 70% |
| Flat wrapper | 10 to 30% | 15 to 35% |
| Cap, butt | under 10% | not attempted |

For scale: a much better equipped system (UR5e arm, depth camera, tactile fingertips, Jetson) reported 80% on first attempts over 60 outdoor pickups, with glass worst at 0.60 [S10]; a 2026 park robot with RTK GPS reported 80% overall [S11]; the VERO quadruped collected 90% of the butts it identified [S12]. Hitting 80% across all litter with this hardware is unlikely [G]. Hitting 80% on Tier A with a retry is a fair target.

**Failure modes to expect:** camera knocked after calibration; wheel slip during the final rotation; stiction making corrections under 2 degrees impossible (let the arm pan absorb the rest); cans rolling when a jaw touches first; servo overload when pressing on the ground; autofocus shifting the image scale.

**Cheapest test:** the static pick grid in plan item 2, then 20 approach-only runs measuring where the item ends up relative to a tape cross under the gripper.

---

## 2. Obstacle detection

**Recommendation: sonar as a speed limiter and stop, enforced in the Pico; restrict where the robot runs; camera free-space is a stretch goal.**

**Sonar firing.** Fire only the sensors facing the direction of travel. That is 4 sensors in 2 slots (outer pair, then inner pair, so no two neighbors fire together), 60 ms per slot, giving about 8 Hz per sensor instead of 4. When stationary or picking, cycle all eight in four slots, pairing each front sensor with a non-facing rear one. [C] on the timing arithmetic, [G] that 60 ms is enough to kill cross-talk; test it

**Filtering.** Clamp to 0.03 to 2.0 m. Median of 3. Trigger on 2 of the last 3 readings below threshold. Treat "no echo" as clear but never as proof: smooth surfaces at an angle and soft clothing return nothing. [C] on the physics, [G] on thresholds

**Thresholds.** With a 0.25 to 0.5 s worst-case reaction time, the robot travels 0.10 to 0.20 m at 0.40 m/s before it reacts [C]. So: slow zone under 0.9 m (cap speed at 0.15 m/s), stop zone under 0.45 m. Search at 0.30 m/s rather than 0.40.

**Where the stop lives.** Put the hard stop in Pico firmware (front reading under 0.25 m and forward command gives zero PWM), so it works when the Pi hangs. Put the slow zone in a small `safety_gate` node on the Pi. During picks the arm and item are in the front beams, so mask the front sonar while the state is PICK.

**Main alternative:** Nav2's collision monitor as the gate. It filters velocity commands between the controller and the base and supports range sensors as sources [S7]. It is a sound choice later; for now a 60-line Python gate is easier for two students to debug. [G]

**Camera free-space (stretch, off the critical path).** Sample the floor color from a trapezoid just ahead of the robot, back-project that histogram on a 320 px wide image, scan each column upward to the first non-floor pixel, and convert to distance with the same ground homography. A few milliseconds per frame [G]. It will be fooled by shadows, painted lines and pavement color changes, so use it only to slow down. Mask out litter boxes. A monocular depth network is too slow on a Pi 5 to be worth it [G].

**Curbs and drop-offs.** Forward sonar cannot see a downward step. [C] Options in order of cost:

1. Run only where there is no drop within 2 m of the trial area, marked with cones. Free, and enough for a fair demonstration.
2. IMU tilt stop: pitch or roll over 10 degrees stops the drive (the docs put tipping at 31 degrees).
3. Two downward time-of-flight sensors (VL53L0X class, a few dollars each) ahead of the front wheels on the Pico's I2C. This is the only real cliff detection on the list. [G] on price and outdoor sunlight performance

Up-curbs are seen by sonar as walls. Add a stall stop too: high PWM with near-zero encoder speed for 0.5 s cuts the motors, which doubles as a bumper and protects the fuse.

**Safety behavior that is enough for a demonstration:** hardware E-stop on the 12 V rail (already planned); Pico watchdog that stops the motors after 300 ms without a command; a wireless deadman (gamepad button through `twist_mux`, both packaged for Jazzy [S4]) held by a person for all early runs; speed cap; base and arm never move at the same time.

**Failure modes:** cross-talk ghosts between neighbors; ground returns on rough pavement beyond about 1 m; thin posts and chair legs falling between beams; people walking in from the side blind sectors.

**Cheapest test:** drive at 0.3 m/s toward a cardboard box, a backpack, a person's leg, a 5 cm post and a wall at 45 degrees, 10 times each. Record stop distance and misses.

---

## 3. Object recognition

**Model and speed.** Use a nano-size YOLO exported to NCNN. Ultralytics' current Pi 5 table lists the nano model at 67 ms per image in NCNN at 640 px, fastest of eight formats, inference only [C, S1]. That is about 15 frames per second for the network; expect 8 to 12 end to end in Python with capture and publishing [G]. Their ONNX timings put the small model at about 2.7 times the nano [S1], so about 5 frames per second in NCNN [G]. Stay with nano.

Two free speedups:

- Export rectangular (`imgsz=[384, 640]`) since the stream is 16:9. A square 640 export pads 40% of the input with nothing. [G] on the gain; measure
- Fix the newest-frame bug first; a slow but fresh detector beats a fast stale one.

Which generation (v8, 11, 26) matters less than the data [G]. Train two on the same data and keep the better one. Note that Ultralytics code is AGPL-licensed, which is fine for an open student repo.

**Camera settings.** Lock focus (the Camera Module 3 has motorised focus [S5]; set a manual lens position for about 1.2 m) and use a short exposure in sunlight. Autofocus hunting changes image scale and blurs frames, as the committed test image shows.

**Datasets.** TACO has 1,500 images and 4,784 annotations [S2] in 60 categories under 28 super-categories [S3]. Its authors' own baseline scored only about 18 to 26 AP and did poorly on cigarettes, which they attribute to object size [S3]. TACO photos are handheld from varied viewpoints, not from 0.64 m looking 35 degrees down at pavement, so it will not be enough alone [G]. Other public sets are aerial, underwater, indoor or classification-only [S8]. Use TACO as extra training data and as one arm of experiment 3.

**Your own data is the main asset.** 600 to 1,000 frames from the robot's camera at its real pose, 5 or more pavement surfaces, sun, shade and overcast, about 30 physical items per class. At least 30% of frames with no litter, full of hard negatives: leaves, shadows, painted lines, drain covers, gum stains, cracks. Roughly 6 hours of labeling [G]. **Split by location, not by frame**: frames from one video are near-duplicates, and a random split inflates every metric. [C]

**Synthetic cut-and-paste.** The reference paper found real plus synthetic beat real alone (for example 88.8 against 86.3 on one benchmark) while synthetic alone trailed (76.2); occlusion, distractor objects and mixing several blending modes mattered most [S6]. Your version is easier than theirs because the camera pose is fixed: photograph 30 to 50 items from 8 angles, cut them out, and paste onto your own empty pavement frames at the scale the ground homography predicts for that image row, with random rotation, brightness jitter, blurred edges, a soft shadow, and leaf cutouts as distractors. A 2,200 image cigarette-butt set built this way already exists [S8]. See experiment 3.

**Classes: five, defined by how the arm treats them.** `can`, `bottle`, `cup` (cups and cartons), `crumpled` (graspable soft items), `flat` (detect, do not attempt). Report both per-class and class-agnostic scores, as TACO does [S3]. If labels are short, merge to one class for detection and keep `can` and `bottle` only.

**False positives on pavement.** Layered, cheapest first:

1. Hard negatives in training.
2. Geometry gate: drop boxes touching an image edge, and boxes whose ground-projected width is under 2 cm or over 40 cm.
3. Temporal gate in the odom frame: a track needs 3 hits in 5 frames within 15 cm (looser at range). Real litter is fixed in the world. The robot's own shadow is fixed in the robot frame, so it fails this test automatically. [C] on the logic
4. Two thresholds: 0.25 to feed the tracker, mean confidence of 0.5 to commit.
5. Re-check at the stop pose, where the item is closest and the robot is still.

**Where a cloud model slots in.** At step 5 only: send the crop at the stop pose with a 2 second timeout. No answer means proceed on the on-device decision. Log both answers so the comparison is a later experiment. Nothing waits on Wi-Fi.

**Evaluation.** Held-out locations. Report mAP50, recall at a fixed false-positive rate (per 100 background frames), and recall by range band (0.7 to 1.2, 1.2 to 2.0, over 2.0 m).

**Failure modes:** low sun and long shadows; wet pavement glare; transparent bottles; items seen only at 15 px; leaves labeled inconsistently.

**Cheapest test:** train on v0, run on one unseen location, and read the false positives by eye before doing anything else.

---

## 4. Object selection

**Recommendation: nearest in travel time, hard commitment, short memory.**

**Tracks.** One list of static points in the odom frame. Associate by nearest neighbor inside a gate of 0.15 m plus 10% of range. Position is a running average weighted toward close-range measurements. No Kalman filter; the items do not move. [G]

**Eligibility (hard filters).** Confirmed track, attemptable class, inside the trial area, not blacklisted, fewer than 2 attempts.

**Score.** Cost = distance / 0.3 m/s + |bearing| / 0.8 rad/s, in seconds. Subtract 2 s for each other eligible track within 0.5 m (cluster bonus). Lowest cost wins. Confidence is a filter, not a score term; otherwise the robot chases the most photogenic item instead of the nearest. [G]

**Commitment.** Version 1 never switches target once committed. One exception: if another eligible item sits in the drive corridor closer than the target (|y| under 0.3 m), take it first, because the wheels would hit it.

**Blind zone.** When the target leaves the image as predicted, keep it as an odom point and mark it `blind`. It stays valid for 0.6 m of travel or 10 s. No detection is expected, so its absence is not a loss.

**Giving up.**

- Not re-seen for 2 s while its predicted image position is well inside the frame: drop it and log a ghost (this is your false-positive count).
- Approach takes over 30 s: abandon.
- Two failed grasps: blacklist a 0.25 m circle. Expire the blacklist after about 10 m of travel, since odometry drift makes old positions meaningless.

**Main alternative:** a tour planner over all visible items. With 1 to 4 items in view at a time it cannot beat greedy by enough to pay for itself. [G]

**Failure modes:** two close items merging into one track; one item splitting into two tracks at long range; a blacklisted item re-detected after drift and attempted again.

**Cheapest test:** replay a recorded bag with 5 items through the tracker and count tracks. It should be 5.

---

## 5. Navigation

**Recommendation: a behavior state machine. Do not use Nav2 for version 1.**

**Reason.** Nav2 is available for Jazzy (1.3.x in the distribution [S4]), and it has a documented GPS workflow. But its value comes from costmaps filled by lidar or depth. With eight sonar beams the costmap is nearly empty, so you would tune planners and controllers to cross an open rectangle [G]. The Nav2 GPS tutorial also needs an earth-referenced heading and warns that plain GPS is good to about 1 to 2 m at best and can jump [S7]. Your lanes are 0.8 m apart and your approach needs centimeters.

**Search pattern.** Lawnmower lanes inside a coned rectangle, for example 8 x 4 m. Drive each lane with gyro heading hold and encoder distance, turn 90 degrees, shift, turn 90 degrees. Start with 0.8 m lane spacing [G]; set it properly from experiment 1 as twice the sideways offset at which recall is still 80%. Five 8 m lanes is about 3 minutes at 0.3 m/s before picks.

After a pick, resume from the current position projected onto the interrupted lane, not from a saved pose.

**Localization and how much to trust each source.**

| Source | Trust | Use |
|---|---|---|
| Encoder distance, straight line | High after calibrating wheel radius; expect 1 to 2% [G] | Lane length, blind drive |
| Encoder heading on skid-steer | Low; scrubbing makes the effective track width uncertain [G] | Do not use for heading |
| Gyro z (yaw rate) | Good over a minute after a standstill bias calibration [G] | Heading hold, in-place turns |
| Magnetometer | Poor next to four motors [G] | Skip |
| GPS | Meters, with jumps [S7] | Log only: geotag picks for a litter map, coarse geofence |

Write a small odometry node (encoder distance plus gyro heading) first; it is about 30 lines and must exist anyway. `robot_localization` is released for Jazzy (3.8.4 [S4]); a single local filter fusing wheel speed and gyro rate is a reasonable week-5 upgrade and reads well in a write-up, but it is not on the critical path. Skip `navsat_transform` and the second filter.

**Accurate approach with skid-steer.** No arcs near the item. Rotate at 0.3 to 0.5 rad/s under gyro control, finish alignment on camera, drive straight slowly with heading hold, ramp down, and calibrate the stopping overshoot once. Motors have a dead band, so corrections under about 2 degrees may not move the robot; accept the residual and let arm pan take it. The track (0.481 m) is wider than the wheelbase (0.335 m), which helps turning. [C] on geometry, [G] on behavior

**Bag full or battery low.** There is no bag sensor. Count successful picks against a per-class limit (start at 15 cans), and read battery voltage on a Pico ADC pin. On either: stop, stow the arm, signal, wait for a person. Driving home on odometry (expect up to 1 m error after a full run [G]) is a stretch goal.

**Main alternative:** Nav2 with GPS waypoint following. Revisit after submission if the area grows beyond what cones can bound.

**Failure modes:** heading drift bending lanes; turn overshoot on smooth concrete; resuming a lane in the wrong direction; drifting out of the coned area late in a run.

**Cheapest test:** drive a 4 m square three times by gyro and encoders, and measure the closing error with a tape.

---

## 6. System design

**Nodes (seven, each testable alone).**

| Node | Subscribes | Publishes |
|---|---|---|
| `detector` | camera stream | `/detections` (`vision_msgs/Detection2DArray`), 1 Hz compressed image for logs |
| `tracker` (projection, tracks, selection) | `/detections`, TF | `/litter/tracks`, `/litter/target` |
| `pico_bridge` | `/cmd_vel_safe` | `/odom`, TF odom to base_link, 8x `sensor_msgs/Range`, `/imu/data_raw`, battery, E-stop state |
| `safety_gate` | `/cmd_vel_nav`, teleop, sonar, IMU | `/cmd_vel_safe` |
| `arm_server` | actions `PickAt`, `RunMacro` | `/joint_states`, gripper position and load |
| `mission` | tracks, odom, arm results | `/cmd_vel_nav`, `/mission/state`, `/mission/event` |
| `fake_robot` (test only) | `/cmd_vel_safe` | fake odom and fake detections |

**Messages.** `Detection2DArray` has a header; each detection carries a box and a list of hypotheses with a string `class_id` and a score [C, S4]. Stamp the header with the frame's capture time and set `frame_id` to the camera optical frame. `vision_msgs` is released for Jazzy (4.1.1 [S4]). Define two small custom messages for tracks (`id`, `class_id`, `score`, `hits`, position in odom, `last_seen`, `in_view`). Add a `base_footprint` frame on the ground so nobody has to remember whether base_link is at axle height.

**Arm: wrap LeRobot in a ROS action server. Do not move to ros2_control now.**

- Reason: your calibration, collision-checked poses, macros and input shaping are all in LeRobot code. Community ros2_control packages for the SO-101 exist [S13], and a Feetech driver is released for Jazzy [S4], but adopting them means rebuilding what you already have, most likely with MoveIt on a Pi.
- `PickAt` takes a point in base_link, a class and a jaw yaw; returns outcome, final gripper position and peak load.
- Failure modes: Python dependency clashes between LeRobot and the ROS system Python (LeRobot currently requires Python 3.12 or newer, which matches Ubuntu 24.04 [C, S9]); the studio and the node both opening the serial port; a blocking motion freezing other callbacks (use an action with its own thread).
- Cheapest test: call the action from the command line 20 times in a row and check for a hung port.

**Wheel loop: on the Pico.** [C] At 110 rpm the encoders produce about 7,300 counts per second per wheel. The Pico's programmable I/O counts that exactly; Linux user code will drop edges. The Pico also gives a watchdog that survives a Pi crash, and it removes the unverified Pi 5 PWM question (RISKS item 10). Speed loop at 100 Hz per side; plain text lines over USB serial at 50 Hz. Avoid micro-ROS: it adds an agent and a build system for no benefit here [G].

Two wiring facts to plan for:

- **Pin budget.** Eight encoder pins, eight motor pins, four sonar triggers, eight sonar echoes and two I2C pins is 30; a Pico exposes 26. Drive each side's two motors from one PWM and direction pair (saves 4), or read one encoder per side. [C] arithmetic
- **HC-SR04 echo is a 5 V signal**; the Pico is 3.3 V. Use a divider on every echo line. [C]

**Simulation: do not port the Gazebo package before submission.** The pairing is right: Gazebo's own table marks Harmonic as the supported match for Jazzy, installed through `ros-jazzy-ros-gz` [C, S14]. But porting the Humble and Classic package costs about a week [G] and tests none of the risky parts (grasping, detection, scrubbing). A 100-line `fake_robot` node that integrates `/cmd_vel` and emits noisy fake detections tests the state machine on a laptop today.

**Detector node fixes (from reading `detector_node.py`).** [C]

- A 1 ms timer calls a blocking read and inference in one callback (lines 209 to 281). Move capture to a thread that keeps only the newest frame and its capture time.
- The stream is `tcp://127.0.0.1:8888` through OpenCV, so frames are encoded and decoded on the same Pi. Measure end-to-end latency by filming a blinking LED.
- It publishes only the top-confidence box, with no class (line 302). Publish all boxes.
- `mapping_enabled` defaults to true and posts camera frames over plain HTTP to a fixed LAN address (lines 66 to 72). Default it off; on campus those frames can contain people.
- The model path points into one person's home directory (line 50). Make it a launch parameter.

**Logging.** Three layers, all automatic:

1. `rosbag2` of every topic except raw images (released for Jazzy [S4]).
2. `trials.csv`, one row per pick attempt: run id, item id, class, measured (x, y) at stop pose, final (x, y), attempt number, gripper position and peak load, outcome code, duration, plus a column a human fills in afterwards with the true outcome.
3. A JPEG at each decision: stop-pose frame, post-grasp frame.

Each run also saves the git hash, parameter file and model file hash. Without that the experiments cannot be reproduced.

**State machine.** A plain Python class with an explicit transition table. Every state has a timeout and logs entry and exit.

```
IDLE -> SEARCH -> APPROACH -> MEASURE -> ALIGN -> BLIND_DRIVE -> PICK -> VERIFY -> DEPOSIT -> SEARCH
```

Failure states, each with a defined exit:

| State | Entered when | Exit |
|---|---|---|
| TARGET_LOST | Track dropped during approach | SEARCH, log ghost |
| ALIGN_TIMEOUT | 4 rotation tries without |y| under 1.5 cm | PICK anyway if inside arm tolerance, else back up and MEASURE |
| GRASP_FAILED | Verify says not held | Retry once, then blacklist, SEARCH |
| DROPPED_IN_TRANSIT | Gripper closes fully during carry | Log, SEARCH |
| OBSTACLE_HOLD | Sonar stop zone | Wait 5 s, resume prior state; after 3 holds skip the target |
| ARM_FAULT | Servo error or overload | Try stow, then HALT |
| STALL, TILT | Encoder stall or IMU tilt | HALT |
| COMM_LOST | No Pico data for 0.5 s | HALT (Pico has already stopped) |
| BAG_FULL, LOW_BATTERY | Counter or voltage | Stow, signal, HALT |
| ESTOP | E-stop line | HALT until reset by a person |

---

## 7. Schedule (today Wed 2026-11-11, submission Wed 2027-01-20)

I do not know the school's exam and holiday dates; I assumed Thanksgiving on Nov 26 and a break from about Dec 21 to Jan 3 [G]. Check whether campus pavement is reachable during the break, since the field trials fall there.

| Week | Dates | Build | Exit test |
|---|---|---|---|
| 1 | Nov 11 to 17 | Code in git; paperwork check; static pick grid; wider pan study and pick table; ground homography; detector fixes; dataset v0 recorded | Pick rate per item type known; projection error under 3 cm at 0.9 m |
| 2 | Nov 18 to 24 | Pico firmware and bridge; gamepad driving; odometry calibration; sonar on Pico; arm action server; model v0 | 5 m straight within 5 cm; `PickAt` works 20 times in a row |
| 3 | Nov 25 to Dec 1 | Tracker, mission state machine, approach, pick, deposit, tested first on `fake_robot` | **M1: one can from 1.5 m, bagged 3 of 10** |
| 4 | Dec 2 to 8 | Gyro heading hold; safety gate; failure states; grasp verification; retry; full logging | **M2: 10 runs in a row with no crash and complete logs** |
| 5 | Dec 9 to 15 | Lawnmower search; multi-item selection; dataset v1 and synthetic generator; pilot of experiment 1 | 5 items in a 6 x 3 m area, at least 3 bagged |
| 6 | Dec 16 to 22 | Experiments 1 and 2 (controlled). **Feature freeze Dec 22** | Data files complete |
| 7 | Dec 23 to 29 | Experiment 4 (approach ablation); experiment 3 training runs finish in the background | Bug fixes only |
| 8 | Dec 30 to Jan 5 | Experiment 5 field trials, sites 1 and 2 | 12 runs logged |
| 9 | Jan 6 to 12 | Site 3 and repeats (buffer); analysis and figures | All plots drafted |
| 10 | Jan 13 to 19 | Write-up, video, submit **Jan 19**, one day early | Submitted |

**Cut list, in the order to cut:**

1. Gazebo port (cut now).
2. Cloud second opinion.
3. GPS fusion, Nav2, drive-home.
4. Camera free-space detection.
5. `robot_localization` filter (keep the simple odometry node).
6. Five classes down to `can` and `bottle`.
7. Lawnmower over an area down to a single 8 m strip.
8. Jaw orientation from the mask (keep the 90 degree retry).
9. Experiment 3 down to three arms and one seed.

**Never cut:** `trials.csv` logging, experiment 2, the E-stop and Pico watchdog. If M1 slips past Dec 8, drop experiment 4 and make experiment 5 smaller rather than skipping controlled trials.

---

## 8. Experiments

Common rules: define success before starting (item inside the bag within 60 s with no human touch). Randomize trial order. A person records the true outcome for every trial. Report proportions with Wilson 95% intervals. With 20 trials per cell the interval is about +/- 20 points, so use 30 for the cells you most care about. For "which parameters matter most", fit one logistic regression of success on item type, lateral offset, range and condition.

**E1. How accurately can one camera locate litter on the ground?**

- Variables: range (0.7, 1.0, 1.5, 2.0, 2.5 m) x lateral offset (-0.4, 0, +0.4 m) x item (standing can, lying can, lying bottle, crumpled paper). Second factor: deliberate camera pitch error of 0, 1 and 2 degrees with a shim.
- Trials: 60 placements x 3 repeats = 180 static captures, about 2 hours.
- Metrics: error in x and y against tape measure; share of estimates inside the arm's tolerance; detection recall by cell (this sets lane spacing).
- Prediction: error grows from about 1.6 cm per degree of pitch at 0.7 m to about 11 cm per degree at 2.2 m.
- Surprising: error at 2 m under 5 cm (a single far measurement would be enough), or box jitter rather than pitch dominating.

**E2. Which litter can a low-cost gripper pick up, and can the robot tell when it failed?**

- Variables: 6 to 8 item types x 2 orientations; position randomized in the pick zone; optional second factor: bare fingertips against rubber pads.
- Trials: 20 to 30 per cell, 240 to 400 grasps at about 30 s each, base stationary.
- Metrics: success rate; failure category (missed, slipped at lift, dropped in transit, did not release); success against lateral offset (the real arm tolerance); confusion matrix of the servo-based "held" signal against the truth.
- Surprising: lateral tolerance much wider than the +/- 9 degree study; can orientation not mattering; the load signal working on thin items.

**E3. Real, synthetic or mixed training data?**

- Arms: real only at 50, 150, 400, 800 images; synthetic only (3,000); each real size plus synthetic; TACO plus real 400. Three seeds each, about 30 training runs on a free cloud GPU.
- Test set: 200 or more real frames from two locations never used in training, plus 100 litter-free frames.
- Metrics: mAP50; recall at 1 false positive per 100 background frames; recall by range band.
- Expected from prior work: mixed beats real, synthetic alone trails [S6].
- Surprising: synthetic-only within 5 points of real-800; 150 real plus synthetic matching 800 real (that would be a practical recipe for other teams); TACO hurting.

**E4. Which approach strategy matters most? (ablation)**

- Conditions: (a) steer to image center, the logic in today's node; (b) ground projection, continuous drive, no stop; (c) stop-and-measure, rotate, blind drive; (d) c plus gyro heading hold; (e) optional, portrait camera with no blind zone.
- Start poses randomized: 1.5 to 2.5 m, +/- 0.5 m sideways, +/- 20 degrees heading. One standing can.
- Trials: 20 per condition, 80 to 100 runs at about a minute each.
- Metrics: item position in base_link at the moment before the grasp (photo from above against a tape cross); pick success; time.
- Prediction: (a) leaves the can about 16 cm off the arm line and mostly fails.
- Surprising: (b) as good as (c), or the gyro making no measurable difference.

**E5. End-to-end field trial with a failure funnel.**

- Setup: 8 x 4 m coned area, 10 items per run (Tier A and B) plus 3 distractors (leaves, a painted mark), positions from a random number table.
- Variables: 3 sites x 2 lighting conditions (hard midday shadows, overcast or late afternoon) x 3 runs = 18 runs, 180 item opportunities.
- Metrics: the funnel per item: present, detected, approached, attempted, grasped, bagged. Also false attempts on distractors, seconds per item, items per watt-hour, human interventions per run.
- The stage where most items are lost is the direct answer to "which design parameters matter most".
- Surprising: detection not being the weakest stage; lighting mattering more than item type.

E1 and E2 need only a parked robot, so they can start before the drive base works. E5 is the headline; E1 to E4 explain it.

---

## Sources opened

- [S1] Ultralytics, Raspberry Pi guide and Pi 5 benchmarks: https://docs.ultralytics.com/guides/raspberry-pi . Quote: "NCNN delivers the best inference performance on Raspberry Pi devices"
- [S2] TACO paper abstract (arXiv 2003.06975): https://arxiv.org/abs/2003.06975v2
- [S3] TACO paper full text: https://ar5iv.labs.arxiv.org/html/2003.06975 ; repository: https://github.com/pedropro/TACO
- [S4] ROS distribution file for Jazzy (package versions): https://raw.githubusercontent.com/ros/rosdistro/master/jazzy/distribution.yaml ; `vision_msgs` message definitions: https://github.com/ros-perception/vision_msgs/tree/ros2/vision_msgs/msg
- [S5] Raspberry Pi camera documentation: https://www.raspberrypi.com/documentation/accessories/camera.html
- [S6] Dwibedi, Misra, Hebert, "Cut, Paste and Learn": https://ar5iv.labs.arxiv.org/html/1708.01642
- [S7] Nav2 tutorials: GPS navigation https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html ; collision monitor https://docs.nav2.org/tutorials/docs/using_collision_monitor.html
- [S8] Waste datasets review: https://github.com/AgaMiko/waste-datasets-review
- [S9] LeRobot source (`so_follower.py`, `pyproject.toml`, version 0.7.1 on the day checked): https://github.com/huggingface/lerobot
- [S10] Paez-Ubieta et al., "Vision and Tactile Robotic System to Grasp Litter in Outdoor Environments": https://arxiv.org/html/2407.08575v4
- [S11] Kao, Pathapati, Davis, "AI for Green Spaces" (arXiv 2601.11876), read through a summary page: https://arxiv.deeppaper.ai/papers/2601.11876v1
- [S12] Engadget report on the VERO quadruped (Journal of Field Robotics, 2024): https://www.engadget.com/this-vacuum-robot-dog-can-find-and-suck-up-trash-with-its-feet-203952526.html
- [S13] SO101 ROS 2 workspace docs: https://so101-ros2.readthedocs.io/latest/
- [S14] Gazebo, "Installing Gazebo with ROS": https://gazebosim.org/docs/all/ros_installation

**Could not open:** docs.ros.org pages for `vision_msgs` and `robot_localization` (blocked by bot protection; I used the package source and the distribution file instead, and the Nav2 GPS tutorial for the `robot_localization` setup), and tacodataset.org/stats (fetch needed an approval that was not given). I did not find a measured NCNN figure for a rectangular or 320 px export on a Pi 5, so those speedups are marked as guesses. S11 and S12 are secondary summaries, not the papers themselves.
