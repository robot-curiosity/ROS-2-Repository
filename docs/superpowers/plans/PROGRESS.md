# Behaviour coordinator — progress

Resume point for `2026-07-30-behaviour-coordinator.md`.
Last updated 2026-08-01. **Phases 1 and 2 complete. The mission runs end to end
to PICKING with bearing and range both closed-loop. Next: the gripper never
opens as commanded — see "Where the mission stands".**

Branch: **`experimental/behaviour-coordinator`**, which is pushed and tracking
`origin`. `master` is untouched. (This file said `feature/...` and "nothing
pushed" through Task 9; both were stale.)

---

## Done

| Task | Commit | Result |
|---|---|---|
| Design spec | `80885d9` | Approved |
| Implementation plan | `77ad22c` | 17 tasks, 5 phases |
| Container tooling | `f450191` | `scripts/dev.sh` + `scripts/docker/Dockerfile` |
| **1** Sensor pod frames | `408c840` | 27 links / 26 joints, +3/+3 over baseline |
| **2** Camera + ToF sensors | `e1f3d2b` | Both publish real data under llvmpipe |
| **3** Bridge camera/ToF/arm | `fecfdab` | **Arm moves from ROS** |
| **4** Litter objects | `84ce48f` | 3 non-static models, poses stable |
| **5** Measure arm workspace | `6baed36` | `docs/arm_workspace.md`, 210 poses |
| **5b** Pod aim investigation | `b9820a5` | Pod stays `rpy 0 0 0`; confirm pose found |

Plan corrections found by executing it: `50454aa`, `b696145`, `20936ee`,
`ca92441`, `2dbf7c3`.

Phase 2 complete: **6** `drivebase_msgs` (`bb23167`), **7** detection adapter
(`881a7c5`), **8** pixel-to-point localisation (`a595fed`), **9** URDF-derived
kinematics, **10** closed-form IK — 50 tests green.

## Next — as of 2026-08-01

The mission runs end to end as far as **PICKING**. `APPROACHING → CONFIRMING →
PICKING` fired seven times across two full runs, having never fired once before
the pod-orientation fix below. Confirmation settles in 0.5–1.2 s with bearing
driven to zero.

**The one remaining blocker: nothing nulls RANGE.** The gate into CONFIRMING
tests apparent size and horizontal centring; neither constrains radius, so the
base stops wherever it stops. Measured over one run, the four grasp targets came
out at r = 0.200, 0.039, 0.015 and 0.070 against a reachable band of
**0.075–0.198** — two of them missed an edge by 2 mm and 5 mm.

The signal for that loop now exists, which it did not before: **vertical image
error is the radial direction.** At the grasp plane the band maps to vertical
error **+0.4655** (near edge) → **−0.0828** (far edge), centred **+0.1862**.
Creeping the base on that during CONFIRMING is the next task, and it is a wide,
comfortable target rather than a knife-edge.

### Where the mission stands, 2026-08-01 (three fixes in)

Latest 190 s run: **3 entries to PICKING, 2 grasp sequences executed, 0 pickups.**
Before this session's work, PICKING had never been reached at all.

| Blocker | State |
|---|---|
| Pod camera imaged the world sideways | **Fixed** — mount rolled +90°, guarded by `scripts/verify_pod_orientation.py` |
| Aim loop integrated the wrong way | **Fixed** — `+=`, authority measured at −0.431 /rad |
| Grasp height taken from a ToF that reads the ground | **Fixed** — ground plane datum, mount height from tf |
| Pre-grasp raked 105 mm sideways through the litter | **Fixed** — both poses solved by IK, descent is vertical |
| **Nothing nulls range** | **Fixed 2026-08-01** — the base creeps on radius during CONFIRMING and the gate asks the IK. See below |
| **`shoulder_lift` has no headroom** | **Open, decision pending** — see `docs/arm_workspace.md` §7 |
| **Gripper never opens as commanded** | **Fixed 2026-08-02** — the shared `p_gain` 4.0 against a constant ~1.2 N·m. `gripper_p_gain` is 40.0 and the joint now tracks to 0.035 rad. See below |
| **The gripper has no opening to grip with** | **Open, and it is the real blocker** — the fixed jaw's collision box swallows the grasp point. Found 2026-08-02, after the gain fix produced two clean grasps and zero pickups |
| **Detector locks onto barrier edges** | **Resolved 2026-08-01 — `value_min` is not the lever.** Measured, not reasoned; see below |

### The range loop, 2026-08-01

Closed on **radius in metres**, not on vertical image error as planned. The
coordinator already computes the grasp target every tick from the same ray ×
ToF product the grasp uses, so radius is observable outright; routing it
through `center_y` and a +0.1862 setpoint would have added a pixels-per-metre
calibration valid at one camera pose and one object size to recover a number
already in hand.

`grasp_min`/`grasp_max` are **gone**. They compared camera range against a
window converted from the workspace by scaling along the beam — the unit
mismatch flagged in the config since it was written. The gate is now the same
pair of IK solves the grasp sequence runs, so it cannot pass a target the grasp
then refuses. The band the base creeps toward is scanned from the URDF at
startup (`reachable_radius_band`), requiring both the grasp *and* the point
above it to solve; `scripts/verify_grasp_band.py` reports it without a
simulator.

Measured, 200 s run on a real GPU:

| | Before | After |
|---|---|---|
| Graspable band | 0.075–0.198, by hand | **0.0760–0.1980, scanned** |
| Grasp radii | 0.200, 0.039, 0.015, 0.070 | **0.1528, 0.1187, 0.1499** |
| Inside the band | none of four | **all three** |
| `unreachable, abandoning` | 3 | **0** |
| Confirmation settling | — | 1.5–3.2 s |

The loop is visible in one sequence: r = 0.2133 (`reachable=False`, past the far
edge) → 0.1709 → 0.1499, the base creeping 63 mm inward while the arm held
`confirm_pose`. `runs/trace/approach.csv` traces the commands through
`velocity_smoother` and `collision_monitor` to odom, including three **reverse**
creep commands — the case that produced 0.039 and 0.015 and could never be
corrected, because nothing backed the base up.

**Pickups are still not verified.** The gripper-opening blocker is untouched.

### The gripper gain, 2026-08-02 — measured, and fixed

The gripper reached 0.937–1.088 of a 1.200 command. `scripts/measure_gripper_sweep.py`
sweeps it with the rest of the arm held still, up and then back down, which
separates the three candidates the single reading could not:

| gripper | achieved at `p_gain` 4.0 | at 40.0 |
|---|---|---|
| 0.400 | +0.110 | +0.370 |
| 0.800 | +0.520 | +0.770 |
| 1.200 | +0.951 | +1.160 |
| 1.600 | +1.362 | +1.561 |

**It was the gains, and the arithmetic is exact.** The joint carries a constant
~1.2 N·m opposing whichever way it is moving — the error is that torque over
the gain and nothing else, 0.317 rad at 4.0 and 0.035 rad at 40.0, a clean
1/p. Not a limit (1.6 tracks fine, and the limit is 1.745), and not contact:
the error reverses sign with the direction of travel, which contact does not.

`--step` shows why it looked like a hard stop. At 4.0 the joint reaches 0.883
within one second and then **crawls at 0.003 rad/s** on the integrator alone —
still 0.180 short after 45 s. `grasp_hold_seconds` is 1.5, so every mission
reading was a snapshot of a ramp. At 40.0 it is at 1.170 by 1.5 s.

`gripper_p_gain` is a launch arg on `sim.launch.py`, defaulted to 40.0 for the
gripper alone; the five joints that hold the arm against gravity keep 4.0.
Revert with `gripper_p_gain:=4.0`. It is a sim plugin gain and says nothing
about the hardware, where the STS3215 closes its own loop.

### The gripper has no opening — the actual blocker, 2026-08-02

With the gain fixed, a 200 s run reached PICKING twice, both times with range
and bearing closed and the IK reporting the target reachable (r = 0.1236 and
0.1272, `settled=True`). **Both picked up nothing.** Sampling all three litter
models' world poses through the run, no piece of litter ever left the ground:
z stayed within 20 µm of its resting height while `litter_can_a` was shoved
about two metres across the field.

`scripts/verify_gripper_clearance.py` says why, off the URDF with no simulator:

```
grasp point (gripper_frame_link) in arm_gripper_link: -0.0079 -0.0002 -0.0981
  *** the grasp point is INSIDE fixed box [1] ***
     centre -0.0026 -0.0020 -0.0517   size 0.0652 x 0.0520 x 0.1054
largest cylinder that fits at the grasp point: 0.0000 at every gripper angle
```

The fixed jaw's collision geometry is a **65 × 52 × 105 mm bounding box of the
whole wrist-roll follower**, and it contains the grasp point. In physics the
gripper is a solid brick with a flap on it: the moving jaw swings, but there is
no space between the jaws for anything to be. The cans were not dropped, they
were never held — they were pushed by the block.

This is the cost of `STATUS.md`'s "arm collision geometry is boxes, not meshes",
which is right for five joints that only have to not hit things and wrong in
kind for the one joint whose entire job is the gap between it and its neighbour.
Gazebo grasps the boxes; the meshes are only drawn.

**Not fixed, because the fix is a choice** — the jaw meshes are 1.44 MB
(`wrist_roll_follower_so101_v1.stl`) and 1.41 MB (`moving_jaw_so101_v1.stl`),
squarely inside the 0.5–2.6 MB range that decision cited, and
`so101_macro.xacro` is generated by `scripts/vendor_so101.py` and must not be
hand-edited. Options, in the order they look cheap:

1. **Mesh collisions on the two jaw links only.** Smallest change, keeps the
   geometry honest, and costs mesh-mesh contacts on 2 of 11 links against
   15 g of litter. Goes in `vendor_so101.py`, not the generated file.
2. **Decompose each jaw into a few boxes** in `vendor_so101.py` — a jaw is
   roughly two slabs, so 2–3 boxes per side may be enough. Keeps box-box
   contacts, at the cost of numbers that are approximations of a mesh rather
   than imports of it.
3. **Accept that grasping is not simulable here** and verify pickup on
   hardware. Cheapest today, and it leaves the whole PICKING path unverified
   until the arm exists.

Nothing else in the pick path is known-broken. Aim, range, height and the
gripper command are all now measured and closing.

### `value_min` — measured, and the plan was wrong

Raising it toward 100 to exclude shaded barrier edges fails twice over
(`scripts/measure_value_min.py`, 194 frames, `runs/value_min/sweep.csv`):

- **The litter is not much brighter.** Of every pixel passing the hue and
  saturation bounds, the value floor is **97** and the 10th percentile is 100.
  100 already clips litter; at 120 the mask empties and the detector reports
  nothing in 0 of 194 frames, silently.
- **It buys nothing.** Those bounds admit 713 pixels in exactly three blobs,
  centred on the three litter models, all at hue 0. The barriers contribute
  none. The sampled barrier corner was a shading condition, not the steady
  state.

Left at 60. If barrier edges do dominate a mask, the lever is shape or area,
not brightness.

### Give Nav2 time to come up, or the approach silently never closes

Two of the three runs behind the numbers above failed for a reason that has
nothing to do with the coordinator, and presents identically to a broken
approach: **Nav2's lifecycle bringup stalls**, `behavior_server` logs `failed
to send response to /behavior_server/change_state (timeout)`, and the manager
never proceeds to `velocity_smoother` or `collision_monitor`. With no
`velocity_smoother`, `/cmd_vel_nav` has no subscriber, and a volatile publisher
with no subscriber discards silently — so the coordinator servos an approach
that the base never executes, `bbox_frac` never grows, and every approach times
out at 30 s.

Launching navigation 20 s after the simulator reproduced it; 40 s, then 35 s
more before the coordinator, did not. **Check `Managed nodes are active` in the
navigation log before reading anything into a run.**

### Next, in detail

**1. Give the jaws an opening — see "The gripper has no opening" above.** This
is the one thing between a converging pick and a pickup, and it needs a
decision about collision geometry rather than an investigation.

The item below is **done**; kept for the reasoning, which held up.

**1a. The gripper never opens as commanded.** `gripper_open` is 1.200 and the
joint reaches 0.937–1.088 across every pose, well inside the URDF limit of
1.74533. It is the last thing between "3 entries to PICKING" and an actual
pickup, and **no pickup has ever been verified** — the runs above prove the arm
is aimed at the right point, not that it closes on anything.

Not yet investigated, in the order worth trying:

- **Read the joint back.** `/joint_states` orders joints alphabetically; confirm
  which value is `arm_gripper` before concluding anything. Command 1.200 with
  the mission stopped and watch the joint settle, so the reading is not confused
  by a sequence step moving on underneath it.
- **Suspect the gz `JointPositionController` gains, not the command.**
  `gazebo.xacro`'s `arm_joint_hold` macro applies `p_gain 4.0`, `i_gain 0.05`,
  `cmd_max 10.0` to all six joints uniformly. The gripper joint's `effort` is
  10 and it is the smallest, lightest joint on the arm; a P term that holds the
  shoulder against gravity is not obviously right for it. A steady-state offset
  that scales with commanded angle is the signature of exactly this.
- **Check for contact.** If the jaws are closing on the litter, the arm, or
  themselves, the controller will hold short and report nothing. The collision
  geometry is boxes, deliberately.
- **Only then suspect the command path.** `arm_driver.go_to` clamps
  `wrist_roll` and nothing else, so the gripper value reaches the publisher
  unmodified; `grasp_sequence` passes `gripper_open` into three of five steps.

The measurement worth having first is whether the shortfall is a fixed offset
or a fixed ratio — command a sweep (0.4, 0.8, 1.2, 1.6) with the arm stationary
and record what the joint reaches. That distinguishes a gain problem from a
limit or a contact, and takes one run.

**2. The mount decision.** `docs/arm_workspace.md` §7 measures the workspace at
a 224 mm mount without adopting it; `shoulder_lift` has no headroom at the
current 260 mm. This is a harness-routing and chassis call, **not one to make
from the simulator** — the print may not have the bracket either way. It is
recorded as pending on purpose.

**3. Timestamp alignment, still not investigated.** The coordinator pairs the
newest detection with the newest ToF and `CameraInfo` with no synchronisation.
There is a staleness guard (`detection_stale_after`) but nothing aligns the
three. It has not caused a visible failure yet, and now that the grasp target
drives a control loop rather than a one-shot gate, a mismatched pair moves the
base rather than just wasting a frame.

### Older, still open

**Task 11: `sim_litter_detector` — code complete and working in sim, one
question open.** Details below.

Execution mode agreed: **subagent-driven**, one agent per task, reviewed
between tasks.

### Task 11 state as of 2026-07-31

All four files are written, build clean, and the node runs in the sim:

- `src/drivebase_sim/drivebase_sim/sim_litter_detector.py` (new)
- `src/drivebase_sim/setup.py` — entry point added
- `src/drivebase_sim/package.xml` — `std_msgs`, `cv_bridge`, `python3-opencv`
- `src/drivebase_sim/launch/sim.launch.py` — `litter_detector:=true` flag
- `scripts/verify_litter_detector.py` (new) — the acceptance check, reusable

**Verified against the real thing before writing:** the plan's JSON key layout,
both topic names (`/vision/target`, `/vision/direction`) and the `deadband`
default of 0.12 all match `trash_vision/detector_node.py` exactly. The shim is
interface-faithful.

**Measured in sim** (headless, real GPU, one run):

| Phase | Result |
|---|---|
| At spawn, nothing in view | 180 msgs, `detected=false` throughout, `NO_TARGET` |
| Driving toward `litter_can_a` at (1.8, 0.5) | 257 msgs, **`detected=true` in 148**, all of `LEFT`/`CENTER`/`RIGHT` seen |

So the loop runs end to end and the direction logic exercises every branch.

#### RESOLVED 2026-08-01: the barriers are NOT kept out of the mask

The note below reasons that "the barriers are orange at hue ~13, so the upper
bound of 10 is what keeps them out." That holds for barrier faces in full light
and **fails on the shaded edge**, because shading rotates orange toward red.

Sampled from a live mission frame where the detector had locked onto a barrier
edge with no litter near it, the pixels there are `srgb(82,28,0)`:

| | Value | Threshold | Passes? |
|---|---|---|---|
| Hue (OpenCV 0-179) | **10.2** | `hue_high` 10 | marginal — some pixels fall under |
| Saturation | 255 | `saturation_min` 120 | yes |
| Value | 82 | `value_min` 60 | **yes** |

So the barrier edge is inside the mask, and the consequence is visible in the
run: the coordinator repeatedly entered APPROACHING on a blob of
`bbox_frac ≈ 0.069` that never grew, drove at it, gave up, and re-acquired it —
six times in one 190 s mission. It was chasing a barrier corner.

**`value_min` is the lever, not `hue_high`.** The offending pixels are dark
(value 82 of 255) because they are shaded; lit litter is not. Raising
`value_min` toward ~100 excludes them without touching the hue bound, which is
already as tight as it can safely go. Not yet changed — it needs a run to
confirm it does not also drop genuine litter in shadow.

#### The one open question: the hue range

I changed the plan's `hue_low`/`hue_high` defaults from **0..10** to
**170..10**, and added `_hue_mask`, which reads `low > high` as a *wrapping*
range and ORs both ends. Reason: OpenCV packs hue into 0-179 and pure red sits
at 0, so a lit red object scatters to both ends; a naive `inRange(0, 10)` keeps
the bright half and silently drops the shaded half, which **moves the centroid
rather than emptying the mask** — a wrong answer, not a missing one.

Detection demonstrably works with 170..10. What is **not** settled is whether
the wrap was necessary, because the diagnostic that was meant to answer it
did not:

> `hue_report` in `verify_litter_detector.py` sampled hue 18 (100k px) and
> hue 30, with **zero** pixels in either 0..10 or 170..179.

That reads like a contradiction but almost certainly is not — **it samples the
last frame received, and that run ended with `detected=false`, i.e. no litter
in view.** Hue 18/30 is the orange barrier and the ground, not the can. The
measurement is inconclusive, not contradictory. Do not act on it as written.

**To close this out:** re-run the verifier with the histogram taken from a
frame where `detected=true` (gate `hue_report` on the last target message
having `detected: true`, or store the frame that produced the largest blob).
Then either keep 170..10 with evidence, or revert to the plan's 0..10 if the
litter turns out to sit entirely in the low band. Note the barriers are orange
at hue ~13, so **the upper bound of 10 is what keeps them out of the mask** —
whatever else changes, do not raise it past ~12.

Nothing here blocks Task 12; the interface is stable either way.

### What Task 10 changed about the plan

The plan's draft `solve()` was wrong in the two ways this file warns about, and
both were caught by tests rather than by reading:

- It took `r = hypot(x, y)` from the base origin. That is trap 1 below — the
  radius must be anchored on the pan axis, so `solve` now uses
  `arm.bearing()` and `arm.to_planar()` rather than rebuilding the geometry.
- It never applied `arm.sense`, so every pitch joint came out negated. Invisible
  at the zero pose, wrong everywhere else, exactly as warned.

`PlanarArm` gained **`bearing(point)`** (the pan command that faces a point) and
**`from_planar(r, z, pan)`** (the inverse of `to_planar`). The tests are built
on `from_planar`; the plan's versions constructed targets as
`(r*cos(pan), r*sin(pan), z)`, which names points the arm cannot reach at that
bearing and asserts a pan the solver has no reason to return.

`solve` tries **both elbow branches**, elbow-down first. The second is an exact
solution of the same target, not a relaxation, so preferring one and accepting
the other costs no accuracy and widens the usable workspace.

## Measured figures — use these, do not re-derive

| Parameter | Value | Source |
|---|---|---|
| `grasp_range` | **0.397 m** | lowest gripper pose, z = 0.0354 m |
| `grasp_min` | **0.287 m** | nearest x with gripper under 0.060 m |
| `grasp_max` | **0.481 m** | furthest x with gripper under 0.060 m |
| Search pose | pan 0, lift **−1.4**, elbow **0.0**, wrist_flex **0.4** | ToF 1.08–1.14 m of ground; gripper tucked to x = 0.267 m |
| Confirm pose | pan 0, lift **0.9**, elbow **0.46**, wrist_flex **−1.40** | ToF 0.2716 m, beam lands x = 0.392 m, mid grasp window |
| `wrist_roll` limit | **not measured** | harness routing; no simulator analogue. The plan's ±1.0 rad is still a guess and is *not* endorsed. |
| Camera intrinsics | fx = fy = 493.7925, cx = 320, cy = 240, 640×480 | `camera_info`, = 320/tan(1.15/2) |

Reach reproduces `STATUS.md`: 220 mm below mount (vs 219), gripper 40.1 mm above
ground (vs 41). The sensor pod did not disturb the arm's kinematics.

### Those radii are `base_footprint` x, NOT the arm's planar r

Found in Task 10, after the first version of a test asked for a point 25 cm
past the arm's reach and got a truthful `None` back.

`grasp_range`, `grasp_min` and `grasp_max` above were measured as gripper **x
in the robot frame**. The IK works in planar `r`, measured from the **pan
axis**, and the two differ by a constant **0.254 m** — the arm sits on the
front face at x = 0.215, and the pan axis is a further 38.8 mm out.

```
base_footprint x  ≈  planar r + 0.254
```

So the 0.287–0.481 m window is planar **r = 0.033–0.227 m**. Feeding a
robot-frame number to `solve()` as a radius always fails, and fails *quietly*
— `None` is also what a genuinely unreachable point returns.

**At grasp height the arm is nearly fully extended.** With litter at
z = −0.2246 in `arm_base_link` (35.4 mm above ground, minus the 260 mm mount),
the reachable band is only **r ≈ 0.10–0.15 m**, ~50 mm wide, and the approach
angle is forced to **−90°…−87°** — vertical, with a few degrees of slack. The
model's deepest tip is z = −0.2248 at r = 0.145, which independently
reproduces the 224.9 mm figure above.

Two consequences for Task 12 onward:

1. **The descent has no choice of approach angle.** Straight down is the only
   thing that reaches, so it is not a preference the grasp sequence gets to
   tune. Higher up there is real freedom — at z = −0.15 the band runs
   r = 0.10–0.33 — so an intermediate waypoint can be aimed, the grasp cannot.
2. **The base must be driven into a 50 mm radial band**, which is tighter than
   the ~0.35 m per-turn drift in `STATUS.md`. This is the same conclusion the
   design already reached from the other direction — aim the arm with the
   camera, never by driving to a coordinate — but it now has a width attached.
   `test_the_grasp_height_band_is_narrow_and_the_approach_is_forced` pins it.

   **Both of those numbers were superseded on 2026-08-01 — see "The grasp
   height is a cliff" below.** The band at 35.4 mm is 19 mm, not 50; and it is
   19 mm only because 35.4 mm is the wrong height to grasp at.

### The grasp height is a cliff, added 2026-08-01

`grasp_range`'s z of **0.0354 m is the arm's lowest reachable height, and that
is exactly why it is the wrong grasp target.** It sits on the singular edge of
the workspace, where the radial window collapses:

| Grasp height above ground | Reachable radial window, straight down |
|---|---|
| 35.4 mm — the lowest pose | **19 mm** (r = 0.127–0.146) |
| 40 mm | 72 mm |
| 50 mm | **123 mm** (r = 0.075–0.198) |
| 70 mm | 181 mm |

A 4.6 mm rise nearly quadruples the tolerance. Relaxing the approach angle does
not substitute for it — the whole −90°…−87° range unions to 24 mm. The
coordinator now grasps at **50 mm** (`grasp_height`), and takes that height from
the **ground plane**, not from the ToF: the ToF is a single ray, at confirm_pose
it lands beside the litter, and the point it yields sits on the ground (measured
1.2 and 1.5 mm below it — the ground read to within its own noise). The ray
gives bearing and radius; the ground gives height. Mount height comes from tf.

---

## Environment

**Two machines now. Check which one you are on before trusting a command.**

### Arch Linux workstation (current, from Task 9 onward)

**Docker and the GPU both work here — corrected 2026-07-31.** This section said
"no Docker, no ROS 2, no Gazebo" through Task 10 and that is no longer true;
the user corrected it and it was verified:

- `docker` runs **without sudo** (server 29.6.1). `drivebase-dev:jazzy-pod`,
  `drivebase-dev:jazzy` and `osrf/ros:jazzy-desktop-full` are all present.
- **`./scripts/dev.sh <command>` works**, and reports
  `windowed on :1, hardware GL via /dev/dri` — **a real GPU, not llvmpipe.**
- `./scripts/dev.sh colcon build --symlink-install` builds **all 8 packages
  clean in 2.5 s**. This is the first verified colcon build on this machine;
  the note below saying it was "not re-verified on Arch" is now stale.

ROS 2 is still not native (`/opt/ros` does not exist, there is no `ros2` or
`colcon` on `PATH`) — it lives in the container, reached through `dev.sh`.

**Consequence: the software-rendering caveats no longer apply here.** Anything
`STATUS.md` or this file defers to "re-check on a real GPU" can now actually be
checked — including known issue #2, the Nav2 controller running at 3.5-11 Hz
against a 20 Hz target, which was always suspected to be a starved controller
rather than a real defect.

`dev-native.sh` is still the fast path for pure-geometry work (no container
start-up), and still covers geometry only. It provisions a Python
3.12 venv at `~/.venvs/drivebase-dev` (numpy, pytest, urdf_parser_py, xacro,
and `ament_index_python` from source — it is not on PyPI) and a minimal ament
index at `~/.cache/drivebase-dev/ament` so xacro can resolve
`$(find drivebase_description)` without a colcon install tree.

```bash
./scripts/dev-native.sh python -m pytest src/drivebase_behaviour/test -v
```

**It covers geometry only** — anything importing `rclpy`, any launch file,
`colcon`, and all of Gazebo still need the container. A
`ModuleNotFoundError` on a ROS package is that boundary, not a bug.

Two things this cost, worth not rediscovering:

- **The repo is on an NTFS/fuseblk mount that drops the executable bit.** A
  venv created *inside* the repo produces console scripts that will not run
  (`Permission denied` on `.venv/bin/xacro`), and new scripts land as `100644`
  — `git update-index --chmod=+x` is how `dev-native.sh` got its mode.
- **Docker and ROS were offered but need a password**, which an agent session
  cannot supply. Install commands are in the handoff notes if the container
  path is wanted back.

### Apple Silicon Mac (Tasks 1–8)

No native ROS 2 or Gazebo either; everything ran through
**`./scripts/dev.sh <command>`**, which mounts the repo at `/ws` in
`drivebase-dev:jazzy-pod` (built once, on top of the pre-existing
`drivebase-dev:jazzy`, adding `ros-jazzy-urdfdom-py`).

Workspace built clean there: `./scripts/dev.sh colcon build --symlink-install`,
6 packages, ~3 s. Since re-verified on Arch: 8 packages, 2.5 s.

### Software rendering — Mac only, not the current machine

Docker on macOS has no GPU passthrough, so Gazebo falls back to llvmpipe.
**Measured: it works** — the camera publishes full 640×480 frames headless.

- **Trust:** geometry, tf, kinematics, frame layout, whether data flows.
- **Do not trust:** anything in Hz, or "how long did it take". Re-check on a
  real GPU. `STATUS.md` known issue #2 may be nothing but a starved controller.

---

## Verified facts worth not rediscovering

**Camera intrinsics are correct end to end.** `camera_info` reports
`fx = fy = 493.7925`, `cx = 320`, `cy = 240` at 640×480 — exactly
`320 / tan(1.15/2)`, computed independently. Task 8's `test_localisation.py`
derives `FX` from that same formula, so the geometry chain will agree.

**The arm is commandable.** `shoulder_pan` −0.00018 → 0.60008 rad on command,
other five joints holding stow. This was the gate for all of Phases 2-4.

**`/tof/pod` reports `radiation_type = 1` (INFRARED).**

### The pod camera imaged the world sideways — fixed 2026-08-01

`arm_wrist_link`'s frame is rolled 90° about its forward axis (its +Z points
along the robot's +Y). A camera mounted square to it produces an image whose
**right is the robot's forward direction** and whose **down is the robot's
left**, at every arm pose — the pod sits before `wrist_roll` deliberately, so
nothing downstream can roll it back.

That inverts the meaning of `horizontal_error`, which the coordinator *and*
`trash_vision` both read as bearing. Sideways it is **range**, and bearing hides
in the vertical coordinate. Measured at confirm_pose before the fix: 1 m of
lateral target motion changed horizontal error by **0.000**, and `shoulder_pan`
— the joint the aim loop nulls that error with — had first-order authority over
it of **exactly zero** (the response is quadratic, peaking at pan 0).

So the earlier "overshooting loop, suspect `aim_gain`" reading was wrong. No
gain would have worked. Fixed by rolling the camera mount +90°, which is a
rotation about the shared view axis and therefore does **not** move the ToF ray
— Task 5b's aim result stands untouched.

`scripts/verify_pod_orientation.py` pins the axes and both sensitivities off the
URDF, no simulator. Run it after touching the pod mount or `arm_wrist_link`.

**Consequence: the aim loop integrates `+=`, not `-=`.** Authority is
−0.431 /rad at confirm_pose and −0.636 at search_pose, so a positive pan step
reduces a positive error. `aim_gain` 1.0 closes 43% per tick; 2.32 is deadbeat.

### Three traps, all silent

1. **`ros2 topic echo --once` lies.** Each CLI call is a fresh node and
   intermittently gives up before discovering a low-rate publisher — it
   reported a *working* bridge as broken. Verify with one long-lived `rclpy`
   subscriber spinning ~25 s instead.
2. **`std_msgs/msg/Double` does not exist.** The gz side is `gz.msgs.Double`,
   so the guess is natural and the resulting topic carries nothing, with no
   error anywhere. Correct ROS type is `std_msgs/Float64`.
3. **Each `scripts/dev.sh` call is a fresh container.** Background processes do
   not survive between calls; launch-then-query must be one invocation.

4. **`/tof/pod` is BEST_EFFORT** (`qos_profile_sensor_data`). A **RELIABLE**
   subscriber gets one warning and then silence. An earlier version of this
   file said the opposite; it cost an agent a whole sweep.
5. **The arm droops up to 0.20 rad** from its commanded pose when extended
   (worst at lift 0.8 / elbow −1.2; the deepest grasp pose converges within
   0.010 rad). Always measure achieved tf — never assume commanded ==
   achieved. **Task 10's IK must not assume the arm reaches what it is told.**
   Whether this is a torque limit or a starved controller under llvmpipe is
   not established.
6. **`/joint_states` uses `arm_`-prefixed names**, not the command-topic names.

### The arm's planar frame — three offsets, all silent

Task 9 reduces the three pitch joints to a planar 3R arm. The reduction is
exact, but only in coordinates anchored the right way. Each of these is
invisible at the zero pose and wrong everywhere else, which is the worst
possible failure shape — the first WIP passed its zero-pose case and failed
the rest.

1. **The pan axis is 38.8 mm off the base origin** (`+x` in `arm_base_link`).
   A radius measured as `hypot(x, y)` from the origin is not conserved when
   the arm pans, so an IK built on it aims at a target that moves when it
   turns.
2. **The pitch chain runs 18.3 mm to one side of the pan plane** (y =
   −0.0183), while the tip returns to y ≈ 0 at the wrist — the gripper is
   deliberately centred. So `hypot` mixes two different planes and *no* set
   of link lengths reconciles them. `r` must be a **signed projection** onto
   the radial direction, which drops the lateral offset legitimately: the
   pitch axes are parallel to it, and rotation about an axis cannot change
   displacement along it.
3. **A positive joint command turns the arm the *negative* way in the plane.**
   The pitch joints turn about `+y`, while the plane's own positive sense
   (r toward z) is a rotation about `−y`. `PlanarArm.sense` is `−1`, derived
   from the axes rather than assumed, and `pan_sense` is `−1` too because the
   pan axis points *down*.

`PlanarArm.to_planar(point, pan)` does all three, `bearing` and `from_planar`
invert it, and `ik.solve` is built on them. Do not hand-roll any of it — and
note `to_planar` takes the pan angle, because `r` is a projection and panning
turns the direction it projects onto.

`from_planar` is only an inverse in one direction: `to_planar` discards the
component perpendicular to the plane. That is not a practical limit — the tip
lies on the plane to within the model's own 1e-6 noise, verified across pans,
because the gripper is centred on the pan axis.

**Tolerances are 1e-5, not 1e-9, and that is the model's fault, not the
code's.** The CAD export writes pi as `3.14159`, which tilts the pan axis
2.65e-6 rad off vertical and moves the tip ~1e-6 m over a pan.

**Cross-checks against measurements already in this file** (all consistent,
all in the same direction): mount height 0.260 m exactly; lowest tip 224.9 mm
below the mount vs **219–220 mm measured in sim**; grasp radius 0.389 m vs
**0.397 m measured**. The model is ideal commanded kinematics and the sim
figures include the **droop of up to 0.20 rad** noted above, so the model
reading a few mm *further* is expected. Task 10's IK inherits this: it solves
where the arm is told to go, never where it ends up.

### Settled: the pod's aim

Investigated at length in Task 5b; **do not reopen without reading
`docs/arm_workspace.md` §3.**

The pod's axis is 87.2° off the gripper's approach direction, and that is
**correct as mounted**. Aiming it at the gripper makes the ToF read a constant
0.058 m — the casting of `arm_gripper_link`, which sits 0.0619 m away along −Y.
The pod is at the *root* of a 160 mm gripper, so any aim at the grasp point
looks down its length. Clearing the castings by 60 mm costs 80 mm of
displacement off the grasp axis; mounting outboard was tried and failed too.

The grasp window *is* viewable — from pan 0, lift 0.9, elbow 0.46,
wrist_flex −1.40. Task 5's claim that no pose could see it was wrong; it never
paired strongly negative `wrist_flex` with positive `shoulder_lift`.

**Consequence for the coordinator:** at the confirm pose the gripper is *not*
over the target (gripper x = 0.572, beam x = 0.392). Seeing the grasp zone and
holding the jaws above it are mutually exclusive on this arm. `CONFIRMING`
ranges and images the target from where it can see; the descent is open-loop
from the stored point, exactly as spec §6 requires.

### Cosmetic, deliberately not fixed

- `tof_to_range` logs "Converting 1 ultrasonic cones" for an infrared sensor.
- `Range.field_of_view` is `0.0` for the pod ToF (single ray, `angle_max ==
  angle_min`). Defensible, but a consumer that renders a cone or divides by it
  gets a degenerate value.
- Gazebo emits `/tof/pod/scan/points` for every `gpu_lidar`. Unused,
  deliberately unbridged.

---

## Still true from the design

`src/trash_vision/` is **out of scope** — another author's package. The typed
message and node changes are a proposal in §9 of the spec, not work to do here.

Figures in `config/coordinator.yaml` marked `# MEASURED` are placeholders until
Task 5 produces them. Do not ship them as real.
