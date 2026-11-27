# Review summary (items 33-36), 2026-11-24

One page for Bryan. Only OPEN findings are listed: the review passes already fixed
the rest (NaN handling, malformed targets, motor stop on crash, node names, etc.;
see the four `REVIEW-*.md` files). Nothing below was changed in code because each
needs a design choice or a real measurement. Tests are rows in `HARDWARE_TESTS.md`
(added date 2026-11-24 unless noted).

## High

| # | Finding | Source | Confirm or clear with |
|---|---------|--------|-----------------------|
| 1 | Wrong `invert_enc_*` sign gives positive feedback: that wheel runs away at full duty until the command times out | drive | "Review pass 2: encoder sign and ticks at speed". Do this first, wheels off the ground |
| 2 | Sonar detection delay of 1 to 1.4 s per sensor (median-of-3, round robin); 0.4 to 0.7 m of travel at 0.3 to 0.5 m/s before a new obstacle shows | drive | "Review pass 2: sonar detection delay" |
| 3 | A sonar with a broken echo wire or stuck echo reads max range (clear) and looks healthy | drive | New row "Review summary: dead sonar" |
| 4 | A front sonar that never reports reads as "not installed": the robot drives with no front protection and no warning | behavior | New row "Review summary: dead sonar" (same test) |
| 5 | Stream aspect ratio vs `camera_vfov_deg`: if the stream is not about 16:9, `base_x` is wrong and nothing detects it | vision | "Review pass 3: projection at known distances and stream aspect ratio" |

## Medium

| # | Finding | Source | Confirm or clear with |
|---|---------|--------|-----------------------|
| 6 | Stopping is not instant: stop and watchdog only zero the target, the slew limit ramps down, so 0.25 to 0.4 m of travel | drive | New row "Review summary: stopping distance" |
| 7 | MDD10A PWM/DIR pins may float when the node stops or is killed | drive | "Review pass 2: motor pins when the node stops" |
| 8 | Lost encoder ticks at high wheel speed (gpiozero callbacks) | drive | "Review pass 2: encoder sign and ticks at speed" |
| 9 | Camera stream loss: last `/vision/target` stays on the topic, grabber never reopens | vision | "Review pass 3: camera loss" |
| 10 | Arm: first move is a joint-space line from stow that can sweep through the hoop or mast; silent clamp at default +-100 limits; no recovery after an error mid-pick | vision/arm | "Review pass 3: first pick path" |
| 11 | RECOVER backs up for 1.5 s blind if a rear sonar never reports | behavior | New row "Review summary: dead sonar" (rear sensor) |
| 12 | SEARCH `spin` wedges (0,0 forever) when the front and both sides are blocked | behavior | New row "Review summary: boxed-in search" |
| 13 | `mapping_enabled: true` with a private-network upload URL and a `model_path` under `/home/christopherg` as defaults: frames upload by default | vision | Check `config/robot.yaml` before the first camera run; no test needed |
| 14 | Pico bridge reader thread is not restarted after a serial error; `firmware` print on USB CDC may block the watchdog | drive | "Pico wheel loop: watchdog and cable pull" |
| 15 | Floor point is the near edge of the litter (3 to 4 cm short); no max-range limit on the projection | vision | "Review pass 3: projection at known distances..." |

## Low (look at when convenient)

NaN/None handling differs between `front_minimum` and `min_range`; late arm reply can
be accepted by a later PICK; search area smaller than twice the edge margin; `rate_hz`
below 2 Hz; `max_wheel_speed_mps` 0 crashes the drive node (safe); Pico serial write
has no timeout; software PWM jitter at 1 kHz; `/gps/fix` and `/imu/data` have no
consumer; `package.xml` licence/maintainer still TODO.

## What to look at first on the real robot

1. Wheels off the ground: encoder signs (#1), motor pins on kill (#7), then behavior disabled stops the wheels.
2. Sonars: each one by hand with `sonar_test`, then the dead-sonar row (#3, #4, #11) and detection delay (#2).
3. Wheels on the floor: stopping distance (#6), before any autonomous run.
4. Camera: aspect ratio and distance markers (#5, #15), then camera loss (#9).
5. Arm last, wheels off, hand on the power: first pick path and limits (#10).

## Design choices waiting for Bryan

Required-sonar list for the front (#4), stop bypassing the slew limit (#6), sonar
timing options (#2), per-sensor health count (#3), reverse-out in SEARCH (#12), and
stow-on-error for the arm (#10). The agent has not decided any of these.
