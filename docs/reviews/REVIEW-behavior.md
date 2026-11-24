# Review pass 1: safety and behavior (2026-11-24)

Scope: `safety_logic.py`, `behavior_logic.py`, `behavior_node.py`, `search_pattern.py`.
Method: read as a skeptical reviewer, no designs changed. Line numbers are as of this commit.

## Fixed in this pass (each has a regression test)

| File | Finding | Severity |
|---|---|---|
| `behavior_logic.py` `valid_target` | A `/vision/target` with `horizontal_error` NaN, `confidence` None/string, or `floor_point_normalized` without `y` was accepted or raised KeyError/TypeError in the tick. NaN made APPROACH turn at full `max_turn_speed`. Now every number must be finite, otherwise the target is ignored. Test: `test_review_malformed_targets_are_rejected`, `test_review_nan_target_never_steers_the_robot`. | medium |
| `behavior_logic.py` `step` | A NaN/inf odometry pose reached the search pattern (NaN distance, spinning at 0.8 rad/s with no progress logic). Non-finite pose is now treated as "no pose" (SEARCH spins slowly). Test: `test_review_nan_pose_falls_back_to_spin`. | low |
| `behavior_node.py` `_tick`/`_watchdog` | Setting `enabled:=false` while the robot was moving published nothing, so the last `cmd_vel` stayed live until the drive node's own timeout. Now one zero Twist is sent on the switch to disabled and the safety stage is reset (`SafetyStage.reset`). Test: `test_review_reset_forgets_motion_and_requires_new_update` (node wiring itself needs ROS, see HARDWARE_TESTS). | medium |
| `behavior_node.py` `_on_target` | A JSON target that is not an object (list, number) crashed later in `.get`. Now treated as no target. | low |

## Open for Bryan (not changed, design questions)

| File:line | Finding | Severity |
|---|---|---|
| `sonar_scheduler.apply_staleness` / `safety_logic.sonars_healthy` | A sonar that never reported is "not installed" (reads inf). If a front sensor is miswired and never reports while others work, the robot drives forward with no front protection and no warning. Consider requiring named front sensors once the robot is fully assembled (e.g. a `safety_required_sonars` list). | high once sonars are fitted |
| `safety_logic.apply` | Only the front minimum is checked. Reversing (RECOVER) relies on `behavior_logic._recover_step` reading the rear sonars; a never-reporting rear sensor reads inf, so RECOVER backs up blind for `recover_backup_time` (1.5 s at 0.1 m/s = about 15 cm). | medium |
| `behavior_logic.min_range` | NaN and None readings are skipped (nothing seen). `safety_logic.front_minimum` treats NaN as blocked, so only the front is protected from NaN; rear/left/right are not. The ultrasonic node publishes max range, never NaN, so this needs a bug elsewhere to matter. A test pins the current behavior (`test_behavior_logic.py` line 16). | low |
| `behavior_logic._limit_spin`, `_step` SEARCH with `search_pattern: spin` | If the front is blocked and both sides are closer than `side_clearance`, SEARCH outputs 0,0 forever: the mission wedges (safe, but never recovers). With the lawnmower pattern `note_blocked` skips waypoints after `search_stuck_timeout`, but the robot still cannot move until something changes. A fix needs a design choice (reverse out, or wait and retry). | medium |
| `behavior_logic.arm_result` | A late reply from an earlier PICK is accepted by a later PICK (state matches, request is not identified). Needs a request id in the node. Only possible after an `arm_timeout` followed by another pick within the late reply window. | low |
| `search_pattern.build_waypoints` | If `search_area_length` or `search_area_width` is not larger than `2 * search_edge_margin`, both waypoints of a row coincide and SEARCH stands still, hopping waypoints forever. Check the area config at bring-up (4 m default is fine). | low |
| `behavior_node.py` `rate_hz` | Below 2 Hz the tick period exceeds `safety_behavior_timeout` (0.5 s) and the watchdog stops the robot every cycle. Safe, but looks like a fault. | low |
| `behavior_node._tick` | An exception in `step` (for example from a new unexpected target shape) skips the publish for that tick; the watchdog then zeroes velocity after 0.5 s. Safe, but the arm state can be stuck until the exception stops. | low |

## Checked and fine

- Sign conventions: positive angular is counter-clockwise; `horizontal_error` positive (target right) gives negative angular; avoid turn flips away from the closer side; `_limit_spin` maps CCW to the left sensors and CW to the right.
- Units: ranges m, speeds m/s and rad/s, times s on every parameter in `DEFAULTS` and `safety_logic.DEFAULTS`.
- Stale or silent sonars read 0 (blocked) and trigger `sonars_stale` in the safety stage; stale detections and stale arm results are ignored.
- Every non-SEARCH state has a timeout (approach, arm, recover) covered by existing tests.
