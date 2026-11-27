# Review pass 3: vision and arm (2026-11-24)

Scope: `detector_node.py`, `detection_logic.py`, `detection_filter.py`,
`ground_projection.py`, `frame_grabber.py`, `arm_logic.py`, `arm_kinematics.py`,
`arm_node.py`. Method: read as a skeptical reviewer, designs unchanged. No camera,
model or arm was available; every fix has a pure-Python regression test in
`test_review_vision_arm.py`.

## Checked and fine

- Image convention: normalized u runs 0..1 left to right, v 0..1 top to bottom. In
  `floor_point_to_base`, `right = (u-0.5)*2*tan(hfov/2)` and `below = (v-0.5)*2*tan(vfov/2)`
  are a plain pinhole model; a larger v means more downward, `base_y = cam.y - scale*right`
  (right of the image is negative y, left is positive) is correct.
- Pitch sign: `pitch_rad` positive is DOWN. Ray = axis + `below` * (perpendicular pointing
  down) gives `forward = cos(p) - below*sin(p)`, `down = sin(p) + below*cos(p)`. Correct.
- With the defaults (pitch 35, vfov 41) the top image row is 14.5 deg below the horizon, so
  every row hits the floor, the farthest visible floor is about 2.4 m (camera 0.642 m up).
  Horizon rows return `None` and the target carries `base_x/base_y = None`.
- Detector uses the bottom edge `y2` of the box and the box centre x as the floor point.
- `arm_kinematics.solve` never clamps, checks pan and the 3 pitch joints against the URDF
  limits, tries elbow-down then elbow-up. `plan_pick_poses` rejects NaN (comparisons false).
- `FrameGrabber`/`LatestFrame`: a thread calls the read function and keeps only the newest
  frame; a read exception is counted and retried, not fatal.

## Fixed in this pass

| File | Finding | Severity |
|---|---|---|
| `arm_logic.clamp_pose` | `max(lo, min(hi, nan))` returns `hi`, so a NaN joint value (bad `pose_*`/`ik_joint_offset` parameter, NaN from IK) was silently clamped to the upper limit and the joint driven there. Now raises `ValueError`, and `arm_node._run` clamps every target BEFORE the first move, so a bad pose is refused with `arm error:` and nothing moves. | high |
| `arm_node._on_target` | `float(data['base_x'])` raised on a non-numeric string or other bad value inside the subscription callback, which propagates out of `executor.spin()` and kills the arm node. Now `arm_logic.parse_target_xy` returns None for bad JSON, non-objects, nulls, bools, strings, NaN and inf. | medium |
| `ground_projection.floor_point_to_base` | NaN or inf for u, v or the camera model produced `(nan, nan)` (the horizon test `down <= 1e-9` is False for NaN). That went into `/vision/target` as `base_x: NaN`. Now returns None. | medium |
| `detection_filter._is_usable` | `confidence < min` is False for NaN, so a NaN-confidence box passed the confidence gate. Now rejected. | low |
| `detection_logic.parse_gpt_response` | NaN confidence (`json.loads` accepts `NaN`) survived the `min/max` clamp. Now replaced by the 0.5 default. | low |
| `frame_grabber.timer_period_s` | A NaN `process_rate_hz` gave a NaN timer period (`min/max` order again) and `create_timer` failed. Now falls back to 10 Hz. | low |

## Open for Bryan (not changed)

| File | Finding | Severity |
|---|---|---|
| `detector_node` frame size vs `camera_vfov_deg` | The projection assumes the processed stream has the 66 x 41 deg field of view, i.e. about 16:9 (tan(33)/tan(20.5) = 1.74). If the TCP stream is 4:3, cropped or scaled non-uniformly, `base_x` is wrong and nothing detects it. Check with markers at known distances (HARDWARE_TESTS). | high until measured |
| `detector_node` / `ground_projection` | The floor point is the bottom edge of the box, i.e. the near edge of the litter. The arm aims at it, so `base_x` is short by about the litter radius (3 to 4 cm for a can). A box cut by the bottom image edge (`y2 == height`) gives a floor point that is too near, since the contact point is out of frame. Consider rejecting boxes touching the bottom or adding half a litter length. | medium |
| `ground_projection` | No range limit. 1 px near the top rows (about 2.4 m away) is many centimetres; with a smaller pitch the horizon can sit inside the image and distances blow up there. Consider reporting `base_x/base_y` only inside a max range (about 1 m is plenty for the 0.35 to 0.56 m pick zone). Lens distortion is not modelled. | medium |
| `detector_node.process_frame` | When the camera stops, `take()` returns None and the node publishes NOTHING (only a warning), so the last `/vision/target` stays on the topic. Consumers rely on their own staleness timeouts (behavior stale-detection guard, arm `target_max_age_s` 1.5 s), which exist. `FrameGrabber` never reopens the `VideoCapture` after a dropped TCP stream, and a blocking `read` has no timeout, so a lost stream needs a node restart. | medium |
| `detector_node` | `frame_age` returned by `LatestFrame.take` is not checked, and the camera pose/FOV values are declared in the node as well as in `robot.yaml` (the yaml wins when launched with it). Also `model_path` default is `/home/christopherg/best_ncnn_model` and `mapping_upload_url` is an IP on a private network with `mapping_enabled: true`: frames are uploaded to that address by default. | medium |
| `arm_node._run` | No overall timeout and no recovery. An exception mid-sequence (serial error, LeRobot error) leaves the arm wherever it is, with the gripper possibly open or holding litter, and the next call starts by interpolating straight from there. A blocking `send_action` holds `_lock`, so state publishing stalls too. A stow-on-error needs a working write path, so it is a design change. | medium |
| `arm_node._run` | The first move interpolates joint-by-joint straight from the current pose (usually stow) to `pre_grasp`. A joint-space line can sweep the gripper through the bag hoop, the camera mast or the floor. Check with `max_step` small and the robot powered but wheels off. | medium |
| `arm_logic._to_backend` | With `use_degrees: false` the IK output is in RADIANS, but LeRobot's non-degree mode uses a -100..100 range scale, so a pick would barely move. `ik_enabled` needs `use_degrees: true` (the default); the code does not enforce it (an existing test uses radians). | medium |
| `arm_node` limits | `joint_min/max` default to +-100 for all six joints and the clamp is silent, so it can move an IK pose without any warning (the "never clamps" promise of `plan_pick_poses` ends there). Set real limits when calibrating and consider failing the pick instead of clamping. | medium |
| `arm_node._run` | `grasp_held` reads the gripper position once after the last move; it does not check that the gripper has stopped. A pick that squeezes hard enough to close fully reports "nothing in gripper" (correct), but a gripper still moving at the read can report held. Needs real data (`grasp_min_gap` 5 is a placeholder). | low |
| `arm_node._on_target` | The target is the position when the message was published; the robot (and the can in the image) may have moved up to `target_max_age_s` (1.5 s) since. The behavior node should stand still during pick; verify on the robot. | low |
| `DetectionFilter` | Two tracks can form on one object (two overlapping boxes further apart than `track_match_radius`); both then confirm and the nearer wins, which is harmless. Tracks are matched by centre only. | low |

## Hardware checks

See the rows added to `docs/HARDWARE_TESTS.md` dated 2026-11-24 (projection at known
distances, stream aspect ratio, camera loss, arm NaN refusal, `base_x` bias).
