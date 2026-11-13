# Design Decisions

The agent records design choices here when a TODO item leaves one open
(for example, which ROS message type to use). Bryan can veto any entry:
edit the line or add "VETOED:" and the next agent run will redo that part.

| Date | Decision | Why | Status |
|------|----------|-----|--------|
| 2026-11-13 | Detections stay as JSON in `std_msgs/String` on `/vision/target` (plus `/vision/direction`), now with a `source` field (`yolo`/`gpt4o`) | Existing consumers already use this; vision_msgs is not in the base CI image. Can migrate to `vision_msgs/Detection2DArray` later | active |
| 2026-11-13 | GPT-4o fallback is off by default (`gpt_fallback_enabled`), runs in a background thread, only when YOLO finds nothing or best conf < `gpt_fallback_low_confidence`, at most every `gpt_fallback_interval_seconds`; key only from `OPENAI_API_KEY` env var | Keeps API cost bounded and never blocks the frame loop; no secret in repo | active |
| 2026-11-13 | GPT is asked for normalized 0..1 boxes as JSON; invalid/out-of-range replies are ignored; a reply is trusted for 2x the interval | GPT bounding boxes are approximate, so reject bad data rather than steer on it | active |
