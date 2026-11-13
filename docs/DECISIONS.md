# Design Decisions

The agent records design choices here when a TODO item leaves one open
(for example, which ROS message type to use). Bryan can veto any entry:
edit the line or add "VETOED:" and the next agent run will redo that part.

| Date | Decision | Why | Status |
|------|----------|-----|--------|
| 2026-11-13 | Detections stay as JSON in `std_msgs/String` on `/vision/target` (plus `/vision/direction`), now with a `source` field (`yolo`/`gpt4o`) | Existing consumers already use this; vision_msgs is not in the base CI image. Can migrate to `vision_msgs/Detection2DArray` later | active |
| 2026-11-13 | GPT-4o fallback is off by default (`gpt_fallback_enabled`), runs in a background thread, only when YOLO finds nothing or best conf < `gpt_fallback_low_confidence`, at most every `gpt_fallback_interval_seconds`; key only from `OPENAI_API_KEY` env var | Keeps API cost bounded and never blocks the frame loop; no secret in repo | active |
| 2026-11-13 | GPT is asked for normalized 0..1 boxes as JSON; invalid/out-of-range replies are ignored; a reply is trusted for 2x the interval | GPT bounding boxes are approximate, so reject bad data rather than steer on it | active |
| 2026-11-13 | Ultrasonics: one `sensor_msgs/Range` topic per sensor, `/ultrasonic/{front,rear}_{1..4}`, frame `ultrasonic_<name>`; +inf = nothing seen, -inf = too close; 3-sample median filter; reads via `gpiozero.DistanceSensor` (lgpio backend on Pi 5), imported lazily | Standard message type; gpiozero works on Pi 5 with no extra service; per-sensor topics keep nav/behavior code simple. Lives in `trash_vision` package for now so existing CI covers it | active |
| 2026-11-13 | GPIO pins are parameters `trig_pins`/`echo_pins` (8 entries each, order front_1..4, rear_1..4), default -1 = unused; node idles with a warning until set | Pins are unknown hardware facts; TODO: Bryan to fill in | active |
