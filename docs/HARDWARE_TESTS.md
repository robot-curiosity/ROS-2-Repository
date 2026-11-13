# Hardware Test Queue

Things that can only be verified on the real robot (Pi 5, camera, motors, arm).
The agent adds a line when a merged change needs a real test. Bryan runs them in
batches, then marks the result. A change is "promoted" to `master` only after
its tests pass here.

| Added | Change (PR) | What to test on the robot | Result |
|-------|-------------|---------------------------|--------|
| 2026-11-13 | [agent] Detection node: GPT-4o fallback + detection_logic refactor | On the Pi: `export OPENAI_API_KEY=...`; `ros2 run trash_vision detector --ros-args -p gpt_fallback_enabled:=true`; `ros2 topic echo /vision/target`. (1) Litter in view: JSON has `"source": "yolo"`, direction correct. (2) Point camera at litter YOLO misses: after a few s a `"source": "gpt4o"` target appears and its bbox is roughly right. (3) Empty floor: `detected:false`. (4) Without the env var: warning logged, node still runs. |  |
| 2026-11-13 | [agent] Ultrasonic node (`ultrasonic`) | TODO: Bryan to fill in BCM pins. On the Pi: `sudo apt install python3-gpiozero python3-lgpio`; `ros2 run trash_vision ultrasonic --ros-args -p trig_pins:="[...8 ints...]" -p echo_pins:="[...8 ints...]"` (order front_1..4, rear_1..4; -1 = unused). Check echo pins go through a 5V->3.3V divider. `ros2 topic echo /ultrasonic/front_1`: hand at 20 cm reads ~0.20 m (tape-measure check at 0.2/0.5/1.0 m, +-2 cm); nothing in front -> `inf`; all 8 topics publish at ~10 Hz (`ros2 topic hz`). Without pins set: warning, node idle. |  |
