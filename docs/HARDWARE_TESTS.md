# Hardware Test Queue

Things that can only be verified on the real robot (Pi 5, camera, motors, arm).
The agent adds a line when a merged change needs a real test. Bryan runs them in
batches, then marks the result. A change is "promoted" to `master` only after
its tests pass here.

| Added | Change (PR) | What to test on the robot | Result |
|-------|-------------|---------------------------|--------|
| 2026-11-13 | [agent] Detection node: GPT-4o fallback + detection_logic refactor | On the Pi: `export OPENAI_API_KEY=...`; `ros2 run trash_vision detector --ros-args -p gpt_fallback_enabled:=true`; `ros2 topic echo /vision/target`. (1) Litter in view: JSON has `"source": "yolo"`, direction correct. (2) Point camera at litter YOLO misses: after a few s a `"source": "gpt4o"` target appears and its bbox is roughly right. (3) Empty floor: `detected:false`. (4) Without the env var: warning logged, node still runs. |  |
