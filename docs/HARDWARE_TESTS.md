# Hardware Test Queue

Things that can only be verified on the real robot (Pi 5, camera, motors, arm).
The agent adds a line when a merged change needs a real test. Bryan runs them in
batches, then marks the result. A change is "promoted" to `master` only after
its tests pass here.

| Added | Change (PR) | What to test on the robot | Result |
|-------|-------------|---------------------------|--------|
