# Test Log

| Date | Test | Result | Notes |
|------|------|--------|-------|
| 2026-11-12 | Repo cleanup (agent): git status/ls-files check only | n/a | No ROS 2 runtime or hardware tests run |
| 2026-11-13 | pytest src/trash_vision/test/test_detection_logic.py (7 tests); py_compile on detector_node.py; flake8 --select=E9,F | pass | ROS 2 not installed locally; ament tests not run; CI runs full build |
| 2026-11-13 | pytest test_ultrasonic_logic.py + test_detection_logic.py (13 tests); flake8 --select=E9,F | pass | ROS 2/gpiozero not installed locally; node not run; CI runs full build |
| 2026-11-13 | pytest test_drive_logic.py + detection + ultrasonic tests (21 tests); flake8 --select=E9,F; py_compile drive_node.py | pass | ROS 2/gpiozero not installed locally; node not run; CI runs full build |
