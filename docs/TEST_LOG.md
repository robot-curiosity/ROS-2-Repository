# Test Log

| Date | Test | Result | Notes |
|------|------|--------|-------|
| 2026-11-12 | Repo cleanup (agent): git status/ls-files check only | n/a | No ROS 2 runtime or hardware tests run |
| 2026-11-13 | pytest src/trash_vision/test/test_detection_logic.py (7 tests); py_compile on detector_node.py; flake8 --select=E9,F | pass | ROS 2 not installed locally; ament tests not run; CI runs full build |
| 2026-11-13 | pytest test_ultrasonic_logic.py + test_detection_logic.py (13 tests); flake8 --select=E9,F | pass | ROS 2/gpiozero not installed locally; node not run; CI runs full build |
| 2026-11-13 | pytest test_drive_logic.py + detection + ultrasonic tests (21 tests); flake8 --select=E9,F; py_compile drive_node.py | pass | ROS 2/gpiozero not installed locally; node not run; CI runs full build |
| 2026-11-13 | pytest test_arm_logic.py + existing logic tests (30 tests); flake8 --select=E9,F; py_compile arm_node.py | pass | ROS 2/LeRobot not installed locally; node not run; CI runs full build |
| 2026-11-13 | pytest test_*_logic.py incl. new test_gps_imu_logic.py (39 tests); flake8 E,F,W on new files | pass | ROS 2/pyserial/smbus2 not installed locally; node not run; CI runs full build |
| 2026-11-14 | pytest test_*_logic.py incl. new test_behavior_logic.py (50 tests); flake8 --select=E9,F; py_compile behavior_node.py | pass | ROS 2 not installed locally; node not run; CI runs full build |
| 2026-11-14 | PYTHONPATH=src/trash_vision pytest test_*_logic.py incl. new test_sim_logic.py; flake8 --select=E9,F; py_compile sim_node.py + launch files | pass | ROS 2 not installed locally; launch files/sim node not run; CI runs full build |
| 2026-11-16 | PYTHONPATH=src/trash_vision pytest test_*_logic.py + new test_ground_projection.py (68 tests); flake8 E,F,W on new/changed logic files; py_compile detector_node.py | pass | ROS 2 not installed locally; detector node not run; CI runs full build |
| 2026-11-16 | PYTHONPATH=src/trash_vision pytest test_behavior_logic.py (20 tests, 9 new wedge/timeout tests); flake8 E,F,W on changed files (only pre-existing W503); py_compile behavior_node.py | pass | ROS 2 not installed locally; node not run; CI runs full build |
| 2026-11-16 | PYTHONPATH=src/trash_vision pytest test_*_logic.py (89 tests incl. 9 new side-clearance tests, sim and layout tests); flake8 --select=E9,F; py_compile changed nodes | pass | ROS 2 not installed locally; nodes not run; CI runs full build |
| 2026-11-16 | PYTHONPATH=src/trash_vision pytest test_*_logic.py + test_ground_projection.py (98 tests incl. 9 new test_config_logic.py); flake8 --select=E9,F; py_compile *_node.py | pass | ROS 2 not installed locally; launch files and node warnings not run; CI runs full build |
| 2026-11-16 | PYTHONPATH=src/trash_vision pytest test_*_logic.py + test_ground_projection.py (117 tests incl. 19 new test_hardware_check_logic.py); flake8 E,F,W on new files | pass | ROS 2 not installed locally; console script run only via pytest on shipped yaml; CI runs full build |
| 2026-11-16 | PYTHONPATH=src/trash_vision pytest test_*_logic.py + test_ground_projection.py (130 tests incl. 13 new test_calibration_logic.py); flake8 E,F,W on new files; py_compile calibrate.py | pass | ROS 2/GPIO not installed locally; calibrate.py not run; CI runs full build |
| 2026-11-17 | docs-only change (BRINGUP.md); commands and topic names cross-checked against setup.py, robot.yaml and gps_imu_node.py by grep; no code changed | pass | guide not run on the robot |
| 2026-11-17 | PYTHONPATH=src/trash_vision pytest test_*_logic.py + test_ground_projection.py + test_sonar_scheduler.py (148 tests incl. 17 new); flake8 on changed files; py_compile ultrasonic_node/behavior_node/calibrate | pass | ROS 2/gpiozero not installed locally; nodes not run; CI runs full build |
