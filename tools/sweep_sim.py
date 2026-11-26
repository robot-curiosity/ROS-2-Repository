#!/usr/bin/env python3
"""Run the sim parameter sweep from the repo: python3 tools/sweep_sim.py.

Same as `ros2 run trash_vision sweep_sim`, but the current values default to
config/robot.yaml. Needs only the repo (PyYAML), no ROS. Sim-only results.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'trash_vision'))

from trash_vision.sweep_sim import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main(default_config=os.path.join(
        ROOT, 'src', 'trash_vision', 'config', 'robot.yaml')))
