#!/usr/bin/env python3
"""Run the wheel tuner from the repo: python3 tools/tune_wheels.py --help.

Same as `ros2 run trash_vision tune_wheels`. Measuring needs a sourced ROS
environment; `--analyze run.csv` works on any PC.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'trash_vision'))

from trash_vision.tune_wheels import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main())
