#!/usr/bin/env python3
"""Traces the velocity chain during an approach, end to end.

    bash scripts/dev.sh bash -c '<launch the stack> ; \
      python3 /ws/scripts/trace_approach.py --seconds 200'

WHY. When an approach does not close, the symptom is identical whichever link
of the chain is broken: the target's apparent size stops growing and the state
times out. The coordinator publishes to /cmd_vel_nav, velocity_smoother
forwards to /cmd_vel_smoothed, collision_monitor forwards to /cmd_vel, and the
gz DiffDrive plugin consumes that. A dead node anywhere in the middle drops
messages with no error at either end - a volatile publisher with no subscriber
simply discards, which is this repo's most-repeated failure mode.

So sample all four points at once and write them to one CSV. A row where
cmd_vel_nav is commanding 0.22 m/s and odom is not moving names the broken
link without a second run.
"""

import argparse
import csv
import math
import pathlib
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

OUT = pathlib.Path("/ws/runs/trace")


class Trace(Node):
    def __init__(self, period: float) -> None:
        super().__init__("approach_trace")
        OUT.mkdir(parents=True, exist_ok=True)
        self.period = period
        self.rows: list[dict] = []
        self.start = time.time()
        self.last = 0.0

        self.nav_cmd = (0.0, 0.0)
        self.smoothed = (0.0, 0.0)
        self.out_cmd = (0.0, 0.0)
        self.pose = (float("nan"), float("nan"))
        self.odom_speed = float("nan")
        # Counted rather than sampled: a chain that is dropping messages shows
        # up as a publisher that ticks while the next hop's count stays flat.
        self.counts = {"nav": 0, "smoothed": 0, "out": 0}

        self.create_subscription(Twist, "/cmd_vel_nav",
                                 self._on_nav, 10)
        self.create_subscription(Twist, "/cmd_vel_smoothed",
                                 self._on_smoothed, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_out, 10)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 self._on_odom, qos_profile_sensor_data)
        self.create_timer(period, self._sample)

    def _on_nav(self, m: Twist) -> None:
        self.nav_cmd = (m.linear.x, m.angular.z)
        self.counts["nav"] += 1

    def _on_smoothed(self, m: Twist) -> None:
        self.smoothed = (m.linear.x, m.angular.z)
        self.counts["smoothed"] += 1

    def _on_out(self, m: Twist) -> None:
        self.out_cmd = (m.linear.x, m.angular.z)
        self.counts["out"] += 1

    def _on_odom(self, m: Odometry) -> None:
        p = m.pose.pose.position
        self.pose = (p.x, p.y)
        v = m.twist.twist.linear
        self.odom_speed = math.hypot(v.x, v.y)

    def _sample(self) -> None:
        self.rows.append({
            "t": round(time.time() - self.start, 2),
            "nav_x": round(self.nav_cmd[0], 4),
            "nav_wz": round(self.nav_cmd[1], 4),
            "smoothed_x": round(self.smoothed[0], 4),
            "out_x": round(self.out_cmd[0], 4),
            "odom_x": round(self.pose[0], 4),
            "odom_y": round(self.pose[1], 4),
            "odom_speed": round(self.odom_speed, 4),
            "n_nav": self.counts["nav"],
            "n_smoothed": self.counts["smoothed"],
            "n_out": self.counts["out"],
        })

    def write(self) -> None:
        if not self.rows:
            print("no samples")
            return
        path = OUT / "approach.csv"
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)
        print(f"{len(self.rows)} samples -> {path}")
        print(f"message counts: {self.counts}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=200.0)
    parser.add_argument("--period", type=float, default=1.0)
    arguments = parser.parse_args()

    rclpy.init()
    node = Trace(arguments.period)
    deadline = time.time() + arguments.seconds
    try:
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
