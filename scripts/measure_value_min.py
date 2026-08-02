#!/usr/bin/env python3
"""Sweeps the litter detector's `value_min` against live pod-camera frames.

    bash scripts/dev.sh bash -c '<launch the stack> ; \
      python3 /ws/scripts/measure_value_min.py --seconds 150'

WHY. `value_min` is the lever that decides whether the field's barriers enter
the detector's mask. The barriers are orange (hue ~13 in OpenCV units) and the
hue bound of 10 excludes their lit faces, but shading rotates orange toward
red: sampled from a live frame, a shaded barrier edge is srgb(82,28,0), which
is hue 10.2, saturation 255, value 82 - inside every bound. Raising `value_min`
should drop it, because the offending pixels are dark precisely BECAUSE they
are shaded, while lit litter is not.

Should. That is the claim this measures, and the risk it measures against is
the obvious one: a threshold high enough to reject a shaded barrier may also
reject genuine litter sitting in shadow. Guessing is how you get a detector
that looks healthier and sees less.

Writes runs/value_min/sweep.csv - one row per frame per threshold, with the
blob each would have reported - and the raw frames beside it, so a
disagreement between two thresholds can be looked at rather than argued about.
"""

import argparse
import csv
import pathlib
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

OUT = pathlib.Path("/ws/runs/value_min")

# The detector's other bounds, held fixed. Only value_min is under test.
HUE_LOW = 170
HUE_HIGH = 10
SATURATION_MIN = 120
MIN_AREA_FRACTION = 0.0004

SWEEP = (60, 80, 100, 120, 140)


def largest_blob(hsv: np.ndarray, value_min: int):
    """The detector's own segmentation, at one threshold.

    Mirrors sim_litter_detector._hue_mask including the wrap at red, because a
    sweep that segments differently from the node measures the wrong thing.
    """
    lower_end = cv2.inRange(
        hsv,
        np.array([0, SATURATION_MIN, value_min]),
        np.array([HUE_HIGH, 255, 255]),
    )
    upper_end = cv2.inRange(
        hsv,
        np.array([HUE_LOW, SATURATION_MIN, value_min]),
        np.array([179, 255, 255]),
    )
    mask = cv2.bitwise_or(lower_end, upper_end)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    height, width = hsv.shape[:2]
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < MIN_AREA_FRACTION * width * height:
        return None

    x, y, w, h = cv2.boundingRect(largest)
    return {
        "area_frac": area / (width * height),
        "center_x": (x + w / 2.0) / width,
        "center_y": (y + h / 2.0) / height,
        "bbox_frac": h / height,
        # Median value of the pixels that passed, which is what separates a
        # shaded barrier edge from lit litter.
        "median_v": float(np.median(
            hsv[y:y + h, x:x + w, 2][mask[y:y + h, x:x + w] > 0]
            if mask[y:y + h, x:x + w].any() else [0])),
    }


class Sweep(Node):
    def __init__(self, period: float) -> None:
        super().__init__("value_min_sweep")
        OUT.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.period = period
        self.last_saved = 0.0
        self.index = 0
        self.rows: list[dict] = []

        self.create_subscription(Image, "/pod_camera/image_raw",
                                 self._on_image, qos_profile_sensor_data)

    def _on_image(self, message: Image) -> None:
        now = time.time()
        if now - self.last_saved < self.period:
            return
        self.last_saved = now

        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imwrite(str(OUT / f"{self.index:03d}.png"), frame)

        for value_min in SWEEP:
            blob = largest_blob(hsv, value_min)
            row = {"frame": self.index, "value_min": value_min,
                   "detected": blob is not None}
            row.update(blob or {})
            self.rows.append(row)
        self.index += 1

    def write(self) -> None:
        if not self.rows:
            print("no frames captured")
            return
        fields = ["frame", "value_min", "detected", "area_frac", "bbox_frac",
                  "center_x", "center_y", "median_v"]
        with (OUT / "sweep.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for row in self.rows:
                writer.writerow({k: row.get(k, "") for k in fields})
        print(f"{self.index} frames -> {OUT / 'sweep.csv'}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=150.0)
    parser.add_argument("--period", type=float, default=1.0)
    arguments = parser.parse_args()

    rclpy.init()
    node = Sweep(arguments.period)
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
