"""Calibration console scripts. Each prints lines to paste into robot.yaml.

  ros2 run trash_vision calibrate_encoders [--turns 10] [--side left|right]
  ros2 run trash_vision calibrate_drive    [--speed 0.2] [--seconds 3]
  ros2 run trash_vision calibrate_pick_zone
  ros2 run trash_vision calibrate_camera --distance 1.0
  ros2 run trash_vision sonar_test

All maths is in calibration_logic.py (tested in CI). These wrappers only
talk to GPIO or ROS topics, so they need the real robot to run.
"""
import argparse
import json
import math
import os
import sys
import time

import yaml

from trash_vision import calibration_logic as cl
from trash_vision import ground_projection as gp
from trash_vision import ultrasonic_logic as ul


def _config(path=None):
    if path is None:
        try:
            from ament_index_python.packages import (
                get_package_share_directory)
            path = os.path.join(
                get_package_share_directory("trash_vision"), "config",
                "robot.yaml")
        except Exception:
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "config", "robot.yaml")
    with open(path) as f:
        return yaml.safe_load(f)


def _params(config, node):
    return (config.get(node) or {}).get("ros__parameters") or {}


def _ros_node(name):
    import rclpy
    rclpy.init()
    return rclpy.create_node(name)


def _spin_for(node, seconds):
    import rclpy
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


# ------------------------------------------------------------ encoders

def encoders(argv=None):
    p = argparse.ArgumentParser(description="Count encoder ticks while you "
                                "turn one wheel by hand.")
    p.add_argument("--turns", type=float, default=10.0)
    p.add_argument("--side", choices=["left", "right"], default="left")
    p.add_argument("--config", default=None)
    a = p.parse_args(argv)
    from trash_vision.drive_logic import QuadratureDecoder
    from gpiozero import DigitalInputDevice  # lazy: Pi only
    drive = _params(_config(a.config), "drive")
    idx = 0 if a.side == "left" else 1
    try:
        pin_a = int(drive["enc_a_pins"][idx])
        pin_b = int(drive["enc_b_pins"][idx])
    except (KeyError, IndexError, TypeError):
        print("Set drive.enc_a_pins and drive.enc_b_pins in robot.yaml.")
        return 2
    if pin_a < 0 or pin_b < 0:
        print("Encoder pins for the %s wheel are unset (-1)." % a.side)
        return 2
    print("Stop the drive node first (it owns these pins).")
    dec = QuadratureDecoder()
    state = {"ticks": 0}
    dev_a = DigitalInputDevice(pin_a, pull_up=True)
    dev_b = DigitalInputDevice(pin_b, pull_up=True)

    def on_edge():
        state["ticks"] += dec.update(int(dev_a.value), int(dev_b.value))

    dev_a.when_activated = dev_a.when_deactivated = on_edge
    dev_b.when_activated = dev_b.when_deactivated = on_edge
    input("Mark the %s wheel. Turn it exactly %g turns in the forward "
          "direction by hand, then press Enter. Press Enter to start. "
          % (a.side, a.turns))
    state["ticks"] = 0
    input("Counting... press Enter when done. ")
    total = state["ticks"]
    try:
        tpr = cl.ticks_per_rev(total, a.turns)
    except ValueError as exc:
        print("FAIL: %s" % exc)
        return 1
    print("counted %d ticks in %g turns" % (total, a.turns))
    if cl.spin_direction(total) < 0:
        print("Counts went DOWN for forward: set invert_enc_%s: true" % a.side)
    print("Paste into drive: " + cl.yaml_line("ticks_per_rev", round(tpr, 1)))
    return 0


# --------------------------------------------------------------- drive

def drive(argv=None):
    p = argparse.ArgumentParser(description="Drive a known command and "
                                "compare odometry with the tape measure.")
    p.add_argument("--speed", type=float, default=0.2, help="m/s")
    p.add_argument("--turn-speed", type=float, default=0.5, help="rad/s")
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--config", default=None)
    a = p.parse_args(argv)
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    drive_p = _params(_config(a.config), "drive")
    radius = float(drive_p.get("wheel_radius_m", 0.0))
    track = float(drive_p.get("track_width_m", 0.0))
    if radius <= 0 or track <= 0:
        print("Set drive.wheel_radius_m and drive.track_width_m first.")
        return 2
    node = _ros_node("calibrate_drive")
    pub = node.create_publisher(Twist, "cmd_vel", 10)
    last = {}

    def on_odom(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        last["pose"] = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                        yaw)

    node.create_subscription(Odometry, "odom", on_odom, 10)
    _spin_for(node, 1.0)
    if "pose" not in last:
        print("No /odom: is the drive node running? Wheels OFF the ground "
              "is fine for this check but then measured values are not real.")
        return 1

    def run(linear, angular):
        yaws = [last["pose"][2]]
        start = last["pose"]
        end = time.time() + a.seconds
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        while time.time() < end:
            pub.publish(msg)
            _spin_for(node, 0.05)
            yaws.append(last["pose"][2])
        pub.publish(Twist())
        _spin_for(node, 1.0)
        yaws.append(last["pose"][2])
        return start, last["pose"], cl.unwrap_yaw(yaws)

    input("Clear 1 m ahead, mark the start position, press Enter to drive "
          "%g m/s for %g s. Hand on power. " % (a.speed, a.seconds))
    s, e, _ = run(a.speed, 0.0)
    d_odom = cl.odom_distance(s[0], s[1], e[0], e[1])
    d_meas = float(input("Odometry says %.3f m. Measured distance in metres: "
                         % d_odom))
    try:
        new_r = cl.wheel_radius_correction(radius, d_odom, d_meas)
    except ValueError as exc:
        print("FAIL: %s" % exc)
        return 1
    print("Paste into drive: " + cl.yaml_line("wheel_radius_m", new_r))
    input("Now mark the robot heading and press Enter to spin in place at "
          "%g rad/s for %g s. " % (a.turn_speed, a.seconds))
    _, _, yaw_odom = run(0.0, a.turn_speed)
    deg = float(input("Odometry says %.1f deg. Measured turn in degrees "
                      "(count full turns, CCW positive): "
                      % math.degrees(yaw_odom)))
    try:
        new_t = cl.track_width_correction(track, yaw_odom, math.radians(deg))
    except ValueError as exc:
        print("FAIL: %s" % exc)
        return 1
    print("Paste into drive: " + cl.yaml_line("track_width_m", new_t))
    print("Note: fix wheel_radius first, then re-run to check the track.")
    return 0


# ----------------------------------------------------- detector samples

def _collect_targets(seconds, config_path):
    """Return the /vision/target JSON dicts seen with detected true."""
    from std_msgs.msg import String
    node = _ros_node("calibrate_detector")
    seen = []

    def on_msg(msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if d.get("detected"):
            seen.append(d)

    node.create_subscription(String, "/vision/target", on_msg, 10)
    _spin_for(node, seconds)
    return seen


def pick_zone(argv=None):
    p = argparse.ArgumentParser(description="Place a can where the arm "
                                "should pick it, then run this.")
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--config", default=None)
    a = p.parse_args(argv)
    input("Put ONE can at the exact pick position, detector running, "
          "press Enter. ")
    seen = _collect_targets(a.seconds, a.config)
    pts = []
    for d in seen:
        fp = d.get("floor_point_normalized") or {}
        if "x" in fp and "y" in fp:
            pts.append((fp["x"], fp["y"]))
    try:
        out = cl.pick_zone(pts)
    except ValueError as exc:
        print("FAIL: %s (is the can visible to the detector?)" % exc)
        return 1
    print("%(n)d samples, y spread %(y_spread)s, x median %(x_median)s" % out)
    print("Paste into behavior:")
    print("  " + cl.yaml_line("pick_floor_y", out["pick_floor_y"]))
    print("  " + cl.yaml_line("pick_center_tolerance",
                              out["pick_center_tolerance"]))
    return 0


def camera(argv=None):
    p = argparse.ArgumentParser(description="Marker on the floor on the "
                                "robot centreline at a known distance.")
    p.add_argument("--distance", type=float, required=True,
                   help="metres from base_link (not from the camera)")
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--config", default=None)
    a = p.parse_args(argv)
    det = _params(_config(a.config), "detector")
    cam = gp.camera_from_degrees(
        det.get("camera_x_m", 0.248), det.get("camera_y_m", -0.1575),
        det.get("camera_height_m", 0.642), det.get("camera_pitch_deg", 35.0),
        det.get("camera_hfov_deg", 66.0), det.get("camera_vfov_deg", 41.0))
    input("Put a can/marker %.2f m ahead of base_link on the centreline, "
          "press Enter. " % a.distance)
    seen = _collect_targets(a.seconds, a.config)
    ys = [d["floor_point_normalized"]["y"] for d in seen
          if "floor_point_normalized" in d]
    xs = [d["floor_point_normalized"]["x"] for d in seen
          if "floor_point_normalized" in d]
    if len(ys) < 5:
        print("FAIL: only %d detections" % len(ys))
        return 1
    ys.sort()
    xs.sort()
    v, u = ys[len(ys) // 2], xs[len(xs) // 2]
    solved = cl.camera_pitch_from_marker(u, v, a.distance, cam)
    if solved is None:
        print("FAIL: no pitch fits; check camera_height_m and the distance.")
        return 1
    err = cl.pitch_error(cam_pitch(det), solved)
    print("configured pitch %.1f deg, fitted %.1f deg (error %+.1f)"
          % (cam_pitch(det), solved, err))
    print("Paste into detector: " + cl.yaml_line("camera_pitch_deg", solved))
    print("Repeat at a second distance; the two answers should agree "
          "within 1-2 deg, otherwise camera_height_m or FOV is off.")
    return 0


def cam_pitch(det):
    return float(det.get("camera_pitch_deg", 35.0))


# --------------------------------------------------------------- sonar

def _collect_sonar(node, series, seconds):
    _spin_for(node, seconds)
    return {k: list(v) for k, v in series.items()}


def sonar(argv=None):
    p = argparse.ArgumentParser(description="Live sonar check.")
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--no-hand-test", action="store_true")
    a = p.parse_args(argv)
    from sensor_msgs.msg import Range
    node = _ros_node("sonar_test")
    series = {n: [] for n in ul.SENSOR_NAMES}
    for n in ul.SENSOR_NAMES:
        node.create_subscription(
            Range, "/ultrasonic/" + n,
            lambda m, n=n: series[n].append(
                float("inf") if m.range >= m.max_range else m.range), 10)
    print("Sampling %g s with nothing near the robot..." % a.seconds)
    _spin_for(node, a.seconds)
    status = cl.analyze_sonar(series)
    bad = 0
    for n in ul.SENSOR_NAMES:
        s, note = status[n]
        last = [v for v in series[n] if v is not None]
        print("[%s] %-8s %s %s" % (
            "PASS" if s == "OK" else "FAIL", n,
            "last %.3f m" % last[-1] if last else "", note))
        bad += s != "OK"
    if not a.no_hand_test:
        asked = {}
        for n in ul.SENSOR_NAMES:
            input("Hold your hand ~10 cm in front of %s, press Enter. " % n)
            for k in series:
                del series[k][:]
            _spin_for(node, 1.5)
            hit = cl.hand_test_reading(series)
            asked[n] = hit[0] if hit and hit[1] < 0.3 else None
        problems = cl.find_swaps(asked)
        for pr in problems:
            print("[FAIL] " + pr)
        bad += len(problems)
        if not problems:
            print("[PASS] every sensor reacted where its name says")
    return 1 if bad else 0


def _entry(func):
    def run():
        sys.exit(func())
    return run


main_encoders = _entry(encoders)
main_drive = _entry(drive)
main_pick_zone = _entry(pick_zone)
main_camera = _entry(camera)
main_sonar = _entry(sonar)
