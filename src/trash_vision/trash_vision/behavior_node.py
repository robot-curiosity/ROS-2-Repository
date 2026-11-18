# Imports:
import json
import math
import os
import time

# ROS:
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Logic:
from trash_vision import behavior_logic as bl
from trash_vision import safety_logic as sfl
from trash_vision import sonar_scheduler as ss
from trash_vision import trial_log as tl
from trash_vision import ultrasonic_logic as ul

FRONT = ul.FRONT_NAMES
REAR = ul.REAR_NAMES
LEFT = ul.LEFT_NAMES
RIGHT = ul.RIGHT_NAMES


# Behavior node:
class BehaviorNode(Node):
    # Initialization:
    def __init__(self) -> None:
        super().__init__("behavior")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("enabled", False)
        self.declare_parameter("trial_log_dir", "~/curiosity_trials")
        for key, val in bl.DEFAULTS.items():
            self.declare_parameter(key, val)
        params = {k: self.get_parameter(k).value for k in bl.DEFAULTS}
        self._machine = bl.BehaviorMachine(params)
        for key, val in sfl.DEFAULTS.items():
            self.declare_parameter(key, val)
        self._safety = sfl.SafetyStage(
            {k: self.get_parameter(k).value for k in sfl.DEFAULTS})
        self._last_safety_reason = ""

        self._recorder = self._make_recorder()
        self._target = None
        self._target_stamp = None
        self._pose = None
        self._pose_stamp = None
        self._ranges = {}
        self._range_stamps = {}
        self._stale_after = float(
            self.get_parameter("target_stale_timeout").value)

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "behavior/state", 10)
        self.create_subscription(
            String, "/vision/target", self._on_target, 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        for name in FRONT + REAR + LEFT + RIGHT:
            self.create_subscription(
                Range, "/ultrasonic/" + name,
                lambda msg, n=name: self._on_range(n, msg.range), 10)
        self._clients = {
            a: self.create_client(Trigger, "arm/" + a)
            for a in ("pick", "deposit", "stow")}
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._tick)
        # Watchdog runs faster than the tick so a stalled tick is caught:
        self.create_timer(0.1, self._watchdog)
        if not self.get_parameter("enabled").value:
            self.get_logger().warning(
                "Behavior disabled (parameter 'enabled' false): robot will "
                "not move. Set enabled:=true to start searching.")

    # Pickup attempt log (JSON Lines, one file per run); '' disables it:
    def _make_recorder(self):
        folder = str(self.get_parameter("trial_log_dir").value).strip()
        self._log_writer = None
        if not folder:
            return None
        run_id = time.strftime("run_%Y%m%d_%H%M%S")
        path = os.path.join(os.path.expanduser(folder), run_id + ".jsonl")
        try:
            self._log_writer = tl.JsonlWriter(path)
        except OSError as exc:
            self.get_logger().error("trial log disabled: %s" % exc)
            return None
        self.get_logger().info("logging pickup attempts to %s" % path)
        return tl.TrialRecorder(self._machine, self._log_writer, run_id)

    # Time in seconds:
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # Sonar callback (local receive time, so clock skew cannot hide a stall):
    def _on_range(self, name, value) -> None:
        self._ranges[name] = value
        self._range_stamps[name] = self._now()

    # Odometry callback (pose for the search pattern):
    def _on_odom(self, msg) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pos = msg.pose.pose.position
        self._pose = (pos.x, pos.y, yaw)
        self._pose_stamp = self._now()

    # Target callback:
    def _on_target(self, msg) -> None:
        try:
            self._target = json.loads(msg.data)
        except ValueError:
            self._target = None
        self._target_stamp = self._now()

    # Main loop:
    def _tick(self) -> None:
        if not self.get_parameter("enabled").value:
            return
        now = self._now()
        # Never-heard sensors read 'inf' (not installed); sensors that went
        # silent read 0 (blocked). See sonar_scheduler.apply_staleness.
        stale = float(self.get_parameter("sonar_stale_timeout").value)

        def sonars(names):
            return ss.apply_staleness(
                names, self._ranges, self._range_stamps, now, stale)

        front = sonars(FRONT)
        rear = sonars(REAR)
        target = bl.fresh_target(
            self._target, self._target_stamp, now, self._stale_after)
        left = sonars(LEFT)
        right = sonars(RIGHT)
        # Without fresh odometry (drive node off) SEARCH falls back to spinning.
        pose = self._pose
        if self._pose_stamp is None or now - self._pose_stamp > 1.0:
            pose = None
        cmd = self._machine.step(
            now, target, front, rear, left, right, pose=pose)
        if self._recorder:
            self._recorder.update(now, target or self._target)
        # Safety output stage is the last filter before cmd_vel:
        self._safety.note_update(now)
        sonars_ok = sfl.sonars_healthy(
            FRONT + REAR + LEFT + RIGHT, self._range_stamps, now, stale,
            bool(self.get_parameter("safety_require_sonars").value))
        lin, ang = self._safety.apply(
            now, float(cmd.linear), float(cmd.angular), min(front),
            sonars_ok)
        self._log_safety()
        twist = Twist()
        twist.linear.x = float(lin)
        twist.angular.z = float(ang)
        self._cmd_pub.publish(twist)
        if cmd.arm_action:
            self._call_arm(cmd.arm_action)
        self._state_pub.publish(String(data=self._machine.state))

    # Publish zero if the main tick stopped running (enabled only):
    def _watchdog(self) -> None:
        if not self.get_parameter("enabled").value:
            return
        if self._safety.timed_out(self._now()):
            self._cmd_pub.publish(Twist())
            self._safety.apply(self._now(), 0.0, 0.0, 0.0, False)
            self._log_safety()

    # Log a safety intervention once per change of reason:
    def _log_safety(self) -> None:
        reason = self._safety.reason
        if reason != self._last_safety_reason:
            if reason and reason not in ("clamped", "accel_limited"):
                self.get_logger().warning("safety stop: %s" % reason)
            self._last_safety_reason = reason

    # Call arm service:
    def _call_arm(self, action) -> None:
        client = self._clients[action]
        if not client.service_is_ready():
            self.get_logger().error("arm/%s service not available" % action)
            self._machine.arm_result(False, self._now(), action)
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f, a=action: self._on_arm_done(a, f))

    # Arm result:
    def _on_arm_done(self, action, future) -> None:
        try:
            res = future.result()
            ok = bool(res.success)
            self.get_logger().info("arm/%s: %s" % (action, res.message))
        except Exception as exc:
            self.get_logger().error("arm/%s failed: %s" % (action, exc))
            ok = False
        self._machine.arm_result(ok, self._now(), action)
        if self._recorder:
            self._recorder.update(self._now(), self._target)


# Main:
def main(args=None) -> None:
    rclpy.init(args=args)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd_pub.publish(Twist())
        if node._log_writer:
            node._log_writer.close()
        node.destroy_node()
        rclpy.try_shutdown()
