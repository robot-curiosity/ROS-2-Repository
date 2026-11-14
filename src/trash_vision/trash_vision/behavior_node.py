# Imports:
import json

# ROS:
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Logic:
from trash_vision import behavior_logic as bl

FRONT = ["front_%d" % i for i in range(1, 5)]
REAR = ["rear_%d" % i for i in range(1, 5)]


# Behavior node:
class BehaviorNode(Node):
    # Initialization:
    def __init__(self) -> None:
        super().__init__("behavior")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("enabled", False)
        for key, val in bl.DEFAULTS.items():
            self.declare_parameter(key, val)
        params = {k: self.get_parameter(k).value for k in bl.DEFAULTS}
        self._machine = bl.BehaviorMachine(params)

        self._target = None
        self._target_stamp = 0.0
        self._ranges = {}
        self._pending = None

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "behavior/state", 10)
        self.create_subscription(
            String, "/vision/target", self._on_target, 10)
        for name in FRONT + REAR:
            self.create_subscription(
                Range, "/ultrasonic/" + name,
                lambda msg, n=name: self._ranges.__setitem__(n, msg.range),
                10)
        self._clients = {
            a: self.create_client(Trigger, "arm/" + a)
            for a in ("pick", "deposit", "stow")}
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._tick)
        if not self.get_parameter("enabled").value:
            self.get_logger().warning(
                "Behavior disabled (parameter 'enabled' false): robot will "
                "not move. Set enabled:=true to start searching.")

    # Time in seconds:
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # Target callback:
    def _on_target(self, msg) -> None:
        try:
            self._target = json.loads(msg.data)
        except ValueError:
            self._target = None

    # Main loop:
    def _tick(self) -> None:
        if not self.get_parameter("enabled").value:
            return
        now = self._now()
        # Missing sensors read as 'inf' (nothing seen); see HARDWARE_TESTS.
        front = [self._ranges.get(n, float("inf")) for n in FRONT]
        rear = [self._ranges.get(n, float("inf")) for n in REAR]
        cmd = self._machine.step(now, self._target, front, rear)
        twist = Twist()
        twist.linear.x = float(cmd.linear)
        twist.angular.z = float(cmd.angular)
        self._cmd_pub.publish(twist)
        if cmd.arm_action:
            self._call_arm(cmd.arm_action)
        self._state_pub.publish(String(data=self._machine.state))

    # Call arm service:
    def _call_arm(self, action) -> None:
        client = self._clients[action]
        if not client.service_is_ready():
            self.get_logger().error("arm/%s service not available" % action)
            self._machine.arm_result(False, self._now())
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
        self._machine.arm_result(ok, self._now())


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
        node.destroy_node()
        rclpy.try_shutdown()
