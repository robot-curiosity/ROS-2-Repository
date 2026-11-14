# Imports:
import json
import math

# ROS:
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Logic:
from trash_vision import sim_logic as sl


# Sim node: stands in for drive, ultrasonic, detector and arm:
class SimNode(Node):
    # Initialization:
    def __init__(self) -> None:
        super().__init__("sim")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("world_width", 4.0)
        self.declare_parameter("world_height", 4.0)
        # Flat list [x1, y1, x2, y2, ...] of litter positions in metres.
        self.declare_parameter("litter", [1.0, 1.0, 3.0, 2.5, 1.5, 3.2])
        flat = list(self.get_parameter("litter").value)
        litter = list(zip(flat[0::2], flat[1::2]))
        w = float(self.get_parameter("world_width").value)
        h = float(self.get_parameter("world_height").value)
        self._world = sl.SimWorld(w, h, litter, x=w / 2, y=h / 2)
        self._v = 0.0
        self._w = 0.0
        self._frame = 0
        self._last = self._now()

        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._target_pub = self.create_publisher(String, "/vision/target", 10)
        self._range_pubs = {
            n: self.create_publisher(Range, "/ultrasonic/" + n, 10)
            for n in sl.SENSOR_BEARINGS}
        self.create_service(Trigger, "arm/pick", self._on_pick)
        self.create_service(Trigger, "arm/deposit", self._on_deposit)
        self.create_service(Trigger, "arm/stow", self._on_stow)
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info("Sim running with %d litter items" % len(litter))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_cmd(self, msg) -> None:
        self._v = msg.linear.x
        self._w = msg.angular.z

    def _on_pick(self, _req, res):
        res.success, res.message = self._world.pick()
        return res

    def _on_deposit(self, _req, res):
        res.success, res.message = self._world.deposit()
        return res

    def _on_stow(self, _req, res):
        res.success, res.message = True, "stowed"
        return res

    # Main loop:
    def _tick(self) -> None:
        now = self._now()
        dt = max(now - self._last, 0.0)
        self._last = now
        world = self._world
        world.step(self._v, self._w, dt)
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = world.x
        odom.pose.pose.position.y = world.y
        odom.pose.pose.orientation.z = math.sin(world.yaw / 2)
        odom.pose.pose.orientation.w = math.cos(world.yaw / 2)
        odom.twist.twist.linear.x = self._v
        odom.twist.twist.angular.z = self._w
        self._odom_pub.publish(odom)

        for name, dist in world.ranges().items():
            rng = Range()
            rng.header.stamp = stamp
            rng.header.frame_id = "ultrasonic_" + name
            rng.radiation_type = Range.ULTRASOUND
            rng.field_of_view = 0.26
            rng.min_range = 0.02
            rng.max_range = sl.SENSOR_MAX_RANGE
            rng.range = dist
            self._range_pubs[name].publish(rng)

        self._frame += 1
        msg = String()
        msg.data = json.dumps(world.target(self._frame))
        self._target_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
