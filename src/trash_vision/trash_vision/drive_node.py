# Imports:
import threading

# ROS:
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Logic:
from trash_vision import drive_logic as dl


# Drive node:
class DriveNode(Node):
    # Initialization:
    def __init__(self) -> None:
        super().__init__("drive")

        # TODO: Bryan to fill in the real BCM pins (-1 = unused). Order: left, right.
        self.declare_parameter("pwm_pins", [-1, -1])
        self.declare_parameter("dir_pins", [-1, -1])
        self.declare_parameter("enc_a_pins", [-1, -1])
        self.declare_parameter("enc_b_pins", [-1, -1])
        # TODO: Bryan to fill in (measure / read from motor datasheet).
        self.declare_parameter("ticks_per_rev", 0.0)
        self.declare_parameter("wheel_radius_m", 0.0)
        self.declare_parameter("track_width_m", 0.0)
        # Tuning defaults (not hardware facts):
        self.declare_parameter("max_wheel_speed_mps", 0.5)
        self.declare_parameter("max_accel_mps2", 1.0)
        self.declare_parameter("min_duty", 0.0)
        self.declare_parameter("kp", 0.5)
        self.declare_parameter("ki", 1.0)
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)
        self.declare_parameter("invert_enc_left", False)
        self.declare_parameter("invert_enc_right", False)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        p = self.get_parameter
        self._ready = (
            p("ticks_per_rev").value > 0.0
            and p("wheel_radius_m").value > 0.0
            and p("track_width_m").value > 0.0
            and all(x >= 0 for x in (list(p("pwm_pins").value)
                                     + list(p("dir_pins").value)
                                     + list(p("enc_a_pins").value)
                                     + list(p("enc_b_pins").value))))
        self._lock = threading.Lock()
        self._target = (0.0, 0.0)
        self._cmd_time = self.get_clock().now()
        self._cmd_applied = [0.0, 0.0]
        self._dec = [dl.QuadratureDecoder(), dl.QuadratureDecoder()]
        self._pwm = [None, None]
        self._dir = [None, None]
        self._enc = []
        self._last_counts = [0, 0]
        self._pi = [dl.PIController(p("kp").value, p("ki").value)
                    for _ in range(2)]

        self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)

        if not self._ready:
            self.get_logger().warning(
                "Drive not configured (pins, ticks_per_rev, wheel_radius_m, "
                "track_width_m); motors disabled. TODO: Bryan to fill in.")
            return

        self._odom = dl.DiffDriveOdometry(
            p("ticks_per_rev").value, p("wheel_radius_m").value,
            p("track_width_m").value)
        if not self._open_hardware():
            self._ready = False
            return
        self._last_time = self.get_clock().now()
        rate = float(p("update_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

    # Open GPIO:
    def _open_hardware(self) -> bool:
        p = self.get_parameter
        try:
            from gpiozero import DigitalInputDevice, DigitalOutputDevice
            from gpiozero import PWMOutputDevice
            for i in range(2):
                self._pwm[i] = PWMOutputDevice(
                    p("pwm_pins").value[i], frequency=1000)
                self._dir[i] = DigitalOutputDevice(p("dir_pins").value[i])
                a = DigitalInputDevice(p("enc_a_pins").value[i], pull_up=True)
                b = DigitalInputDevice(p("enc_b_pins").value[i], pull_up=True)
                self._dec[i] = dl.QuadratureDecoder(a.value, b.value)
                cb = self._make_encoder_cb(i, a, b)
                a.when_activated = a.when_deactivated = cb
                b.when_activated = b.when_deactivated = cb
                self._enc.extend([a, b])
            return True
        except Exception as exc:
            self.get_logger().error("Cannot open drive GPIO: %s" % exc)
            return False

    # Encoder callback factory:
    def _make_encoder_cb(self, idx, a, b):
        def cb():
            with self._lock:
                self._dec[idx].update(a.value, b.value)
        return cb

    # cmd_vel:
    def _on_cmd(self, msg: Twist) -> None:
        left, right = dl.twist_to_wheels(
            msg.linear.x, msg.angular.z,
            self.get_parameter("track_width_m").value)
        left, right = dl.limit_wheel_speeds(
            left, right, self.get_parameter("max_wheel_speed_mps").value)
        with self._lock:
            self._target = (left, right)
            self._cmd_time = self.get_clock().now()

    # Control loop:
    def _tick(self) -> None:
        p = self.get_parameter
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0:
            return

        with self._lock:
            counts = [self._dec[0].count, self._dec[1].count]
            target = self._target
            age = (now - self._cmd_time).nanoseconds * 1e-9
        d = [counts[i] - self._last_counts[i] for i in range(2)]
        self._last_counts = counts
        sign_l = -1 if p("invert_enc_left").value else 1
        sign_r = -1 if p("invert_enc_right").value else 1
        d = [d[0] * sign_l, d[1] * sign_r]
        self._odom.update(d[0], d[1], dt)
        speeds = self._odom.wheel_speeds(d[0], d[1], dt)

        # Command timeout -> stop:
        if age > p("cmd_timeout_s").value:
            target = (0.0, 0.0)

        max_speed = p("max_wheel_speed_mps").value
        inv = [p("invert_left").value, p("invert_right").value]
        for i in range(2):
            self._cmd_applied[i] = dl.slew(
                self._cmd_applied[i], target[i],
                p("max_accel_mps2").value, dt)
            if self._cmd_applied[i] == 0.0:
                self._pi[i].reset()
                out = 0.0
            else:
                ff = self._cmd_applied[i] / max_speed
                out = self._pi[i].update(
                    self._cmd_applied[i], speeds[i], dt, ff)
            duty, fwd = dl.command_to_pwm(out, p("min_duty").value)
            self._pwm[i].value = duty
            self._dir[i].value = (fwd != inv[i])
        self._publish_odom(now)

    # Odometry:
    def _publish_odom(self, now) -> None:
        o = self._odom
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.get_parameter("odom_frame").value
        msg.child_frame_id = self.get_parameter("base_frame").value
        msg.pose.pose.position.x = o.x
        msg.pose.pose.position.y = o.y
        qz, qw = dl.yaw_to_quaternion(o.theta)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = o.v
        msg.twist.twist.angular.z = o.w
        self._odom_pub.publish(msg)

    # Stop motors on shutdown:
    def stop_motors(self) -> None:
        for dev in self._pwm:
            if dev is not None:
                try:
                    dev.value = 0.0
                except Exception:
                    pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
