"""Pico bridge node: same interface as the drive node, wheels run on a Pico.

/cmd_vel in, /odom out. The wheel speed loop runs in firmware/pico_wheels on
a Raspberry Pi Pico connected over USB serial. Option, not the default: start
it with `wheel_backend:=pico` (see launch files). Do not run it together with
the `drive` node.
"""

# Imports:
import queue
import threading
import time

# ROS:
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

# Logic:
from trash_vision import config_logic as cl
from trash_vision import pico_bridge_logic as pbl


# Pico bridge node:
class PicoBridgeNode(Node):
    """Forward cmd_vel to the Pico and publish odometry from its ticks."""

    # Initialization:
    def __init__(self) -> None:
        """Declare parameters and open the serial port."""
        super().__init__('pico_bridge')

        # TODO: Bryan to fill in (serial port of the Pico, e.g. /dev/ttyACM1).
        self.declare_parameter('serial_port', '')
        self.declare_parameter('serial_baud', 115200)
        # TODO: Bryan to fill in the Pico GPIO numbers, in this order:
        # pwm_left, pwm_right, dir_left, dir_right,
        # enc_a_left, enc_b_left, enc_a_right, enc_b_right.
        self.declare_parameter('pico_pins', [-1, -1, -1, -1, -1, -1, -1, -1])
        # Same meaning as in the drive node:
        self.declare_parameter('ticks_per_rev', 0.0)
        self.declare_parameter('wheel_radius_m', 0.0)
        self.declare_parameter('track_width_m', 0.0)
        self.declare_parameter('max_wheel_speed_mps', 0.5)
        self.declare_parameter('max_accel_mps2', 1.0)
        self.declare_parameter('min_duty', 0.0)
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 1.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('invert_enc_left', False)
        self.declare_parameter('invert_enc_right', False)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # Pico bridge only:
        self.declare_parameter('pico_cmd_timeout_s', 0.5)
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('link_timeout_s', 1.0)

        p = self.get_parameter
        self._params = {n: p(n).value for n in (
            'serial_port', 'pico_pins', 'ticks_per_rev', 'wheel_radius_m',
            'track_width_m', 'max_wheel_speed_mps', 'max_accel_mps2',
            'min_duty', 'kp', 'ki', 'invert_left', 'invert_right',
            'invert_enc_left', 'invert_enc_right', 'pico_cmd_timeout_s')}
        self._ready = False
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0)
        self._cmd_time = time.monotonic()
        self._last_rx = None
        self._link_warned = False
        self._rx = queue.Queue()
        self._serial = None

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)

        missing = cl.unset_message('pico_bridge', self._params)
        if missing:
            self.get_logger().warning(missing)
            return
        if not self._open_serial():
            return
        self._session = pbl.BridgeSession(self._write_line, self._params)
        self._session.start()
        self._ready = True
        threading.Thread(target=self._reader, daemon=True).start()
        rate = max(float(p('send_rate_hz').value), 1.0)
        self.create_timer(1.0 / rate, self._tick)

    # Serial port:
    def _open_serial(self) -> bool:
        try:
            import serial
            self._serial = serial.Serial(
                self._params['serial_port'],
                int(self.get_parameter('serial_baud').value), timeout=0.2)
            return True
        except Exception as exc:
            self.get_logger().error('Cannot open Pico serial port: %s' % exc)
            return False

    def _write_line(self, line: str) -> None:
        try:
            with self._lock:
                self._serial.write((line + '\n').encode('ascii'))
        except Exception as exc:
            self.get_logger().error('Pico write failed: %s' % exc)

    # Reader thread: only moves lines into the queue.
    def _reader(self) -> None:
        while rclpy.ok():
            try:
                raw = self._serial.readline()
            except Exception as exc:
                self.get_logger().error('Pico read failed: %s' % exc)
                return
            if raw:
                self._rx.put(raw.decode('ascii', errors='ignore'))

    # cmd_vel:
    def _on_cmd(self, msg: Twist) -> None:
        with self._lock:
            self._cmd = (msg.linear.x, msg.angular.z)
            self._cmd_time = time.monotonic()

    # Periodic step:
    def _tick(self) -> None:
        now = time.monotonic()
        changed = False
        while True:
            try:
                line = self._rx.get_nowait()
            except queue.Empty:
                break
            self._last_rx = now
            self._link_warned = False
            changed = self._session.on_line(line) or changed
        for event in self._session.events:
            self.get_logger().warning(event)
        self._session.events.clear()

        with self._lock:
            linear, angular = self._cmd
            age = now - self._cmd_time
        if age > float(self.get_parameter('cmd_timeout_s').value):
            linear, angular = 0.0, 0.0
        if pbl.link_lost(now, self._last_rx,
                         float(self.get_parameter('link_timeout_s').value)):
            if not self._link_warned:
                self.get_logger().warning(
                    'No data from the Pico: check the cable and firmware.')
                self._link_warned = True
        self._session.send_velocity(linear, angular)
        if changed:
            self._publish_odom()

    # Odometry:
    def _publish_odom(self) -> None:
        x, y, theta, v, w = self._session.pose()
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('odom_frame').value
        msg.child_frame_id = self.get_parameter('base_frame').value
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qz, qw = pbl.yaw_quaternion(theta)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        self._odom_pub.publish(msg)

    # Stop the wheels on shutdown:
    def stop_motors(self) -> None:
        """Tell the Pico to stop."""
        if self._ready:
            self._session.stop()


def main(args=None) -> None:
    """Run the Pico bridge node."""
    rclpy.init(args=args)
    node = PicoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
