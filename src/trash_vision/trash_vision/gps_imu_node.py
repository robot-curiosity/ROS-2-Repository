# Imports:
# ROS:
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

# Logic:
from trash_vision import gps_imu_logic as gl


# GPS/IMU node:
class GpsImu(Node):
    # Initialization:
    def __init__(self) -> None:
        super().__init__("gps_imu")

        # TODO: Bryan to fill in the real GPS serial port and IMU model/bus.
        self.declare_parameter("gps_port", "")
        self.declare_parameter("gps_baud", 9600)
        self.declare_parameter("gps_frame", "gps_link")
        self.declare_parameter("imu_model", "")  # "" = off, "bno055"
        self.declare_parameter("imu_i2c_bus", 1)
        self.declare_parameter("imu_i2c_addr", gl.BNO055_ADDR)
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("imu_rate_hz", 50.0)

        self._gps_pub = self.create_publisher(NavSatFix, "gps/fix", 10)
        self._imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self._serial = None
        self._bus = None

        port = self.get_parameter("gps_port").value
        if port:
            self._open_gps(port)
        else:
            self.get_logger().warning(
                "gps_port not set; GPS idle. TODO: Bryan to fill in.")
        model = self.get_parameter("imu_model").value
        if model == "bno055":
            self._open_imu()
        elif model:
            self.get_logger().error("Unsupported imu_model: " + model)
        else:
            self.get_logger().warning(
                "imu_model not set; IMU idle. TODO: Bryan to fill in.")

        if self._serial is not None:
            self.create_timer(0.1, self._gps_tick)
        if self._bus is not None:
            rate = float(self.get_parameter("imu_rate_hz").value)
            self.create_timer(1.0 / max(rate, 1.0), self._imu_tick)

    # Open GPS serial port:
    def _open_gps(self, port) -> None:
        try:
            import serial
            self._serial = serial.Serial(
                port, int(self.get_parameter("gps_baud").value), timeout=0)
            self._buffer = b""
        except Exception as exc:
            self.get_logger().error("Cannot open GPS %s: %s" % (port, exc))

    # Open IMU on I2C:
    def _open_imu(self) -> None:
        try:
            import time
            from smbus2 import SMBus
            self._addr = int(self.get_parameter("imu_i2c_addr").value)
            bus = SMBus(int(self.get_parameter("imu_i2c_bus").value))
            bus.write_byte_data(
                self._addr, gl.BNO055_REG_OPR_MODE, gl.BNO055_MODE_CONFIG)
            time.sleep(0.03)
            bus.write_byte_data(
                self._addr, gl.BNO055_REG_OPR_MODE, gl.BNO055_MODE_NDOF)
            time.sleep(0.02)
            self._bus = bus
        except Exception as exc:
            self.get_logger().error("Cannot open IMU: %s" % exc)

    # Read GPS lines:
    def _gps_tick(self) -> None:
        try:
            self._buffer += self._serial.read(512)
        except Exception as exc:
            self.get_logger().error("GPS read failed: %s" % exc)
            return
        self._buffer = self._buffer[-2048:]
        *lines, self._buffer = self._buffer.split(b"\n")
        for raw in lines:
            fix = gl.parse_gga(raw.decode("ascii", errors="ignore"))
            if fix is not None:
                self._publish_fix(fix)

    def _publish_fix(self, fix) -> None:
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("gps_frame").value
        msg.status.status = gl.fix_status(fix["quality"])
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = fix["lat"]
        msg.longitude = fix["lon"]
        msg.altitude = fix["alt"]
        msg.position_covariance = gl.position_covariance(fix["hdop"])
        msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_APPROXIMATED)
        self._gps_pub.publish(msg)

    # Read and publish IMU:
    def _imu_tick(self) -> None:
        try:
            quat = gl.bno055_decode_quaternion(self._bus.read_i2c_block_data(
                self._addr, gl.BNO055_REG_QUAT, 8))
            gyr = gl.bno055_decode_vec3(self._bus.read_i2c_block_data(
                self._addr, gl.BNO055_REG_GYR, 6), gl.BNO055_GYR_SCALE)
            acc = gl.bno055_decode_vec3(self._bus.read_i2c_block_data(
                self._addr, gl.BNO055_REG_ACC, 6), gl.BNO055_ACC_SCALE)
        except Exception as exc:
            self.get_logger().error(
                "IMU read failed: %s" % exc, throttle_duration_sec=5.0)
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("imu_frame").value
        if quat is None:
            msg.orientation_covariance[0] = -1.0  # orientation unknown
        else:
            (msg.orientation.x, msg.orientation.y,
             msg.orientation.z, msg.orientation.w) = quat
            msg.orientation_covariance = gl.diagonal_covariance(0.01)
        (msg.angular_velocity.x, msg.angular_velocity.y,
         msg.angular_velocity.z) = gyr
        msg.angular_velocity_covariance = gl.diagonal_covariance(0.001)
        (msg.linear_acceleration.x, msg.linear_acceleration.y,
         msg.linear_acceleration.z) = acc
        msg.linear_acceleration_covariance = gl.diagonal_covariance(0.1)
        self._imu_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
