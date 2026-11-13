# Imports:
import math

# ROS:
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

# Logic:
from trash_vision import ultrasonic_logic as ul


# Ultrasonic array:
class UltrasonicArray(Node):
    # Initialization:
    def __init__(self) -> None:
        super().__init__("ultrasonic_array")

        n = len(ul.SENSOR_NAMES)
        # TODO: Bryan to fill in the real BCM GPIO pins (-1 = sensor unused).
        self.declare_parameter("trig_pins", [-1] * n)
        self.declare_parameter("echo_pins", [-1] * n)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("median_window", 3)
        self.declare_parameter("frame_prefix", "ultrasonic_")

        configured, problems = ul.build_sensor_config(
            ul.SENSOR_NAMES,
            list(self.get_parameter("trig_pins").value),
            list(self.get_parameter("echo_pins").value),
        )
        for problem in problems:
            self.get_logger().error(problem)
        if problems:
            configured = []
        if not configured:
            self.get_logger().warning(
                "No ultrasonic pins configured (trig_pins/echo_pins); "
                "node idle. TODO: Bryan to fill in pins.")

        self._sensors = []
        for cfg in configured:
            sensor = self._open_sensor(cfg)
            if sensor is None:
                continue
            self._sensors.append({
                "name": cfg["name"],
                "device": sensor,
                "filter": ul.MedianFilter(
                    self.get_parameter("median_window").value),
                "pub": self.create_publisher(
                    Range, "ultrasonic/" + cfg["name"], 10),
            })

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._tick)

    # Open one sensor:
    def _open_sensor(self, cfg):
        try:
            from gpiozero import DistanceSensor
            return DistanceSensor(
                echo=cfg["echo"], trigger=cfg["trig"],
                max_distance=ul.HC_SR04_MAX_RANGE_M)
        except Exception as exc:
            self.get_logger().error(
                "Cannot open %s (trig %d, echo %d): %s"
                % (cfg["name"], cfg["trig"], cfg["echo"], exc))
            return None

    # Read and publish:
    def _tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        prefix = self.get_parameter("frame_prefix").value
        for s in self._sensors:
            try:
                raw = s["device"].distance * ul.HC_SR04_MAX_RANGE_M
            except Exception:
                raw = None
            rng = ul.classify_range(raw)
            if math.isfinite(rng):
                rng = s["filter"].update(rng)
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = prefix + s["name"]
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26  # ~15 deg, HC-SR04 datasheet
            msg.min_range = ul.HC_SR04_MIN_RANGE_M
            msg.max_range = ul.HC_SR04_MAX_RANGE_M
            msg.range = float(rng)
            s["pub"].publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasonicArray()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
