# Imports:
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from trash_vision import config_logic as cl
from trash_vision import sonar_scheduler as ss
from trash_vision import ultrasonic_logic as ul


# Ultrasonic array:
class UltrasonicArray(Node):
    """Fire the sonars one at a time and publish their ranges."""

    # Initialization:
    def __init__(self) -> None:
        super().__init__('ultrasonic_array')

        n = len(ul.SENSOR_NAMES)
        # TODO: Bryan to fill in the real BCM GPIO pins (-1 = sensor unused).
        self.declare_parameter('trig_pins', [-1] * n)
        self.declare_parameter('echo_pins', [-1] * n)
        # Gap between pings; sensors are fired one at a time, round-robin.
        self.declare_parameter('settle_gap_s', 0.06)
        # Longest wait for an echo per sensor (4 m round trip is ~23 ms).
        self.declare_parameter('echo_timeout_s', 0.025)
        self.declare_parameter('median_window', 3)
        self.declare_parameter('frame_prefix', 'ultrasonic_')

        configured, problems = ul.build_sensor_config(
            ul.SENSOR_NAMES,
            list(self.get_parameter('trig_pins').value),
            list(self.get_parameter('echo_pins').value),
        )
        for problem in problems:
            self.get_logger().error(problem)
        if problems:
            configured = []
        if not configured:
            self.get_logger().warning(cl.unset_message('ultrasonic', {
                k: self.get_parameter(k).value
                for k in cl.REQUIRED['ultrasonic']
            }) or 'No ultrasonic sensors configured; node idle.')

        self._sensors = []
        for cfg in configured:
            sensor = self._open_sensor(cfg)
            if sensor is None:
                continue
            self._sensors.append({
                'name': cfg['name'],
                'device': sensor,
                'filter': ul.MedianFilter(
                    self.get_parameter('median_window').value),
                'pub': self.create_publisher(
                    Range, 'ultrasonic/' + cfg['name'], 10),
            })

        self._sched = ss.SonarScheduler(
            [s['name'] for s in self._sensors],
            self.get_parameter('settle_gap_s').value)
        self._by_name = {s['name']: s for s in self._sensors}
        self._timeout = float(self.get_parameter('echo_timeout_s').value)
        self.create_timer(0.005, self._tick)

    # Open one sensor:
    def _open_sensor(self, cfg):
        try:
            from gpiozero import DigitalInputDevice, DigitalOutputDevice
            return {
                'trig': DigitalOutputDevice(cfg['trig']),
                'echo': DigitalInputDevice(cfg['echo']),
            }
        except Exception as exc:
            self.get_logger().error(
                'Cannot open %s (trig %d, echo %d): %s'
                % (cfg['name'], cfg['trig'], cfg['echo'], exc))
            return None

    # Fire one sensor and read its echo:
    def _ping(self, device):
        device['trig'].on()
        time.sleep(1e-5)
        device['trig'].off()
        return ss.measure_echo(
            lambda: device['echo'].value, time.perf_counter, self._timeout)

    # Fire the next sensor when the settle gap has passed:
    def _tick(self) -> None:
        name = self._sched.next_due(time.monotonic())
        if name is None:
            return
        s = self._by_name[name]
        try:
            pulse = self._ping(s['device'])
        except Exception:
            pulse = None
        self._sched.fired(time.monotonic())
        rng, _status = ss.validate_range(pulse)
        rng = s['filter'].update(rng)
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_prefix').value + name
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 deg, HC-SR04 datasheet
        msg.min_range = ul.HC_SR04_MIN_RANGE_M
        msg.max_range = ul.HC_SR04_MAX_RANGE_M
        msg.range = float(rng)
        s['pub'].publish(msg)


def main(args=None) -> None:
    """Run the ultrasonic node until interrupted."""
    rclpy.init(args=args)
    node = UltrasonicArray()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
