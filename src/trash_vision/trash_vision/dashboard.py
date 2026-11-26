"""Tuning dashboard node: ROS topics to a small web page on the Pi.

``ros2 run trash_vision dashboard`` is read-only. Add ``--allow-write`` to
change behavior parameters from the page. There is no password: use it only
on a network you trust.
"""
import argparse
import sys
import threading
import time

from trash_vision import dashboard_logic as dl
from trash_vision import tuning_logic as tu
from trash_vision import ultrasonic_logic as ul

# Shown and needed for cross-checks, not editable:
EXTRA_PARAMS = ['safety_hard_stop_distance']


def parse_args(argv):
    """Parse the command line (ROS arguments already removed)."""
    parser = argparse.ArgumentParser(
        prog='dashboard', description='Live tuning web page for Curiosity.')
    parser.add_argument(
        '--host', default=None,
        help='local network address to listen on (default: the LAN address)')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument(
        '--allow-write', action='store_true',
        help='let the page change behavior parameters')
    return parser.parse_args(argv)


def build_node(allow_write):
    """Create the ROS node (imports ROS here so --help works without it)."""
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically
    from rclpy.node import Node
    from sensor_msgs.msg import Range
    from std_msgs.msg import Float32, String

    class DashboardNode(Node):
        """Collect robot data and forward parameter writes."""

        def __init__(self):
            super().__init__('dashboard')
            self.allow_write = allow_write
            self.data = dl.DashboardData()
            sub = self.create_subscription
            for name in (ul.FRONT_NAMES + ul.REAR_NAMES + ul.LEFT_NAMES
                         + ul.RIGHT_NAMES):
                sub(Range, '/ultrasonic/' + name,
                    lambda m, n=name: self._put('sonar/' + n, m.range), 10)
            sub(Twist, 'cmd_vel', lambda m: self._put(
                'cmd_vel', (m.linear.x, m.angular.z)), 10)
            sub(Odometry, 'odom', lambda m: self._put(
                'odom', (m.twist.twist.linear.x, m.twist.twist.angular.z)), 10)
            sub(String, 'behavior/state',
                lambda m: self._put('state', m.data), 10)
            sub(String, 'behavior/safety',
                lambda m: self.data.put_safety(m.data, time.monotonic()), 10)
            sub(String, '/vision/target', lambda m: self.data.put_detection(
                m.data, time.monotonic()), 10)
            sub(Float32, '/vision/fps',
                lambda m: self._put('fps', round(m.data, 1)), 10)
            self._get = self.create_client(
                GetParameters, '/behavior/get_parameters')
            self._set = self.create_client(
                SetParametersAtomically, '/behavior/set_parameters_atomically')
            # Wheel base from whichever drive backend is running:
            self._track = [
                self.create_client(GetParameters, '/%s/get_parameters' % n)
                for n in ('drive', 'pico_bridge')]
            self.create_timer(1.0, self._poll_params)

        def _put(self, key, value):
            self.data.put(key, value, time.monotonic())

        def _poll_params(self):
            names = list(tu.TUNABLES) + EXTRA_PARAMS
            if self._get.service_is_ready():
                self._get.call_async(
                    GetParameters.Request(names=names)
                ).add_done_callback(lambda f: self._on_params(names, f))
            for client in self._track:
                if client.service_is_ready():
                    client.call_async(GetParameters.Request(
                        names=['track_width_m'])
                    ).add_done_callback(self._on_track)

        def _on_params(self, names, future):
            values = {
                n: v.double_value
                for n, v in zip(names, future.result().values)
                if v.type == ParameterType.PARAMETER_DOUBLE}
            self._put('params', values)

        def _on_track(self, future):
            values = future.result().values
            if values and values[0].double_value > 0.0:
                self._put('track_width', values[0].double_value)

        def snapshot(self):
            return self.data.snapshot(time.monotonic())

        def write(self, changes):
            """Set parameters on the behavior node (called by web threads)."""
            if not self._set.service_is_ready():
                return False, 'behavior node is not running'
            request = SetParametersAtomically.Request(parameters=[
                Parameter(name=n, value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=v))
                for n, v in changes.items()])
            done = threading.Event()
            future = self._set.call_async(request)
            future.add_done_callback(lambda f: done.set())
            if not done.wait(2.0):
                return False, 'no answer from the behavior node'
            result = future.result().result
            if not result.successful:
                return False, result.reason
            self.get_logger().warning('dashboard changed %s' % changes)
            return True, 'set'

    return DashboardNode()


def main(args=None):
    """Run the dashboard until Ctrl-C."""
    argv = sys.argv if args is None else ['dashboard'] + list(args)
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    options = parse_args(argv[1:])
    import rclpy
    rclpy.init(args=args)
    node = build_node(options.allow_write)
    host = options.host or dl.default_host()
    try:
        server = dl.make_server(node, host, options.port)
    except (ValueError, OSError) as exc:
        node.get_logger().error('cannot start the web page: %s' % exc)
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(1)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info('dashboard on http://%s:%d (%s)' % (
        host, options.port,
        'WRITE ENABLED' if options.allow_write else 'read-only'))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
