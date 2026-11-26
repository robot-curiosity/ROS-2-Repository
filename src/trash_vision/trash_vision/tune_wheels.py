"""Wheel step-response tuner (WHEELS OFF THE GROUND).

  ros2 run trash_vision tune_wheels [--speed 0.2] [--seconds 2] [--side left]
  ros2 run trash_vision tune_wheels --analyze run.csv

Needs the wheel driver running (drive or pico_bridge) and nothing else
publishing /cmd_vel. All maths is in wheel_tune_logic.py (tested in CI);
this wrapper only talks to ROS, so it needs the real robot to run.
"""
import argparse
import os
import sys
import time

from trash_vision import wheel_tune_logic as wl

DRIVERS = ('drive', 'pico_bridge')


class _RosLink:
    """The ROS side: cmd_vel out, odom in, driver parameters."""

    def __init__(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        self._rclpy = rclpy
        self._twist = Twist
        rclpy.init()
        self.node = rclpy.create_node('tune_wheels')
        self._pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self._samples = []
        self.node.create_subscription(Odometry, 'odom', self._on_odom, 50)

    def _on_odom(self, msg):
        self._samples.append((self.clock(), msg.twist.twist.linear.x,
                              msg.twist.twist.angular.z))

    def clock(self):
        return time.monotonic()

    def send(self, linear, angular):
        msg = self._twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._pub.publish(msg)

    def sleep(self, seconds):
        end = self.clock() + seconds
        while self.clock() < end:
            self._rclpy.spin_once(self.node,
                                  timeout_sec=max(end - self.clock(), 0.0))

    def read(self):
        out, self._samples = self._samples, []
        return out

    def driver_params(self):
        """Parameters of the one running wheel driver, or an error text."""
        from rcl_interfaces.srv import GetParameters
        running = [n for n in DRIVERS if n in self.node.get_node_names()]
        if len(running) != 1:
            return None, ('exactly one of drive or pico_bridge must be '
                          'running (found %s)' % (running or 'none'))
        client = self.node.create_client(
            GetParameters, '/%s/get_parameters' % running[0])
        if not client.wait_for_service(timeout_sec=3.0):
            return None, 'cannot read the parameters of %s' % running[0]
        request = GetParameters.Request()
        request.names = list(wl.PARAM_NAMES)
        future = client.call_async(request)
        self._rclpy.spin_until_future_complete(self.node, future,
                                               timeout_sec=3.0)
        if not future.done() or len(future.result().values) != \
                len(wl.PARAM_NAMES):
            return None, 'cannot read the parameters of %s' % running[0]
        params = {n: float(v.double_value or v.integer_value)
                  for n, v in zip(wl.PARAM_NAMES, future.result().values)}
        print('Wheel driver: %s' % running[0])
        return params, None

    def other_publishers(self):
        return self.node.count_publishers('cmd_vel') - 1

    def close(self):
        self.node.destroy_node()
        self._rclpy.shutdown()


def _measure(args):
    link = _RosLink()
    try:
        link.sleep(1.5)  # discovery
        params, error = link.driver_params()
        if error:
            print('FAIL: %s' % error)
            return None
        if link.other_publishers() > 0:
            print('FAIL: something else publishes /cmd_vel (stop behavior, '
                  'teleop and the other tools first).')
            return None
        if params['track_width_m'] <= 0 or params['max_wheel_speed_mps'] <= 0:
            print('FAIL: set track_width_m and max_wheel_speed_mps first.')
            return None
        speed = wl.step_speed(args.speed, params['max_wheel_speed_mps'])
        print('The wheels will spin at %.2f m/s. Lift the robot so ALL wheels '
              'are off the ground and keep a hand on motor power.' % speed)
        if input('Type yes to start: ').strip().lower() != 'yes':
            print('Cancelled.')
            return None
        records = {}
        sides = [args.side] if args.side else list(wl.SIDES)
        for side in sides:
            link.send(0.0, 0.0)
            link.sleep(1.0)
            print('Stepping the %s side (Ctrl-C stops)...' % side)
            samples, reason = wl.run_step(
                side, speed, params['track_width_m'], link.send, link.read,
                link.clock, link.sleep, duration_s=args.seconds)
            link.sleep(0.2)  # let the stop go out
            records[side] = (speed, samples)
            if reason:
                print('ABORTED on the %s side: %s' % (side, reason))
                return records, params, False
        return records, params, True
    except KeyboardInterrupt:
        print('\nStopped.')
        return None
    finally:
        try:
            link.send(0.0, 0.0)
        finally:
            link.close()


def main(argv=None):
    """Run the step test (or re-analyze a CSV) and print the report."""
    p = argparse.ArgumentParser(
        description='Wheel speed step response, metrics and suggested PI '
        'gains. WHEELS OFF THE GROUND.')
    p.add_argument('--speed', type=float, default=0.2,
                   help='step in m/s (capped at %g)' % wl.SPEED_CAP_MPS)
    p.add_argument('--seconds', type=float, default=2.0,
                   help='length of each step (1 to 10)')
    p.add_argument('--side', choices=wl.SIDES, default=None)
    p.add_argument('--out', default='~/curiosity_logs/wheel_tune',
                   help='folder for the CSV and SVG')
    p.add_argument('--analyze', metavar='CSV', default=None,
                   help='re-analyze a saved run (no robot needed)')
    args = p.parse_args(argv)
    if not 1.0 <= args.seconds <= 10.0:
        p.error('--seconds must be between 1 and 10')
    if not args.speed > 0.0:
        p.error('--speed must be positive')

    complete = True
    if args.analyze:
        try:
            with open(args.analyze, encoding='utf-8') as fh:
                records, params = wl.from_csv(fh.read())
        except (OSError, ValueError) as exc:
            print('FAIL: %s' % exc)
            return 2
    else:
        measured = _measure(args)
        if measured is None:
            return 1
        records, params, complete = measured
        folder = os.path.expanduser(args.out)
        os.makedirs(folder, exist_ok=True)
        base = os.path.join(folder, time.strftime('tune_%Y%m%d_%H%M%S'))
        for suffix, text in (('.csv', wl.to_csv(records, params)),
                             ('.svg', wl.to_svg(records))):
            with open(base + suffix, 'w', encoding='utf-8') as fh:
                fh.write(text)
        print('Saved %s.csv and %s.svg' % (base, base))

    results = {}
    for side, (target, samples) in records.items():
        try:
            results[side] = wl.analyze_side(samples, target, params)
        except ValueError as exc:
            print('%s: cannot analyze (%s)' % (side, exc))
            complete = False
    if results:
        print(wl.format_report(results, params))
    return 0 if complete and results else 1


def run():
    """Console script entry point."""
    sys.exit(main())


if __name__ == '__main__':
    run()
