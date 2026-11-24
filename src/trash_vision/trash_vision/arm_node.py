"""Arm node: SO101 pick service with a dry-run or LeRobot backend."""

# Imports:
import json
import threading
import time

# ROS:
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Logic:
from trash_vision import arm_logic as al


# Backend that only logs (used when no serial port is set):
class DryRunBackend:
    """Arm backend that only remembers the last pose."""

    def __init__(self) -> None:
        """Start at the stow pose."""
        self.pose = list(al.DEFAULT_POSES['stow'])

    def read(self):
        """Return the stored pose."""
        return list(self.pose)

    def write(self, pose) -> None:
        """Store the commanded pose."""
        self.pose = list(pose)

    def close(self) -> None:
        """Nothing to release."""


# Backend using LeRobot SO101 follower (lazy import):
class LeRobotBackend:
    """Arm backend using the LeRobot SO101 follower."""

    def __init__(self, port, robot_id, use_degrees) -> None:
        """Connect to the arm."""
        # TODO: Bryan to verify against the installed LeRobot version.
        from lerobot.robots.so101_follower import (
            SO101Follower, SO101FollowerConfig)
        cfg = SO101FollowerConfig(
            port=port, id=robot_id, use_degrees=use_degrees)
        self._robot = SO101Follower(cfg)
        self._robot.connect()

    def read(self):
        """Return the joint positions."""
        obs = self._robot.get_observation()
        return [float(obs[j + '.pos']) for j in al.JOINT_NAMES]

    def write(self, pose) -> None:
        """Send a joint pose to the arm."""
        self._robot.send_action(
            {j + '.pos': v for j, v in zip(al.JOINT_NAMES, pose)})

    def close(self) -> None:
        """Disconnect from the arm."""
        self._robot.disconnect()


# Arm node:
class ArmNode(Node):
    """Pick service and joint state publisher for the SO101 arm."""

    # Initialization:
    def __init__(self) -> None:
        """Declare parameters and set up the arm."""
        super().__init__('arm')
        # TODO: Bryan to fill in the serial port (e.g. /dev/ttyACM0).
        self.declare_parameter('port', '')
        self.declare_parameter('robot_id', 'curiosity_follower')
        self.declare_parameter('use_degrees', True)
        # TODO: Bryan to calibrate limits, gripper values and poses.
        self.declare_parameter('joint_min', [-100.0] * 6)
        self.declare_parameter('joint_max', [100.0] * 6)
        self.declare_parameter('gripper_open', 50.0)
        self.declare_parameter('gripper_closed', 0.0)
        self.declare_parameter('grasp_min_gap', 5.0)
        self.declare_parameter('max_step', 3.0)
        self.declare_parameter('step_period_s', 0.05)
        self.declare_parameter('state_rate_hz', 10.0)
        # IK pick waypoints (arm_kinematics.py). Off until Bryan has
        # calibrated joint_sign/offset and the mount position.
        self.declare_parameter('ik_enabled', False)
        self.declare_parameter('ik_mount_x', 0.115)
        self.declare_parameter('ik_mount_y', 0.0)
        self.declare_parameter('ik_mount_height', 0.2155)
        self.declare_parameter('ik_grasp_height', 0.03)
        self.declare_parameter('ik_clearance', 0.06)
        self.declare_parameter('ik_zone_min_x', 0.35)
        self.declare_parameter('ik_zone_max_x', 0.56)
        self.declare_parameter('ik_zone_max_abs_y', 0.30)
        self.declare_parameter('ik_approach_deg', [-90.0, -75.0, -60.0])
        self.declare_parameter('ik_joint_sign', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('ik_joint_offset', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('target_max_age_s', 1.5)
        for name, pose in al.DEFAULT_POSES.items():
            self.declare_parameter('pose_' + name, list(pose))

        overrides = {
            n: list(self.get_parameter('pose_' + n).value)
            for n in al.DEFAULT_POSES}
        self._poses, problems = al.build_poses(overrides)
        for problem in problems:
            self.get_logger().error(problem)

        lo = list(self.get_parameter('joint_min').value)
        hi = list(self.get_parameter('joint_max').value)
        self._limits = list(zip(lo, hi))

        self._lock = threading.Lock()
        self._backend = self._open_backend()
        self._busy = False
        self._target = None
        self._target_time = 0.0
        self.create_subscription(
            String, '/vision/target', self._on_target, 10)

        group = ReentrantCallbackGroup()
        self._state_pub = self.create_publisher(
            JointState, 'arm/joint_states', 10)
        for seq in al.SEQUENCES:
            self.create_service(
                Trigger, 'arm/' + seq,
                lambda req, res, s=seq: self._run(s, res),
                callback_group=group)
        rate = float(self.get_parameter('state_rate_hz').value)
        self.create_timer(
            1.0 / max(rate, 0.1), self._publish_state,
            callback_group=group)

    # Latest detector target (base_x/base_y in base_link):
    def _on_target(self, msg) -> None:
        xy = al.parse_target_xy(msg.data)
        if xy is not None:
            self._target = xy
            self._target_time = time.monotonic()

    # Build the pose table for one pick (IK if enabled and target fresh):
    def _pick_poses(self):
        """Return (poses, error). error is None when poses can be used."""
        if not bool(self.get_parameter('ik_enabled').value):
            return self._poses, None
        age = time.monotonic() - self._target_time
        if (self._target is None
                or age > float(self.get_parameter('target_max_age_s').value)):
            return None, 'no fresh target with base_x/base_y'
        g = self.get_parameter
        cfg = al.PickConfig(
            mount_x=float(g('ik_mount_x').value),
            mount_y=float(g('ik_mount_y').value),
            mount_height=float(g('ik_mount_height').value),
            grasp_height=float(g('ik_grasp_height').value),
            clearance=float(g('ik_clearance').value),
            zone_min_x=float(g('ik_zone_min_x').value),
            zone_max_x=float(g('ik_zone_max_x').value),
            zone_max_abs_y=float(g('ik_zone_max_abs_y').value),
            approach_deg=tuple(g('ik_approach_deg').value),
            use_degrees=bool(g('use_degrees').value),
            joint_sign=tuple(g('ik_joint_sign').value),
            joint_offset=tuple(g('ik_joint_offset').value),
            gripper_open=float(g('gripper_open').value))
        poses, why = al.plan_pick_poses(
            self._target[0], self._target[1], cfg)
        if poses is None:
            return None, why
        merged = dict(self._poses)
        merged.update(poses)
        return merged, None

    # Open backend:
    def _open_backend(self):
        port = self.get_parameter('port').value
        if not port:
            self.get_logger().warning(
                'arm idle: set port in config/robot.yaml (dry-run, no '
                'motion). TODO: Bryan to fill in.')
            return DryRunBackend()
        try:
            return LeRobotBackend(
                port, self.get_parameter('robot_id').value,
                bool(self.get_parameter('use_degrees').value))
        except Exception as exc:
            self.get_logger().error(
                'Cannot open arm on %s: %s; using dry-run' % (port, exc))
            return DryRunBackend()

    # Publish joint state:
    def _publish_state(self) -> None:
        with self._lock:
            try:
                pos = self._backend.read()
            except Exception as exc:
                self.get_logger().error('arm read failed: %s' % exc)
                return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(al.JOINT_NAMES)
        msg.position = [float(p) for p in pos]
        self._state_pub.publish(msg)

    # Run a sequence (service callback):
    def _run(self, name, res):
        with self._lock:
            if self._busy:
                res.success = False
                res.message = 'arm busy'
                return res
            self._busy = True
        try:
            grip_open = float(self.get_parameter('gripper_open').value)
            grip_closed = float(self.get_parameter('gripper_closed').value)
            poses = self._poses
            if name == 'pick':
                poses, error = self._pick_poses()
                if error:
                    res.success = False
                    res.message = 'pick rejected: ' + error
                    return res
            targets = al.plan_sequence(
                name, poses, grip_open, grip_closed)
            step = float(self.get_parameter('max_step').value)
            period = float(self.get_parameter('step_period_s').value)
            # Check every pose before moving (a bad one raises here).
            targets = [al.clamp_pose(t, self._limits) for t in targets]
            for target in targets:
                with self._lock:
                    current = self._backend.read()
                for pose in al.interpolate(current, target, step):
                    with self._lock:
                        self._backend.write(al.clamp_pose(pose, self._limits))
                    time.sleep(period)
            res.success = True
            res.message = name + ' done'
            if name == 'pick':
                with self._lock:
                    grip = self._backend.read()[-1]
                held = al.grasp_held(
                    grip, grip_closed,
                    float(self.get_parameter('grasp_min_gap').value))
                res.success = held
                res.message = ('item held' if held
                               else 'pick finished, nothing in gripper')
        except Exception as exc:
            res.success = False
            res.message = 'arm error: %s' % exc
            self.get_logger().error(res.message)
        finally:
            with self._lock:
                self._busy = False
        return res

    # Shutdown:
    def close(self) -> None:
        """Release the backend."""
        try:
            self._backend.close()
        except Exception:
            pass


def main(args=None) -> None:
    """Run the arm node."""
    rclpy.init(args=args)
    node = ArmNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
