# Imports:
import threading
import time

# ROS:
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# Logic:
from trash_vision import arm_logic as al


# Backend that only logs (used when no serial port is set):
class DryRunBackend:
    def __init__(self) -> None:
        self.pose = list(al.DEFAULT_POSES["stow"])

    def read(self):
        return list(self.pose)

    def write(self, pose) -> None:
        self.pose = list(pose)

    def close(self) -> None:
        pass


# Backend using LeRobot SO101 follower (lazy import):
class LeRobotBackend:
    def __init__(self, port, robot_id, use_degrees) -> None:
        # TODO: Bryan to verify against the installed LeRobot version.
        from lerobot.robots.so101_follower import (
            SO101Follower, SO101FollowerConfig)
        cfg = SO101FollowerConfig(
            port=port, id=robot_id, use_degrees=use_degrees)
        self._robot = SO101Follower(cfg)
        self._robot.connect()

    def read(self):
        obs = self._robot.get_observation()
        return [float(obs[j + ".pos"]) for j in al.JOINT_NAMES]

    def write(self, pose) -> None:
        self._robot.send_action(
            {j + ".pos": v for j, v in zip(al.JOINT_NAMES, pose)})

    def close(self) -> None:
        self._robot.disconnect()


# Arm node:
class ArmNode(Node):
    # Initialization:
    def __init__(self) -> None:
        super().__init__("arm")
        # TODO: Bryan to fill in the serial port (e.g. /dev/ttyACM0).
        self.declare_parameter("port", "")
        self.declare_parameter("robot_id", "curiosity_follower")
        self.declare_parameter("use_degrees", True)
        # TODO: Bryan to calibrate limits, gripper values and poses.
        self.declare_parameter("joint_min", [-100.0] * 6)
        self.declare_parameter("joint_max", [100.0] * 6)
        self.declare_parameter("gripper_open", 50.0)
        self.declare_parameter("gripper_closed", 0.0)
        self.declare_parameter("grasp_min_gap", 5.0)
        self.declare_parameter("max_step", 3.0)
        self.declare_parameter("step_period_s", 0.05)
        self.declare_parameter("state_rate_hz", 10.0)
        for name, pose in al.DEFAULT_POSES.items():
            self.declare_parameter("pose_" + name, list(pose))

        overrides = {
            n: list(self.get_parameter("pose_" + n).value)
            for n in al.DEFAULT_POSES}
        self._poses, problems = al.build_poses(overrides)
        for problem in problems:
            self.get_logger().error(problem)

        lo = list(self.get_parameter("joint_min").value)
        hi = list(self.get_parameter("joint_max").value)
        self._limits = list(zip(lo, hi))

        self._lock = threading.Lock()
        self._backend = self._open_backend()
        self._busy = False

        group = ReentrantCallbackGroup()
        self._state_pub = self.create_publisher(
            JointState, "arm/joint_states", 10)
        for seq in al.SEQUENCES:
            self.create_service(
                Trigger, "arm/" + seq,
                lambda req, res, s=seq: self._run(s, res),
                callback_group=group)
        rate = float(self.get_parameter("state_rate_hz").value)
        self.create_timer(
            1.0 / max(rate, 0.1), self._publish_state,
            callback_group=group)

    # Open backend:
    def _open_backend(self):
        port = self.get_parameter("port").value
        if not port:
            self.get_logger().warning(
                "No arm 'port' set; running dry-run (no motion). "
                "TODO: Bryan to fill in the serial port.")
            return DryRunBackend()
        try:
            return LeRobotBackend(
                port, self.get_parameter("robot_id").value,
                bool(self.get_parameter("use_degrees").value))
        except Exception as exc:
            self.get_logger().error(
                "Cannot open arm on %s: %s; using dry-run" % (port, exc))
            return DryRunBackend()

    # Publish joint state:
    def _publish_state(self) -> None:
        with self._lock:
            try:
                pos = self._backend.read()
            except Exception as exc:
                self.get_logger().error("arm read failed: %s" % exc)
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
                res.message = "arm busy"
                return res
            self._busy = True
        try:
            grip_open = float(self.get_parameter("gripper_open").value)
            grip_closed = float(self.get_parameter("gripper_closed").value)
            targets = al.plan_sequence(
                name, self._poses, grip_open, grip_closed)
            step = float(self.get_parameter("max_step").value)
            period = float(self.get_parameter("step_period_s").value)
            for target in targets:
                target = al.clamp_pose(target, self._limits)
                with self._lock:
                    current = self._backend.read()
                for pose in al.interpolate(current, target, step):
                    with self._lock:
                        self._backend.write(al.clamp_pose(pose, self._limits))
                    time.sleep(period)
            res.success = True
            res.message = name + " done"
            if name == "pick":
                with self._lock:
                    grip = self._backend.read()[-1]
                held = al.grasp_held(
                    grip, grip_closed,
                    float(self.get_parameter("grasp_min_gap").value))
                res.success = held
                res.message = ("item held" if held
                               else "pick finished, nothing in gripper")
        except Exception as exc:
            res.success = False
            res.message = "arm error: %s" % exc
            self.get_logger().error(res.message)
        finally:
            with self._lock:
                self._busy = False
        return res

    # Shutdown:
    def close(self) -> None:
        try:
            self._backend.close()
        except Exception:
            pass


def main(args=None) -> None:
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


if __name__ == "__main__":
    main()
