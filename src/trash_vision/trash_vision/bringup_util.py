"""Shared launch description builder for the per-subsystem bringup files."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from trash_vision.config_logic import SUBSYSTEMS


def subsystem_launch(subsystem):
    """Launch only the nodes of one subsystem with config/robot.yaml."""
    params = os.path.join(
        get_package_share_directory("trash_vision"), "config", "robot.yaml")
    return LaunchDescription([
        Node(package="trash_vision", executable=n, name=n, output="screen",
             parameters=[params])
        for n in SUBSYSTEMS[subsystem]])
