"""Run the behavior node against the pure-Python kinematic simulator.

No hardware, camera or models needed: `sim` replaces drive, ultrasonic,
detector and arm. Try: ros2 launch trash_vision sim.launch.py
then watch `ros2 topic echo /behavior/state`.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="trash_vision", executable="sim", name="sim",
             output="screen"),
        Node(package="trash_vision", executable="behavior", name="behavior",
             output="screen", parameters=[{"enabled": True}]),
    ])
