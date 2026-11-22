"""Launch the Curiosity robot stack on the real robot.

Hardware parameters (pins, wheel size, ports) live in config/robot.yaml and
are TODO: Bryan to fill in. Nodes whose hardware is not ready can be switched
off with e.g. `use_arm:=false`. `behavior` stays disabled unless
`enable_behavior:=true`. `wheel_backend:=pico` runs the wheel loop on a Pico
(`pico_bridge` node) instead of the Pi `drive` node; the default is `pi`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from trash_vision.bringup_util import (
    backend_condition, wheel_backend_argument)

NODES = ['drive', 'ultrasonic', 'detector', 'arm', 'gps_imu']


def generate_launch_description():
    share = get_package_share_directory('trash_vision')
    params = os.path.join(share, 'config', 'robot.yaml')
    actions = [DeclareLaunchArgument('use_' + n, default_value='true')
               for n in NODES]
    actions.append(DeclareLaunchArgument(
        'enable_behavior', default_value='false'))
    actions.append(wheel_backend_argument())
    for n in NODES:
        if n == 'drive':
            continue
        actions.append(Node(
            package='trash_vision', executable=n, name=n, output='screen',
            parameters=[params],
            condition=IfCondition(LaunchConfiguration('use_' + n))))
    for backend, node in (('pi', 'drive'), ('pico', 'pico_bridge')):
        actions.append(Node(
            package='trash_vision', executable=node, name=node,
            output='screen', parameters=[params],
            condition=backend_condition(
                backend, LaunchConfiguration('use_drive'))))
    actions.append(Node(
        package='trash_vision', executable='behavior', name='behavior',
        output='screen',
        parameters=[params, {'enabled': ParameterValue(
            LaunchConfiguration('enable_behavior'), value_type=bool)}]))
    return LaunchDescription(actions)
