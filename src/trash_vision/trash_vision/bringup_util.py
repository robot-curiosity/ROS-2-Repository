"""Shared launch description builder for the per-subsystem bringup files."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from trash_vision.config_logic import SUBSYSTEMS

# Wheel backends: the Pi drive node (default) or the Pico bridge (option).
WHEEL_BACKENDS = {'pi': 'drive', 'pico': 'pico_bridge'}


def backend_condition(backend, extra=None):
    """IfCondition that is true when wheel_backend equals backend."""
    expr = ["'", LaunchConfiguration('wheel_backend'), "' == '%s'" % backend]
    if extra is not None:
        expr = ["'", extra, "' == 'true' and "] + expr
    return IfCondition(PythonExpression(expr))


def wheel_backend_argument():
    """Launch argument wheel_backend:=pi|pico (default pi)."""
    return DeclareLaunchArgument(
        'wheel_backend', default_value='pi',
        description='pi = drive node on the Pi (default), '
                    'pico = pico_bridge node, wheel loop on a Pico')


def subsystem_launch(subsystem):
    """Launch only the nodes of one subsystem with config/robot.yaml."""
    params = os.path.join(
        get_package_share_directory('trash_vision'), 'config', 'robot.yaml')
    actions = []
    for n in SUBSYSTEMS[subsystem]:
        if subsystem == 'drive':
            continue
        actions.append(Node(package='trash_vision', executable=n, name=n,
                            output='screen', parameters=[params]))
    if subsystem == 'drive':
        actions.append(wheel_backend_argument())
        for backend, node in WHEEL_BACKENDS.items():
            actions.append(Node(
                package='trash_vision', executable=node, name=node,
                output='screen', parameters=[params],
                condition=backend_condition(backend)))
    return LaunchDescription(actions)
