"""Start only the camera subsystem with the values in config/robot.yaml."""
from trash_vision.bringup_util import subsystem_launch


def generate_launch_description():
    return subsystem_launch("camera")
