import ast
import os

import yaml

from trash_vision import arm_logic as al
from trash_vision import behavior_logic as bl
from trash_vision import config_logic as cl
from trash_vision import gps_imu_logic as gl
from trash_vision import safety_logic as sfl

PKG = os.path.join(os.path.dirname(__file__), "..")
NODE_FILES = {
    "drive": "drive_node.py",
    "ultrasonic": "ultrasonic_node.py",
    "arm": "arm_node.py",
    "gps_imu": "gps_imu_node.py",
    "detector": "detector_node.py",
    "behavior": "behavior_node.py",
}
_UNKNOWN = object()


def load_config():
    with open(os.path.join(PKG, "config", "robot.yaml")) as f:
        return yaml.safe_load(f)


def declared(node):
    """{parameter name: literal default or _UNKNOWN} from the node source."""
    path = os.path.join(PKG, "trash_vision", NODE_FILES[node])
    with open(path) as f:
        tree = ast.parse(f.read())
    found = {}

    def literal(expr):
        try:
            return ast.literal_eval(expr)
        except ValueError:
            return _UNKNOWN

    for call in ast.walk(tree):
        if not isinstance(call, ast.Call):
            continue
        func = getattr(call.func, "attr", "")
        if func == "declare_parameter" and call.args:
            name = literal(call.args[0])
            if isinstance(name, str):
                found[name] = (literal(call.args[1])
                               if len(call.args) > 1 else _UNKNOWN)
        elif func == "declare_parameters":
            for kw in call.keywords:
                if kw.arg == "parameters":
                    for item in kw.value.elts:
                        found[literal(item.elts[0])] = literal(item.elts[1])
    return found


def expected(node):
    names = declared(node)
    if node == "behavior":
        names.update(bl.DEFAULTS)
        names.update(sfl.DEFAULTS)
    if node == "arm":
        for pose, value in al.DEFAULT_POSES.items():
            names["pose_" + pose] = list(value)
    if node == "gps_imu":
        names["imu_i2c_addr"] = gl.BNO055_ADDR
    return {k: v for k, v in names.items() if isinstance(k, str)}


def same_type(value, default):
    if default is _UNKNOWN:
        return True
    if isinstance(default, list):
        if not isinstance(value, list):
            return False
        if value and default:
            return type(value[0]) is type(default[0])
        return True
    return type(value) is type(default)


def test_every_declared_parameter_is_in_yaml():
    config = load_config()
    for node in NODE_FILES:
        params = cl.node_params(config, node)
        for name in expected(node):
            assert name in params, "%s.%s missing in robot.yaml" % (node, name)


def test_yaml_has_no_unknown_parameters():
    config = load_config()
    for node in NODE_FILES:
        known = expected(node)
        for name in cl.node_params(config, node):
            assert name in known, "%s.%s not declared by node" % (node, name)


def test_yaml_value_types_match_node_defaults():
    # ROS rejects an int where a double is declared (and the reverse).
    config = load_config()
    for node in NODE_FILES:
        params = cl.node_params(config, node)
        for name, default in expected(node).items():
            assert same_type(params[name], default), (
                "%s.%s type differs from node default" % (node, name))


def test_every_yaml_value_has_a_comment():
    with open(os.path.join(PKG, "config", "robot.yaml")) as f:
        lines = f.read().splitlines()
    for line in lines:
        s = line.strip()
        if not s or s.startswith("#") or s.endswith(":"):
            continue
        if s.startswith("ros__parameters"):
            continue
        # Plain string names (frames) are self-explanatory.
        if "_frame:" in s or "frame_prefix:" in s:
            continue
        assert "#" in s, "no comment: " + s


def test_shipped_yaml_is_unset_for_hardware():
    config = load_config()
    for node in cl.REQUIRED:
        assert cl.find_unset(node, cl.node_params(config, node)) == \
            cl.REQUIRED[node]


def test_is_unset_markers():
    assert cl.is_unset(-1) and cl.is_unset(0.0) and cl.is_unset("")
    assert cl.is_unset([4, -1]) and cl.is_unset([])
    assert cl.is_unset(None)
    assert not cl.is_unset(17) and not cl.is_unset(0.0381)
    assert not cl.is_unset("/dev/ttyACM0") and not cl.is_unset([4, 5])
    assert not cl.is_unset(False)


def test_unset_message_names_missing_parameters():
    params = {"pwm_pins": [12, 13], "dir_pins": [5, -1], "enc_a_pins": [1, 2],
              "enc_b_pins": [3, 4], "ticks_per_rev": 0.0,
              "wheel_radius_m": 0.06, "track_width_m": 0.481}
    msg = cl.unset_message("drive", params)
    assert "dir_pins" in msg and "ticks_per_rev" in msg
    assert "pwm_pins" not in msg and "wheel_radius_m" not in msg
    assert cl.unset_message("drive", dict(
        params, dir_pins=[5, 6], ticks_per_rev=1320.0)) is None
    assert "arm_unknown" not in (cl.unset_message("arm", {}) or "")
    assert cl.unset_message("detector", {}) is None


def test_missing_parameter_counts_as_unset():
    assert cl.find_unset("arm", {}) == ["port"]


def test_each_subsystem_has_a_launch_file():
    launch = os.path.join(PKG, "launch")
    for sub, nodes in cl.SUBSYSTEMS.items():
        path = os.path.join(launch, "bringup_%s.launch.py" % sub)
        assert os.path.exists(path), path
        with open(path) as f:
            assert 'subsystem_launch("%s")' % sub in f.read()
        for n in nodes:
            assert n in NODE_FILES
    assert os.path.exists(os.path.join(launch, "bringup.launch.py"))
