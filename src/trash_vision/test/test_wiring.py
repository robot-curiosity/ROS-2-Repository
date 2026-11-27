"""Static wiring checks: yaml vs declared parameters, node names, topics."""
import ast
import os
import re
import subprocess
import sys

import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
MOD = os.path.join(PKG, 'trash_vision')
sys.path.insert(0, PKG)

# yaml section -> source file of the node that reads it
NODE_FILES = {
    'drive': 'drive_node.py', 'pico_bridge': 'pico_bridge_node.py',
    'ultrasonic': 'ultrasonic_node.py', 'arm': 'arm_node.py',
    'gps_imu': 'gps_imu_node.py', 'detector': 'detector_node.py',
    'behavior': 'behavior_node.py',
}


def _tree(name):
    with open(os.path.join(MOD, name)) as f:
        return ast.parse(f.read())


def _declared(name):
    """Literal parameter names declared in a node file."""
    out = set()
    for n in ast.walk(_tree(name)):
        if not isinstance(n, ast.Call):
            continue
        fn = getattr(n.func, 'attr', '')
        if fn == 'declare_parameter' and n.args:
            a = n.args[0]
            if isinstance(a, ast.Constant):
                out.add(a.value)
            elif isinstance(a, ast.BinOp) and isinstance(
                    a.left, ast.Constant):
                out.add(a.left.value)  # 'pose_' + name: keep the prefix
        if fn == 'declare_parameters':
            for kw in n.keywords:
                if kw.arg == 'parameters':
                    for t in kw.value.elts:
                        out.add(t.elts[0].value)
    return out


def _yaml():
    with open(os.path.join(PKG, 'config', 'robot.yaml')) as f:
        return yaml.safe_load(f)


def _all_declared(section):
    names = _declared(NODE_FILES[section])
    if section == 'behavior':
        from trash_vision import behavior_logic as bl
        from trash_vision import safety_logic as sfl
        names |= set(bl.DEFAULTS) | set(sfl.DEFAULTS)
    return names


def _known(declared, key):
    return key in declared or any(
        d.endswith('_') and key.startswith(d) for d in declared)


def test_every_yaml_key_is_declared_by_its_node():
    cfg = _yaml()
    for section, file in NODE_FILES.items():
        declared = _all_declared(section)
        params = cfg[section]['ros__parameters']
        for key in params:
            assert _known(declared, key), (section, key)


def test_every_hardware_parameter_is_in_yaml():
    cfg = _yaml()
    # sim has no yaml section; behavior and detector tuning may use defaults.
    for section in ('drive', 'pico_bridge', 'ultrasonic', 'arm', 'gps_imu'):
        declared = {d for d in _all_declared(section)
                    if not d.endswith('_')}
        missing = declared - set(cfg[section]['ros__parameters'])
        assert not missing, (section, sorted(missing))


def test_yaml_sections_match_launch_node_names():
    from trash_vision.config_logic import SUBSYSTEMS
    cfg = _yaml()
    for nodes in SUBSYSTEMS.values():
        for n in nodes:
            assert n in cfg, n


def test_node_names_match_yaml_sections():
    # `ros2 run ... --params-file robot.yaml` only works if they agree.
    for section, file in NODE_FILES.items():
        for n in ast.walk(_tree(file)):
            if (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                    and n.func.attr == '__init__' and n.args
                    and isinstance(n.args[0], ast.Constant)):
                assert n.args[0].value == section, (file, n.args[0].value)


def _consts(name, method):
    out = []
    for n in ast.walk(_tree(name)):
        if (isinstance(n, ast.Call) and getattr(n.func, 'attr', '') == method
                and len(n.args) >= 2 and isinstance(n.args[1], ast.Constant)):
            out.append(n.args[1].value)
    return out


def test_behavior_subscriptions_have_publishers():
    subs = set(_consts('behavior_node.py', 'create_subscription'))
    assert {'/vision/target', 'odom'} <= subs
    pubs = set()
    for f in ('detector_node.py', 'sim_node.py'):
        text = open(os.path.join(MOD, f)).read()
        pubs |= set(re.findall(r"'(/vision/target)'", text))
    assert '/vision/target' in pubs
    for f in ('drive_node.py', 'pico_bridge_node.py', 'sim_node.py'):
        assert 'odom' in _consts(f, 'create_publisher'), f


def test_cmd_vel_has_one_consumer_per_backend():
    assert 'cmd_vel' in _consts('behavior_node.py', 'create_publisher')
    for f in ('drive_node.py', 'pico_bridge_node.py', 'sim_node.py'):
        assert 'cmd_vel' in _consts(f, 'create_subscription'), f


def test_arm_services_cover_behavior_clients():
    from trash_vision import arm_logic as al
    for name in ('pick', 'deposit', 'stow'):
        assert name in al.SEQUENCES


def test_helper_scripts_print_help_without_ros():
    for mod in ('trial_summary', 'hardware_check', 'dashboard'):
        r = subprocess.run(
            [sys.executable, '-m', 'trash_vision.' + mod, '--help'],
            cwd=PKG, capture_output=True, text=True, timeout=60)
        assert r.returncode == 0, (mod, r.stderr[-300:])
        assert 'usage' in r.stdout.lower(), mod


def test_calibrate_scripts_print_help_without_ros():
    for fn in ('main_encoders', 'main_drive', 'main_pick_zone',
               'main_camera', 'main_sonar'):
        code = ('import sys; sys.argv = ["x", "--help"]; '
                'from trash_vision import calibrate as c; '
                'sys.exit(c.%s())' % fn)
        r = subprocess.run(
            [sys.executable, '-c', code], cwd=PKG,
            capture_output=True, text=True, timeout=60)
        assert 'usage' in r.stdout.lower(), (fn, r.stderr[-300:])


def test_setup_entry_points_resolve():
    with open(os.path.join(PKG, 'setup.py')) as f:
        pairs = re.findall(r"'(\w+) = trash_vision\.(\w+):(\w+)'", f.read())
    assert len(pairs) >= 14
    for script, module, func in pairs:
        names = {n.name for n in _tree(module + '.py').body
                 if isinstance(n, ast.FunctionDef)}
        names |= {t.id for n in _tree(module + '.py').body
                  if isinstance(n, ast.Assign) for t in n.targets
                  if isinstance(t, ast.Name)}
        assert func in names, (script, module, func)


def test_bringup_doc_names_exist():
    root = os.path.abspath(os.path.join(PKG, '..', '..'))
    text = open(os.path.join(root, 'docs', 'BRINGUP.md')).read()
    with open(os.path.join(PKG, 'setup.py')) as f:
        scripts = set(re.findall(r"'(\w+) = ", f.read()))
    for s in re.findall(r'ros2 run trash_vision (\w+)', text):
        assert s in scripts, s
    for s in re.findall(r'(\w+\.launch\.py)', text):
        assert os.path.exists(os.path.join(PKG, 'launch', s)), s
