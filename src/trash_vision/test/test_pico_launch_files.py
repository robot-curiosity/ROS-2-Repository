"""Static checks that the Pico option is behind wheel_backend, default pi."""
import os

PKG = os.path.join(os.path.dirname(__file__), '..')


def _read(*parts):
    with open(os.path.join(PKG, *parts)) as f:
        return f.read()


def test_wheel_backend_defaults_to_pi():
    """Check: the launch argument defaults to the Pi drive node."""
    util = _read('trash_vision', 'bringup_util.py')
    assert "'wheel_backend', default_value='pi'" in util
    assert "{'pi': 'drive', 'pico': 'pico_bridge'}" in util


def test_both_launch_entry_points_use_the_backend_argument():
    """Check: bringup and bringup_drive offer wheel_backend."""
    assert 'wheel_backend_argument()' in _read('launch', 'bringup.launch.py')
    assert "subsystem_launch('drive')" in _read(
        'launch', 'bringup_drive.launch.py')
    assert "subsystem == 'drive'" in _read('trash_vision', 'bringup_util.py')


def test_console_script_exists():
    """Check: setup.py registers pico_bridge."""
    assert 'pico_bridge = trash_vision.pico_bridge_node:main' in _read(
        'setup.py')


def test_drive_node_not_touched_by_pico_option():
    """Check: the Pi drive node does not mention the Pico."""
    source = _read('trash_vision', 'drive_node.py')
    assert 'pico_bridge' not in source and 'wheel_backend' not in source
