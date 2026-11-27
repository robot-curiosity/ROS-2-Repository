"""Check README.md only names launch files, scripts and docs that exist."""
import os
import re

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
PKG = os.path.join(ROOT, 'src', 'trash_vision')


def _readme():
    with open(os.path.join(ROOT, 'README.md')) as f:
        return f.read()


def _console_scripts():
    with open(os.path.join(PKG, 'setup.py')) as f:
        return set(re.findall(r"'(\w+) = trash_vision", f.read()))


def test_readme_launch_files_exist():
    names = set(re.findall(r'(\w+\.launch\.py)', _readme()))
    assert names
    for name in names:
        assert os.path.exists(os.path.join(PKG, 'launch', name)), name


def test_readme_ros2_run_commands_are_console_scripts():
    scripts = _console_scripts()
    for name in re.findall(r'ros2 run trash_vision (\w+)', _readme()):
        assert name in scripts, name


def test_readme_doc_and_tool_paths_exist():
    for path in re.findall(r'`((?:docs|tools)/[\w./]+)`', _readme()):
        assert os.path.exists(os.path.join(ROOT, path)), path
    assert 'docs/BRINGUP.md' in _readme()
