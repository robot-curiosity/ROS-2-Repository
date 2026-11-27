"""Check that every command named in docs/QUICKSTART.md really exists."""
import glob
import os
import re

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
PKG = os.path.join(ROOT, 'src', 'trash_vision')
DOC = os.path.join(ROOT, 'docs', 'QUICKSTART.md')


def _read(*parts):
    with open(os.path.join(*parts), encoding='utf-8') as f:
        return f.read()


def _console_scripts():
    return set(re.findall(r"'(\w+) = trash_vision", _read(PKG, 'setup.py')))


def _launch_files():
    return {os.path.basename(p)
            for p in glob.glob(os.path.join(PKG, 'launch', '*.launch.py'))}


def test_run_commands_are_console_scripts():
    names = re.findall(r'ros2 run trash_vision (\w+)', _read(DOC))
    assert names
    assert set(names) <= _console_scripts()


def test_every_console_script_is_covered():
    named = set(re.findall(r'ros2 run trash_vision (\w+)', _read(DOC)))
    assert _console_scripts() - named == set()


def test_short_command_names_are_console_scripts():
    # Backticked words with an underscore that look like tool names.
    prefixes = ('calibrate_', 'sonar_', 'record_t', 'analyze_', 'replay_',
                'trial_s', 'sweep_', 'tune_', 'hardware_')
    for word in re.findall(r'`(\w+)`', _read(DOC)):
        if word.startswith(prefixes):
            assert word in _console_scripts(), word


def test_launch_files_exist_and_all_are_covered():
    named = set(re.findall(r'(\w+\.launch\.py)', _read(DOC)))
    assert named == _launch_files()


def test_tools_and_links_exist_and_all_tools_are_covered():
    text = _read(DOC)
    named = set(re.findall(r'(tools/\w+\.py)', text))
    real = {'tools/' + os.path.basename(p)
            for p in glob.glob(os.path.join(ROOT, 'tools', '*.py'))}
    assert named == real
    links = re.findall(r'\]\(([^)#]+)\)', text)
    assert links
    for link in links:
        assert os.path.exists(os.path.join(ROOT, 'docs', link)), link


def test_options_and_launch_arguments_exist():
    sources = ''.join(
        _read(p) for p in glob.glob(os.path.join(PKG, 'trash_vision', '*.py')))
    # colcon, pip and git options are not ours.
    outside = {'--packages-select', '--break-system-packages'}
    options = set(re.findall(r'(--[a-z][\w-]+)', _read(DOC))) - outside
    assert options
    for option in options - {'--help'}:
        assert "'%s'" % option in sources, option
    launch = _read(PKG, 'launch', 'bringup.launch.py') + _read(
        PKG, 'trash_vision', 'bringup_util.py')
    for name in re.findall(r'(\w+):=', _read(DOC)):
        assert name in launch, name


def test_readme_links_the_quickstart():
    assert 'docs/QUICKSTART.md' in _read(ROOT, 'README.md')
