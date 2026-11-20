"""Checks that package.xml has a real description and declares key dependencies."""
import os
import xml.etree.ElementTree as ET

PKG = os.path.join(os.path.dirname(__file__), '..', 'package.xml')


def _root():
    return ET.parse(PKG).getroot()


def test_description_is_real():
    desc = _root().findtext('description')
    assert desc and not desc.startswith('TODO')


def test_license_left_for_bryan():
    assert _root().findtext('license').startswith('TODO')


def test_python_dependencies_declared():
    deps = {e.text for e in _root() if e.tag in ('depend', 'exec_depend')}
    for key in ('python3-opencv', 'python3-requests', 'python3-yaml', 'python3-serial'):
        assert key in deps
