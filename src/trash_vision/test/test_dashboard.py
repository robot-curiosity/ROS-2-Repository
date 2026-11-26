"""Tests for the tuning dashboard data, limits and web request handling."""
import ast
import json
import os
import threading
import urllib.error
import urllib.request

import pytest

from trash_vision import behavior_logic as bl
from trash_vision import dashboard as dash
from trash_vision import dashboard_logic as dl
from trash_vision import safety_logic as sfl
from trash_vision import tuning_logic as tu

CURRENT = dict(bl.DEFAULTS, **sfl.DEFAULTS)
HEADERS = {'host': '192.168.1.20:8080', 'content-type': 'application/json'}


class FakeApp:
    """Stands in for the ROS node."""

    def __init__(self, allow_write=True, ok=True):
        self.allow_write = allow_write
        self.ok = ok
        self.written = []
        self.data = dl.DashboardData()
        self.data.put('params', {n: CURRENT[n] for n in tu.TUNABLES}, 0.0)

    def snapshot(self):
        return self.data.snapshot(1.0)

    def write(self, changes):
        self.written.append(changes)
        return self.ok, 'set' if self.ok else 'behavior node is not running'


def post(app, body, headers=HEADERS, client='192.168.1.5'):
    status, _, data = dl.handle_request(
        app, 'POST', '/api/params', headers, body, client)
    return status, json.loads(data)


def test_every_tunable_is_a_float_default_inside_its_limits():
    for name, spec in tu.TUNABLES.items():
        assert isinstance(CURRENT[name], float), name
        assert spec[0] <= CURRENT[name] <= spec[1], name
    assert tu.validate_changes(
        {n: CURRENT[n] for n in tu.TUNABLES}, CURRENT) == []


def test_enable_and_safety_switches_are_not_tunable():
    for name in ('enabled', 'safety_require_sonars',
                 'safety_hard_stop_distance', 'safety_hard_stop_escape'):
        assert tu.validate_changes({name: 1.0}, CURRENT)


@pytest.mark.parametrize('value', [
    float('nan'), float('inf'), True, '0.3', None, 99.0, -1.0])
def test_bad_values_are_refused(value):
    assert tu.validate_changes({'approach_speed': value}, CURRENT)


def test_distance_rules():
    assert tu.validate_changes({'stop_distance': 0.6}, CURRENT)
    assert tu.validate_changes({'slow_distance': 0.25}, CURRENT)
    assert not tu.validate_changes(
        {'stop_distance': 0.7, 'slow_distance': 0.9}, CURRENT)
    hard = dict(CURRENT, safety_hard_stop_distance=0.3)
    assert tu.validate_changes({'stop_distance': 0.25}, hard)
    # An unrelated change is not blocked by an already odd pair:
    odd = dict(CURRENT, slow_distance=0.2)
    assert not tu.validate_changes({'approach_speed': 0.1}, odd)


def test_live_change_reaches_the_machine_and_rebuilds_rows():
    machine = bl.BehaviorMachine()
    machine.step(0.0, None, [2.0, 2.0], [2.0, 2.0], pose=(0.0, 0.0, 0.0))
    before = len(machine.search.waypoints)
    machine.set_param('search_row_spacing', 0.5)
    machine.set_param('search_speed', 0.1)
    assert machine.p['search_speed'] == machine.search.p['search_speed'] == 0.1
    machine.step(0.1, None, [2.0, 2.0], [2.0, 2.0], pose=(0.0, 0.0, 0.0))
    assert len(machine.search.waypoints) > before
    machine.set_param('stop_distance', 0.5)
    cmd = machine.step(0.2, None, [0.4, 2.0], [2.0, 2.0])
    assert cmd.linear == 0.0 and cmd.angular != 0.0


def test_snapshot_wheels_ages_and_valid_json():
    data = dl.DashboardData()
    data.put('sonar/front_1', float('inf'), 1.0)
    data.put('cmd_vel', (0.2, 1.0), 1.0)
    assert data.snapshot(2.0)['wheels'] == {}  # track width unknown
    data.put('track_width', 0.4, 1.0)
    data.put('odom', (0.1, 0.0), 1.5)
    data.put_detection('{"base_x": NaN, "base_y": 0.1}', 1.0)
    data.put_detection('not json', 1.0)
    data.put_detection('[1]', 1.0)
    snap = data.snapshot(2.0)
    json.dumps(snap, allow_nan=False)
    assert snap['sonar/front_1'] == {'value': 'inf', 'age': 1.0}
    assert snap['wheels']['target']['left'] == pytest.approx(0.0)
    assert snap['wheels']['target']['right'] == pytest.approx(0.4)
    assert snap['wheels']['measured']['age'] == 0.5
    assert snap['detection']['value']['base_y'] == 0.1


def test_stop_history_records_changes_only():
    data = dl.DashboardData()
    reasons = ['', 'accel_limited', 'sonars_stale', 'sonars_stale', '',
               'front_hard_stop', 'clamped']
    for t, reason in enumerate(reasons):
        data.put_safety(reason, float(t))
    stops = data.snapshot(10.0)['stops']
    assert [s['reason'] for s in stops] == ['front_hard_stop', 'sonars_stale']
    for t in range(100):
        data.put_safety('sonars_stale' if t % 2 else '', float(t))
    assert len(data.snapshot(100.0)['stops']) == dl.STOP_HISTORY


def test_page_and_state_are_served_read_only():
    app = FakeApp(allow_write=False)
    status, kind, page = dl.handle_request(
        app, 'GET', '/', HEADERS, b'', '127.0.0.1')
    assert status == 200 and kind.startswith('text/html')
    assert b'/api/state' in page and b'innerHTML' not in page
    status, _, body = dl.handle_request(
        app, 'GET', '/api/state?x=1', HEADERS, b'', '10.0.0.2')
    state = json.loads(body)
    assert status == 200 and state['allow_write'] is False
    assert set(state['tunables']) == set(tu.TUNABLES)
    status, reply = post(app, b'{"approach_speed": 0.1}')
    assert status == 403 and app.written == []


def test_write_accepts_good_and_refuses_bad_requests():
    app = FakeApp()
    assert post(app, b'{"approach_speed": 0.1}') == (200, {'message': 'set'})
    assert app.written == [{'approach_speed': 0.1}]
    for body in (b'{"approach_speed": 5}', b'{"approach_speed": NaN}',
                 b'{"enabled": 1}', b'[1]', b'{}', b'oops', b'\xff'):
        assert post(app, body)[0] == 400, body
    plain = dict(HEADERS, **{'content-type': 'text/plain'})
    assert post(app, b'{"approach_speed": 0.1}', plain)[0] == 415
    rebound = dict(HEADERS, host='evil.example.com')
    assert post(app, b'{"approach_speed": 0.1}', rebound)[0] == 403
    assert post(app, b'{"approach_speed": 0.1}', client='8.8.8.8')[0] == 403
    assert len(app.written) == 1
    assert post(FakeApp(ok=False), b'{"approach_speed": 0.1}')[0] == 502


def test_only_local_addresses_can_be_bound():
    for host in ('0.0.0.0', '8.8.8.8', 'example.com', ''):
        with pytest.raises(ValueError):
            dl.make_server(FakeApp(), host, 0)
    assert dl.is_local_address(dl.default_host())


def test_real_server_round_trip():
    app = FakeApp()
    server = dl.make_server(app, '127.0.0.1', 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = 'http://127.0.0.1:%d' % server.server_address[1]
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(url + '/api/state', timeout=5) as r:
            assert json.load(r)['allow_write'] is True
        request = urllib.request.Request(
            url + '/api/params', data=b'{"slow_distance": 0.8}',
            headers={'Content-Type': 'application/json'})
        with opener.open(request, timeout=5) as r:
            assert r.status == 200
        request.data = b'x' * (dl.MAX_BODY + 1)
        with pytest.raises((urllib.error.URLError, ConnectionError)):
            opener.open(request, timeout=5)
    finally:
        server.shutdown()
        server.server_close()
    assert app.written == [{'slow_distance': 0.8}]


def test_command_line_defaults_to_read_only():
    options = dash.parse_args([])
    assert options.allow_write is False and options.port == 8080
    assert dash.parse_args(['--allow-write']).allow_write


def test_behavior_node_publishes_safety_and_checks_live_changes():
    path = os.path.join(os.path.dirname(dl.__file__), 'behavior_node.py')
    with open(path) as f:
        text = f.read()
    ast.parse(text)
    assert "'behavior/safety'" in text
    assert 'add_on_set_parameters_callback' in text
