"""Data store and web request handling for the tuning dashboard.

Pure Python standard library, no ROS. ``dashboard.py`` feeds the store from
topics and passes parameter writes to the behavior node.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
import socket
import threading

from trash_vision import tuning_logic as tu
from trash_vision.dashboard_page import PAGE

MAX_BODY = 4096
STOP_HISTORY = 20
# Safety reasons that only shape a command; everything else is a stop.
NOT_STOPS = ('', 'clamped', 'accel_limited')


def wheel_speeds(linear, angular, track_width):
    """Return (left, right) wheel speed in m/s, or None without a track."""
    if not track_width or track_width <= 0:
        return None
    half = angular * track_width / 2.0
    return linear - half, linear + half


def json_safe(value):
    """Replace inf/NaN (not valid JSON) with strings, recursively."""
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


class DashboardData:
    """Thread-safe latest values, each with the time it arrived."""

    def __init__(self):
        """Create an empty store."""
        self._lock = threading.Lock()
        self._values = {}
        self._stops = []
        self._last_reason = ''

    def put(self, key, value, now):
        """Store the latest value for key."""
        with self._lock:
            self._values[key] = (value, now)

    def put_detection(self, text, now):
        """Store a /vision/target JSON message; bad text is ignored."""
        try:
            data = json.loads(text)
        except ValueError:
            return
        if isinstance(data, dict):
            self.put('detection', data, now)

    def put_safety(self, reason, now):
        """Store the safety reason and remember each new stop."""
        with self._lock:
            self._values['safety'] = (reason, now)
            if reason != self._last_reason and reason not in NOT_STOPS:
                self._stops.append((reason, now))
                del self._stops[:-STOP_HISTORY]
            self._last_reason = reason

    def snapshot(self, now):
        """Return a JSON-safe dict: values with ages, wheels and stops."""
        with self._lock:
            values = dict(self._values)
            stops = list(self._stops)
        out = {
            key: {'value': value, 'age': round(max(0.0, now - stamp), 2)}
            for key, (value, stamp) in values.items()}
        track = values.get('track_width', (None, 0))[0]
        wheels = {}
        for label, key in (('target', 'cmd_vel'), ('measured', 'odom')):
            if key not in values:
                continue
            speeds = wheel_speeds(*values[key][0], track)
            if speeds:
                wheels[label] = {
                    'left': speeds[0], 'right': speeds[1],
                    'age': out[key]['age']}
        out['wheels'] = wheels
        out['stops'] = [
            {'reason': r, 'age': round(max(0.0, now - t), 1)}
            for r, t in reversed(stops)]
        return json_safe(out)


def is_local_address(text):
    """Return True for a private, link-local or loopback IP address."""
    try:
        ip = ipaddress.ip_address(text)
    except ValueError:
        return False
    if ip.is_unspecified:
        return False
    return ip.is_private or ip.is_loopback or ip.is_link_local


def default_host():
    """Return this machine's LAN address, or loopback if there is none."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            # Connecting a UDP socket sends nothing; it only picks a route.
            sock.connect(('10.255.255.255', 1))
            host = sock.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    return host if is_local_address(host) else '127.0.0.1'


def host_header_ok(header):
    """Accept IP, localhost, *.local or this host name (no DNS rebinding)."""
    if not header:
        return False
    name = header.strip().lower()
    if name.startswith('['):
        name = name[1:].split(']')[0]
    elif name.count(':') == 1:
        name = name.split(':')[0]
    if name in ('localhost', socket.gethostname().lower()):
        return True
    if name.endswith('.local'):
        return True
    return is_local_address(name)


def _reply(status, body):
    return status, 'application/json', json.dumps(body).encode()


def handle_request(app, method, path, headers, body, client_ip):
    """Answer one HTTP request: return (status, content type, bytes).

    app has ``allow_write`` (bool), ``snapshot()`` returning a dict and
    ``write(changes)`` returning (ok, message). headers keys are lower case.
    """
    if not is_local_address(client_ip):
        return _reply(403, {'error': 'local network only'})
    if not host_header_ok(headers.get('host')):
        return _reply(403, {'error': 'unexpected Host header'})
    path = path.split('?')[0]
    if method == 'GET' and path == '/':
        return 200, 'text/html; charset=utf-8', PAGE.encode()
    if method == 'GET' and path == '/api/state':
        state = app.snapshot()
        state['allow_write'] = app.allow_write
        state['tunables'] = {
            name: dict(zip(('min', 'max', 'step', 'unit', 'label'), spec))
            for name, spec in tu.TUNABLES.items()}
        return _reply(200, state)
    if method == 'POST' and path == '/api/params':
        if not app.allow_write:
            return _reply(403, {'error': 'read-only: start with --allow-write'})
        # A JSON content type cannot be sent cross-site without permission.
        kind = headers.get('content-type', '').split(';')[0].strip()
        if kind != 'application/json':
            return _reply(415, {'error': 'send application/json'})
        try:
            changes = json.loads(body.decode())
        except (UnicodeDecodeError, ValueError):
            return _reply(400, {'error': 'bad JSON'})
        if not isinstance(changes, dict) or not changes:
            return _reply(400, {'error': 'send {"name": value}'})
        params = app.snapshot().get('params', {}).get('value') or {}
        errors = tu.validate_changes(changes, params)
        if errors:
            return _reply(400, {'error': '; '.join(errors)})
        ok, message = app.write(
            {name: float(value) for name, value in changes.items()})
        return _reply(200 if ok else 502,
                      {'message': message} if ok else {'error': message})
    return _reply(404, {'error': 'not found'})


def make_server(app, host, port):
    """Create the HTTP server; host must be a local-network address."""
    if not is_local_address(host):
        raise ValueError(
            'host %r is not a local network address (0.0.0.0 is refused)'
            % host)

    class Handler(BaseHTTPRequestHandler):
        timeout = 5

        def _serve(self):
            headers = {k.lower(): v for k, v in self.headers.items()}
            try:
                length = int(headers.get('content-length', 0))
            except ValueError:
                length = -1
            if not 0 <= length <= MAX_BODY:
                status, kind, data = _reply(413, {'error': 'body too large'})
            else:
                body = self.rfile.read(length) if length else b''
                status, kind, data = handle_request(
                    app, self.command, self.path, headers, body,
                    self.client_address[0])
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        do_GET = _serve  # noqa: N815
        do_POST = _serve  # noqa: N815

        def log_message(self, *args):
            """Keep the once-per-second polling out of the terminal."""

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
