"""Odometry-driven coverage search (lawnmower rows) and a coverage grid.

Pure Python, no ROS. Poses are (x, y, yaw) in the odometry frame (the
robot's start pose is the origin), metres and radians. The search area is a
rectangle centred on the start pose. All sizes are working defaults, not
measurements: TODO: Bryan to set the real area for the test field.
"""
import math

DEFAULTS = {
    # "lawnmower" (odometry rows) or "spin" (old behavior, no odometry).
    'search_pattern': 'lawnmower',
    'search_area_length': 4.0,   # m along the start heading; TODO: Bryan
    'search_area_width': 4.0,    # m across the start heading; TODO: Bryan
    'search_edge_margin': 0.3,   # m kept clear of the area edge
    # 0 = derive from the camera field of view (see row_spacing_from_fov).
    'search_row_spacing': 0.0,
    'search_detect_range': 1.5,  # m at which a can is reliably seen; guess
    'search_row_overlap': 0.2,   # fraction of overlap between sweeps
    # from CAD V16-18 per STATE.md, not measured:
    'search_camera_hfov_deg': 66.0,
    'search_speed': 0.2,         # m/s along a row
    'search_waypoint_tolerance': 0.15,  # m, waypoint counts as reached
    'search_stuck_timeout': 10.0,  # s without progress, skip the waypoint
}


def row_spacing_from_fov(hfov_deg, detect_range, overlap):
    """Distance between rows so neighbouring camera sweeps overlap a bit."""
    width = 2.0 * detect_range * math.tan(math.radians(hfov_deg) / 2.0)
    return max(0.2, width * (1.0 - min(max(overlap, 0.0), 0.9)))


def wrap(angle):
    """Wrap an angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def build_waypoints(start, length, width, spacing, margin):
    """Boustrophedon waypoints for rows along x, stepping across in y.

    start is the robot's (x, y); rows begin at the corner nearest to it. The
    area is centred on the odometry origin, shrunk by margin on every side.
    """
    half_l = max(length / 2.0 - margin, 0.0)
    half_w = max(width / 2.0 - margin, 0.0)
    n_rows = max(2, int(math.ceil(2 * half_w / spacing)) + 1) \
        if half_w > 0 else 1
    ys = [-half_w + i * (2 * half_w / (n_rows - 1)) for i in range(n_rows)] \
        if n_rows > 1 else [0.0]
    if abs(start[1] - ys[-1]) < abs(start[1] - ys[0]):
        ys.reverse()
    first_x = -half_l if abs(start[0] + half_l) <= abs(start[0] - half_l) \
        else half_l
    points = []
    x_dir = first_x
    for y in ys:
        other = -x_dir
        points.append((x_dir, y))
        points.append((other, y))
        x_dir = other
    return points


class LawnmowerSearch:
    """Drive through the waypoints; skip one that makes no progress."""

    def __init__(self, params=None):
        """Create the search from optional parameter overrides."""
        self.p = dict(DEFAULTS)
        if params:
            self.p.update({k: v for k, v in params.items() if k in DEFAULTS})
        self.waypoints = []
        self.index = 0
        self.laps = 0
        self.skipped = 0
        self._best_dist = math.inf
        self._progress_time = None
        self._blocked_since = None
        self._last_step = -math.inf

    def spacing(self):
        """Return the row spacing in metres."""
        p = self.p
        if p['search_row_spacing'] > 0:
            return float(p['search_row_spacing'])
        return row_spacing_from_fov(
            p['search_camera_hfov_deg'], p['search_detect_range'],
            p['search_row_overlap'])

    def _ensure(self, pose):
        if self.waypoints:
            return
        p = self.p
        self.waypoints = build_waypoints(
            (pose[0], pose[1]), p['search_area_length'],
            p['search_area_width'], self.spacing(), p['search_edge_margin'])
        self.index = 0
        self._best_dist = math.inf
        self._progress_time = None

    def _next(self):
        self.index += 1
        if self.index >= len(self.waypoints):
            self.index = 0
            self.laps += 1
        self._best_dist = math.inf
        self._progress_time = None

    @property
    def target(self):
        """Return the current waypoint, or None if there are none."""
        return self.waypoints[self.index] if self.waypoints else None

    def step(self, pose, now):
        """Return (linear, angular) toward the current waypoint."""
        self._ensure(pose)
        p = self.p
        self._blocked_since = None
        if now - self._last_step > 1.0:
            # Away from the pattern (avoiding, picking): restart the timer.
            self._best_dist = math.inf
            self._progress_time = now
        self._last_step = now
        wx, wy = self.waypoints[self.index]
        dx, dy = wx - pose[0], wy - pose[1]
        dist = math.hypot(dx, dy)
        if dist <= p['search_waypoint_tolerance']:
            self._next()
            return 0.0, 0.0
        if self._progress_time is None:
            self._progress_time = now
        if dist < self._best_dist - 0.05:
            self._best_dist = dist
            self._progress_time = now
        elif now - self._progress_time > p['search_stuck_timeout']:
            self.skipped += 1
            self._next()
            return 0.0, 0.0
        err = wrap(math.atan2(dy, dx) - pose[2])
        speed = float(p['search_speed'])
        angular = max(-0.8, min(0.8, 1.5 * err))
        if abs(err) > 0.5:
            return 0.0, angular if angular else 0.3
        return speed * max(0.0, math.cos(err)), angular

    def note_blocked(self, now):
        """Front is blocked while searching; skip a waypoint if it persists."""
        if not self.waypoints:
            return
        if self._blocked_since is None:
            self._blocked_since = now
        elif now - self._blocked_since > self.p['search_stuck_timeout']:
            self.skipped += 1
            self._next()
            self._blocked_since = None

    def reset_progress(self, now):
        """Call after the robot was busy elsewhere (pick, approach)."""
        self._best_dist = math.inf
        self._progress_time = now


class CoverageGrid:
    """Fraction of a rectangle the camera has swept (for tests and logs)."""

    def __init__(self, x_min, x_max, y_min, y_max, cell=0.1):
        """Create an empty grid over the given area with square cells."""
        self.x_min, self.y_min, self.cell = x_min, y_min, cell
        self.nx = max(1, int(math.ceil((x_max - x_min) / cell)))
        self.ny = max(1, int(math.ceil((y_max - y_min) / cell)))
        self._seen = set()

    def mark(self, pose, detect_range, hfov_deg):
        """Mark cells inside the camera's view wedge as covered."""
        half = math.radians(hfov_deg) / 2.0
        reach = int(detect_range / self.cell) + 1
        ci = int((pose[0] - self.x_min) / self.cell)
        cj = int((pose[1] - self.y_min) / self.cell)
        for i in range(max(0, ci - reach), min(self.nx, ci + reach + 1)):
            for j in range(max(0, cj - reach), min(self.ny, cj + reach + 1)):
                cx = self.x_min + (i + 0.5) * self.cell - pose[0]
                cy = self.y_min + (j + 0.5) * self.cell - pose[1]
                d = math.hypot(cx, cy)
                if d > detect_range:
                    continue
                if d < 0.05 or abs(wrap(math.atan2(cy, cx) - pose[2])) <= half:
                    self._seen.add((i, j))

    def fraction(self):
        """Return the fraction of cells visited, 0..1."""
        return len(self._seen) / float(self.nx * self.ny)
