"""Pure-Python 2D kinematic world used by the sim node (no ROS needed)."""
import math

from trash_vision import detection_logic as dl

# Sensor mounting (sim only, NOT real hardware facts): bearing in radians
# relative to robot heading (CCW positive), front_1 = leftmost.
SENSOR_BEARINGS = {
    "front_1": 0.45, "front_2": 0.15, "front_3": -0.15, "front_4": -0.45,
    "rear_1": math.pi - 0.45, "rear_2": math.pi - 0.15,
    "rear_3": math.pi + 0.15, "rear_4": math.pi + 0.45,
}
SENSOR_MAX_RANGE = 2.0
CAMERA_FOV = 1.2          # horizontal field of view, rad (sim only)
CAMERA_MAX_DIST = 3.0     # m
FLOOR_K = 0.1225          # floor row y = 0.5 + FLOOR_K / distance (sim only)
PICK_REACH = 0.40         # m, litter closer than this and in front is pickable


def wrap(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class SimWorld:
    """Robot in a walled rectangle [0, width] x [0, height] with litter."""

    def __init__(self, width=4.0, height=4.0, litter=None,
                 x=2.0, y=2.0, yaw=0.0):
        self.width = float(width)
        self.height = float(height)
        self.litter = [tuple(p) for p in (litter or [])]
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.held = False
        self.collected = 0

    def step(self, v, w, dt):
        """Integrate unicycle motion; walls stop the robot."""
        self.yaw = wrap(self.yaw + w * dt)
        nx = self.x + v * math.cos(self.yaw) * dt
        ny = self.y + v * math.sin(self.yaw) * dt
        margin = 0.05
        self.x = min(max(nx, margin), self.width - margin)
        self.y = min(max(ny, margin), self.height - margin)

    def range_at(self, bearing):
        """Ray distance to the walls along robot-relative bearing."""
        a = self.yaw + bearing
        dx, dy = math.cos(a), math.sin(a)
        best = float("inf")
        if dx > 1e-9:
            best = min(best, (self.width - self.x) / dx)
        elif dx < -1e-9:
            best = min(best, -self.x / dx)
        if dy > 1e-9:
            best = min(best, (self.height - self.y) / dy)
        elif dy < -1e-9:
            best = min(best, -self.y / dy)
        return best if best <= SENSOR_MAX_RANGE else float("inf")

    def ranges(self):
        """Dict sensor name -> distance in metres (inf = nothing seen)."""
        return {n: self.range_at(b) for n, b in SENSOR_BEARINGS.items()}

    def _polar(self, item):
        dx, dy = item[0] - self.x, item[1] - self.y
        return math.hypot(dx, dy), wrap(math.atan2(dy, dx) - self.yaw)

    def nearest_visible(self):
        """(distance, bearing) of nearest litter in camera view or None."""
        best = None
        for item in self.litter:
            d, b = self._polar(item)
            if d <= CAMERA_MAX_DIST and abs(b) <= CAMERA_FOV / 2:
                if best is None or d < best[0]:
                    best = (d, b)
        return best

    def target(self, frame_number, deadband=0.1):
        """Target dict in the /vision/target format."""
        seen = self.nearest_visible()
        if seen is None:
            return dl.build_no_target(frame_number)
        d, b = seen
        width, height = 640.0, 480.0
        cx = (0.5 - b / CAMERA_FOV) * width
        floor_y = min(0.5 + FLOOR_K / max(d, 0.01), 0.99) * height
        half = 20.0
        return dl.build_target(cx - half, floor_y - 2 * half, cx + half,
                               floor_y, width, height, 0.9,
                               frame_number, deadband, source="sim")

    def pick(self):
        """Try to pick litter in reach. Returns (success, message)."""
        if self.held:
            return False, "already holding"
        seen = self.nearest_visible()
        if seen is None or seen[0] > PICK_REACH:
            return False, "nothing in reach"
        for item in list(self.litter):
            d, b = self._polar(item)
            if abs(d - seen[0]) < 1e-9 and abs(b - seen[1]) < 1e-9:
                self.litter.remove(item)
                break
        self.held = True
        return True, "item held"

    def deposit(self):
        """Drop the held item into the bin."""
        if not self.held:
            return False, "nothing held"
        self.held = False
        self.collected += 1
        return True, "deposited"
