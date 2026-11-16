"""Pure-Python 2D kinematic world used by the sim node (no ROS needed)."""
import math

from trash_vision import detection_logic as dl

# Sensor mounting (sim only, NOT real hardware facts; layout per TODO.md,
# "working assumption until Bryan sends exact positions"): name ->
# (forward offset m, left offset m, bearing rad CCW from robot heading).
# Side sensors are staggered fore/aft by an assumed +-0.10 m.
SENSOR_MOUNT = {
    "front_1": (0.0, 0.0, 0.20), "front_2": (0.0, 0.0, -0.20),
    "rear_1": (0.0, 0.0, math.pi - 0.20),
    "rear_2": (0.0, 0.0, math.pi + 0.20),
    "left_1": (0.10, 0.0, math.pi / 2), "left_2": (-0.10, 0.0, math.pi / 2),
    "right_1": (0.10, 0.0, -math.pi / 2),
    "right_2": (-0.10, 0.0, -math.pi / 2),
}
SENSOR_BEARINGS = {n: m[2] for n, m in SENSOR_MOUNT.items()}
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

    def range_at(self, bearing, forward=0.0, left=0.0):
        """Ray distance to the walls along robot-relative bearing.

        forward/left give the sensor's offset from the robot centre.
        """
        a = self.yaw + bearing
        dx, dy = math.cos(a), math.sin(a)
        ox = self.x + forward * math.cos(self.yaw) - left * math.sin(self.yaw)
        oy = self.y + forward * math.sin(self.yaw) + left * math.cos(self.yaw)
        best = float("inf")
        if dx > 1e-9:
            best = min(best, (self.width - ox) / dx)
        elif dx < -1e-9:
            best = min(best, -ox / dx)
        if dy > 1e-9:
            best = min(best, (self.height - oy) / dy)
        elif dy < -1e-9:
            best = min(best, -oy / dy)
        return best if best <= SENSOR_MAX_RANGE else float("inf")

    def ranges(self):
        """Dict sensor name -> distance in metres (inf = nothing seen)."""
        return {n: self.range_at(b, f, lf)
                for n, (f, lf, b) in SENSOR_MOUNT.items()}

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
