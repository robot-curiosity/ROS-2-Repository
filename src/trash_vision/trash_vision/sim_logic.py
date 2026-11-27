"""Pure-Python 2D kinematic world used by the sim node (no ROS needed)."""
from dataclasses import dataclass, field
import math
import random

from trash_vision import arm_logic as al
from trash_vision import detection_logic as dl

# Sensor mounting (sim only, NOT real hardware facts; layout per TODO.md,
# "working assumption until Bryan sends exact positions"): name ->
# (forward offset m, left offset m, bearing rad CCW from robot heading).
# Side sensors are staggered fore/aft by an assumed +-0.10 m.
SENSOR_MOUNT = {
    'front_1': (0.0, 0.0, 0.20), 'front_2': (0.0, 0.0, -0.20),
    'rear_1': (0.0, 0.0, math.pi - 0.20),
    'rear_2': (0.0, 0.0, math.pi + 0.20),
    'left_1': (0.10, 0.0, math.pi / 2), 'left_2': (-0.10, 0.0, math.pi / 2),
    'right_1': (0.10, 0.0, -math.pi / 2),
    'right_2': (-0.10, 0.0, -math.pi / 2),
}
SENSOR_BEARINGS = {n: m[2] for n, m in SENSOR_MOUNT.items()}
SENSOR_MAX_RANGE = 2.0
CAMERA_FOV = 1.2          # horizontal field of view, rad (sim only)
CAMERA_MAX_DIST = 3.0     # m
FLOOR_K = 0.1225          # floor row y = 0.5 + FLOOR_K / distance (sim only)
PICK_REACH = 0.40         # m, litter closer than this and in front is pickable


DEFAULT_LITTER_TYPE = 'can'


def _default_grasp_prob():
    # Sim-only guesses, NOT measured on the real gripper.
    return {'can': 0.9, 'bottle': 0.85, 'wrapper': 0.7}


@dataclass(frozen=True)
class SimNoise:
    """Seeded randomness for the sim. All values are sim assumptions.

    None of these are measured on the real robot; they only make the sim
    harder than the ideal one. The same seed gives the same run.
    """

    seed: int = 0
    grasp_prob: dict = field(default_factory=_default_grasp_prob)
    default_grasp_prob: float = 0.8   # litter type not in grasp_prob
    detect_dropout_prob: float = 0.10   # per frame, real target missed
    false_positive_prob: float = 0.01   # per frame, phantom target
    sonar_sigma: float = 0.01   # m, gaussian noise on a reading
    sonar_dropout_prob: float = 0.02   # per reading, no echo (inf)


def litter_type(item):
    """Litter type of an item tuple (x, y[, type])."""
    return item[2] if len(item) > 2 else DEFAULT_LITTER_TYPE


def wrap(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class SimWorld:
    """Robot in a walled rectangle [0, width] x [0, height] with litter."""

    def __init__(self, width=4.0, height=4.0, litter=None,
                 x=2.0, y=2.0, yaw=0.0, noise=None, arm=None):
        """Create a world.

        noise: a SimNoise, or None for the ideal sensors and grasps.
        arm: an arm_logic.PickConfig; when given, a pick only works if the
        litter is in the pick zone and arm IK finds a solution.
        """
        self.width = float(width)
        self.height = float(height)
        self.litter = [tuple(p) for p in (litter or [])]
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.held = False
        self.held_type = None
        self.collected = 0
        self.collected_by_type = {}
        self.grasp_stats = {}   # type -> [grasp attempts in zone, successes]
        self.noise = noise
        self.arm = arm
        self._rng = random.Random(noise.seed if noise else 0)

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
        best = float('inf')
        if dx > 1e-9:
            best = min(best, (self.width - ox) / dx)
        elif dx < -1e-9:
            best = min(best, -ox / dx)
        if dy > 1e-9:
            best = min(best, (self.height - oy) / dy)
        elif dy < -1e-9:
            best = min(best, -oy / dy)
        return best if best <= SENSOR_MAX_RANGE else float('inf')

    def ranges(self):
        """Return a dict of sensor name -> distance in metres (inf = nothing seen).

        With noise set, readings get gaussian noise and random dropouts.
        """
        out = {}
        for n, (f, lf, b) in SENSOR_MOUNT.items():
            out[n] = self._noisy_range(self.range_at(b, f, lf))
        return out

    def _noisy_range(self, value):
        nz = self.noise
        if nz is None:
            return value
        if self._rng.random() < nz.sonar_dropout_prob:
            return float('inf')
        if value == float('inf'):
            return value
        return max(0.02, value + self._rng.gauss(0.0, nz.sonar_sigma))

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
        nz = self.noise
        if nz is not None:
            if seen is not None and self._rng.random() < nz.detect_dropout_prob:
                seen = None
            if seen is None and self._rng.random() < nz.false_positive_prob:
                seen = (self._rng.uniform(0.5, CAMERA_MAX_DIST),
                        self._rng.uniform(-CAMERA_FOV / 2, CAMERA_FOV / 2))
        if seen is None:
            return dl.build_no_target(frame_number)
        d, b = seen
        width, height = 640.0, 480.0
        cx = (0.5 - b / CAMERA_FOV) * width
        floor_y = min(0.5 + FLOOR_K / max(d, 0.01), 0.99) * height
        half = 20.0
        return dl.build_target(cx - half, floor_y - 2 * half, cx + half,
                               floor_y, width, height, 0.9,
                               frame_number, deadband, source='sim')

    def pick(self):
        """Try to pick litter in reach. Returns (success, message)."""
        if self.held:
            return False, 'already holding'
        seen = self.nearest_visible()
        if seen is None:
            return False, 'nothing in reach'
        if self.arm is None and seen[0] > PICK_REACH:
            return False, 'nothing in reach'
        item = None
        for cand in self.litter:
            d, b = self._polar(cand)
            if abs(d - seen[0]) < 1e-9 and abs(b - seen[1]) < 1e-9:
                item = cand
                break
        if item is None:
            return False, 'nothing in reach'
        if self.arm is not None:
            d, b = seen
            poses, reason = al.plan_pick_poses(
                d * math.cos(b), d * math.sin(b), self.arm)
            if poses is None:
                return False, reason
        if self.noise is not None:
            nz = self.noise
            prob = nz.grasp_prob.get(litter_type(item),
                                     nz.default_grasp_prob)
            stat = self.grasp_stats.setdefault(litter_type(item), [0, 0])
            stat[0] += 1
            if self._rng.random() >= prob:
                return False, 'grasp failed'
            stat[1] += 1
        self.litter.remove(item)
        self.held = True
        self.held_type = litter_type(item)
        return True, 'item held'

    def deposit(self):
        """Drop the held item into the bin."""
        if not self.held:
            return False, 'nothing held'
        self.held = False
        self.collected += 1
        kind = self.held_type or DEFAULT_LITTER_TYPE
        self.collected_by_type[kind] = self.collected_by_type.get(kind, 0) + 1
        self.held_type = None
        return True, 'deposited'
