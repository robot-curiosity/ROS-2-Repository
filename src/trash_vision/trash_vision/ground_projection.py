"""Ground-plane projection: normalized image floor point -> base_link metres.

Pure Python (no ROS, camera or model). Ported from the formula in
REPO-REVIEW.md on branch docs/consolidated-state.

Frame: base_link, x forward, y left. Normalized image (u, v) runs 0..1 from
the top-left. The camera is pitched down by ``pitch_rad`` and has no yaw/roll.
Defaults are from CAD V16-18 per STATE.md, not measured (camera height is
0.582 m above base_link + 0.060 m wheel radius). The vertical field of view
(41 deg) is the standard Camera Module 3 lens guess: check against the stream.
"""
from dataclasses import dataclass
from math import cos, radians, sin, tan


@dataclass(frozen=True)
class CameraModel:
    """Camera pose and field of view (from CAD, not measured)."""

    x: float = 0.248            # m ahead of base_link
    y: float = -0.1575          # m left of base_link (negative = right)
    height: float = 0.642       # m above the ground
    pitch_rad: float = radians(35.0)   # down from horizontal
    hfov_rad: float = radians(66.0)
    vfov_rad: float = radians(41.0)


def camera_from_degrees(x, y, height, pitch_deg, hfov_deg, vfov_deg):
    """Build a CameraModel from parameters given in degrees."""
    return CameraModel(float(x), float(y), float(height),
                       radians(pitch_deg), radians(hfov_deg),
                       radians(vfov_deg))


def floor_point_to_base(u, v, cam=None):
    """Project a normalized image point on the ground to (x, y) in base_link.

    Returns None if the point is at or above the horizon, or the camera
    model is invalid.
    """
    cam = cam or CameraModel()
    if cam.height <= 0.0:
        return None
    right = (u - 0.5) * 2.0 * tan(cam.hfov_rad / 2.0)   # right of axis
    below = (v - 0.5) * 2.0 * tan(cam.vfov_rad / 2.0)   # below axis
    forward = cos(cam.pitch_rad) - below * sin(cam.pitch_rad)
    down = sin(cam.pitch_rad) + below * cos(cam.pitch_rad)
    if down <= 1e-9:
        return None
    scale = cam.height / down
    return cam.x + scale * forward, cam.y - scale * right
