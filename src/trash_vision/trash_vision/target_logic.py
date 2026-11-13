"""ROS-free target maths for the trash detector (unit testable)."""


def get_direction(horizontal_error: float, deadband: float) -> str:
    """Return LEFT, RIGHT or CENTER for a normalized horizontal error."""
    if horizontal_error < -deadband:
        return "LEFT"
    if horizontal_error > deadband:
        return "RIGHT"
    return "CENTER"


def build_target(
    bbox: tuple,
    image_size: tuple,
    confidence: float,
    frame_number: int,
    deadband: float,
) -> dict:
    """Build the /vision/target dict from a pixel bbox (x1, y1, x2, y2)."""
    x1, y1, x2, y2 = bbox
    width, height = image_size

    center_x_pixels = (x1 + x2) / 2.0
    center_y_pixels = (y1 + y2) / 2.0
    center_x = center_x_pixels / width
    center_y = center_y_pixels / height
    floor_x = center_x_pixels / width
    floor_y = y2 / height
    horizontal_error = center_x - 0.5

    return {
        "detected": True,
        "frame_number": frame_number,
        "confidence": round(confidence, 4),
        "bbox_pixels": {
            "x1": round(x1, 1),
            "y1": round(y1, 1),
            "x2": round(x2, 1),
            "y2": round(y2, 1),
        },
        "center_normalized": {
            "x": round(center_x, 4),
            "y": round(center_y, 4),
        },
        "floor_point_normalized": {
            "x": round(floor_x, 4),
            "y": round(floor_y, 4),
        },
        "horizontal_error": round(horizontal_error, 4),
        "direction": get_direction(horizontal_error, deadband),
    }
