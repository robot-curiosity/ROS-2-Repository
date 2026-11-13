"""Pure-Python helpers for the trash detector (no ROS, camera or model needed)."""
import base64
import json
import re

GPT_PROMPT = (
    "You are the vision system of a litter-collecting robot. Find the single "
    "most prominent piece of litter on the ground in this image. Reply with "
    "JSON only: {\"found\": true|false, \"x1\": f, \"y1\": f, \"x2\": f, "
    "\"y2\": f, \"confidence\": f} where coordinates are normalized 0..1 "
    "(origin top-left) and confidence is 0..1."
)


def get_direction(horizontal_error, deadband):
    """Return LEFT / RIGHT / CENTER for a normalized horizontal error."""
    if horizontal_error < -deadband:
        return "LEFT"
    if horizontal_error > deadband:
        return "RIGHT"
    return "CENTER"


def build_target(x1, y1, x2, y2, width, height, confidence,
                 frame_number, deadband, source="yolo"):
    """Build the target dict published on /vision/target (pixel bbox in)."""
    center_x_pixels = (x1 + x2) / 2.0
    center_y_pixels = (y1 + y2) / 2.0
    center_x = center_x_pixels / width
    center_y = center_y_pixels / height
    horizontal_error = center_x - 0.5
    direction = get_direction(horizontal_error, deadband)
    return {
        "detected": True,
        "source": source,
        "frame_number": frame_number,
        "confidence": round(float(confidence), 4),
        "bbox_pixels": {
            "x1": round(x1, 1), "y1": round(y1, 1),
            "x2": round(x2, 1), "y2": round(y2, 1),
        },
        "center_normalized": {"x": round(center_x, 4), "y": round(center_y, 4)},
        "floor_point_normalized": {
            "x": round(center_x, 4), "y": round(y2 / height, 4)},
        "horizontal_error": round(horizontal_error, 4),
        "direction": direction,
    }


def build_no_target(frame_number):
    """Target dict for a frame with nothing detected."""
    return {"detected": False, "frame_number": frame_number}


def should_use_fallback(num_yolo_boxes, best_confidence, now, last_call_time,
                        enabled, min_interval, low_conf_threshold):
    """Decide whether to ask GPT-4o: YOLO saw nothing/weak, rate limited."""
    if not enabled:
        return False
    if now - last_call_time < min_interval:
        return False
    if num_yolo_boxes == 0:
        return True
    return best_confidence is not None and best_confidence < low_conf_threshold


def encode_jpeg_base64(jpeg_bytes):
    """Base64 text of JPEG bytes, for the OpenAI image_url data URI."""
    return base64.b64encode(jpeg_bytes).decode("ascii")


def build_gpt_payload(jpeg_b64, model="gpt-4o"):
    """Chat-completions request body asking GPT-4o to locate litter."""
    return {
        "model": model,
        "max_tokens": 120,
        "temperature": 0,
        "messages": [{
            "role": "user",
            "content": [
                {"type": "text", "text": GPT_PROMPT},
                {"type": "image_url", "image_url": {
                    "url": "data:image/jpeg;base64," + jpeg_b64}},
            ],
        }],
    }


def parse_gpt_response(text):
    """Parse GPT reply into a normalized box dict, or None if no/invalid litter.

    Accepts bare JSON or JSON wrapped in a markdown code fence / extra prose.
    Rejects boxes outside 0..1 or with non-positive size.
    """
    if not text:
        return None
    match = re.search(r"\{.*\}", text, re.DOTALL)
    if match is None:
        return None
    try:
        data = json.loads(match.group(0))
    except ValueError:
        return None
    if not isinstance(data, dict) or data.get("found") is not True:
        return None
    try:
        x1, y1, x2, y2 = (float(data[k]) for k in ("x1", "y1", "x2", "y2"))
        confidence = float(data.get("confidence", 0.5))
    except (KeyError, TypeError, ValueError):
        return None
    if not all(0.0 <= v <= 1.0 for v in (x1, y1, x2, y2)):
        return None
    if x2 <= x1 or y2 <= y1:
        return None
    confidence = min(max(confidence, 0.0), 1.0)
    return {"x1": x1, "y1": y1, "x2": x2, "y2": y2, "confidence": confidence}
