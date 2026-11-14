import json

from trash_vision import detection_logic as dl


def test_direction_bands():
    assert dl.get_direction(-0.2, 0.12) == "LEFT"
    assert dl.get_direction(0.2, 0.12) == "RIGHT"
    assert dl.get_direction(0.12, 0.12) == "CENTER"
    assert dl.get_direction(0.0, 0.12) == "CENTER"


def test_build_target_values():
    t = dl.build_target(100, 200, 200, 400, 640, 480, 0.8765432, 7, 0.12)
    assert t["detected"] is True
    assert t["source"] == "yolo"
    assert t["frame_number"] == 7
    assert t["confidence"] == 0.8765
    assert t["center_normalized"] == {"x": 0.2344, "y": 0.625}
    assert t["floor_point_normalized"]["y"] == round(400 / 480, 4)
    assert t["direction"] == "LEFT"
    json.dumps(t)


def test_no_target():
    assert dl.build_no_target(3) == {"detected": False, "frame_number": 3}


def test_fallback_decision():
    kw = dict(enabled=True, min_interval=5.0, low_conf_threshold=0.3)
    assert dl.should_use_fallback(0, None, 10.0, 0.0, **kw)
    assert not dl.should_use_fallback(0, None, 3.0, 0.0, **kw)
    assert dl.should_use_fallback(1, 0.2, 10.0, 0.0, **kw)
    assert not dl.should_use_fallback(1, 0.9, 10.0, 0.0, **kw)
    assert not dl.should_use_fallback(
        0, None, 10.0, 0.0, enabled=False, min_interval=5.0,
        low_conf_threshold=0.3)


def test_payload_shape():
    p = dl.build_gpt_payload(dl.encode_jpeg_base64(b"abc"))
    assert p["model"] == "gpt-4o"
    url = p["messages"][0]["content"][1]["image_url"]["url"]
    assert url == "data:image/jpeg;base64,YWJj"


def test_parse_valid_and_fenced():
    raw = '{"found": true, "x1": 0.1, "y1": 0.2, "x2": 0.3, "y2": 0.4, "confidence": 0.9}'
    assert dl.parse_gpt_response(raw)["x2"] == 0.3
    fenced = "```json\n" + raw + "\n```"
    assert dl.parse_gpt_response(fenced)["confidence"] == 0.9


def test_parse_rejects_bad():
    assert dl.parse_gpt_response("") is None
    assert dl.parse_gpt_response("no json") is None
    assert dl.parse_gpt_response('{"found": false}') is None
    assert dl.parse_gpt_response('{"found": true, "x1": 0.5}') is None
    bad = '{"found": true, "x1": 0.5, "y1": 0.1, "x2": 0.4, "y2": 0.9}'
    assert dl.parse_gpt_response(bad) is None
    out = '{"found": true, "x1": 0.1, "y1": 0.1, "x2": 1.5, "y2": 0.9}'
    assert dl.parse_gpt_response(out) is None
