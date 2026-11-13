from trash_vision.target_logic import build_target, get_direction


def test_direction_left_right_center():
    assert get_direction(-0.2, 0.12) == "LEFT"
    assert get_direction(0.2, 0.12) == "RIGHT"
    assert get_direction(0.0, 0.12) == "CENTER"
    assert get_direction(0.12, 0.12) == "CENTER"
    assert get_direction(-0.12, 0.12) == "CENTER"


def test_build_target_centered():
    t = build_target((300, 200, 340, 280), (640, 480), 0.9, 7, 0.12)
    assert t["detected"] is True
    assert t["frame_number"] == 7
    assert t["horizontal_error"] == 0.0
    assert t["direction"] == "CENTER"
    assert t["center_normalized"] == {"x": 0.5, "y": 0.5}
    assert t["floor_point_normalized"]["y"] == round(280 / 480, 4)


def test_build_target_right_of_center():
    t = build_target((500, 100, 600, 200), (640, 480), 0.5, 1, 0.12)
    assert t["direction"] == "RIGHT"
    assert t["horizontal_error"] > 0.12
