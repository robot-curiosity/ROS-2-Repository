from trash_vision.detection_filter import (
    Candidate, DetectionFilter, in_region, parse_regions)


def cand(x=0.5, y=0.5, conf=0.8, dist=None, size=0.05):
    return Candidate(x - size, y - size, x + size, y + size, conf, dist)


def test_needs_n_of_m_frames():
    f = DetectionFilter(confirm_frames=3, window_frames=5)
    assert f.update([cand()]) is None
    assert f.update([cand()]) is None
    assert f.update([cand()]) is not None


def test_single_frame_flicker_never_reported():
    f = DetectionFilter(confirm_frames=3, window_frames=5)
    for i in range(10):
        got = f.update([cand()] if i % 4 == 0 else [])
        assert got is None


def test_survives_one_missed_frame():
    f = DetectionFilter(confirm_frames=3, window_frames=5)
    f.update([cand()])
    f.update([cand()])
    assert f.update([cand()]) is not None
    assert f.update([]) is None  # not seen now, nothing reported
    assert f.update([cand()]) is not None  # still 4 of last 5


def test_track_expires_after_window():
    f = DetectionFilter(confirm_frames=2, window_frames=3)
    f.update([cand()])
    f.update([cand()])
    for _ in range(3):
        f.update([])
    assert f.update([cand()]) is None


def test_moving_target_keeps_track():
    f = DetectionFilter(confirm_frames=3, window_frames=5, match_radius=0.1)
    f.update([cand(x=0.50)])
    f.update([cand(x=0.55)])
    assert f.update([cand(x=0.60)]) is not None


def test_jump_beyond_radius_is_new_track():
    f = DetectionFilter(confirm_frames=2, window_frames=3, match_radius=0.1)
    f.update([cand(x=0.2)])
    assert f.update([cand(x=0.8)]) is None


def test_min_confidence():
    f = DetectionFilter(min_confidence=0.5, confirm_frames=1,
                        window_frames=1)
    assert f.update([cand(conf=0.49)]) is None
    assert f.update([cand(conf=0.5)]) is not None


def test_ignore_region():
    regions = parse_regions([0.0, 0.8, 1.0, 1.0])
    f = DetectionFilter(confirm_frames=1, window_frames=1,
                        ignore_regions=regions)
    assert f.update([cand(y=0.9)]) is None
    assert f.update([cand(y=0.5)]) is not None


def test_parse_regions_normalizes_and_drops_partial():
    assert parse_regions([0.5, 0.9, 0.1, 0.2, 0.3]) == [(0.1, 0.2, 0.5, 0.9)]
    assert parse_regions([]) == []


def test_in_region_uses_center():
    c = Candidate(0.0, 0.0, 0.4, 0.4, 0.9)
    assert in_region(c, [(0.1, 0.1, 0.3, 0.3)])
    assert not in_region(c, [(0.5, 0.5, 1.0, 1.0)])


def test_best_by_distance_then_confidence():
    f = DetectionFilter(confirm_frames=1, window_frames=1)
    far = cand(x=0.2, dist=1.5, conf=0.95)
    near = cand(x=0.8, dist=0.6, conf=0.5)
    assert f.update([far, near]) is near
    f.reset()
    a = cand(x=0.2, dist=1.0, conf=0.6)
    b = cand(x=0.8, dist=1.0, conf=0.9)
    assert f.update([a, b]) is b


def test_unknown_distance_ranks_last():
    f = DetectionFilter(confirm_frames=1, window_frames=1)
    unknown = cand(x=0.2, dist=None, conf=0.99)
    known = cand(x=0.8, dist=2.0, conf=0.4)
    assert f.update([unknown, known]) is known


def test_unconfirmed_closer_target_does_not_win():
    f = DetectionFilter(confirm_frames=2, window_frames=3)
    old = cand(x=0.2, dist=2.0)
    f.update([old])
    got = f.update([old, cand(x=0.8, dist=0.5)])
    assert got is old


def test_confirm_clamped_to_window():
    f = DetectionFilter(confirm_frames=9, window_frames=2)
    f.update([cand()])
    assert f.update([cand()]) is not None


def test_payload_returned():
    f = DetectionFilter(confirm_frames=1, window_frames=1)
    c = cand()
    c.payload = 7
    assert f.update([c]).payload == 7
