import time

from trash_vision import frame_grabber as fg


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_take_returns_newest_only_once():
    clock = Clock()
    latest = fg.LatestFrame(clock)
    assert latest.take() is None
    latest.put("f1")
    clock.t = 0.2
    frame, seq, age = latest.take()
    assert (frame, seq) == ("f1", 1)
    assert abs(age - 0.2) < 1e-9
    assert latest.take() is None


def test_old_frames_are_dropped_not_queued():
    latest = fg.LatestFrame(Clock())
    for name in ("a", "b", "c"):
        latest.put(name)
    frame, seq, _ = latest.take()
    assert frame == "c" and seq == 3
    assert latest.dropped == 2
    assert latest.take() is None


def test_taken_frame_replaced_is_not_counted_dropped():
    latest = fg.LatestFrame(Clock())
    latest.put("a")
    latest.take()
    latest.put("b")
    assert latest.dropped == 0


def test_run_once_stores_good_frames_and_counts_failures():
    results = iter([(True, "x"), (False, None), (True, None)])
    grabber = fg.FrameGrabber(lambda: next(results))
    assert grabber.run_once() is True
    assert grabber.run_once() is False
    assert grabber.run_once() is False
    assert grabber.failures == 2
    assert grabber.latest.take()[0] == "x"


def test_read_exception_is_a_failure_not_a_crash():
    def boom():
        raise OSError("camera gone")
    grabber = fg.FrameGrabber(boom)
    assert grabber.run_once() is False
    assert grabber.failures == 1


def test_thread_keeps_newest_frame_while_consumer_is_slow():
    counter = {"n": 0}

    def read():
        counter["n"] += 1
        time.sleep(0.001)
        return True, counter["n"]

    grabber = fg.FrameGrabber(read)
    grabber.start()
    time.sleep(0.15)
    got = grabber.latest.take()
    grabber.stop()
    assert got is not None
    # The consumer was slow: it gets a recent frame, not frame 1.
    assert got[0] > 5
    assert grabber.latest.dropped > 0


def test_thread_survives_read_failures():
    state = {"n": 0}

    def read():
        state["n"] += 1
        if state["n"] < 3:
            return False, None
        return True, state["n"]

    grabber = fg.FrameGrabber(read, retry_delay_s=0.001)
    grabber.start()
    time.sleep(0.1)
    grabber.stop()
    assert grabber.failures == 2
    assert grabber.latest.take() is not None


def test_rate_meter_measures_fps():
    meter = fg.RateMeter(window_s=2.0)
    for i in range(11):
        meter.tick(i * 0.1)
    assert abs(meter.rate(1.0) - 10.0) < 1e-9


def test_rate_meter_empty_and_stalled():
    meter = fg.RateMeter(window_s=2.0)
    assert meter.rate(0.0) == 0.0
    meter.tick(0.0)
    assert meter.rate(0.0) == 0.0
    meter.tick(0.5)
    # No frames for longer than the window: rate falls to zero.
    assert meter.rate(10.0) == 0.0


def test_rate_meter_forgets_old_events():
    meter = fg.RateMeter(window_s=1.0)
    for i in range(5):
        meter.tick(i * 0.1)
    for i in range(5):
        meter.tick(10.0 + i * 0.5)
    assert abs(meter.rate(12.0) - 2.0) < 1e-9


def test_timer_period_from_rate_parameter():
    assert abs(fg.timer_period_s(10.0) - 0.1) < 1e-9
    assert abs(fg.timer_period_s(0.0) - 2.0) < 1e-9   # clamped to 0.5 Hz
    assert abs(fg.timer_period_s(1000.0) - 1 / 60.0) < 1e-9
    assert abs(fg.timer_period_s("bad") - 0.1) < 1e-9
