"""Pure-Python newest-frame grabbing and FPS measurement (no ROS, no cv2).

A camera stream buffers frames, so reading one frame per detector cycle makes
the detector work on stale images (RISKS.md #11). ``FrameGrabber`` reads
continuously in its own thread and ``LatestFrame`` keeps only the newest
frame; the detector takes whatever is newest, at most once per frame.
``RateMeter`` measures the real processing rate over a sliding window.
"""

import collections
import threading
import time


class LatestFrame:
    """Thread-safe single-slot holder: a new frame replaces the old one."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._frame = None
        self._time = 0.0
        self._seq = 0
        self._taken_seq = 0
        self.dropped = 0

    def put(self, frame):
        with self._lock:
            if self._seq > self._taken_seq:
                # The previous frame was never processed: it is dropped.
                self.dropped += 1
            self._frame = frame
            self._time = self._clock()
            self._seq += 1

    def take(self):
        """Return (frame, seq, age_s) for a frame not yet taken, else None."""
        with self._lock:
            if self._frame is None or self._seq == self._taken_seq:
                return None
            self._taken_seq = self._seq
            return self._frame, self._seq, self._clock() - self._time


class FrameGrabber:
    """Reads ``read_fn() -> (ok, frame)`` in a thread into a LatestFrame."""

    def __init__(self, read_fn, latest=None, retry_delay_s=0.05):
        self.read_fn = read_fn
        self.latest = latest if latest is not None else LatestFrame()
        self.retry_delay_s = retry_delay_s
        self.failures = 0
        self._stop = threading.Event()
        self._thread = None

    def run_once(self):
        """Read one frame. Returns True if a frame was stored."""
        try:
            ok, frame = self.read_fn()
        except Exception:
            ok, frame = False, None
        if not ok or frame is None:
            self.failures += 1
            return False
        self.latest.put(frame)
        return True

    def _loop(self):
        while not self._stop.is_set():
            if not self.run_once():
                self._stop.wait(self.retry_delay_s)

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(
                target=self._loop, name="frame-grabber", daemon=True)
            self._thread.start()

    def stop(self, timeout=2.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None


class RateMeter:
    """Events per second over the last ``window_s`` seconds."""

    def __init__(self, window_s=2.0):
        self.window_s = float(window_s)
        self._times = collections.deque()

    def tick(self, now):
        self._times.append(now)
        self._trim(now)

    def _trim(self, now):
        while self._times and now - self._times[0] > self.window_s:
            self._times.popleft()

    def rate(self, now):
        """FPS, or 0.0 with fewer than two events in the window."""
        self._trim(now)
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        if span <= 0:
            return 0.0
        return (len(self._times) - 1) / span


def timer_period_s(rate_hz, minimum_hz=0.5, maximum_hz=60.0):
    """Timer period for a processing-rate parameter, clamped to 0.5-60 Hz."""
    try:
        hz = float(rate_hz)
    except (TypeError, ValueError):
        hz = 10.0
    hz = min(max(hz, minimum_hz), maximum_hz)
    return 1.0 / hz
