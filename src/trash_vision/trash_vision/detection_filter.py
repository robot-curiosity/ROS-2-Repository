"""Pure-Python detection filtering (no ROS, camera or model needed).

Keeps a target from being reported until it has been seen in N of the last M
frames, drops detections inside ignore regions (where the robot's own bag
hoop or arm can appear), and picks the best remaining target by distance,
then confidence.
"""
import math
from collections import deque
from dataclasses import dataclass
from typing import Any, List, Optional, Sequence


@dataclass
class Candidate:
    """One detection in one frame, coordinates normalized 0..1."""

    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    distance: Optional[float] = None  # metres from base_link, None unknown
    payload: Any = None  # caller data handed back unchanged

    @property
    def center(self):
        """Normalized box centre (x, y)."""
        return (self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0


def parse_regions(flat: Sequence[float]) -> List[tuple]:
    """Turn a flat [x1, y1, x2, y2, ...] list into rectangles.

    A trailing incomplete group is ignored (ROS parameters cannot nest).
    """
    values = [float(v) for v in flat]
    regions = []
    for i in range(0, len(values) - len(values) % 4, 4):
        x1, y1, x2, y2 = values[i:i + 4]
        if x1 == x2 or y1 == y2:
            continue  # zero-area placeholder (ROS needs a non-empty default)
        regions.append((min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)))
    return regions


def in_region(candidate: Candidate, regions: Sequence[tuple]) -> bool:
    """True if the candidate's centre lies inside any ignore region."""
    cx, cy = candidate.center
    return any(x1 <= cx <= x2 and y1 <= cy <= y2
               for x1, y1, x2, y2 in regions)


class _Track:
    def __init__(self, center, window):
        self.center = center
        self.history = deque(maxlen=window)
        self.matched_now = False
        self.candidate = None

    def seen_count(self):
        return sum(1 for seen in self.history if seen)


class DetectionFilter:
    """Confirm targets over several frames and choose the best one."""

    def __init__(self, min_confidence=0.4, confirm_frames=3,
                 window_frames=5, match_radius=0.1, ignore_regions=()):
        self.min_confidence = float(min_confidence)
        self.window_frames = max(1, int(window_frames))
        self.confirm_frames = max(
            1, min(int(confirm_frames), self.window_frames))
        self.match_radius = float(match_radius)
        self.ignore_regions = list(ignore_regions)
        self._tracks: List[_Track] = []

    def reset(self):
        """Forget all tracks."""
        self._tracks = []

    def update(self, candidates: Sequence[Candidate]) -> Optional[Candidate]:
        """Feed one frame (possibly empty); return the best confirmed one."""
        usable = [c for c in candidates
                  if c.confidence >= self.min_confidence
                  and not in_region(c, self.ignore_regions)]
        for track in self._tracks:
            track.matched_now = False
            track.candidate = None
        # Greedy nearest matching, closest pairs first.
        pairs = []
        for ti, track in enumerate(self._tracks):
            for ci, cand in enumerate(usable):
                d = math.dist(track.center, cand.center)
                if d <= self.match_radius:
                    pairs.append((d, ti, ci))
        pairs.sort()
        used_tracks, used_cands = set(), set()
        for _, ti, ci in pairs:
            if ti in used_tracks or ci in used_cands:
                continue
            used_tracks.add(ti)
            used_cands.add(ci)
            track = self._tracks[ti]
            track.center = usable[ci].center
            track.matched_now = True
            track.candidate = usable[ci]
        for ci, cand in enumerate(usable):
            if ci not in used_cands:
                track = _Track(cand.center, self.window_frames)
                track.matched_now = True
                track.candidate = cand
                self._tracks.append(track)
        for track in self._tracks:
            track.history.append(track.matched_now)
        self._tracks = [t for t in self._tracks if t.seen_count() > 0]

        confirmed = [t.candidate for t in self._tracks
                     if t.matched_now
                     and t.seen_count() >= self.confirm_frames]
        if not confirmed:
            return None
        return min(confirmed, key=_rank)


def _rank(candidate: Candidate):
    distance = (math.inf if candidate.distance is None
                else candidate.distance)
    return distance, -candidate.confidence
