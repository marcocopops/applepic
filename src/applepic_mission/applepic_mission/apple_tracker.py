"""Pure-Python fusion of per-frame apple detections into a list of known apples (no rclpy).

Each frame of world-frame detections is associated greedily (closest first) to existing tracks
within `gate`; matched tracks are refined with a running mean, unmatched detections create new
tracks. A track is `confirmed` once seen `min_views` times. Tracks are kept when out of view (the
camera loses an apple when the robot gets close), and are removed by `mark_picked` / `mark_failed`.
"""
import math
from dataclasses import dataclass


@dataclass
class Track:
    id: int
    x: float
    y: float
    views: int = 1
    last_seen: float = 0.0
    state: str = 'active'   # active | picked | failed


class AppleTracker:

    def __init__(self, gate=0.15, min_views=2, max_mean_views=20):
        self.gate = gate
        self.min_views = min_views
        self.max_mean_views = max_mean_views   # caps the averaging window (apples may be pushed)
        self.tracks = []
        self._next_id = 0

    def update(self, detections, stamp=0.0):
        """Fuse one frame of detections [(x, y), ...]. Return the list of updated tracks."""
        active = [t for t in self.tracks if t.state == 'active']
        pairs = sorted(
            (math.hypot(t.x - dx, t.y - dy), ti, di)
            for ti, t in enumerate(active) for di, (dx, dy) in enumerate(detections))
        used_t, used_d, updated = set(), set(), []
        for d, ti, di in pairs:
            if d > self.gate:
                break
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            t = active[ti]
            n = min(t.views, self.max_mean_views - 1)
            dx, dy = detections[di]
            t.x = (t.x * n + dx) / (n + 1)
            t.y = (t.y * n + dy) / (n + 1)
            t.views += 1
            t.last_seen = stamp
            updated.append(t)
        for di, (dx, dy) in enumerate(detections):
            if di in used_d:
                continue
            # A detection near an already picked/failed apple is not a new apple.
            if any(t.state != 'active' and math.hypot(t.x - dx, t.y - dy) <= self.gate
                   for t in self.tracks):
                continue
            t = Track(self._next_id, dx, dy, last_seen=stamp)
            self._next_id += 1
            self.tracks.append(t)
            updated.append(t)
        return updated

    def confirmed(self):
        return [t for t in self.tracks if t.state == 'active' and t.views >= self.min_views]

    def nearest(self, x, y):
        """Closest confirmed active track to (x, y), or None."""
        cands = self.confirmed()
        if not cands:
            return None
        return min(cands, key=lambda t: math.hypot(t.x - x, t.y - y))

    def get(self, track_id):
        for t in self.tracks:
            if t.id == track_id:
                return t
        return None

    def mark_picked(self, track_id):
        self._set_state(track_id, 'picked')

    def mark_failed(self, track_id):
        self._set_state(track_id, 'failed')

    def _set_state(self, track_id, state):
        t = self.get(track_id)
        if t is not None:
            t.state = state
