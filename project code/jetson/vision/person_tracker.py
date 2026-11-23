"""Short-lived spatial IDs, not biometric identification or re-identification."""
from dataclasses import dataclass
import math

from .person_detector import iou
from .types import Detection


@dataclass
class Track:
    id: int
    detection: Detection
    seen_at: float
    center_x: float
    center_y: float


class PersonTracker:
    def __init__(self, lost_after=.75, smoothing=.4):
        self.lost_after, self.smoothing = lost_after, smoothing
        self.tracks = {}
        self.next_id = 1
        self.target_id = None

    def reset(self):
        self.tracks.clear()
        self.target_id = None
        # Do not recycle IDs after a camera reconnect within the same process.

    def update(self, detections, now, width, height):
        self.tracks = {i: t for i, t in self.tracks.items() if now - t.seen_at <= self.lost_after}
        candidates = []
        for ident, track in self.tracks.items():
            a = track.detection.bbox
            for index, detection in enumerate(detections):
                b = detection.bbox
                overlap = iou(a, b)
                dx = ((a[0] + a[2]) - (b[0] + b[2])) / (2 * width)
                dy = ((a[1] + a[3]) - (b[1] + b[3])) / (2 * height)
                distance = math.hypot(dx, dy)
                ratio = (b[2] - b[0]) * (b[3] - b[1]) / max(1, (a[2] - a[0]) * (a[3] - a[1]))
                if overlap >= .25 or (distance < .12 and .5 < ratio < 2):
                    candidates.append((overlap - distance, ident, index))
        matched_tracks, matched_detections = set(), set()
        assignments = {}
        for _, ident, index in sorted(candidates, reverse=True):
            if ident not in matched_tracks and index not in matched_detections:
                assignments[index] = ident
                matched_tracks.add(ident)
                matched_detections.add(index)
        visible = []
        for index, detection in enumerate(detections):
            x1, y1, x2, y2 = detection.bbox
            cx, cy = (x1 + x2) / (2 * width), (y1 + y2) / (2 * height)
            ident = assignments.get(index)
            if ident is None:
                ident = self.next_id
                self.next_id += 1
            else:
                old = self.tracks[ident]
                # After a gap, start from the current measurement rather than old coordinates.
                if now - old.seen_at < .3:
                    a = self.smoothing
                    cx, cy = a * cx + (1 - a) * old.center_x, a * cy + (1 - a) * old.center_y
            track = Track(ident, detection, now, cx, cy)
            self.tracks[ident] = track
            visible.append(track)
        if self.target_id not in self.tracks:
            self.target_id = None
        return visible

    def choose_target(self, people):
        by_id = {p["track_id"]: p for p in people}
        if self.target_id is not None:
            # Hold identity during a short occlusion, but output no stale position.
            return by_id.get(self.target_id)
        if not people:
            return None
        selected = min(people, key=lambda p: (p["distance_m"] is None,
                                             p["distance_m"] if p["distance_m"] is not None
                                             else abs(p["center_x"] - .5)))
        self.target_id = selected["track_id"]
        return selected
