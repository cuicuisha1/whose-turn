"""Current-frame torso depth; holes/poor coverage are unknown, never zero metres."""
from dataclasses import asdict, dataclass
import math

import numpy as np


@dataclass(frozen=True)
class DepthEstimate:
    distance_m: float | None
    valid: bool
    valid_ratio: float
    samples: int
    iqr_m: float | None
    reason: str
    roi: tuple[int, int, int, int]

    def to_dict(self):
        return asdict(self)


class DepthEstimator:
    def __init__(self, min_m=.2, max_m=6.0, min_ratio=.25, min_samples=20, max_iqr_m=.6):
        if not 0 < min_m < max_m or not 0 < min_ratio <= 1 or min_samples < 1 or max_iqr_m <= 0:
            raise ValueError("Invalid depth estimation limits")
        self.min_m, self.max_m = min_m, max_m
        self.min_ratio, self.min_samples, self.max_iqr_m = min_ratio, min_samples, max_iqr_m

    def estimate(self, depth, scale, bbox):
        if depth.ndim != 2 or not math.isfinite(scale) or scale <= 0:
            raise ValueError("Depth must be a 2D array with a positive finite scale")
        if len(bbox) != 4 or not all(math.isfinite(x) for x in bbox):
            raise ValueError("Invalid bounding box")
        h, w = depth.shape
        x1, y1, x2, y2 = bbox
        # Sample inside the torso, avoiding head, legs and box boundary background.
        bw, bh = max(0, x2 - x1), max(0, y2 - y1)
        xa, xb = [min(w, max(0, int(v))) for v in (x1 + .3 * bw, x1 + .7 * bw)]
        ya, yb = [min(h, max(0, int(v))) for v in (y1 + .2 * bh, y1 + .65 * bh)]
        roi = (xa, ya, xb, yb)
        patch = depth[ya:yb, xa:xb].astype(np.float32) * scale
        if patch.size == 0:
            return DepthEstimate(None, False, 0.0, 0, None, "empty_roi", roi)
        values = patch[np.isfinite(patch) & (patch >= self.min_m) & (patch <= self.max_m)]
        n, ratio = int(values.size), float(values.size / patch.size)
        if n < self.min_samples or ratio < self.min_ratio:
            return DepthEstimate(None, False, ratio, n, None, "insufficient_valid_depth", roi)
        q25, median, q75 = np.percentile(values, [25, 50, 75])
        iqr = float(q75 - q25)
        if iqr > self.max_iqr_m:
            return DepthEstimate(None, False, ratio, n, iqr, "mixed_depths", roi)
        return DepthEstimate(float(median), True, ratio, n, iqr, "ok", roi)
