from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class Detection:
    bbox: tuple[float, float, float, float]  # x1,y1,x2,y2; color-image pixels
    score: float
    score_kind: str = "model_confidence"


@dataclass(frozen=True)
class Frame:
    color: np.ndarray  # BGR uint8; owned copy, never an SDK buffer view
    depth: np.ndarray  # Z16, aligned to color; zero means no measurement
    depth_scale: float
    received_at: float  # host monotonic seconds, before alignment/copy
    color_number: int
    depth_number: int
    color_timestamp_ms: float
    depth_timestamp_ms: float
    color_clock: str
    depth_clock: str
    source: str
    device: dict = field(default_factory=dict)
    synthetic_detections: Optional[tuple[Detection, ...]] = None
