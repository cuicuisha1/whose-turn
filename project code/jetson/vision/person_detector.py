"""Two real detectors and an explicit fixture detector for offline tests."""
from pathlib import Path

import numpy as np

from .types import Detection


def iou(a, b):
    overlap = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    area_a = max(0, a[2] - a[0]) * max(0, a[3] - a[1])
    area_b = max(0, b[2] - b[0]) * max(0, b[3] - b[1])
    return overlap / max(1e-9, area_a + area_b - overlap)


def suppress(detections, threshold=.45):
    kept = []
    for detection in sorted(detections, key=lambda d: d.score, reverse=True):
        if all(iou(detection.bbox, other.bbox) <= threshold for other in kept):
            kept.append(detection)
    return kept


class HogDetector:
    name = "opencv_hog"

    def __init__(self, threshold=.5):
        import cv2
        self.cv2 = cv2
        self.threshold = threshold
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detect(self, frame):
        h, w = frame.color.shape[:2]
        scale = min(1.0, 480 / w)
        image = self.cv2.resize(frame.color, (round(w * scale), round(h * scale)))
        if image.shape[0] < 128 or image.shape[1] < 64:
            return []
        boxes, weights = self.hog.detectMultiScale(image, hitThreshold=self.threshold,
                                                  winStride=(8, 8), padding=(8, 8), scale=1.05)
        result = []
        for (x, y, bw, bh), score in zip(boxes, np.asarray(weights).reshape(-1)):
            bbox = (max(0., x / scale), max(0., y / scale),
                    min(float(w), (x + bw) / scale), min(float(h), (y + bh) / scale))
            result.append(Detection(bbox, float(score), "svm_margin"))
        return suppress(result)


class YoloDetector:
    name = "ultralytics_yolo"

    def __init__(self, model, device="cpu", confidence=.45, image_size=640):
        path = Path(model).expanduser().resolve()
        if not path.is_file():
            raise ValueError(f"YOLO model not found: {path}. Supply a local .pt or .engine; no automatic model download.")
        if path.suffix.lower() not in {".pt", ".engine"}:
            raise ValueError("Supported YOLO model formats: .pt and .engine")
        from ultralytics import YOLO
        self.model = YOLO(str(path), task="detect")
        self.device, self.confidence, self.image_size = device, confidence, image_size
        self.person_class = None

    def detect(self, frame):
        # Resolve class by name instead of assuming custom models use COCO class 0.
        if self.person_class is None:
            names = self.model.names
            pairs = names.items() if isinstance(names, dict) else enumerate(names)
            ids = [int(i) for i, name in pairs if name.lower() == "person"]
            if len(ids) != 1:
                raise RuntimeError("Model must have exactly one class named 'person'")
            self.person_class = ids[0]
        result = self.model.predict(source=frame.color, classes=[self.person_class],
                                    conf=self.confidence, imgsz=self.image_size, device=self.device,
                                    verbose=False, save=False)[0]
        if result.boxes is None:
            return []
        boxes = result.boxes.xyxy.cpu().numpy()
        scores = result.boxes.conf.cpu().numpy()
        return [Detection(tuple(map(float, box)), float(score)) for box, score in zip(boxes, scores)]


class SyntheticDetector:
    name = "synthetic_fixture"

    def detect(self, frame):
        if frame.source != "simulation" or frame.synthetic_detections is None:
            raise ValueError("Synthetic detector can only read synthetic test frames")
        return list(frame.synthetic_detections)


class NoDetector:
    name = "disabled"

    def detect(self, frame):
        return []
