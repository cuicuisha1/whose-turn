import math

from .depth_estimator import DepthEstimator
from .person_tracker import PersonTracker
from .types import Detection


class Perception:
    def __init__(self, detector, min_depth=.2, max_depth=6.0):
        self.detector = detector
        self.depth = DepthEstimator(min_m=min_depth, max_m=max_depth)
        self.tracker = PersonTracker()
        self.generation = None

    def reset(self):
        self.tracker.reset()
        self.generation = None

    def process(self, frame, generation):
        if generation != self.generation:
            self.reset()
            self.generation = generation
        h, w = frame.color.shape[:2]
        if frame.depth.shape != (h, w):
            raise ValueError("Perception requires depth aligned to the color image")
        detections = []
        for d in self.detector.detect(frame):
            if not all(math.isfinite(v) for v in (*d.bbox, d.score)):
                continue
            x1, y1, x2, y2 = d.bbox
            box = (max(0., x1), max(0., y1), min(float(w), x2), min(float(h), y2))
            if box[2] > box[0] and box[3] > box[1]:
                detections.append(Detection(box, d.score, d.score_kind))
        visible = self.tracker.update(detections, frame.received_at, w, h)
        people = []
        for track in visible:
            d = track.detection
            estimate = self.depth.estimate(frame.depth, frame.depth_scale, d.bbox)
            people.append({"track_id": track.id, "bbox_xyxy": list(d.bbox),
                           "center_x": track.center_x, "center_y": track.center_y,
                           "score": d.score, "score_kind": d.score_kind,
                           "confidence": d.score if d.score_kind == "model_confidence" else None,
                           "distance_m": estimate.distance_m, "depth": estimate.to_dict()})
        target = self.tracker.choose_target(people)
        attention = None if target is None else {
            "horizontal_error": 2 * target["center_x"] - 1,
            "vertical_error": 1 - 2 * target["center_y"],
            "units": "normalized_image_error", "actuation_enabled": False}
        valid_pixels = (frame.depth > 0) & (frame.depth * frame.depth_scale >= self.depth.min_m) & (
            frame.depth * frame.depth_scale <= self.depth.max_m)
        return {"people": people, "person_count": len(people),
                "person_detected": bool(people) if self.detector.name != "disabled" else None,
                "target": target, "attention": attention,
                "depth_valid_ratio": float(valid_pixels.mean())}


def make_observation(status, result, processed_frame, processed_generation, now,
                     max_age, detector_name, source, run_id):
    """Re-check at publication, including inference latency and reconnect races."""
    age = None if processed_frame is None else max(0., now - processed_frame.received_at)
    valid = (status.state == "streaming" and result is not None and age is not None
             and age <= max_age and status.generation == processed_generation)
    state = status.state
    if state == "streaming":
        state = "ok" if valid else "stale_or_processing"
    frame = processed_frame if valid else None
    message = {"schema_version": 1, "type": "perception", "run_id": run_id,
               "source": source, "simulated": source == "simulation", "detector": detector_name,
               "detection_enabled": detector_name != "disabled",
               "host_monotonic_s": now, "valid": valid, "status": state,
               "frame_age_ms": None if age is None else round(age * 1000, 2),
               "max_frame_age_ms": max_age * 1000, "camera_generation": status.generation,
               "capture_sequence": status.sequence, "capture_fps": round(status.capture_fps, 2),
               "restarts": status.restarts, "camera_error": status.error,
               "frame": None, "people": [], "person_count": 0, "person_detected": None,
               "target": None, "attention": None, "depth_valid_ratio": None,
               "distance_reference": "camera_optical_axis_z_m", "actuation_enabled": False}
    if valid:
        message.update(result)
        message["frame"] = {"color_number": frame.color_number, "depth_number": frame.depth_number,
                            "color_timestamp_ms": frame.color_timestamp_ms,
                            "depth_timestamp_ms": frame.depth_timestamp_ms,
                            "color_clock": frame.color_clock, "depth_clock": frame.depth_clock,
                            "received_monotonic_s": frame.received_at,
                            "width": frame.color.shape[1], "height": frame.color.shape[0],
                            "depth_scale_m": frame.depth_scale, "device": frame.device}
    return message
