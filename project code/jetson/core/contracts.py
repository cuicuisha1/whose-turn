"""Validated data at process boundaries; timestamps use this host's monotonic clock."""
from dataclasses import asdict, dataclass
import math


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class Target:
    track_id: int
    center_x: float
    center_y: float
    distance_m: float | None


@dataclass(frozen=True)
class Observation:
    valid: bool
    reason: str
    simulated: bool
    captured_at: float | None = None
    source_id: str = ""
    target: Target | None = None
    person_count: int = 0

    def fresh(self, now, max_age):
        return self.valid and self.captured_at is not None and 0 <= now - self.captured_at <= max_age

    def to_dict(self):
        return asdict(self)


def parse_observation(data, expected_simulation, now, max_age):
    invalid = lambda reason: Observation(False, reason, expected_simulation)
    if (not isinstance(data, dict) or data.get("type") != "perception"
            or type(data.get("schema_version")) is not int or data["schema_version"] != 1):
        return invalid("invalid_vision_schema")
    if data.get("simulated") is not expected_simulation:
        return invalid("vision_source_mismatch")
    expected_source = "simulation" if expected_simulation else "realsense"
    if data.get("source") != expected_source:
        return invalid("vision_source_mismatch")
    if data.get("valid") is not True:
        return invalid(str(data.get("status", "vision_invalid"))[:100])
    frame = data.get("frame")
    if not isinstance(frame, dict) or not finite(frame.get("received_monotonic_s")):
        return invalid("missing_frame_timestamp")
    timestamp = frame["received_monotonic_s"]
    if not 0 <= now - timestamp <= max_age:
        return invalid("expired_or_future_frame")
    run_id, generation = data.get("run_id"), data.get("camera_generation")
    if not isinstance(run_id, str) or not run_id or len(run_id) > 100 or type(generation) is not int or generation < 1:
        return invalid("invalid_source_identity")
    count = data.get("person_count")
    if type(count) is not int or not 0 <= count <= 300:
        return invalid("invalid_person_count")
    raw_target, target = data.get("target"), None
    if raw_target is not None:
        if not isinstance(raw_target, dict) or count < 1:
            return invalid("invalid_target")
        ident = raw_target.get("track_id")
        x, y = raw_target.get("center_x"), raw_target.get("center_y")
        if type(ident) is not int or ident < 1 or not finite(x) or not finite(y) or not 0 <= x <= 1 or not 0 <= y <= 1:
            return invalid("invalid_target")
        depth = raw_target.get("depth")
        distance = raw_target.get("distance_m")
        if isinstance(depth, dict) and depth.get("valid") is True:
            if not finite(distance) or not 0 < distance <= 20:
                return invalid("invalid_target_distance")
        else:
            distance = None
        target = Target(ident, float(x), float(y), distance)
    return Observation(True, "ok", expected_simulation, timestamp, f"{run_id}:{generation}", target, count)


@dataclass(frozen=True)
class Command:
    kind: str
    text: str
    created_at: float
    source: str

    @classmethod
    def create(cls, kind, now, text="", source="operator"):
        if not isinstance(kind, str) or kind not in {"pause", "resume", "reset", "shutdown", "text"}:
            raise ValueError("Unknown command; use pause/resume/reset/shutdown/text")
        if not finite(now) or not isinstance(source, str) or not 1 <= len(source) <= 64:
            raise ValueError("Command requires a finite timestamp and a short source name")
        if not isinstance(text, str) or len(text) > 500 or (kind == "text" and not text.strip()):
            raise ValueError("Text must contain 1..500 characters")
        return cls(kind, text.strip(), now, source)
