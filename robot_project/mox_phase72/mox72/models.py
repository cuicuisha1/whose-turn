"""Shared immutable observations; importing this module opens no hardware."""
from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    pan: float
    tilt: float


@dataclass(frozen=True)
class FaceObservation:
    time_s: float             # Host receipt/acquisition observation, not video T0.
    sequence: int
    track_id: int | None
    count: int                # Eligible faces; exactly one required for control.
    point_m: tuple[float, float, float] | None
    horizontal_deg: float | None  # Positive = image right, from camera intrinsics.
    vertical_deg: float | None    # Positive = image down.
    score: float = 0.0
    camera_frame_number: int | None = None
    device_timestamp_ms: float | None = None
    error: str | None = None


@dataclass(frozen=True)
class HeadSnapshot:
    time_s: float
    pose: Pose
    target: Pose
    speed: float
    ramp_done: bool
    busy: bool
    settled_since_s: float | None
    version: str = "2.0.0"
    simulated: bool = False


# Driver contract:
# head.open() / head.close(); head.move(Pose, speed, reason), nonblocking;
# head.sample(now) -> HeadSnapshot, raises on a sticky health fault;
# head.stop(reason="operator_stop"), cancels all queued motion, keeps PWM.
# For deterministic simulation: SimHead starts opened and uses passed clock.
# Player contract:
# player.play("welcome" | "prompt", now), nonblocking;
# player.poll(now) -> bool (True = active, False = successful completion/idle);
# failed playback raises; player.stop() / close().
# All emit callbacks: emit(event_name: str, **fields), use time_s=... for a
# supplied monotonic timestamp; no raw images/audio are logged by default.
