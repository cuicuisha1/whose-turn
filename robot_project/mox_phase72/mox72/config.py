"""Phase 7.2 behavioural parameters. No imports can open hardware."""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Any


@dataclass(frozen=True)
class Config:
    # All angles are the existing firmware's command coordinates.
    pan_a_base: float = 45.0
    pan_b_base: float = -45.0
    tilt_base: float = 0.0
    pan_correction_limit: float = 10.0
    tilt_limit: float = 25.0
    pan_sign: int = -1
    tilt_sign: int | None = None  # Must be verified on this assembled robot.
    a_speed: float = 20.0
    b_speed: float = 60.0
    main_speed: float = 40.0

    # A must occupy the station while the camera itself is stationary.
    stable_duration_s: float = 2.0
    stable_span_m: float = 0.05
    station_min_z_m: float = 0.65
    station_max_z_m: float = 1.60
    station_max_abs_x_m: float = 0.50
    face_acquire_s: float = 0.30
    max_face_gap_s: float = 0.15
    face_fresh_s: float = 0.15
    center_deadband_deg: float = 1.50
    center_dwell_s: float = 0.25
    center_reacquire_s: float = 0.30
    center_recovery_budget_s: float = 2.0
    correction_gain: float = 0.65
    max_step_deg: float = 2.0
    face_ema_alpha: float = 0.35
    settle_s: float = 0.12

    a_center_timeout_s: float = 12.0
    b_center_timeout_s: float = 4.0
    move_timeout_s: float = 15.0
    wait_a_timeout_s: float = 120.0
    wait_join_timeout_s: float = 120.0
    attempt_timeout_s: float = 300.0
    camera_timeout_s: float = 1.0

    # Unchanged trigger parameters; W is metadata based on external-video T0.
    fixed_delay_s: float = 1.0
    pause_s: float = 0.6
    max_wait_s: float = 3.0
    hold_s: float = 0.5
    observation_window_s: float = 5.0
    audio_fresh_s: float = 0.25
    post_prompt_guard_s: float = 0.30

    def __post_init__(self) -> None:
        for item in fields(self):
            value = getattr(self, item.name)
            if item.name in {"pan_sign", "tilt_sign"}:
                if item.name == "tilt_sign" and value is None:
                    continue
                if isinstance(value, bool) or not isinstance(value, int) or value not in (-1, 1):
                    raise ValueError(f"{item.name} must be -1 or +1")
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{item.name} must be a finite number")
            object.__setattr__(self, item.name, float(value))
        nonnegative = {"fixed_delay_s", "hold_s", "post_prompt_guard_s", "settle_s"}
        signed = {"pan_a_base", "pan_b_base", "tilt_base"}
        for item in fields(self):
            if item.name in signed | {"pan_sign", "tilt_sign"}:
                continue
            value = getattr(self, item.name)
            if value < 0 or (value == 0 and item.name not in nonnegative):
                raise ValueError(f"{item.name} must be {'nonnegative' if item.name in nonnegative else 'positive'}")
        if not 0 < self.pan_a_base <= 120 or not -120 <= self.pan_b_base < 0:
            raise ValueError("A base must be positive and B base negative, within firmware +/-120")
        if max(abs(self.pan_a_base), abs(self.pan_b_base)) + self.pan_correction_limit > 120:
            raise ValueError("Pan correction envelope exceeds firmware +/-120")
        if self.pan_a_base-self.pan_b_base <= 2*(self.pan_correction_limit+self.center_deadband_deg):
            raise ValueError("A/B candidate angle regions including visual tolerance must not overlap")
        if not abs(self.tilt_base) <= self.tilt_limit <= 60:
            raise ValueError("tilt_base must be inside tilt_limit <= 60")
        if min(self.a_speed, self.b_speed, self.main_speed) < 10 or max(self.a_speed, self.b_speed, self.main_speed) > 120:
            raise ValueError("Firmware speed range is 10..120 degrees/s")
        if not self.station_min_z_m < self.station_max_z_m:
            raise ValueError("station_min_z_m must be less than station_max_z_m")
        if not 0 < self.face_ema_alpha <= 1:
            raise ValueError("face_ema_alpha must be in (0, 1]")
        if not 0 < self.correction_gain <= 1:
            raise ValueError("correction_gain must be in (0, 1]")
        if self.max_step_deg > self.pan_correction_limit:
            raise ValueError("max_step_deg must not exceed the Pan correction envelope")
        if self.face_fresh_s > self.camera_timeout_s or self.max_face_gap_s > self.camera_timeout_s:
            raise ValueError("Face freshness/gap limits must not exceed camera timeout")
        if self.stable_duration_s < self.max_face_gap_s:
            raise ValueError("The stability interval must exceed its allowed frame gap")
        if self.center_reacquire_s < self.max_face_gap_s:
            raise ValueError("center_reacquire_s must cover max_face_gap_s")
        if not self.center_reacquire_s <= self.center_recovery_budget_s <= 10:
            raise ValueError("center_recovery_budget_s must cover reacquisition and be <= 10 seconds")
        if self.center_reacquire_s >= min(self.a_center_timeout_s, self.b_center_timeout_s):
            raise ValueError("center_reacquire_s must be shorter than both active centering budgets")
        if self.audio_fresh_s < 0.02:
            raise ValueError("audio_fresh_s must cover at least one 20 ms audio frame")

    def validate_live(self) -> None:
        if self.tilt_sign is None:
            raise ValueError("Live automatic motion requires an explicitly verified tilt_sign (-1 or +1)")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, values: Mapping[str, Any]) -> "Config":
        if not isinstance(values, Mapping):
            raise ValueError("Configuration must be a JSON object")
        unknown = set(values) - {item.name for item in fields(cls)}
        if unknown:
            raise ValueError(f"Unknown configuration fields: {sorted(unknown)}")
        return cls(**dict(values))

    @property
    def sha256(self) -> str:
        encoded = json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()


def load_config(path: str | Path | None = None) -> Config:
    if path is None:
        return Config()
    return Config.from_dict(json.loads(Path(path).read_text(encoding="utf-8-sig")))
