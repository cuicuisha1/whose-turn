"""Explicit, immutable settings for the offline head-turn experiment.

There are deliberately no research defaults. Example values used by a replay
or a test are not validated parameters for a participant study.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass, fields
import hashlib
import json
import math
from numbers import Real
from typing import Any


def _finite_number(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number, not {value!r}")
    try:
        result = float(value)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class ExperimentConfig:
    fixed_delay_s: float
    pause_s: float
    max_wait_s: float
    hold_s: float
    observation_window_s: float
    pan_a_deg: float
    pan_b_deg: float
    tilt_deg: float
    speed_deg_s: float

    def __post_init__(self) -> None:
        for item in fields(self):
            value = _finite_number(item.name, getattr(self, item.name))
            object.__setattr__(self, item.name, value)
        for name in ("fixed_delay_s", "hold_s"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} must be >= 0")
        for name in ("pause_s", "max_wait_s", "observation_window_s", "speed_deg_s"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be > 0")

    @classmethod
    def from_dict(cls, values: Mapping[str, Any]) -> ExperimentConfig:
        if not isinstance(values, Mapping):
            raise ValueError("Experiment configuration must be a mapping")
        expected = {item.name for item in fields(cls)}
        supplied = set(values)
        missing = expected - supplied
        unknown = supplied - expected
        if missing or unknown:
            details = []
            if missing:
                details.append("missing: " + ", ".join(sorted(missing)))
            if unknown:
                details.append("unknown: " + ", ".join(sorted(map(repr, unknown))))
            raise ValueError("Invalid experiment configuration (" + "; ".join(details) + ")")
        return cls(**dict(values))

    def to_dict(self) -> dict[str, float]:
        """Return a new mapping, so callers cannot mutate the frozen config."""
        return asdict(self)

    def fingerprint(self) -> str:
        encoded = json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"),
                             allow_nan=False).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()
