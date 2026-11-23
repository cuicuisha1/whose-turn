"""Deterministic trigger policies; no devices, background tasks or real clocks.

AudioFrame times describe the supplied observation timeline, not measured
human speaking turns. UNKNOWN and MASKED are never treated as silence.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from .config import _finite_number


# Absorb arithmetic noise from repeated 20 ms additions, not missing frames.
_EPS = 1e-9


class AudioStatus(str, Enum):
    VOICED = "voiced"
    UNVOICED = "unvoiced"
    UNKNOWN = "unknown"
    MASKED = "masked"


@dataclass(frozen=True)
class AudioFrame:
    start_s: float
    end_s: float
    status: AudioStatus

    def __post_init__(self) -> None:
        start = _finite_number("start_s", self.start_s)
        end = _finite_number("end_s", self.end_s)
        if end <= start:
            raise ValueError("AudioFrame end_s must be greater than start_s")
        try:
            status = AudioStatus(self.status)
        except (ValueError, TypeError) as exc:
            raise ValueError(f"Unknown audio status: {self.status!r}") from exc
        object.__setattr__(self, "start_s", start)
        object.__setattr__(self, "end_s", end)
        object.__setattr__(self, "status", status)


@dataclass(frozen=True)
class Trigger:
    reason: str
    due_at: float


class _Policy:
    def __init__(self) -> None:
        self._join: float | None = None
        self._last_frame_end: float | None = None
        self._last_evaluation: float | None = None
        self._fired = False

    def start(self, join_time: float) -> None:
        join_time = _finite_number("join_time", join_time)
        if self._join is not None:
            raise RuntimeError("This policy already received JOIN; create a new policy for a new trial")
        if not all(math.isfinite(join_time + offset) for offset in self._deadline_offsets()):
            raise ValueError("JOIN plus a policy duration must produce a finite deadline")
        self._join = join_time

    def _deadline_offsets(self) -> tuple[float, ...]:
        raise NotImplementedError

    def _frame_start(self, frame: AudioFrame) -> float | None:
        if not isinstance(frame, AudioFrame):
            raise ValueError("observe requires an AudioFrame")
        if self._join is None or frame.end_s <= self._join:
            return None
        start = max(frame.start_s, self._join)
        if self._last_frame_end is not None and start < self._last_frame_end - _EPS:
            raise ValueError("Audio frames must be ordered and must not overlap")
        if self._last_evaluation is not None and frame.end_s < self._last_evaluation - _EPS:
            raise ValueError("Audio frame is older than the latest policy evaluation")
        return start

    def _evaluate_time(self, now: float) -> float | None:
        now = _finite_number("now", now)
        if self._join is None:
            return None
        if now < self._join - _EPS:
            raise ValueError("Evaluation cannot precede JOIN")
        if self._last_frame_end is not None and now < self._last_frame_end - _EPS:
            raise ValueError("Evaluation cannot use future audio frames")
        if self._last_evaluation is not None and now < self._last_evaluation - _EPS:
            raise ValueError("Evaluation time must not move backwards")
        self._last_evaluation = now
        return now


class FixedDelayPolicy(_Policy):
    """The audio content has no effect on the fixed trigger deadline."""

    def __init__(self, delay_s: float):
        super().__init__()
        self.delay_s = _finite_number("delay_s", delay_s)
        if self.delay_s < 0:
            raise ValueError("delay_s must be >= 0")

    def _deadline_offsets(self) -> tuple[float, ...]:
        return (self.delay_s,)

    def observe(self, frame: AudioFrame) -> None:
        start = self._frame_start(frame)
        if start is not None:
            self._last_frame_end = frame.end_s

    def evaluate(self, now: float) -> Trigger | None:
        now = self._evaluate_time(now)
        if now is None or self._fired:
            return None
        due = self._join + self.delay_s
        if now + _EPS < due:
            return None
        self._fired = True
        return Trigger("fixed_delay", due)


class PausePolicy(_Policy):
    """Count contiguous valid non-speech after JOIN, with a fixed timeout.

Only supplied frames add quiet time. Polling does not extend a pause. The
first qualifying pause is latched, so delayed evaluation retains its cause.
If pause and timeout have exactly the same deadline (within 1 ns arithmetic
tolerance), timeout wins. A real device fault is an independent controller
input and must prevent either trigger from being executed.
"""

    def __init__(self, pause_s: float, max_wait_s: float):
        super().__init__()
        self.pause_s = _finite_number("pause_s", pause_s)
        self.max_wait_s = _finite_number("max_wait_s", max_wait_s)
        if self.pause_s <= 0 or self.max_wait_s <= 0:
            raise ValueError("pause_s and max_wait_s must be > 0")
        self._quiet_start: float | None = None
        self._pause_due: float | None = None

    def _deadline_offsets(self) -> tuple[float, ...]:
        return (self.pause_s, self.max_wait_s)

    def observe(self, frame: AudioFrame) -> None:
        start = self._frame_start(frame)
        if start is None:
            return
        contiguous = (self._last_frame_end is not None and
                      math.isclose(start, self._last_frame_end, rel_tol=0, abs_tol=_EPS))
        if frame.status is AudioStatus.UNVOICED:
            if self._quiet_start is None or not contiguous:
                self._quiet_start = start
            due = self._quiet_start + self.pause_s
            # A later quiet span can overflow even when JOIN deadlines were
            # finite. Such a pause cannot beat the already finite timeout.
            if math.isfinite(due) and frame.end_s + _EPS >= due and self._pause_due is None:
                self._pause_due = due
        else:
            self._quiet_start = None
        self._last_frame_end = frame.end_s

    def evaluate(self, now: float) -> Trigger | None:
        now = self._evaluate_time(now)
        if now is None or self._fired:
            return None
        timeout_due = self._join + self.max_wait_s
        if self._pause_due is not None and self._pause_due < timeout_due - _EPS:
            reason, due = "speech_pause", self._pause_due
        else:
            reason, due = "timeout", timeout_due
        if now + _EPS < due:
            return None
        self._fired = True
        return Trigger(reason, due)
