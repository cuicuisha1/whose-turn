"""ReSpeaker capture adapter: ALSA bytes -> timestamped 20 ms WebRTC VAD.

Importing this module opens nothing. Only ``ArecordAudioSource.start`` starts
arecord, on Linux, and imports the already installed webrtcvad package. Sample
times use one host-receipt anchor plus sample counts, not hardware timestamps.
The raw ALSA pipe cannot identify every short physical sample loss. Fatal ALSA
xruns, pipe faults, excessive backlog and host-visible timing gaps stop capture;
these checks do not claim hardware capture-time or sample-loss measurement.
"""
from __future__ import annotations

from collections import deque
from collections.abc import Mapping
from dataclasses import asdict, dataclass, fields
import math
import os
from pathlib import Path
import select
import struct
import subprocess
import sys
import time
from typing import Any, Callable

from .config import _finite_number
from .policies import AudioFrame, AudioStatus


RATE, CHANNELS, FRAME_MS = 16000, 2, 20
FRAME_S = FRAME_MS / 1000
SAMPLES = RATE * FRAME_MS // 1000
FRAME_BYTES, MONO_BYTES = SAMPLES * CHANNELS * 2, SAMPLES * 2
BYTES_PER_SECOND = RATE * CHANNELS * 2
EPSILON = 1e-9


class AudioFault(RuntimeError):
    def __init__(self, code: str, detail: str):
        self.code, self.detail = code, detail
        super().__init__(f"{code}: {detail}")


@dataclass(frozen=True)
class AudioConfig:
    device: str
    channel: int
    vad_mode: int
    min_dbfs: float
    buffer_frames: int
    startup_timeout_s: float
    timeout_s: float
    max_latency_s: float
    poll_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.device, str) or not self.device.strip() or "\0" in self.device:
            raise ValueError("device must be a nonempty ALSA device name")
        for name in ("channel", "vad_mode", "buffer_frames"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{name} must be an integer")
        if self.channel not in (0, 1) or self.vad_mode not in (0, 1, 2, 3):
            raise ValueError("channel must be 0/1 and vad_mode must be 0..3")
        if self.buffer_frames <= 0:
            raise ValueError("buffer_frames must be positive")
        for name in ("min_dbfs", "startup_timeout_s", "timeout_s", "max_latency_s", "poll_s"):
            object.__setattr__(self, name, _finite_number(name, getattr(self, name)))
        if not -180 <= self.min_dbfs <= 0:
            raise ValueError("min_dbfs must be in -180..0")
        if min(self.startup_timeout_s, self.timeout_s, self.max_latency_s, self.poll_s) <= 0:
            raise ValueError("audio time limits must be positive")
        if self.max_latency_s < FRAME_S or self.buffer_frames / RATE > self.max_latency_s:
            raise ValueError("max_latency_s must cover one frame and the configured ALSA buffer")
        if self.poll_s > min(self.max_latency_s, self.timeout_s, self.startup_timeout_s):
            raise ValueError("poll_s must not exceed latency or timeout limits")

    @classmethod
    def from_dict(cls, values: Mapping[str, Any]) -> AudioConfig:
        if not isinstance(values, Mapping):
            raise ValueError("audio configuration must be an object")
        expected = {item.name for item in fields(cls)}
        if set(values) != expected:
            raise ValueError(f"audio configuration keys must be exactly {sorted(expected)}")
        return cls(**dict(values))

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TimedAudio:
    frame: AudioFrame
    received_at_s: float
    sequence: int
    dbfs: float


class PCMFramer:
    """Pure byte framing and classification; empty calls never invent silence.

    The first complete batch anchors its *entire* complete stereo-sample span,
    including any sub-frame tail, to the receive time. Later timestamps are
    sample-count derived without reanchoring. Scheduling jitter may make a
    small number of modeled frames early; the source holds those for delivery.
    """

    def __init__(self, config: AudioConfig, vad: Any):
        self.config, self.vad = config, vad
        self._pending = bytearray()
        self._pending_since: float | None = None
        self._last_call: float | None = None
        self._last_data: float | None = None
        self._anchor: float | None = None
        self._sequence = 0

    @property
    def pending_bytes(self) -> int:
        return len(self._pending)

    @property
    def pending_since(self) -> float | None:
        return self._pending_since

    def feed(self, data: bytes, received_at: float) -> list[TimedAudio]:
        try:
            now = _finite_number("received_at", received_at)
        except ValueError as exc:
            raise AudioFault("invalid_timestamp", str(exc)) from exc
        if self._last_call is not None and now < self._last_call:
            raise AudioFault("clock_backwards", "audio receipt clock moved backwards")
        if not isinstance(data, bytes):
            raise AudioFault("invalid_pcm", "capture input must be bytes")
        self._last_call = now
        if not data:
            return []
        if self._last_data is not None and now - self._last_data > self.config.max_latency_s:
            raise AudioFault("capture_gap", "PCM delivery gap exceeds max_latency_s; timeline not rebased")
        if len(self._pending) + len(data) > self.config.max_latency_s * BYTES_PER_SECOND:
            raise AudioFault("backlog", "PCM batch exceeds the configured latency budget")
        if self._pending_since is not None and now-self._pending_since > self.config.max_latency_s:
            raise AudioFault("stale_audio", "partial PCM frame became stale")
        if not self._pending:
            self._pending_since = now
        self._pending.extend(data)
        self._last_data = now
        complete_frames = len(self._pending) // FRAME_BYTES
        if not complete_frames:
            return []
        if self._anchor is None:
            complete_samples = len(self._pending) // (CHANNELS * 2)
            self._anchor = now - complete_samples / RATE
        first_end = self._anchor + (self._sequence + 1) * FRAME_S
        final_end = self._anchor + (self._sequence + complete_frames) * FRAME_S
        if now-first_end > self.config.max_latency_s + EPSILON:
            raise AudioFault("stale_audio", "sample timeline is older than the latency budget")
        if final_end-now > self.config.max_latency_s + EPSILON:
            raise AudioFault("clock_discontinuity", "sample timeline advanced implausibly ahead of host time")
        result = []
        for _ in range(complete_frames):
            raw = bytes(self._pending[:FRAME_BYTES])
            del self._pending[:FRAME_BYTES]
            stereo = struct.unpack("<" + "h" * (SAMPLES * CHANNELS), raw)
            samples = stereo[self.config.channel::CHANNELS]
            mono = struct.pack("<" + "h" * SAMPLES, *samples)
            rms = math.sqrt(sum(sample * sample for sample in samples) / SAMPLES) / 32768
            level = 20 * math.log10(max(rms, 1e-9))
            try:
                voiced = bool(self.vad.is_speech(mono, RATE)) and level >= self.config.min_dbfs
            except Exception as exc:
                raise AudioFault("vad_error", f"WebRTC VAD failed: {type(exc).__name__}: {exc}") from exc
            self._sequence += 1
            start = self._anchor + (self._sequence - 1) * FRAME_S
            end = self._anchor + self._sequence * FRAME_S
            result.append(TimedAudio(AudioFrame(start, end, AudioStatus.VOICED if voiced
                                                else AudioStatus.UNVOICED),
                                     now, self._sequence, level))
        self._pending_since = now if self._pending else None
        return result


class ArecordAudioSource:
    """Single-owner Linux source; constructor/import do not open hardware.

    A fatal read or timing error is sticky and closes the process. Restart by
    constructing a new source with a new stderr path and fresh VAD/timeline.
    Raw PCM is not persisted. stderr is retained for diagnosis.
    """

    def __init__(self, config: AudioConfig, stderr_path: Path, *,
                 clock: Callable[[], float] = time.monotonic,
                 process_factory: Callable[..., Any] = subprocess.Popen,
                 vad_factory: Callable[[int], Any] | None = None,
                 framer_factory: Callable[[AudioConfig, Callable[[int], Any]], Any] | None = None):
        self.config, self.stderr_path = config, Path(stderr_path)
        self.clock, self.process_factory, self.vad_factory = clock, process_factory, vad_factory
        # Optional diagnostics reuse the exact capture/watchdog/cleanup path.
        # A custom framer exposes feed, pending_bytes, pending_since and returns
        # records with frame.end_s. The normal experiment path is unchanged.
        self.framer_factory = framer_factory
        self.proc: Any = None
        self._stderr: Any = None
        self._framer: Any = None
        self._held: deque[Any] = deque()
        self._started_at: float | None = None
        self._last_complete: float | None = None
        self._last_poll: float | None = None
        self._last_clock: float | None = None
        self._ever_started = False
        self._closed = False
        self._cleanup_complete = False
        self._fault: AudioFault | None = None

    @property
    def ready(self) -> bool:
        return self._last_complete is not None and not self._closed and self._fault is None

    def _now(self) -> float:
        try:
            now = _finite_number("capture clock", self.clock())
        except ValueError as exc:
            raise AudioFault("invalid_timestamp", str(exc)) from exc
        if self._last_clock is not None and now < self._last_clock:
            raise AudioFault("clock_backwards", "capture monotonic clock moved backwards")
        self._last_clock = now
        return now

    def start(self) -> None:
        if self._ever_started or self._closed:
            raise AudioFault("already_started", "create a new audio source for every start/restart")
        if not sys.platform.startswith("linux"):
            raise AudioFault("unsupported_platform", "real ALSA capture is enabled only on Linux; use fake streams on Windows")
        self._ever_started = True
        try:
            factory = self.vad_factory
            if factory is None:
                import webrtcvad
                factory = webrtcvad.Vad
            self._framer = (PCMFramer(self.config, factory(self.config.vad_mode))
                            if self.framer_factory is None else
                            self.framer_factory(self.config, factory))
            self._stderr = self.stderr_path.open("xb")
            command = ["arecord", "-q", "-D", self.config.device, "-t", "raw", "-f", "S16_LE",
                       "-r", str(RATE), "-c", str(CHANNELS),
                       f"--buffer-size={self.config.buffer_frames}", "--fatal-errors", "-"]
            self._started_at = self._now()
            self.proc = self.process_factory(command, stdin=subprocess.DEVNULL,
                                             stdout=subprocess.PIPE, stderr=self._stderr, bufsize=0)
            if self.proc.stdout is None:
                raise AudioFault("capture_start", "arecord did not provide a stdout pipe")
            os.set_blocking(self.proc.stdout.fileno(), False)
            self._check_process()
        except Exception as exc:
            fault = exc if isinstance(exc, AudioFault) else AudioFault(
                "capture_start", f"{type(exc).__name__}: {exc}")
            self._fault = fault
            try:
                self.close()
            except AudioFault as cleanup:
                self._fault = AudioFault("cleanup_error", f"After {fault}: {cleanup.detail}")
                raise self._fault from exc
            if fault is exc:
                raise
            raise fault from exc

    def _check_process(self) -> None:
        code = self.proc.poll()
        if code is not None:
            raise AudioFault("device_exit", f"arecord exited with code {code}; see {self.stderr_path}")

    def _check_progress(self, now: float) -> None:
        if self._last_complete is None:
            if now-self._started_at > self.config.startup_timeout_s:
                raise AudioFault("startup_timeout", "no complete PCM frame arrived before the startup deadline")
        elif now-self._last_complete > self.config.timeout_s:
            raise AudioFault("timeout", "no complete PCM frame arrived before the audio deadline")
        if self._last_complete is not None and self._last_poll is not None:
            if now-self._last_poll > self.config.max_latency_s:
                raise AudioFault("capture_gap", "capture polling stalled beyond the latency budget")
        if self._framer.pending_since is not None and now-self._framer.pending_since > self.config.max_latency_s:
            raise AudioFault("stale_audio", "unfinished PCM frame exceeded the latency budget")
        if self._held and now-self._held[0].frame.end_s > self.config.max_latency_s:
            raise AudioFault("stale_audio", "queued classified audio became stale")

    def poll(self) -> list[Any]:
        if self._fault is not None:
            raise self._fault
        if self.proc is None or self._closed:
            raise AudioFault("not_started", "audio source is not running")
        try:
            return self._poll()
        except Exception as exc:
            fault = exc if isinstance(exc, AudioFault) else AudioFault(
                "capture_error", f"{type(exc).__name__}: {exc}")
            self._fault = fault
            self._held.clear()
            try:
                self.close()
            except AudioFault as cleanup:
                self._fault = AudioFault("cleanup_error", f"After {fault}: {cleanup.detail}")
                raise self._fault from exc
            if fault is exc:
                raise
            raise fault from exc

    def _poll(self) -> list[Any]:
        now = self._now()
        self._check_process()
        self._check_progress(now)
        self._last_poll = now
        fd = self.proc.stdout.fileno()
        wait = self.config.poll_s
        if self._held:
            wait = min(wait, max(0.0, self._held[0].frame.end_s-now))
        available, _, _ = select.select([fd], [], [], wait)
        data = bytearray()
        budget = int(self.config.max_latency_s * BYTES_PER_SECOND)
        while available:
            remaining = budget - len(data) - self._framer.pending_bytes - len(self._held) * FRAME_BYTES
            if remaining < 0:
                raise AudioFault("backlog", "capture backlog exceeds the latency budget")
            try:
                block = os.read(fd, min(FRAME_BYTES * 8, remaining + 1))
            except BlockingIOError:
                break
            if not block:
                raise AudioFault("eof", f"arecord pipe closed; see {self.stderr_path}")
            data.extend(block)
            if len(data) + self._framer.pending_bytes + len(self._held) * FRAME_BYTES > budget:
                raise AudioFault("backlog", "capture backlog exceeds the latency budget")
            available, _, _ = select.select([fd], [], [], 0)
        self._check_process()
        received = self._now()
        # Deadline checks precede decoding, so a stale catch-up read cannot
        # disguise a timeout or an event-loop stall as fresh capture.
        self._check_progress(received)
        if data:
            frames = self._framer.feed(bytes(data), received)
            self._held.extend(frames)
            if frames:
                self._last_complete = received
        now = self._now()
        self._check_process()
        self._check_progress(now)
        ready = []
        while self._held and self._held[0].frame.end_s <= now:
            ready.append(self._held.popleft())
        return ready

    def close(self) -> None:
        if self._cleanup_complete:
            return
        # Faulted/closed sources can never deliver again, but a failed reap
        # must remain retryable instead of pretending cleanup succeeded.
        self._closed = True
        self._held.clear()
        errors = []
        try:
            if self.proc is not None:
                if self.proc.poll() is None:
                    try:
                        self.proc.terminate()
                    except ProcessLookupError:
                        pass
                    except OSError as exc:
                        errors.append(str(exc))
                try:
                    self.proc.wait(timeout=1.5)
                except subprocess.TimeoutExpired:
                    self.proc.kill()
                    self.proc.wait(timeout=1.5)
        except Exception as exc:
            errors.append(str(exc))
        finally:
            for stream in (getattr(self.proc, "stdout", None), self._stderr):
                if stream is not None:
                    try:
                        stream.close()
                    except Exception as exc:
                        errors.append(str(exc))
        if errors:
            raise AudioFault("cleanup_error", "; ".join(errors))
        self._cleanup_complete = True
