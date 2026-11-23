"""Threaded VAD, prerecorded playback and append-only engineering logs.

No microphone/camera/serial device is opened by importing this module.
"""
from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid
import wave

from .vendor.live_audio import ArecordAudioSource, AudioConfig


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def wav_info(path: Path) -> dict:
    try:
        with wave.open(str(path), "rb") as stream:
            rate, frames = stream.getframerate(), stream.getnframes()
            channels, width = stream.getnchannels(), stream.getsampwidth()
            if (not 8000 <= rate <= 192000 or frames <= 0 or channels not in (1, 2)
                    or width not in (1, 2, 3, 4) or stream.getcomptype() != "NONE"):
                raise ValueError(f"需要非空、单/双声道、8–192 kHz 未压缩 PCM WAV：{path}")
            duration = frames / rate
            if duration > 90:
                raise ValueError(f"单段固定音频超过 90 秒：{path}")
            # A valid-looking header is insufficient if the PCM data is truncated.
            actual_bytes = 0
            while True:
                block = stream.readframes(16384)
                if not block:
                    break
                actual_bytes += len(block)
            if actual_bytes != frames * channels * width:
                raise ValueError(f"WAV PCM 数据不完整：{path}")
    except (wave.Error, EOFError) as exc:
        raise ValueError(f"无效 WAV：{path}: {exc}") from exc
    return {"path": str(Path(path).absolute()), "duration_s": duration,
            "sample_rate": rate, "channels": channels,
            "sample_width": width, "sha256": file_sha256(path)}


def _finite_time(value: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("invalid monotonic time")
    return float(value)


class Journal:
    """One fresh directory per process; thread-safe JSONL with sticky failures."""
    def __init__(self, output: Path, trial_id: str, pair_id: str, *, clock=time.monotonic):
        self.clock = clock
        self.trial_id, self.pair_id = trial_id, pair_id
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        self.directory = Path(output) / f"run_{stamp}_{uuid.uuid4().hex[:8]}"
        self.directory.mkdir(parents=True, exist_ok=False)
        self._file = (self.directory / "events.jsonl").open("x", encoding="utf-8")
        self._lock = threading.Lock()
        self._sequence = 0
        self.fault = None
        self.closed = False

    def emit(self, event: str, **fields):
        with self._lock:
            if self.fault:
                raise RuntimeError(f"日志已失败：{self.fault}")
            if self.closed:
                raise RuntimeError("日志已关闭")
            try:
                if not isinstance(event, str) or not event:
                    raise ValueError("event must be a nonempty string")
                reserved = {"sequence", "event", "recorded_at_s", "trial_id", "pair_id"}
                if reserved.intersection(fields):
                    raise ValueError(f"reserved event fields: {sorted(reserved.intersection(fields))}")
                self._sequence += 1
                received = _finite_time(self.clock())
                at = _finite_time(fields.pop("time_s", received))
                row = {"sequence": self._sequence, "event": event, "time_s": at,
                       "recorded_at_s": received, "trial_id": self.trial_id,
                       "pair_id": self.pair_id, **fields}
                self._file.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
                self._file.flush()
            except Exception as exc:
                self.fault = str(exc)
                raise

    def write_json(self, name: str, value):
        with self._lock:
            if self.fault:
                raise RuntimeError(f"日志已失败：{self.fault}")
            if self.closed:
                raise RuntimeError("日志已关闭")
            try:
                if Path(name).name != name or not name.endswith(".json"):
                    raise ValueError("JSON filename must be a .json basename")
                path = self.directory / name
                temporary = path.with_name(path.name + ".tmp")
                data = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)
                with temporary.open("w", encoding="utf-8") as stream:
                    stream.write(data + "\n")
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, path)
            except Exception as exc:
                self.fault = str(exc)
                raise

    def close(self):
        with self._lock:
            if not self.closed:
                self.closed = True
                try:
                    self._file.close()
                except Exception as exc:
                    self.fault = str(exc)
                    raise


class AudioCapture:
    """Keep ALSA polling independent of face inference and serial requests."""
    def __init__(self, emit, directory: Path, *, device="plughw:CARD=Array,DEV=0",
                 channel=0, vad_mode=2, min_dbfs=-48.0, clock=time.monotonic,
                 source_factory=ArecordAudioSource):
        self.emit, self.clock = emit, clock
        self.config = AudioConfig(device=device, channel=channel, vad_mode=vad_mode,
                                  min_dbfs=min_dbfs, buffer_frames=2048,
                                  startup_timeout_s=5.0, timeout_s=1.0,
                                  max_latency_s=0.25, poll_s=0.01)
        self.source = source_factory(self.config, Path(directory) / "arecord_stderr.log", clock=clock)
        self._queue = deque()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._fault = None
        self._last = None
        self._started = None
        self._closed = False

    def start(self):
        if self._thread is not None or self._closed:
            raise RuntimeError("new AudioCapture required for each trial")
        self._started = _finite_time(self.clock())
        self._thread = threading.Thread(target=self._run, daemon=True, name="mox72-audio")
        self._thread.start()

    def _run(self):
        try:
            self.source.start()
            self.emit("AUDIO_STARTED", config=self.config.to_dict(), raw_recording=False,
                      timestamp_basis="host_receipt_anchor_plus_sample_count")
            while not self._stop.is_set():
                items = self.source.poll()
                for item in items:
                    self.emit("AUDIO_CAPTURED", start_s=item.frame.start_s, end_s=item.frame.end_s,
                              status=item.frame.status.value, received_at_s=item.received_at_s,
                              audio_sequence=item.sequence, dbfs=item.dbfs)
                    with self._lock:
                        # 0.25 s latency bound is checked before draining too.
                        if len(self._queue) >= 32:
                            raise RuntimeError("AUDIO_CONTROLLER_BACKLOG")
                        self._queue.append(item)
                        self._last = item
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._fault = exc
        finally:
            try:
                self.source.close()
            except Exception as exc:
                with self._lock:
                    self._fault = exc if self._fault is None else RuntimeError(
                        f"{self._fault}; cleanup: {exc}")

    def drain(self, now: float):
        with self._lock:
            if self._fault:
                raise RuntimeError(f"AUDIO_FAILED: {self._fault}") from self._fault
            if self._closed:
                raise RuntimeError("AudioCapture already closed")
            if self._started is None:
                raise RuntimeError("AudioCapture not started")
            try:
                now = _finite_time(now)
                if self._last is None:
                    if now - self._started > 5.0:
                        raise RuntimeError("AUDIO_STARTUP_TIMEOUT")
                    return []
                if now - self._last.received_at_s > 1.0:
                    raise RuntimeError("AUDIO_STALE")
                if self._queue and now - self._queue[0].frame.end_s > 0.25:
                    raise RuntimeError("AUDIO_CONTROLLER_LATENCY")
                items = []
                # A producer may have published after the main loop sampled now.
                # Preserve those observations until the next controller tick.
                while (self._queue and self._queue[0].frame.end_s <= now
                       and self._queue[0].received_at_s <= now):
                    items.append(self._queue.popleft())
                return items
            except Exception as exc:
                self._fault = exc
                self._stop.set()
                raise

    @property
    def ready(self):
        with self._lock:
            return (self._last is not None and self._fault is None and not self._closed
                    and not self._stop.is_set())

    def close(self):
        with self._lock:
            self._closed = True
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=4.0)
            if self._thread.is_alive():
                raise RuntimeError("AudioCapture cleanup timed out")
        else:
            self.source.close()
        if self._fault is not None:
            raise RuntimeError(f"AudioCapture fault/cleanup: {self._fault}")


class WavePlayer:
    """Two pre-generated clips. No LLM/Piper loading in a running trial."""
    def __init__(self, clips: dict[str, Path], emit, directory: Path,
                 *, device="plughw:CARD=UACDemoV10,DEV=0", process_factory=subprocess.Popen):
        if not clips or set(clips) - {"welcome", "prompt"}:
            raise ValueError("fixed clip names must be welcome and/or prompt")
        self.clips = {key: Path(path).expanduser().absolute() for key, path in clips.items()}
        self.info = {key: wav_info(path) for key, path in self.clips.items()}
        self.emit, self.directory, self.device = emit, Path(directory), device
        self._factory = process_factory
        self._proc = None
        self._log = None
        self._name = None
        self._deadline = None
        self._played = set()
        self._fault = None
        self._stopped = False

    def play(self, name: str, now: float):
        if self._fault is not None:
            raise RuntimeError(f"PLAYBACK_FAULT: {self._fault}") from self._fault
        if self._stopped:
            raise RuntimeError("PLAYBACK_STOPPED: use a new player for another trial")
        if self._proc is not None:
            raise RuntimeError("PLAYBACK_ALREADY_ACTIVE")
        if name not in self.clips or name in self._played:
            raise RuntimeError(f"Invalid/repeated fixed clip: {name}")
        try:
            now = _finite_time(now)
            if wav_info(self.clips[name])["sha256"] != self.info[name]["sha256"]:
                raise RuntimeError(f"FIXED_CLIP_CHANGED: {name}")
            self._played.add(name)
            self._name = name
            self._deadline = now + self.info[name]["duration_s"] + 5.0
            self._log = (self.directory / f"aplay_{name}.log").open("xb")
            self._proc = self._factory(["aplay", "-q", "-D", self.device, str(self.clips[name])],
                                       stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                       stderr=self._log, start_new_session=True)
            self.emit("PLAYBACK_STARTED", time_s=now, clip=name, **self.info[name])
        except Exception as exc:
            self._fault = exc
            try:
                self.stop()
            except Exception as cleanup:
                self._fault = RuntimeError(f"{exc}; cleanup: {cleanup}")
            raise

    def poll(self, now: float) -> bool:
        if self._fault is not None:
            raise RuntimeError(f"PLAYBACK_FAULT: {self._fault}") from self._fault
        now = _finite_time(now)
        if self._proc is None:
            return False
        code = self._proc.poll()
        if code is None:
            if now > self._deadline:
                self._fault = RuntimeError("PLAYBACK_TIMEOUT")
                try:
                    self.stop()
                finally:
                    raise self._fault
            return True
        name = self._name
        self._proc = None
        if self._log:
            self._log.close()
            self._log = None
        if code:
            self._fault = RuntimeError(f"PLAYBACK_FAILED {name}: exit {code}; see aplay log")
            raise self._fault
        try:
            self.emit("PLAYBACK_FINISHED", time_s=now, clip=name,
                      meaning="host observed successful aplay exit, not measured acoustic offset")
        except Exception as exc:
            self._fault = exc
            raise
        return False

    def stop(self):
        self._stopped = True
        try:
            if self._proc is not None:
                if self._proc.poll() is None:
                    self._proc.terminate()
                    try:
                        self._proc.wait(timeout=0.5)
                    except subprocess.TimeoutExpired:
                        self._proc.kill()
                        self._proc.wait(timeout=0.5)
                self._proc = None
        finally:
            if self._log:
                self._log.close()
                self._log = None

    close = stop


class SimPlayer:
    def __init__(self, emit, durations=None):
        self.emit = emit
        self.durations = durations or {"welcome": 3.0, "prompt": 2.0}
        self.until = None
        self.name = None
        self.played = set()

    def play(self, name, now):
        if self.until is not None or name in self.played:
            raise RuntimeError("repeated/overlapping simulated clip")
        self.played.add(name)
        self.name = name
        self.until = now + self.durations[name]
        self.emit("PLAYBACK_STARTED", time_s=now, clip=name, simulated=True)

    def poll(self, now):
        if self.until is None:
            return False
        if now < self.until:
            return True
        self.emit("PLAYBACK_FINISHED", time_s=now, clip=self.name, simulated=True)
        self.until = None
        return False

    def stop(self):
        self.until = None

    close = stop
