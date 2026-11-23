"""RealSense + existing YuNet helper, with a latest-only observation handoff.

All optional imports and device access occur in a spawned camera process.
The parent only receives small observations; a blocked native SDK call cannot
leave a native camera thread alive inside the experiment controller. This module can be imported by offline tests without OpenCV, NumPy,
pyrealsense2, or camera access. Face track IDs indicate spatial continuity;
they are not biometric identities or proof of who is standing on a marker.
"""
from __future__ import annotations

import json
import math
import multiprocessing
import os
from pathlib import Path
import socket
import time
from typing import Any, Callable

from .models import FaceObservation


class VisionFault(RuntimeError):
    pass


class ObservationBuilder:
    """Small, hardware-independent conversion and continuity layer.

    ``depth_at_face`` and ``deproject`` are injected so tests can verify
    invalid-depth and ambiguity handling without loading the SDK or images.
    """

    def __init__(self, intrinsics: Any, depth_at_face: Callable,
                 deproject: Callable, *, face_score: float = .75,
                 max_gap_s: float = .15, max_image_jump: float = .15):
        self.intrinsics = intrinsics
        self.depth_at_face = depth_at_face
        self.deproject = deproject
        self.face_score = face_score
        self.max_gap_s = max_gap_s
        self.max_image_jump = max_image_jump
        values = (intrinsics.width, intrinsics.height, intrinsics.fx,
                  intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
        if not all(math.isfinite(float(x)) for x in values):
            raise VisionFault("invalid camera intrinsics")
        if min(intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy) <= 0:
            raise VisionFault("nonpositive camera intrinsics")
        self.sequence = 0
        self.last_frame_number: int | None = None
        self.last_time: float | None = None
        self.next_track = 0
        self.previous: tuple[float, float, float, float, int] | None = None

    def build(self, faces: list[dict], depth: Any, scale: float,
              time_s: float, frame_number: int,
              device_timestamp_ms: float | None = None) -> FaceObservation | None:
        if not math.isfinite(time_s):
            raise VisionFault("nonfinite camera observation time")
        if self.last_time is not None and time_s < self.last_time:
            raise VisionFault("camera host clock moved backwards")
        if self.last_frame_number is not None:
            if frame_number == self.last_frame_number:
                return None  # Duplicate RGB must not refresh freshness/stability.
            if frame_number < self.last_frame_number:
                raise VisionFault("camera RGB frame counter moved backwards")
        self.last_frame_number = frame_number
        self.last_time = time_s
        self.sequence += 1
        eligible = []
        for face in faces:
            try:
                x, y = map(float, face["aim"])
                score = float(face["score"])
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
            if (all(math.isfinite(v) for v in (x, y, score))
                    and score >= self.face_score
                    and 0 <= x < self.intrinsics.width
                    and 0 <= y < self.intrinsics.height):
                eligible.append(face)

        def invalid(reason: str, score: float = 0) -> FaceObservation:
            self.previous = None
            return FaceObservation(time_s, self.sequence, None, len(eligible),
                                   None, None, None, score, frame_number,
                                   device_timestamp_ms, reason)

        if not eligible:
            return invalid("NO_FACE")
        if len(eligible) != 1:
            return invalid("MULTIPLE_FACES")
        face = eligible[0]
        score = float(face["score"])
        z = self.depth_at_face(depth, face, scale)
        if z is None or not math.isfinite(z) or not .35 <= z <= 3.5:
            return invalid("INVALID_DEPTH", score)
        x, y = map(float, face["aim"])
        xyz = tuple(float(v) for v in self.deproject(self.intrinsics, [x, y], z))
        if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz) or xyz[2] <= 0:
            return invalid("INVALID_DEPROJECTION", score)

        previous = self.previous
        if (previous is None or time_s - previous[3] > self.max_gap_s
                or math.hypot((x-previous[0])/self.intrinsics.width,
                              (y-previous[1])/self.intrinsics.height) > self.max_image_jump
                or abs(z-previous[2]) > .25):
            self.next_track += 1
            track_id = self.next_track
        else:
            track_id = previous[4]
        self.previous = (x, y, z, time_s, track_id)
        return FaceObservation(
            time_s, self.sequence, track_id, 1, xyz,
            math.degrees(math.atan2(xyz[0], xyz[2])),
            math.degrees(math.atan2(xyz[1], xyz[2])),
            score, frame_number, device_timestamp_ms, None)


# A datagram is delivered whole or not at all. Unlike Connection.recv() after
# poll(), this cannot wait for the rest of a message from a killed writer.
_PACKET_BYTES = 16384
_MAX_DRAIN = 256


def _camera_worker_main(settings, channel, preview_buffer, preview_lock,
                        observation_buffer, observation_lock):
    from .camera_worker import run_camera_worker
    run_camera_worker(settings, channel, preview_buffer, preview_lock,
                      observation_buffer, observation_lock)


class VisionCamera:
    """Linux camera process with bounded, explicitly acknowledged shutdown.

    `clock` is only a parent-side clock seam for offline tests. Live workers
    use time.monotonic(), the same host clock domain on supported Linux.
    Runtime faults latch. A forced stop is a cleanup error, never success.
    """

    STARTUP_S = 5.0
    STALE_S = 1.0
    CLOSE_GRACE_S = 2.0
    TERMINATE_S = 1.0
    KILL_S = 1.0
    _worker_target = staticmethod(_camera_worker_main)

    def __init__(self, model_path: str | Path, emit: Callable, *,
                 camera_serial: str = "", width: int = 640, height: int = 480,
                 fps: int = 30, face_score: float = .75, preview: bool = False,
                 clock: Callable[[], float] = time.monotonic):
        self.model_path = Path(model_path).expanduser().absolute()
        self.emit = emit
        self.camera_serial = camera_serial
        self.width, self.height, self.fps = width, height, fps
        self.face_score, self.preview = face_score, preview
        self.clock = clock
        if width <= 0 or height <= 0 or fps <= 0 or not 0 < face_score < 1:
            raise ValueError("invalid camera dimensions/fps/face score")
        self._ctx = multiprocessing.get_context("spawn")
        self._process = None
        self._channel = None
        self._preview_buffer = None
        self._preview_lock = None
        self._observation_buffer = None
        self._observation_lock = None
        self._latest: FaceObservation | None = None
        self._fault: VisionFault | None = None
        self._started_at: float | None = None
        self._ever_started = False
        self._closed = False
        self._close_completed = False
        self._stop_requested = False
        self._worker_finished = False
        self._cleanup_received = False
        self._cleanup_ok = False
        self._cleanup_error: VisionFault | None = None
        self._event_logging_error: VisionFault | None = None

    @property
    def ready(self) -> bool:
        self._pump()
        return (self._latest is not None and self._fault is None
                and not self._closed and not self._worker_finished)

    @property
    def fault(self) -> VisionFault | None:
        self._pump()
        return self._fault

    def start(self) -> None:
        if self._ever_started or self._closed:
            raise VisionFault("create a new VisionCamera for each session")
        if not self.model_path.is_file():
            raise VisionFault(f"YuNet model not found: {self.model_path}")
        self._ever_started = True
        self._started_at = self.clock()
        child_channel = None
        try:
            self._channel, child_channel = socket.socketpair(
                socket.AF_UNIX, socket.SOCK_DGRAM)
            for channel in (self._channel, child_channel):
                channel.setblocking(False)
                channel.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
                channel.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
            self._observation_buffer = self._ctx.RawArray("B", _PACKET_BYTES + 4)
            self._observation_lock = self._ctx.Lock()
            if self.preview:
                self._preview_buffer = self._ctx.RawArray("B", self.width*self.height*3)
                self._preview_lock = self._ctx.Lock()
            settings = dict(model_path=str(self.model_path), camera_serial=self.camera_serial,
                            width=self.width, height=self.height, fps=self.fps,
                            face_score=self.face_score, preview=self.preview,
                            parent_pid=os.getpid())
            self._process = self._ctx.Process(
                target=self._worker_target,
                args=(settings, child_channel, self._preview_buffer, self._preview_lock,
                      self._observation_buffer, self._observation_lock),
                name="mox-camera", daemon=True)
            self._process.start()
            self._emit("CAMERA_WORKER_STARTED", time_s=self.clock(),
                       pid=self._process.pid, worker_mode="spawn_process",
                       ipc="lifecycle_datagrams_and_shared_latest_observation",
                       native_sdk_in_parent=False)
        except BaseException as exc:
            self._set_fault(VisionFault(f"camera worker start failed: {exc}"))
            raise self._fault
        finally:
            if child_channel is not None:
                child_channel.close()

    def _request_stop(self) -> None:
        if self._stop_requested:
            return
        self._stop_requested = True
        if self._channel is not None:
            try:
                self._channel.send(b'{"kind":"stop"}')
            except OSError:
                # A dead peer or a blocked startup is handled by bounded close.
                pass

    def _set_fault(self, error: BaseException) -> None:
        if self._fault is None:
            self._fault = (error if isinstance(error, VisionFault) else
                           VisionFault(f"{type(error).__name__}: {error}"))
        self._request_stop()

    def _emit(self, event: str, **fields) -> None:
        try:
            self.emit(event, **fields)
        except BaseException as exc:
            self._event_logging_error = VisionFault(f"camera event logging failed: {exc}")
            self._set_fault(self._event_logging_error)

    def _pump(self) -> None:
        """Drain without blocking; only the most recent observation survives."""
        if self._channel is not None:
            for _ in range(_MAX_DRAIN):
                try:
                    data = self._channel.recv(_PACKET_BYTES + 1)
                except BlockingIOError:
                    break
                except OSError as exc:
                    if not self._closed:
                        self._set_fault(VisionFault(f"camera IPC read failed: {exc}"))
                    break
                try:
                    if len(data) > _PACKET_BYTES:
                        raise ValueError("oversized packet")
                    msg = json.loads(data)
                    if not isinstance(msg, dict):
                        raise ValueError("message must be an object")
                    kind = msg["kind"]
                    if kind == "observation":
                        self._accept_observation(msg["observation"])
                    elif kind == "event":
                        self._emit(msg["event"], **msg["fields"])
                    elif kind == "fault":
                        self._set_fault(VisionFault(msg["reason"]))
                        self._emit("CAMERA_FAULT", time_s=msg["time_s"],
                                   reason=msg["reason"], worker_mode="spawn_process")
                    elif kind == "closed":
                        self._worker_finished = True
                        self._cleanup_received = True
                        self._cleanup_ok = msg["cleanup_ok"] is True
                        if not self._cleanup_ok:
                            self._cleanup_error = VisionFault(
                                "camera cleanup failed: " + msg.get("reason", "unknown"))
                            self._set_fault(self._cleanup_error)
                        self._emit("CAMERA_WORKER_CLOSED", time_s=msg["time_s"],
                                   cleanup_ok=self._cleanup_ok,
                                   pipeline_started=msg.get("pipeline_started", False),
                                   reason=msg.get("reason"))
                        if not self._closed and self._fault is None:
                            self._set_fault(VisionFault("camera worker stopped unexpectedly"))
                    else:
                        raise ValueError("unknown message kind")
                except (ValueError, TypeError, KeyError, OverflowError) as exc:
                    self._set_fault(VisionFault(f"invalid camera IPC message: {exc}"))
                    break
        self._read_shared_observation()
        proc = self._process
        if proc is not None and proc.pid is not None and proc.exitcode is not None:
            if not self._closed and self._fault is None:
                self._set_fault(VisionFault(
                    f"camera worker exited unexpectedly with code {proc.exitcode}"))

    def _accept_observation(self, payload) -> None:
        if not isinstance(payload, dict):
            raise ValueError("observation must be an object")
        if payload.get("point_m") is not None:
            payload["point_m"] = tuple(payload["point_m"])
        obs = FaceObservation(**payload)
        if (isinstance(obs.time_s, bool) or not isinstance(obs.time_s, (int, float))
                or not math.isfinite(obs.time_s)
                or type(obs.sequence) is not int or obs.sequence <= 0):
            raise ValueError("invalid observation timestamp/sequence")
        if self._latest is not None:
            if obs.sequence == self._latest.sequence:
                return  # Same latest-value slot, not a fresh frame.
            if obs.sequence < self._latest.sequence:
                raise ValueError("camera IPC sequence moved backwards")
            if obs.time_s < self._latest.time_s:
                raise ValueError("camera IPC host time moved backwards")
        self._latest = obs

    def _read_shared_observation(self) -> None:
        if (self._closed or self._observation_buffer is None
                or self._observation_lock is None):
            return
        if not self._observation_lock.acquire(timeout=.01):
            return  # A killed owner cannot block shutdown or liveness checks.
        try:
            data = memoryview(self._observation_buffer).cast("B")
            count = int.from_bytes(data[:4], "little")
            if count == 0:
                return
            if not 0 < count <= _PACKET_BYTES:
                raise ValueError("invalid latest-observation size")
            packet = bytes(data[4:4+count])
        except (ValueError, TypeError) as exc:
            self._set_fault(VisionFault(f"invalid camera shared observation: {exc}"))
            return
        finally:
            self._observation_lock.release()
        try:
            self._accept_observation(json.loads(packet))
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self._set_fault(VisionFault(f"invalid camera shared observation: {exc}"))

    def latest(self, now: float | None = None) -> FaceObservation | None:
        now = self.clock() if now is None else now
        if not math.isfinite(now):
            raise VisionFault("nonfinite camera poll time")
        self._pump()
        if self._fault is not None:
            raise self._fault
        if self._closed:
            raise VisionFault("camera already closed")
        if self._started_at is None:
            raise VisionFault("camera not started")
        latest = self._latest
        if latest is None:
            if now - self._started_at > self.STARTUP_S:
                self._set_fault(VisionFault("camera startup exceeded 5 seconds"))
                raise self._fault
            return None
        if latest.time_s > now:
            return None  # A caller sampled `now` before the worker snapshot.
        if now - latest.time_s > self.STALE_S:
            self._set_fault(VisionFault("camera observation stale for more than 1 second"))
            raise self._fault
        return latest

    def preview_frame(self):
        """Optional shared-memory RGB copy; a killed lock owner cannot block us."""
        if self._closed or self._preview_buffer is None or self._preview_lock is None:
            return None
        if not self._preview_lock.acquire(timeout=.01):
            return None
        try:
            if self._latest is None:
                return None
            import numpy as np  # Parent GUI-only path, never offline/headless.
            return np.frombuffer(self._preview_buffer, dtype=np.uint8).reshape(
                self.height, self.width, 3).copy()
        finally:
            self._preview_lock.release()

    def _join_draining(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while self._process.is_alive():
            self._pump()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            self._process.join(timeout=min(.02, remaining))
        self._pump()

    def close(self) -> None:
        if self._close_completed:
            if self._cleanup_error is not None:
                raise self._cleanup_error
            return
        self._closed = True
        self._request_stop()
        proc = self._process
        errors = []
        forced = None

        def attempt(label, action):
            try:
                return action()
            except BaseException as exc:
                # In particular, a second Ctrl-C must not bypass terminate/kill
                # and leave native workers running when the parent exits.
                errors.append(f"{label}: {type(exc).__name__}: {exc}")
                return None

        def is_alive():
            try:
                return proc is not None and proc.pid is not None and proc.is_alive()
            except BaseException as exc:
                errors.append(f"camera liveness check: {type(exc).__name__}: {exc}")
                return True  # Unconfirmed death cannot be reported as cleanup.

        has_child = proc is not None and proc.pid is not None
        alive = is_alive() if has_child else False
        if has_child:
            attempt("camera graceful join", lambda: self._join_draining(self.CLOSE_GRACE_S))
            alive = is_alive()
            if alive:
                forced = "terminate"
                attempt("camera terminate", proc.terminate)
                attempt("camera terminate join", lambda: self._join_draining(self.TERMINATE_S))
                alive = is_alive()
            if alive:
                forced = "kill"
                attempt("camera kill", proc.kill)
                attempt("camera kill join", lambda: self._join_draining(self.KILL_S))
                alive = is_alive()
            # Drain a final acknowledgement even after an interrupted join.
            attempt("camera final lifecycle drain", self._pump)
            if alive:
                errors.append("camera worker still alive after kill deadline")
            if forced:
                errors.append(f"camera graceful shutdown timed out or was interrupted; worker {forced} required")
            exitcode = attempt("camera exit-code read", lambda: proc.exitcode)
            if exitcode not in (None, 0) and not forced:
                errors.append(f"camera worker exited with code {exitcode}")
            if not self._cleanup_received and not forced:
                errors.append("camera worker cleanup acknowledgement missing")
            if self._cleanup_error is not None:
                errors.append(str(self._cleanup_error))
            self._emit("CAMERA_SHUTDOWN", time_s=self.clock(),
                       worker_pid=proc.pid, exitcode=exitcode,
                       forced=forced is not None, escalation=forced,
                       cleanup_acknowledged=self._cleanup_received,
                       cleanup_ok=self._cleanup_ok and not errors
                       and self._event_logging_error is None)
            if self._event_logging_error is not None:
                errors.append(str(self._event_logging_error))

        if not alive:
            if self._channel is not None:
                attempt("camera IPC close", self._channel.close)
                self._channel = None
            self._preview_buffer = None
            self._preview_lock = None
            self._observation_buffer = None
            self._observation_lock = None
            if proc is not None:
                attempt("camera process handle close", proc.close)
                self._process = None
            self._close_completed = True
        # If the OS has not reaped a process, retain its handles so close() can
        # be retried. Never pretend that a still-live worker was cleaned up.
        if errors:
            self._cleanup_error = VisionFault("; ".join(errors))
            self._set_fault(self._cleanup_error)
            raise self._cleanup_error
