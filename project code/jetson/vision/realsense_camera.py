"""A single owner for the SDK pipeline; a one-frame mailbox for consumers."""
import logging
import math
import threading
import time
from dataclasses import dataclass, replace

import numpy as np

from .types import Detection, Frame

LOG = logging.getLogger(__name__)


class RealSenseCamera:
    def __init__(self, serial=None, width=640, height=480, fps=30, timeout_ms=1000):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError("pyrealsense2 is missing. Run jetson.diagnose and follow jetson/README.md.") from exc
        self.rs = rs
        self.serial = serial
        self.width, self.height, self.fps = width, height, fps
        self.timeout_ms = timeout_ms
        self.pipeline = None
        self.last_numbers = None

    def open(self):
        rs = self.rs
        ctx = rs.context()
        devices = list(ctx.query_devices())
        if not self.serial:
            if len(devices) != 1:
                raise RuntimeError(f"Expected one RealSense, found {len(devices)}; use --serial for multiple cameras.")
            # Keep this identity across reconnects. Never silently switch cameras.
            self.serial = devices[0].get_info(rs.camera_info.serial_number)
        pipeline = rs.pipeline(ctx)
        config = rs.config()
        config.enable_device(self.serial)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        started = False
        try:
            profile = pipeline.start(config)
            started = True
            device = profile.get_device()
            self.depth_scale = float(device.first_depth_sensor().get_depth_scale())
            if not math.isfinite(self.depth_scale) or self.depth_scale <= 0:
                raise RuntimeError("Camera reported an invalid depth scale")
            self.info = {"serial": self.serial}
            for name in ("name", "firmware_version", "usb_type_descriptor"):
                key = getattr(rs.camera_info, name)
                if device.supports(key):
                    self.info[name] = device.get_info(key)
            self.align = rs.align(rs.stream.color)
            self.last_numbers = None
            self.pipeline = pipeline
            self.context = ctx
            LOG.info("Opened %s at %sx%s@%s", self.info, self.width, self.height, self.fps)
            if str(self.info.get("usb_type_descriptor", "")).startswith("2"):
                LOG.warning("Camera reports USB 2.x. Check cable/port with lsusb -t; requested profiles may fail.")
        except Exception:
            if started:
                pipeline.stop()
            raise

    def read(self):
        deadline = time.monotonic() + self.timeout_ms / 1000
        while time.monotonic() < deadline:
            remaining = max(1, int((deadline - time.monotonic()) * 1000))
            ok, frames = self.pipeline.try_wait_for_frames(remaining)
            received = time.monotonic()
            if not ok:
                break
            # Timestamp the arrival before doing alignment, so its cost counts as age.
            color_raw, depth_raw = frames.get_color_frame(), frames.get_depth_frame()
            if not color_raw or not depth_raw:
                continue
            numbers = (color_raw.get_frame_number(), depth_raw.get_frame_number())
            if self.last_numbers and any(a <= b for a, b in zip(numbers, self.last_numbers)):
                continue  # A repeated depth or RGB frame must not renew freshness.
            ct, dt = float(color_raw.get_timestamp()), float(depth_raw.get_timestamp())
            cc = str(color_raw.get_frame_timestamp_domain())
            dc = str(depth_raw.get_frame_timestamp_domain())
            if not math.isfinite(ct) or not math.isfinite(dt):
                raise RuntimeError("Non-finite camera timestamp")
            if cc == dc and abs(ct - dt) > 100:
                raise RuntimeError("RGB/depth timestamp skew exceeds 100 ms")
            aligned = self.align.process(frames)
            color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
            if not color or not depth:
                continue
            rgb = np.asanyarray(color.get_data()).copy()
            z16 = np.asanyarray(depth.get_data()).copy()
            if rgb.shape[:2] != z16.shape or rgb.ndim != 3 or rgb.shape[2] != 3:
                raise RuntimeError("RGB/depth alignment shape mismatch")
            rgb.flags.writeable = z16.flags.writeable = False
            self.last_numbers = numbers
            return Frame(rgb, z16, self.depth_scale, received, *numbers, ct, dt, cc, dc,
                         "realsense", dict(self.info))
        raise TimeoutError("No fresh RGB + depth pair within camera timeout")

    def close(self):
        if self.pipeline is not None:
            pipeline, self.pipeline = self.pipeline, None
            pipeline.stop()


class SyntheticCamera:
    """Deterministic fixtures, explicitly labeled; never used as real-camera fallback."""
    def __init__(self, fps=30, width=640, height=480):
        self.fps, self.width, self.height = fps, width, height
        self.number = 0
        self.next_frame = 0.0

    def open(self):
        self.next_frame = time.monotonic()

    def read(self):
        time.sleep(max(0, self.next_frame - time.monotonic()))
        now = time.monotonic()
        self.next_frame = now + 1 / self.fps
        self.number += 1
        phase = self.number / self.fps
        w, h = self.width, self.height
        color = np.full((h, w, 3), 25, dtype=np.uint8)
        depth = np.full((h, w), 4000, dtype=np.uint16)
        x = int(w * (0.35 + 0.18 * math.sin(phase * 0.7)))
        box = (x, int(h * .15), x + int(w * .22), int(h * .9))
        x1, y1, x2, y2 = box
        color[y1:y2, x1:x2] = (80, 180, 90)
        depth[y1:y2, x1:x2] = int(1600 + 250 * math.sin(phase))
        depth[y1:y2:7, x1:x2:7] = 0
        # Brief depth outage every 8s exercises invalid-depth behavior.
        if 6 <= phase % 8 < 7:
            depth[y1:y2, x1:x2] = 0
        detections = (Detection(tuple(map(float, box)), 1.0, "synthetic_fixture"),)
        # Brief occlusion: no detection, never replay the old target.
        if 4 <= phase % 8 < 4.5:
            detections = ()
            color[y1:y2, x1:x2] = 25
            depth[y1:y2, x1:x2] = 4000
        color.flags.writeable = depth.flags.writeable = False
        return Frame(color, depth, .001, now, self.number, self.number, phase * 1000,
                     phase * 1000, "synthetic", "synthetic", "simulation",
                     {"name": "SYNTHETIC TEST DATA"}, detections)

    def close(self):
        pass


@dataclass(frozen=True)
class CaptureStatus:
    state: str = "starting"
    generation: int = 0
    sequence: int = 0
    restarts: int = 0
    capture_fps: float = 0.0
    error: str | None = None
    frame: Frame | None = None


class CaptureWorker:
    def __init__(self, camera, retry_initial=.5, retry_max=5.0):
        self.camera = camera
        self.retry_initial, self.retry_max = retry_initial, retry_max
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._status = CaptureStatus()
        self._thread = threading.Thread(target=self._run, name="rgbd-capture", daemon=True)

    def start(self):
        self._thread.start()

    def snapshot(self):
        with self._lock:
            return self._status

    def _publish(self, **fields):
        with self._lock:
            self._status = replace(self._status, **fields)

    def _run(self):
        retry = self.retry_initial
        generation = sequence = restarts = 0
        try:
            while not self._stop.is_set():
                try:
                    self._publish(state="connecting", frame=None)
                    self.camera.open()
                    generation += 1
                    previous = None
                    fps = 0.0
                    while not self._stop.is_set():
                        frame = self.camera.read()
                        if previous is not None and frame.received_at > previous:
                            instant = 1 / (frame.received_at - previous)
                            fps = instant if fps == 0 else .1 * instant + .9 * fps
                        previous = frame.received_at
                        sequence += 1
                        self._publish(state="streaming", frame=frame, generation=generation,
                                      sequence=sequence, restarts=restarts, capture_fps=fps, error=None)
                        retry = self.retry_initial
                except Exception as exc:
                    restarts += 1
                    self._publish(state="reconnecting", frame=None, error=str(exc), restarts=restarts)
                    LOG.warning("Camera unavailable (%s); retry in %.1fs", exc, retry)
                finally:
                    try:
                        self.camera.close()
                    except Exception as exc:
                        LOG.warning("Camera cleanup: %s", exc)
                if self._stop.wait(retry):
                    break
                retry = min(self.retry_max, retry * 2)
        finally:
            self._publish(state="stopped", frame=None)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=5)
        if self._thread.is_alive():
            LOG.error("SDK call did not return during shutdown; capture thread remains blocked")
            return False
        return True
