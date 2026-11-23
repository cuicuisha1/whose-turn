"""Native RealSense/YuNet worker, isolated from the experiment process.

Only run_camera_worker imports hardware libraries. UNIX datagrams are bounded
and atomic; lifecycle events must be delivered or the child exits nonzero.
Observations use one shared latest-value slot, so blocking the controller for
a serial handshake does not build a queue of stale frames. No raw images are
written to disk.
"""
from __future__ import annotations

from dataclasses import asdict
import json
import math
import os
import select
import signal
import socket
import time

_PACKET_BYTES = 16384


def _send(channel, message, *, critical=False):
    data = json.dumps(message, ensure_ascii=False, allow_nan=False,
                      separators=(",", ":")).encode("utf-8")
    if len(data) > _PACKET_BYTES:
        raise ValueError("camera IPC packet exceeds bound")
    deadline = time.monotonic() + (.25 if critical else 0.)
    while True:
        try:
            channel.send(data)
            return True
        except BlockingIOError:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            select.select([], [channel], [], remaining)
        except OSError:
            return False


def _stopping(channel, parent_pid):
    if os.getppid() != parent_pid:
        return True
    try:
        command = json.loads(channel.recv(_PACKET_BYTES))
    except BlockingIOError:
        return False
    except OSError:
        return True
    if command.get("kind") != "stop":
        raise RuntimeError("invalid camera worker control message")
    return True


def _publish_observation(buffer, lock, observation):
    data = json.dumps(asdict(observation), allow_nan=False,
                      separators=(",", ":")).encode("utf-8")
    if len(data) > _PACKET_BYTES:
        raise ValueError("camera observation exceeds shared slot")
    if not lock.acquire(timeout=.01):
        return False
    try:
        slot = memoryview(buffer).cast("B")
        slot[4:4+len(data)] = data
        slot[:4] = len(data).to_bytes(4, "little")
    finally:
        lock.release()
    return True


def run_camera_worker(settings, channel, preview_buffer, preview_lock,
                      observation_buffer, observation_lock):
    # Ctrl-C is owned by the controller. It requests a graceful child stop.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    channel.setblocking(False)
    pipeline = None
    started = False
    cleanup_error = None
    delivery_failed = False
    stop_observed = False
    # NumPy views can retain SDK buffers. Clear views before their frame owners
    # when leaving the native loop, including exception and stop paths.
    frames = aligned = color = depth = bgr = depth_image = preview = None

    def emit(event, **fields):
        if not _send(channel, {"kind": "event", "event": event, "fields": fields},
                     critical=True):
            raise RuntimeError("camera lifecycle message delivery failed")

    def should_stop(checkpoint):
        nonlocal stop_observed
        if stop_observed:
            return True
        if not _stopping(channel, settings["parent_pid"]):
            return False
        stop_observed = True
        emit("CAMERA_STOP_OBSERVED", time_s=time.monotonic(),
             checkpoint=checkpoint, stop_condition="control_stop_or_parent_exit")
        return True

    def cleanup_stage(stage, **fields):
        nonlocal delivery_failed
        # Logging trouble must not skip native resource cleanup. A delivery
        # failure still makes the worker exit nonzero after attempting stop.
        try:
            delivered = _send(channel, {"kind": "event", "event": "CAMERA_CLEANUP_STAGE",
                              "fields": {"time_s": time.monotonic(), "stage": stage, **fields}},
                              critical=True)
        except BaseException:
            delivered = False
        if not delivered:
            delivery_failed = True

    try:
        if should_stop("before_dependencies"):
            return
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        from .vendor.face_tracking_helpers import detect_faces, depth_at_face
        from .vision import ObservationBuilder, VisionFault

        width, height, fps = settings["width"], settings["height"], settings["fps"]
        detector = cv2.FaceDetectorYN.create(settings["model_path"], "", (320, 320),
                                            settings["face_score"], .3, 5000)
        if should_stop("before_pipeline_start"):
            return
        pipeline = rs.pipeline()
        config = rs.config()
        if settings["camera_serial"]:
            config.enable_device(settings["camera_serial"])
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = pipeline.start(config)
        started = True
        if should_stop("after_pipeline_start"):
            return
        device = profile.get_device()
        scale = float(device.first_depth_sensor().get_depth_scale())
        if not math.isfinite(scale) or scale <= 0:
            raise VisionFault("invalid depth scale")
        intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        builder = ObservationBuilder(intrinsics, depth_at_face,
                                     rs.rs2_deproject_pixel_to_point,
                                     face_score=settings["face_score"])
        align = rs.align(rs.stream.color)
        metadata = {}
        for key in ("name", "serial_number", "firmware_version", "usb_type_descriptor"):
            info = getattr(rs.camera_info, key)
            if device.supports(info):
                metadata[key] = device.get_info(info)
        emit("CAMERA_READY", time_s=time.monotonic(), **metadata,
             width=intrinsics.width, height=intrinsics.height, fps=fps,
             fx=intrinsics.fx, fy=intrinsics.fy, ppx=intrinsics.ppx, ppy=intrinsics.ppy,
             distortion_model=str(intrinsics.model), coefficients=list(intrinsics.coeffs),
             depth_scale_m=scale, color_format="bgr8", depth_format="z16",
             alignment="depth_to_color", raw_recording=False,
             coordinates="camera: x right, y down, z forward; meters",
             timestamp_semantics="host_frameset_receipt; not exposure time or video T0",
             worker_mode="spawn_process")
        while not should_stop("before_capture"):
            ok, frames = pipeline.try_wait_for_frames(200)
            if not ok:
                continue  # Parent independently enforces startup/freshness.
            acquired_at = time.monotonic()
            if should_stop("after_capture"):
                break
            aligned = align.process(frames)
            if should_stop("after_alignment"):
                break
            color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
            if not color or not depth:
                raise VisionFault("frameset missing color or aligned depth")
            number = int(color.get_frame_number())
            if builder.last_frame_number == number:
                continue
            bgr = np.asanyarray(color.get_data())
            depth_image = np.asanyarray(depth.get_data())
            if bgr.shape[:2] != (height, width) or depth_image.shape != bgr.shape[:2]:
                raise VisionFault("RGB/depth dimensions do not match the requested aligned profile")
            faces = detect_faces(detector, bgr)
            if should_stop("after_detection"):
                break
            stamp = float(color.get_timestamp())
            observation = builder.build(faces, depth_image, scale, acquired_at, number,
                                        stamp if math.isfinite(stamp) else None)
            if observation is None:
                continue
            if settings["preview"]:
                preview = bgr.copy()
                cv2.drawMarker(preview, (round(intrinsics.ppx), round(intrinsics.ppy)),
                               (255, 255, 255), cv2.MARKER_CROSS, 15, 1)
                for face in faces:
                    x1, y1, x2, y2 = map(int, face["box"])
                    cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 200, 255), 1)
                    cv2.circle(preview, tuple(map(round, face["aim"])), 3, (255, 0, 255), -1)
                cv2.putText(preview, observation.error or "ONE FACE / VALID DEPTH",
                            (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
                if preview_lock.acquire(timeout=.01):
                    try:
                        # One fixed-size in-memory image; never a queued video stream.
                        memoryview(preview_buffer).cast("B")[:] = preview.tobytes()
                    finally:
                        preview_lock.release()
            _publish_observation(observation_buffer, observation_lock, observation)
    except BaseException as exc:
        delivery_failed = not _send(channel, {
            "kind": "fault", "time_s": time.monotonic(),
            "reason": f"{type(exc).__name__}: {exc}"}, critical=True)
    finally:
        cleanup_stage("frame_release_started", pipeline_started=started,
                      stop_observed=stop_observed)
        bgr = depth_image = preview = None
        color = depth = aligned = frames = None
        cleanup_stage("frame_release_finished", pipeline_started=started)
        if started:
            stop_started = time.monotonic()
            cleanup_stage("pipeline_stop_started")
            try:
                pipeline.stop()
            except BaseException as exc:
                cleanup_error = f"{type(exc).__name__}: {exc}"
                cleanup_stage("pipeline_stop_failed", reason=cleanup_error,
                              duration_s=time.monotonic()-stop_started)
            else:
                cleanup_stage("pipeline_stop_finished",
                              duration_s=time.monotonic()-stop_started)
        acknowledged = _send(channel, {
            "kind": "closed", "time_s": time.monotonic(),
            "pipeline_started": started, "cleanup_ok": cleanup_error is None,
            "reason": cleanup_error}, critical=True)
        channel.close()
        if delivery_failed or not acknowledged:
            raise SystemExit(3)
