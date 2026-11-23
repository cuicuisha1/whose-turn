"""Real spawned child processes, with no camera/NumPy/OpenCV dependencies."""
from dataclasses import asdict
import os
from pathlib import Path
import signal
import socket
import tempfile
import sys
import types
import time
import unittest
import weakref
from unittest.mock import patch

from mox72.camera_worker import _send, _stopping, _publish_observation
from mox72.models import FaceObservation
from mox72.vision import VisionCamera, VisionFault


def observation(channel, buffer, lock, sequence=1):
    now = time.monotonic()
    obs = FaceObservation(now, sequence, 1, 1, (0., 0., 1.), 0., 0.)
    _publish_observation(buffer, lock, obs)


def event(channel, name):
    _send(channel, {"kind": "event", "event": name,
                    "fields": {"time_s": time.monotonic()}}, critical=True)


def closed(channel, ok=True):
    _send(channel, {"kind": "closed", "time_s": time.monotonic(),
                    "pipeline_started": True, "cleanup_ok": ok,
                    "reason": None if ok else "fake pipeline.stop failed"}, critical=True)
    channel.close()


def good_worker(settings, channel, buffer, lock, obs_buffer, obs_lock):
    event(channel, "CAMERA_READY")
    observation(channel, obs_buffer, obs_lock)
    while not _stopping(channel, settings["parent_pid"]):
        time.sleep(.005)
    closed(channel)


def blocked_start_worker(settings, channel, buffer, lock, obs_buffer, obs_lock):
    event(channel, "TEST_START_BLOCKED")
    while True:
        time.sleep(.05)


def blocked_inference_worker(settings, channel, buffer, lock, obs_buffer, obs_lock):
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    observation(channel, obs_buffer, obs_lock)
    event(channel, "TEST_INFERENCE_BLOCKED")
    while True:
        time.sleep(.05)


def dead_worker(settings, channel, buffer, lock, obs_buffer, obs_lock):
    observation(channel, obs_buffer, obs_lock)
    os._exit(17)


def no_ack_worker(settings, channel, buffer, lock, obs_buffer, obs_lock):
    observation(channel, obs_buffer, obs_lock)
    while not _stopping(channel, settings["parent_pid"]):
        time.sleep(.005)
    channel.close()


def failed_cleanup_worker(settings, channel, buffer, lock, obs_buffer, obs_lock):
    observation(channel, obs_buffer, obs_lock)
    while not _stopping(channel, settings["parent_pid"]):
        time.sleep(.005)
    closed(channel, ok=False)


def lock_owner_worker(settings, channel, buffer, lock, obs_buffer, obs_lock):
    lock.acquire()
    observation(channel, obs_buffer, obs_lock)
    event(channel, "TEST_PREVIEW_LOCKED")
    while True:
        time.sleep(.05)


def burst_worker(settings, channel, buffer, lock, obs_buffer, obs_lock):
    for seq in range(1, 11):
        observation(channel, obs_buffer, obs_lock, seq)
    while not _stopping(channel, settings["parent_pid"]):
        time.sleep(.005)
    closed(channel)


def latest_overwrite_worker(settings, channel, buffer, lock, obs_buffer, obs_lock):
    for sequence in range(1, 501):
        observation(channel, obs_buffer, obs_lock, sequence)
    event(channel, "TEST_OVERWRITE_DONE")
    while not _stopping(channel, settings["parent_pid"]):
        time.sleep(.005)
    closed(channel)


def observation_lock_owner_worker(settings, channel, buffer, lock, obs_buffer, obs_lock):
    observation(channel, obs_buffer, obs_lock)
    obs_lock.acquire()
    event(channel, "TEST_OBSERVATION_LOCKED")
    while True:
        time.sleep(.05)


def _real_worker_fake_sdk(settings, channel, buffer, lock, obs_buffer, obs_lock,
                          *, frames_enabled=False, stop_mode="ok", pause_detection=False):
    # Exercise real worker orchestration, including Python views retaining
    # frame objects. These mocks do not claim any physical SDK behavior.
    from mox72.camera_worker import run_camera_worker
    intr = types.SimpleNamespace(width=640, height=480, fx=500., fy=500.,
                                 ppx=320., ppy=240., model="fake", coeffs=[0.]*5)
    device = types.SimpleNamespace(
        first_depth_sensor=lambda: types.SimpleNamespace(get_depth_scale=lambda: .001),
        supports=lambda key: False)
    profile = types.SimpleNamespace(
        get_device=lambda: device,
        get_stream=lambda _: types.SimpleNamespace(as_video_stream_profile=lambda:
            types.SimpleNamespace(get_intrinsics=lambda: intr)))
    frame_refs = []

    class ArrayView:
        def __init__(self, frame, color):
            self.owner = frame
            self.shape = (480, 640, 3) if color else (480, 640)

    class Frame:
        def __init__(self, number, color):
            self.number, self.color = number, color
            frame_refs.append(weakref.ref(self))
        def get_frame_number(self): return self.number
        def get_timestamp(self): return self.number*33.3
        def get_data(self): return ArrayView(self, self.color)

    class FrameSet:
        def __init__(self, number):
            self.color, self.depth = Frame(number, True), Frame(number, False)
            frame_refs.append(weakref.ref(self))
        def get_color_frame(self): return self.color
        def get_depth_frame(self): return self.depth

    class Pipeline:
        number = 0
        def start(self, config):
            return profile
        def try_wait_for_frames(self, timeout):
            time.sleep(.005)
            self.number += 1
            return (True, FrameSet(self.number)) if frames_enabled else (False, None)
        def stop(self):
            event(channel, "TEST_NATIVE_STOP_CALLED")
            if any(ref() is not None for ref in frame_refs):
                raise RuntimeError("fake frame buffer still retained at stop")
            event(channel, "TEST_NATIVE_FRAMES_RELEASED")
            if stop_mode == "fail":
                raise RuntimeError("fake pipeline.stop failed")
            if stop_mode == "block":
                while True:
                    time.sleep(.01)

    def detect_faces(*args):
        if pause_detection:
            event(channel, "TEST_DETECTION_ENTERED")
            while True:
                try:
                    channel.recv(16384, socket.MSG_PEEK)
                    break
                except BlockingIOError:
                    time.sleep(.005)
        return [{"aim": (320., 240.), "score": .9, "box": (300., 220., 340., 260.)}]

    cv2 = types.ModuleType("cv2")
    cv2.FaceDetectorYN = types.SimpleNamespace(create=lambda *a: object())
    np = types.ModuleType("numpy")
    np.asanyarray = lambda data: data
    rs = types.ModuleType("pyrealsense2")
    rs.pipeline = Pipeline
    rs.config = lambda: types.SimpleNamespace(enable_device=lambda *a: None,
                                              enable_stream=lambda *a: None)
    rs.stream = types.SimpleNamespace(color=1, depth=2)
    rs.format = types.SimpleNamespace(bgr8=1, z16=2)
    rs.camera_info = types.SimpleNamespace(name=1, serial_number=2, firmware_version=3,
                                          usb_type_descriptor=4)
    rs.align = lambda _: types.SimpleNamespace(process=lambda frames: frames)
    rs.rs2_deproject_pixel_to_point = lambda *a: (0., 0., 1.)
    helper = types.ModuleType("mox72.vendor.face_tracking_helpers")
    helper.detect_faces = detect_faces
    helper.depth_at_face = lambda *a: 1.
    sys.modules.update({"cv2": cv2, "numpy": np, "pyrealsense2": rs,
                        "mox72.vendor.face_tracking_helpers": helper})
    run_camera_worker(settings, channel, buffer, lock, obs_buffer, obs_lock)


def real_worker_fake_pipeline(*args):
    _real_worker_fake_sdk(*args)


def real_worker_retained_frames(*args):
    _real_worker_fake_sdk(*args, frames_enabled=True)


def real_worker_failed_stop(*args):
    _real_worker_fake_sdk(*args, stop_mode="fail")


def real_worker_blocked_stop(*args):
    _real_worker_fake_sdk(*args, stop_mode="block")


def real_worker_stop_during_detection(*args):
    _real_worker_fake_sdk(*args, frames_enabled=True, pause_detection=True)


class TestSpawnedCameraLifecycle(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.model = Path(self.temp.name)/"fake.onnx"
        self.model.write_bytes(b"not loaded by fake worker")
        self.events = []
        self.cameras = []

    def tearDown(self):
        for cam in self.cameras:
            try:
                cam.close()
            except VisionFault:
                pass
        self.temp.cleanup()

    def camera(self, worker=good_worker, emit=None, preview=False):
        cam = VisionCamera(self.model, emit or (lambda e, **v: self.events.append((e, v))),
                           preview=preview)
        cam._worker_target = worker
        cam.CLOSE_GRACE_S = .15
        cam.TERMINATE_S = .15
        cam.KILL_S = .5
        self.cameras.append(cam)
        cam.start()
        return cam

    def wait_until(self, test, timeout=3):
        end = time.monotonic()+timeout
        while not test():
            if time.monotonic() >= end:
                self.fail("spawned worker did not reach expected state")
            time.sleep(.005)

    def wait_event(self, cam, name):
        def found():
            cam._pump()
            return any(e == name for e, _ in self.events)
        self.wait_until(found)

    def assert_reaped(self, pid):
        with self.assertRaises(ProcessLookupError):
            os.kill(pid, 0)

    def test_graceful_close_receives_ack_and_reaps_process(self):
        cam = self.camera()
        pid = cam._process.pid
        self.wait_until(lambda: cam.ready)
        self.assertIsNotNone(cam.latest())
        self.assertEqual(cam.latest().point_m, (0., 0., 1.))
        self.assertNotEqual(pid, os.getpid())
        cam.close()
        cam.close()
        self.assert_reaped(pid)
        shutdown = [v for e, v in self.events if e == "CAMERA_SHUTDOWN"][-1]
        self.assertFalse(shutdown["forced"])
        self.assertTrue(shutdown["cleanup_acknowledged"])
        self.assertTrue(shutdown["cleanup_ok"])

    def test_native_start_blocked_is_bounded_and_is_not_success(self):
        cam = self.camera(blocked_start_worker)
        pid = cam._process.pid
        self.wait_event(cam, "TEST_START_BLOCKED")
        with self.assertRaisesRegex(VisionFault, "startup exceeded"):
            cam.latest(cam._started_at+5.1)
        before = time.monotonic()
        with self.assertRaisesRegex(VisionFault, "terminate required"):
            cam.close()
        self.assertLess(time.monotonic()-before, 2)
        self.assert_reaped(pid)
        self.assertFalse([v for e,v in self.events if e == "CAMERA_SHUTDOWN"][-1]["cleanup_ok"])

    def test_stuck_inference_ignoring_term_requires_kill(self):
        cam = self.camera(blocked_inference_worker)
        pid = cam._process.pid
        self.wait_event(cam, "TEST_INFERENCE_BLOCKED")
        before = time.monotonic()
        with self.assertRaisesRegex(VisionFault, "kill required"):
            cam.close()
        self.assertLess(time.monotonic()-before, 2)
        self.assert_reaped(pid)
        self.assertTrue(cam.fault)

    def test_dead_process_is_detected_before_stale_deadline(self):
        cam = self.camera(dead_worker)
        self.wait_until(lambda: cam._process.exitcode is not None)
        with self.assertRaisesRegex(VisionFault, "exited unexpectedly with code 17"):
            cam.latest()
        self.assertLess(time.monotonic()-cam._latest.time_s, 1.)
        with self.assertRaisesRegex(VisionFault, "cleanup acknowledgement missing"):
            cam.close()

    def test_exit_zero_without_cleanup_ack_is_failure(self):
        cam = self.camera(no_ack_worker)
        self.wait_until(lambda: cam.ready)
        with self.assertRaisesRegex(VisionFault, "cleanup acknowledgement missing"):
            cam.close()

    def test_pipeline_stop_exception_is_not_success(self):
        cam = self.camera(failed_cleanup_worker)
        self.wait_until(lambda: cam.ready)
        with self.assertRaisesRegex(VisionFault, "fake pipeline.stop failed"):
            cam.close()

    def test_log_error_latches_fault_but_cleanup_still_reaps(self):
        def emit(e, **fields):
            if e == "CAMERA_READY":
                raise OSError("disk full")
            self.events.append((e, fields))
        cam = self.camera(emit=emit)
        pid = cam._process.pid
        self.wait_until(lambda: cam.fault is not None)
        with self.assertRaisesRegex(VisionFault, "logging failed"):
            cam.latest()
        with self.assertRaisesRegex(VisionFault, "logging failed"):
            cam.close()
        self.assert_reaped(pid)

    def test_killed_preview_lock_owner_cannot_block_parent(self):
        cam = self.camera(lock_owner_worker, preview=True)
        self.wait_event(cam, "TEST_PREVIEW_LOCKED")
        before = time.monotonic()
        self.assertIsNone(cam.preview_frame())
        self.assertLess(time.monotonic()-before, .1)
        pid = cam._process.pid
        with self.assertRaisesRegex(VisionFault, "terminate required"):
            cam.close()
        self.assert_reaped(pid)
        self.assertIsNone(cam.preview_frame())

    def test_only_latest_observation_is_exposed(self):
        cam = self.camera(burst_worker)
        self.wait_until(lambda: cam.ready and cam.latest().sequence == 10)
        self.assertEqual(cam.latest().sequence, 10)
        cam.close()

    def test_overwriting_latest_slot_needs_no_parent_drain(self):
        cam = self.camera(latest_overwrite_worker)
        self.wait_event(cam, "TEST_OVERWRITE_DONE")
        # 500 snapshots fit into the same single fixed-size slot. The first
        # parent poll sees number 500, not a full queue of old frames.
        self.assertEqual(cam.latest().sequence, 500)
        self.assertLessEqual(len(cam._observation_buffer), 16388)
        cam.close()

    def test_killed_observation_lock_owner_cannot_hang_cleanup(self):
        cam = self.camera(observation_lock_owner_worker)
        self.wait_event(cam, "TEST_OBSERVATION_LOCKED")
        before = time.monotonic()
        cam._pump()
        self.assertLess(time.monotonic()-before, .1)
        pid = cam._process.pid
        with self.assertRaisesRegex(VisionFault, "terminate required"):
            cam.close()
        self.assert_reaped(pid)

    def test_real_worker_calls_pipeline_stop_and_sends_clean_ack(self):
        cam = self.camera(real_worker_fake_pipeline)
        self.wait_event(cam, "CAMERA_READY")
        self.assertFalse(cam.ready)  # A pipeline alone is not a fresh frame.
        cam.close()
        self.assertEqual(sum(e == "TEST_NATIVE_STOP_CALLED" for e, _ in self.events), 1)
        self.assertTrue([v for e,v in self.events if e == "CAMERA_SHUTDOWN"][-1]["cleanup_ok"])

    def test_real_worker_releases_frame_views_before_pipeline_stop(self):
        cam = self.camera(real_worker_retained_frames)
        self.wait_until(lambda: cam.ready)
        cam.close()
        self.assertEqual(sum(e == "TEST_NATIVE_FRAMES_RELEASED" for e,_ in self.events), 1)
        stages = [v["stage"] for e,v in self.events if e == "CAMERA_CLEANUP_STAGE"]
        self.assertEqual(stages, ["frame_release_started", "frame_release_finished",
                                  "pipeline_stop_started", "pipeline_stop_finished"])
        self.assertEqual(sum(e == "CAMERA_STOP_OBSERVED" for e,_ in self.events), 1)
        self.assertTrue([v for e,v in self.events if e == "CAMERA_SHUTDOWN"][-1]["cleanup_ok"])

    def test_real_blocked_stop_reports_last_stage_and_forced_cleanup(self):
        cam = self.camera(real_worker_blocked_stop)
        self.wait_event(cam, "CAMERA_READY")
        with self.assertRaisesRegex(VisionFault, "terminate required"):
            cam.close()
        stages = [v["stage"] for e,v in self.events if e == "CAMERA_CLEANUP_STAGE"]
        self.assertIn("pipeline_stop_started", stages)
        self.assertNotIn("pipeline_stop_finished", stages)
        self.assertFalse([v for e,v in self.events if e == "CAMERA_SHUTDOWN"][-1]["cleanup_ok"])

    def test_real_failed_stop_is_explicitly_logged(self):
        cam = self.camera(real_worker_failed_stop)
        self.wait_event(cam, "CAMERA_READY")
        with self.assertRaisesRegex(VisionFault, "fake pipeline.stop failed"):
            cam.close()
        stages = [v for e,v in self.events if e == "CAMERA_CLEANUP_STAGE"]
        self.assertEqual(stages[-1]["stage"], "pipeline_stop_failed")
        self.assertIn("fake pipeline.stop failed", stages[-1]["reason"])

    def test_stop_received_during_detection_does_not_publish_another_observation(self):
        cam = self.camera(real_worker_stop_during_detection)
        self.wait_event(cam, "TEST_DETECTION_ENTERED")
        self.assertIsNone(cam._latest)
        cam.close()
        self.assertIsNone(cam._latest)
        checkpoint = [v["checkpoint"] for e,v in self.events if e == "CAMERA_STOP_OBSERVED"]
        self.assertEqual(checkpoint, ["after_detection"])
        self.assertTrue([v for e,v in self.events if e == "CAMERA_SHUTDOWN"][-1]["cleanup_ok"])

    def test_interrupted_join_still_stops_and_reaps_child(self):
        cam = self.camera(blocked_start_worker)
        self.wait_event(cam, "TEST_START_BLOCKED")
        pid = cam._process.pid
        real_join = cam._join_draining
        calls = 0
        def interrupted_first_join(timeout):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise KeyboardInterrupt("second Ctrl-C")
            return real_join(timeout)
        with patch.object(cam, "_join_draining", side_effect=interrupted_first_join):
            with self.assertRaisesRegex(VisionFault, "KeyboardInterrupt"):
                cam.close()
        self.assert_reaped(pid)
        self.assertTrue(cam._close_completed)
        self.assertGreaterEqual(calls, 2)

    def test_terminate_error_still_attempts_kill_and_reaps(self):
        cam = self.camera(blocked_start_worker)
        self.wait_event(cam, "TEST_START_BLOCKED")
        pid = cam._process.pid
        with patch.object(cam._process, "terminate", side_effect=OSError("fake terminate failed")):
            with self.assertRaisesRegex(VisionFault, "fake terminate failed"):
                cam.close()
        self.assert_reaped(pid)
        self.assertEqual([v for e,v in self.events if e == "CAMERA_SHUTDOWN"][-1]["escalation"], "kill")

    def test_unreaped_worker_is_not_marked_closed_and_close_can_retry(self):
        class Unreaped:
            pid = 999999999
            exitcode = None
            alive = True
            handle_closed = False
            def is_alive(self): return self.alive
            def join(self, timeout): pass
            def terminate(self): pass
            def kill(self): pass
            def close(self): self.handle_closed = True
        cam = VisionCamera(self.model, lambda *a, **v: None)
        self.cameras.append(cam)
        proc = Unreaped()
        cam._process = proc
        cam.CLOSE_GRACE_S = cam.TERMINATE_S = cam.KILL_S = .001
        with self.assertRaisesRegex(VisionFault, "still alive"):
            cam.close()
        self.assertFalse(cam._close_completed)
        self.assertIs(cam._process, proc)
        self.assertFalse(proc.handle_closed)
        proc.alive, proc.exitcode = False, -9
        with self.assertRaises(VisionFault):
            cam.close()  # Previous forced cleanup remains an invalid attempt.
        self.assertTrue(cam._close_completed)
        self.assertTrue(proc.handle_closed)

    def test_malformed_ipc_packets_latch_fault_instead_of_escaping_cleanup(self):
        payloads = [b'[]', b'{"kind":"observation","observation":[]}',
                    b'{"kind":"observation","observation":{"time_s":1,"sequence":true,"track_id":1,"count":1,"point_m":null,"horizontal_deg":0,"vertical_deg":0}}']
        for payload in payloads:
            with self.subTest(payload=payload):
                cam = VisionCamera(self.model, lambda *a, **v: None)
                parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
                parent.setblocking(False)
                child.setblocking(False)
                cam._channel = parent
                child.send(payload)
                cam._pump()
                self.assertIsInstance(cam.fault, VisionFault)
                cam.close()
                child.close()

    def test_datagram_pressure_does_not_block_observation_sender(self):
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            parent.setblocking(False)
            child.setblocking(False)
            child.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
            start = time.monotonic()
            results = [_send(child, {"kind":"observation", "pad":"x"*4000})
                       for _ in range(100)]
            self.assertLess(time.monotonic()-start, 1)
            self.assertIn(False, results)
        finally:
            parent.close()
            child.close()


if __name__ == "__main__":
    unittest.main()
