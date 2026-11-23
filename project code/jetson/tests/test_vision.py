import contextlib
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from dataclasses import replace

import numpy as np

from jetson.main import arguments
from jetson.vision.depth_estimator import DepthEstimator
from jetson.vision.perception import Perception, make_observation
from jetson.vision.person_detector import HogDetector, NoDetector, SyntheticDetector, YoloDetector, suppress
from jetson.vision.person_tracker import PersonTracker
from jetson.vision.realsense_camera import CaptureStatus, CaptureWorker, RealSenseCamera, SyntheticCamera
from jetson.vision.types import Detection, Frame


def fixture(now=10.0, boxes=((20., 5., 80., 95.),)):
    return Frame(np.zeros((100, 100, 3), dtype=np.uint8), np.full((100, 100), 1500, dtype=np.uint16),
                 .001, now, 10, 10, 100., 100., "hardware", "hardware", "simulation", {},
                 tuple(Detection(b, .9, "synthetic_fixture") for b in boxes))


class DepthTests(unittest.TestCase):
    def setUp(self):
        self.estimator = DepthEstimator()

    def test_median_ignores_holes_and_sparse_outliers(self):
        depth = np.full((100, 100), 2000, dtype=np.uint16)
        depth[::4, ::4] = 0
        depth[::9, ::9] = 5500
        result = self.estimator.estimate(depth, .001, (0, 0, 100, 100))
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.distance_m, 2, places=4)
        self.assertLess(result.valid_ratio, 1)

    def test_empty_or_invalid_depth_is_unknown_not_zero(self):
        for value in (0., float("nan"), float("inf"), -1., 100000.):
            result = self.estimator.estimate(np.full((100, 100), value), .001, (0, 0, 100, 100))
            self.assertFalse(result.valid)
            self.assertIsNone(result.distance_m)

    def test_mixed_foreground_background_is_rejected(self):
        depth = np.full((100, 100), 1000, dtype=np.uint16)
        depth[:, 50:] = 4000
        result = self.estimator.estimate(depth, .001, (0, 0, 100, 100))
        self.assertEqual(result.reason, "mixed_depths")
        self.assertIsNone(result.distance_m)

    def test_coverage_and_clipped_roi(self):
        depth = np.zeros((100, 100), dtype=np.uint16)
        depth[30, 40] = 1000
        self.assertFalse(self.estimator.estimate(depth, .001, (0, 0, 100, 100)).valid)
        for box in ((-500, -500, -1, -1), (200, 200, 250, 250), (10, 10, 10, 20)):
            self.assertEqual(self.estimator.estimate(depth, .001, box).reason, "empty_roi")

    def test_real_scale_is_used(self):
        result = self.estimator.estimate(np.full((100, 100), 1000), .002, (0, 0, 100, 100))
        self.assertAlmostEqual(result.distance_m, 2)
        with self.assertRaises(ValueError):
            self.estimator.estimate(np.zeros((10, 10)), float("nan"), (0, 0, 10, 10))


class PerceptionTests(unittest.TestCase):
    def test_missing_depth_never_reuses_old_distance(self):
        p = Perception(SyntheticDetector())
        good = fixture()
        first = p.process(good, 1)
        bad = replace(good, received_at=10.1, depth=np.zeros((100, 100), dtype=np.uint16))
        second = p.process(bad, 1)
        self.assertEqual(first["target"]["track_id"], second["target"]["track_id"])
        self.assertIsNone(second["target"]["distance_m"])
        self.assertFalse(second["target"]["depth"]["valid"])
        self.assertIsNotNone(second["attention"])

    def test_short_occlusion_clears_target_but_retains_id(self):
        p = Perception(SyntheticDetector())
        first = p.process(fixture(), 1)["target"]["track_id"]
        lost = p.process(fixture(10.2, boxes=()), 1)
        self.assertIsNone(lost["target"])
        self.assertIsNone(lost["attention"])
        self.assertFalse(lost["person_detected"])
        self.assertEqual(p.process(fixture(10.4), 1)["target"]["track_id"], first)
        self.assertNotEqual(p.process(fixture(12), 1)["target"]["track_id"], first)

    def test_camera_generation_resets_tracking_without_id_reuse(self):
        p = Perception(SyntheticDetector())
        first = p.process(fixture(), 1)["target"]["track_id"]
        second = p.process(fixture(10.1), 2)["target"]["track_id"]
        self.assertNotEqual(first, second)

    def test_attention_and_score_semantics(self):
        result = Perception(SyntheticDetector()).process(fixture(), 1)
        self.assertAlmostEqual(result["target"]["center_x"], .5)
        self.assertAlmostEqual(result["attention"]["horizontal_error"], 0)
        self.assertFalse(result["attention"]["actuation_enabled"])
        self.assertIsNone(result["target"]["confidence"])
        camera_only = Perception(NoDetector()).process(fixture(), 1)
        self.assertIsNone(camera_only["person_detected"])

    def test_alignment_mismatch_and_synthetic_guard(self):
        with self.assertRaises(ValueError):
            Perception(SyntheticDetector()).process(replace(fixture(), depth=np.zeros((50, 100))), 1)
        with self.assertRaises(ValueError):
            SyntheticDetector().detect(replace(fixture(), source="realsense"))

    def test_stale_and_reconnect_race_invalidate_all_outputs(self):
        frame = fixture()
        result = Perception(SyntheticDetector()).process(frame, 1)
        status = CaptureStatus(state="streaming", generation=1, frame=frame)
        for state, generation, now in (("streaming", 1, 11), ("reconnecting", 1, 10.1), ("streaming", 2, 10.1)):
            out = make_observation(replace(status, state=state, generation=generation), result, frame,
                                   1, now, .5, "synthetic_fixture", "simulation", "test")
            self.assertFalse(out["valid"])
            self.assertEqual(out["people"], [])
            self.assertIsNone(out["target"])
            self.assertIsNone(out["attention"])
            self.assertIsNone(out["person_detected"])
        out = make_observation(status, result, frame, 1, 10.1, .5, "synthetic_fixture", "simulation", "test")
        self.assertTrue(out["valid"])
        json.dumps(out, allow_nan=False)


class TrackingTests(unittest.TestCase):
    def test_one_to_one_assignment_and_sticky_target(self):
        tracker = PersonTracker()
        ds = [Detection((5, 0, 30, 90), .8), Detection((60, 0, 90, 90), .9)]
        first = tracker.update(ds, 1, 100, 100)
        people = [{"track_id": t.id, "distance_m": d, "center_x": t.center_x} for t, d in zip(first, [2, 3])]
        selected = tracker.choose_target(people)["track_id"]
        second = tracker.update(list(reversed(ds)), 1.1, 100, 100)
        self.assertEqual([t.id for t in second], [first[1].id, first[0].id])
        people[1]["distance_m"] = 1
        self.assertEqual(tracker.choose_target(people)["track_id"], selected)

    def test_nms_removes_duplicate_boxes(self):
        ds = [Detection((0, 0, 10, 10), .9), Detection((1, 0, 11, 10), .8), Detection((30, 0, 40, 10), .7)]
        self.assertEqual(len(suppress(ds)), 2)


class CaptureTests(unittest.TestCase):
    def test_latest_only_disconnect_clear_and_reconnect(self):
        class FailingCamera:
            opened = 0
            reads = 0
            closed = 0

            def open(self):
                self.opened += 1

            def read(self):
                time.sleep(.005)
                self.reads += 1
                if self.opened == 1 and self.reads >= 3:
                    raise TimeoutError("injected unplug")
                return fixture(time.monotonic())

            def close(self):
                self.closed += 1

        camera = FailingCamera()
        worker = CaptureWorker(camera, retry_initial=.08, retry_max=.08)
        worker.start()
        deadline = time.monotonic() + 2
        saw_outage = saw_reconnect = False
        while time.monotonic() < deadline:
            status = worker.snapshot()
            if status.state == "reconnecting":
                saw_outage = True
                self.assertIsNone(status.frame)
            if status.generation >= 2 and status.sequence >= 5:
                saw_reconnect = True
                break
            time.sleep(.002)
        self.assertTrue(worker.close())
        self.assertTrue(saw_outage and saw_reconnect)
        self.assertGreaterEqual(camera.closed, 2)
        self.assertEqual(worker.snapshot().state, "stopped")
        self.assertIsNone(worker.snapshot().frame)

    def test_synthetic_frames_are_owned_and_explicit(self):
        camera = SyntheticCamera(fps=30)
        camera.open()
        frame = camera.read()
        self.assertEqual(frame.source, "simulation")
        self.assertFalse(frame.color.flags.writeable)
        self.assertFalse(frame.depth.flags.writeable)


class IntegrationTests(unittest.TestCase):
    def test_cli_rejects_invalid_or_ambiguous_settings(self):
        for argv in (["--detector", "synthetic"], ["--detector", "yolo"], ["--duration", "nan"],
                     ["--min-depth", "5", "--max-depth", "1"], ["--output-hz", "0"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                arguments(argv)
        with self.assertRaises(ValueError):
            YoloDetector(Path("absent-model.pt"))

    def test_cli_offline_json_shutdown_and_summary(self):
        temporary_root = Path(__file__).resolve().parents[1] / "runs" / "test-temp"
        temporary_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=temporary_root) as directory:
            completed = subprocess.run([sys.executable, "-m", "jetson.main", "--simulate", "--duration", "1.2",
                                        "--stdout", "--output-dir", directory], capture_output=True, text=True,
                                       encoding="utf-8", env={**os.environ, "PYTHONIOENCODING": "utf-8"}, timeout=15)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            messages = [json.loads(line) for line in completed.stdout.splitlines()]
            self.assertTrue(any(m["valid"] and m["target"] for m in messages))
            self.assertTrue(all(m["simulated"] and not m["actuation_enabled"] for m in messages))
            self.assertEqual(messages[-1]["status"], "stopped")
            self.assertFalse(messages[-1]["valid"])
            run = next(Path(directory).iterdir())
            self.assertEqual(json.loads((run / "latest.json").read_text())["status"], "stopped")
            self.assertTrue(json.loads((run / "summary.json").read_text())["capture_thread_stopped"])

    def test_opencv_hog_and_render_on_actual_library(self):
        try:
            import cv2
        except ImportError:
            self.skipTest("OpenCV not installed; HOG/render runtime check skipped")
        from jetson.preview import render
        camera = SyntheticCamera()
        camera.open()
        frame = camera.read()
        detector = HogDetector()
        self.assertEqual(detector.detect(replace(frame, color=np.zeros_like(frame.color))), [])
        result = Perception(SyntheticDetector()).process(frame, 1)
        obs = make_observation(CaptureStatus(state="streaming", generation=1), result, frame, 1,
                               frame.received_at + .1, .5, "synthetic_fixture", "simulation", "test")
        image = render(frame, obs)
        self.assertEqual(image.shape, (562, 1280, 3))
        self.assertTrue(cv2.imencode(".png", image)[0])

    def test_snapshot_in_unicode_folder(self):
        try:
            import cv2
        except ImportError:
            self.skipTest("OpenCV not installed; snapshot check skipped")
        temporary_root = Path(__file__).resolve().parents[1] / "runs" / "测试截图"
        temporary_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=temporary_root) as directory:
            result = subprocess.run([sys.executable, "-m", "jetson.main", "--simulate", "--duration", ".5",
                                     "--snapshot", "--output-dir", directory], capture_output=True, text=True,
                                    encoding="utf-8", env={**os.environ, "PYTHONIOENCODING": "utf-8"}, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr)
            picture = next(Path(directory).rglob("last_preview.png"))
            image = cv2.imdecode(np.frombuffer(picture.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
            self.assertEqual(image.shape, (562, 1280, 3))


if __name__ == "__main__":
    unittest.main()
