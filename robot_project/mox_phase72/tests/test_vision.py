import math
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest

from mox72.vision import ObservationBuilder, VisionCamera, VisionFault


def intrinsics():
    return types.SimpleNamespace(width=640, height=480, fx=500, fy=500, ppx=320, ppy=240)


def face(x=320, y=240, score=.9):
    return {"aim": (x, y), "score": score, "box": (x-20, y-20, x+20, y+20)}


def deproject(intr, xy, depth):
    return ((xy[0]-intr.ppx)*depth/intr.fx, (xy[1]-intr.ppy)*depth/intr.fy, depth)


class TestObservationBuilder(unittest.TestCase):
    def setUp(self):
        self.z = 1.
        self.builder = ObservationBuilder(intrinsics(), lambda *_: self.z, deproject)

    def observation(self, faces=None, t=1., number=1):
        return self.builder.build([face()] if faces is None else faces, None, .001, t, number, t*1000)

    def test_optical_center_zero_errors(self):
        obs = self.observation()
        self.assertEqual(obs.point_m, (0., 0., 1.))
        self.assertEqual((obs.horizontal_deg, obs.vertical_deg), (0., 0.))
        self.assertEqual(obs.track_id, 1)

    def test_signed_angles_are_camera_right_down(self):
        obs = self.observation([face(420, 140)])
        self.assertAlmostEqual(obs.horizontal_deg, math.degrees(math.atan(.2)))
        self.assertAlmostEqual(obs.vertical_deg, -math.degrees(math.atan(.2)))

    def test_two_faces_never_choose_one(self):
        obs = self.observation([face(200), face(440)])
        self.assertEqual(obs.count, 2)
        self.assertIsNone(obs.point_m)
        self.assertIsNone(obs.track_id)
        self.assertEqual(obs.error, "MULTIPLE_FACES")

    def test_missing_or_weak_face_is_invalid(self):
        for faces in ([], [face(score=.5)], [face(float("nan"))], [face(900)]):
            obs = self.observation(faces, number=self.builder.sequence+1)
            self.assertEqual(obs.count, 0)
            self.assertIsNone(obs.point_m)

    def test_bad_depth_resets_track(self):
        first = self.observation()
        self.z = None
        bad = self.observation(t=1.05, number=2)
        self.assertEqual(bad.count, 1)
        self.assertEqual(bad.error, "INVALID_DEPTH")
        self.z = 1.
        restored = self.observation(t=1.1, number=3)
        self.assertNotEqual(first.track_id, restored.track_id)

    def test_nearby_continuous_track_retained(self):
        a = self.observation()
        b = self.observation([face(330, 250)], t=1.05, number=2)
        self.assertEqual(a.track_id, b.track_id)

    def test_gap_large_jump_and_depth_jump_start_new_tracks(self):
        a = self.observation()
        b = self.observation(t=1.2, number=2)
        c = self.observation([face(500)], t=1.25, number=3)
        self.z = 1.4
        d = self.observation([face(500)], t=1.3, number=4)
        self.assertEqual([a.track_id, b.track_id, c.track_id, d.track_id], [1, 2, 3, 4])

    def test_duplicate_frame_does_not_refresh_time_or_sequence(self):
        self.observation()
        self.assertIsNone(self.observation(t=2., number=1))
        self.assertEqual(self.builder.sequence, 1)
        self.assertEqual(self.builder.last_time, 1.)

    def test_frame_counter_and_host_clock_backwards_fault(self):
        self.observation(number=3)
        with self.assertRaises(VisionFault):
            self.observation(t=1.05, number=2)
        with self.assertRaises(VisionFault):
            self.observation(t=.9, number=4)

    def test_nonfinite_intrinsics_and_deprojection_rejected(self):
        intr = intrinsics()
        intr.fx = 0
        with self.assertRaises(VisionFault):
            ObservationBuilder(intr, lambda *_: 1., deproject)
        self.builder.deproject = lambda *_: [float("nan"), 0, 1]
        self.assertEqual(self.observation().error, "INVALID_DEPROJECTION")


class TestCameraLifecycle(unittest.TestCase):
    def test_import_does_not_load_optional_dependencies(self):
        result = subprocess.run([sys.executable, "-B", "-S", "-c",
            "import sys; import mox72.vision; assert not any(x in sys.modules for x in ('cv2','numpy','pyrealsense2'))"],
            text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_missing_model_does_not_start_worker(self):
        cam = VisionCamera("/missing/mox-model.onnx", lambda *a, **kw: None)
        with self.assertRaises(VisionFault):
            cam.start()
        self.assertIsNone(cam._process)

    def test_startup_and_stale_failures_latch(self):
        cam = VisionCamera("unused", lambda *a, **kw: None)
        cam._started_at = 1
        self.assertIsNone(cam.latest(2))
        with self.assertRaises(VisionFault):
            cam.latest(7)
        self.assertIsNotNone(cam.fault)
        with self.assertRaises(VisionFault):
            cam.latest(2)
        cam = VisionCamera("unused", lambda *a, **kw: None)
        cam._started_at = 0
        builder = ObservationBuilder(intrinsics(), lambda *_: 1., deproject)
        cam._latest = builder.build([face()], None, .001, 1, 1)
        self.assertIs(cam.latest(1.2), cam._latest)
        with self.assertRaises(VisionFault):
            cam.latest(2.1)

    def test_close_is_idempotent_and_does_not_open_devices(self):
        cam = VisionCamera("unused", lambda *a, **kw: None)
        cam.close()
        cam.close()
        with self.assertRaises(VisionFault):
            cam.start()

    def test_newer_worker_snapshot_waits_for_next_controller_tick(self):
        cam = VisionCamera("unused", lambda *a, **kw: None)
        cam._started_at = 0
        builder = ObservationBuilder(intrinsics(), lambda *_: 1., deproject)
        cam._latest = builder.build([face()], None, .001, 1.01, 1)
        self.assertIsNone(cam.latest(1.0))
        self.assertIsNone(cam.fault)
        self.assertIsNotNone(cam.latest(1.02))


if __name__ == "__main__":
    unittest.main()
