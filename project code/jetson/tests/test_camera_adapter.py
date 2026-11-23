"""SDK contract tests with stubs; these do not validate real USB or alignment geometry."""
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from jetson.vision.realsense_camera import RealSenseCamera


class ImageFrame:
    def __init__(self, array, number=1, timestamp=30., clock="hardware_clock"):
        self.array, self.number, self.timestamp, self.clock = array, number, timestamp, clock

    def get_frame_number(self):
        return self.number

    def get_timestamp(self):
        return self.timestamp

    def get_frame_timestamp_domain(self):
        return self.clock

    def get_data(self):
        return self.array


class Frames:
    def __init__(self, number=1, depth_time=30.):
        self.color = ImageFrame(np.full((480, 640, 3), 50, dtype=np.uint8), number)
        self.depth = ImageFrame(np.full((480, 640), 1000, dtype=np.uint16), number, depth_time)

    def get_color_frame(self):
        return self.color

    def get_depth_frame(self):
        return self.depth


class SDK:
    stream = SimpleNamespace(color="color", depth="depth")
    format = SimpleNamespace(z16="z16", bgr8="bgr8")
    camera_info = SimpleNamespace(name="name", serial_number="serial", firmware_version="firmware", usb_type_descriptor="usb")

    def __init__(self):
        self.queue = []
        self.serial = "test-camera-001"
        self.devices = [self]
        self.stops = self.alignments = 0
        self.streams = []

    def context(self):
        return SimpleNamespace(query_devices=lambda: self.devices)

    def pipeline(self, ctx):
        return SimpleNamespace(start=lambda cfg: SimpleNamespace(get_device=lambda: self),
                               stop=self.stop, try_wait_for_frames=self.wait)

    def stop(self):
        self.stops += 1

    def wait(self, timeout):
        return (True, self.queue.pop(0)) if self.queue else (False, None)

    def config(self):
        return SimpleNamespace(enable_device=self.bind, enable_stream=lambda *args: self.streams.append(args))

    def bind(self, serial):
        self.bound = serial

    def supports(self, key):
        return True

    def get_info(self, key):
        return {"serial": self.serial, "name": "Fake D435if", "firmware": "test", "usb": "3.2"}[key]

    def first_depth_sensor(self):
        return SimpleNamespace(get_depth_scale=lambda: .002)

    def align(self, stream):
        assert stream == "color"
        return SimpleNamespace(process=self.process)

    def process(self, frames):
        self.alignments += 1
        return frames


class CameraAdapterTests(unittest.TestCase):
    def open_camera(self):
        sdk = SDK()
        with patch.dict("sys.modules", {"pyrealsense2": sdk}):
            camera = RealSenseCamera()
        camera.open()
        self.addCleanup(camera.close)
        return sdk, camera

    def test_profiles_alignment_scale_and_owned_copies(self):
        sdk, camera = self.open_camera()
        frames = Frames()
        sdk.queue.append(frames)
        result = camera.read()
        self.assertEqual(sdk.streams, [("depth", 640, 480, "z16", 30), ("color", 640, 480, "bgr8", 30)])
        self.assertEqual(sdk.alignments, 1)
        self.assertEqual(result.depth_scale, .002)
        self.assertEqual(result.device["serial"], "test-camera-001")
        frames.color.array[:] = 0
        frames.depth.array[:] = 0
        self.assertTrue(np.all(result.color == 50))
        self.assertTrue(np.all(result.depth == 1000))
        self.assertFalse(result.color.flags.writeable)

    def test_duplicate_frames_cannot_renew_freshness(self):
        sdk, camera = self.open_camera()
        sdk.queue = [Frames(1)]
        camera.read()
        sdk.queue = [Frames(1)]
        with self.assertRaises(TimeoutError):
            camera.read()
        sdk.queue = [Frames(1), Frames(2)]
        self.assertEqual(camera.read().color_number, 2)

    def test_time_domains_and_same_domain_skew(self):
        sdk, camera = self.open_camera()
        sdk.queue = [Frames(1, depth_time=500.)]
        with self.assertRaisesRegex(RuntimeError, "skew"):
            camera.read()
        frames = Frames(2, depth_time=500.)
        frames.depth.clock = "system_time"
        sdk.queue = [frames]
        self.assertEqual(camera.read().depth_clock, "system_time")

    def test_serial_is_pinned_after_reconnect(self):
        sdk, camera = self.open_camera()
        camera.close()
        sdk.serial = "different-camera-002"
        camera.open()
        self.assertEqual(sdk.bound, "test-camera-001")
        self.assertEqual(sdk.stops, 1)

    def test_ambiguous_device_selection_is_rejected(self):
        sdk = SDK()
        sdk.devices = [sdk, sdk]
        with patch.dict("sys.modules", {"pyrealsense2": sdk}):
            camera = RealSenseCamera()
        with self.assertRaisesRegex(RuntimeError, "--serial"):
            camera.open()


if __name__ == "__main__":
    unittest.main()
