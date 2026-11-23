"""Regression tests from the v0.2 lifecycle/code review."""
import contextlib
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import unittest
from unittest.mock import Mock, patch

from jetson import robot
from jetson.control.arduino_bridge import ArduinoMonitor
from jetson.core.bus import LatestMailbox
from jetson.core.config import ROOT, Settings
from jetson.core.contracts import Command, parse_observation
from jetson.core.recording import RunRecorder
from jetson.core.vision_process import VisionProcess
from jetson.tests.test_robot import packet, temporary


class HealthyVision:
    """Deterministic input for lifecycle fault tests; never opens a device or process."""
    def __init__(self, *_):
        self.mailbox = LatestMailbox()
        self.state, self.restarts, self.closed = "starting", 0, False

    def start(self):
        self.state = "running"

    def tick(self, now):
        self.mailbox.put(packet(now))

    def health(self):
        return {"state": self.state, "pid": None, "restarts": 0, "error": None}

    def close(self):
        self.closed, self.state = True, "stopped"


class RuntimeEdgeTests(unittest.TestCase):
    def test_signal_during_queue_critical_section_does_not_deadlock(self):
        # Isolate the test so a future regression cannot deadlock the test runner.
        # No child worker or hardware exists in this subprocess.
        code = '''
import signal, sys
from unittest.mock import patch
from jetson import robot
from jetson.core.bus import CommandInbox
from jetson.tests.test_runtime_edges import HealthyVision
original = CommandInbox.drain
injected = False
def interrupt_drain(self):
    global injected
    if not injected:
        injected = True
        with self._lock:
            signal.raise_signal(signal.SIGINT)
    return original(self)
with patch.object(CommandInbox, "drain", interrupt_drain), patch.object(robot, "VisionProcess", HealthyVision):
    raise SystemExit(robot.main(["--no-web", "--duration", "1", "--output-dir", sys.argv[1]]))
'''
        with temporary() as directory:
            completed = subprocess.run([sys.executable, "-c", code, directory], cwd=ROOT,
                                       capture_output=True, text=True, encoding="utf-8", timeout=5,
                                       env={**os.environ, "PYTHONIOENCODING": "utf-8"})
            self.assertEqual(completed.returncode, 0, completed.stderr)
            summary = json.loads(next(Path(directory).glob("*/summary.json")).read_text(encoding="utf-8"))
            self.assertEqual(summary["reason"], "operator_shutdown")
            self.assertEqual(summary["cleanup_errors"], [])

    def test_failed_state_output_still_closes_resources_and_restores_signals(self):
        vision = HealthyVision()
        previous = signal.getsignal(signal.SIGINT)
        with temporary() as directory, patch.object(robot, "VisionProcess", return_value=vision), \
                patch.object(RunRecorder, "state", side_effect=OSError("injected disk full")), \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(robot.main(["--no-web", "--duration", ".1", "--output-dir", directory]), 2)
            summary = json.loads(next(Path(directory).glob("*/summary.json")).read_text(encoding="utf-8"))
            self.assertIn("disk full", summary["output_errors"][0])
            self.assertEqual(summary["exit_code"], 2)
            self.assertIn("STOPPED", summary["states_seen"])
        self.assertTrue(vision.closed)
        self.assertIs(signal.getsignal(signal.SIGINT), previous)

    def test_failed_summary_is_reported_on_stdout_with_error_exit(self):
        vision, stdout = HealthyVision(), io.StringIO()
        original = robot.write_json
        previous = signal.getsignal(signal.SIGTERM)
        def write(path, value):
            if Path(path).name == "summary.json":
                raise OSError("injected summary failure")
            return original(path, value)
        with temporary() as directory, patch.object(robot, "VisionProcess", return_value=vision), \
                patch.object(robot, "write_json", side_effect=write), contextlib.redirect_stdout(stdout), \
                contextlib.redirect_stderr(io.StringIO()):
            result = robot.main(["--no-web", "--duration", ".1", "--output-dir", directory])
        final = json.loads(stdout.getvalue().splitlines()[-1])
        self.assertEqual(result, 2)
        self.assertEqual(final["exit_code"], 2)
        self.assertIn("summary failure", final["output_errors"][0])
        self.assertTrue(vision.closed)
        self.assertIs(signal.getsignal(signal.SIGTERM), previous)

    def test_jsonl_write_error_is_not_silently_swallowed(self):
        with temporary() as directory:
            recorder = RunRecorder(directory)
            stream = Mock()
            stream.tell.return_value = 0
            stream.write.side_effect = OSError("injected log failure")
            try:
                with patch.object(recorder.handlers["events"], "stream", stream), self.assertRaises(OSError):
                    recorder.append("events", {"type": "test"})
            finally:
                recorder.close()

    def test_monitor_reports_transport_close_failure_and_disconnects_state(self):
        client = Mock()
        client.request.return_value = {"type": "status", "state": "SAFE", "faults": 0}
        client.transport.close.side_effect = OSError("injected close failure")
        cfg = Settings().arduino
        cfg.mode, cfg.port = "monitor", "test-only"
        monitor = ArduinoMonitor(cfg, lambda: client)
        try:
            monitor.start()
            until = time.monotonic() + 2
            while not monitor.snapshot()["connected"] and time.monotonic() < until:
                time.sleep(.01)
            self.assertTrue(monitor.snapshot()["connected"])
        finally:
            self.assertFalse(monitor.close())
        self.assertFalse(monitor.snapshot()["connected"])
        self.assertIsNone(monitor.snapshot()["status"])
        self.assertIn("serial_close_failed", monitor.snapshot()["error"])

    def test_schema_and_command_types_fail_cleanly(self):
        for version in (True, 1.0, "1"):
            message = packet(10)
            message["schema_version"] = version
            self.assertFalse(parse_observation(message, True, 10, .5).valid)
        for kind in ([], {}, None):
            with self.assertRaises(ValueError):
                Command.create(kind, 10)
        for now in (True, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                Command.create("text", now, "hello")

    def test_non_json_worker_output_is_rejected_and_process_reaped(self):
        cfg = Settings().vision
        cfg.max_restarts = 0
        with temporary() as directory:
            worker = VisionProcess(cfg, directory)
            worker._command = lambda: [sys.executable, "-u", "-c", "import time; print('not-json'); time.sleep(60)"]
            try:
                worker.start()
                until = time.monotonic() + 4
                while worker.state != "failed" and time.monotonic() < until:
                    worker.tick(time.monotonic())
                    time.sleep(.01)
                self.assertEqual(worker.state, "failed")
                self.assertIsNotNone(worker.reader_error)
                self.assertIsNone(worker.mailbox.get()[1])
                self.assertIsNone(worker.process)
            finally:
                worker.close()


if __name__ == "__main__":
    unittest.main()
