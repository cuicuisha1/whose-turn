import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, build_opener, ProxyHandler

from jetson.control.arduino_bridge import ArduinoMonitor
from jetson.control.head_controller import VirtualHead
from jetson.core.bus import CommandInbox, LatestMailbox, StateStore
from jetson.core.config import ROOT, Settings, load_settings
from jetson.core.contracts import Command, Target, parse_observation
from jetson.core.engine import RobotEngine
from jetson.core.inputs import Scenario
from jetson.core.vision_process import VisionProcess
from jetson.interface.server import StatusServer

TEMP = ROOT / "jetson/runs/test-temp"
urlopen = build_opener(ProxyHandler({})).open  # local API tests must never use a system HTTP proxy


def temporary():
    TEMP.mkdir(parents=True, exist_ok=True)
    return tempfile.TemporaryDirectory(dir=TEMP)


def packet(now, *, distance=1.5, ident=1, generation=1, target=True):
    return {"schema_version": 1, "type": "perception", "run_id": "test",
            "source": "simulation", "simulated": True, "valid": True,
            "camera_generation": generation, "person_count": int(target),
            "frame": {"received_monotonic_s": now},
            "target": {"track_id": ident, "center_x": .8, "center_y": .2,
                       "distance_m": distance, "depth": {"valid": distance is not None}} if target else None}


HEALTH = {"state": "running", "error": None}
ARDUINO = {"mode": "disabled", "connected": False, "received_at": None, "status": None}


class ConfigTests(unittest.TestCase):
    def test_examples_load(self):
        self.assertEqual(load_settings(ROOT / "jetson/config/simulation.json").vision.source, "simulation")
        self.assertEqual(load_settings(ROOT / "jetson/config/hardware_observe.json").vision.source, "realsense")

    def test_strict_config_rejects_unknown_duplicates_nonfinite_and_wrong_types(self):
        cases = ['{"extra": {}}', '{"runtime":{"oops":1}}', '{"vision":{"fps":true}}',
                 '{"runtime":{"tick_hz":NaN}}', '{"audio":{},"audio":{}}',
                 '{"control":{"mode":"hardware"}}', '{"arduino":{"mode":"monitor"}}',
                 '{"vision":{"source":"realsense","detector":"synthetic"}}',
                 '{"interface":{"port":4.5}}', '{"audio":{"mode":"tts"}}']
        with temporary() as directory:
            path = Path(directory) / "config.json"
            for text in cases:
                path.write_text(text, encoding="utf-8")
                with self.subTest(text=text), self.assertRaises(ValueError):
                    load_settings(path)

    def test_invalid_scenario_rejected_before_runtime(self):
        with temporary() as directory:
            path = Path(directory) / "scenario.json"
            for value in ([{"at_s": float("nan"), "kind": "pause"}],
                          [{"at_s": 2, "kind": "pause"}, {"at_s": 1, "kind": "resume"}],
                          [{"at_s": 1, "kind": "move"}], [{"at_s": 1, "kind": "text", "text": ""}]):
                path.write_text(json.dumps(value), encoding="utf-8")
                with self.assertRaises(ValueError):
                    Scenario(path)


class BusAndContractTests(unittest.TestCase):
    def test_latest_mailbox_owns_copies_and_replaces_backlog(self):
        bus = LatestMailbox()
        original = {"x": [1]}
        bus.put(original)
        original["x"][0] = 9
        self.assertEqual(bus.get()[1]["x"], [1])
        bus.get()[1]["x"].append(2)
        for i in range(100):
            bus.put(i)
        self.assertEqual(bus.get(), (101, 99))

    def test_bounded_commands_pause_flushes_and_shutdown_is_terminal(self):
        inbox = CommandInbox(2)
        inbox.post(Command.create("resume", 1))
        inbox.post(Command.create("text", 1, "你好"))
        self.assertFalse(inbox.post(Command.create("resume", 1)))
        self.assertTrue(inbox.post(Command.create("pause", 1)))
        self.assertEqual([c.kind for c in inbox.drain()], ["pause"])
        inbox.post(Command.create("shutdown", 1))
        self.assertFalse(inbox.post(Command.create("pause", 1)))
        self.assertFalse(inbox.post(Command.create("resume", 1)))
        self.assertEqual([c.kind for c in inbox.drain()], ["shutdown"])

    def test_source_future_and_expired_packets_are_invalid(self):
        self.assertFalse(parse_observation(packet(10), False, 10, .5).valid)
        for now in (9.9, 10.51):
            self.assertFalse(parse_observation(packet(10), True, now, .5).valid)
        self.assertTrue(parse_observation(packet(10), True, 10.1, .5).valid)

    def test_invalid_targets_rejected_and_unknown_depth_cleared(self):
        for field, value in (("center_x", float("nan")), ("center_y", 2), ("track_id", True), ("distance_m", float("inf"))):
            data = packet(10)
            data["target"][field] = value
            self.assertFalse(parse_observation(data, True, 10, .5).valid)
        data = packet(10)
        data["target"]["depth"]["valid"] = False
        self.assertIsNone(parse_observation(data, True, 10, .5).target.distance_m)


class EngineTests(unittest.TestCase):
    def setUp(self):
        self.config = Settings()
        self.engine = RobotEngine(self.config, "unit", 0)

    def step(self, at, data=None, commands=(), health=None, arduino=None):
        return self.engine.step(packet(at) if data is None else data, health or HEALTH,
                                arduino or ARDUINO, commands, at)

    def confirm(self, at=10):
        self.step(at)
        return self.step(at + .4)

    def test_confirmation_requires_new_capture_time(self):
        self.assertEqual(self.step(10)["behavior"]["state"], "PERSON_DETECTED")
        repeated = self.step(10.4, packet(10))
        self.assertEqual(repeated["behavior"]["state"], "PERSON_DETECTED")
        self.assertFalse(repeated["head"]["active"])
        self.assertEqual(self.step(10.41)["behavior"]["state"], "TRACKING")

    def test_timestamp_regression_invalidates_target(self):
        self.confirm()
        state = self.step(10.5, packet(10.3))
        self.assertFalse(state["perception"]["valid"])
        self.assertFalse(state["head"]["active"])

    def test_loss_immediately_zeros_motion_and_reconfirmation_required(self):
        self.assertTrue(self.confirm()["head"]["active"])
        lost = self.step(10.45, packet(10.45, target=False))
        self.assertEqual(lost["behavior"]["state"], "IDLE")
        self.assertEqual(lost["head"]["pan_velocity_deg_s"], 0)
        self.assertEqual(self.step(10.5)["behavior"]["state"], "PERSON_DETECTED")

    def test_stale_perception_clears_caption_and_action(self):
        self.confirm()
        state = self.step(10.45, commands=[Command.create("text", 10.45, "距离")])
        self.assertIn("1.50", state["audio"]["caption"])
        stale = self.step(11, packet(10.45))
        self.assertEqual(stale["behavior"]["state"], "PERCEPTION_UNAVAILABLE")
        self.assertIsNone(stale["audio"]["caption"])
        self.assertIsNone(stale["perception"]["target"])
        self.assertEqual(stale["head"]["pan_velocity_deg_s"], 0)

    def test_camera_reconnect_and_target_change_restart_confirmation(self):
        self.confirm()
        state = self.step(10.5, packet(10.5, generation=2))
        self.assertEqual(state["behavior"]["state"], "PERSON_DETECTED")
        state = self.step(10.9, packet(10.9, generation=2, ident=3))
        self.assertEqual(state["behavior"]["state"], "PERSON_DETECTED")

    def test_missing_depth_clears_distance_and_old_distance_caption(self):
        self.confirm()
        self.step(10.45, commands=[Command.create("text", 10.45, "距离")])
        state = self.step(10.5, packet(10.5, distance=None))
        self.assertEqual(state["behavior"]["reason"], "depth_unknown")
        self.assertIsNone(state["perception"]["target"]["distance_m"])
        self.assertIsNone(state["audio"]["caption"])

    def test_close_person_changes_behavior_without_chassis_commands(self):
        self.confirm()
        state = self.step(10.5, packet(10.5, distance=.4))
        self.assertEqual(state["behavior"]["state"], "KEEP_DISTANCE")
        self.assertNotIn("chassis", state)
        self.assertNotIn("chassis_policy", state["capabilities"])
        self.assertFalse(state["system"]["physical_actuation"])

    def test_pause_resume_and_delayed_resume(self):
        self.confirm()
        self.assertEqual(self.step(10.5, commands=[Command.create("pause", 1)])["behavior"]["state"], "PAUSED")
        self.assertEqual(self.step(10.6, commands=[Command.create("resume", 1)])["behavior"]["state"], "PAUSED")
        self.assertEqual(self.step(10.7, commands=[Command.create("resume", 10.7)])["behavior"]["state"], "PERSON_DETECTED")

    def test_shutdown_terminal_even_with_following_resume(self):
        state = self.step(10, commands=[Command.create("shutdown", 10), Command.create("resume", 10)])
        self.assertEqual(state["system"]["status"], "stopped")
        self.assertIsNone(state["perception"]["target"])
        self.assertEqual(self.step(10.2)["behavior"]["state"], "STOPPED")

    def test_fault_latches_reset_only_when_cause_cleared(self):
        failed = {"state": "failed", "error": "injected"}
        state = self.step(10, health=failed, commands=[Command.create("reset", 10)])
        self.assertEqual(state["behavior"]["state"], "FAULT")
        self.assertEqual(self.step(10.1)["behavior"]["state"], "FAULT")
        self.assertEqual(self.step(10.2, commands=[Command.create("reset", 10.2)])["behavior"]["state"], "PERSON_DETECTED")

    def test_arduino_stale_or_fault_latches_without_clear_command(self):
        self.config.arduino.mode = "monitor"
        fresh = {"connected": True, "received_at": 10, "status": {"state": "SAFE", "faults": 0}}
        self.assertEqual(self.step(10, arduino=fresh)["faults"], [])
        state = self.step(12, arduino=fresh)
        self.assertIn("arduino_telemetry_unavailable", state["faults"])
        fresh["received_at"] = 12.1
        fresh["status"]["faults"] = 4
        state = self.step(12.1, arduino=fresh, commands=[Command.create("reset", 12.1)])
        self.assertIn("arduino_fault_reported", state["faults"])

    def test_speech_and_live_actuation_remain_explicitly_unavailable(self):
        state = self.step(10, commands=[Command.create("text", 10, "你好")])
        self.assertTrue(state["audio"]["caption"])
        for key in ("microphone_active", "asr_available", "tts_available", "speech_playing"):
            self.assertFalse(state["audio"][key])
        self.assertFalse(state["head"]["physical_output"])
        self.assertNotIn("chassis", state)

    def test_api_independently_expires_state_if_loop_stalls(self):
        state = self.confirm()
        store = StateStore()
        store.put(state)
        stale = store.read(10.95)
        self.assertFalse(stale["perception"]["valid"])
        self.assertFalse(stale["head"]["active"])
        self.assertFalse(stale["behavior"]["head_allowed"])
        self.assertEqual(store.read(12)["system"]["status"], "stale")
        self.assertTrue(store.get()[1]["perception"]["valid"])

    def test_virtual_head_rate_and_angle_limits(self):
        head = VirtualHead(self.config.control)
        target = Target(1, 1, 0, 1)
        for i in range(200):
            state = head.update(target, True, i * .1)
            self.assertLessEqual(abs(state["pan_velocity_deg_s"]), 25.000001)
            self.assertLessEqual(abs(state["pan_deg"]), 35)
            self.assertLessEqual(abs(state["tilt_deg"]), 20)
        stopped = head.update(target, False, 20)
        self.assertEqual(stopped["pan_velocity_deg_s"], 0)

    def test_api_expiry_does_not_hide_latched_fault(self):
        state = self.step(10, health={"state": "failed"})
        store = StateStore()
        store.put(state)
        expired = store.read(12)
        self.assertEqual(expired["system"]["status"], "stale")
        self.assertEqual(expired["behavior"]["state"], "FAULT")
        self.assertFalse(expired["perception"]["valid"])

    def test_reported_arduino_fault_takes_effect_during_startup_grace(self):
        self.config.arduino.mode = "monitor"
        state = self.step(.1, arduino={"connected": True, "received_at": .1,
                                      "status": {"state": "FAULT", "faults": 1}})
        self.assertIn("arduino_fault_reported", state["faults"])
        self.assertEqual(state["behavior"]["state"], "FAULT")


class AdapterTests(unittest.TestCase):
    def test_monitor_only_queries_status_and_closes_transport(self):
        class Client:
            def __init__(self):
                self.calls, self.closed, self.transport = [], False, self
            def request(self, kind):
                self.calls.append(kind)
                return {"type": "status", "state": "SAFE", "faults": 0}
            def close(self):
                self.closed = True
        client = Client()
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
            self.assertTrue(monitor.close())
        self.assertEqual(client.calls, ["status"])
        self.assertTrue(client.closed)
        self.assertFalse(monitor.snapshot()["connected"])
        self.assertIsNone(monitor.snapshot()["status"])

    def test_monitor_against_existing_native_firmware_simulator(self):
        exe = ROOT / "build/tests" / ("simulator.exe" if os.name == "nt" else "simulator")
        if not exe.is_file():
            self.skipTest("Native firmware simulator is not built on this platform")
        from host.robot_test import Client, SimulationTransport
        cfg = Settings().arduino
        cfg.mode, cfg.port = "monitor", "native-simulator"
        monitor = ArduinoMonitor(cfg, lambda: Client(SimulationTransport(exe), timeout=.5))
        try:
            monitor.start()
            until = time.monotonic() + 3
            while not monitor.snapshot()["connected"] and time.monotonic() < until:
                time.sleep(.01)
            status = monitor.snapshot()
            self.assertTrue(status["connected"], status)
            # Firmware intentionally boots with BootReset=1; monitor must not clear it.
            self.assertEqual(status["status"]["faults"], 1)
            self.assertEqual(status["status"]["state"], "FAULT")
        finally:
            self.assertTrue(monitor.close())

    def test_http_readonly_status_and_no_file_server(self):
        cfg = Settings().interface
        cfg.port = 0
        store = StateStore()
        now = time.monotonic()
        store.put(RobotEngine(Settings(), "http", now).step(packet(now), HEALTH, ARDUINO, [], now))
        server = StatusServer(cfg, store)
        try:
            server.start()
            with urlopen(server.url + "api/status", timeout=2) as response:
                data = json.load(response)
                self.assertEqual(response.headers["Cache-Control"], "no-store")
                self.assertFalse(data["system"]["physical_actuation"])
            with urlopen(server.url, timeout=2) as response:
                self.assertIn("机器人运行台", response.read().decode("utf-8"))
            for request, expected in ((server.url + "../config/simulation.json", 404),
                                      (Request(server.url + "api/status", data=b"{}", method="POST"), 501)):
                with self.assertRaises(HTTPError) as context:
                    urlopen(request, timeout=2)
                self.assertEqual(context.exception.code, expected)
        finally:
            server.close()
        self.assertFalse(server.thread.is_alive())

    def test_worker_exit_restart_limit_and_mailbox_invalidation(self):
        cfg = Settings().vision
        cfg.max_restarts = 1
        with temporary() as directory:
            worker = VisionProcess(cfg, directory)
            worker._command = lambda: [sys.executable, "-c", "raise SystemExit(7)"]
            try:
                worker.start()
                until = time.monotonic() + 5
                while worker.state != "failed" and time.monotonic() < until:
                    worker.tick(time.monotonic())
                    time.sleep(.02)
                self.assertEqual(worker.state, "failed")
                self.assertEqual(worker.restarts, 1)
                self.assertIsNone(worker.mailbox.get()[1])
                self.assertIsNone(worker.process)
            finally:
                worker.close()

    def test_silent_worker_killed_and_reaped(self):
        cfg = Settings().vision
        cfg.max_restarts = 0
        with temporary() as directory:
            worker = VisionProcess(cfg, directory)
            worker._command = lambda: [sys.executable, "-c", "import time; time.sleep(60)"]
            try:
                worker.start()
                worker.last_packet = time.monotonic() - 100
                until = time.monotonic() + 4
                while worker.state != "failed" and time.monotonic() < until:
                    worker.tick(time.monotonic())
                    time.sleep(.01)
                self.assertEqual(worker.state, "failed")
                self.assertEqual(worker.error, "vision_process_silent")
                self.assertIsNone(worker.process)
            finally:
                worker.close()

    def test_full_runtime_with_real_child_scenario_and_cleanup(self):
        with temporary() as directory:
            scenario = Path(directory) / "scenario.json"
            scenario.write_text(json.dumps([
                {"at_s": 1.7, "kind": "text", "text": "你好"},
                {"at_s": 2, "kind": "pause"}, {"at_s": 2.4, "kind": "resume"},
                {"at_s": 3.4, "kind": "shutdown"}]), encoding="utf-8")
            completed = subprocess.run([sys.executable, "-m", "jetson.robot", "--duration", "6", "--port", "0",
                                        "--scenario", str(scenario), "--output-dir", str(Path(directory) / "runs")],
                                       cwd=ROOT, capture_output=True, text=True, encoding="utf-8", timeout=20,
                                       env={**os.environ, "PYTHONIOENCODING": "utf-8"})
            self.assertEqual(completed.returncode, 0, completed.stderr)
            run = next((Path(directory) / "runs").iterdir())
            summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
            self.assertGreater(summary["valid_perception_ticks"], 0)
            self.assertGreater(summary["target_ticks"], 0)
            self.assertEqual(summary["cleanup_errors"], [])
            self.assertTrue({"PAUSED", "TRACKING", "STOPPED"}.issubset(summary["states_seen"]))
            latest = json.loads((run / "latest.json").read_text(encoding="utf-8"))
            self.assertEqual(latest["system"]["status"], "stopped")
            self.assertIsNone(latest["perception"]["target"])
            self.assertIsNone(latest["modules"]["vision"]["pid"])
            self.assertFalse(latest["head"]["active"])
            states = [json.loads(line) for line in (run / "state.jsonl").read_text(encoding="utf-8").splitlines()]
            self.assertTrue(any(s["audio"]["caption"] for s in states))
            self.assertTrue(all(not s["head"]["physical_output"] for s in states))


if __name__ == "__main__":
    unittest.main()
