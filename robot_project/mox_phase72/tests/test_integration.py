"""Cross-module behaviour and the shipped CLI, with no attached devices."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import robot_phase72 as cli
from mox72.config import Config
from mox72.runtime_io import Journal, file_sha256
from mox72.simulation import run_simulation

BUNDLE = Path(__file__).absolute().parents[1]


class IntegrationTests(unittest.TestCase):
    def exercise(self, condition="pause", scenario="silence", scope="full"):
        events = []
        result = run_simulation(Config(tilt_sign=1), condition, scope, scenario,
                                lambda event, **fields: events.append({"event": event, **fields}))
        return result, events

    def test_complete_conditions_share_centering_rules_but_different_trigger(self):
        results = []
        for condition, due, reason in (("fixed", 1.0, "fixed_delay"), ("pause", .6, "speech_pause")):
            with self.subTest(condition=condition):
                result, events = self.exercise(condition)
                self.assertEqual(result["state"], "COMPLETED")
                self.assertEqual(result["trigger_reason"], reason)
                self.assertAlmostEqual(result["trigger_due_at_s"] - result["join_received_at_s"], due)
                moves = [e for e in events if e["event"] == "HEAD_MOVE_REQUESTED"]
                outgoing = next(e for e in moves if e["reason"] == "turn_to_b_base")
                self.assertGreaterEqual(outgoing["time_s"], result["trigger_due_at_s"] - 1e-8)
                self.assertFalse(any(e["target"]["pan"] < 0 and e["time_s"] < outgoing["time_s"] for e in moves))
                self.assertEqual(moves[-1]["reason"], "return_to_locked_a")
                self.assertEqual(moves[-1]["target"], result["command_pose_a"])
                for e in moves:
                    if e["reason"] == "b_face_correction":
                        self.assertEqual(e["speed_deg_s"], 60)
                self.assertEqual(moves[-1]["speed_deg_s"], 40)
                self.assertEqual(result["analysis_window_origin"], "external_video_T0_not_available")
                self.assertEqual(result["analysis_window_s"], 5)
                self.assertNotIn("T0", result)
                results.append(result)
        self.assertEqual(results[0]["command_pose_a"], results[1]["command_pose_a"])
        self.assertEqual(results[0]["command_pose_b"], results[1]["command_pose_b"])

    def test_continuous_speech_reaches_timeout_without_fabricated_pause(self):
        result, _ = self.exercise(scenario="speech")
        self.assertEqual(result["state"], "COMPLETED")
        self.assertEqual(result["trigger_reason"], "timeout")
        self.assertAlmostEqual(result["trigger_due_at_s"] - result["join_received_at_s"], 3)

    def test_missing_b_fails_and_does_not_generate_return_or_fake_angle(self):
        result, events = self.exercise(scenario="missing_b")
        self.assertEqual(result["state"], "ABORTED")
        self.assertEqual(result["fault"], "b_center_timeout")
        self.assertIsNone(result["command_pose_b"])
        self.assertFalse(any(e.get("reason") == "return_to_locked_a" for e in events))
        self.assertTrue(any(e["event"] == "HEAD_STOP_REQUESTED" for e in events))

    def test_a_only_does_not_start_question_or_b_sequence(self):
        result, events = self.exercise(scope="a")
        self.assertEqual(result["state"], "A_READY")
        self.assertTrue(result["a_locked"])
        self.assertFalse(result["b_locked"])
        self.assertIsNone(result["join_received_at_s"])
        clips = [e["clip"] for e in events if e["event"] == "PLAYBACK_STARTED"]
        self.assertEqual(clips, ["welcome"])

    def test_shipped_cli_runs_without_site_packages_and_logs_distinct_sequences(self):
        for condition in ("fixed", "pause"):
            with self.subTest(condition=condition), tempfile.TemporaryDirectory() as tmp:
                self._check_shipped_cli(condition, tmp)

    def _check_shipped_cli(self, condition, tmp):
        completed = subprocess.run([
            sys.executable, "-B", "-S", str(BUNDLE / "robot_phase72.py"),
            "--mode", "simulate", "--scope", "full", "--condition", condition,
            "--trial-id", "integration-01", "--pair-id", "integration-pair",
            "--output", tmp], capture_output=True, text=True, timeout=10)
        self.assertEqual(completed.returncode, 0, completed.stderr + completed.stdout[-2000:])
        directory, = Path(tmp).iterdir()
        summary = json.loads((directory / "summary.json").read_text())
        target = json.loads((directory / "face_targets.json").read_text())
        config = json.loads((directory / "config.json").read_text())
        events = [json.loads(line) for line in (directory / "events.jsonl").read_text().splitlines()]
        self.assertTrue(summary["engineering_run_ok"])
        self.assertTrue(summary["simulated"])
        self.assertTrue(summary["motion_simulated"])
        self.assertFalse(target["reuse_in_future_trial"])
        self.assertEqual(config["config_sha256"], summary["config_sha256"])
        self.assertEqual([e["sequence"] for e in events], list(range(1, len(events) + 1)))
        self.assertTrue(all(e["trial_id"] == "integration-01" for e in events))
        self.assertTrue(any(e["event"] == "AUDIO_OBSERVED" and "audio_sequence" in e for e in events))

    def test_vendor_sources_are_exact_copies_of_reviewed_versions(self):
        manifest = json.loads((BUNDLE / "SOURCE_REUSE.json").read_text())
        for name, source in manifest.items():
            with self.subTest(name=name):
                self.assertEqual(file_sha256(BUNDLE / name), source["sha256"])
                self.assertFalse(source["modified"])

    def test_camera_failure_during_serial_handshake_prevents_first_move(self):
        opened = {"serial": False}
        moves = []

        class Camera:
            ready = True
            def __init__(self, *a, **kw): pass
            def start(self): pass
            def latest(self, now):
                if opened["serial"]:
                    raise RuntimeError("camera disconnected during handshake")
            def close(self): pass

        class Head:
            def __init__(self, *a, **kw): pass
            def open(self): opened["serial"] = True
            def sample(self, now): pass
            def move(self, *a): moves.append(a)
            def stop(self, reason): pass
            def close(self): pass

        class Player:
            def __init__(self, *a, **kw): pass
            def close(self): pass

        args = SimpleNamespace(scope="a", welcome_wav=Path("unused"), prompt_wav=Path("unused"),
                               audio_output="unused", model=Path("unused"), camera_serial="unused",
                               preview=False, port="unused", condition="pause")
        with tempfile.TemporaryDirectory() as tmp, patch.object(cli, "VisionCamera", Camera), \
                patch.object(cli, "SerialHead", Head), patch.object(cli, "WavePlayer", Player), \
                patch.object(cli.Console, "start"):
            journal = Journal(Path(tmp), "test", "pair")
            try:
                result = cli.run_live(args, Config(tilt_sign=1), journal)
            finally:
                journal.close()
        self.assertEqual(moves, [])
        self.assertEqual(result["state"], "ABORTED")
        self.assertIn("camera disconnected", result["fault"])


if __name__ == "__main__":
    unittest.main()
