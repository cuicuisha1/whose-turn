"""End-to-end recovery scenarios through simulation and the real controller.

All positions and observations here are synthetic, including the 2.4 m
movement. Passing these checks is not evidence of physical station compliance.
"""
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from mox72.config import Config
from mox72.models import FaceObservation
from mox72.simulation import run_simulation


class MovementIntegrationTests(unittest.TestCase):
    def test_moving_cli_keeps_station_quality_with_the_saved_angle_file(self):
        bundle = Path(__file__).resolve().parents[1]
        for condition in ("fixed", "pause"):
            with self.subTest(condition=condition), tempfile.TemporaryDirectory() as directory:
                result = subprocess.run(
                    [sys.executable, "-B", "-S", str(bundle / "robot_phase72.py"),
                     "--mode", "simulate", "--scope", "full", "--scenario", "moving",
                     "--condition", condition, "--trial-id", "moving-cli",
                     "--pair-id", "synthetic-pair", "--output", directory],
                    capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stderr + result.stdout[-2000:])
                output, = Path(directory).iterdir()
                summary = json.loads((output / "summary.json").read_text())
                targets = json.loads((output / "face_targets.json").read_text())
                metadata = json.loads((output / "config.json").read_text())
                self.assertTrue(summary["engineering_run_ok"])
                self.assertEqual(summary["state"], "COMPLETED")
                self.assertEqual(targets["software_version"], "0.2.3")
                self.assertEqual(targets["software_version"], summary["software_version"])
                self.assertEqual(targets["centering_quality"], summary["centering_quality"])
                self.assertEqual(targets["station_compliance"], "requires_external_review")
                self.assertFalse(targets["same_person_verified"])
                self.assertFalse(targets["reuse_in_future_trial"])
                self.assertTrue(targets["simulated"])
                self.assertEqual(targets["config_sha256"], metadata["config_sha256"])
                for role in ("A", "B"):
                    self.assertTrue(targets["centering_quality"][role]["locked_outside_initial_station"])

    def exercise(self, condition="fixed", *, ambiguous_after_center=False):
        cfg = Config(tilt_sign=-1)
        events = []
        current = {"state": "NEW"}

        def emit(event, **fields):
            events.append({"event": event, **fields})
            if event == "STATE":
                current["state"] = fields["state"]

        def observations(*args, **kwargs):
            face = FaceObservation(*args, **kwargs)
            if ambiguous_after_center and current["state"] == "CENTER_A":
                # Fresh camera frames continue, but two faces cannot identify
                # the sole role candidate. Do not turn this into camera loss.
                return replace(face, count=2, track_id=None, point_m=None,
                               horizontal_deg=None, vertical_deg=None,
                               error="MULTIPLE_FACES")
            return face

        with patch("mox72.simulation.FaceObservation", side_effect=observations):
            result = run_simulation(cfg, condition, "full", "moving", emit)
        return cfg, result, events

    def test_24m_movement_and_brief_face_loss_recover_in_both_conditions(self):
        for condition, reason, delay in (("fixed", "fixed_delay", 1.),
                                         ("pause", "speech_pause", .6)):
            with self.subTest(condition=condition):
                cfg, result, events = self.exercise(condition)
                self.assertEqual(result["state"], "COMPLETED", result)
                self.assertIsNone(result["fault"])
                self.assertTrue(result["a_locked"])
                self.assertTrue(result["b_locked"])
                self.assertTrue(result["simulated"])
                self.assertTrue(result["motion_simulated"])
                self.assertFalse(result["position_feedback"])
                self.assertFalse(result["same_person_verified"])
                self.assertEqual(result["station_compliance"], "requires_external_review")
                self.assertEqual(result["trigger_reason"], reason)
                self.assertAlmostEqual(result["trigger_due_at_s"]-result["join_received_at_s"], delay)
                starts = [e for e in events if e["event"] == "FACE_RECOVERY_STARTED"]
                ends = [e for e in events if e["event"] == "FACE_RECOVERY_COMPLETED"]
                self.assertGreaterEqual(len(starts), 1)
                self.assertEqual(len(starts), len(ends))
                for event in starts + ends:
                    self.assertFalse(event["same_person_verified"])
                for event in ends:
                    self.assertGreaterEqual(event["candidate_continuity_s"] + 1e-8,
                                            cfg.center_reacquire_s)
                    self.assertLessEqual(event["total_recovery_wait_s"], cfg.center_recovery_budget_s)

                frames = {e["face_sequence"]: e for e in events if e["event"] == "VISION_OBSERVATION"}
                locked = [e for e in events if e["event"] == "FACE_LOCKED"]
                self.assertEqual([e["role"] for e in locked], ["A", "B"])
                for event in locked:
                    face = frames[event["face_sequence"]]
                    self.assertAlmostEqual(face["point_m"][2], 2.4)
                    quality = result["centering_quality"][event["role"]]
                    self.assertTrue(quality["outside_initial_station_seen"])
                    self.assertTrue(quality["locked_outside_initial_station"])
                    self.assertLessEqual(quality["recovery_wait_s"], cfg.center_recovery_budget_s)
                self.assertEqual(result["centering_quality"]["A"]["recovery_completed_count"], len(ends))

    def test_recovery_holds_motion_without_replaying_prompt_or_moving_b_early(self):
        cfg, result, events = self.exercise("pause")
        self.assertEqual(result["state"], "COMPLETED", result)
        playback = [e["clip"] for e in events if e["event"] == "PLAYBACK_STARTED"]
        self.assertEqual(playback, ["welcome", "prompt"])
        prompt_starts = [e for e in events if e["event"] == "PROMPT_STARTED"]
        self.assertEqual(len(prompt_starts), 1)
        prompt_end = next(e for e in events if e["event"] == "PROMPT_FINISHED")
        a_lock = next(e for e in events if e["event"] == "FACE_LOCKED" and e["role"] == "A")
        stable = next(e for e in events if e["event"] == "A_STABLE")
        self.assertGreaterEqual(stable["duration_s"]+1e-8, cfg.stable_duration_s)
        self.assertLess(stable["time_s"], a_lock["time_s"])
        self.assertGreaterEqual(result["join_received_at_s"], a_lock["time_s"])
        self.assertGreaterEqual(result["join_received_at_s"],
                                prompt_end["time_s"] + cfg.post_prompt_guard_s)
        waiting = False
        for event in events:
            if event["event"] == "FACE_RECOVERY_STARTED":
                waiting = True
            elif event["event"] == "FACE_RECOVERY_COMPLETED":
                waiting = False
            elif waiting:
                self.assertNotIn(event["event"], ("HEAD_MOVE_REQUESTED", "HEAD_STOP_REQUESTED"))
        moves = [e for e in events if e["event"] == "HEAD_MOVE_REQUESTED"]
        b_move = next(e for e in moves if e["reason"] == "turn_to_b_base")
        self.assertGreaterEqual(b_move["time_s"], result["trigger_due_at_s"] - 1e-8)
        self.assertFalse(any(e["target"]["pan"] < 0 and e["time_s"] < b_move["time_s"] for e in moves))
        self.assertEqual(moves[-1]["reason"], "return_to_locked_a")
        self.assertEqual(moves[-1]["target"], result["command_pose_a"])
        self.assertEqual(result["analysis_window_s"], 5.)
        self.assertEqual(result["analysis_window_origin"], "external_video_T0_not_available")

    def test_persistent_multiple_faces_exhaust_recovery_without_silent_target_selection(self):
        for condition in ("fixed", "pause"):
            with self.subTest(condition=condition):
                cfg, result, events = self.exercise(condition, ambiguous_after_center=True)
                self.assertEqual(result["state"], "ABORTED")
                self.assertEqual(result["fault"], "a_center_recovery_budget_exhausted")
                self.assertFalse(result["a_locked"])
                self.assertFalse(result["b_locked"])
                self.assertIsNone(result["join_received_at_s"])
                self.assertFalse(any(e["event"] == "FACE_RECOVERY_COMPLETED" for e in events))
                self.assertFalse(any(e.get("reason") == "turn_to_b_base" for e in events))
                started = next(e for e in events if e["event"] == "FACE_RECOVERY_STARTED")
                failed = next(e for e in events if e["event"] == "FACE_CENTER_FAILED")
                self.assertEqual(started["reason"], "MULTIPLE_FACES")
                self.assertGreater(failed["time_s"]-started["time_s"], cfg.center_recovery_budget_s)
                self.assertLessEqual(failed["time_s"]-started["time_s"], cfg.center_recovery_budget_s+.021)
                self.assertEqual([e["clip"] for e in events if e["event"] == "PLAYBACK_STARTED"],
                                 ["welcome", "prompt"])


if __name__ == "__main__":
    unittest.main()
