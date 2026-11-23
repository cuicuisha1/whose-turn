"""Concurrent speech and resource cleanup through the public simulation/CLI.

Only device adapters are substituted in live-loop tests. These tests verify
program decisions and output files; they do not validate hardware timing.
"""
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import robot_phase72 as cli
from mox72.config import Config
from mox72.head_serial import SimHead
from mox72.models import FaceObservation
from mox72.runtime_io import Journal, SimPlayer
from mox72.simulation import SimClock, run_simulation
from mox72.vendor.live_audio import TimedAudio
from mox72.vendor.policies import AudioFrame, AudioStatus


class ConcurrentSimulationTests(unittest.TestCase):
    def test_short_and_long_questions_keep_join_after_both_gates(self):
        for condition in ("fixed", "pause"):
            for duration in (.05, 5.0):
                with self.subTest(condition=condition, prompt_duration=duration):
                    events = []
                    def emit(event, **fields):
                        events.append({"event": event, **fields})
                    def player_factory(callback):
                        return SimPlayer(callback, {"welcome": .2, "prompt": duration})
                    with patch("mox72.simulation.SimPlayer", side_effect=player_factory):
                        cfg = Config(tilt_sign=-1)
                        result = run_simulation(cfg, condition, "full", "silence", emit)
                    self.assertEqual(result["state"], "COMPLETED", result)
                    select = lambda kind: [e for e in events if e["event"] == kind]
                    prompt_starts = select("PROMPT_STARTED")
                    prompt_ends = select("PROMPT_FINISHED")
                    self.assertEqual(len(prompt_starts), 1)
                    self.assertEqual(len(prompt_ends), 1)
                    self.assertEqual(prompt_starts[0]["concurrent_with"], "CENTER_A")
                    center_start = next(e for e in select("FACE_CENTER_STARTED") if e["role"] == "A")
                    locked = next(e for e in select("FACE_LOCKED") if e["role"] == "A")
                    self.assertEqual(prompt_starts[0]["time_s"], center_start["time_s"])
                    self.assertLess(prompt_starts[0]["time_s"], locked["time_s"])
                    if duration < 1:
                        self.assertLess(prompt_ends[0]["time_s"], locked["time_s"])
                    else:
                        self.assertLess(locked["time_s"], prompt_ends[0]["time_s"])
                    self.assertGreaterEqual(result["join_received_at_s"], locked["time_s"])
                    self.assertGreaterEqual(result["join_received_at_s"],
                                            prompt_ends[0]["time_s"] + cfg.post_prompt_guard_s)
                    self.assertFalse(any(e.get("policy_active") for e in select("AUDIO_OBSERVED")
                                         if e["time_s"] < result["join_received_at_s"]))
                    self.assertEqual([e["clip"] for e in select("PLAYBACK_STARTED")],
                                     ["welcome", "prompt"])


class LiveLoopCancellationTests(unittest.TestCase):
    def exercise_cancel(self, command):
        clock = SimClock()
        objects, trace = {}, []

        class Camera:
            ready = True
            def __init__(self, *args, **kwargs):
                self.sequence = 0
            def start(self):
                pass
            def latest(self, now):
                self.sequence += 1
                return FaceObservation(now, self.sequence, 1, 1,
                                       (0., 0., 1.), 0., 0., score=.99)
            def close(self):
                trace.append("camera.close")

        class Head(SimHead):
            def __init__(self, port, emit, **kwargs):
                super().__init__(emit, clock=clock,
                                 pan_limit=kwargs["pan_limit"], tilt_limit=kwargs["tilt_limit"])
                objects["head"] = self
            def stop(self, reason="operator_stop"):
                trace.append("head.stop:" + reason)
                super().stop(reason)
            def close(self):
                trace.append("head.close")
                super().close()

        class Audio:
            ready = True
            def __init__(self, *args, **kwargs):
                self.end = clock()
                self.sequence = 0
            def start(self):
                pass
            def drain(self, now):
                if now <= self.end:
                    return []
                self.sequence += 1
                frame = AudioFrame(self.end, now, AudioStatus.UNVOICED)
                self.end = now
                return [TimedAudio(frame, now, self.sequence, -55.)]
            def close(self):
                trace.append("audio.close")

        class Player(SimPlayer):
            def __init__(self, clips, emit, directory, **kwargs):
                super().__init__(emit, {"welcome": .2, "prompt": 5.})
                objects["player"] = self
            def stop(self):
                trace.append("player.stop")
                super().stop()
            def close(self):
                trace.append("player.close")
                self.stop()

        class Console:
            def start(self):
                pass
            def drain(self):
                player = objects["player"]
                if player.name == "prompt" and player.until is not None:
                    if command == "CTRL_C":
                        raise KeyboardInterrupt
                    return [command]
                return []

        def sleep(seconds):
            clock.now += seconds

        args = SimpleNamespace(scope="full", welcome_wav=Path("unused"), prompt_wav=Path("unused"),
                               audio_output="unused", audio_input="unused", channel=0,
                               vad_mode=2, gate_dbfs=-48., model=Path("unused"),
                               camera_serial="unused", preview=False, port="unused", condition="pause")
        with tempfile.TemporaryDirectory() as tmp, \
                patch.object(cli, "VisionCamera", Camera), patch.object(cli, "SerialHead", Head), \
                patch.object(cli, "AudioCapture", Audio), patch.object(cli, "WavePlayer", Player), \
                patch.object(cli, "Console", Console), patch.object(cli.time, "monotonic", clock), \
                patch.object(cli.time, "sleep", sleep), contextlib.redirect_stdout(io.StringIO()):
            journal = Journal(Path(tmp), "cancel-test", "pair", clock=clock)
            try:
                result = cli.run_live(args, Config(tilt_sign=-1), journal)
                events = [json.loads(line) for line in
                          (journal.directory / "events.jsonl").read_text().splitlines()]
            finally:
                journal.close()
        return result, events, trace, objects

    def test_stop_quit_and_ctrl_c_cancel_during_question_and_close_every_device(self):
        for command in ("STOP", "QUIT", "CTRL_C"):
            with self.subTest(command=command):
                result, events, trace, objects = self.exercise_cancel(command)
                self.assertEqual(result["state"], "ABORTED")
                expected = "keyboard_interrupt" if command == "CTRL_C" else "operator_" + command.lower()
                self.assertEqual(result["fault"], expected)
                self.assertEqual(result["cleanup_errors"], [])
                self.assertIn("prompt", objects["player"].played)
                self.assertIsNone(objects["player"].until)
                self.assertFalse(any(e["event"] == "JOIN_MARK_RECEIVED" for e in events))
                self.assertFalse(any(e.get("reason") == "turn_to_b_base" for e in events))
                for name in ("player", "head", "audio", "camera"):
                    self.assertIn(name + ".close", trace)
                self.assertTrue(trace[0].startswith("head.stop:"), trace)


class ResultPublicationTests(unittest.TestCase):
    def test_cleanup_fault_cannot_publish_success_even_after_completed_controller(self):
        for cleanup in ("camera: forced worker termination", "head: STOP acknowledgement timeout"):
            with self.subTest(cleanup=cleanup), tempfile.TemporaryDirectory() as tmp:
                completed = {"state": "COMPLETED", "fault": None, "runtime_fault": None,
                             "cleanup_errors": [cleanup], "logging_failed": False,
                             "command_pose_a": {"pan": 45., "tilt": 12.},
                             "command_pose_b": {"pan": -45., "tilt": 10.},
                             "position_feedback": False, "simulated": False,
                             "motion_simulated": False}
                with patch.object(cli, "requirements", return_value={}), \
                        patch.object(cli, "run_live", return_value=completed), \
                        patch.object(cli.sys.stdin, "isatty", return_value=True), \
                        contextlib.redirect_stdout(io.StringIO()):
                    code = cli.main(["--mode", "live", "--scope", "full", "--condition", "fixed",
                                     "--port", "unused", "--tilt-sign", "-1", "--trial-id", "result-test",
                                     "--pair-id", "pair", "--output", tmp])
                self.assertEqual(code, 2)
                directory, = Path(tmp).iterdir()
                result = json.loads((directory / "summary.json").read_text())
                targets = json.loads((directory / "face_targets.json").read_text())
                events = [json.loads(line) for line in (directory / "events.jsonl").read_text().splitlines()]
                self.assertFalse(result["engineering_run_ok"])
                self.assertEqual(result["cleanup_errors"], [cleanup])
                self.assertFalse(targets["engineering_run_ok"])
                self.assertFalse(targets["reuse_in_future_trial"])
                end = next(e for e in events if e["event"] == "SESSION_END")
                self.assertFalse(end["engineering_run_ok"])


if __name__ == "__main__":
    unittest.main()
