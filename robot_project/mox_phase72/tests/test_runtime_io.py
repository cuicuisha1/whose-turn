"""Lifecycle regressions without ALSA, Piper, cameras or serial hardware."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import wave

import prepare_audio
from mox72.runtime_io import AudioCapture, Journal, WavePlayer, wav_info
from mox72.vendor.live_audio import TimedAudio
from mox72.vendor.policies import AudioFrame, AudioStatus


def write_wav(path, *, samples=160, value=0):
    with wave.open(str(path), "wb") as stream:
        stream.setnchannels(1)
        stream.setsampwidth(2)
        stream.setframerate(16000)
        stream.writeframes(int(value).to_bytes(2, "little", signed=True) * samples)


def timed_audio(end=1.0, received=1.0, sequence=1):
    return TimedAudio(AudioFrame(end-.02, end, AudioStatus.UNVOICED),
                      received, sequence, -55.)


class FakeSource:
    def __init__(self, *_args, **_kwargs):
        self.started = self.closed = False

    def start(self):
        self.started = True

    def poll(self):
        raise RuntimeError("injected device failure")

    def close(self):
        self.closed = True


class FakeProcess:
    def __init__(self):
        self.code = None
        self.terminated = self.killed = False

    def poll(self):
        return self.code

    def terminate(self):
        self.terminated = True
        self.code = -15

    def kill(self):
        self.killed = True
        self.code = -9

    def wait(self, timeout=None):
        return self.code


class RuntimeTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def test_wav_checks_actual_pcm_not_just_header(self):
        path = self.root / "test.wav"
        write_wav(path)
        self.assertEqual(wav_info(path)["duration_s"], .01)
        path.write_bytes(path.read_bytes()[:-30])
        with self.assertRaisesRegex(ValueError, "不完整"):
            wav_info(path)

    def test_journal_cannot_overwrite_identity_or_sequence(self):
        journal = Journal(self.root, "trial-a", "pair-a", clock=lambda: 10.)
        self.addCleanup(journal.close)
        journal.emit("AUDIO_CAPTURED", audio_sequence=99, time_s=9.9)
        row = json.loads((journal.directory / "events.jsonl").read_text())
        self.assertEqual((row["sequence"], row["audio_sequence"]), (1, 99))
        with self.assertRaisesRegex(ValueError, "reserved"):
            journal.emit("BAD", trial_id="different")
        self.assertIsNotNone(journal.fault)
        with self.assertRaisesRegex(RuntimeError, "日志已失败"):
            journal.emit("MORE")

    def test_journal_nonfinite_payload_and_clock_are_sticky(self):
        for clock, value in ((lambda: 10., float("nan")), (lambda: float("inf"), 1.)):
            journal = Journal(self.root, "trial", "pair", clock=clock)
            self.addCleanup(journal.close)
            with self.assertRaises(ValueError):
                journal.emit("TEST", nested={"value": value})
            self.assertIsNotNone(journal.fault)

    def test_journal_json_failure_preserves_previous_and_blocks_future(self):
        journal = Journal(self.root, "trial", "pair")
        self.addCleanup(journal.close)
        journal.write_json("summary.json", {"old": True})
        with patch("mox72.runtime_io.os.replace", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                journal.write_json("summary.json", {"new": True})
        self.assertEqual(json.loads((journal.directory / "summary.json").read_text()), {"old": True})
        with self.assertRaises(RuntimeError):
            journal.emit("CANNOT_HIDE_DISK_ERROR")

    def capture(self):
        return AudioCapture(lambda *_args, **_kw: None, self.root,
                            source_factory=FakeSource, clock=lambda: 1.)

    def test_audio_is_inert_until_start_and_cannot_restart_after_close(self):
        audio = self.capture()
        self.assertFalse(audio.source.started)
        audio.close()
        self.assertTrue(audio.source.closed)
        self.assertFalse(audio.ready)
        with self.assertRaises(RuntimeError):
            audio.start()
        with self.assertRaises(RuntimeError):
            audio.drain(1.)

    def test_audio_startup_timeout_latches_and_prevents_late_recovery(self):
        audio = self.capture()
        audio._started = 1.
        with self.assertRaisesRegex(RuntimeError, "STARTUP_TIMEOUT"):
            audio.drain(6.01)
        audio._last = timed_audio(6.01, 6.01)
        self.assertFalse(audio.ready)
        with self.assertRaisesRegex(RuntimeError, "AUDIO_FAILED"):
            audio.drain(6.02)

    def test_audio_future_publication_is_held_until_next_tick(self):
        audio = self.capture()
        audio._started = 0.
        first, future = timed_audio(), timed_audio(1.02, 1.021, 2)
        audio._queue.extend((first, future))
        audio._last = future
        self.assertEqual(audio.drain(1.01), [first])
        self.assertEqual(audio.drain(1.02), [])
        self.assertEqual(audio.drain(1.03), [future])

    def test_audio_stale_and_backlog_cannot_be_replayed_as_fresh(self):
        for now, expected in ((1.3, "CONTROLLER_LATENCY"), (2.1, "STALE")):
            audio = self.capture()
            audio._started = 0.
            audio._last = timed_audio()
            audio._queue.append(audio._last)
            with self.assertRaisesRegex(RuntimeError, expected):
                audio.drain(now)
            with self.assertRaisesRegex(RuntimeError, "AUDIO_FAILED"):
                audio.drain(now + .01)

    def test_audio_worker_device_and_log_failures_close_source(self):
        def broken_emit(*_args, **_kwargs):
            raise OSError("journal write failed")
        for emit in (lambda *_a, **_kw: None, broken_emit):
            audio = AudioCapture(emit, self.root, source_factory=FakeSource, clock=lambda: 1.)
            audio.start()
            audio._thread.join(.5)
            self.assertFalse(audio._thread.is_alive())
            self.assertTrue(audio.source.closed)
            with self.assertRaisesRegex(RuntimeError, "AUDIO_FAILED"):
                audio.drain(1.1)
            with self.assertRaises(RuntimeError):
                audio.close()

    def player(self, emit=None):
        path = self.root / "welcome.wav"
        write_wav(path)
        proc = FakeProcess()
        player = WavePlayer({"welcome": path}, emit or (lambda *_a, **_kw: None), self.root,
                            process_factory=lambda *_a, **_kw: proc)
        self.addCleanup(player.close)
        return player, proc

    def test_playback_completion_is_successful_exit_and_clip_is_once_only(self):
        events = []
        player, proc = self.player(lambda event, **fields: events.append(event))
        player.play("welcome", 1.)
        self.assertTrue(player.poll(1.02))
        proc.code = 0
        self.assertFalse(player.poll(1.05))
        self.assertEqual(events, ["PLAYBACK_STARTED", "PLAYBACK_FINISHED"])
        with self.assertRaisesRegex(RuntimeError, "repeated"):
            player.play("welcome", 2.)

    def test_changed_clip_cannot_silently_bypass_recorded_hash(self):
        player, proc = self.player()
        write_wav(self.root / "welcome.wav", value=42)
        with self.assertRaisesRegex(RuntimeError, "FIXED_CLIP_CHANGED"):
            player.play("welcome", 1.)
        self.assertIsNone(player._proc)

    def test_started_log_failure_terminates_spawned_playback(self):
        def fail(*_args, **_kwargs):
            raise OSError("no journal")
        player, proc = self.player(fail)
        with self.assertRaises(OSError):
            player.play("welcome", 1.)
        self.assertTrue(proc.terminated)
        self.assertIsNone(player._log)
        with self.assertRaisesRegex(RuntimeError, "PLAYBACK_FAULT"):
            player.poll(2.)

    def test_playback_nonzero_exit_latches(self):
        player, proc = self.player()
        player.play("welcome", 1.)
        proc.code = 1
        with self.assertRaisesRegex(RuntimeError, "PLAYBACK_FAILED"):
            player.poll(1.1)
        with self.assertRaisesRegex(RuntimeError, "PLAYBACK_FAULT"):
            player.poll(1.2)

    def test_playback_timeout_terminates_and_stop_is_final(self):
        player, proc = self.player()
        player.play("welcome", 1.)
        with self.assertRaisesRegex(RuntimeError, "PLAYBACK_TIMEOUT"):
            player.poll(8.)
        self.assertTrue(proc.terminated)
        with self.assertRaises(RuntimeError):
            player.play("welcome", 9.)


class PrepareAudioTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.assets = self.root / "assets"
        self.assets.mkdir()
        self.interpreter = self.root / "venv_python"
        self.interpreter.symlink_to(sys.executable)
        self.model = self.root / "voice.onnx"
        self.model.write_bytes(b"dummy model")
        Path(str(self.model)+".json").write_text("{}")
        for name in ("welcome", "prompt"):
            (self.assets / f"{name}.txt").write_text("你好，这是固定文本。", encoding="utf-8")
        self.argv = ["prepare_audio.py", "--piper-python", str(self.interpreter),
                     "--model", str(self.model), "--assets", str(self.assets)]

    @staticmethod
    def synthesize(argv, **kwargs):
        request = json.loads(kwargs["input"])
        for item in request["items"]:
            write_wav(Path(item["output"]))
        return subprocess.CompletedProcess(argv, 0, "", "")

    def test_venv_symlink_is_preserved_and_pair_manifest_match(self):
        with patch.object(sys, "argv", self.argv), patch("prepare_audio.subprocess.run", side_effect=self.synthesize) as run:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(prepare_audio.main(), 0)
        self.assertEqual(run.call_args.args[0][0], str(self.interpreter))
        self.assertNotEqual(run.call_args.args[0][0], str(self.interpreter.resolve()))
        self.assertEqual(run.call_args.kwargs["encoding"], "utf-8")
        manifest = json.loads((self.assets / "audio_manifest.json").read_text())
        for name in ("welcome", "prompt"):
            self.assertEqual(manifest["clips"][name], wav_info(self.assets / f"{name}.wav"))

    def test_invalid_second_clip_preserves_existing_pair(self):
        for name in ("welcome", "prompt"):
            write_wav(self.assets / f"{name}.wav", value=10)
        originals = {name: (self.assets / f"{name}.wav").read_bytes() for name in ("welcome", "prompt")}
        def bad_second(argv, **kwargs):
            request = json.loads(kwargs["input"])
            write_wav(Path(request["items"][0]["output"]))
            Path(request["items"][1]["output"]).write_bytes(b"invalid WAV")
            return subprocess.CompletedProcess(argv, 0, "", "")
        with patch.object(sys, "argv", self.argv + ["--overwrite"]), patch("prepare_audio.subprocess.run", side_effect=bad_second):
            with self.assertRaises(ValueError):
                prepare_audio.main()
        for name, data in originals.items():
            self.assertEqual((self.assets / f"{name}.wav").read_bytes(), data)

    def test_publication_failure_restores_first_file(self):
        staged = self.root / "staged"
        staged.mkdir()
        for name in ("welcome.wav", "prompt.wav", "audio_manifest.json"):
            (staged / name).write_bytes(b"new")
            (self.assets / name).write_bytes(b"old")
        replace = prepare_audio.os.replace
        def fail_second(source, target):
            if Path(source) == staged / "prompt.wav":
                raise OSError("injected second rename failure")
            return replace(source, target)
        with patch("prepare_audio.os.replace", side_effect=fail_second):
            with self.assertRaises(OSError):
                prepare_audio._publish_assets(staged, self.assets,
                    ("welcome.wav", "prompt.wav", "audio_manifest.json"))
        for name in ("welcome.wav", "prompt.wav", "audio_manifest.json"):
            self.assertEqual((self.assets / name).read_bytes(), b"old")


if __name__ == "__main__":
    unittest.main()
