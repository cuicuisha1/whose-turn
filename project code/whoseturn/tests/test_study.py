import json
from pathlib import Path
import tempfile
import unittest

from whoseturn.core import Config, Study, load_config
from whoseturn.devices import Head, Speech


class FakeSpeech:
    def __init__(self):
        self.busy, self.prompt = False, None
    def play(self, prompt, now):
        self.prompt = prompt
    def step(self, now):
        pass
    def stop(self):
        self.busy = False


class StudyTests(unittest.TestCase):
    def setUp(self):
        self.events, self.now = [], 0.0
        self.cfg = Config()
        self.head = Head(self.cfg, self.events.append, 0)
        self.s = Study(self.cfg, self.head, FakeSpeech(), self.events.append, 0)

    def run_until(self, state):
        for _ in range(1000):
            if self.s.state == state:
                return
            self.now += .02
            self.s.step(self.now)
        self.fail("did not reach " + state)

    def test_ack_does_not_allocate_response_to_b(self):
        self.s.command("start", self.now)
        self.run_until("WAIT_A")
        self.s.command("join", self.now)
        self.assertEqual(self.s.head_target, "B")
        self.assertEqual(self.s.addressee, "A")
        self.assertEqual(self.s.intent, "acknowledge_presence")
        self.run_until("WAIT_A")
        self.s.command("advance", self.now)
        self.run_until("WAIT_B")
        self.assertEqual(self.s.addressee, "B")

    def test_mark_does_not_advance_or_assign_interpretation(self):
        self.s.command("mark", self.now, "B speaks")
        self.assertEqual(self.s.state, "READY")
        self.assertIsNone(self.events[-1]["participant_interpretation"])

    def test_same_outbound_primitive_and_pulse_profile(self):
        self.s.command("start", self.now)
        self.run_until("WAIT_A")
        start = len(self.events)
        self.s.command("join", self.now)
        self.run_until("ACK_HOLD")
        ack = [e["pan_us"] for e in self.events[start:] if e["event"] == "servo_command"]
        self.run_until("WAIT_A")
        start = len(self.events)
        self.s.command("advance", self.now)
        self.run_until("WAIT_B")
        handover = [e["pan_us"] for e in self.events[start:] if e["event"] == "servo_command"]
        self.assertEqual(ack, handover)

    def test_stop_is_terminal_and_prevents_new_commands(self):
        self.s.command("stop", 0)
        with self.assertRaises(ValueError):
            self.s.command("start", .1)
        self.assertTrue(self.head.stopped)

    def test_gap_stops_without_catching_up(self):
        self.s.command("start", 0)
        with self.assertRaises(RuntimeError):
            self.s.step(.3)
        self.assertEqual(self.s.state, "STOPPED")

    def test_invalid_order_rejected(self):
        for cmd in ("join", "advance", "repair", "finish"):
            with self.subTest(cmd=cmd), self.assertRaises(ValueError):
                self.s.command(cmd, 0)

    def test_join_only_once(self):
        self.s.command("start", 0)
        self.run_until("WAIT_A")
        self.s.command("join", self.now)
        self.run_until("WAIT_A")
        with self.assertRaises(ValueError):
            self.s.command("join", self.now)

    def test_repair_preserves_negotiation(self):
        self.s.command("start", 0)
        self.run_until("WAIT_A")
        self.s.command("repair", self.now)
        self.run_until("WAIT_A")
        self.assertIsNone(self.s.addressee)
        self.assertEqual(self.s.intent, "open_negotiation")

    def test_slew_and_range(self):
        self.head.target(self.cfg.b_us, 0)
        last = self.head.pan
        for tick in range(1, 31):
            self.head.step(tick * .02)
            self.assertLessEqual(abs(self.head.pan - last), 1.600001)
            self.assertLessEqual(self.head.pan, self.cfg.b_us)
            last = self.head.pan
        with self.assertRaises(ValueError):
            self.head.target(1601, .7)

    def test_hardware_requires_record_and_rejects_boolean_numbers(self):
        with self.assertRaises(ValueError):
            Config().validate(hardware=True)
        for kwargs in ({"slew_us_per_s": True}, {"a_us": True}, {"ack_hold_s": float("nan")},
                       {"calibrated": "yes"}, {"pan_max_us": 2500}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Config(**kwargs).validate()

    def test_duplicate_config_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder) / "config.json"
            p.write_text('{"a_us": 1450, "a_us": 1550}')
            with self.assertRaises(ValueError):
                load_config(p)

    def test_missing_wav_is_not_silently_replaced_by_text(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(FileNotFoundError):
                Speech({"a": "hello"}, self.events.append, "wav", folder)

    def test_explicit_variant_and_clean_completion(self):
        self.s.variant = "explicit"
        self.s.command("start", 0)
        self.run_until("WAIT_A")
        self.s.command("join", self.now)
        self.assertEqual(self.s.state, "ACK_SPEECH")
        self.run_until("WAIT_A")
        self.s.command("advance", self.now)
        self.run_until("WAIT_B")
        self.s.command("finish", self.now, "skip")
        self.run_until("DONE")
        self.assertTrue(self.head.stopped)


if __name__ == "__main__":
    unittest.main()
