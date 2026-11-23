import json
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "host"))
from robot_test import Client, CommandRejected, LineDecoder, ProtocolError, SimulationTransport, HELP

class MemoryTransport:
    def __init__(self, reply=True):
        self.data, self.writes, self.reply = [], [], reply
    def write(self, data):
        self.writes.append(data)
        packet = json.loads(data)
        if self.reply:
            self.data.append(json.dumps({"type": "ack", "sequence": packet.get("sequence", 0),
                                        "ok": True, "state": "ARMED", "arm_epoch": 3}).encode()+b"\n")
    def read(self):
        return self.data.pop(0) if self.data else b""

class HostUnitTests(unittest.TestCase):
    def test_fragmented_and_batched_lines(self):
        d = LineDecoder()
        self.assertEqual(d.feed(b'{"type":"sta'), [])
        records = d.feed(b'tus","state":"FAULT"}\n{"type":"ack","ok":true}\n')
        self.assertEqual([r["type"] for r in records], ["status", "ack"])

    def test_decoder_rejects_corrupt_or_unbounded_data(self):
        for raw in (b'no json\n', b'[]\n', b'{"type":"other"}\n', b'{"type":"ack","v":NaN}\n', b'x'*2049):
            with self.subTest(raw=raw[:30]), self.assertRaises(ProtocolError):
                LineDecoder().feed(raw)

    def test_unique_session_and_sequence(self):
        a, b = Client(MemoryTransport()), Client(MemoryTransport())
        self.assertNotEqual(a.session, b.session)
        a.hello()
        a.request("heartbeat")
        packets = [json.loads(w) for w in a.transport.writes]
        self.assertEqual([p["sequence"] for p in packets], [1, 2])
        self.assertTrue(all(p["session"] == a.session for p in packets))

    def test_timeout_never_replays_servo(self):
        t = MemoryTransport(reply=False)
        c = Client(t, timeout=.01)
        with self.assertRaises(TimeoutError):
            c.request("servo", pan_us=1500, tilt_us=1500, arm_epoch=3)
        self.assertEqual(len(t.writes), 1)

    def test_status_and_stop_do_not_need_a_session(self):
        t = MemoryTransport()
        c = Client(t)
        c.request("status")
        c.request("stop")
        self.assertTrue(all("session" not in json.loads(w) for w in t.writes))

    def test_console_and_client_have_no_drivetrain_actions(self):
        self.assertFalse(hasattr(Client, "motion"))
        self.assertFalse(hasattr(Client, "finite_move"))
        for word in ("move ", "zero", "drive", "enc |"):
            self.assertNotIn(word, HELP)

class EndToEndTests(unittest.TestCase):
    def setUp(self):
        exe = ROOT / "build/tests" / ("simulator.exe" if sys.platform == "win32" else "simulator")
        self.transport = SimulationTransport(exe)
        self.c = Client(self.transport, timeout=.15)
        time.sleep(.15)
        self.c.hello()
    def tearDown(self):
        self.c.best_effort_stop()
        self.transport.close()
    def arm(self, mode="head"):
        self.c.request("clear_fault")
        self.c.request("prepare", mode=mode)
        self.assertEqual(self.c.request("status")["state"], "ARMED")

    def test_head_commands_and_stop(self):
        self.arm()
        s = self.c.request("status")
        self.assertEqual(s["protocol_version"], "head-1")
        self.assertTrue(s["pwm_output_disabled"])
        self.assertNotIn("targets_counts", s)
        self.c.request("servo", pan_us=1500, tilt_us=1550, arm_epoch=self.c.epoch)
        self.assertTrue(self.c.request("status")["servo_active"])
        self.c.request("stop")
        s = self.c.request("status")
        self.assertEqual(s["state"], "DISARMED")
        self.assertTrue(s["pwm_output_disabled"])

    def test_link_timeout_disables_pwm(self):
        self.arm()
        self.c.request("servo", pan_us=1500, tilt_us=1500, arm_epoch=self.c.epoch)
        time.sleep(.25)
        s = self.c.request("status")
        self.assertEqual(s["state"], "FAULT")
        self.assertTrue(s["faults"] & 4)
        self.assertTrue(s["pwm_output_disabled"])

    def test_servo_failure_latches_after_recovery(self):
        self.arm()
        self.c.simulate("sim_servo_fail")
        with self.assertRaises(CommandRejected):
            self.c.request("servo", pan_us=1500, tilt_us=1500, arm_epoch=self.c.epoch)
        self.c.simulate("sim_servo_restore")
        s = self.c.request("status")
        self.assertEqual(s["state"], "FAULT")
        self.assertTrue(s["faults"] & 256)
        self.assertTrue(s["pwm_output_disabled"])

    def test_removed_commands_fault_active_controller(self):
        legacy = [
            {"type": "motion", "wheels_rpm": [0, 0, 0, 0], "arm_epoch": 1},
            {"type": "prepare", "mode": "drive"},
            {"type": "encoder_raw"},
        ]
        for packet in legacy:
            with self.subTest(packet=packet):
                self.arm()
                packet.update(session=self.c.session, sequence=self.c.sequence+1)
                self.transport.write(json.dumps(packet).encode()+b"\n")
                time.sleep(.02)
                s = self.c.request("status")
                self.assertEqual(s["state"], "FAULT")
                self.assertTrue(s["faults"] & 32)
                self.assertTrue(s["pwm_output_disabled"])

    def test_malformed_json_trips_active_controller(self):
        self.arm("bench")
        self.transport.write(b'{"type":"status","type":"stop"}\n')
        time.sleep(.02)
        self.assertTrue(self.c.request("status")["faults"] & 32)

if __name__ == "__main__":
    unittest.main()
