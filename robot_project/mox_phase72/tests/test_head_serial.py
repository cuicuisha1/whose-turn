import math
import threading
import time
import unittest

from mox72.head_serial import HeadFault, SerialHead, SimHead, parse_status
from mox72.models import Pose


FOOTER = "Position feedback: NONE. ready is software readiness, not a mechanical safety check."


def status_lines(pan=0, tilt=0, speed=60, *, target_pan=None, target_tilt=None, done=True):
    target_pan = pan if target_pan is None else target_pan
    target_tilt = tilt if target_tilt is None else target_tilt
    return [
        f"STATUS version=2.0.0 ready=1 nominal_speed_deg_s={speed:.2f} ramp_done={int(done)} action=NONE config_ok=1 pca_initialized=1 output_fault=0",
        f"PAN command_deg={pan:.2f} target_deg={target_pan:.2f} last_requested_us=1500 limits=-120.00..120.00 center_us=1500.00 gain=1.058824 direction=1",
        f"TILT command_deg={tilt:.2f} target_deg={target_tilt:.2f} last_requested_us=1500 limits=-60.00..60.00 center_us=1500.00 gain=1.058824 direction=1",
        FOOTER,
    ]


class FakeSerial:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.output = bytearray()
        self.commands = []
        self.lock = threading.Lock()
        self.pan = self.tilt = 0.0
        self.speed = 60.0
        self.is_open = True
        self.silent = None
        self.bad_ack = None
        self.reset_on = None
        self.short_write = None
        self.speed_seen = threading.Event()
        self.status_transform = lambda lines: lines

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.output)

    def reset_input_buffer(self):
        with self.lock:
            self.output.clear()

    def read(self, n):
        with self.lock:
            data = bytes(self.output[:n])
            del self.output[:n]
            return data

    def write(self, payload):
        if not self.is_open:
            raise OSError("closed")
        command = payload.decode("ascii").strip()
        self.commands.append(command)
        if command.startswith("SPEED"):
            self.speed_seen.set()
        if command == self.short_write:
            return len(payload)-1
        if command == self.silent:
            return len(payload)
        if command == self.reset_on:
            lines = ["Robot Head Controller Ready v2.0.0"]
        elif command == "STATUS":
            lines = self.status_transform(status_lines(self.pan, self.tilt, self.speed))
        elif command == "STOP":
            lines = ["Action stopped; command ramp held, PWM remains active"]
        elif command.startswith("SPEED "):
            self.speed = float(command.split()[1])
            lines = ["Speed: " + ("99.00" if command == self.bad_ack else f"{self.speed:.2f}")]
        elif command.startswith("P "):
            self.pan = float(command.split()[1])
            lines = ["Pan target: " + ("99.00" if command == self.bad_ack else f"{self.pan:.2f}")]
        elif command.startswith("T "):
            self.tilt = float(command.split()[1])
            lines = [f"Tilt target: {self.tilt:.2f}"]
        else:
            lines = ["ERROR COMMAND"]
        with self.lock:
            self.output.extend(("\n".join(lines) + "\n").encode("ascii"))
        return len(payload)

    def close(self):
        self.is_open = False


class StatusTests(unittest.TestCase):
    def test_complete_status_reports_command_coordinates(self):
        s = parse_status(status_lines(45, -3, 40), 5)
        self.assertEqual(s.pose, Pose(45, -3))
        self.assertEqual(s.target, Pose(45, -3))
        self.assertTrue(s.ramp_done)
        self.assertFalse(s.simulated)

    def test_missing_footer_or_axis_rejected(self):
        for lines in (status_lines()[:3], status_lines()[:-1]+["bad footer"]):
            with self.assertRaises(HeadFault):
                parse_status(lines, 1)

    def test_bad_health_version_numbers_and_unexpected_action_rejected(self):
        replacements = [
            ("version=2.0.0", "version=1.0.0"), ("ready=1", "ready=0"),
            ("config_ok=1", "config_ok=0"), ("pca_initialized=1", "pca_initialized=0"),
            ("output_fault=0", "output_fault=1"), ("action=NONE", "action=LOOK"),
            ("nominal_speed_deg_s=60.00", "nominal_speed_deg_s=nan"),
            ("command_deg=0.00", "command_deg=inf"),
            ("limits=-120.00..120.00", "limits=-15.00..15.00"),
            ("gain=1.058824", "gain=0"), ("direction=1", "direction=0"),
        ]
        for old,new in replacements:
            with self.subTest(old=old):
                lines=[line.replace(old,new) for line in status_lines()]
                with self.assertRaises(HeadFault):
                    parse_status(lines, 1)

    def test_done_cannot_disagree_with_target(self):
        with self.assertRaises(HeadFault):
            parse_status(status_lines(pan=0,target_pan=45,done=True), 1)

    def test_boot_pose_outside_host_envelope_rejected(self):
        with self.assertRaises(HeadFault):
            parse_status(status_lines(pan=90), 1)


class SerialTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeSerial()
        self.events = []
        self.driver = SerialHead("/dev/test", lambda name,**kw:self.events.append((name,kw)),
                                 serial_factory=lambda **kw:self.fake, startup_wait_s=0,
                                 ack_timeout_s=.09, status_timeout_s=.5, poll_s=.03)
        self.expected_close_fault = False

    def tearDown(self):
        if self.expected_close_fault and self.driver.ser is not None:
            with self.assertRaises(HeadFault):
                self.driver.close()
        else:
            self.driver.close()

    def await_condition(self, condition, timeout=.8):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if condition():
                return
            time.sleep(.002)
        self.fail("condition not reached")

    def await_fault(self):
        def failed():
            try:
                self.driver.sample(time.monotonic())
            except HeadFault:
                return True
            return False
        self.await_condition(failed)
        self.expected_close_fault = True

    def test_open_is_read_only_and_move_uses_numeric_acks(self):
        self.driver.open()
        self.assertTrue(all(c == "STATUS" for c in self.fake.commands))
        self.driver.move(Pose(45,-3),20,"prepare_a")
        self.assertTrue(self.driver.sample(time.monotonic()).busy)
        self.await_condition(lambda: not self.driver.sample(time.monotonic()).busy)
        s=self.driver.sample(time.monotonic())
        self.assertEqual(s.pose, Pose(45,-3))
        self.assertEqual(s.speed,20)
        writes=[c for c in self.fake.commands if c!='STATUS']
        self.assertEqual(writes,['SPEED 20.00','P 45.00','T -3.00'])
        self.assertIsNotNone(s.settled_since_s)
        self.assertTrue(any(n=='HEAD_COMMAND_SENT' and d['command']=='P 45.00' for n,d in self.events))

    def test_bad_numeric_ack_stops_without_later_axis(self):
        self.driver.open()
        self.fake.bad_ack='P 45.00'
        self.driver.move(Pose(45,-3),20,"bad_ack")
        self.await_fault()
        self.await_condition(lambda:'STOP' in self.fake.commands)
        self.assertNotIn('T -3.00',self.fake.commands)

    def test_timeout_no_retry_and_no_next_axis(self):
        self.driver.open()
        self.fake.silent='SPEED 20.00'
        self.driver.move(Pose(45,0),20,"timeout")
        self.await_fault()
        self.assertEqual(self.fake.commands.count('SPEED 20.00'),1)
        self.assertNotIn('P 45.00',self.fake.commands)

    def test_reset_banner_any_suffix_is_fault(self):
        self.driver.open()
        self.fake.reset_on='P 45.00'
        self.driver.move(Pose(45,0),20,"reset")
        self.await_fault()
        self.assertNotIn('T 0.00',self.fake.commands)

    def test_stop_preempts_inflight_transaction_and_is_terminal(self):
        self.driver.open()
        self.fake.silent='SPEED 20.00'
        self.driver.move(Pose(45,0),20,"cancel")
        self.assertTrue(self.fake.speed_seen.wait(.5))
        self.driver.stop("operator_stop")
        self.await_condition(lambda:'STOP' in self.fake.commands)
        self.assertNotIn('P 45.00',self.fake.commands)
        with self.assertRaises(HeadFault):
            self.driver.move(Pose(0,0),20,"must_not_resume")

    def test_busy_and_bad_goals_are_rejected(self):
        self.driver.open()
        for pose,speed in ((Pose(61,0),20),(Pose(0,11),20),(Pose(math.nan,0),20),(Pose(0,0),0)):
            with self.assertRaises(HeadFault):
                self.driver.move(pose,speed,"bad")
        self.fake.silent='SPEED 20.00'
        self.driver.move(Pose(45,0),20,"busy")
        with self.assertRaises(HeadFault):
            self.driver.move(Pose(-45,0),20,"double_queue")

    def test_short_write_latches_fault(self):
        self.driver.open()
        self.fake.short_write='P 45.00'
        self.driver.move(Pose(45,0),20,"short")
        self.await_fault()
        self.assertNotIn('T 0.00',self.fake.commands)

    def test_status_change_by_other_writer_faults(self):
        self.driver.open()
        self.driver.move(Pose(45,0),20,"move")
        self.await_condition(lambda:not self.driver.sample(time.monotonic()).busy)
        self.fake.pan=46
        self.await_fault()

    def test_missing_status_response_faults(self):
        self.driver.open()
        self.fake.silent='STATUS'
        self.await_fault()

    def test_long_unterminated_response_faults(self):
        self.driver.open()
        self.fake.status_transform=lambda lines:['x'*9000]
        self.await_fault()

    def test_logging_failure_still_attempts_stop(self):
        self.driver.open()
        failed=[False]
        def fail_log(name,**fields):
            if name=='HEAD_COMMAND_ATTEMPT' and fields.get('command')=='P 45.00':
                failed[0]=True
            if failed[0]:
                raise OSError('log storage failed')
        self.driver.emit=fail_log
        self.driver.move(Pose(45,0),20,'log_fail')
        self.await_fault()
        self.await_condition(lambda:'STOP' in self.fake.commands)
        self.assertNotIn('P 45.00',self.fake.commands)

    def test_stuck_command_ramp_faults(self):
        self.driver.move_timeout=.04
        self.driver.open()
        def freeze(lines):
            if self.fake.pan==45:
                return status_lines(0,0,self.fake.speed,target_pan=45,done=False)
            return lines
        self.fake.status_transform=freeze
        self.driver.move(Pose(45,0),20,'stuck')
        self.await_fault()

    def test_newer_worker_snapshot_is_not_false_clock_fault(self):
        self.driver.open()
        with self.driver._lock:
            stamp=self.driver._snapshot.time_s
        self.assertEqual(self.driver.sample(stamp-.2).pose,Pose(0,0))

    def test_close_reports_acknowledged_stop_and_is_idempotent(self):
        self.driver.open()
        self.driver.close()
        self.assertTrue(self.driver.stop_confirmed)
        self.assertIsNone(self.driver.stop_error)
        self.assertEqual(self.fake.commands.count('STOP'), 1)
        self.assertFalse(self.fake.is_open)
        self.driver.close()
        self.assertEqual(self.fake.commands.count('STOP'), 1)

    def test_close_raises_when_stop_ack_is_missing(self):
        self.driver.open()
        self.fake.silent='STOP'
        with self.assertRaisesRegex(HeadFault, 'STOP acknowledgement failed'):
            self.driver.close()
        self.assertFalse(self.driver.stop_confirmed)
        self.assertIn('timeout', self.driver.stop_error)
        self.assertTrue(self.driver._stop_done.is_set())
        self.assertFalse(self.fake.is_open)
        self.assertFalse(self.driver._thread.is_alive())
        self.assertEqual(self.fake.commands.count('STOP'), 1)
        self.assertTrue(any(name=='HEAD_STOP_UNCONFIRMED' for name,_ in self.events))

    def test_close_retains_original_fault_and_separate_stop_failure(self):
        self.driver.open()
        self.fake.bad_ack='P 45.00'
        self.fake.silent='STOP'
        self.driver.move(Pose(45,0),20,'bad_ack_then_stop_failure')
        self.await_fault()
        with self.assertRaises(HeadFault) as caught:
            self.driver.close()
        self.assertIn('Numeric ACK mismatch', str(caught.exception))
        self.assertIn('STOP acknowledgement failed', str(caught.exception))
        self.assertFalse(self.driver.stop_confirmed)
        self.assertFalse(self.fake.is_open)

    def test_async_fault_before_cleanup_cannot_be_hidden_by_successful_stop(self):
        self.driver.open()
        self.fake.silent='STATUS'
        self.await_fault()
        with self.assertRaisesRegex(HeadFault, 'serial worker fault'):
            self.driver.close()
        self.assertTrue(self.driver.stop_confirmed)
        self.assertIsNone(self.driver.stop_error)

    def test_failed_read_only_handshake_does_not_claim_missing_session_stop(self):
        self.fake.silent='STATUS'
        with self.assertRaises(HeadFault):
            self.driver.open()
        self.driver.close()
        self.assertNotIn('STOP', self.fake.commands)
        self.assertFalse(self.driver.stop_confirmed)
        self.assertIsNone(self.driver.stop_error)


class SimTests(unittest.TestCase):
    def test_motion_and_stop_use_trial_clock(self):
        now=[10.0]
        h=SimHead(clock=lambda:now[0])
        h.move(Pose(40,10),20,"move")
        now[0]=11.0
        self.assertEqual(h.sample(now[0]).pose,Pose(20,10))
        now[0]=12.0
        s=h.sample(now[0])
        self.assertEqual(s.pose,Pose(40,10))
        self.assertTrue(s.ramp_done)
        self.assertAlmostEqual(s.settled_since_s,12.0)
        h.stop()
        with self.assertRaises(HeadFault):
            h.move(Pose(0,0),20,"after_stop")

    def test_new_instance_does_not_reuse_prior_pose(self):
        now=[0.0]
        h=SimHead(clock=lambda:now[0])
        h.move(Pose(45,0),20,"A")
        now[0]=3
        self.assertEqual(h.sample(3).pose,Pose(45,0))
        another=SimHead(clock=lambda:now[0])
        self.assertEqual(another.sample(3).pose,Pose(0,0))


if __name__ == '__main__':
    unittest.main()
