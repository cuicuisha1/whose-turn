"""Pure simulated observations and drivers; opens no camera or serial port."""
from dataclasses import replace
import math
import unittest

from mox72.config import Config
from mox72.engine import Engine
from mox72.models import Pose, FaceObservation, HeadSnapshot
from mox72.vendor.live_audio import TimedAudio
from mox72.vendor.policies import AudioFrame, AudioStatus


class FakeHead:
    def __init__(self):
        self.now = 10.0
        self.pose = self.target = Pose(0.0, 0.0)
        self.speed = 40.0
        self.settled = self.now
        self.moves = []
        self.stops = []
        self.busy = False

    def move(self, pose, speed, reason):
        self.moves.append((self.now, pose, speed, reason))
        self.pose = self.target = pose
        self.speed = speed
        self.settled = self.now

    def sample(self, now):
        return HeadSnapshot(now, self.pose, self.target, self.speed,
                            not self.busy, self.busy, None if self.busy else self.settled,
                            simulated=True)

    def stop(self, reason):
        self.stops.append(reason)
        self.busy = False


class FakePlayer:
    def __init__(self, welcome=0.5, prompt=0.2):
        self.durations = {"welcome": welcome, "prompt": prompt}
        self.until = 0.0
        self.plays = []
        self.stopped = False

    def play(self, kind, now):
        if now < self.until:
            raise RuntimeError("Overlapping playback")
        self.plays.append((kind, now))
        self.until = now + self.durations[kind]

    def poll(self, now):
        return now < self.until

    def stop(self):
        self.stopped = True
        self.until = 0


class Harness:
    def __init__(self, *, condition="pause", a_only=False, cfg=None, welcome=0.5,
                 prompt=0.2):
        self.t = 10.0
        self.seq = 0
        self.audio_seq = 0
        self.head = FakeHead()
        self.player = FakePlayer(welcome=welcome, prompt=prompt)
        self.events = []
        self.cfg = cfg or Config(tilt_sign=1)
        self.engine = Engine(self.cfg, condition, self.head, self.player,
                             lambda name, **kw: self.events.append((name, kw)), a_only=a_only)
        self.engine.begin(self.t)

    def tick(self, *, dt=0.02, face=True, point=(0.0, 0.0, 1.0), track=1,
             count=1, horizontal=0.0, vertical=0.0, audio=True,
             audio_status=AudioStatus.UNVOICED, old_face_time=None, face_error=None):
        previous = self.t
        self.t += dt
        self.head.now = self.t
        obs = None
        if face:
            self.seq += 1
            obs = FaceObservation(self.t if old_face_time is None else old_face_time,
                                  self.seq, track, count, point, horizontal, vertical, score=.9,
                                  error=face_error)
        items = []
        if audio:
            self.audio_seq += 1
            items = [TimedAudio(AudioFrame(previous, self.t, audio_status), self.t, self.audio_seq, -45.0)]
        self.engine.tick(self.t, obs, items)

    def run(self, duration, **kwargs):
        for _ in range(round(duration / 0.02)):
            self.tick(**kwargs)

    def until(self, state, limit=20.0, **kwargs):
        start = self.t
        while self.engine.state != state and not self.engine.terminal:
            if self.t - start > limit:
                raise AssertionError(f"Never reached {state}; got {self.engine.snapshot()}")
            self.tick(**kwargs)
        if self.engine.state != state:
            raise AssertionError(f"Expected {state}, got {self.engine.snapshot()}")

    def ready(self):
        self.until("WAIT_JOIN")
        while not self.engine.snapshot()["ready_for_join"]:
            self.tick()

    def command(self, value):
        return self.engine.command(value, self.t)

    def named(self, name):
        return [kw for event, kw in self.events if event == name]


class ConfigTests(unittest.TestCase):
    def test_live_requires_explicit_tilt_direction(self):
        with self.assertRaises(ValueError):
            Config().validate_live()
        Config(tilt_sign=-1).validate_live()

    def test_invalid_config_and_unknown_fields_rejected(self):
        for kwargs in ({"a_speed": 200}, {"pan_a_base": 119}, {"tilt_sign": True},
                       {"stable_duration_s": float("nan")}, {"station_min_z_m": 2.0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Config(**kwargs)
        with self.assertRaises(ValueError):
            Config.from_dict({"pan_limt": 10})
        self.assertEqual(Config().sha256, Config.from_dict(Config().to_dict()).sha256)


class EngineTests(unittest.TestCase):
    def test_normal_invalid_observations_wait_then_allow_a_entry(self):
        h = Harness(a_only=True)
        for reason in ("NO_FACE", "MULTIPLE_FACES", "INVALID_DEPTH", "INVALID_DEPROJECTION"):
            h.run(.6, point=None, count=0, track=None, face_error=reason)
        self.assertEqual(h.engine.state, "WAIT_A")
        self.assertFalse(h.player.plays)
        h.until("A_READY")
        self.assertTrue(h.engine.pose_a)

    def test_a_in_distance_gate_and_two_continuous_seconds(self):
        h = Harness(a_only=True)
        h.run(3.0, point=(0.0, 0.0, 3.0))
        self.assertEqual(h.engine.state, "WAIT_A")
        h.run(1.8)
        self.assertEqual(h.engine.state, "WAIT_A")
        h.until("A_READY")
        stable = h.named("A_STABLE")
        self.assertEqual(len(stable), 1)
        self.assertGreaterEqual(stable[0]["duration_s"] + 1e-8, 2.0)
        self.assertTrue(h.engine.pose_a)
        self.assertIsNone(h.engine.pose_b)
        self.assertEqual([kind for kind, _ in h.player.plays], ["welcome"])

    def test_welcome_and_stability_parallel_but_both_needed(self):
        h = Harness(a_only=True, welcome=5.0)
        h.run(3.0)
        self.assertEqual(h.engine.state, "WAIT_A")
        self.assertFalse(h.named("FACE_CENTER_STARTED"))
        h.until("A_READY")
        started = h.named("FACE_CENTER_STARTED")[0]["time_s"]
        ended = h.named("WELCOME_FINISHED")[0]["time_s"]
        self.assertGreaterEqual(started, ended)
        self.assertLess(started - ended, 0.05)  # Does not wait an unnecessary new two seconds.

    def test_loss_track_change_and_position_motion_reset_stability(self):
        for kind in ("gap", "track", "motion", "depth", "multiple"):
            with self.subTest(kind=kind):
                h = Harness(a_only=True)
                h.run(1.7)
                if kind == "gap":
                    h.run(.18, face=False)
                elif kind == "track":
                    h.tick(track=2)
                elif kind == "motion":
                    h.tick(point=(.12, 0.0, 1.0))
                elif kind == "depth":
                    h.tick(point=None)
                else:
                    h.tick(count=2)
                h.run(.5)
                self.assertEqual(h.engine.state, "WAIT_A")
                h.until("A_READY")
                self.assertTrue(h.named("A_STABILITY_RESET"))
                self.assertEqual([k for k, _ in h.player.plays].count("welcome"), 1)

    def test_moving_camera_never_accumulates_a_stability(self):
        h = Harness(a_only=True)
        h.until("WAIT_A")
        h.head.busy = True
        h.run(3.0)
        self.assertEqual(h.engine.state, "WAIT_A")
        self.assertFalse(h.named("A_STABLE"))
        h.head.busy = False
        h.head.settled = h.t
        h.run(1.9)
        self.assertEqual(h.engine.state, "WAIT_A")
        h.until("A_READY")

    def test_a_center_change_holds_without_inheriting_center_dwell(self):
        for kwargs in ({"track": 2},
                       {"count": 0, "point": None, "face_error": "NO_FACE"}):
            with self.subTest(kwargs=kwargs):
                h = Harness(a_only=True)
                h.until("CENTER_A")
                h.run(.10)
                h.tick(**kwargs)
                self.assertEqual(h.engine.state, "CENTER_A")
                self.assertTrue(h.engine.snapshot()["center_recovering"])
                self.assertIsNone(h.engine._center_since)
                self.assertIsNone(h.engine._filtered_error)
                self.assertIsNone(h.engine.pose_a)
                self.assertFalse(h.head.stops)

    def test_bounded_correction_allows_one_explicit_station_reacquisition(self):
        h = Harness(a_only=True)
        h.until("CENTER_A")
        h.tick(horizontal=6)
        self.assertEqual(h.head.moves[-1][3], "a_face_correction")
        h.run(.12, track=2)
        self.assertEqual(h.engine.state, "CENTER_A")
        h.until("A_READY", track=2)
        reacquired = h.named("FACE_REACQUIRED")
        self.assertEqual(len(reacquired), 1)
        self.assertEqual(reacquired[0]["previous_track_id"], 1)
        self.assertEqual(reacquired[0]["track_id"], 2)
        self.assertFalse(reacquired[0]["same_person_verified"])

    def test_stationary_track_change_after_bounded_reacquisition_requires_recovery(self):
        h = Harness(a_only=True)
        h.until("CENTER_A")
        h.tick(horizontal=6)
        h.run(.12, track=2)
        h.tick(track=2)
        self.assertEqual(len(h.named("FACE_REACQUIRED")), 1)
        h.tick(track=3)
        self.assertEqual(h.engine.state, "CENTER_A")
        self.assertEqual(h.engine.snapshot()["center_recovery_reason"], "track_changed")
        self.assertIsNone(h.engine.pose_a)
        h.until("A_READY", track=3)
        self.assertEqual(len(h.named("FACE_RECOVERY_COMPLETED")), 1)

    def test_b_initial_face_outside_tracking_depth_cannot_drive_or_lock(self):
        h = Harness()
        h.ready()
        h.command("JOIN")
        h.until("CENTER_B")
        moves = len(h.head.moves)
        h.run(.4, point=(0, 0, 4), horizontal=8)
        self.assertEqual(h.engine.state, "CENTER_B")
        self.assertIsNone(h.engine.pose_b)
        self.assertEqual(len(h.head.moves), moves)
        h.until("HOLD_B")

    def test_join_blocked_during_welcome_prompt_guard_and_stale_audio(self):
        # A long prompt exercises the remaining-playback gate after A locks.
        # A shorter concurrent prompt can correctly skip the PROMPT state.
        h = Harness(welcome=1.0, prompt=1.0)
        self.assertFalse(h.command("JOIN"))
        h.until("PROMPT")
        self.assertFalse(h.command("JOIN"))
        h.until("WAIT_JOIN")
        self.assertFalse(h.command("JOIN"))
        h.run(.5, audio=False)
        self.assertFalse(h.command("JOIN"))
        h.tick()
        self.assertTrue(h.command("JOIN"))
        self.assertFalse(h.command("JOIN"))
        self.assertEqual(len(h.named("JOIN_MARK_RECEIVED")), 1)

    def test_b_cannot_be_requested_before_join_or_trigger(self):
        h = Harness(condition="fixed")
        h.ready()
        h.run(1.0, horizontal=10.0)
        self.assertTrue(all(pose.pan > 0 for _, pose, _, _ in h.head.moves))
        h.command("JOIN")
        h.run(.8, horizontal=10.0)
        self.assertEqual(h.engine.state, "WAIT_TRIGGER")
        self.assertTrue(all(pose.pan > 0 for _, pose, _, _ in h.head.moves))
        h.until("TURN_B")
        self.assertEqual(h.head.moves[-1][1], Pose(-45, 0))
        self.assertEqual(h.engine.trigger_reason, "fixed_delay")

    def test_audio_watermark_not_wall_clock_and_no_empty_poll_silence(self):
        h = Harness(condition="pause")
        h.ready()
        h.command("JOIN")
        h.run(.5)
        self.assertEqual(h.engine.state, "WAIT_TRIGGER")
        h.run(.12, audio=False)
        self.assertEqual(h.engine.state, "WAIT_TRIGGER")
        # A later supplied frame still belongs after the last watermark; the
        # unchanged policy must not reject it as older than a wall-clock poll.
        h.tick()
        self.assertEqual(h.engine.state, "WAIT_TRIGGER")  # There is an audio gap, not 0.6s quiet.
        h.run(.6)
        self.assertEqual(h.engine.trigger_reason, "speech_pause")
        self.assertNotEqual(h.engine.state, "ABORTED")

    def test_delayed_frames_processed_in_sample_order(self):
        h = Harness(condition="pause")
        h.ready()
        h.command("JOIN")
        previous = h.t
        h.t += .12
        h.head.now = h.t
        h.seq += 1
        frames = []
        for i in range(6):
            h.audio_seq += 1
            frames.append(TimedAudio(AudioFrame(previous+i*.02, previous+(i+1)*.02,
                                                AudioStatus.UNVOICED), h.t, h.audio_seq, -50))
        face = FaceObservation(h.t, h.seq, 1, 1, (0, 0, 1), 0, 0)
        h.engine.tick(h.t, face, frames)
        self.assertEqual(h.engine.state, "WAIT_TRIGGER")
        h.run(.5)
        self.assertEqual(h.engine.trigger_reason, "speech_pause")
        self.assertNotEqual(h.engine.state, "ABORTED")

    def test_first_audio_frame_straddles_join_and_counts_only_post_join(self):
        h = Harness(condition="pause")
        h.ready()
        previous_audio_end = h.t
        # JOIN is received halfway through the next 20 ms audio frame.
        h.t += .01
        self.assertTrue(h.command("JOIN"))
        join = h.t
        h.t = previous_audio_end + .02
        h.head.now = h.t
        h.seq += 1
        h.audio_seq += 1
        face = FaceObservation(h.t, h.seq, 1, 1, (0, 0, 1), 0, 0)
        crossing = TimedAudio(AudioFrame(previous_audio_end, h.t, AudioStatus.UNVOICED),
                              h.t, h.audio_seq, -50)
        h.engine.tick(h.t, face, [crossing])
        self.assertEqual(h.engine.state, "WAIT_TRIGGER")
        h.run(.58)
        self.assertEqual(h.engine.state, "WAIT_TRIGGER")
        h.run(.04)
        self.assertEqual(h.engine.trigger_reason, "speech_pause")
        self.assertAlmostEqual(h.engine.trigger_due_at, join + .6)
        self.assertNotEqual(h.engine.state, "ABORTED")
        for observed in h.named("AUDIO_OBSERVED"):
            self.assertIn("audio_sequence", observed)
            self.assertNotIn("sequence", observed)

    def test_fixed_and_pause_abort_when_audio_becomes_stale(self):
        for condition in ("fixed", "pause"):
            with self.subTest(condition=condition):
                h = Harness(condition=condition)
                h.ready()
                h.command("JOIN")
                h.run(.30, audio=False)
                self.assertEqual(h.engine.state, "ABORTED")
                self.assertEqual(h.engine.fault, "audio_stale")
                self.assertTrue(all(p.pan > 0 for _, p, _, _ in h.head.moves))

    def test_old_face_after_movement_cannot_drive_new_correction(self):
        h = Harness()
        h.ready()
        h.command("JOIN")
        h.until("TURN_B")
        h.until("CENTER_B")
        previous_moves = len(h.head.moves)
        h.tick(horizontal=8, old_face_time=h.t-.10)
        # A timestamp backwards against the prior camera observation is a
        # fault, never a correction based on an old pose.
        self.assertEqual(len(h.head.moves), previous_moves)
        self.assertEqual(h.engine.state, "ABORTED")

    def test_b_fast_correction_then_hold_then_return_speed_restored(self):
        h = Harness()
        h.ready()
        locked_a = h.engine.pose_a
        h.command("JOIN")
        h.until("CENTER_B")
        h.run(.32, horizontal=6)
        self.assertEqual(h.head.moves[-1][3], "b_face_correction")
        self.assertEqual(h.head.moves[-1][2], 60)
        self.assertEqual(h.head.moves[-1][1].pan, -47)
        h.until("HOLD_B")
        self.assertEqual(h.engine.pose_b.pan, -47)
        hold_start = h.t
        h.run(.40)
        self.assertEqual(h.engine.state, "HOLD_B")
        h.until("RETURN_A")
        self.assertGreaterEqual(h.t-hold_start+1e-8, .5)
        self.assertEqual(h.head.moves[-1][1], locked_a)
        self.assertEqual(h.head.moves[-1][2], 40)
        h.until("OBSERVING")
        h.run(1.0)
        self.assertFalse(h.engine.terminal)
        h.command("END")
        self.assertEqual(h.engine.state, "COMPLETED")

    def test_b_failure_is_failed_attempt_and_does_not_reuse_angle(self):
        h = Harness(cfg=Config(tilt_sign=1, b_center_timeout_s=.5))
        h.ready()
        h.command("JOIN")
        h.until("CENTER_B")
        h.run(.6, count=0, point=None, face_error="NO_FACE")
        self.assertEqual(h.engine.state, "ABORTED")
        self.assertIsNone(h.engine.pose_b)
        self.assertEqual(h.engine.fault, "b_center_timeout")
        fresh = Harness()
        self.assertIsNone(fresh.engine.pose_a)
        self.assertIsNone(fresh.engine.pose_b)

    def test_no_face_during_b_turn_and_acquisition_is_not_immediate_fault(self):
        h = Harness()
        h.ready()
        h.command("JOIN")
        h.until("TURN_B")
        h.until("CENTER_B", count=0, point=None, face_error="NO_FACE")
        h.run(.4, count=0, point=None, face_error="NO_FACE")
        self.assertEqual(h.engine.state, "CENTER_B")
        self.assertFalse(h.engine.terminal)
        h.until("HOLD_B")
        self.assertTrue(h.engine.pose_b)

    def test_stop_and_early_end_abort_without_returning_or_centering(self):
        for command in ("STOP", "END"):
            h = Harness()
            h.ready()
            moves = len(h.head.moves)
            h.command(command)
            self.assertEqual(h.engine.state, "ABORTED")
            h.run(1.0)
            self.assertEqual(len(h.head.moves), moves)
            self.assertTrue(h.head.stops)

    def test_abort_stops_devices_even_when_logging_fails(self):
        h = Harness()
        def broken_emit(*args, **kwargs):
            raise OSError("disk full")
        h.engine.emit = broken_emit
        with self.assertRaises(OSError):
            h.command("STOP")
        self.assertTrue(h.head.stops)
        self.assertTrue(h.player.stopped)

    def test_external_failure_preserves_reason_and_terminal_result(self):
        h = Harness()
        self.assertTrue(h.engine.fail("camera_adapter_failed: frame timeout", h.t))
        self.assertEqual(h.engine.state, "ABORTED")
        self.assertEqual(h.engine.fault, "camera_adapter_failed: frame timeout")
        self.assertTrue(h.head.stops)
        self.assertTrue(h.player.stopped)
        ready = Harness(a_only=True)
        ready.until("A_READY")
        self.assertFalse(ready.engine.fail("late_cleanup_error", ready.t))
        self.assertEqual(ready.engine.state, "A_READY")


class CenterRegressionTests(unittest.TestCase):
    def centered_stage(self, role, *, cfg=None, prompt=0.2):
        h = Harness(a_only=(role == "A"),
                    cfg=cfg or Config(tilt_sign=-1), prompt=prompt)
        if role == "B":
            h.ready()
            self.assertTrue(h.command("JOIN"))
        h.until("CENTER_" + role)
        if role == "B":
            # v0.2.3 confirms B's first unique candidate before correction.
            h.run(.32)
        return h

    def test_logged_near_deadband_sample_does_not_claim_a_limit(self):
        for role in ("A", "B"):
            with self.subTest(role=role):
                h = self.centered_stage(role)
                h.tick(horizontal=1.33677960175309, vertical=-.8980520333979504)
                moves = len(h.head.moves)
                h.tick(horizontal=1.527181430823293, vertical=-.8837460137135601)
                self.assertEqual(h.engine.state, "CENTER_" + role)
                self.assertIsNone(h.engine.fault)
                self.assertEqual(len(h.head.moves), moves)
                self.assertFalse(h.named("FACE_CENTER_FAILED"))
                self.assertIsNone(h.engine._center_since)
                self.assertLess(h.engine._filtered_error[0], 1.5)
                h.until("A_READY" if role == "A" else "HOLD_B", horizontal=1.3)

    def test_persisting_raw_error_eventually_moves_after_ema_catches_up(self):
        h = self.centered_stage("A")
        h.tick(horizontal=1.3)
        moves = len(h.head.moves)
        h.tick(horizontal=1.53)
        self.assertEqual(len(h.head.moves), moves)
        for _ in range(10):
            h.tick(horizontal=1.53)
            if len(h.head.moves) > moves:
                break
        self.assertEqual(len(h.head.moves), moves + 1)
        self.assertLess(h.head.moves[-1][1].pan, 45)
        self.assertEqual(h.engine.state, "CENTER_A")

    def test_noop_wait_does_not_disable_centering_timeout(self):
        cfg = Config(tilt_sign=-1, face_ema_alpha=.000001, b_center_timeout_s=.5)
        h = self.centered_stage("B", cfg=cfg)
        h.tick(horizontal=0)
        moves = len(h.head.moves)
        h.run(.6, horizontal=1.51)
        self.assertEqual(h.engine.fault, "b_center_timeout")
        self.assertEqual(len(h.head.moves), moves)

    def test_actual_tilt_boundary_holds_with_diagnostics_then_exhausts_budget(self):
        for role in ("A", "B"):
            with self.subTest(role=role):
                h = self.centered_stage(role)
                base = 45 if role == "A" else -45
                h.head.pose = h.head.target = Pose(base, 25)
                h.engine._expected_pose = h.head.pose
                moves = len(h.head.moves)
                h.tick(horizontal=.93, vertical=-4.64)
                self.assertTrue(h.engine.snapshot()["center_recovering"])
                self.assertIsNone(h.engine.fault)
                self.assertEqual(len(h.head.moves), moves)
                event = h.named("FACE_CORRECTION_LIMIT")[-1]
                self.assertEqual(event["blocked_axes"], ["tilt"])
                self.assertEqual(event["current_pose"], {"pan": base, "tilt": 25})
                self.assertGreater(event["unclamped_target"]["tilt"], 25)
                self.assertEqual(event["clamped_target"]["tilt"], 25)
                self.assertEqual(event["vertical_error_deg"], -4.64)
                h.run(2.04, horizontal=.93, vertical=-4.64)
                self.assertEqual(h.engine.fault, role.lower()+"_center_recovery_budget_exhausted")
                self.assertEqual(len(h.head.moves), moves)

    def test_actual_pan_boundaries_hold_without_outward_commands(self):
        for role in ("A", "B"):
            for side in (-1, 1):
                with self.subTest(role=role, side=side):
                    h = self.centered_stage(role)
                    base = 45 if role == "A" else -45
                    h.head.pose = h.head.target = Pose(base + side * 10, 0)
                    h.engine._expected_pose = h.head.pose
                    moves = len(h.head.moves)
                    h.tick(horizontal=-side * 5)
                    self.assertTrue(h.engine.snapshot()["center_recovering"])
                    self.assertEqual(h.engine.snapshot()["center_recovery_reason"], "outside_role_angle_region")
                    self.assertEqual(len(h.head.moves), moves)
                    h.run(2.04, horizontal=-side*5)
                    self.assertEqual(h.engine.fault, role.lower()+"_center_recovery_budget_exhausted")

    def test_boundary_can_correct_inward_without_spurious_abort(self):
        h = self.centered_stage("A")
        h.head.pose = h.head.target = Pose(35, 25)
        h.engine._expected_pose = h.head.pose
        h.tick(horizontal=-5, vertical=5)
        self.assertEqual(h.engine.state, "CENTER_A")
        self.assertGreater(h.head.target.pan, 35)
        self.assertLess(h.head.target.tilt, 25)

    def test_mixed_axis_movement_stays_bounded_then_reports_blocked_axis(self):
        h = self.centered_stage("A")
        h.head.pose = h.head.target = Pose(45, 25)
        h.engine._expected_pose = h.head.pose
        h.tick(horizontal=5, vertical=-5)
        self.assertEqual(h.engine.state, "CENTER_A")
        self.assertLess(h.head.target.pan, 45)
        self.assertEqual(h.head.target.tilt, 25)
        h.run(.2, horizontal=0, vertical=-5)
        self.assertTrue(h.engine.snapshot()["center_recovering"])
        self.assertEqual(h.engine.snapshot()["center_recovery_reason"], "correction_limit")

    def test_raw_inside_axis_is_not_moved_by_old_filtered_error(self):
        h = self.centered_stage("A")
        h.engine._filtered_error = (4.0, 0.0)
        h.tick(horizontal=0.0, vertical=6.0)
        self.assertEqual(h.head.target.pan, 45.0)
        self.assertLess(h.head.target.tilt, 0.0)

    def test_old_filtered_sign_cannot_drive_opposite_to_current_error(self):
        h = self.centered_stage("A")
        h.engine._filtered_error = (6.0, 0.0)
        moves = len(h.head.moves)
        h.tick(horizontal=-2.0)
        self.assertEqual(h.engine.state, "CENTER_A")
        self.assertEqual(len(h.head.moves), moves)
        for _ in range(12):
            h.tick(horizontal=-2.0)
            if len(h.head.moves) > moves:
                break
        self.assertGreater(h.head.target.pan, 45.0)

    def test_tiny_configured_step_is_not_mislabelled_as_limit(self):
        h = self.centered_stage("A", cfg=Config(tilt_sign=-1, correction_gain=.001))
        h.tick(horizontal=2.0)
        self.assertEqual(h.engine.state, "CENTER_A")
        self.assertFalse(h.named("FACE_CENTER_FAILED"))
        self.assertEqual(h.named("FACE_CORRECTION_DEFERRED")[-1]["reason"],
                         "step_below_command_tolerance")

    def test_exact_deadband_is_accepted_only_after_full_continuous_dwell(self):
        h = self.centered_stage("A")
        h.tick(horizontal=1.5, vertical=-1.5)
        started = h.t
        h.run(.24, horizontal=1.5, vertical=-1.5)
        self.assertEqual(h.engine.state, "CENTER_A")
        h.tick(horizontal=1.5, vertical=-1.5)
        self.assertEqual(h.engine.state, "A_READY")
        self.assertGreaterEqual(h.named("FACE_LOCKED")[-1]["time_s"] - started, .25)

    def test_single_raw_outlier_resets_dwell_even_while_ema_inside(self):
        h = self.centered_stage("A")
        h.run(.20, horizontal=1.3)
        h.tick(horizontal=1.501)
        self.assertEqual(h.engine.state, "CENTER_A")
        self.assertIsNone(h.engine._center_since)
        h.run(.24, horizontal=1.3)
        self.assertEqual(h.engine.state, "CENTER_A")
        h.run(.04, horizontal=1.3)
        self.assertEqual(h.engine.state, "A_READY")

    def test_duplicate_frame_does_not_advance_ema_or_dwell(self):
        h = self.centered_stage("A")
        h.tick(horizontal=1.3)
        h.tick(horizontal=1.53)
        filtered = h.engine._filtered_error
        obs = h.engine._last_face
        h.t += .04
        h.head.now = h.t
        h.engine.tick(h.t, obs)
        self.assertEqual(h.engine._filtered_error, filtered)
        self.assertIsNone(h.engine._center_since)
        self.assertEqual(h.engine.state, "CENTER_A")

    def test_duplicate_frame_does_not_create_a_locked_pose(self):
        h = self.centered_stage("A")
        h.tick(horizontal=1.0)
        obs = h.engine._last_face
        h.t += .30
        h.head.now = h.t
        h.engine.tick(h.t, obs)
        self.assertIsNone(h.engine.pose_a)
        self.assertTrue(h.engine.snapshot()["center_recovering"])
        h.t += .8
        h.engine.tick(h.t, obs)
        self.assertEqual(h.engine.fault, "camera_stale")


class ConcurrentPromptTests(unittest.TestCase):
    def test_prompt_starts_once_when_centering_starts_and_finishes_first(self):
        h = Harness(prompt=.1)
        h.until("CENTER_A")
        self.assertFalse(h.engine.pose_a)
        prompt = h.named("PROMPT_STARTED")
        self.assertEqual(len(prompt), 1)
        self.assertEqual(prompt[0]["concurrent_with"], "CENTER_A")
        h.run(.12)
        self.assertEqual(h.engine.state, "CENTER_A")
        self.assertIsNotNone(h.engine._prompt_done_at)
        self.assertFalse(h.command("JOIN"))
        h.until("WAIT_JOIN")
        self.assertTrue(h.engine.pose_a)
        self.assertEqual([name for name, _ in h.player.plays], ["welcome", "prompt"])
        self.assertEqual(len(h.named("PROMPT_FINISHED")), 1)

    def test_centering_finishes_first_and_join_waits_for_playback(self):
        h = Harness(prompt=1)
        h.until("PROMPT")
        self.assertTrue(h.engine.pose_a)
        self.assertIsNone(h.engine._prompt_done_at)
        self.assertFalse(h.command("JOIN"))
        h.until("WAIT_JOIN")
        self.assertFalse(h.command("JOIN"))  # Acoustic guard remains separate.
        h.run(.32)
        self.assertTrue(h.command("JOIN"))
        self.assertEqual([name for name, _ in h.player.plays], ["welcome", "prompt"])

    def test_failed_playback_aborts_while_centering_and_stops_motion(self):
        h = Harness(prompt=1)
        h.until("CENTER_A")
        def broken_poll(now):
            raise RuntimeError("aplay failed")
        h.player.poll = broken_poll
        h.tick(horizontal=8)
        self.assertEqual(h.engine.state, "ABORTED")
        self.assertIn("aplay failed", h.engine.fault)
        self.assertTrue(h.head.stops)
        self.assertTrue(h.player.stopped)
        self.assertIsNone(h.engine.pose_a)

    def test_stop_interrupts_concurrent_prompt_without_replay_or_b_turn(self):
        h = Harness(prompt=2)
        h.until("CENTER_A")
        self.assertTrue(h.player.poll(h.t))
        moves = len(h.head.moves)
        self.assertTrue(h.command("STOP"))
        self.assertEqual(h.engine.fault, "operator_stop")
        self.assertTrue(h.player.stopped)
        h.run(1)
        self.assertEqual(len(h.head.moves), moves)
        self.assertEqual([name for name, _ in h.player.plays], ["welcome", "prompt"])


if __name__ == "__main__":
    unittest.main()
