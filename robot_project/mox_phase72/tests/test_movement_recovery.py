"""v0.2.3 moving-target recovery: synthetic observations, no hardware."""
import unittest

from mox72.config import Config
from mox72.models import FaceObservation, Pose
from test_engine import Harness


class MovementRecoveryTests(unittest.TestCase):
    def start_a(self, *, full=False, cfg=None, prompt=.2):
        h = Harness(a_only=not full, cfg=cfg or Config(tilt_sign=-1), prompt=prompt)
        h.until("CENTER_A")
        return h

    def start_b(self, *, cfg=None):
        h = Harness(cfg=cfg or Config(tilt_sign=-1))
        h.ready()
        self.assertTrue(h.command("JOIN"))
        h.until("CENTER_B")
        return h

    def recover(self, h, *, track=2, point=(0, 0, 2.4), horizontal=0, vertical=0):
        start = h.t
        while h.engine.snapshot()["center_recovering"] and not h.engine.terminal:
            self.assertLess(h.t-start, .8)
            h.tick(track=track, point=point, horizontal=horizontal, vertical=vertical)
        self.assertIsNone(h.engine.fault)

    def invalid(self, h, duration=.02, *, error="NO_FACE", count=0):
        h.run(duration, count=count, track=None, point=None, face_error=error)

    def test_a_can_move_from_1m_to_2point4m_with_new_track(self):
        h = self.start_a()
        h.tick(horizontal=1)
        h.tick(track=11, point=(0, 0, 2.422))
        self.assertTrue(h.engine.snapshot()["center_recovering"])
        self.assertIsNone(h.engine._filtered_error)
        self.assertIsNone(h.engine._center_since)
        self.assertFalse(h.head.stops)
        h.until("A_READY", track=11, point=(0, 0, 2.422))
        q = h.engine.snapshot()["centering_quality"]["A"]
        self.assertTrue(q["outside_initial_station_seen"])
        self.assertTrue(q["locked_outside_initial_station"])
        self.assertEqual(q["recovery_count"], 1)
        self.assertEqual(q["recovery_completed_count"], 1)
        self.assertEqual(len(h.named("A_STABLE")), 1)
        event = h.named("FACE_RECOVERY_COMPLETED")[0]
        self.assertEqual(event["previous_track_id"], 1)
        self.assertEqual(event["track_id"], 11)
        self.assertFalse(event["same_person_verified"])
        self.assertGreaterEqual(event["candidate_continuity_s"]+1e-9, .3)

    def test_same_track_motion_is_not_a_station_abort(self):
        h = self.start_a()
        h.tick(point=(.15, 0, 2.4), horizontal=4)
        self.assertEqual(h.engine.state, "CENTER_A")
        self.assertFalse(h.engine.snapshot()["center_recovering"])
        self.assertEqual(h.head.moves[-1][3], "a_face_correction")
        self.assertTrue(h.engine.snapshot()["centering_quality"]["A"]["outside_initial_station_seen"])

    def test_initial_far_a_cannot_skip_station_and_two_second_gate(self):
        h = Harness(a_only=True, cfg=Config(tilt_sign=-1))
        h.run(4, point=(0, 0, 2.4))
        self.assertEqual(h.engine.state, "WAIT_A")
        self.assertFalse(h.named("A_STABLE"))
        h.run(1.9)
        self.assertEqual(h.engine.state, "WAIT_A")
        h.until("A_READY")
        self.assertGreaterEqual(h.named("A_STABLE")[0]["duration_s"]+1e-9, 2.)

    def test_first_b_at_2point4m_requires_confirmation_but_not_old_station(self):
        h = self.start_b()
        moves = len(h.head.moves)
        h.run(.28, track=2, point=(0, 0, 2.4), horizontal=4)
        self.assertEqual(len(h.head.moves), moves)
        self.assertIsNone(h.engine.pose_b)
        h.run(.06, track=2, point=(0, 0, 2.4), horizontal=4)
        self.assertGreater(len(h.head.moves), moves)
        h.until("HOLD_B", track=2, point=(0, 0, 2.4))
        q = h.engine.snapshot()["centering_quality"]["B"]
        self.assertTrue(q["locked_outside_initial_station"])
        self.assertEqual(q["recovery_count"], 0)
        self.assertGreaterEqual(h.named("FACE_TARGET_ACQUIRED")[-1]["confirmation_s"]+1e-9, .3)

    def test_initial_missing_b_retains_full_four_second_acquisition_budget(self):
        h = self.start_b()
        began = h.t
        self.invalid(h, 3.8)
        self.assertEqual(h.engine.state, "CENTER_B")
        self.assertFalse(h.engine.snapshot()["center_recovering"])
        self.assertEqual(h.engine.snapshot()["center_recovery_wait_s"], 0)
        self.invalid(h, .24)
        self.assertEqual(h.engine.fault, "b_center_timeout")
        self.assertLessEqual(h.t-began, 4.05)
        self.assertFalse(h.named("FACE_RECOVERY_STARTED"))

    def test_loss_during_first_b_candidate_confirmation_uses_recovery(self):
        h = self.start_b()
        h.run(.10, track=2)
        self.assertIsNone(h.engine._center_track)
        self.invalid(h, .10)
        self.assertTrue(h.engine.snapshot()["center_recovering"])
        self.recover(h, track=3)
        h.until("HOLD_B", track=3, point=(0, 0, 2.4))
        self.assertEqual(h.engine.snapshot()["centering_quality"]["B"]["recovery_completed_count"], 1)

    def test_short_loss_multiple_faces_and_invalid_depth_hold_then_resume(self):
        for error, count in (("NO_FACE", 0), ("MULTIPLE_FACES", 2), ("INVALID_DEPTH", 1)):
            with self.subTest(error=error):
                h = self.start_a()
                h.run(.12, horizontal=1.)
                moves = len(h.head.moves)
                self.invalid(h, .16, error=error, count=count)
                self.assertTrue(h.engine.snapshot()["center_recovering"])
                self.assertIsNone(h.engine._filtered_error)
                self.assertIsNone(h.engine._center_since)
                self.assertEqual(len(h.head.moves), moves)
                self.recover(h, track=2)
                self.assertEqual(h.engine.state, "CENTER_A")
                recovered_at = h.t
                h.run(.24, track=2, point=(0, 0, 2.4))
                self.assertEqual(h.engine.state, "CENTER_A")
                h.until("A_READY", track=2, point=(0, 0, 2.4))
                self.assertGreaterEqual(h.t-recovered_at, .25)
                self.assertFalse(h.head.stops)

    def test_opposite_role_candidate_cannot_be_adopted(self):
        h = self.start_a()
        moves = len(h.head.moves)
        # At commanded A=+45, a +90deg image error estimates candidate=-45.
        h.run(.5, track=2, horizontal=90)
        self.assertEqual(len(h.head.moves), moves)
        self.assertTrue(h.engine.snapshot()["center_recovering"])
        self.assertEqual(h.engine.snapshot()["center_recovery_reason"], "outside_role_angle_region")
        self.assertIsNone(h.engine.snapshot()["center_candidate_track_id"])
        self.assertFalse(h.engine.snapshot()["centering_quality"]["A"]["outside_initial_station_seen"])
        self.recover(h, track=3)
        self.assertEqual(h.engine._center_track, 3)
        self.assertTrue(all(pose.pan > 0 for _,pose,_,_ in h.head.moves))

    def test_depth_outside_sdk_tracking_range_holds_without_motion(self):
        for depth in (.2, 4.):
            with self.subTest(depth=depth):
                h = self.start_a()
                moves = len(h.head.moves)
                h.run(.4, point=(0, 0, depth), horizontal=8)
                self.assertEqual(len(h.head.moves), moves)
                self.assertEqual(h.engine.snapshot()["center_recovery_reason"], "depth_outside_tracking_range")
                self.assertIsNone(h.engine.pose_a)

    def test_changing_candidate_restarts_confirmation_not_recovery_deadline(self):
        h = self.start_a()
        self.invalid(h)
        began = h.engine._center_hold_since
        h.run(.20, track=2, point=(0, 0, 2.4))
        self.assertGreater(h.engine.snapshot()["center_candidate_confirmed_s"], .15)
        h.tick(track=3, point=(0, 0, 2.4))
        self.assertEqual(h.engine.snapshot()["center_candidate_confirmed_s"], 0)
        self.assertEqual(h.engine._center_hold_since, began)
        h.run(.24, track=3, point=(0, 0, 2.4))
        self.assertTrue(h.engine.snapshot()["center_recovering"])
        self.recover(h, track=3)
        self.assertEqual(len(h.named("FACE_RECOVERY_STARTED")), 1)

    def test_large_candidate_position_jump_restarts_confirmation(self):
        h = self.start_a()
        self.invalid(h)
        h.run(.20, track=2, point=(0, 0, 2.4))
        h.tick(track=2, point=(0, 0, 1.2))
        self.assertEqual(h.engine.snapshot()["center_candidate_confirmed_s"], 0)
        h.run(.24, track=2, point=(0, 0, 1.2))
        self.assertTrue(h.engine.snapshot()["center_recovering"])
        self.recover(h, track=2, point=(0, 0, 1.2))

    def test_duplicate_sequence_does_not_accumulate_candidate_time(self):
        h = self.start_a()
        self.invalid(h)
        h.tick(track=2, point=(0, 0, 2.4))
        obs = h.engine._last_face
        for _ in range(5):
            h.t += .02
            h.engine.tick(h.t, obs)
        self.assertTrue(h.engine.snapshot()["center_recovering"])
        self.assertEqual(h.engine.snapshot()["center_candidate_confirmed_s"], 0)
        self.assertIsNone(h.engine.pose_a)

    def test_candidate_confirmation_never_accumulates_during_camera_motion(self):
        h = self.start_a()
        self.invalid(h)
        h.head.busy = True
        h.run(.4, track=2)
        self.assertIsNone(h.engine._candidate_track)
        h.head.busy = False
        h.head.settled = h.t
        h.run(.10, track=2)
        self.assertIsNone(h.engine._candidate_track)
        h.run(.20, track=2)
        self.assertTrue(h.engine.snapshot()["center_recovering"])
        self.assertLess(h.engine.snapshot()["center_candidate_confirmed_s"], .3)
        self.recover(h, track=2, point=(0, 0, 1.))

    def test_repeated_losses_share_one_cumulative_two_second_budget(self):
        h = self.start_a()
        for track in (2, 3):
            self.invalid(h, .4)
            self.recover(h, track=track)
        self.assertEqual(h.engine.snapshot()["centering_quality"]["A"]["recovery_completed_count"], 2)
        used = h.engine.snapshot()["center_recovery_wait_s"]
        self.assertGreater(used, 1.3)
        self.invalid(h, .7)
        self.assertEqual(h.engine.fault, "a_center_recovery_budget_exhausted")
        self.assertLessEqual(h.engine.snapshot()["center_recovery_wait_s"], 2.03)
        self.assertEqual(len(h.named("FACE_RECOVERY_STARTED")), 3)

    def test_candidate_churn_cannot_restart_hold_budget(self):
        h = self.start_a()
        self.invalid(h)
        began = h.t
        for index in range(110):
            h.tick(track=index+2)
            if h.engine.terminal:
                break
        self.assertEqual(h.engine.fault, "a_center_recovery_budget_exhausted")
        self.assertLessEqual(h.t-began, 2.03)
        self.assertFalse(h.named("FACE_RECOVERY_COMPLETED"))

    def test_limit_hold_can_resume_inward_without_expanding_bounds(self):
        h = self.start_a()
        h.head.pose = h.head.target = Pose(45, 25)
        h.engine._expected_pose = h.head.pose
        h.tick(vertical=-5)
        self.assertTrue(h.engine.snapshot()["center_recovering"])
        moves = len(h.head.moves)
        h.run(.2, vertical=-5)
        self.assertEqual(len(h.head.moves), moves)
        self.recover(h, track=1, point=(0, 0, 1), vertical=5)
        self.assertLess(h.head.target.tilt, 25)
        self.assertTrue(all(abs(pose.tilt) <= 25 for _,pose,_,_ in h.head.moves))
        self.assertFalse(h.head.stops)

    def test_in_band_face_at_command_boundary_is_not_role_rejected(self):
        h = self.start_a()
        h.head.pose = h.head.target = Pose(35, 25)
        h.engine._expected_pose = h.head.pose
        h.until("A_READY", horizontal=1., vertical=-1.)
        self.assertEqual(h.engine.pose_a, Pose(35,25))
        self.assertFalse(h.named("FACE_RECOVERY_STARTED"))

    def test_prompt_not_replayed_and_locked_a_pose_does_not_follow_movement(self):
        h = self.start_a(full=True, prompt=.2)
        self.invalid(h, .2)
        self.recover(h, track=2)
        h.until("WAIT_JOIN", track=2, point=(0, 0, 2.4))
        locked = h.engine.pose_a
        moves = len(h.head.moves)
        h.run(.5, track=3, point=(1,0,3), horizontal=50)
        self.assertEqual(h.engine.pose_a, locked)
        self.assertEqual(len(h.head.moves), moves)
        self.assertEqual([kind for kind,_ in h.player.plays], ["welcome", "prompt"])
        self.assertEqual(len(h.named("FACE_CENTER_STARTED")), 1)
        self.assertEqual(h.engine.snapshot()["station_compliance"], "requires_external_review")

    def test_real_camera_failure_is_not_recoverable_target_loss(self):
        h = self.start_a()
        self.invalid(h)
        h.tick(face_error="USB camera transport failed", count=0, point=None)
        self.assertEqual(h.engine.state, "ABORTED")
        self.assertIn("USB camera transport failed", h.engine.fault)
        self.assertTrue(h.head.stops)

    def test_quality_records_return_inside_and_final_lock_position(self):
        h = self.start_a()
        h.tick(track=2, point=(0,0,2.4))
        self.recover(h, track=2, point=(0,0,1.))
        h.until("A_READY", track=2)
        q = h.engine.snapshot()["centering_quality"]["A"]
        self.assertTrue(q["outside_initial_station_seen"])
        self.assertFalse(q["locked_outside_initial_station"])
        self.assertEqual([e["inside_initial_station"] for e in h.named("CENTER_STATION_STATUS")], [False, True])
        frozen = dict(q)
        h.run(.5, track=2)
        self.assertEqual(h.engine.snapshot()["centering_quality"]["A"], frozen)

    def test_recovery_config_constraints_are_explicit(self):
        for values in ({"center_reacquire_s": .01}, {"center_recovery_budget_s": 0},
                       {"center_reacquire_s": 1, "center_recovery_budget_s": .5},
                       {"center_recovery_budget_s": 11},
                       {"center_reacquire_s": 4},
                       {"pan_a_base": 10, "pan_b_base": -10}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Config(**values)


if __name__ == "__main__":
    unittest.main()
