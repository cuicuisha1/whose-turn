"""Deterministic per-attempt workflow. No camera, serial or audio device imports.

Head positions are software command coordinates; completion is not encoder
feedback. Video T0 and physical motion onset remain external observations.
"""
from __future__ import annotations

from collections import deque
from dataclasses import asdict
import math
from typing import Any, Iterable

from .config import Config
from .models import Pose, FaceObservation, HeadSnapshot
from .vendor.policies import FixedDelayPolicy, PausePolicy
from .vendor.live_audio import TimedAudio


TERMINAL = {"COMPLETED", "ABORTED", "A_READY"}
INVALID_FACE_REASONS = {"NO_FACE", "MULTIPLE_FACES", "INVALID_DEPTH", "INVALID_DEPROJECTION"}
EPS = 1e-9


class Engine:
    def __init__(self, cfg: Config, condition: str, head: Any, player: Any,
                 emit: Any, a_only: bool = False):
        if condition not in {"fixed", "pause"}:
            raise ValueError("condition must be fixed or pause")
        self.cfg, self.condition = cfg, condition
        self.head, self.player, self.emit = head, player, emit
        self.a_only = bool(a_only)
        self.state = "NEW"
        self.pose_a: Pose | None = None
        self.pose_b: Pose | None = None
        self.fault: str | None = None
        self.join_at: float | None = None
        self.trigger_reason: str | None = None
        self.trigger_due_at: float | None = None
        self._now: float | None = None
        self._began_at: float | None = None
        self._state_at = 0.0
        self._move_at = 0.0
        self._expected_pose: Pose | None = None
        self._last_head: HeadSnapshot | None = None
        self._last_face: FaceObservation | None = None
        self._camera_at: float | None = None
        self._processed_face_sequence: int | None = None
        self._last_audio: TimedAudio | None = None
        self._policy = None
        self._welcome_started = False
        self._welcome_done = False
        self._welcome_at: float | None = None
        self._prompt_done_at: float | None = None
        self._prompt_started = False
        self._acquire_track: int | None = None
        self._acquire_since: float | None = None
        self._acquire_last: float | None = None
        self._stable_points: deque[tuple[float, tuple[float, float, float]]] = deque()
        self._stable_track: int | None = None
        self._center_track: int | None = None
        self._center_since: float | None = None
        self._center_last: float | None = None
        self._center_min_time = 0.0
        self._center_steps = 0
        self._center_reacquire = False
        self._filtered_error: tuple[float, float] | None = None
        self._center_role: str | None = None
        self._center_started_at: float | None = None
        self._center_ended_at: float | None = None
        self._center_hold_since: float | None = None
        self._center_hold_reason: str | None = None
        self._center_wait_total = 0.0
        self._center_candidate_seen = False
        self._candidate_track: int | None = None
        self._candidate_since: float | None = None
        self._candidate_last: float | None = None
        self._candidate_point: tuple[float, float, float] | None = None
        self._station_status = {"A": None, "B": None}
        self._centering_quality = {
            role: dict(outside_initial_station_seen=False,
                       locked_outside_initial_station=None,
                       recovery_count=0, recovery_completed_count=0,
                       recovery_wait_s=0.0, elapsed_s=0.0, active_s=0.0)
            for role in ("A", "B")
        }
        self._hold_until: float | None = None
        self._closed_at: float | None = None

    @property
    def terminal(self) -> bool:
        return self.state in TERMINAL

    def _clock(self, now: float) -> None:
        if isinstance(now, bool) or not isinstance(now, (int, float)) or not math.isfinite(now):
            raise ValueError("now must be finite")
        if self._now is not None and now < self._now - EPS:
            raise ValueError("Engine time cannot move backwards")
        self._now = float(now)

    def _event(self, name: str, **fields: Any) -> None:
        self.emit(name, time_s=self._now, **fields)

    def _state(self, name: str) -> None:
        old = self.state
        self.state, self._state_at = name, float(self._now)
        self._event("STATE", previous=old, state=name)

    def _move(self, pose: Pose, speed: float, reason: str) -> None:
        self.head.move(pose, speed, reason)
        self._expected_pose = pose
        self._move_at = float(self._now)
        self._event("HEAD_MOVE_REQUESTED", target=asdict(pose), speed_deg_s=speed,
                    reason=reason, position_feedback=False,
                    timing_meaning="host_request_queued_not_serial_write")

    def begin(self, now: float) -> None:
        if self.state != "NEW":
            raise RuntimeError("One Engine represents one attempt; create a fresh engine for a new attempt")
        self._clock(now)
        self._began_at = float(now)
        self._event("ATTEMPT_BEGIN", condition=self.condition, a_only=self.a_only,
                    config=self.cfg.to_dict(), config_sha256=self.cfg.sha256,
                    analysis_window_s=self.cfg.observation_window_s,
                    analysis_window_origin="external_video_T0_not_available",
                    per_trial_face_centering=True)
        try:
            self._move(Pose(self.cfg.pan_a_base, self.cfg.tilt_base), self.cfg.main_speed, "prepare_a_base")
            self._state("PREPARE_A")
        except Exception as exc:
            self._abort(f"begin_failed: {exc}")

    def _abort(self, reason: str) -> None:
        if self.state == "ABORTED":
            return
        if self.state in {"CENTER_A", "CENTER_B"}:
            self._capture_center_timing()
            self._center_ended_at = float(self._now)
        self.fault = reason
        cleanup = []
        try:
            self.head.stop(reason)
        except Exception as exc:
            cleanup.append(f"head_stop: {exc}")
        try:
            self.player.stop()
        except Exception as exc:
            cleanup.append(f"player_stop: {exc}")
        self._state("ABORTED")
        self._event("ATTEMPT_ABORTED", reason=reason, cleanup_errors=cleanup,
                    stop_is_power_cut=False)

    @staticmethod
    def _same_pose(a: Pose, b: Pose) -> bool:
        return abs(a.pan - b.pan) <= 0.06 and abs(a.tilt - b.tilt) <= 0.06

    def _settled(self, hs: HeadSnapshot) -> bool:
        return (not hs.busy and hs.ramp_done and hs.settled_since_s is not None and
                float(self._now) - hs.settled_since_s + EPS >= self.cfg.settle_s and
                (self._expected_pose is None or
                 (self._same_pose(hs.pose, self._expected_pose) and
                  self._same_pose(hs.target, self._expected_pose))))

    def _fresh_face(self, face: FaceObservation | None) -> bool:
        if face is None or face.error or face.count != 1 or face.track_id is None or face.point_m is None:
            return False
        if face.horizontal_deg is None or face.vertical_deg is None:
            return False
        values = (*face.point_m, face.horizontal_deg, face.vertical_deg)
        return (len(face.point_m) == 3 and all(math.isfinite(x) for x in values) and
                face.point_m[2] > 0 and -EPS <= float(self._now) - face.time_s <= self.cfg.face_fresh_s)

    def _station(self, face: FaceObservation) -> bool:
        x, _, z = face.point_m
        return self.cfg.station_min_z_m <= z <= self.cfg.station_max_z_m and abs(x) <= self.cfg.station_max_abs_x_m

    def _reset_stability(self, reason: str) -> None:
        if self._stable_points:
            self._event("A_STABILITY_RESET", reason=reason)
        self._stable_points.clear()
        self._stable_track = None

    def _observe_a(self, face: FaceObservation | None, hs: HeadSnapshot, is_new: bool) -> None:
        if not self._settled(hs) or not self._fresh_face(face):
            self._reset_stability("camera_moving_or_invalid_face")
            self._acquire_since = self._acquire_last = None
            self._acquire_track = None
            return
        if not is_new:
            return
        if hs.settled_since_s is None or face.time_s + EPS < hs.settled_since_s + self.cfg.settle_s:
            self._reset_stability("frame_precedes_settle")
            return

        if (face.track_id != self._acquire_track or self._acquire_last is None or
                face.time_s - self._acquire_last > self.cfg.max_face_gap_s + EPS):
            self._acquire_track = face.track_id
            self._acquire_since = face.time_s
        self._acquire_last = face.time_s
        if not self._welcome_started and face.time_s - self._acquire_since + EPS >= self.cfg.face_acquire_s:
            self.player.play("welcome", float(self._now))
            self._welcome_started = True
            self._welcome_at = float(self._now)
            self._event("WELCOME_STARTED", track_id=face.track_id)

        if not self._station(face):
            self._reset_stability("outside_a_station")
            return
        if (self._stable_track != face.track_id or
                (self._stable_points and face.time_s - self._stable_points[-1][0] > self.cfg.max_face_gap_s + EPS)):
            self._reset_stability("track_changed_or_frame_gap")
        self._stable_track = face.track_id
        self._stable_points.append((face.time_s, face.point_m))
        cutoff = face.time_s - self.cfg.stable_duration_s
        while len(self._stable_points) > 1 and self._stable_points[1][0] <= cutoff:
            self._stable_points.popleft()
        # Bounding-box diagonal is a conservative 3-D span, not only drift
        # from the immediately previous sample.
        span = math.sqrt(sum((max(p[i] for _, p in self._stable_points) -
                              min(p[i] for _, p in self._stable_points)) ** 2 for i in range(3)))
        if span > self.cfg.stable_span_m:
            self._reset_stability("position_span_exceeded")
            self._stable_track = face.track_id
            self._stable_points.append((face.time_s, face.point_m))
            return
        duration = face.time_s - self._stable_points[0][0]
        if self._welcome_done and duration + EPS >= self.cfg.stable_duration_s:
            self._event("A_STABLE", duration_s=duration, span_m=span,
                        station_is_face_proxy=True, track_id=face.track_id)
            self._start_center("A")

    def _start_center(self, role: str) -> None:
        if self.cfg.tilt_sign is None:
            self._abort("tilt_direction_not_verified")
            return
        self._state("CENTER_" + role)
        # A must retain the target which earned the two-second gate. B has
        # never been observed at this stage and may acquire its first target.
        self._center_track = self._stable_track if role == "A" else None
        self._center_since = None
        self._center_last = self._stable_points[-1][0] if role == "A" and self._stable_points else None
        self._center_min_time = float(self._now)
        self._center_steps = 0
        self._center_reacquire = False
        self._filtered_error = None
        self._center_role = role
        self._center_started_at = float(self._now)
        self._center_ended_at = None
        self._center_hold_since = None
        self._center_hold_reason = None
        self._center_wait_total = 0.0
        self._center_candidate_seen = role == "A"
        self._reset_center_candidate()
        self._event("FACE_CENTER_STARTED", role=role,
                    speed_deg_s=self.cfg.a_speed if role == "A" else self.cfg.b_speed)
        # Full trials: start the prerecorded prompt while A is being centered.
        # A-only validation must remain silent after its existing welcome clip.
        if role == "A" and not self.a_only:
            self.player.play("prompt", float(self._now))
            self._prompt_started = True
            self._event("PROMPT_STARTED", concurrent_with="CENTER_A")

    def _reset_center_candidate(self) -> None:
        self._candidate_track = None
        self._candidate_since = self._candidate_last = None
        self._candidate_point = None

    def _capture_center_timing(self) -> tuple[float, float, float]:
        if self._center_started_at is None:
            return 0.0, 0.0, 0.0
        end = self._center_ended_at if self._center_ended_at is not None else float(self._now)
        elapsed = max(0.0, end - self._center_started_at)
        waiting = self._center_wait_total
        if self._center_hold_since is not None:
            waiting += max(0.0, end - self._center_hold_since)
        active = max(0.0, elapsed - waiting)
        self._centering_quality[self._center_role].update(
            elapsed_s=elapsed, recovery_wait_s=waiting, active_s=active)
        return elapsed, waiting, active

    def _center_hold(self, role: str, reason: str, **details) -> None:
        """A recoverable observation problem holds commands, never sends STOP.

        Initial B acquisition still uses its original active budget. Recovery
        starts only after a valid candidate has been seen or a target bound.
        """
        self._center_since = self._center_last = None
        self._filtered_error = None
        self._reset_center_candidate()
        if not self._center_candidate_seen and self._center_track is None:
            return
        if self._center_hold_since is None:
            self._center_hold_since = float(self._now)
            self._center_hold_reason = reason
            self._center_reacquire = False
            self._centering_quality[role]["recovery_count"] += 1
            self._event("FACE_RECOVERY_STARTED", role=role, reason=reason,
                        previous_track_id=self._center_track,
                        recovery_budget_s=self.cfg.center_recovery_budget_s,
                        budget_already_used_s=self._center_wait_total,
                        action="hold_existing_command_no_new_motion",
                        same_person_verified=False, **details)
        elif reason != self._center_hold_reason:
            previous = self._center_hold_reason
            self._center_hold_reason = reason
            self._event("FACE_RECOVERY_REASON_CHANGED", role=role,
                        previous_reason=previous, reason=reason,
                        recovery_started_at_s=self._center_hold_since, **details)

    def _confirm_center_candidate(self, role: str, face: FaceObservation) -> bool:
        new_candidate = (
            self._candidate_track != face.track_id or self._candidate_last is None or
            face.time_s - self._candidate_last > self.cfg.max_face_gap_s + EPS or
            (self._candidate_point is not None and
             math.dist(face.point_m, self._candidate_point) > .25))
        if new_candidate:
            self._candidate_track = face.track_id
            self._candidate_since = face.time_s
            self._event("FACE_CANDIDATE_CONFIRMING", role=role, track_id=face.track_id,
                        required_s=self.cfg.center_reacquire_s,
                        recovery=self._center_hold_since is not None,
                        same_person_verified=False)
        self._candidate_last = face.time_s
        self._candidate_point = face.point_m
        self._center_candidate_seen = True
        return face.time_s - self._candidate_since + EPS >= self.cfg.center_reacquire_s

    def _resume_center(self, role: str, face: FaceObservation) -> None:
        duration = float(self._now) - self._center_hold_since
        self._center_wait_total += duration
        self._centering_quality[role]["recovery_completed_count"] += 1
        self._event("FACE_RECOVERY_COMPLETED", role=role, reason=self._center_hold_reason,
                    previous_track_id=self._center_track, track_id=face.track_id,
                    wait_duration_s=duration, total_recovery_wait_s=self._center_wait_total,
                    candidate_continuity_s=face.time_s - self._candidate_since,
                    identity_basis="role_local_command_angle_proxy_not_face_identity",
                    same_person_verified=False, face_sequence=face.sequence)
        self._center_hold_since = None
        self._center_hold_reason = None
        self._center_track = face.track_id
        self._center_since = self._center_last = None
        self._filtered_error = None
        self._reset_center_candidate()

    def _record_center_station(self, role: str, face: FaceObservation) -> None:
        inside = self._station(face)
        if not inside:
            self._centering_quality[role]["outside_initial_station_seen"] = True
        if self._station_status[role] is not inside:
            self._station_status[role] = inside
            self._event("CENTER_STATION_STATUS", role=role,
                        inside_initial_station=inside, point_m=face.point_m,
                        track_id=face.track_id, face_sequence=face.sequence,
                        station_measurement="camera_coordinate_face_proxy_not_foot_position",
                        control_effect="quality_marker_only_during_centering",
                        same_person_verified=False)

    def _candidate_can_progress(self, role: str, face: FaceObservation, hs: HeadSnapshot) -> bool:
        """After a limit hold, do not 'recover' to another still-blocked target."""
        h, v = face.horizontal_deg, face.vertical_deg
        if abs(h) <= self.cfg.center_deadband_deg and abs(v) <= self.cfg.center_deadband_deg:
            return True
        base = self.cfg.pan_a_base if role == "A" else self.cfg.pan_b_base
        dp = self.cfg.pan_sign * h if abs(h) > self.cfg.center_deadband_deg else 0.0
        dt = self.cfg.tilt_sign * v if abs(v) > self.cfg.center_deadband_deg else 0.0
        candidate = Pose(
            max(base-self.cfg.pan_correction_limit,
                min(base+self.cfg.pan_correction_limit, hs.pose.pan + dp)),
            max(-self.cfg.tilt_limit, min(self.cfg.tilt_limit, hs.pose.tilt + dt)))
        return not self._same_pose(candidate, hs.pose)

    def _center(self, role: str, face: FaceObservation | None, hs: HeadSnapshot, is_new: bool) -> None:
        timeout = self.cfg.a_center_timeout_s if role == "A" else self.cfg.b_center_timeout_s
        elapsed, waiting, active = self._capture_center_timing()
        if waiting > self.cfg.center_recovery_budget_s + EPS:
            self._event("FACE_CENTER_FAILED", role=role, reason="recovery_budget_exhausted",
                        recovery_reason=self._center_hold_reason, recovery_wait_s=waiting,
                        elapsed_s=elapsed, active_s=active)
            self._abort(f"{role.lower()}_center_recovery_budget_exhausted")
            return
        if active > timeout + EPS or elapsed > timeout + self.cfg.center_recovery_budget_s + EPS:
            self._event("FACE_CENTER_FAILED", role=role, reason="timeout", steps=self._center_steps,
                        elapsed_s=elapsed, recovery_wait_s=waiting, active_s=active,
                        absolute_cap_s=timeout+self.cfg.center_recovery_budget_s)
            self._abort(f"{role.lower()}_center_timeout")
            return
        if not self._settled(hs):
            self._center_since = self._center_last = None
            self._reset_center_candidate()
            return
        minimum = max(self._center_min_time, hs.settled_since_s + self.cfg.settle_s)
        if face is None or float(self._now) - face.time_s > self.cfg.face_fresh_s + EPS:
            self._center_hold(role, "face_not_fresh")
            return
        if not is_new or face.time_s + EPS < minimum:
            return
        if not self._fresh_face(face):
            self._center_hold(role, face.error or ("MULTIPLE_FACES" if face.count > 1 else "INVALID_FACE"))
            return
        if not .35 <= face.point_m[2] <= 3.5:
            self._center_hold(role, "depth_outside_tracking_range")
            return
        base = self.cfg.pan_a_base if role == "A" else self.cfg.pan_b_base
        candidate_pan = hs.pose.pan + self.cfg.pan_sign * face.horizontal_deg
        margin = self.cfg.pan_correction_limit + self.cfg.center_deadband_deg
        if not base - margin - EPS <= candidate_pan <= base + margin + EPS:
            self._center_hold(role, "outside_role_angle_region",
                              candidate_pan_proxy_deg=candidate_pan,
                              accepted_pan_proxy_range_deg=[base-margin, base+margin],
                              coordinate_basis="command_space_proxy_not_measured_world_angle")
            return
        self._record_center_station(role, face)
        if self._center_hold_since is not None:
            if not self._candidate_can_progress(role, face, hs):
                self._center_hold(role, "correction_limit")
                return
            if not self._confirm_center_candidate(role, face):
                return
            self._resume_center(role, face)
        elif self._center_reacquire:
            self._event("FACE_REACQUIRED", role=role,
                        previous_track_id=self._center_track, track_id=face.track_id,
                        reason="after_bounded_correction_in_role_angle_region",
                        same_person_verified=False, face_sequence=face.sequence)
            self._center_reacquire = False
            self._center_since = self._center_last = None
        elif self._center_track is None:
            if self._candidate_last is not None and (
                    face.track_id != self._candidate_track or
                    face.time_s-self._candidate_last > self.cfg.max_face_gap_s + EPS):
                self._center_hold(role, "initial_candidate_interrupted",
                                  candidate_track_id=face.track_id)
                self._confirm_center_candidate(role, face)
                return
            if not self._confirm_center_candidate(role, face):
                return
            self._event("FACE_TARGET_ACQUIRED", role=role, track_id=face.track_id,
                        identity_basis="role_local_command_angle_proxy_not_face_identity",
                        confirmation_s=face.time_s-self._candidate_since,
                        same_person_verified=False, face_sequence=face.sequence)
            self._reset_center_candidate()
        elif face.track_id != self._center_track:
            self._center_hold(role, "track_changed", candidate_track_id=face.track_id)
            self._confirm_center_candidate(role, face)
            return
        elif self._center_last is not None and face.time_s - self._center_last > self.cfg.max_face_gap_s + EPS:
            self._center_hold(role, "face_gap")
            self._confirm_center_candidate(role, face)
            return
        self._center_track, self._center_last = face.track_id, face.time_s
        h_error, v_error = face.horizontal_deg, face.vertical_deg
        # Smooth only fresh, settled observations. Never reuse an error across
        # an active head movement or after target reacquisition.
        if self._filtered_error is None:
            self._filtered_error = (h_error, v_error)
        else:
            alpha = self.cfg.face_ema_alpha
            old_h, old_v = self._filtered_error
            self._filtered_error = (alpha*h_error + (1-alpha)*old_h,
                                    alpha*v_error + (1-alpha)*old_v)
        inside = abs(h_error) <= self.cfg.center_deadband_deg and abs(v_error) <= self.cfg.center_deadband_deg
        if inside:
            if self._center_since is None:
                self._center_since = face.time_s
            if face.time_s - self._center_since + EPS >= self.cfg.center_dwell_s:
                self._lock(role, hs.pose, face)
            return

        self._center_since = None
        base = self.cfg.pan_a_base if role == "A" else self.cfg.pan_b_base
        def step(raw: float, filtered: float, sign: int) -> float:
            # Raw observations define acceptance. EMA only smooths commands:
            # it must not move an already accepted axis or drive against the
            # current error while its sign still reflects older observations.
            if (abs(raw) <= self.cfg.center_deadband_deg or
                    abs(filtered) <= self.cfg.center_deadband_deg or
                    raw * filtered <= 0):
                return 0.0
            # Large errors permit longer continuous Arduino ramps; smaller
            # errors taper naturally via proportional gain. The firmware still
            # handles the 20 ms motion interpolation.
            delta = sign * self.cfg.correction_gain * filtered
            return max(-self.cfg.max_step_deg, min(self.cfg.max_step_deg, delta))
        filtered_h, filtered_v = self._filtered_error
        requested = Pose(
            hs.pose.pan + step(h_error, filtered_h, self.cfg.pan_sign),
            hs.pose.tilt + step(v_error, filtered_v, self.cfg.tilt_sign),
        )
        target = Pose(
            max(base - self.cfg.pan_correction_limit, min(base + self.cfg.pan_correction_limit,
                requested.pan)),
            max(-self.cfg.tilt_limit, min(self.cfg.tilt_limit,
                requested.tilt)),
        )
        if self._same_pose(target, hs.pose):
            # A raw value just outside the deadband can coexist with an EMA
            # value still inside it. That means "wait for new evidence", not
            # "the head reached a limit". Dwell was reset above, and both the
            # freshness checks and the original centering timeout still apply.
            blocked_axes = [axis for axis in ("pan", "tilt")
                            if abs(getattr(requested, axis) - getattr(target, axis)) > EPS]
            details = dict(
                role=role, face_sequence=face.sequence,
                horizontal_error_deg=h_error, vertical_error_deg=v_error,
                filtered_horizontal_error_deg=filtered_h,
                filtered_vertical_error_deg=filtered_v,
                current_pose=asdict(hs.pose), unclamped_target=asdict(requested),
                clamped_target=asdict(target), blocked_axes=blocked_axes,
                pan_bounds_deg=[base - self.cfg.pan_correction_limit,
                                base + self.cfg.pan_correction_limit],
                tilt_bounds_deg=[-self.cfg.tilt_limit, self.cfg.tilt_limit],
                position_feedback=False,
            )
            if blocked_axes:
                self._event("FACE_CORRECTION_LIMIT", reason="correction_limit", **details)
                self._center_hold(role, "correction_limit", blocked_axes=blocked_axes)
            else:
                reason = ("filter_wait" if requested == hs.pose
                          else "step_below_command_tolerance")
                self._event("FACE_CORRECTION_DEFERRED", reason=reason, **details)
            return
        self._center_steps += 1
        self._move(target, self.cfg.a_speed if role == "A" else self.cfg.b_speed,
                   role.lower() + "_face_correction")
        self._center_min_time = float(self._now)
        self._center_reacquire = True
        self._filtered_error = None
        self._center_last = None
        self._event("FACE_CORRECTION", role=role, step=self._center_steps,
                    face_sequence=face.sequence, horizontal_error_deg=h_error,
                    vertical_error_deg=v_error,
                    filtered_horizontal_error_deg=filtered_h,
                    filtered_vertical_error_deg=filtered_v,
                    unclamped_target=asdict(requested), target=asdict(target))

    def _lock(self, role: str, pose: Pose, face: FaceObservation) -> None:
        self._capture_center_timing()
        self._center_ended_at = float(self._now)
        self._centering_quality[role]["locked_outside_initial_station"] = not self._station(face)
        self._event("FACE_LOCKED", role=role, command_pose=asdict(pose),
                    face_sequence=face.sequence, track_id=face.track_id,
                    steps=self._center_steps, centering_duration_s=float(self._now) - self._state_at,
                    centering_quality=dict(self._centering_quality[role]),
                    station_compliance="requires_external_review",
                    position_feedback=False)
        if role == "A":
            self.pose_a = pose
            if self.a_only:
                self._state("A_READY")
                return
            # Centering and playback are independent gates. Never replay prompt.
            if self._prompt_done_at is not None:
                self._state("WAIT_JOIN")
            else:
                self._state("PROMPT")
        else:
            self.pose_b = pose
            self._hold_until = float(self._now) + self.cfg.hold_s
            self._state("HOLD_B")
            self._event("B_HOLD_STARTED", until_s=self._hold_until,
                        origin="software_visual_center_complete_not_physical_arrival")

    def _audio_fresh(self) -> bool:
        if self._last_audio is None:
            return False
        now = float(self._now)
        return (-EPS <= now - self._last_audio.received_at_s <= self.cfg.audio_fresh_s + EPS and
                -EPS <= now - self._last_audio.frame.end_s <= self.cfg.audio_fresh_s + EPS)

    def _ingest_audio(self, items: Iterable[TimedAudio]) -> None:
        for item in items:
            if not isinstance(item, TimedAudio):
                raise ValueError("audio_items must contain TimedAudio")
            if (not math.isfinite(item.received_at_s) or
                    item.received_at_s > float(self._now) + EPS or
                    item.frame.end_s > float(self._now) + EPS):
                raise ValueError("Audio item has a future or invalid timestamp")
            if self._last_audio is not None:
                if item.sequence != self._last_audio.sequence + 1:
                    raise ValueError("Audio sequence is discontinuous")
                if item.frame.start_s < self._last_audio.frame.end_s - EPS:
                    raise ValueError("Audio observations overlap or move backwards")
                if item.received_at_s < self._last_audio.received_at_s - EPS:
                    raise ValueError("Audio receipt time moves backwards")
            self._last_audio = item
            self._event("AUDIO_OBSERVED", audio_sequence=item.sequence, start_s=item.frame.start_s,
                        end_s=item.frame.end_s, status=item.frame.status.value,
                        received_at_s=item.received_at_s, dbfs=item.dbfs,
                        policy_active=self.state == "WAIT_TRIGGER")
            if self.state != "WAIT_TRIGGER" or item.frame.end_s <= self.join_at:
                continue
            self._policy.observe(item.frame)
            # Never evaluate using wall clock ahead of the audio watermark.
            result = self._policy.evaluate(item.frame.end_s)
            if result is not None and self.trigger_reason is None:
                self.trigger_reason = result.reason
                self.trigger_due_at = result.due_at
                self._event("TRIGGER_DUE", reason=result.reason, due_at_s=result.due_at,
                            audio_watermark_s=item.frame.end_s)

    def tick(self, now: float, face: FaceObservation | None = None,
             audio_items: Iterable[TimedAudio] = ()) -> None:
        self._clock(now)
        if self.terminal:
            return
        if self.state == "NEW":
            raise RuntimeError("Call begin before tick")
        try:
            if now - self._began_at > self.cfg.attempt_timeout_s:
                self._abort("attempt_timeout")
                return
            is_new = False
            if face is not None:
                if not math.isfinite(face.time_s) or face.time_s > now + EPS:
                    raise ValueError("Face has a future or invalid timestamp")
                # A fresh camera frame with no usable face is ordinary during
                # approach and head turns. It resets face control/stability;
                # it is not a camera transport failure.
                if face.error and face.error not in INVALID_FACE_REASONS:
                    raise RuntimeError("camera: " + face.error)
                if self._last_face is not None and face.sequence < self._last_face.sequence:
                    raise ValueError("Camera sequence moved backwards")
                if self._last_face is None or face.sequence > self._last_face.sequence:
                    if self._last_face is not None and face.time_s < self._last_face.time_s - EPS:
                        raise ValueError("Camera time moved backwards")
                    self._last_face = face
                    self._camera_at = face.time_s
                    is_new = True
            face = self._last_face
            camera_origin = self._camera_at if self._camera_at is not None else self._began_at
            if now - camera_origin > self.cfg.camera_timeout_s:
                self._abort("camera_stale")
                return
            self._ingest_audio(audio_items)
            active = self.player.poll(now)
            if self._welcome_started and not self._welcome_done and not active:
                self._welcome_done = True
                self._event("WELCOME_FINISHED")
            # Prompt may finish during CENTER_A, before the A pose is locked.
            if self._prompt_started and self._prompt_done_at is None and not active:
                self._prompt_done_at = now
                self._event("PROMPT_FINISHED", join_guard_until_s=now + self.cfg.post_prompt_guard_s)
            hs = self.head.sample(now)
            self._last_head = hs

            if self.state in {"WAIT_TRIGGER", "TURN_B", "CENTER_B", "HOLD_B", "RETURN_A", "OBSERVING"}:
                if not self._audio_fresh():
                    self._abort("audio_stale")
                    return
            if self.state == "PREPARE_A":
                if self._settled(hs):
                    self._state("WAIT_A")
                elif now - self._move_at > self.cfg.move_timeout_s:
                    self._abort("prepare_a_move_timeout")
            elif self.state == "WAIT_A":
                if now - self._state_at > self.cfg.wait_a_timeout_s:
                    self._abort("wait_a_timeout")
                else:
                    self._observe_a(face, hs, is_new)
            elif self.state == "CENTER_A":
                self._center("A", face, hs, is_new)
            elif self.state == "PROMPT":
                if self._prompt_done_at is not None:
                    self._state("WAIT_JOIN")
            elif self.state == "WAIT_JOIN":
                if now - self._state_at > self.cfg.wait_join_timeout_s:
                    self._abort("wait_join_timeout")
            elif self.state == "WAIT_TRIGGER":
                if self.trigger_reason is not None:
                    self._move(Pose(self.cfg.pan_b_base, self.cfg.tilt_base), self.cfg.main_speed, "turn_to_b_base")
                    self._event("TRIGGER_COMMAND_QUEUED", reason=self.trigger_reason,
                                due_at_s=self.trigger_due_at, queue_latency_s=now-self.trigger_due_at,
                                timing_meaning="host_request_queued_not_serial_write")
                    self._state("TURN_B")
            elif self.state == "TURN_B":
                if self._settled(hs):
                    self._start_center("B")
                elif now - self._move_at > self.cfg.move_timeout_s:
                    self._abort("turn_b_move_timeout")
            elif self.state == "CENTER_B":
                self._center("B", face, hs, is_new)
            elif self.state == "HOLD_B":
                if now + EPS >= self._hold_until:
                    self._move(self.pose_a, self.cfg.main_speed, "return_to_locked_a")
                    self._state("RETURN_A")
            elif self.state == "RETURN_A":
                if self._settled(hs):
                    self._event("MOTION_SEQUENCE_COMPLETE", position_feedback=False)
                    self._state("OBSERVING")
                elif now - self._move_at > self.cfg.move_timeout_s:
                    self._abort("return_a_move_timeout")
        except Exception as exc:
            self._abort(f"runtime_fault: {type(exc).__name__}: {exc}")

    def fail(self, reason: str, now: float) -> bool:
        """Abort a running attempt for an external adapter/UI/logging failure.

        Terminal results are not rewritten here; the enclosing session still
        owns resource cleanup and any error after a terminal result.
        """
        self._clock(now)
        if self.terminal:
            return False
        self._abort(str(reason))
        return True

    def command(self, cmd: str, now: float) -> bool:
        self._clock(now)
        cmd = cmd.strip().upper()
        if cmd == "STATUS":
            self._event("STATUS_REQUEST", snapshot=self.snapshot())
            return True
        if self.terminal:
            self._event("COMMAND_REJECTED", command=cmd, reason="attempt_terminal")
            return False
        if cmd in {"STOP", "QUIT"}:
            self._abort("operator_" + cmd.lower())
            return True
        if cmd == "END":
            if self.state != "OBSERVING":
                self._abort("early_end_before_motion_sequence_complete")
            else:
                self._closed_at = now
                self._state("COMPLETED")
                self._event("OBSERVATION_CLOSED", observation_closed_at_s=now,
                            analysis_window_origin="external_video_T0_not_available")
            return True
        if cmd == "JOIN":
            reason = None
            if self.state != "WAIT_JOIN":
                reason = "not_waiting_for_join"
            elif self._prompt_done_at is None or now < self._prompt_done_at + self.cfg.post_prompt_guard_s:
                reason = "prompt_echo_guard"
            elif not self._audio_fresh():
                reason = "audio_not_fresh"
            elif self._last_audio.frame.start_s < self._prompt_done_at + self.cfg.post_prompt_guard_s - EPS:
                reason = "audio_precedes_prompt_guard"
            if reason:
                self._event("COMMAND_REJECTED", command=cmd, reason=reason)
                return False
            self.join_at = now
            self._policy = (FixedDelayPolicy(self.cfg.fixed_delay_s) if self.condition == "fixed"
                            else PausePolicy(self.cfg.pause_s, self.cfg.max_wait_s))
            self._policy.start(now)
            self._event("JOIN_MARK_RECEIVED", join_received_at_s=now,
                        t0_source="external_video_not_measured_here")
            self._state("WAIT_TRIGGER")
            return True
        self._event("COMMAND_REJECTED", command=cmd, reason="use_JOIN_STOP_END_STATUS_QUIT")
        return False

    def snapshot(self) -> dict[str, Any]:
        ready = (self.state == "WAIT_JOIN" and self._prompt_done_at is not None and
                 float(self._now) >= self._prompt_done_at + self.cfg.post_prompt_guard_s and
                 self._audio_fresh() and
                 self._last_audio.frame.start_s >= self._prompt_done_at + self.cfg.post_prompt_guard_s - EPS)
        elapsed, waiting, active = self._capture_center_timing()
        centering = self.state in {"CENTER_A", "CENTER_B"}
        recovering = centering and self._center_hold_since is not None
        candidate_duration = (max(0.0, self._candidate_last-self._candidate_since)
                              if self._candidate_last is not None and self._candidate_since is not None else 0.0)
        return {
            "state": self.state, "condition": self.condition, "a_only": self.a_only,
            "terminal": self.terminal, "ready_for_join": ready,
            "a_locked": self.pose_a is not None, "b_locked": self.pose_b is not None,
            "command_pose_a": asdict(self.pose_a) if self.pose_a else None,
            "command_pose_b": asdict(self.pose_b) if self.pose_b else None,
            "join_received_at_s": self.join_at, "trigger_reason": self.trigger_reason,
            "trigger_due_at_s": self.trigger_due_at, "fault": self.fault,
            "last_time_s": self._now, "observation_closed_at_s": self._closed_at,
            "analysis_window_s": self.cfg.observation_window_s,
            "analysis_window_origin": "external_video_T0_not_available",
            "position_feedback": False, "config_sha256": self.cfg.sha256,
            "center_role": self._center_role,
            "center_recovering": recovering,
            "center_recovery_reason": self._center_hold_reason,
            "center_recovery_wait_s": waiting,
            "center_recovery_budget_remaining_s": max(0.0, self.cfg.center_recovery_budget_s-waiting),
            "center_candidate_track_id": self._candidate_track if centering else None,
            "center_candidate_confirmed_s": candidate_duration if centering else 0.0,
            "center_elapsed_s": elapsed, "center_active_s": active,
            "centering_quality": {role: dict(q) for role, q in self._centering_quality.items()},
            "station_compliance": "requires_external_review",
            "same_person_verified": False,
        }
