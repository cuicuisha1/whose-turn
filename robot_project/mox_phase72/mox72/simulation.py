"""Synthetic integration exercise; no device, model or third-party import.

These generated observations are demonstrations, never human-study evidence.
The simulator models the commanded ramp only, not loaded servo mechanics.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .config import Config
from .engine import Engine
from .head_serial import SimHead
from .models import FaceObservation, Pose
from .runtime_io import SimPlayer
from .vendor.live_audio import TimedAudio
from .vendor.policies import AudioFrame, AudioStatus


@dataclass
class SimClock:
    now: float = 1000.0

    def __call__(self):
        return self.now


def run_simulation(cfg: Config, condition: str, scope: str, scenario: str,
                   emit, *, clock: SimClock | None = None, progress=None):
    if scope not in {"a", "full"} or scenario not in {"silence", "speech", "missing_b", "moving"}:
        raise ValueError("Invalid simulation scope/scenario")
    cfg.validate_live()  # Sign here is synthetic, explicitly assigned by CLI.
    clock = clock or SimClock()
    origin = clock()
    head = SimHead(emit, clock=clock,
                   pan_limit=max(60, max(abs(cfg.pan_a_base), abs(cfg.pan_b_base)) + cfg.pan_correction_limit),
                   tilt_limit=cfg.tilt_limit)
    player = SimPlayer(emit)
    engine = Engine(cfg, condition, head, player, emit, a_only=scope == "a")
    emit("SIMULATION_BEGIN", time_s=clock(), scenario=scenario,
         simulated=True, motion_simulated=True, synthetic_camera_fps=30,
         synthetic_audio_frame_s=0.02, robot_mechanics_simulated=False)
    engine.begin(clock())
    face = None
    camera_index = 0
    ready_at = observing_at = None
    moving_a_started_at = None
    last_state = None
    # Desired image-centred poses vary from the nominal bases in this example.
    a_pose = Pose(cfg.pan_a_base + min(3, cfg.pan_correction_limit / 2),
                  max(-cfg.tilt_limit, cfg.tilt_base - 4))
    b_pose = Pose(cfg.pan_b_base - min(2, cfg.pan_correction_limit / 2),
                  min(cfg.tilt_limit, cfg.tilt_base + 3))
    try:
        for index in range(1, int((cfg.attempt_timeout_s + 2) / .02) + 1):
            previous = origin + (index - 1) * .02
            clock.now = origin + index * .02
            elapsed = clock() - origin
            hs = head.sample(clock())
            if scenario == "moving" and engine.state == "CENTER_A" and moving_a_started_at is None:
                moving_a_started_at = clock()
                emit("SIMULATED_PARTICIPANT_MOVEMENT", time_s=clock(), role="A",
                     target_depth_m=2.4, spatial_track_changes=True,
                     brief_no_face_s=.16, simulated=True,
                     meaning="synthetic_tracking_input_not_measured_participant_motion")
            if elapsed + 1e-9 >= camera_index / 30:
                camera_index += 1
                seeing_b = engine.state in {"TURN_B", "CENTER_B", "HOLD_B"}
                visible = elapsed >= 1.5 and not (seeing_b and scenario == "missing_b")
                desired = b_pose if seeing_b else a_pose
                h = (desired.pan - hs.pose.pan) / cfg.pan_sign
                v = (desired.tilt - hs.pose.tilt) / cfg.tilt_sign
                # A approaches from farther away, reaching the marker at t=5.
                z = 1.0 if seeing_b else max(1.0, 2.5 - max(0, elapsed - 2) * .5)
                track_id = 2 if seeing_b else 1
                if scenario == "moving":
                    if seeing_b:
                        z = 2.4
                    elif moving_a_started_at is not None:
                        z = 2.4
                        since_move = clock() - moving_a_started_at
                        # Change spatial ID at the depth jump, briefly lose the
                        # face, then reacquire a new ID. Initial WAIT_A is intact.
                        visible = visible and not (.10 <= since_move < .26)
                        track_id = 11 if since_move < .26 else 12
                visible = visible and abs(h) < 30 and abs(v) < 22
                point = (math.tan(math.radians(h)) * z, math.tan(math.radians(v)) * z, z) if visible else None
                face = FaceObservation(clock(), camera_index,
                                       track_id if visible else None,
                                       1 if visible else 0, point,
                                       h if visible else None, v if visible else None,
                                       score=.99 if visible else 0,
                                       camera_frame_number=camera_index,
                                       device_timestamp_ms=elapsed * 1000)
                emit("VISION_OBSERVATION", time_s=clock(), face_sequence=face.sequence,
                     count=face.count, track_id=face.track_id, point_m=face.point_m,
                     horizontal_deg=face.horizontal_deg, vertical_deg=face.vertical_deg,
                     simulated=True)
            items = []
            if scope == "full":
                status = AudioStatus.VOICED if scenario == "speech" else AudioStatus.UNVOICED
                items = [TimedAudio(AudioFrame(previous, clock(), status), clock(), index,
                                    -28.0 if status is AudioStatus.VOICED else -55.0)]
            engine.tick(clock(), face, items)
            if engine.snapshot()["ready_for_join"]:
                if ready_at is None:
                    ready_at = clock()
                elif clock() - ready_at >= .50:
                    engine.command("JOIN", clock())
            if engine.state == "OBSERVING":
                if observing_at is None:
                    observing_at = clock()
                elif clock() - observing_at >= .5:
                    engine.command("END", clock())
            if engine.state != last_state:
                last_state = engine.state
                if progress:
                    progress(engine.snapshot())
            if engine.terminal:
                break
        if not engine.terminal:
            engine.fail("simulation_iteration_limit", clock())
        return {**engine.snapshot(), "simulated": True, "motion_simulated": True,
                "input_source": "synthetic", "scenario": scenario,
                "simulated_elapsed_s": round(clock() - origin, 3)}
    finally:
        head.close()
        player.close()
