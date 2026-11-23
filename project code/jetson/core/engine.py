"""Deterministic robot coordination, with no camera, network or serial I/O."""
from .. import __version__
from ..audio.service import TextConversation
from ..behavior.behavior_manager import BehaviorManager
from ..control.head_controller import VirtualHead
from ..control.safety_manager import SafetyManager
from .bus import invalidate_snapshot
from .contracts import parse_observation


CAPABILITIES = {
    "camera_input": ["simulation", "realsense"],
    "person_detection": ["synthetic_fixture", "hog", "optional_local_yolo"],
    "depth": "aligned_camera_z_m", "head_control": "virtual_only",
    "arduino": "optional_status_only",
    "dialogue": "text_input_local_rules_captions", "dashboard": "read_only_http",
    "physical_actuation": False, "physical_stop_confirmed": False,
    "microphone": False, "asr": False, "tts": False, "navigation": False,
}


class RobotEngine:
    def __init__(self, config, run_id, started_at):
        self.config, self.run_id = config, run_id
        self.behavior = BehaviorManager(config.behavior, started_at, config.runtime.startup_grace_s)
        self.safety = SafetyManager(config, started_at)
        self.head = VirtualHead(config.control)
        self.audio = TextConversation(config.audio)
        self.paused = self.shutdown_requested = False
        self.previous_state = None
        self.previous_target = None
        self.last_capture = {}
        self.events = []
        self.state = None

    def step(self, packet, vision_health, arduino, commands, now):
        self.events = []
        max_age = min(self.config.behavior.max_target_age_s, self.config.vision.max_frame_age_ms / 1000)
        observation = parse_observation(packet, self.config.vision.source == "simulation", now, max_age)
        # Accept repeated publications of the same capture, but not timestamp regression.
        if observation.valid:
            last = self.last_capture.get(observation.source_id)
            if last is not None and observation.captured_at < last:
                observation = parse_observation(None, observation.simulated, now, max_age)
            else:
                self.last_capture = {observation.source_id: observation.captured_at}
        target_key = (observation.source_id, observation.target.track_id,
                      observation.target.distance_m is not None) if observation.target else None
        if target_key != self.previous_target:
            self.audio.snapshot(now, blocked=True)
        self.previous_target = target_key
        text_commands, reset = [], False
        for command in commands:
            # Stop/pause still apply if delayed. Delayed resume/text/reset must expire.
            accepted = (command.kind in {"pause", "shutdown"} or 0 <= now - command.created_at <= 2)
            accepted = accepted and not self.shutdown_requested
            if accepted:
                if command.kind == "pause":
                    self.paused = True
                elif command.kind == "resume":
                    self.paused = False
                elif command.kind == "reset":
                    reset = True
                elif command.kind == "shutdown":
                    self.shutdown_requested = True
                elif command.kind == "text":
                    text_commands.append(command)
            self.events.append({"type": "command", "kind": command.kind, "source": command.source,
                                "accepted": accepted, "at": now})
        faults = self.safety.evaluate(vision_health, arduino, now, reset)
        behavior = self.behavior.decide(observation, now, self.paused, faults)
        blocked = bool(faults or self.paused or self.shutdown_requested or not observation.valid)
        for command in text_commands:
            if not blocked:
                self.audio.accept(command, observation, now)
        state = {
            "schema_version": 2, "run_id": self.run_id, "updated_monotonic_s": now,
            "perception_expires_at": observation.captured_at + max_age if observation.valid else now,
            "system": {"status": "running", "version": __version__,
                       "mode": self.config.vision.source, "physical_actuation": False},
            "perception": observation.to_dict(), "behavior": behavior, "faults": faults,
            "head": self.head.update(observation.target, behavior["head_allowed"] and not self.shutdown_requested, now),
            "audio": self.audio.snapshot(now, blocked),
            "modules": {"vision": vision_health, "arduino": arduino},
            "capabilities": CAPABILITIES,
        }
        if self.shutdown_requested:
            self._stop(state, "operator_shutdown")
        if state["behavior"]["state"] != self.previous_state:
            self.events.append({"type": "transition", "from": self.previous_state,
                                "to": state["behavior"]["state"], "reason": state["behavior"]["reason"], "at": now})
            self.previous_state = state["behavior"]["state"]
        self.state = state
        return state

    @staticmethod
    def _stop(state, reason):
        invalidate_snapshot(state, reason)
        state["system"]["status"] = "stopped"
        state["behavior"] = {"state": "STOPPED", "reason": reason, "head_allowed": False}

    def stop(self, now, reason):
        self.shutdown_requested = True
        if self.state is not None:
            self._stop(self.state, reason)
            self.state["updated_monotonic_s"] = now
            self.state["perception_expires_at"] = now
        return self.state
