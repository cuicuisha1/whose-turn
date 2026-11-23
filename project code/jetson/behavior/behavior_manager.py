class BehaviorManager:
    def __init__(self, config, started_at, startup_grace_s):
        self.config, self.started_at, self.startup_grace_s = config, started_at, startup_grace_s
        self.candidate = None
        self.first_seen = None

    def decide(self, observation, now, paused, faults):
        if faults or paused or not observation.fresh(now, self.config.max_target_age_s) or not observation.target:
            self.candidate = self.first_seen = None
            if faults:
                state, reason = "FAULT", ",".join(faults)
            elif paused:
                state, reason = "PAUSED", "operator_pause"
            elif not observation.fresh(now, self.config.max_target_age_s):
                state = "STARTING" if now - self.started_at < self.startup_grace_s else "PERCEPTION_UNAVAILABLE"
                reason = observation.reason if not observation.valid else "frame_expired"
            else:
                state, reason = "IDLE", "no_current_target"
            return {"state": state, "reason": reason, "head_allowed": False}
        key = (observation.source_id, observation.target.track_id)
        if key != self.candidate:
            self.candidate, self.first_seen = key, observation.captured_at
        # Repeated publication of one frame cannot satisfy the confirmation interval.
        if observation.captured_at - self.first_seen < self.config.confirmation_s:
            return {"state": "PERSON_DETECTED", "reason": "confirming_target", "head_allowed": False}
        distance = observation.target.distance_m
        if distance is not None and distance < self.config.keep_distance_m:
            return {"state": "KEEP_DISTANCE", "reason": "inside_configured_interaction_distance", "head_allowed": True}
        return {"state": "TRACKING", "reason": "depth_unknown" if distance is None else "target_confirmed", "head_allowed": True}
