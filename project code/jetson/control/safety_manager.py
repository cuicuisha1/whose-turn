class SafetyManager:
    """Software supervision only. Has no authority to clear firmware faults or cut power."""
    def __init__(self, config, started_at):
        self.config, self.started_at = config, started_at
        self.faults = set()

    def current_faults(self, vision_health, arduino, now):
        current = set()
        if vision_health["state"] == "failed":
            current.add("vision_process_failed")
        if self.config.arduino.mode == "monitor":
            stamp = arduino.get("received_at")
            if not arduino.get("connected") or stamp is None or now - stamp > self.config.arduino.stale_after_s:
                if now - self.started_at > self.config.runtime.startup_grace_s:
                    current.add("arduino_telemetry_unavailable")
            else:
                status = arduino.get("status") or {}
                if status.get("faults", 0) or status.get("state") == "FAULT":
                    current.add("arduino_fault_reported")
        return current

    def evaluate(self, vision_health, arduino, now, reset=False):
        current = self.current_faults(vision_health, arduino, now)
        self.faults.update(current)
        if reset and not current:
            self.faults.clear()
        return sorted(self.faults)
