class VirtualHead:
    """Relative angle simulation, not calibrated servo pulses or measured joint pose."""
    def __init__(self, config):
        self.config = config
        self.pan = self.tilt = 0.0
        self.last_time = None

    def update(self, target, allowed, now):
        dt = 0 if self.last_time is None else max(0, min(.1, now - self.last_time))
        self.last_time = now
        vx = vy = 0.0
        if allowed and target is not None:
            def speed(error):
                if abs(error) <= self.config.deadband:
                    return 0.0
                maximum = self.config.max_speed_deg_s
                return max(-maximum, min(maximum, error * self.config.gain_deg_s))
            vx, vy = speed(2 * target.center_x - 1), speed(1 - 2 * target.center_y)
        old_pan, old_tilt = self.pan, self.tilt
        self.pan = max(-self.config.pan_limit_deg, min(self.config.pan_limit_deg, self.pan + vx * dt))
        self.tilt = max(-self.config.tilt_limit_deg, min(self.config.tilt_limit_deg, self.tilt + vy * dt))
        return {"mode": "virtual", "active": bool(allowed and target), "pan_deg": self.pan, "tilt_deg": self.tilt,
                "pan_velocity_deg_s": (self.pan - old_pan) / dt if dt else 0.0,
                "tilt_velocity_deg_s": (self.tilt - old_tilt) / dt if dt else 0.0,
                "physical_output": False}
