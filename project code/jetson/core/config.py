from dataclasses import asdict, dataclass, field, fields
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


@dataclass
class RuntimeConfig:
    tick_hz: float = 20.0
    output_hz: float = 5.0
    startup_grace_s: float = 8.0
    output_dir: str = "jetson/runs/robot"


@dataclass
class VisionConfig:
    source: str = "simulation"
    detector: str = "synthetic"
    serial: str = ""
    model: str = ""
    device: str = "cpu"
    fps: int = 30
    detection_hz: float = 8.0
    max_frame_age_ms: float = 500.0
    max_restarts: int = 2
    stall_timeout_s: float = 15.0


@dataclass
class BehaviorConfig:
    confirmation_s: float = .35
    max_target_age_s: float = .6
    keep_distance_m: float = .7


@dataclass
class ControlConfig:
    mode: str = "virtual"
    pan_limit_deg: float = 35.0
    tilt_limit_deg: float = 20.0
    max_speed_deg_s: float = 25.0
    gain_deg_s: float = 30.0
    deadband: float = .06


@dataclass
class ArduinoConfig:
    mode: str = "disabled"
    port: str = ""
    poll_hz: float = 2.0
    timeout_s: float = .3
    stale_after_s: float = 1.5


@dataclass
class InterfaceConfig:
    enabled: bool = True
    host: str = "127.0.0.1"
    port: int = 8765


@dataclass
class AudioConfig:
    mode: str = "text"
    caption_duration_s: float = 10.0


@dataclass
class Settings:
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    behavior: BehaviorConfig = field(default_factory=BehaviorConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    arduino: ArduinoConfig = field(default_factory=ArduinoConfig)
    interface: InterfaceConfig = field(default_factory=InterfaceConfig)
    audio: AudioConfig = field(default_factory=AudioConfig)

    def to_dict(self):
        return asdict(self)

    def validate(self):
        for section in fields(self):
            obj = getattr(self, section.name)
            defaults = type(obj)()
            for entry in fields(obj):
                value, expected = getattr(obj, entry.name), type(getattr(defaults, entry.name))
                if not (type(value) in (int, float) if expected is float else type(value) is expected):
                    raise ValueError(f"{section.name}.{entry.name} has an invalid type")
        def bounds(value, low, high, name):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name} must be finite in [{low}, {high}]")
        bounds(self.runtime.tick_hz, 5, 60, "runtime.tick_hz")
        bounds(self.runtime.output_hz, .5, self.runtime.tick_hz, "runtime.output_hz")
        bounds(self.runtime.startup_grace_s, .1, 120, "runtime.startup_grace_s")
        if self.vision.source not in {"simulation", "realsense"}:
            raise ValueError("vision.source must be simulation or realsense")
        if self.vision.detector not in {"synthetic", "hog", "yolo", "none"}:
            raise ValueError("Unknown vision detector")
        if self.vision.source != "simulation" and self.vision.detector == "synthetic":
            raise ValueError("Synthetic detection is forbidden for real camera input")
        if self.vision.detector == "yolo" and not Path(self.vision.model).is_file():
            raise ValueError("YOLO requires an existing local model file")
        if self.vision.fps not in {15, 30}:
            raise ValueError("vision.fps must be 15 or 30")
        bounds(self.vision.detection_hz, .5, 30, "vision.detection_hz")
        bounds(self.vision.max_frame_age_ms, 50, 2000, "vision.max_frame_age_ms")
        bounds(self.vision.max_restarts, 0, 5, "vision.max_restarts")
        bounds(self.vision.stall_timeout_s, 3, 120, "vision.stall_timeout_s")
        bounds(self.behavior.confirmation_s, 0, 5, "behavior.confirmation_s")
        bounds(self.behavior.max_target_age_s, .05, 2, "behavior.max_target_age_s")
        bounds(self.behavior.keep_distance_m, .2, 3, "behavior.keep_distance_m")
        if self.control.mode != "virtual":
            raise ValueError("v0.2 only supports virtual actuation; real motion requires a commissioned adapter")
        bounds(self.control.pan_limit_deg, 1, 90, "control.pan_limit_deg")
        bounds(self.control.tilt_limit_deg, 1, 60, "control.tilt_limit_deg")
        bounds(self.control.max_speed_deg_s, 1, 60, "control.max_speed_deg_s")
        bounds(self.control.gain_deg_s, 1, 120, "control.gain_deg_s")
        bounds(self.control.deadband, 0, .5, "control.deadband")
        if self.arduino.mode not in {"disabled", "monitor"}:
            raise ValueError("Arduino mode must be disabled or monitor")
        if self.arduino.mode == "monitor" and not self.arduino.port.strip():
            raise ValueError("Arduino monitor requires an explicit port")
        bounds(self.arduino.poll_hz, .2, 5, "arduino.poll_hz")
        bounds(self.arduino.timeout_s, .05, 1, "arduino.timeout_s")
        bounds(self.arduino.stale_after_s, 1 / self.arduino.poll_hz + self.arduino.timeout_s, 10, "arduino.stale_after_s")
        if self.interface.host not in {"127.0.0.1", "0.0.0.0"}:
            raise ValueError("interface.host must be 127.0.0.1 or explicitly 0.0.0.0")
        bounds(self.interface.port, 0, 65535, "interface.port")
        if self.audio.mode not in {"text", "disabled"}:
            raise ValueError("v0.2 audio mode must be text or disabled; no ASR/TTS model is installed")
        bounds(self.audio.caption_duration_s, 1, 60, "audio.caption_duration_s")
        return self


def load_settings(path):
    path = Path(path).resolve()
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f"Duplicate configuration key: {key}")
            result[key] = value
        return result
    data = json.loads(path.read_text(encoding="utf-8-sig"), object_pairs_hook=pairs,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    cfg = Settings()
    if not isinstance(data, dict) or set(data) - {f.name for f in fields(cfg)}:
        raise ValueError("Configuration must contain only known section names")
    for section, overrides in data.items():
        obj = getattr(cfg, section)
        if not isinstance(overrides, dict) or set(overrides) - {f.name for f in fields(obj)}:
            raise ValueError(f"Unknown configuration field in {section}")
        for name, value in overrides.items():
            original = getattr(obj, name)
            expected = type(original)
            allowed = type(value) in (int, float) if expected is float else type(value) is expected
            if not allowed:
                raise ValueError(f"{section}.{name} has an invalid type")
            setattr(obj, name, value)
    if cfg.vision.model and not Path(cfg.vision.model).is_absolute():
        cfg.vision.model = str((path.parent / cfg.vision.model).resolve())
    return cfg.validate()
