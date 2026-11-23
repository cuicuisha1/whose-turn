"""可离线运行的任务状态机；系统意图绝不自动当成人的理解。"""
from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class Config:
    version: str = "WT-01"
    calibrated: bool = False
    calibration_record: str = ""
    pan_min_us: int = 1400
    pan_max_us: int = 1600
    neutral_us: int = 1500
    a_us: int = 1460
    b_us: int = 1540
    tilt_us: int = 1500
    slew_us_per_s: float = 80
    ack_hold_s: float = 1.0
    settle_s: float = 0.4
    max_session_s: float = 1200

    def validate(self, hardware=False):
        if type(self.calibrated) is not bool:
            raise ValueError("calibrated 必须是布尔值")
        if not isinstance(self.version, str) or not self.version.strip():
            raise ValueError("缺少版本")
        for k in ("pan_min_us", "pan_max_us", "neutral_us", "a_us", "b_us", "tilt_us"):
            if type(getattr(self, k)) is not int:
                raise ValueError(f"{k} 必须是整数脉宽")
        # 首版禁止扩大既有固件的窄调试范围；扩展必须另行审查并实测。
        if not 1400 <= self.pan_min_us < self.pan_max_us <= 1600:
            raise ValueError("本原型的偏航范围必须在 1400–1600 us 内")
        if not all(self.pan_min_us <= x <= self.pan_max_us for x in (self.neutral_us, self.a_us, self.b_us)):
            raise ValueError("目标超出范围")
        if self.a_us == self.b_us or not 1400 <= self.tilt_us <= 1600:
            raise ValueError("A/B 必须不同；俯仰超限")
        for k, low, high in (("slew_us_per_s", 1, 100), ("ack_hold_s", .2, 5),
                             ("settle_s", .1, 3), ("max_session_s", 10, 1800)):
            v = getattr(self, k)
            if type(v) not in (int, float) or not math.isfinite(v) or not low <= v <= high:
                raise ValueError(f"无效参数: {k}")
        if not isinstance(self.calibration_record, str):
            raise ValueError("标定记录必须为字符串")
        if hardware and (not self.calibrated or not self.calibration_record.strip()):
            raise ValueError("真机执行需要已完成的标定记录；示例配置不能直接启用")
        return self


def load_config(path):
    def pairs(items):
        result = {}
        for k, v in items:
            if k in result:
                raise ValueError(f"重复字段: {k}")
            result[k] = v
        return result
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"), object_pairs_hook=pairs)
    return Config(**data).validate()


class Study:
    """step 是唯一动作入口；操作者标注与任务控制分开记录。"""
    def __init__(self, cfg, head, speech, log, now, variant="head_only"):
        if variant not in {"head_only", "explicit"}:
            raise ValueError("未知设计版本")
        self.cfg, self.head, self.speech, self.log = cfg.validate(), head, speech, log
        self.variant, self.state = variant, "READY"
        self.started, self.last = now, now
        self.intent, self.addressee, self.head_target = "none", None, "neutral"
        self.joined = False
        self.until = None
        self.return_phase = None
        self.emit("session_design", now, variant=variant, config=asdict(cfg))

    def emit(self, event, now, **data):
        self.log({"event": event, "monotonic_s": now, "state": self.state,
                  "design_intent": self.intent, "intended_addressee": self.addressee,
                  "head_target_role": self.head_target, **data})

    def turn(self, role, now):
        self.head_target = role
        self.head.target(getattr(self.cfg, role.lower() + "_us"), now)
        self.emit("head_target_requested", now, motion_primitive="bounded_pan",
                  measured_pose=None)

    def say(self, prompt, now):
        self.speech.play(prompt, now)
        self.emit("speech_requested", now, prompt_id=prompt)

    def stop(self, now, reason="operator_stop"):
        if self.state == "STOPPED":
            return
        self.state, self.intent, self.addressee = "STOPPED", "ended", None
        try:
            self.speech.stop()
        finally:
            self.head.stop()
        self.emit("session_stopped", now, reason=reason, physical_stop_confirmed=False)

    def command(self, command, now, text="", source="operator"):
        self.emit("command", now, command=command, text=text, source=source)
        if command == "stop":
            self.stop(now)
            return
        if self.state in {"STOPPED", "DONE"}:
            raise ValueError("会话已结束，请新建会话；不重放旧动作")
        if command == "mark":
            # 原始观察，不进行“理解正确/错误”的自动编码。
            if not text.strip() or len(text) > 500:
                raise ValueError("标注需为 1–500 字符")
            self.emit("operator_observation", now, observation=text, participant_interpretation=None)
        elif command == "sync":
            self.emit("video_sync_marker", now, label=text or "SYNC")
        elif command == "start" and self.state == "READY":
            self.intent, self.addressee = "invite_response", "A"
            self.turn("A", now)
            self.state = "TURN_A"
        elif command == "join" and self.state == "WAIT_A" and not self.joined:
            self.joined, self.intent = True, "acknowledge_presence"
            self.emit("newcomer_confirmed_by_operator", now, role="B", automatic_detection=False)
            if self.variant == "explicit":
                self.say("ack_explicit", now)
                self.state = "ACK_SPEECH"
            else:
                self.turn("B", now)
                self.state = "ACK_TURN"
        elif command == "advance" and self.state == "WAIT_A" and self.joined:
            self.intent, self.addressee = "invite_response", "B"
            self.turn("B", now)
            self.state = "TURN_B"
        elif command == "repair" and self.state in {"WAIT_A", "WAIT_B"}:
            self.return_phase = self.state
            self.state, self.intent, self.addressee = "REPAIR", "open_negotiation", None
            self.say("repair", now)
        elif command == "finish" and self.state == "WAIT_B" and text in {"materials", "interaction", "skip"}:
            self.intent, self.addressee = "close_activity", None
            self.state = "CONTENT"
            self.say(text, now)
        else:
            raise ValueError(f"当前状态 {self.state} 不接受 {command} {text}")

    def step(self, now):
        if self.state == "STOPPED":
            return
        if not math.isfinite(now) or now < self.last or now - self.last > .25:
            self.stop(now, "clock_or_loop_gap")
            raise RuntimeError("主循环失去时效；停止，不能跳过动作补帧")
        self.last = now
        if now - self.started >= self.cfg.max_session_s:
            self.stop(now, "session_limit")
            return
        self.head.step(now)
        self.speech.step(now)
        if self.state in {"TURN_A", "TURN_B", "ACK_TURN", "RETURN_A"} and self.head.ready(now):
            if self.state == "TURN_A":
                self.say("ask_a", now)
                self.state = "ASK_A"
            elif self.state == "TURN_B":
                self.say("ask_b", now)
                self.state = "ASK_B"
            elif self.state == "ACK_TURN":
                self.state, self.until = "ACK_HOLD", now + self.cfg.ack_hold_s
            else:
                self.state, self.intent, self.addressee = "WAIT_A", "continue_current_response", "A"
        elif self.state == "ACK_HOLD" and now >= self.until:
            self.turn("A", now)
            self.state = "RETURN_A"
        elif self.state == "ACK_SPEECH" and not self.speech.busy:
            self.turn("B", now)
            self.state = "ACK_TURN"
        elif self.state in {"ASK_A", "ASK_B"} and not self.speech.busy:
            self.state = "WAIT_A" if self.state == "ASK_A" else "WAIT_B"
            self.emit("response_opportunity", now)
        elif self.state == "REPAIR" and not self.speech.busy:
            self.state = self.return_phase
            # 开放协商后不恢复“必须由某人答”的假设；计划下一阶段仍由操作员触发。
        elif self.state == "CONTENT" and not self.speech.busy:
            self.head.stop()
            self.state = "DONE"
            self.emit("activity_complete", now, participant_study_completed=False)

    def snapshot(self):
        return {"state": self.state, "design_intent": self.intent,
                "intended_addressee": self.addressee, "head_target_role": self.head_target,
                "commanded_pan_us": self.head.pan, "measured_pan": None,
                "actual_respondent": None, "participant_interpretation": None,
                "speech_busy": self.speech.busy, "speech_prompt": self.speech.prompt}
