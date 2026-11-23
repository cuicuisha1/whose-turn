from typing import Protocol


class SpeechRecognizer(Protocol):
    def transcribe(self, pcm: bytes, sample_rate: int) -> str: ...


class SpeechOutput(Protocol):
    def speak(self, text: str) -> None: ...
    def cancel(self) -> None: ...


def reply_to(text, observation):
    message = text.lower()
    target = observation.target if observation.valid else None
    if any(word in message for word in ("距离", "多远", "distance", "how far")):
        if target is not None and target.distance_m is not None:
            return f"当前目标的相机深度约为 {target.distance_m:.2f} 米。"
        return "我目前没有有效的目标距离。"
    if any(word in message for word in ("看见", "看到", "see me")):
        if not observation.valid:
            return "视觉输入暂时不可用，我还不能确认。"
        return "画面中有一个当前关注目标。" if target else "当前画面没有可用的关注目标。"
    if any(word in message for word in ("你好", "hello", "hi", "嗨")):
        return "你好，我在这里。我们可以先保持这个距离。"
    return "我现在可以显示目标位置和距离。语音识别与自由对话尚未接入。"


class TextConversation:
    """Real text input -> deterministic reply -> caption. Never claims speech playback."""
    def __init__(self, config):
        self.config = config
        self.caption = None
        self.received_text = None
        self.source = None
        self.until = 0.0

    def accept(self, command, observation, now):
        if self.config.mode != "text":
            return False
        self.received_text, self.source = command.text, command.source
        self.caption = reply_to(command.text, observation)
        self.until = now + self.config.caption_duration_s
        return True

    def snapshot(self, now, blocked=False):
        if blocked or now >= self.until:
            self.caption = self.received_text = self.source = None
        return {"mode": self.config.mode, "state": "CAPTION_READY" if self.caption else "IDLE",
                "caption": self.caption, "input_text": self.received_text, "input_source": self.source,
                "microphone_active": False, "asr_available": False, "tts_available": False,
                "speech_playing": False, "dialogue_backend": "local_rules"}
