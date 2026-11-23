"""使用 head-1 USB 协议；保留供电与舵机验收门槛。"""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import wave

from host.robot_test import Client, SerialTransport, SimulationTransport, ProtocolError


class Head:
    def __init__(self, cfg, log, now, client=None):
        self.cfg, self.log, self.client = cfg, log, client
        self.pan = float(cfg.neutral_us)
        self.goal, self.last, self.arrival = self.pan, now, now
        self.armed, self.stopped = client is None, False
        self.last_sent = None

    def prepare(self):
        # 由显式 prepare 命令触发；不自动 clear、不自动重连。
        if self.client:
            self.client.request("prepare", mode="head")
            self.client.wait(.1)
            status = self.client.request("status")
            if status.get("protocol_version") != "head-1" or status.get("state") != "ARMED" or status.get("mode") != "head":
                raise ProtocolError("底层协议不匹配或未进入 head / ARMED")
            self.client.request("servo", pan_us=self.cfg.neutral_us,
                                tilt_us=self.cfg.tilt_us, arm_epoch=self.client.epoch)
        self.armed = True
        self.last = time.monotonic()

    def target(self, pulse, now):
        if not self.armed or self.stopped:
            raise ProtocolError("头部未准备或已停止")
        if type(pulse) is not int or not self.cfg.pan_min_us <= pulse <= self.cfg.pan_max_us:
            raise ValueError("脉宽越界")
        self.goal, self.arrival = float(pulse), None

    def step(self, now):
        if self.stopped:
            return
        dt = now - self.last
        if dt < 0 or dt > .25:
            raise RuntimeError("头部更新间隔异常")
        self.last = now
        if self.client:
            self.client.maintain()
            if self.armed and self.client.state != "ARMED":
                raise ProtocolError("底层退出 ARMED；禁止继续动作")
        if not self.armed:
            return
        difference = self.goal - self.pan
        delta = min(abs(difference), self.cfg.slew_us_per_s * dt)
        self.pan += delta if difference > 0 else -delta
        pulse = round(self.pan)
        if pulse != self.last_sent:
            if self.client:
                self.client.request("servo", pan_us=pulse, tilt_us=self.cfg.tilt_us,
                                    arm_epoch=self.client.epoch)
            self.log({"event": "servo_command", "monotonic_s": now, "pan_us": pulse,
                      "tilt_us": self.cfg.tilt_us, "protocol_ack": self.client is not None,
                      "measured_pose": None})
            self.last_sent = pulse
        if abs(self.pan - self.goal) < .001 and self.arrival is None:
            self.arrival = now

    def ready(self, now):
        # 仅是指令插值与预留等待结束，不是编码器测得的真实到位。
        return self.arrival is not None and now - self.arrival >= self.cfg.settle_s

    def stop(self):
        self.stopped, self.armed = True, False
        if self.client:
            self.client.best_effort_stop()


class Speech:
    def __init__(self, prompts, log, mode="text", audio_dir=None):
        self.prompts, self.log, self.mode = prompts, log, mode
        self.audio_dir = Path(audio_dir) if audio_dir else None
        self.prompt, self.busy, self.process, self.end = None, False, None, None
        if mode == "wav":
            if self.audio_dir is None:
                raise ValueError("wav 模式需要 --audio-dir")
            for name in prompts:
                with wave.open(str(self.audio_dir / (name + ".wav")), "rb") as stream:
                    if stream.getnframes() < 1 or stream.getcomptype() != "NONE":
                        raise ValueError("语音文件必须为非空 PCM WAV")
            if sys.platform != "win32" and shutil.which("aplay") is None:
                raise ValueError("Linux WAV 播放需要系统 aplay；不会自动安装")

    def play(self, prompt, now):
        if self.busy:
            raise RuntimeError("禁止重叠启动语音")
        self.prompt, self.busy = prompt, True
        if self.mode == "wav":
            path = str((self.audio_dir / (prompt + ".wav")).resolve())
            command = [sys.executable, "-c", "import sys,winsound;winsound.PlaySound(sys.argv[1],winsound.SND_FILENAME)", path] if sys.platform == "win32" else ["aplay", "-q", path]
            self.process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            self.end = now + max(1.0, len(self.prompts[prompt].split()) / 2.5)
        self.log({"event": "audio_start_requested", "monotonic_s": now, "prompt_id": prompt,
                  "text": self.prompts[prompt], "mode": self.mode, "acoustic_onset_measured": False})

    def step(self, now):
        if not self.busy:
            return
        if self.mode == "wav":
            rc = self.process.poll()
            if rc is None:
                return
            if rc != 0:
                self.busy = False
                raise RuntimeError(f"WAV 播放失败，退出码 {rc}")
            self.process = None
        elif now < self.end:
            return
        self.busy = False
        self.log({"event": "audio_completed", "monotonic_s": now, "prompt_id": self.prompt,
                  "mode": self.mode, "acoustic_end_measured": False})

    def stop(self):
        self.busy = False
        if self.process and self.process.poll() is None:
            self.process.terminate()
            self.process.wait(timeout=2)
        self.process = None


def open_client(mode, port=None):
    if mode == "virtual":
        return None
    root = Path(__file__).resolve().parents[1]
    transport = SimulationTransport(root / "build/tests" / ("simulator.exe" if sys.platform == "win32" else "simulator")) if mode == "firmware_sim" else SerialTransport(port)
    client = Client(transport, timeout=.12)
    try:
        time.sleep(.2)
        status = client.request("status")
        if status.get("protocol_version") != "head-1":
            raise ProtocolError("需要 head-1 头部固件，拒绝连接旧四轮固件")
        if status.get("state") == "ARMED":
            raise ProtocolError("设备已活动；请用原工具显式停止后重试")
        client.hello()
        return client
    except BaseException:
        transport.close()
        raise
