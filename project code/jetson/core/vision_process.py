"""Supervise the existing vision CLI as an isolated child process."""
import json
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

from .bus import LatestMailbox
from .config import ROOT


class VisionProcess:
    def __init__(self, config, directory):
        self.config, self.directory = config, Path(directory)
        self.mailbox = LatestMailbox()
        self.process = None
        self.threads = []
        self.restarts = 0
        self.last_packet = 0.0
        self.error = None
        self.state = "starting"
        self.next_start = 0.0
        self.reader_error = None
        self.log = RotatingFileHandler(self.directory / "vision_worker.log", maxBytes=2_000_000,
                                      backupCount=2, encoding="utf-8")

    def _command(self):
        c = self.config
        command = [sys.executable, "-u", "-m", "jetson.main", "--stdout", "--detector", c.detector,
                   "--fps", str(c.fps), "--detection-hz", str(c.detection_hz), "--output-hz", "10",
                   "--max-frame-age-ms", str(c.max_frame_age_ms), "--output-dir", str(self.directory / "vision")]
        if c.source == "simulation":
            command.append("--simulate")
        if c.serial:
            command += ["--serial", c.serial]
        if c.model:
            command += ["--model", c.model, "--device", c.device]
        return command

    def start(self):
        self._launch()

    def _launch(self):
        self.mailbox.put(None)
        self.reader_error = self.error = None
        self.last_packet = time.monotonic()
        try:
            self.process = subprocess.Popen(self._command(), cwd=ROOT, stdin=subprocess.DEVNULL,
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                            env={**os.environ, "PYTHONIOENCODING": "utf-8"},
                                            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
            self.state = "running"
            self.threads = [threading.Thread(target=self._read, args=(self.process.stdout,), daemon=True),
                            threading.Thread(target=self._logs, args=(self.process.stderr,), daemon=True)]
            for thread in self.threads:
                thread.start()
        except OSError as exc:
            self.error = str(exc)
            self.state = "failed"

    def _read(self, pipe):
        try:
            while True:
                line = pipe.readline(262145)
                if not line:
                    break
                if len(line) > 262144 or not line.endswith(b"\n"):
                    raise ValueError("Oversized or truncated vision record")
                packet = json.loads(line, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
                if not isinstance(packet, dict):
                    raise ValueError("Vision record must be an object")
                self.mailbox.put(packet)
                self.last_packet = time.monotonic()
        except (ValueError, OSError) as exc:
            self.reader_error = str(exc)
        finally:
            # A dead producer cannot leave a valid current target behind.
            self.mailbox.put(None)

    def _logs(self, pipe):
        try:
            while True:
                line = pipe.readline(4096)
                if not line:
                    break
                self.log.emit(logging.LogRecord("vision-child", logging.INFO, "", 0,
                                                 line.decode("utf-8", "replace").rstrip(), (), None))
        except OSError:
            return

    def tick(self, now):
        if self.state in {"failed", "stopped"}:
            return
        if self.state == "restarting":
            if now >= self.next_start:
                self._launch()
            return
        if self.process is None:
            return
        if self.process.poll() is None and (self.reader_error or now - self.last_packet > self.config.stall_timeout_s):
            self.error = self.reader_error or "vision_process_silent"
            self.mailbox.put(None)
            self.process.kill()
        code = self.process.poll()
        if code is not None:
            self.error = self.error or self.reader_error or f"vision_process_exited:{code}"
            self._reap()
            self.mailbox.put(None)
            if self.restarts < self.config.max_restarts:
                self.restarts += 1
                self.next_start = now + min(3, .5 * 2 ** (self.restarts - 1))
                self.state = "restarting"
            else:
                self.state = "failed"

    def _reap(self):
        for thread in self.threads:
            thread.join(timeout=1)
        if self.process:
            self.process.stdout.close()
            self.process.stderr.close()
        self.process = None
        self.threads = []

    def health(self):
        return {"state": self.state, "pid": self.process.pid if self.process else None,
                "restarts": self.restarts, "error": self.error,
                "last_message_monotonic_s": self.last_packet}

    def close(self):
        if self.process:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
            self._reap()
        self.state = "stopped"
        self.mailbox.put(None)
        self.log.close()
