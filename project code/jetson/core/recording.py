"""Bounded per-run diagnostics and an atomically replaced latest snapshot."""
from datetime import datetime, timezone
import json
import logging
from logging.handlers import RotatingFileHandler
import os
import sys
from pathlib import Path
import uuid


def write_json(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(temporary, path)


class StrictRotatingFileHandler(RotatingFileHandler):
    def handleError(self, record):
        # The default logging handler swallows disk errors. Runtime evidence must
        # either be recorded or fail explicitly so main() can perform cleanup.
        error = sys.exc_info()[1]
        if error is not None:
            raise error
        raise OSError("Could not write runtime log")


class RunRecorder:
    def __init__(self, root):
        self.run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
        self.directory = Path(root) / self.run_id
        self.directory.mkdir(parents=True)
        self.handlers = {name: StrictRotatingFileHandler(self.directory / (name + ".jsonl"), maxBytes=2_000_000,
                                                  backupCount=2, encoding="utf-8")
                         for name in ("state", "events")}

    def append(self, name, value):
        self.handlers[name].emit(logging.LogRecord(name, logging.INFO, "", 0,
                                                  json.dumps(value, ensure_ascii=False, allow_nan=False), (), None))

    def state(self, value):
        self.append("state", value)
        write_json(self.directory / "latest.json", value)

    def close(self):
        for handler in self.handlers.values():
            handler.close()
