"""Bounded operator commands and reproducible, time-based integration scenarios."""
import json
from pathlib import Path
import sys
import threading

from .contracts import Command, finite


class Scenario:
    def __init__(self, path=None):
        self.events, self.position = [], 0
        if path is None:
            return
        data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        if not isinstance(data, list) or len(data) > 1000:
            raise ValueError("Scenario must be a list with at most 1000 commands")
        last = -1
        for item in data:
            if not isinstance(item, dict) or set(item) - {"at_s", "kind", "text"}:
                raise ValueError("Unknown scenario field")
            at = item.get("at_s")
            if not finite(at) or at < 0 or at < last:
                raise ValueError("Scenario times must be finite, nonnegative and sorted")
            command = Command.create(item.get("kind"), at, item.get("text", ""), "scenario")
            self.events.append(command)
            last = at

    def dispatch(self, now, started_at, inbox):
        while self.position < len(self.events):
            event = self.events[self.position]
            if now - started_at < event.created_at:
                break
            inbox.post(Command.create(event.kind, started_at + event.created_at, event.text, "scenario"))
            self.position += 1


def start_console(inbox, clock):
    def run():
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            kind, _, text = line.partition(" ")
            if kind == "quit":
                kind = "shutdown"
            try:
                command = Command.create(kind, clock(), text)
            except ValueError as exc:
                print(str(exc), file=sys.stderr)
                continue
            if not inbox.post(command):
                print("Command rejected: queue full or shutdown pending", file=sys.stderr)
            if kind == "shutdown":
                break
    thread = threading.Thread(target=run, name="operator-console", daemon=True)
    thread.start()
    return thread
