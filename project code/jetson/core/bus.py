from collections import deque
import copy
import threading
import time


class LatestMailbox:
    """One current state per producer; slow consumers never accumulate a backlog."""
    def __init__(self):
        self._lock = threading.Lock()
        self._value = None
        self._sequence = 0

    def put(self, value):
        with self._lock:
            self._value = copy.deepcopy(value)
            self._sequence += 1

    def get(self):
        with self._lock:
            return self._sequence, copy.deepcopy(self._value)


class CommandInbox:
    def __init__(self, capacity=32):
        self.capacity = capacity
        self._queue = deque()
        self._lock = threading.Lock()
        self.dropped = 0

    def post(self, command):
        with self._lock:
            if any(c.kind == "shutdown" for c in self._queue):
                self.dropped += 1
                return False
            if command.kind in {"pause", "shutdown"}:
                # A pending resume/text must not defeat a newer operator pause.
                self.dropped += len(self._queue)
                self._queue.clear()
            elif len(self._queue) >= self.capacity:
                self.dropped += 1
                return False
            self._queue.append(command)
            return True

    def drain(self):
        with self._lock:
            items = list(self._queue)
            self._queue.clear()
            return items


def invalidate_snapshot(state, reason):
    state["perception"]["valid"] = False
    state["perception"]["target"] = None
    state["perception"]["person_count"] = 0
    state["perception"]["reason"] = reason
    state["head"]["active"] = False
    state["head"]["pan_velocity_deg_s"] = 0.0
    state["head"]["tilt_velocity_deg_s"] = 0.0
    state["audio"]["caption"] = None
    state["audio"]["input_text"] = None
    state["audio"]["input_source"] = None
    state["audio"]["state"] = "IDLE"
    state["behavior"]["head_allowed"] = False
    return state


class StateStore(LatestMailbox):
    def read(self, now=None):
        now = time.monotonic() if now is None else now
        _, state = self.get()
        if state is None:
            return {"schema_version": 2, "system": {"status": "starting"}}
        if state["system"]["status"] != "running":
            return state
        if now - state["updated_monotonic_s"] > 1.0:
            invalidate_snapshot(state, "runtime_snapshot_expired")
            state["system"]["status"] = "stale"
            if state["behavior"]["state"] not in {"FAULT", "PAUSED"}:
                state["behavior"] = {"state": "PERCEPTION_UNAVAILABLE", "reason": "runtime_snapshot_expired", "head_allowed": False}
        elif state["perception"]["valid"] and now > state["perception_expires_at"]:
            invalidate_snapshot(state, "frame_expired")
            if state["behavior"]["state"] not in {"FAULT", "PAUSED"}:
                state["behavior"] = {"state": "PERCEPTION_UNAVAILABLE", "reason": "frame_expired", "head_allowed": False}
        return state
