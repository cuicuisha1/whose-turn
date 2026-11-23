import threading
import time

from ..core.bus import LatestMailbox


class ArduinoMonitor:
    """One serial owner, status-only. Does not establish a motion session or send heartbeat."""
    def __init__(self, config, client_factory=None):
        self.config = config
        self.client_factory = client_factory
        self.mailbox = LatestMailbox()
        self.mailbox.put({"mode": config.mode, "connected": False, "received_at": None,
                          "status": None, "error": None})
        self.stop_event = threading.Event()
        self.thread = None
        self.close_error = None

    def start(self):
        if self.config.mode == "disabled":
            return
        self.thread = threading.Thread(target=self._run, name="arduino-monitor", daemon=True)
        self.thread.start()

    def _run(self):
        client = None
        try:
            if self.client_factory is None:
                from host.robot_test import Client, SerialTransport
                client = Client(SerialTransport(self.config.port), timeout=self.config.timeout_s)
            else:
                client = self.client_factory()
            if self.stop_event.wait(.2):
                return
            while not self.stop_event.is_set():
                response = client.request("status")
                if (response.get("type") != "status" or not isinstance(response.get("state"), str)
                        or type(response.get("faults")) is not int or response["faults"] < 0):
                    raise ValueError("Invalid Arduino status response")
                self.mailbox.put({"mode": "monitor", "connected": True, "received_at": time.monotonic(),
                                  "status": response, "error": None})
                self.stop_event.wait(1 / self.config.poll_hz)
        except Exception as exc:
            self.mailbox.put({"mode": "monitor", "connected": False, "received_at": None,
                              "status": None, "error": str(exc)})
            # No automatic reconnect, clear_fault, prepare, motion or stop.
        finally:
            state = self.mailbox.get()[1]
            if client:
                try:
                    client.transport.close()
                except Exception as exc:
                    self.close_error = f"serial_close_failed: {exc}"
                    state["error"] = self.close_error
            state.update(connected=False, received_at=None, status=None)
            self.mailbox.put(state)

    def snapshot(self):
        state = self.mailbox.get()[1]
        if self.stop_event.is_set():
            state.update(connected=False, received_at=None, status=None)
        return state

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                return False
        return self.close_error is None
