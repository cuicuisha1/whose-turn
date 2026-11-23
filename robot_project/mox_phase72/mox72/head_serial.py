"""Single-owner Arduino v2.0.0 adapter; all angles are command coordinates.

No imports open a port. STOP keeps PWM active. There is no firmware link
watchdog, encoder feedback, automatic reconnect, or automatic homing here.
"""
from __future__ import annotations

from dataclasses import replace
import math
import sys
import threading
import time
from typing import Callable

from .models import HeadSnapshot, Pose


class HeadFault(RuntimeError):
    pass


def _number(value, name):
    if isinstance(value, bool):
        raise HeadFault(f"{name}: boolean is not a number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise HeadFault(f"{name}: invalid number") from exc
    if not math.isfinite(result):
        raise HeadFault(f"{name}: non-finite number")
    return result


def _pairs(line, prefix, expected):
    if not line.startswith(prefix + " "):
        raise HeadFault(f"Expected {prefix} status line")
    result = {}
    for token in line[len(prefix) + 1:].split():
        if token.count("=") != 1:
            raise HeadFault(f"Malformed {prefix} status token")
        key, value = token.split("=", 1)
        if not value or key in result:
            raise HeadFault(f"Duplicate/empty {prefix} status token")
        result[key] = value
    if set(result) != set(expected):
        raise HeadFault(f"Unexpected {prefix} status fields")
    return result


def parse_status(lines, now, *, pan_limit=60.0, tilt_limit=10.0):
    """Parse a complete four-line STATUS response, not individual ACKs."""
    if len(lines) != 4:
        raise HeadFault("STATUS response is incomplete")
    header = _pairs(lines[0], "STATUS", {
        "version", "ready", "nominal_speed_deg_s", "ramp_done", "action",
        "config_ok", "pca_initialized", "output_fault",
    })
    if header["version"] != "2.0.0":
        raise HeadFault(f"Expected firmware 2.0.0, got {header['version']}")
    if any(header[k] != "1" for k in ("ready", "config_ok", "pca_initialized")):
        raise HeadFault("Arduino is not ready or PCA/config initialization failed")
    if header["output_fault"] != "0" or header["action"] != "NONE":
        raise HeadFault("Arduino output fault or unexpected autonomous action")
    if header["ramp_done"] not in {"0", "1"}:
        raise HeadFault("Invalid ramp_done flag")
    speed = _number(header["nominal_speed_deg_s"], "speed")
    if not 10 <= speed <= 120:
        raise HeadFault("Firmware speed outside 10..120")
    axes = []
    fields = {"command_deg", "target_deg", "last_requested_us", "limits",
              "center_us", "gain", "direction"}
    for line, name, limit in zip(lines[1:3], ("PAN", "TILT"), (pan_limit, tilt_limit)):
        values = _pairs(line, name, fields)
        bound_text = values["limits"].split("..")
        if len(bound_text) != 2:
            raise HeadFault("Invalid firmware angle limits")
        lo, hi = (_number(x, "limit") for x in bound_text)
        if not lo <= -limit < limit <= hi:
            raise HeadFault(f"{name} firmware limits do not cover host work envelope")
        command = _number(values["command_deg"], "command_deg")
        target = _number(values["target_deg"], "target_deg")
        if not -limit - .02 <= command <= limit + .02 or not -limit - .02 <= target <= limit + .02:
            raise HeadFault(f"{name} current/target outside host envelope; inspect manually")
        pulse = _number(values["last_requested_us"], "last_requested_us")
        centre = _number(values["center_us"], "center_us")
        gain = _number(values["gain"], "gain")
        if not 500 <= pulse <= 2500 or not 500 <= centre <= 2500 or gain <= 0:
            raise HeadFault("Invalid firmware pulse/calibration status")
        if values["direction"] not in {"1", "-1"}:
            raise HeadFault("Invalid axis direction")
        axes.append((command, target))
    if lines[3] != "Position feedback: NONE. ready is software readiness, not a mechanical safety check.":
        raise HeadFault("Missing firmware position-feedback disclaimer/status terminator")
    done = header["ramp_done"] == "1"
    if done and any(abs(c - t) > .025 for c, t in axes):
        raise HeadFault("ramp_done contradicts command/target values")
    return HeadSnapshot(
        _number(now, "time"), Pose(axes[0][0], axes[1][0]),
        Pose(axes[0][1], axes[1][1]), speed, done, not done,
        float(now) if done else None,
    )


def _check_goal(pose, speed, pan_limit, tilt_limit):
    if not isinstance(pose, Pose):
        raise HeadFault("move requires a Pose")
    pan = _number(pose.pan, "pan")
    tilt = _number(pose.tilt, "tilt")
    speed = _number(speed, "speed")
    if abs(pan) > pan_limit or abs(tilt) > tilt_limit:
        raise HeadFault("Requested pose exceeds host work envelope")
    if not 10 <= speed <= 120:
        raise HeadFault("Requested speed must be 10..120")
    return Pose(round(pan, 2), round(tilt, 2)), round(speed, 2)


class _Interrupted(Exception):
    pass


class SerialHead:
    """A worker is the sole serial writer after the initial read-only handshake.

    At most one pose transaction can be queued/in progress. No retry can replay
    motion after a timeout. stop() is terminal for this driver instance.
    """
    def __init__(self, port, emit, *, pan_limit=60, tilt_limit=10,
                 ack_timeout_s=.7, status_timeout_s=1.0, poll_s=.1,
                 serial_factory=None, clock=time.monotonic,
                 startup_wait_s=2.0, move_timeout_s=15.0):
        self.port, self.emit, self.clock = port, emit, clock
        self.pan_limit = _number(pan_limit, "pan_limit")
        self.tilt_limit = _number(tilt_limit, "tilt_limit")
        self.ack_timeout = _number(ack_timeout_s, "ack_timeout")
        self.status_timeout = _number(status_timeout_s, "status_timeout")
        self.poll_s = _number(poll_s, "poll_s")
        self.startup_wait = _number(startup_wait_s, "startup_wait")
        self.move_timeout = _number(move_timeout_s, "move_timeout")
        if not 0 < self.pan_limit <= 120 or not 0 < self.tilt_limit <= 60:
            raise ValueError("Invalid host angle envelope")
        if min(self.ack_timeout, self.status_timeout, self.poll_s, self.move_timeout) <= 0 or self.startup_wait < 0:
            raise ValueError("Invalid serial timing configuration")
        if self.poll_s >= self.status_timeout:
            raise ValueError("Status poll interval must be shorter than stale timeout")
        self.factory = serial_factory
        self.ser = None
        self._rx = bytearray()
        self._lock = threading.RLock()
        self._wake = threading.Event()
        self._stop_requested = threading.Event()
        self._stop_done = threading.Event()
        self._stop_confirmed = False
        self._stop_error = None
        self._closing = threading.Event()
        self._thread = None
        self._snapshot = None
        self._fault = None
        self._pending = None
        self._active = False
        self._expected = None
        self._expected_speed = None
        self._move_deadline = None
        self._counter = 0
        self._stop_reason = "operator_stop"
        self._opened = False
        self._ever_opened = False

    def open(self):
        if self._ever_opened:
            raise HeadFault("Create a new SerialHead for each session; no reconnect")
        self._ever_opened = True
        if self.factory is None:
            import serial
            self.factory = serial.Serial
        kwargs = dict(port=self.port, baudrate=115200, timeout=.02, write_timeout=.3)
        if sys.platform.startswith("linux"):
            kwargs["exclusive"] = True
        try:
            self.ser = self.factory(**kwargs)
            if self._stop_requested.wait(self.startup_wait):
                raise HeadFault("Stopped during serial startup")
            self.ser.reset_input_buffer()
            self._rx.clear()
            snapshot = self._status()
            self._snapshot = snapshot
            self._opened = True
            self.emit("HEAD_CONNECTED", time_s=self.clock(), port=self.port,
                      firmware=snapshot.version, simulated=False, position_feedback=False)
            self._thread = threading.Thread(target=self._worker, name="mox-head", daemon=True)
            self._thread.start()
        except Exception:
            self._opened = False
            if self.ser is not None:
                self.ser.close()
            raise

    def _safe_emit(self, name, **fields):
        try:
            self.emit(name, **fields)
        except Exception:
            pass

    def _tx(self, command, reason, operation_id=None, *, interruptible=True):
        if interruptible and self._stop_requested.is_set():
            raise _Interrupted()
        now = self.clock()
        fields = dict(time_s=now, command=command, reason=reason,
                      operation_id=operation_id, simulated=False)
        if command == "STOP":
            self._safe_emit("HEAD_COMMAND_ATTEMPT", **fields)
        else:
            self.emit("HEAD_COMMAND_ATTEMPT", **fields)
        if interruptible and self._stop_requested.is_set():
            raise _Interrupted()
        payload = (command + "\n").encode("ascii")
        if self.ser.write(payload) != len(payload):
            raise HeadFault(f"Incomplete serial write: {command}")
        finished = self.clock()
        fields.update(time_s=finished, write_started_at_s=now, write_completed_at_s=finished)
        if command == "STOP":
            self._safe_emit("HEAD_COMMAND_SENT", **fields)
        else:
            self.emit("HEAD_COMMAND_SENT", **fields)

    def _line(self, deadline, *, interruptible=True):
        while self.clock() < deadline:
            if interruptible and self._stop_requested.is_set():
                raise _Interrupted()
            if b"\n" in self._rx:
                line, _, rest = self._rx.partition(b"\n")
                self._rx[:] = rest
                try:
                    text = line.decode("ascii").strip()
                except UnicodeDecodeError as exc:
                    raise HeadFault("Non-ASCII Arduino response") from exc
                if text.startswith("Robot Head Controller Ready"):
                    raise HeadFault("Arduino restarted; command coordinates are no longer trusted")
                if text.startswith("ERROR") or text == "Unknown command":
                    raise HeadFault(f"Arduino rejected command: {text}")
                if text:
                    return text
                continue
            waiting = int(self.ser.in_waiting)
            if waiting:
                self._rx.extend(self.ser.read(min(waiting, 4096)))
                if len(self._rx) > 8192:
                    raise HeadFault("Arduino response exceeds bounded receive buffer")
            else:
                time.sleep(.003)
        raise HeadFault("Arduino response timeout; no command retry")

    def _request(self, command, expected, reason, operation_id=None, value=None, *, interruptible=True):
        self._tx(command, reason, operation_id, interruptible=interruptible)
        deadline = self.clock() + self.ack_timeout
        for _ in range(48):
            line = self._line(deadline, interruptible=interruptible)
            if line.startswith(expected):
                if value is not None:
                    number = _number(line[len(expected):].strip(), "acknowledged value")
                    if abs(number - value) > .025:
                        raise HeadFault(f"Numeric ACK mismatch: {command} / {line}")
                elif not (line == expected or command == "STOP" and line.startswith("Action stopped;")):
                    raise HeadFault("Malformed command acknowledgement")
                event = dict(time_s=self.clock(), command=command, response=line,
                             operation_id=operation_id, simulated=False)
                if command == "STOP":
                    self._safe_emit("HEAD_COMMAND_ACK", **event)
                else:
                    self.emit("HEAD_COMMAND_ACK", **event)
                return
            # During STOP an interrupted transaction may still have one ACK or
            # STATUS reply in flight. Drain only these known response forms.
            if command == "STOP" and line.startswith(("Speed:", "Pan target:", "Tilt target:", "STATUS ", "PAN ", "TILT ", "Position feedback:")):
                continue
            raise HeadFault(f"Unexpected Arduino reply: {line}")
        raise HeadFault("Too many unrelated Arduino responses")

    def _status(self, *, interruptible=True):
        self._tx("STATUS", "status", interruptible=interruptible)
        deadline = self.clock() + self.ack_timeout
        lines = [self._line(deadline, interruptible=interruptible) for _ in range(4)]
        return parse_status(lines, self.clock(), pan_limit=self.pan_limit, tilt_limit=self.tilt_limit)

    def _publish(self, snapshot, *, operation_complete=False):
        with self._lock:
            if self._expected is not None:
                if (abs(snapshot.target.pan - self._expected.pan) > .025 or
                        abs(snapshot.target.tilt - self._expected.tilt) > .025 or
                        abs(snapshot.speed - self._expected_speed) > .025):
                    raise HeadFault("Firmware targets/speed changed unexpectedly")
            old = self._snapshot
            settled_since = None
            if snapshot.ramp_done:
                if (old is not None and old.ramp_done and not old.busy and
                        old.target == snapshot.target and not operation_complete):
                    settled_since = old.settled_since_s
                else:
                    settled_since = snapshot.time_s
                self._move_deadline = None
            elif self._move_deadline is not None and snapshot.time_s > self._move_deadline:
                raise HeadFault("Arduino command ramp did not finish before deadline")
            if operation_complete:
                self._active = False
            self._snapshot = replace(snapshot, settled_since_s=settled_since,
                                     busy=self._active or self._pending is not None or not snapshot.ramp_done)
        self.emit("HEAD_STATUS", time_s=snapshot.time_s, command_pan=snapshot.pose.pan,
                  command_tilt=snapshot.pose.tilt, target_pan=snapshot.target.pan,
                  target_tilt=snapshot.target.tilt, ramp_done=snapshot.ramp_done,
                  nominal_speed=snapshot.speed, simulated=False)

    def _do_stop(self):
        with self._lock:
            self._pending = None
            self._active = False
            self._expected = self._expected_speed = self._move_deadline = None
        try:
            self._request("STOP", "Action stopped", self._stop_reason, interruptible=False)
        except Exception as exc:
            with self._lock:
                self._stop_error = HeadFault(f"STOP acknowledgement failed: {type(exc).__name__}: {exc}")
                self._stop_confirmed = False
            self._safe_emit("HEAD_STOP_UNCONFIRMED", time_s=self.clock(), detail=str(exc),
                            simulated=False, stop_is_power_cut=False)
            raise
        else:
            with self._lock:
                self._stop_confirmed = True
        finally:
            # Completion of the attempt is different from receiving the ACK.
            self._stop_done.set()

    def _worker(self):
        next_poll = self.clock()
        try:
            while True:
                if self._stop_requested.is_set():
                    if not self._stop_done.is_set():
                        self._do_stop()
                    if self._closing.is_set():
                        break
                    self._wake.wait(.02)
                    self._wake.clear()
                    continue
                with self._lock:
                    work = self._pending
                    if work is not None:
                        self._pending = None
                        self._active = True
                try:
                    if work is not None:
                        pose, speed, reason, number = work
                        self._request(f"SPEED {speed:.2f}", "Speed:", reason, number, speed)
                        self._request(f"P {pose.pan:.2f}", "Pan target:", reason, number, pose.pan)
                        self._request(f"T {pose.tilt:.2f}", "Tilt target:", reason, number, pose.tilt)
                        with self._lock:
                            self._expected, self._expected_speed = pose, speed
                            self._move_deadline = self.clock() + self.move_timeout
                        self._publish(self._status(), operation_complete=True)
                        next_poll = self.clock() + self.poll_s
                    elif self.clock() >= next_poll:
                        self._publish(self._status())
                        next_poll = self.clock() + self.poll_s
                    else:
                        self._wake.wait(min(.02, next_poll - self.clock()))
                        self._wake.clear()
                except _Interrupted:
                    continue
        except Exception as exc:
            with self._lock:
                self._fault = HeadFault(f"{type(exc).__name__}: {exc}")
                self._pending = None
                self._active = False
            self._safe_emit("HEAD_FAULT", time_s=self.clock(), detail=str(exc), simulated=False)
            self._stop_reason = "head_fault"
            self._stop_requested.set()
            if not self._stop_done.is_set():
                try:
                    self._do_stop()
                except Exception:
                    pass  # _do_stop retains and reports its independent failure.

    @property
    def stop_confirmed(self):
        """Whether this session received STOP's ACK; not a physical stop sensor."""
        with self._lock:
            return self._stop_confirmed

    @property
    def stop_error(self):
        with self._lock:
            return None if self._stop_error is None else str(self._stop_error)

    def move(self, pose, speed, reason):
        pose, speed = _check_goal(pose, speed, self.pan_limit, self.tilt_limit)
        with self._lock:
            if not self._opened or self._closing.is_set() or self._stop_requested.is_set():
                raise HeadFault("Head is not open or has been stopped")
            if self._fault is not None:
                raise self._fault
            if self._pending is not None or self._active or self._snapshot is None or not self._snapshot.ramp_done:
                raise HeadFault("Head is busy; wait for the current command ramp")
            if self.clock() - self._snapshot.time_s > self.status_timeout:
                raise HeadFault("Head status is stale")
            self._counter += 1
            self.emit("HEAD_MOVE_QUEUED", time_s=self.clock(), pan=pose.pan, tilt=pose.tilt,
                      speed=speed, reason=reason, operation_id=self._counter, simulated=False)
            self._pending = (pose, speed, str(reason), self._counter)
            self._snapshot = replace(self._snapshot, busy=True, settled_since_s=None)
        self._wake.set()

    def sample(self, now):
        now = _number(now, "now")
        with self._lock:
            if self._fault is not None:
                raise self._fault
            if not self._opened or self._snapshot is None:
                raise HeadFault("Head is not open")
            snapshot = self._snapshot
            # A worker can publish between the main loop taking `now` and
            # this call. A slightly newer snapshot is fresh, not a clock fault.
            if now - snapshot.time_s > self.status_timeout:
                raise HeadFault("Arduino status became stale")
            return replace(snapshot, busy=snapshot.busy or self._active or self._pending is not None)

    def stop(self, reason="operator_stop"):
        with self._lock:
            self._pending = None
            self._stop_reason = str(reason)
            self._stop_requested.set()
        self._safe_emit("HEAD_STOP_REQUESTED", time_s=self.clock(), reason=str(reason), simulated=False)
        self._wake.set()

    def close(self):
        if self.ser is None:
            return
        errors = []
        if self._thread is not None:
            self.stop("close")
            self._closing.set()
            self._wake.set()
            self._thread.join(timeout=2 * self.ack_timeout + .5)
        # Failed read-only handshakes never created a worker or accepted a
        # movement; do not misreport that as an unacknowledged session STOP.
        try:
            # Closing also unblocks a faulty transport, but cannot disable the
            # PCA9685. Preserve close errors alongside missing STOP ACKs.
            self.ser.close()
        except Exception as exc:
            errors.append(f"serial close: {type(exc).__name__}: {exc}")
        finally:
            self._opened = False
        if self._thread is not None:
            if self._thread.is_alive():
                self._thread.join(timeout=.2)
                if self._thread.is_alive():
                    errors.append("serial worker did not stop")
            with self._lock:
                if self._fault is not None:
                    errors.append(f"serial worker fault: {self._fault}")
                if not self._stop_confirmed:
                    errors.append(str(self._stop_error or "STOP acknowledgement not received"))
        self.ser = None
        if errors:
            raise HeadFault("; ".join(errors))


class SimHead:
    """Deterministic command-ramp model for offline tests; no mechanical claim."""
    def __init__(self, emit=lambda *a, **k: None, *, clock=time.monotonic,
                 pan_limit=60, tilt_limit=10):
        self.emit, self.clock = emit, clock
        self.pan_limit, self.tilt_limit = pan_limit, tilt_limit
        self._pose = self._target = Pose(0.0, 0.0)
        self._speed = 60.0
        self._last = self.clock()
        self._settled = self._last
        self._stopped = False

    def open(self):
        return None

    def sample(self, now):
        now = _number(now, "now")
        if now < self._last - 1e-9:
            raise HeadFault("Simulation clock moved backwards")
        elapsed = max(0.0, now - self._last)
        distance = max(abs(self._target.pan - self._pose.pan), abs(self._target.tilt - self._pose.tilt))
        step = self._speed * elapsed
        def advance(current, target):
            return target if abs(target-current) <= step else current + math.copysign(step, target-current)
        self._pose = Pose(advance(self._pose.pan, self._target.pan), advance(self._pose.tilt, self._target.tilt))
        done = self._pose == self._target
        if done and self._settled is None:
            self._settled = self._last + distance / self._speed
        self._last = now
        return HeadSnapshot(now, self._pose, self._target, self._speed, done, not done,
                            self._settled, simulated=True)

    def move(self, pose, speed, reason):
        pose, speed = _check_goal(pose, speed, self.pan_limit, self.tilt_limit)
        now = self.clock()
        snap = self.sample(now)
        if self._stopped or snap.busy:
            raise HeadFault("Simulated head stopped/busy")
        self._target, self._speed, self._settled = pose, speed, None
        self.emit("HEAD_MOVE_QUEUED", time_s=now, pan=pose.pan, tilt=pose.tilt,
                  speed=speed, reason=reason, simulated=True)
        for command in (f"SPEED {speed:.2f}", f"P {pose.pan:.2f}", f"T {pose.tilt:.2f}"):
            self.emit("HEAD_COMMAND_ATTEMPT", time_s=now, command=command,
                      reason=reason, simulated=True)
            self.emit("HEAD_COMMAND_SENT", time_s=now, command=command,
                      reason=reason, write_started_at_s=now, write_completed_at_s=now,
                      simulated=True)

    def stop(self, reason="operator_stop"):
        now = self.clock()
        self.sample(now)
        self._target = self._pose
        self._settled = now
        self._stopped = True
        self.emit("HEAD_STOP_REQUESTED", time_s=now, reason=reason, simulated=True)

    def close(self):
        self.stop("close")
