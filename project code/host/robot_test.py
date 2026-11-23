#!/usr/bin/env python3
"""Head-only USB CDC console; no chassis commands or automatic replay."""
from __future__ import annotations
import argparse
import json
import queue
import secrets
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools" if (ROOT / "tools").exists() else ROOT.parent / "tools"
if (TOOLS / "python").exists():
    sys.path.insert(0, str(TOOLS / "python"))
ACTIVE = {"ARMED"}


class ProtocolError(RuntimeError):
    pass


class CommandRejected(ProtocolError):
    def __init__(self, reply, state):
        self.reply = reply
        super().__init__(f"Rejected: {reply.get('error')}; state={state}, faults={reply.get('faults')}")


class LineDecoder:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        records = []
        while b"\n" in self.buffer:
            raw, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            if not raw.strip():
                continue
            if len(raw) > 2048:
                raise ProtocolError("Response exceeds 2048 bytes")
            try:
                obj = json.loads(raw, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            except (ValueError, UnicodeError) as exc:
                raise ProtocolError("Invalid JSON response") from exc
            if not isinstance(obj, dict) or obj.get("type") not in {"ack", "status"}:
                raise ProtocolError("Unexpected response schema")
            records.append(obj)
        if len(self.buffer) > 2048:
            raise ProtocolError("Unterminated response exceeds limit")
        return records


class SerialTransport:
    def __init__(self, port: str):
        import serial
        self.serial = serial.Serial(port, 115200, timeout=0, write_timeout=0.1)

    def write(self, data: bytes):
        if self.serial.write(data) != len(data):
            raise OSError("Short serial write")

    def read(self):
        return self.serial.read(min(4096, self.serial.in_waiting))

    def close(self):
        self.serial.close()


class SimulationTransport:
    def __init__(self, exe: Path):
        self.process = subprocess.Popen([str(exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.received = queue.Queue()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in iter(self.process.stdout.readline, b""):
            self.received.put(line)
        self.received.put(None)

    def write(self, data):
        if self.process.poll() is not None:
            raise OSError("Simulator exited")
        self.process.stdin.write(data)
        self.process.stdin.flush()

    def read(self):
        try:
            data = self.received.get_nowait()
        except queue.Empty:
            return b""
        if data is None:
            raise OSError("Simulator disconnected")
        return data

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                self.process.wait(timeout=1)
        if not self.process.stdin.closed:
            self.process.stdin.close()
        self.reader.join(timeout=1)
        self.process.stdout.close()



class Client:
    def __init__(self, transport, timeout=0.1):
        self.transport = transport
        self.timeout = timeout
        self.decoder = LineDecoder()
        self.session = secrets.randbelow(0xFFFFFFFF) + 1
        self.sequence = 0
        self.state = "UNKNOWN"
        self.epoch = 0
        self.status = {}
        self.responses = {}
        self.last_heartbeat = time.monotonic()
        self.connected_session = False
        self.events = []

    def pump(self):
        for _ in range(32):
            data = self.transport.read()
            if not data:
                break
            for obj in self.decoder.feed(data):
                if "state" in obj:
                    self.state = obj["state"]
                    self.epoch = obj.get("arm_epoch", self.epoch)
                if obj["type"] == "status":
                    self.status = obj
                    key = obj.get("reply_to", 0)
                else:
                    key = obj.get("sequence", 0)
                if key:
                    self.responses[key] = obj
            if len(self.responses) > 16:
                raise ProtocolError("Too many unmatched responses")

    def request(self, kind, **fields):
        self.sequence += 1
        if self.sequence > 0xFFFFFFFF:
            raise ProtocolError("Sequence exhausted; stop and establish a fresh session")
        seq = self.sequence
        packet = {"type": kind, "sequence": seq}
        if kind not in {"status", "stop"}:
            packet["session"] = self.session
        packet.update(fields)
        raw = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("ascii")
        if len(raw) > 384:
            raise ValueError("Frame too long")
        self.transport.write(raw + b"\n")
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            self.pump()
            if seq in self.responses:
                reply = self.responses.pop(seq)
                if reply.get("ok") is False:
                    raise CommandRejected(reply, self.state)
                return reply
            time.sleep(0.001)
        # Never resend a command after an ambiguous ACK timeout.
        raise TimeoutError(f"No acknowledgement for {kind}; no retry was sent")

    def hello(self):
        reply = self.request("hello")
        self.connected_session = True
        return reply

    def maintain(self):
        self.pump()
        now = time.monotonic()
        if self.connected_session and self.state in ACTIVE and now-self.last_heartbeat >= 0.05:
            try:
                self.request("heartbeat")
            except CommandRejected as exc:
                # An emergency/timeout can occur between the last telemetry and
                # this heartbeat. Remain available for status and manual clear.
                if self.state not in {"FAULT", "DISARMED"}:
                    raise
                self.events.append(str(exc))
            self.last_heartbeat = time.monotonic()

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.maintain()
            time.sleep(0.002)

    def best_effort_stop(self):
        try:
            self.transport.write(b'{"type":"stop"}\n')
        except (OSError, ValueError):
            pass

    def simulate(self, kind):
        if not isinstance(self.transport, SimulationTransport):
            raise ValueError("Simulation inputs are unavailable on real hardware")
        self.transport.write(json.dumps({"type": kind}).encode() + b"\n")


HELP = """status | clear | prepare bench/head | servo PAN_US TILT_US
stop | wdt-test outputs_off | quit
Simulation only: sim-servo-fail | sim-servo-restore
prepare permits later head commands without starting PWM.
No auto clear/reconnect. stop disables PWM; it does not cut servo power."""


def console(client):
    print(HELP)
    commands = queue.Queue()

    def read_console():
        try:
            while True:
                line = input("robot> ")
                commands.put(line)
        except EOFError:
            commands.put("quit")

    threading.Thread(target=read_console, daemon=True).start()
    while True:
        client.maintain()
        while client.events:
            print(client.events.pop(0), file=sys.stderr)
        try:
            line = commands.get_nowait()
        except queue.Empty:
            time.sleep(0.002)
            continue
        try:
            words = shlex.split(line)
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            continue
        if not words:
            continue
        try:
            cmd = words[0]
            if cmd == "quit":
                return
            if cmd == "status":
                reply = client.request("status")
            elif cmd == "clear":
                reply = client.request("clear_fault")
            elif cmd == "prepare" and len(words) == 2 and words[1] in {"bench", "head"}:
                reply = client.request("prepare", mode=words[1])
            elif cmd == "servo" and len(words) == 3:
                reply = client.request("servo", pan_us=int(words[1]), tilt_us=int(words[2]), arm_epoch=client.epoch)
            elif cmd == "stop":
                reply = client.request("stop")
            elif cmd == "wdt-test" and words[1:] == ["outputs_off"]:
                reply = client.request("watchdog_test", confirm="outputs_off")
            elif cmd.startswith("sim-"):
                client.simulate(cmd.replace("-", "_"))
                reply = {"simulation_input": cmd}
            else:
                print(HELP)
                continue
            print(json.dumps(reply, ensure_ascii=False))
        except (ValueError, ProtocolError) as exc:
            # A rejected command is visible. Firmware decides any active fault;
            # do not automatically clear it or resume a previously requested move.
            print(str(exc), file=sys.stderr)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="COM5 or /dev/serial/by-id/...")
    parser.add_argument("--simulate", action="store_true", help="Native fake HAL, no hardware")
    parser.add_argument("action", choices=["ports", "status", "stop", "console"], nargs="?", default="console")
    args = parser.parse_args(argv)
    if args.action == "ports":
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(p.device, p.description, p.hwid)
        return 0
    if args.simulate and args.port:
        parser.error("Use either --port or --simulate")
    if not args.simulate and not args.port:
        parser.error("Specify --port or --simulate")
    if args.simulate:
        exe = ROOT / "build/tests" / ("simulator.exe" if sys.platform == "win32" else "simulator")
        print("SIMULATION ONLY; head PWM and hardware are simulated.", file=sys.stderr)
        transport = SimulationTransport(exe)
    else:
        transport = SerialTransport(args.port)
    client = Client(transport)
    try:
        # Opening USB can reset an Arduino. Wait briefly, then request status;
        # never issue a reset or replay a previous motion automatically.
        time.sleep(0.15)
        if args.action in {"status", "stop"}:
            print(json.dumps(client.request(args.action), ensure_ascii=False, indent=2))
        else:
            status = client.request("status")
            if status.get("protocol_version") != "head-1":
                raise ProtocolError("Install head-1 firmware before using this head-only console")
            if client.state in ACTIVE:
                raise ProtocolError("Controller already active; use stop explicitly before starting a new session")
            client.hello()
            console(client)
        return 0
    except (OSError, TimeoutError, ProtocolError, KeyboardInterrupt) as exc:
        print(f"Session ended: {exc}", file=sys.stderr)
        return 1
    finally:
        if args.action == "console":
            client.best_effort_stop()
        transport.close()


if __name__ == "__main__":
    raise SystemExit(main())
