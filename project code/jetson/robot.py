"""Unified robot runtime. Run from project code: python -m jetson.robot --duration 10."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

from .control.arduino_bridge import ArduinoMonitor
from .core.bus import CommandInbox, StateStore
from .core.config import ROOT, load_settings
from .core.contracts import Command
from .core.engine import RobotEngine
from .core.inputs import Scenario, start_console
from .core.recording import RunRecorder, write_json
from .core.vision_process import VisionProcess
from .interface.server import StatusServer


def parser():
    p = argparse.ArgumentParser(description="Before We Approach robot runtime (virtual actuation)")
    p.add_argument("--config", type=Path, default=Path(__file__).parent / "config/simulation.json")
    p.add_argument("--duration", type=float, default=0, help="Seconds to run; 0 means until Ctrl+C or shutdown")
    p.add_argument("--scenario", type=Path, help="JSON timed command scenario")
    p.add_argument("--console", action="store_true", help="Read pause/resume/reset/quit/text <message> from stdin")
    p.add_argument("--no-web", action="store_true")
    p.add_argument("--port", type=int, help="Read-only status server port (0 chooses an available port)")
    p.add_argument("--output-dir", type=Path)
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        config = load_settings(args.config)
        if not math.isfinite(args.duration) or args.duration < 0:
            raise ValueError("duration must be finite and nonnegative")
        if args.no_web:
            config.interface.enabled = False
        if args.port is not None:
            config.interface.port = args.port
        config.validate()
        scenario = Scenario(args.scenario)
        output_root = args.output_dir if args.output_dir is not None else Path(config.runtime.output_dir)
        if not output_root.is_absolute():
            output_root = ROOT / output_root
        recorder = RunRecorder(output_root)
    except (OSError, ValueError) as exc:
        print(f"Configuration error: {exc}", file=sys.stderr)
        return 2
    started = time.monotonic()
    engine = RobotEngine(config, recorder.run_id, started)
    inbox, store = CommandInbox(), StateStore()
    monitor = ArduinoMonitor(config.arduino)
    worker = server = None
    reason, error, exit_code = "duration_complete", None, 0
    ticks = valid_ticks = target_ticks = 0
    states, faults_seen = set(), set()
    url = None
    signal_handlers = {}
    signal_requested = False

    def signal_stop(*_):
        # Signal handlers can interrupt this thread while it owns the inbox lock.
        # Only set a flag here; all queue operations remain in the ordinary loop.
        nonlocal signal_requested
        signal_requested = True

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal_handlers[sig] = signal.getsignal(sig)
            signal.signal(sig, signal_stop)
        write_json(recorder.directory / "config.json", config.to_dict())
        worker = VisionProcess(config.vision, recorder.directory)
        worker.start()
        monitor.start()
        # Publish an initial coherent state before opening the read-only interface.
        store.put(engine.step(None, worker.health(), monitor.snapshot(), [], time.monotonic()))
        if config.interface.enabled:
            server = StatusServer(config.interface, store)
            server.start()
            url = server.url
        if args.console:
            start_console(inbox, time.monotonic)
        print(json.dumps({"event": "started", "run_dir": str(recorder.directory), "status_url": url,
                          "source": config.vision.source, "physical_actuation": False}, ensure_ascii=False), flush=True)
        next_tick = next_output = started
        while True:
            now = time.monotonic()
            if signal_requested:
                inbox.post(Command.create("shutdown", now, source="signal"))
            if args.duration and now - started >= args.duration:
                break
            worker.tick(now)
            scenario.dispatch(now, started, inbox)
            packet = worker.mailbox.get()[1]
            # Sample now after reading the concurrent mailbox, avoiding a future timestamp race.
            now = time.monotonic()
            state = engine.step(packet, worker.health(), monitor.snapshot(), inbox.drain(), now)
            state["commands_dropped"] = inbox.dropped
            store.put(state)
            for event in engine.events:
                recorder.append("events", event)
                if event["type"] == "transition":
                    print(f"{event['from']} -> {event['to']}: {event['reason']}", file=sys.stderr, flush=True)
            ticks += 1
            valid_ticks += int(state["perception"]["valid"])
            target_ticks += int(state["perception"]["target"] is not None)
            states.add(state["behavior"]["state"])
            faults_seen.update(state["faults"])
            if now >= next_output:
                recorder.state(state)
                next_output = now + 1 / config.runtime.output_hz
            if engine.shutdown_requested:
                reason = "operator_shutdown"
                break
            if worker.state == "failed":
                raise RuntimeError(worker.error or "vision_process_failed")
            next_tick += 1 / config.runtime.tick_hz
            remaining = next_tick - time.monotonic()
            if remaining < 0:
                next_tick = time.monotonic()  # no burst of stale catch-up ticks
            else:
                time.sleep(remaining)
        if not valid_ticks or faults_seen:
            exit_code = 2
    except Exception as exc:
        reason, error, exit_code = "runtime_error", f"{type(exc).__name__}: {exc}", 2
        print(error, file=sys.stderr)
    finally:
        # Invalidate visible targets before any potentially slow cleanup.
        final = engine.stop(time.monotonic(), reason)
        if final is not None:
            store.put(final)
        cleanup_errors = []
        for name, closer in (("vision", worker.close if worker else None), ("arduino", monitor.close),
                             ("http", server.close if server else None)):
            if closer:
                try:
                    if closer() is False:
                        cleanup_errors.append(name + " did not stop within timeout")
                except Exception as exc:
                    cleanup_errors.append(f"{name}: {exc}")
        output_errors = []
        if final is not None:
            final["modules"]["vision"] = worker.health() if worker else {"state": "not_started"}
            final["modules"]["arduino"] = monitor.snapshot()
            states.add("STOPPED")
            try:
                if engine.previous_state != "STOPPED":
                    recorder.append("events", {"type": "transition", "from": engine.previous_state,
                                               "to": "STOPPED", "reason": reason, "at": time.monotonic()})
                recorder.state(final)
            except Exception as exc:
                output_errors.append(f"final_state: {exc}")
        try:
            recorder.close()
        except Exception as exc:
            cleanup_errors.append(f"recorder: {exc}")
        if cleanup_errors or output_errors:
            exit_code = 2
        summary = {"run_id": recorder.run_id, "at_utc": datetime.now(timezone.utc).isoformat(),
                   "duration_s": round(time.monotonic() - started, 3), "exit_code": exit_code,
                   "reason": reason, "error": error, "cleanup_errors": cleanup_errors, "output_errors": output_errors,
                   "ticks": ticks, "valid_perception_ticks": valid_ticks, "target_ticks": target_ticks,
                   "states_seen": sorted(states), "faults_seen": sorted(faults_seen),
                   "vision_restarts": worker.restarts if worker else 0, "commands_dropped": inbox.dropped,
                   "source": config.vision.source, "physical_actuation": False,
                   "hardware_acceptance_complete": False, "status_url": url}
        try:
            write_json(recorder.directory / "summary.json", summary)
        except Exception as exc:
            output_errors.append(f"summary: {exc}")
            exit_code = summary["exit_code"] = 2
        finally:
            for sig, handler in signal_handlers.items():
                signal.signal(sig, handler)
        try:
            print(json.dumps({"event": "stopped", "run_dir": str(recorder.directory), **summary}, ensure_ascii=False), flush=True)
        except OSError:
            exit_code = 2
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
