#!/usr/bin/env python3
"""MOX Phase 7.2 engineering entry point. Default mode opens no hardware."""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import importlib.util
import json
import math
import os
from pathlib import Path
import queue
import re
import shutil
import sys
import threading
import time

from mox72 import __version__
from mox72.config import load_config
from mox72.engine import Engine
from mox72.head_serial import SerialHead
from mox72.runtime_io import Journal, AudioCapture, WavePlayer, file_sha256, wav_info
from mox72.simulation import SimClock, run_simulation
from mox72.vision import VisionCamera

BUNDLE = Path(__file__).absolute().parent
PROJECT = BUNDLE.parent


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--version", action="version", version=f"MOX Phase 7.2 {__version__}")
    p.add_argument("--mode", choices=("check", "simulate", "vision", "live"), default="check")
    p.add_argument("--scope", choices=("a", "full"), default="a",
                   help="a: welcome/stability/centre A only; full: complete experiment rehearsal")
    p.add_argument("--condition", choices=("fixed", "pause"))
    p.add_argument("--scenario", choices=("silence", "speech", "missing_b", "moving"), default="silence",
                   help="Synthetic input; only used in simulate mode")
    p.add_argument("--config", type=Path, default=BUNDLE / "examples/engineering.json")
    p.add_argument("--tilt-sign", type=int, choices=(-1, 1),
                   help="Verified image-down error to Tilt command sign; required for live motion")
    p.add_argument("--port", help="Exact Arduino port, preferably /dev/serial/by-id/...; required for live")
    p.add_argument("--trial-id")
    p.add_argument("--pair-id")
    p.add_argument("--model", type=Path, default=PROJECT / "models/face_detection_yunet_2023mar.onnx")
    p.add_argument("--camera-serial", default="242322076252")
    p.add_argument("--welcome-wav", type=Path, default=BUNDLE / "assets/welcome.wav")
    p.add_argument("--prompt-wav", type=Path, default=BUNDLE / "assets/prompt.wav")
    p.add_argument("--audio-input", default="plughw:CARD=Array,DEV=0")
    p.add_argument("--audio-output", default="plughw:CARD=UACDemoV10,DEV=0")
    p.add_argument("--channel", type=int, choices=(0, 1), default=0)
    p.add_argument("--vad-mode", type=int, choices=(0, 1, 2, 3), default=2)
    p.add_argument("--gate-dbfs", type=float, default=-48.0)
    p.add_argument("--output", type=Path, default=PROJECT / "camera_test/phase72")
    p.add_argument("--preview", action="store_true", help="Show camera image on a Jetson graphical desktop")
    p.add_argument("--duration", type=float, default=30.0, help="Vision-only duration in seconds")
    return p


def say(value):
    print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)


def identifier(value, name, required=False):
    if required and not value:
        raise ValueError(f"--{name} is required for a live attempt")
    value = value or ("demo-trial" if name == "trial-id" else "demo-pair")
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", value):
        raise ValueError(f"--{name}: use 1..80 letters, digits, underscores or hyphens")
    return value


def requirements(args, *, vision_only=False):
    """Inspect module availability and files, without importing device modules."""
    modules = ["cv2", "numpy", "pyrealsense2"]
    commands = []
    clips = {}
    if not vision_only:
        modules.append("serial")
        commands.append("aplay")
        clips["welcome"] = args.welcome_wav
        if args.scope == "full":
            modules.append("webrtcvad")
            commands.append("arecord")
            clips["prompt"] = args.prompt_wav
    problems, info = [], {"python": sys.executable, "python_version": sys.version,
                           "modules": {}, "commands": {}, "assets": {}}
    if args.preview and sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        problems.append("--preview 需要图形桌面；SSH 终端请去掉该参数")
    for name in modules:
        present = importlib.util.find_spec(name) is not None
        info["modules"][name] = present
        if not present:
            problems.append(f"当前 Python 找不到模块 {name}；请使用项目原有 .venv")
    for name in commands:
        path = shutil.which(name)
        info["commands"][name] = path
        if path is None:
            problems.append(f"找不到系统命令 {name}")
    if not args.model.is_file():
        problems.append(f"找不到 YuNet 模型：{args.model}")
    else:
        info["model"] = {"path": str(args.model.absolute()), "sha256": file_sha256(args.model)}
    for name, path in clips.items():
        try:
            info["assets"][name] = wav_info(path)
        except Exception as exc:
            problems.append(f"{name}: {exc}；先执行 prepare_audio.py")
    info["problems"] = problems
    return info


def code_hashes():
    return {str(path.relative_to(BUNDLE)): file_sha256(path)
            for path in sorted(BUNDLE.rglob("*.py")) if "__pycache__" not in path.parts}


def get_metadata(args, cfg, trial_id, pair_id, preflight):
    return {"software_version": __version__, "engineering_validation": True,
            "mode": args.mode, "scope": args.scope, "condition": args.condition,
            "trial_id": trial_id, "pair_id": pair_id,
            "simulated": args.mode == "simulate", "motion_simulated": args.mode != "live",
            "motion_enabled": args.mode == "live", "position_feedback": False,
            "config": cfg.to_dict(), "config_sha256": cfg.sha256,
            "runtime": {"port": args.port, "camera_serial": args.camera_serial,
                        "audio_input": args.audio_input, "audio_output": args.audio_output,
                        "channel": args.channel, "vad_mode": args.vad_mode,
                        "gate_dbfs": args.gate_dbfs, "preview": args.preview,
                        "python": sys.executable},
            "preflight": preflight, "source_sha256": code_hashes(),
            "analysis_window_s": cfg.observation_window_s,
            "analysis_window_origin": "external_video_T0_not_available",
            "raw_audio_recorded": False, "raw_images_recorded": False}


class Console:
    """Read-only input thread; commands run on the controller thread."""
    def __init__(self):
        self.items = queue.Queue(maxsize=16)
        self.overflow = threading.Event()
        self.thread = threading.Thread(target=self._read, name="mox-console", daemon=True)

    def _read(self):
        try:
            for line in sys.stdin:
                text = line.strip().upper()
                if text:
                    try:
                        self.items.put_nowait(text)
                    except queue.Full:
                        self.overflow.set()
                        return
        finally:
            try:
                self.items.put_nowait("QUIT")
            except queue.Full:
                self.overflow.set()

    def start(self):
        self.thread.start()

    def drain(self):
        if self.overflow.is_set():
            raise RuntimeError("CONSOLE_INPUT_OVERFLOW")
        result = []
        while True:
            try:
                result.append(self.items.get_nowait())
            except queue.Empty:
                return result


def show_preview(camera, state):
    import cv2
    frame = camera.preview_frame()
    if frame is not None:
        cv2.putText(frame, state, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    .65, (0, 255, 255), 2)
        cv2.imshow("MOX Phase 7.2 - q stops", frame)
    return cv2.waitKey(1) & 0xff == ord("q")


def log_face(emit, face):
    fields = asdict(face)
    fields["face_sequence"] = fields.pop("sequence")
    emit("VISION_OBSERVATION", controller_consumed=True, simulated=False, **fields)


def run_live(args, cfg, journal):
    emit = journal.emit
    camera = audio = head = player = engine = None
    cleanup_errors, failure = [], None
    snapshot = {"state": "STARTUP", "fault": None}
    try:
        clips = {"welcome": args.welcome_wav}
        if args.scope == "full":
            clips["prompt"] = args.prompt_wav
        # Validate clips before creating/opening any hardware interface.
        player = WavePlayer(clips, emit, journal.directory, device=args.audio_output)
        camera = VisionCamera(args.model, emit, camera_serial=args.camera_serial, preview=args.preview)
        camera.start()
        start = time.monotonic()
        while not camera.ready:
            camera.latest(time.monotonic())
            if time.monotonic() - start > 6:
                raise RuntimeError("CAMERA_STARTUP_TIMEOUT")
            time.sleep(.02)
        head = SerialHead(args.port, emit, pan_limit=60, tilt_limit=cfg.tilt_limit,
                          move_timeout_s=cfg.move_timeout_s)
        head.open()  # No app movement here; board opening/reset behaviour is hardware dependent.
        if args.scope == "full":
            audio = AudioCapture(emit, journal.directory, device=args.audio_input,
                                 channel=args.channel, vad_mode=args.vad_mode, min_dbfs=args.gate_dbfs)
            audio.start()
            start = time.monotonic()
            while not audio.ready:
                camera.latest(time.monotonic())
                audio.drain(time.monotonic())
                if time.monotonic() - start > 5.5:
                    raise RuntimeError("AUDIO_STARTUP_TIMEOUT")
                time.sleep(.01)
            # Discard startup frames explicitly; none can contribute to JOIN.
            discarded = audio.drain(time.monotonic())
            emit("AUDIO_STARTUP_DRAINED", count=len(discarded))
        console = Console()
        console.start()
        # The serial handshake took time; recheck sensors before the first move.
        camera.latest(time.monotonic())
        head.sample(time.monotonic())
        if audio is not None:
            discarded = audio.drain(time.monotonic())
            emit("AUDIO_PRE_BEGIN_DRAINED", count=len(discarded))
        engine = Engine(cfg, args.condition or "pause", head, player, emit, a_only=args.scope == "a")
        engine.begin(time.monotonic())
        print("Commands (type the whole word, then Enter): JOIN STOP END STATUS QUIT.", flush=True)
        print("STOP / QUIT / Ctrl-C abort this attempt. END completes observation only in OBSERVING; do not use Ctrl-Z to pause a trial.", flush=True)
        print("Full mode: prompt and A centering overlap; wait for ready_for_join=true, then mark B arrival with JOIN. A_READY ends A-only validation.", flush=True)
        print("During centering, a moving or briefly lost face can enter bounded recovery; watch center_recovering and its reason in STATUS. A locked pose stays fixed for this trial.", flush=True)
        print("W is based on external video T0, not the time JOIN was entered.", flush=True)
        last_state, last_report, last_preview, last_face_sequence = None, 0.0, 0.0, None
        while not engine.terminal:
            commands = console.drain()
            emergency = next((item for item in commands if item in {"STOP", "QUIT"}), None)
            if emergency is not None:
                engine.command(emergency, time.monotonic())
                break
            face = camera.latest(time.monotonic())
            items = audio.drain(time.monotonic()) if audio else []
            if face is not None and face.sequence != last_face_sequence:
                log_face(emit, face)
                last_face_sequence = face.sequence
            # Sample now AFTER reading asynchronous publications.
            now = time.monotonic()
            engine.tick(now, face, items)
            for command in commands:
                accepted = engine.command(command, time.monotonic())
                if command == "STATUS" or not accepted:
                    say({"command": command, "accepted": accepted, **engine.snapshot()})
                if engine.terminal:
                    break
            if args.preview and not engine.terminal and now - last_preview >= 1 / 15:
                last_preview = now
                if show_preview(camera, engine.state):
                    engine.command("QUIT", time.monotonic())
            if engine.state != last_state or now - last_report >= 1:
                last_state, last_report = engine.state, now
                report = engine.snapshot()
                if face is not None:
                    report["face"] = {"count": face.count, "point_m": face.point_m,
                                      "horizontal_deg": face.horizontal_deg,
                                      "vertical_deg": face.vertical_deg, "status": face.error}
                say(report)
            time.sleep(.01)
        snapshot = engine.snapshot()
    except KeyboardInterrupt:
        failure = "keyboard_interrupt"
    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"
    finally:
        if failure:
            if engine is not None:
                try:
                    engine.fail(failure, time.monotonic())
                except Exception as exc:
                    cleanup_errors.append(f"engine_abort: {exc}")
            snapshot = engine.snapshot() if engine is not None else {"state": "ABORTED", "fault": failure}
        # STOP request is first, before potentially slower device cleanup.
        if head is not None:
            try:
                head.stop("session_end")
            except Exception as exc:
                cleanup_errors.append(f"head_stop: {exc}")
        for name, obj in (("player", player), ("head", head), ("audio", audio), ("camera", camera)):
            if obj is not None:
                try:
                    obj.close()
                except Exception as exc:
                    cleanup_errors.append(f"{name}: {exc}")
        if args.preview:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception as exc:
                cleanup_errors.append(f"preview: {exc}")
    return {**snapshot, "simulated": False, "motion_simulated": False,
            "input_source": "live_camera_and_audio" if args.scope == "full" else "live_camera",
            "runtime_fault": failure, "cleanup_errors": cleanup_errors,
            "stop_acknowledged": getattr(head, "stop_confirmed", False) if head else None,
            "stop_error": getattr(head, "stop_error", None) if head else None,
            "logging_failed": journal.fault is not None}


def run_vision(args, journal):
    camera = VisionCamera(args.model, journal.emit, camera_serial=args.camera_serial, preview=args.preview)
    failure, cleanup, frames, last_sequence = None, [], 0, None
    try:
        camera.start()
        started, reported = time.monotonic(), 0.0
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            face = camera.latest(now)
            if face is not None and face.sequence != last_sequence:
                frames += 1
                last_sequence = face.sequence
                log_face(journal.emit, face)
            if face is not None and now - reported >= .5:
                reported = now
                say({"mode": "vision", "face": asdict(face), "motion_enabled": False})
            if args.preview and show_preview(camera, "VISION ONLY - no head commands"):
                break
            time.sleep(.02)
    except KeyboardInterrupt:
        failure = "keyboard_interrupt"
    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            camera.close()
        except Exception as exc:
            cleanup.append(str(exc))
        if args.preview:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception as exc:
                cleanup.append(str(exc))
    return {"state": "ABORTED" if failure or cleanup else "COMPLETED",
            "simulated": False, "motion_simulated": True, "motion_enabled": False,
            "input_source": "live_camera", "observations": frames,
            "fault": failure, "cleanup_errors": cleanup, "logging_failed": journal.fault is not None}


def main(argv=None):
    args = parser().parse_args(argv)
    journal = None
    try:
        cfg = load_config(args.config)
        if args.tilt_sign is not None:
            cfg = replace(cfg, tilt_sign=args.tilt_sign)
        if not math.isfinite(args.duration) or not 1 <= args.duration <= 600:
            raise ValueError("--duration must be within 1..600 seconds")
        if not math.isfinite(args.gate_dbfs) or not -180 <= args.gate_dbfs <= 0:
            raise ValueError("--gate-dbfs must be finite and within -180..0")
        live = args.mode == "live"
        trial_id = identifier(args.trial_id, "trial-id", live)
        pair_id = identifier(args.pair_id, "pair-id", live)
        if live:
            cfg.validate_live()
            if not args.port:
                raise ValueError("Live mode needs --port; no serial port is guessed")
            if args.scope == "full" and args.condition is None:
                raise ValueError("Full live mode needs --condition fixed or pause")
            if max(abs(cfg.pan_a_base), abs(cfg.pan_b_base)) + cfg.pan_correction_limit > 60:
                raise ValueError("This initial live entry uses the tested Pan +/-60 envelope")
            if not sys.stdin.isatty():
                raise ValueError("Live mode requires an interactive terminal for JOIN/STOP/END; do not pipe stdin")
        if args.mode == "simulate" and cfg.tilt_sign is None:
            cfg = replace(cfg, tilt_sign=1)
            print("SIMULATION ONLY: tilt_sign=+1 is synthetic, not a measurement of your robot.", flush=True)
        args.condition = args.condition or "pause"
        preflight = {} if args.mode == "simulate" else requirements(args, vision_only=args.mode == "vision")
        if args.mode == "check":
            say({"mode": "check", "hardware_opened": False, "config_valid": True,
                 "tilt_direction_verified_in_config": cfg.tilt_sign is not None,
                 "scope": args.scope, **preflight})
            return 2 if preflight["problems"] else 0
        if preflight.get("problems"):
            say({"state": "PREFLIGHT_FAILED", "hardware_opened": False, **preflight})
            return 2
        sim_clock = SimClock() if args.mode == "simulate" else None
        journal = Journal(args.output, trial_id, pair_id, clock=sim_clock or time.monotonic)
        metadata = get_metadata(args, cfg, trial_id, pair_id, preflight)
        journal.write_json("config.json", metadata)
        journal.emit("SESSION_BEGIN", software_version=__version__, mode=args.mode,
                     engineering_validation=True, simulated=args.mode == "simulate",
                     motion_simulated=args.mode != "live")
        print(f"Run directory: {journal.directory}", flush=True)
        if args.mode == "simulate":
            summary = run_simulation(cfg, args.condition, args.scope, args.scenario,
                                     journal.emit, clock=sim_clock, progress=say)
        elif args.mode == "vision":
            summary = run_vision(args, journal)
        else:
            summary = run_live(args, cfg, journal)
        summary.update(software_version=__version__, trial_id=trial_id, pair_id=pair_id,
                       engineering_validation=True, mode=args.mode, scope=args.scope,
                       config_sha256=cfg.sha256, output_directory=str(journal.directory))
        healthy = (summary["state"] in {"COMPLETED", "A_READY"}
                   and not summary.get("cleanup_errors") and not summary.get("logging_failed")
                   and not summary.get("runtime_fault"))
        summary["engineering_run_ok"] = healthy
        try:
            journal.emit("SESSION_END", state=summary["state"], engineering_run_ok=healthy)
        except Exception as exc:
            summary.update(logging_failed=True, engineering_run_ok=False, session_end_log_error=str(exc))
            healthy = False
        try:
            journal.write_json("face_targets.json", {
                "trial_id": trial_id, "pair_id": pair_id, "config_sha256": cfg.sha256,
                "software_version": __version__,
                "command_pose_a": summary.get("command_pose_a"),
                "command_pose_b": summary.get("command_pose_b"),
                "centering_quality": summary.get("centering_quality"),
                "station_compliance": "requires_external_review",
                "same_person_verified": False,
                "completed_state": summary["state"], "engineering_run_ok": healthy,
                "reuse_in_future_trial": False, "position_feedback": False,
                "simulated": args.mode == "simulate"})
            # Summary is published last. Target angles alone are not a success record.
            journal.write_json("summary.json", summary)
        except Exception as exc:
            summary.update(logging_failed=True, engineering_run_ok=False, result_write_error=str(exc))
            healthy = False
            print("结果文件写入失败；本次不能当作成功记录。以下仅为终端诊断。", file=sys.stderr)
        say(summary)
        return 0 if healthy else 2
    except KeyboardInterrupt:
        print("Stopped during setup.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    finally:
        if journal is not None:
            journal.close()


if __name__ == "__main__":
    raise SystemExit(main())
