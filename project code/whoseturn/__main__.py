"""运行: python -m whoseturn --demo；默认无硬件、无声音。"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import queue
import signal
import sys
import threading
import time
import uuid

from .core import Study, load_config
from .devices import Head, Speech, open_client
from jetson.core.contracts import parse_observation
from jetson.core.recording import write_json

HERE = Path(__file__).resolve().parent


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["virtual", "firmware_sim", "hardware"], default="virtual")
    parser.add_argument("--port")
    parser.add_argument("--config", type=Path, default=HERE / "config.json")
    parser.add_argument("--variant", choices=["head_only", "explicit"], default="head_only")
    parser.add_argument("--audio", choices=["text", "wav"], default="text")
    parser.add_argument("--audio-dir", type=Path)
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--motors-disconnected", action="store_true")
    parser.add_argument("--head-centred", action="store_true")
    parser.add_argument("--vision-latest", type=Path, help="同机独立视觉 latest.json；仅记录，不控制动作")
    parser.add_argument("--output", type=Path, default=HERE.parent / "build/whoseturn/runs")
    args = parser.parse_args(argv)
    cfg = load_config(args.config).validate(hardware=args.mode == "hardware")
    if args.mode == "hardware" and (args.demo or not args.port or not args.motors_disconnected or not args.head_centred):
        parser.error("真机必须显式指定端口、确认动力线已断开及头部已对中；禁止 --demo")
    if args.mode != "hardware" and args.port:
        parser.error("模拟模式不能指定真实端口")
    prompts = json.loads((HERE / "prompts.json").read_text(encoding="utf-8"))
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
    output = args.output / run_id
    output.mkdir(parents=True, exist_ok=False)
    events = (output / "events.jsonl").open("x", encoding="utf-8")
    sequence = 0
    def log(data):
        nonlocal sequence
        sequence += 1
        events.write(json.dumps({"seq": sequence, "run_id": run_id, "mode": args.mode,
                                 "synthetic_scenario": args.demo, **data}, ensure_ascii=False, allow_nan=False) + "\n")
        events.flush()  # 证据写入失败必须终止，不能假装已记录。

    manifest = {str(p.relative_to(HERE)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in HERE.rglob("*") if p.is_file() and p.suffix in {".py", ".json", ".md"}}
    audio_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in args.audio_dir.glob("*.wav")} if args.audio_dir else {}
    write_json(output / "run.json", {"run_id": run_id, "mode": args.mode, "variant": args.variant,
        "config": asdict(cfg), "prompts": prompts, "source_sha256": manifest, "audio_sha256": audio_hashes,
        "audio_mode": args.audio, "participant_data": False if args.demo else "not_asserted",
        "hardware_verified": False, "research_roles_assigned_by": "operator"})
    inbox = queue.Queue(maxsize=32)
    signal_stop = threading.Event()
    old_handlers = {}
    def reader():
        while not signal_stop.is_set():
            try:
                line = input()
            except EOFError:
                signal_stop.set()
                return
            if line.strip() == "stop":
                signal_stop.set()
                return
            try:
                inbox.put_nowait((time.monotonic(), line))
            except queue.Full:
                signal_stop.set()
                return
    client = speech = study = head = None
    rc, error, last_vision = 0, None, 0
    try:
        speech = Speech(prompts, log, args.audio, args.audio_dir)
        client = open_client(args.mode, args.port)
        now = time.monotonic()
        head = Head(cfg, log, now, client)
        study = Study(cfg, head, speech, log, now, args.variant)
        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, lambda *_: signal_stop.set())
        print("Whose Turn? | " + args.mode + " | " + args.audio, flush=True)
        print("commands: clear, prepare, start, join, advance, repair, finish materials|interaction|skip, mark TEXT, sync LABEL, stop", flush=True)
        print("研究日志: " + str(output), flush=True)
        if args.demo and client:
            # 仅显式 --demo 的假 HAL 才自动清理/准备；实机永远不能走此路径。
            client.request("clear_fault")
            head.prepare()
            study.last = head.last
        if not args.demo:
            threading.Thread(target=reader, daemon=True).start()
        previous, entered, repaired = None, now, False
        while study.state not in {"STOPPED", "DONE"}:
            now = time.monotonic()
            if signal_stop.is_set():
                study.stop(now, "signal_or_operator_stop")
                break
            study.step(now)
            if study.state != previous:
                log({"event": "state_entered", "monotonic_s": now, **study.snapshot()})
                print(study.state, "|", study.intent, "| planned respondent:", study.addressee, flush=True)
                if speech.busy:
                    print("ROBOT:", prompts[speech.prompt], flush=True)
                previous, entered = study.state, now
            if args.demo:
                cmd, text = None, ""
                if study.state == "READY":
                    cmd = "start"
                elif study.state == "WAIT_A" and now - entered > .8:
                    cmd = "advance" if study.joined else "join"
                elif study.state == "WAIT_B" and now - entered > .8:
                    if not repaired:
                        study.command("mark", now, "SYNTHETIC: B asks whether the question is for them", "scenario")
                        cmd, repaired = "repair", True
                    else:
                        cmd, text = "finish", "materials"
                if cmd:
                    study.command(cmd, now, text, "scenario")
            else:
                try:
                    created, line = inbox.get_nowait()
                except queue.Empty:
                    line = None
                if line:
                    if now - created > 2:
                        log({"event": "stale_command_discarded", "monotonic_s": now, "text": line[:500]})
                    else:
                        cmd, _, text = line.partition(" ")
                        try:
                            if cmd in {"clear", "prepare"}:
                                if study.state != "READY" or not client:
                                    raise ValueError("clear/prepare 仅用于未开始的串口会话")
                                log({"event": "operator_device_command", "monotonic_s": now, "command": cmd})
                                if cmd == "clear":
                                    client.request("clear_fault")
                                else:
                                    head.prepare()
                                    study.last = head.last
                            else:
                                study.command(cmd, now, text)
                        except ValueError as exc:
                            print(str(exc), flush=True)
            now = time.monotonic()
            if args.vision_latest and now - last_vision > .5:
                try:
                    if args.vision_latest.stat().st_size > 262144:
                        raise ValueError("视觉消息过大")
                    packet = json.loads(args.vision_latest.read_text(encoding="utf-8"))
                    observation = parse_observation(packet, False, now, .5).to_dict()
                except (OSError, ValueError) as exc:
                    observation = {"valid": False, "reason": str(exc)[:120]}
                log({"event": "vision_observation_only", "monotonic_s": now,
                     "observation": observation, "mapped_to_research_role": False})
                last_vision = now
            write_json(output / "latest.json", {"run_id": run_id, "updated_monotonic_s": now,
                       "mode": args.mode, "synthetic_scenario": args.demo, **study.snapshot()})
            time.sleep(.02)
    except (Exception, KeyboardInterrupt) as exc:
        rc, error = 2, f"{type(exc).__name__}: {exc}"
        print(error, file=sys.stderr)
    finally:
        # 清理故障不能跳过串口停止；每个资源分别收尾。
        cleanup = []
        for action in ((speech.stop if speech else None), (head.stop if head else None),
                       (client.transport.close if client else None)):
            try:
                if action:
                    action()
            except Exception as exc:
                cleanup.append(str(exc))
        signal_stop.set()
        for sig, old in old_handlers.items():
            signal.signal(sig, old)
        try:
            log({"event": "run_closed", "monotonic_s": time.monotonic(), "error": error,
                 "cleanup_errors": cleanup, "physical_stop_confirmed": False})
        finally:
            events.close()
        write_json(output / "summary.json", {"run_id": run_id, "exit_code": 2 if cleanup else rc,
            "final_state": study.state if study else "STARTUP_FAILED", "error": error,
            "cleanup_errors": cleanup, "hardware_verified": False,
            "participant_study_completed": False, "synthetic_scenario": args.demo})
        # 清除退出时留下的“仍在播放/等待”的实时状态，旧日志只作为证据。
        write_json(output / "latest.json", {"run_id": run_id, "state": "CLOSED",
                   "updated_monotonic_s": time.monotonic(), "valid": False, "error": error})
    return 2 if cleanup else rc


if __name__ == "__main__":
    raise SystemExit(main())
