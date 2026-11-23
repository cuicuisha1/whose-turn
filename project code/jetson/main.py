"""Run from project code: python -m jetson.main --simulate --duration 10."""
import argparse
from collections import deque
from dataclasses import replace
from datetime import datetime, timezone
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import signal
import sys
import time
import uuid

from . import __version__

LOG = logging.getLogger("jetson")


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return number


def arguments(argv=None):
    p = argparse.ArgumentParser(description="RGB-D person perception; no actuator or serial-port control.")
    p.add_argument("--simulate", action="store_true", help="Use clearly labeled synthetic fixtures")
    p.add_argument("--detector", choices=["hog", "yolo", "none", "synthetic"])
    p.add_argument("--model", type=Path, help="Local YOLO .pt or .engine file; no automatic model download")
    p.add_argument("--device", default="cpu", help="YOLO inference device, e.g. cpu or 0")
    p.add_argument("--confidence", type=positive, default=.45, help="YOLO confidence threshold")
    p.add_argument("--serial", help="RealSense serial number")
    p.add_argument("--fps", type=int, choices=[15, 30], default=30)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--camera-timeout-ms", type=int, default=1000)
    p.add_argument("--max-frame-age-ms", type=positive, default=500)
    p.add_argument("--detection-hz", type=positive, default=8)
    p.add_argument("--output-hz", type=positive, default=5)
    p.add_argument("--min-depth", type=positive, default=.2)
    p.add_argument("--max-depth", type=positive, default=6.0)
    p.add_argument("--duration", type=positive, help="Stop after this many seconds; default: until Ctrl+C")
    p.add_argument("--preview", action="store_true", help="Show OpenCV preview on a local desktop")
    p.add_argument("--snapshot", action="store_true", help="Save last rendered diagnostic image to the run folder")
    p.add_argument("--stdout", action="store_true", help="Also emit NDJSON to stdout")
    p.add_argument("--output-dir", type=Path, default=Path(__file__).parent / "runs")
    a = p.parse_args(argv)
    a.detector = a.detector or ("synthetic" if a.simulate else "hog")
    if a.detector == "synthetic" and not a.simulate:
        p.error("--detector synthetic requires --simulate")
    if a.detector == "yolo" and a.model is None:
        p.error("--detector yolo requires --model with a local file")
    if not (64 <= a.width <= 1920 and 128 <= a.height <= 1080):
        p.error("width/height out of range")
    if not 100 <= a.camera_timeout_ms <= 3000:
        p.error("--camera-timeout-ms must be 100..3000")
    if a.min_depth >= a.max_depth or a.confidence > 1:
        p.error("require min-depth < max-depth and confidence <= 1")
    return a


class Output:
    def __init__(self, directory, stdout=False):
        self.directory, self.stdout = directory, stdout
        self.handler = RotatingFileHandler(directory / "observations.jsonl", maxBytes=5_000_000,
                                           backupCount=3, encoding="utf-8")
        self.handler.setFormatter(logging.Formatter("%(message)s"))

    def write(self, value):
        text = json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        temporary = self.directory / "latest.json.tmp"
        temporary.write_text(text + "\n", encoding="utf-8")
        os.replace(temporary, self.directory / "latest.json")
        self.handler.emit(logging.LogRecord("perception", logging.INFO, "", 0, text, (), None))
        if self.stdout:
            print(text, flush=True)

    def close(self):
        self.handler.close()


def run(args):
    # Imported here so --help works even on a Jetson with no vision packages yet.
    from .vision.realsense_camera import CaptureWorker, RealSenseCamera, SyntheticCamera
    from .vision.person_detector import HogDetector, NoDetector, SyntheticDetector, YoloDetector
    from .vision.perception import Perception, make_observation

    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid.uuid4().hex[:8]
    directory = args.output_dir.expanduser().resolve() / run_id
    directory.mkdir(parents=True, exist_ok=False)
    log_file = RotatingFileHandler(directory / "events.log", maxBytes=2_000_000, backupCount=2, encoding="utf-8")
    log_file.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logging.getLogger().addHandler(log_file)
    settings = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    (directory / "run.json").write_text(json.dumps({"version": __version__, "run_id": run_id,
                                                   "settings": settings, "python": sys.version}, indent=2), encoding="utf-8")
    LOG.info("Run folder: %s", directory)
    source = "simulation" if args.simulate else "realsense"
    detector = None
    worker = None
    sink = Output(directory, args.stdout)
    started = time.monotonic()
    processed = valid_outputs = invalid_outputs = targets_seen = 0
    inference_times = deque(maxlen=1000)
    result = frame = processed_generation = None
    last_sequence = -1
    next_detection = next_output = next_log = next_preview = 0.0
    last_image = last_message = None
    exit_code = 0
    stopped = False
    stop_requested = False
    old_sigterm = signal.getsignal(signal.SIGTERM)

    def request_stop(signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGTERM, request_stop)
    try:
        if args.preview or args.snapshot:
            import cv2
            from .preview import render
            if args.preview:
                if sys.platform.startswith("linux") and not (os.getenv("DISPLAY") or os.getenv("WAYLAND_DISPLAY")):
                    raise RuntimeError("No desktop display. Omit --preview for SSH/headless use.")
                cv2.namedWindow("Before We Approach - RGB-D", cv2.WINDOW_NORMAL)
        if args.detector == "hog":
            detector = HogDetector()
        elif args.detector == "yolo":
            detector = YoloDetector(args.model, args.device, args.confidence)
        elif args.detector == "synthetic":
            detector = SyntheticDetector()
        else:
            detector = NoDetector()
        perception = Perception(detector, args.min_depth, args.max_depth)
        camera = (SyntheticCamera(args.fps, args.width, args.height) if args.simulate else
                  RealSenseCamera(args.serial, args.width, args.height, args.fps, args.camera_timeout_ms))
        worker = CaptureWorker(camera)
        worker.start()
        loop_started = time.monotonic()
        LOG.info("Source=%s detector=%s; Ctrl+C to stop", source, detector.name)
        while not stop_requested and (args.duration is None or time.monotonic() - loop_started < args.duration):
            now = time.monotonic()
            status = worker.snapshot()
            if status.state != "streaming":
                result = frame = processed_generation = None
                perception.reset()
            elif (status.sequence != last_sequence and now >= next_detection and status.frame is not None):
                candidate = status.frame
                last_sequence = status.sequence
                next_detection = now + 1 / args.detection_hz
                if now - candidate.received_at <= args.max_frame_age_ms / 1000:
                    inference_start = time.monotonic()
                    result = perception.process(candidate, status.generation)
                    inference_times.append((time.monotonic() - inference_start) * 1000)
                    frame, processed_generation = candidate, status.generation
                    processed += 1
            now = time.monotonic()
            # A camera may disconnect or reconnect while inference is running.
            status = worker.snapshot()
            message = make_observation(status, result, frame, processed_generation, now,
                                       args.max_frame_age_ms / 1000, detector.name, source, run_id)
            if now >= next_output:
                message["emitted_at_utc"] = datetime.now(timezone.utc).isoformat()
                message["inference_ms"] = round(inference_times[-1], 2) if inference_times else None
                sink.write(message)
                last_message = message
                valid_outputs += int(message["valid"])
                invalid_outputs += int(not message["valid"])
                targets_seen += int(message["target"] is not None)
                next_output = now + 1 / args.output_hz
            if now >= next_log:
                LOG.info("%s | capture %.1f fps | people %d | age %s ms | restarts %d",
                         message["status"], status.capture_fps, message["person_count"],
                         message["frame_age_ms"], status.restarts)
                next_log = now + 5
            if (args.preview or args.snapshot) and frame is not None and now >= next_preview:
                last_image = render(frame, message, args.max_depth)
                next_preview = now + 1 / 15
            if args.preview:
                if last_image is not None:
                    # Re-render on an outage with cleared targets and an explicit invalid overlay.
                    if frame is None:
                        last_image[:] = 20
                        cv2.putText(last_image, "CAMERA UNAVAILABLE / NO CURRENT TARGET", (15, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, .7, (60, 60, 255), 2)
                    cv2.imshow("Before We Approach - RGB-D", last_image)
                key = cv2.waitKey(1) & 0xff
                if key in (27, ord("q")) or cv2.getWindowProperty("Before We Approach - RGB-D", cv2.WND_PROP_VISIBLE) < 1:
                    break
            time.sleep(.005)
        if not valid_outputs or not last_message or not last_message["valid"]:
            exit_code = 2
    except KeyboardInterrupt:
        LOG.info("Stopped by keyboard")
    except Exception:
        LOG.exception("Vision stopped with an error")
        exit_code = 2
    finally:
        signal.signal(signal.SIGTERM, old_sigterm)
        if worker is not None:
            stopped = worker.close()
            if not stopped:
                exit_code = 2
            status = worker.snapshot()
        else:
            from .vision.realsense_camera import CaptureStatus
            status = CaptureStatus()
        final = make_observation(replace(status, state="stopped" if exit_code == 0 else "error", frame=None),
                                 None, None, None, time.monotonic(), args.max_frame_age_ms / 1000,
                                 detector.name if detector else args.detector, source, run_id)
        final["emitted_at_utc"] = datetime.now(timezone.utc).isoformat()
        try:
            sink.write(final)
            if args.snapshot and last_image is not None:
                ok, encoded = cv2.imencode(".png", last_image)
                if not ok:
                    raise RuntimeError("Could not save snapshot")
                # imwrite on Windows does not reliably accept non-ASCII paths.
                (directory / "last_preview.png").write_bytes(encoded.tobytes())
            summary = {"source": source, "simulated": args.simulate, "exit_code": exit_code,
                       "duration_s": round(time.monotonic() - started, 2), "processed_frames": processed,
                       "valid_outputs": valid_outputs, "invalid_outputs": invalid_outputs,
                       "outputs_with_target": targets_seen, "camera_restarts": status.restarts,
                       "capture_thread_stopped": stopped,
                       "mean_recent_inference_ms": sum(inference_times) / len(inference_times) if inference_times else None,
                       "actuation_enabled": False}
            (directory / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        except Exception:
            LOG.exception("Could not finalize output files")
            exit_code = 2
        sink.close()
        if args.preview and "cv2" in locals():
            cv2.destroyAllWindows()
        logging.getLogger().removeHandler(log_file)
        log_file.close()
    return exit_code


def main(argv=None):
    args = arguments(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        return run(args)
    except (ImportError, RuntimeError, OSError) as exc:
        LOG.error("Startup failed: %s. See jetson/README.md and run python -m jetson.diagnose.", exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
