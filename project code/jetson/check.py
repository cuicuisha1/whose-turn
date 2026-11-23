"""Run vision + robot integration checks; save evidence under build/robot."""
from datetime import datetime, timezone
import importlib.metadata
import io
import json
from pathlib import Path
import platform
import sys
import unittest
from .core.evidence import source_hashes


def main():
    root = Path(__file__).resolve().parent
    output = root.parent / "build" / "robot"
    output.mkdir(parents=True, exist_ok=True)
    suite = unittest.defaultTestLoader.discover(str(root / "tests"), pattern="test_*.py")
    stream = io.StringIO()
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    text = stream.getvalue()
    print(text)
    (output / "tests.log").write_text(text, encoding="utf-8")
    versions = {}
    for package in ("numpy", "opencv-python", "opencv-python-headless", "pyrealsense2", "ultralytics"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    report = {"at_utc": datetime.now(timezone.utc).isoformat(), "result": "passed" if result.wasSuccessful() else "failed",
              "tests_run": result.testsRun, "failures": len(result.failures), "errors": len(result.errors),
              "skipped": [{"test": str(t), "reason": reason} for t, reason in result.skipped],
              "python": sys.version, "platform": platform.platform(), "packages": versions,
              "real_camera_tested": False, "jetson_tested": False, "yolo_runtime_tested": False,
              "physical_actuation_tested": False, "speech_runtime_tested": False,
              "scope": "Vision algorithms/SDK stubs; robot behavior, stale data, commands, process recovery, HTTP, text captions, status-only Arduino simulator, full CLI lifecycle",
              "source_sha256": source_hashes()}
    (output / "tests_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
