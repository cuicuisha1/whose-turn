"""Package v0.2 after checking that executable sources match passing test evidence."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from jetson import __version__
from jetson.core.evidence import source_hashes


def main():
    report_path = ROOT / "build/robot/tests_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report["result"] != "passed" or report["source_sha256"] != source_hashes():
        raise SystemExit("Sources changed or tests failed. Run python -m jetson.check first.")
    release = "v" + __version__
    output = ROOT / "output" / ("robot_software_" + release)
    output.mkdir(parents=True, exist_ok=True)
    destination = output / ("机器人软件架构_" + release + ".zip")
    files = [p for p in sorted((ROOT / "jetson").rglob("*")) if p.is_file()
             and not any(part in {".venv", "__pycache__", "runs", "models"} for part in p.relative_to(ROOT / "jetson").parts)]
    files += list((ROOT / "host").glob("*.py")) + [ROOT / "host/requirements.txt", Path(__file__).resolve(),
               ROOT / "build/robot/tests.log", report_path]
    with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as bundle:
        for path in files:
            bundle.write(path, path.relative_to(ROOT).as_posix())
        bundle.writestr("START_HERE.txt", f"Before We Approach robot software {release}\n"
                        "Read jetson/ROBOT_README.md and jetson/ARCHITECTURE.md.\n"
                        "Run from this extracted folder: python3 -m jetson.robot --duration 10\n"
                        "Default: simulation, virtual actuators, Arduino disabled.\n"
                        "On Jetson, first run: python3 -m jetson.diagnose\n"
                        "RealSense/Jetson/head actuation/ASR/TTS hardware acceptance is pending; chassis control is out of scope.\n"
                        "No models, recordings, native simulator or firmware binary included.\n")
    manifest = {"version": __version__, "file": destination.name,
                "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(), "files": len(files) + 1,
                "tests_run": report["tests_run"], "skipped": report["skipped"],
                "source_sha256": source_hashes(), "real_camera_tested": False, "jetson_tested": False,
                "physical_actuation": False, "speech_available": False}
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(destination)
    print(json.dumps({k: v for k, v in manifest.items() if k != "source_sha256"}, ensure_ascii=False))


if __name__ == "__main__":
    main()
