"""Package the vision module and matching offline verification, without models or recordings."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    if (ROOT / "jetson/robot.py").exists():
        raise SystemExit("The source is now v0.2. Use scripts/package_robot.py; the archived v0.1 ZIP is preserved.")
    report_path = ROOT / "build/vision/tests_report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report["result"] != "passed":
        raise SystemExit("Run python -m jetson.check successfully first")
    for name, expected in report["source_sha256"].items():
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != expected:
            raise SystemExit(f"Source changed since test run: {name}")
    output = ROOT / "output/jetson_vision_v0.1"
    output.mkdir(parents=True, exist_ok=True)
    destination = output / "Jetson视觉程序_v0.1.zip"
    files = [p for p in sorted((ROOT / "jetson").rglob("*")) if p.is_file()
             and not any(part in {".venv", "__pycache__", "runs", "models"} for part in p.relative_to(ROOT / "jetson").parts)]
    files += [ROOT / "build/vision/tests.log", report_path]
    with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as bundle:
        for path in files:
            bundle.write(path, str(path.relative_to(ROOT)))
        bundle.writestr("START_HERE.txt", "Unzip into a new folder on Jetson. Run commands from that folder.\n"
                        "1. python3 -m jetson.diagnose --output jetson/runs/environment_jetson.json\n"
                        "2. Follow jetson/README.md for environment setup and camera testing.\n"
                        "Only offline Windows validation is included; real camera/Jetson acceptance is pending.\n")
    manifest = {"file": destination.name, "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
                "files": len(files) + 1, "real_camera_tested": False, "jetson_tested": False}
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(destination)
    print(json.dumps(manifest, ensure_ascii=False))


if __name__ == "__main__":
    main()
