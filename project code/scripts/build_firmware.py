"""Compile only; never uploads or touches a serial port."""
import argparse
import datetime
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools" if (ROOT / "tools").exists() else ROOT.parent / "tools"
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cli", type=Path)
    args = parser.parse_args()
    cli = args.cli or TOOLS / "arduino" / ("arduino-cli.exe" if sys.platform == "win32" else "arduino-cli")
    if not cli.exists():
        found = shutil.which("arduino-cli")
        if not found:
            raise SystemExit("Install Arduino CLI, renesas_uno@1.6.0 and ArduinoJson@6.21.5; see firmware/README.md")
        cli = Path(found)
    command = [str(cli)]
    config = TOOLS / "arduino-cli.yaml"
    if config.exists():
        command += ["--config-file", str(config)]
    command += ["compile", "--fqbn", "arduino:renesas_uno:unor4wifi", "--output-dir", str(ROOT / "build/firmware"),
                str(ROOT / "firmware/robot_controller")]
    (ROOT / "build").mkdir(exist_ok=True)
    (ROOT / "build/firmware_report.json").unlink(missing_ok=True)
    result = subprocess.run(command, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    log = result.stdout.decode("utf-8", errors="replace")
    (ROOT / "build/compile.log").write_text(log, encoding="utf-8")
    print(log)
    if result.returncode:
        raise SystemExit(result.returncode)
    files = sorted((ROOT / "firmware/robot_controller").rglob("*"))
    report = {"compiled_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "board": "arduino:renesas_uno:unor4wifi", "core": "arduino:renesas_uno@1.6.0",
              "json_library": "ArduinoJson@6.21.5", "hardware_tested": False, "uploaded": False,
              "protocol_version": "head-1", "chassis_control": False, "default_servo_enabled": False,
              "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()}}
    (ROOT / "build/firmware_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
