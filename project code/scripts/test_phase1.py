"""Compile the production controller/protocol into native tests and a fake-HAL simulator."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools" if (ROOT / "tools").exists() else ROOT.parent / "tools"
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def main():
    out = ROOT / "build/tests"
    out.mkdir(parents=True, exist_ok=True)
    (ROOT / "build/tests_report.json").unlink(missing_ok=True)
    zig = next((TOOLS / "native").glob("*/zig.exe"), None) if sys.platform == "win32" else None
    compiler = [str(zig), "c++"] if zig else [os.environ.get("CXX", shutil.which("c++") or "c++")]
    library = TOOLS / "arduino-user/libraries/ArduinoJson/src"
    if not library.exists():
        library = Path(os.environ.get("ARDUINOJSON_INCLUDE", ""))
    if not (library / "ArduinoJson.h").exists():
        raise SystemExit("Set ARDUINOJSON_INCLUDE to ArduinoJson 6.21.5 src directory")
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    temp = TOOLS / "temp"
    temp.mkdir(parents=True, exist_ok=True)
    env.update(TMP=str(temp), TEMP=str(temp), ZIG_GLOBAL_CACHE_DIR=str(TOOLS / "zig-cache-global"),
               ZIG_LOCAL_CACHE_DIR=str(TOOLS / "zig-cache-local"))
    log = []

    def run(args):
        print("Running:", " ".join(str(a) for a in args), flush=True)
        p = subprocess.run([str(a) for a in args], cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        text = p.stdout.decode("utf-8", errors="replace")
        print(text, flush=True)
        log.append(text)
        (ROOT / "build/phase1_tests.log").write_text("\n".join(log), encoding="utf-8")
        if p.returncode:
            raise SystemExit(p.returncode)

    suffix = ".exe" if sys.platform == "win32" else ""
    for name in ("controller_tests", "simulator"):
        run(compiler + ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", ROOT / "firmware/robot_controller/src",
                        "-I", library, ROOT / f"tests/native/{name}.cpp", "-o", out / f"{name}{suffix}"])
    run([out / f"controller_tests{suffix}"])
    run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_*.py", "-v"])
    (ROOT / "build/tests_report.json").write_text(json.dumps({
        "at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(), "native_test_groups": 9,
        "scope": "Production C++ controller/protocol, Python host, fake-HAL end-to-end",
        "wiring_regression_run": False,
        "wiring_scope": "Head-only software; historical four-wheel wiring is outside acceptance scope",
        "source_sha256": {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                          for folder in ("firmware", "host", "tests", "scripts")
                          for p in sorted((ROOT / folder).rglob("*"))
                          if p.is_file() and p.suffix in {".py", ".hpp", ".cpp", ".ino"}},
        "hardware_tested": False, "result": "passed"}, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
