"""Read-only environment inspection. No stream startup, installs, or device reset."""
import argparse
from datetime import datetime, timezone
import importlib
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys


def command(args):
    if not shutil.which(args[0]):
        return {"available": False}
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=10, errors="replace")
        return {"available": True, "returncode": result.returncode,
                "stdout": result.stdout[-16000:], "stderr": result.stderr[-2000:]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"available": True, "error": str(exc)}


def inspect():
    report = {"at_utc": datetime.now(timezone.utc).isoformat(), "platform": platform.platform(),
              "machine": platform.machine(), "python": sys.version, "executable": sys.executable,
              "packages": {}, "cameras": [], "camera_probe": "not_run"}
    for file in ("/etc/nv_tegra_release", "/etc/os-release"):
        path = Path(file)
        report[file] = path.read_text(errors="replace") if path.is_file() else None
    for module in ("numpy", "cv2", "pyrealsense2"):
        try:
            imported = importlib.import_module(module)
            report["packages"][module] = {"import_ok": True, "version": getattr(imported, "__version__", "unknown")}
        except Exception as exc:
            report["packages"][module] = {"import_ok": False, "error": str(exc)}
    for package in ("ultralytics", "torch", "torchvision", "tensorrt"):
        try:
            report["packages"][package] = {"installed_version": importlib.metadata.version(package), "import_tested": False}
        except importlib.metadata.PackageNotFoundError:
            report["packages"][package] = {"installed_version": None}
    if sys.platform.startswith("linux"):
        report["usb_devices"] = command(["lsusb"])
        report["usb_tree"] = command(["lsusb", "-t"])
        report["jetpack_packages"] = command(["dpkg-query", "-W", "nvidia-jetpack", "nvidia-l4t-core"])
        report["video_devices"] = sorted(str(p) for p in Path("/dev").glob("video*"))
    if report["packages"]["pyrealsense2"]["import_ok"]:
        try:
            import pyrealsense2 as rs
            ctx = rs.context()
            for device in ctx.query_devices():
                info = {}
                for name in ("name", "serial_number", "firmware_version", "usb_type_descriptor"):
                    key = getattr(rs.camera_info, name)
                    if device.supports(key):
                        info[name] = device.get_info(key)
                report["cameras"].append(info)
            report["camera_probe"] = "enumerated_only_not_stream_tested"
        except Exception as exc:
            report["camera_probe"] = str(exc)
    report["hardware_stream_tested"] = False
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    text = json.dumps(inspect(), ensure_ascii=False, indent=2)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
