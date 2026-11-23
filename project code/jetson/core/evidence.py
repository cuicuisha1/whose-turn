"""Executable inputs included in the offline verification record."""
import hashlib
from .config import ROOT


def source_hashes():
    files = [p for p in (ROOT / "jetson").rglob("*") if p.is_file()
             and p.suffix in {".py", ".json", ".html", ".txt"}
             and not any(part in {".venv", "runs", "models", "__pycache__"}
                         for part in p.relative_to(ROOT / "jetson").parts)]
    files += list((ROOT / "host").glob("*.py")) + [ROOT / "host/requirements.txt", ROOT / "scripts/package_robot.py"]
    return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
