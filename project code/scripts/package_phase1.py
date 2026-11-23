"""Create a verified source + build-evidence delivery bundle; no hardware access."""
import datetime
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output/current_source"


def main():
    report = json.loads((ROOT / "build/firmware_report.json").read_text(encoding="utf-8"))
    tests = json.loads((ROOT / "build/tests_report.json").read_text(encoding="utf-8"))
    assert tests["result"] == "passed"
    for name, digest in {**report["source_sha256"], **tests["source_sha256"]}.items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest, f"Recompile changed source: {name}"
    OUT.mkdir(parents=True, exist_ok=True)
    for source, name in [(ROOT / "firmware/README.md", "使用与调试说明.md"),
                         (ROOT / "firmware/commissioning.csv", "实物验收记录_待填写.csv"),
                         (ROOT / "build/compile.log", "Uno_R4_编译日志.txt"),
                         (ROOT / "build/phase1_tests.log", "软件测试日志.txt")]:
        shutil.copyfile(source, OUT / name)
    summary = """# 当前头部固件与 USB 调试工具交付｜head-1

2026-11-10：实体模型保留四轮，软件移除四轮控制。固件只有 USB 串口、D10/D11 双舵机及通信/故障保护，不包含轮驱动、I²C、编码器、CPR 或 drive 模式。

Uno R4 WiFi 核心1.6.0、ArduinoJson6.21.5编译通过。测试范围见 build/tests_report.json 和完整日志；模拟器替代头部PWM及USB状态，不构成实物验证。WDT是否能实际复位仍需真机验收。

尚未烧录或实物验收。config.hpp 供电与舵机限位标记默认false，真实固件默认拒绝prepare。prepare不输出PWM；stop关闭PWM但不切断外部舵机电源，也不证明机械停止。

本包包含底层固件、主机调试工具、原生测试、构建/打包脚本、验收表及编译证据；不包含 whoseturn 研究程序、统一视觉程序、大型工具链或本地模拟器二进制。研究入口在工作目录 whoseturn；使用底层模拟控制台前运行 python scripts/test_phase1.py 构建模拟器。

从ZIP解压后先读 firmware/README.md。旧四轮源码已在毕业设计淘汰目录另存快照，不与当前包混用。
"""
    (OUT / "交付说明.md").write_text(summary, encoding="utf-8")
    sources = []
    for folder in ("firmware", "host", "tests"):
        sources.extend(p for p in (ROOT / folder).rglob("*") if p.is_file() and "__pycache__" not in p.parts)
    sources += [ROOT / "scripts" / name for name in ("setup_native.py", "build_firmware.py", "test_phase1.py", "package_phase1.py")]
    sources += [ROOT / "build" / name for name in ("compile.log", "phase1_tests.log", "firmware_report.json", "tests_report.json")]
    sources += [ROOT / "build/firmware/robot_controller.ino.bin", ROOT / "build/firmware/robot_controller.ino.hex"]
    sources += [OUT / "交付说明.md"]
    payload = {}
    for source in sources:
        name = "交付说明.md" if source.parent == OUT else source.relative_to(ROOT).as_posix()
        payload[name] = source.read_bytes()
    manifest = {name: hashlib.sha256(data).hexdigest() for name, data in payload.items()}
    payload["MANIFEST_SHA256.json"] = json.dumps(manifest, ensure_ascii=False, indent=2).encode("utf-8")
    archive = OUT / "Arduino底层控制与USB测试工具_当前源码.zip"
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as z:
        for name, data in sorted(payload.items()):
            z.writestr(name, data)
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for name, digest in manifest.items():
            assert hashlib.sha256(z.read(name)).hexdigest() == digest
    (OUT / "交付校验.json").write_text(json.dumps({
        "created_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "zip_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        "files": len(payload), "hardware_tested": False, "uploaded": False,
        "firmware_report": report, "tests_report": tests}, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"Verified {len(payload)} files: {archive}")


if __name__ == "__main__":
    main()
