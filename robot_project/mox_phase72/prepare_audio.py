#!/usr/bin/env python3
"""Generate fixed welcome/prompt WAV once using the existing separate Piper venv."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile

from mox72.runtime_io import wav_info

WORKER = r'''
import json, sys, wave
from piper import PiperVoice
request=json.load(sys.stdin)
voice=PiperVoice.load(request["model"])
if not callable(getattr(voice, "synthesize_wav", None)):
    raise RuntimeError("This Piper environment does not provide PiperVoice.synthesize_wav; verify its installed version/API.")
for item in request["items"]:
    with wave.open(item["output"], "wb") as stream:
        voice.synthesize_wav(item["text"], stream)
'''


def _publish_assets(staged: Path, assets: Path, names: tuple[str, ...]):
    """Publish a validated pair + manifest, restoring old files on I/O failure.

    This is a rollback transaction, not an atomic multi-file filesystem rename.
    Audio preparation must finish before starting a trial.
    """
    backups = staged / "previous"
    backups.mkdir()
    existing = set()
    for name in names:
        destination = assets / name
        if destination.exists():
            shutil.copyfile(destination, backups / name)
            existing.add(name)
    published = []
    try:
        for name in names:
            os.replace(staged / name, assets / name)
            published.append(name)
    except OSError as exc:
        errors = []
        for name in reversed(published):
            try:
                if name in existing:
                    os.replace(backups / name, assets / name)
                else:
                    (assets / name).unlink()
            except OSError as rollback_error:
                errors.append(f"{name}: {rollback_error}")
        if errors:
            raise OSError(f"Audio publication failed ({exc}); rollback failed: {errors}") from exc
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--piper-python", type=Path, default=Path.home()/"piper_env/bin/python")
    parser.add_argument("--model", type=Path, default=Path.home()/"piper_models/zh_CN-huayan-medium.onnx")
    parser.add_argument("--assets", type=Path, default=Path(__file__).absolute().parent/"assets")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    # Do not resolve the venv interpreter symlink to the system Python.
    interpreter = args.piper_python.expanduser().absolute()
    model = args.model.expanduser().absolute()
    assets = args.assets.expanduser().absolute()
    if not interpreter.is_file() or not model.is_file() or not Path(str(model)+".json").is_file():
        parser.error("请核对独立 Piper Python、ONNX 模型及同名 .onnx.json；本脚本不安装或下载依赖。")
    texts = {}
    for name in ("welcome", "prompt"):
        path = assets / f"{name}.txt"
        if not path.is_file():
            parser.error(f"缺少固定文本：{path}")
        texts[name] = path.read_text(encoding="utf-8-sig").strip()
        if not texts[name] or len(texts[name]) > 1000:
            parser.error(f"文本为空或过长：{path}")
        if (assets/f"{name}.wav").exists() and not args.overwrite:
            parser.error("WAV 已存在；需要改稿重生成时明确使用 --overwrite。正式采集期间保持固定版本。")
    assets.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="mox72_piper_", dir=assets) as tmp:
        items = [{"text": text, "output": str(Path(tmp)/f"{name}.wav")} for name,text in texts.items()]
        request = {"model": str(model), "items": items}
        result = subprocess.run([str(interpreter), "-c", WORKER],
                                input=json.dumps(request, ensure_ascii=False), text=True,
                                encoding="utf-8",
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=180)
        if result.returncode:
            print(result.stderr[-3000:], file=sys.stderr)
            return 1
        clips = {}
        for name in texts:
            clips[name] = wav_info(Path(tmp) / f"{name}.wav")
            clips[name]["path"] = str(assets / f"{name}.wav")
        manifest = {"model": str(model), "piper_python": str(interpreter), "texts": texts,
                    "clips": clips,
                    "purpose": "engineering example; replace texts with approved study scripts before freezing"}
        (Path(tmp)/"audio_manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
        _publish_assets(Path(tmp), assets, ("welcome.wav", "prompt.wav", "audio_manifest.json"))
    print("已生成 welcome.wav 与 prompt.wav；未播放、未打开摄像头或串口。")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, subprocess.TimeoutExpired, ValueError) as exc:
        print(f"音频准备失败：{exc}",file=sys.stderr)
        raise SystemExit(1)
