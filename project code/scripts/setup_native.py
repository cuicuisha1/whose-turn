"""Fetch a pinned portable C++ compiler for executing the actual controller tests."""
import hashlib
import json
from pathlib import Path
import urllib.request
import zipfile

ROOT=Path(__file__).resolve().parents[1]
TOOLS=ROOT/'tools' if (ROOT/'tools').exists() or not (ROOT.parent/'tools').exists() else ROOT.parent/'tools'
VERSION='0.14.1'
DEST=TOOLS/'native'
DEST.mkdir(parents=True,exist_ok=True)
index=json.load(urllib.request.urlopen('https://ziglang.org/download/index.json',timeout=30))
asset=index[VERSION]['x86_64-windows']
archive=DEST/'zig.zip'
urllib.request.urlretrieve(asset['tarball'],archive)
actual=hashlib.sha256(archive.read_bytes()).hexdigest()
if actual!=asset['shasum']:raise SystemExit('Official Zig SHA256 mismatch')
with zipfile.ZipFile(archive) as z:
    for item in z.infolist():
        if not (DEST/item.filename).resolve().is_relative_to(DEST.resolve()):
            raise SystemExit('Invalid archive path')
    z.extractall(DEST)
(DEST/'download.json').write_text(json.dumps(dict(version=VERSION,url=asset['tarball'],sha256=actual),indent=2),encoding='utf-8')
print(next(DEST.glob('*/zig.exe')))
