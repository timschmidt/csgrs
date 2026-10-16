"""Restore the previously pinned Go toolchain into a fresh isolated target."""
import hashlib
import json
from pathlib import Path
import tarfile
import urllib.request

E = Path(__file__).resolve().parent
TARGET = Path('/tmp/csgrs-adapter-failure-kinds/go-tools-v1')
metadata = json.loads((E.parent / '2026-10-15-bounds-failures/go-toolchain.json').read_text())
record = metadata['file']
assert record['filename'] == 'go1.27.2.linux-amd64.tar.gz'
archive = TARGET / record['filename']
assert not archive.exists() and not (TARGET / 'go').exists()
with urllib.request.urlopen('https://go.dev/dl/' + record['filename'], timeout=120) as response:
    with archive.open('xb') as output:
        while block := response.read(1024 * 1024):
            output.write(block)
assert archive.stat().st_size == record['size']
assert hashlib.sha256(archive.read_bytes()).hexdigest() == record['sha256']
with tarfile.open(archive) as source:
    source.extractall(TARGET, filter='data')
print(json.dumps({'archive': str(archive), 'bytes': archive.stat().st_size,
                  'sha256': record['sha256'], 'verified_before_extraction': True}))
