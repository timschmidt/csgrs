"""Retain exact raw log bytes before removing trailing whitespace for Git.

Run once after copying all runtime evidence into this directory. Subsequent
runs verify existing records and may add newly retained logs.
"""
import gzip
import hashlib
import json
from pathlib import Path

record = Path(__file__).resolve().parent
digest = lambda data: hashlib.sha256(data).hexdigest()
path = record / "log-storage.json"
stored = json.loads(path.read_text()) if path.exists() else {}
for log in sorted(record.rglob("*.log")):
    name = str(log.relative_to(record))
    if name in stored:
        row = stored[name]
        raw = gzip.decompress((record / row["raw_archive"]).read_bytes())
        assert digest(raw) == row["raw_sha256"]
        assert digest(log.read_bytes()) == row["display_sha256"]
        continue
    raw = log.read_bytes()
    clean = b"\n".join(line.rstrip(b" \t\r") for line in raw.split(b"\n"))
    if raw == clean:
        continue
    archive = log.with_suffix(".raw.log.gz")
    archive.write_bytes(gzip.compress(raw, mtime=0))
    log.write_bytes(clean)
    stored[name] = dict(raw_archive=str(archive.relative_to(record)),
                       raw_bytes=len(raw), raw_sha256=digest(raw),
                       archive_sha256=digest(archive.read_bytes()),
                       display_bytes=len(clean), display_sha256=digest(clean))
path.write_text(json.dumps(stored, indent=2) + "\n")
print(f"Retained raw bytes for {len(stored)} whitespace-normalized logs.")
