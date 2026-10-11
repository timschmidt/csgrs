#!/usr/bin/env python3
"""Freeze each isolated prototype and retain every validation result."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
TARGET = ROOT / "target/scalar-evaluation-candidate/target"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    phase = sys.argv[1]
    out = EVIDENCE / phase
    out.mkdir()
    original = json.loads(gzip.decompress(
        (EVIDENCE / "candidate-preparation/source-hashes.json.gz").read_bytes()
    ))
    guard = json.loads(gzip.decompress(
        (EVIDENCE / "candidate-preparation/main-hashes.json.gz").read_bytes()
    ))
    assert all(digest(ROOT / name) == value for name, value in guard.items())
    source = {name: digest(SOURCE / name) for name in original}
    # Include any newly introduced runtime sources in the immutable record.
    for path in (SOURCE / "crates/hyperreal").rglob("*.rs"):
        source[str(path.relative_to(SOURCE))] = digest(path)
    for name, value in source.items():
        if original.get(name) != value:
            target = out / "inputs" / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes((SOURCE / name).read_bytes())
    (out / "source-hashes.json.gz").write_bytes(gzip.compress(
        (json.dumps(source, sort_keys=True) + "\n").encode(), mtime=0
    ))
    command = ["cargo", *(sys.argv[2:] or ["check", "--release", "-p", "hyperreal", "--lib", "--all-features"])]
    if sys.argv[2:3] == ["--exec"]:
        command = sys.argv[3:]
        assert command, "explicit executable is required"
    env = os.environ | {
        "CARGO_HOME": "/tmp/csgrs-properties-cargo",
        "CARGO_NET_OFFLINE": "true",
        "CARGO_TARGET_DIR": str(TARGET),
        "CCACHE_DIR": "/tmp/csgrs-properties-ccache",
    }
    (out / "command.json").write_text(json.dumps({
        "command": command, "cwd": str(SOURCE),
        "source_hashes_sha256": digest(out / "source-hashes.json.gz"),
        "environment": {key: env[key] for key in ["CARGO_HOME", "CARGO_NET_OFFLINE", "CARGO_TARGET_DIR", "CCACHE_DIR"]},
    }, indent=2) + "\n")
    (out / "runner.py").write_bytes(Path(__file__).read_bytes())
    start = time.monotonic()
    with (out / "stdout.log").open("wb") as stdout, (out / "stderr.log").open("wb") as stderr:
        result = subprocess.run(command, cwd=SOURCE, env=env, stdout=stdout, stderr=stderr)
    source_unchanged = all(digest(SOURCE / name) == value for name, value in source.items())
    main_unchanged = all(digest(ROOT / name) == value for name, value in guard.items())
    record = {
        "exit_code": result.returncode, "seconds": time.monotonic() - start,
        "source_unchanged": source_unchanged, "main_unchanged": main_unchanged,
        "status": "isolated prototype validation; not runtime admission",
    }
    (out / "result.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record))
    assert source_unchanged and main_unchanged


if __name__ == "__main__":
    main()
