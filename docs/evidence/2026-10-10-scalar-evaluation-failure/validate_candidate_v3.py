#!/usr/bin/env python3
"""Freeze each isolated prototype, including all introduced scalar input types."""
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


def source_hashes(source, original):
    """Hash original workspace inputs and every introduced scalar input file."""
    result = {name: digest(source / name) for name in original}
    for directory, children, names in os.walk(source / "crates/hyperreal"):
        children[:] = sorted(name for name in children
                             if name not in {"target", ".git", "__pycache__"})
        for name in [*children, *names]:
            path = Path(directory) / name
            if path.is_symlink():
                raise RuntimeError(f"Symbolic-link scalar input is not supported: {path}")
        for name in sorted(names):
            path = Path(directory) / name
            if path.is_file():
                result[str(path.relative_to(source))] = digest(path)
    return result


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
    source = source_hashes(SOURCE, original)
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
    source_snapshot_error = None
    try:
        source_unchanged = source_hashes(SOURCE, original) == source
    except (OSError, RuntimeError) as error:
        source_unchanged = False
        source_snapshot_error = str(error)
    main_unchanged = all(digest(ROOT / name) == value for name, value in guard.items())
    record = {
        "exit_code": result.returncode, "seconds": time.monotonic() - start,
        "source_unchanged": source_unchanged, "main_unchanged": main_unchanged,
        "source_catalog_rechecked": True, "source_snapshot_error": source_snapshot_error,
        "status": "isolated prototype validation; not runtime admission",
    }
    (out / "result.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record))
    assert source_unchanged and main_unchanged


if __name__ == "__main__":
    main()
