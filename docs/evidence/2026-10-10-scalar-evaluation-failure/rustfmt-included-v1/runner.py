#!/usr/bin/env python3
"""Apply rustfmt to a terminal validation snapshot and retain the exact delta."""
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


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    prior = EVIDENCE / sys.argv[1]
    assert (prior / "result.json").exists(), "prior validation must be terminal"
    before = json.loads(gzip.decompress((prior / "source-hashes.json.gz").read_bytes()))
    guard = json.loads(gzip.decompress(
        (EVIDENCE / "candidate-preparation/main-hashes.json.gz").read_bytes()
    ))
    assert all(digest(SOURCE / name) == value for name, value in before.items())
    assert all(digest(ROOT / name) == value for name, value in guard.items())
    out = EVIDENCE / sys.argv[2]
    out.mkdir()
    command = ["cargo", "fmt", "--all"]
    if len(sys.argv) > 3:
        assert sys.argv[3:] == ["included"]
        original = json.loads(gzip.decompress(
            (EVIDENCE / "candidate-preparation/source-hashes.json.gz").read_bytes()
        ))
        changed_rust = sorted(name for name, value in before.items()
                              if name.endswith(".rs") and original.get(name) != value)
        assert all(name.startswith("crates/") for name in changed_rust)
        # cargo fmt does not visit include! fragments. These crate files use
        # the crate style, and skip_children prevents unrelated module edits.
        command = ["rustfmt", "--edition", "2024", "--config-path", "crates/.rustfmt.toml",
                   "--config", "skip_children=true", *changed_rust]
    environment = {"CARGO_HOME": "/tmp/csgrs-properties-cargo", "CARGO_NET_OFFLINE": "true"}
    (out / "runner.py").write_bytes(Path(__file__).read_bytes())
    (out / "command.json").write_text(json.dumps({
        "command": command, "cwd": str(SOURCE), "environment": environment,
        "before_source_hashes": str(prior / "source-hashes.json.gz"),
        "before_source_hashes_sha256": digest(prior / "source-hashes.json.gz"),
    }, indent=2) + "\n")
    start = time.monotonic()
    with (out / "stdout.log").open("wb") as stdout, (out / "stderr.log").open("wb") as stderr:
        result = subprocess.run(command, cwd=SOURCE, env=os.environ | environment,
                                stdout=stdout, stderr=stderr)
    after = {name: digest(SOURCE / name) for name in before}
    changed = {name: {"before": before[name], "after": value}
               for name, value in after.items() if value != before[name]}
    for name in changed:
        path = out / "inputs" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((SOURCE / name).read_bytes())
    (out / "source-hashes.json.gz").write_bytes(gzip.compress(
        (json.dumps(after, sort_keys=True) + "\n").encode(), mtime=0
    ))
    main_unchanged = all(digest(ROOT / name) == value for name, value in guard.items())
    (out / "result.json").write_text(json.dumps({
        "exit_code": result.returncode, "seconds": time.monotonic() - start,
        "main_unchanged": main_unchanged, "changed_sources": changed,
    }, indent=2) + "\n")
    print(json.dumps({"exit_code": result.returncode, "changed_sources": len(changed),
                      "main_unchanged": main_unchanged}))
    assert main_unchanged and result.returncode == 0


if __name__ == "__main__":
    main()
