#!/usr/bin/env python3
"""Retain a checked isolated candidate, with explicit checkpoint metadata."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
from parse_test_results_v2 import parse_phase

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def main():
    metadata = read(EVIDENCE / sys.argv[1])
    out = EVIDENCE / sys.argv[2]
    assert not out.exists()
    previous = read(EVIDENCE / metadata["previous_checkpoint"])
    for name, value in previous["artifacts_sha256"].items():
        assert digest(EVIDENCE / name) == value, name
    record = EVIDENCE / sys.argv[3] / "source-hashes.json.gz"
    source = json.loads(gzip.decompress(record.read_bytes()))
    for name, value in source.items():
        assert digest(SOURCE / name) == value, name
    original = json.loads(gzip.decompress(
        (EVIDENCE / "candidate-preparation/source-hashes.json.gz").read_bytes()
    ))
    changed = {name: value for name, value in source.items() if original.get(name) != value}
    for name, value in changed.items():
        assert digest(record.parent / "inputs" / name) == value, name
    guard = json.loads(gzip.decompress(
        (EVIDENCE / "candidate-preparation/main-hashes.json.gz").read_bytes()
    ))
    for name, value in guard.items():
        assert digest(ROOT / name) == value, name
    expected = read(EVIDENCE.parent / "2026-10-10-artifact-numeric-keys/retention.json")[
        "staged_verification"]["preexisting_status"]
    status = subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).splitlines()
    remainder = [line for line in status if not line[3:].startswith(str(EVIDENCE.relative_to(ROOT)))
                 and line[3:] != "docs/properties-status.json"]
    assert remainder == expected
    validations = {}
    for phase in sys.argv[4:]:
        path = EVIDENCE / phase
        result = read(path / "result.json")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        assert digest(path / "source-hashes.json.gz") == digest(record), phase
        validations[phase] = {
            "command": read(path / "command.json")["command"], "result": result,
            "source_hashes_sha256": digest(path / "source-hashes.json.gz"),
            "tests": parse_phase(path),
        }
    before = json.loads(gzip.decompress((EVIDENCE / previous["source_record"]).read_bytes()))
    data = metadata | {
        "source_directory": str(SOURCE), "source_record": str(record.relative_to(EVIDENCE)),
        "source_record_sha256": digest(record), "source_files": len(source),
        "archived_changed_sources_verified": len(changed), "main_guarded_files": len(guard),
        "main_runtime_unchanged": True, "original_worktree_entries_preserved": len(expected),
        "previous_checkpoint_artifacts_verified": len(previous["artifacts_sha256"]),
        "changed_since_previous_checkpoint": {
            name: {"before": before.get(name), "after": value}
            for name, value in source.items() if before.get(name) != value
        },
        "validation": validations,
        "artifacts_sha256": {
            str(path.relative_to(EVIDENCE)): digest(path)
            for path in sorted(EVIDENCE.rglob("*"))
            if path.is_file() and "__pycache__" not in path.parts
        },
    }
    out.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"status": data["status"], "source_files": len(source),
                      "changed_sources": len(changed), "validated_phases": len(validations),
                      "artifacts": len(data["artifacts_sha256"])}))


if __name__ == "__main__":
    main()
