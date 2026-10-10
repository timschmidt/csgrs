#!/usr/bin/env python3
"""Retain the caller/lint checkpoint without admitting its runtime changes."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = EVIDENCE / "typed-api-prototype-v24-state.json"
    assert not out.exists()
    previous = json.loads((EVIDENCE / "typed-api-prototype-v11-state.json").read_text())
    assert all(digest(EVIDENCE / name) == value
               for name, value in previous["artifacts_sha256"].items())
    record = EVIDENCE / "typed-api-prototype-v24-fuzz-fmt/source-hashes.json.gz"
    source = json.loads(gzip.decompress(record.read_bytes()))
    assert all(digest(SOURCE / name) == value for name, value in source.items())
    original = json.loads(gzip.decompress(
        (EVIDENCE / "candidate-preparation/source-hashes.json.gz").read_bytes()
    ))
    changed = {name: value for name, value in source.items() if original.get(name) != value}
    assert all(digest(record.parent / "inputs" / name) == value
               for name, value in changed.items())
    guard = json.loads(gzip.decompress(
        (EVIDENCE / "candidate-preparation/main-hashes.json.gz").read_bytes()
    ))
    assert all(digest(ROOT / name) == value for name, value in guard.items())
    expected = json.loads((EVIDENCE.parent / "2026-10-10-artifact-numeric-keys/retention.json")
                          .read_text())["staged_verification"]["preexisting_status"]
    status = subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).splitlines()
    remainder = [line for line in status if not line[3:].startswith(str(EVIDENCE.relative_to(ROOT)))
                 and line[3:] != "docs/properties-status.json"]
    assert remainder == expected
    validations = {}
    for phase in ["v14-clippy", "v15-downstream-clippy", "v16-fuzz-clippy", "v19-stack-clippy",
                  "v20-fmt", "v22-included-fmt", "v24-fuzz-fmt"]:
        path = EVIDENCE / ("typed-api-prototype-" + phase)
        result = json.loads((path / "result.json").read_text())
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        validations[path.name] = {
            "command": json.loads((path / "command.json").read_text())["command"],
            "result": result,
        }
    data = {
        "status": "isolated caller, formatting and strict-lint checkpoint; no runtime admission",
        "source_directory": str(SOURCE), "source_record": str(record.relative_to(EVIDENCE)),
        "source_record_sha256": digest(record), "source_files": len(source),
        "archived_changed_sources_verified": len(changed), "main_guarded_files": len(guard),
        "main_runtime_unchanged": True, "original_worktree_entries_preserved": len(expected),
        "previous_checkpoint_artifacts_verified": len(previous["artifacts_sha256"]),
        "validation": validations,
        "changes": [
            "Seven numeric benchmark files require successful evaluation before timing or consuming output.",
            "Both graph fuzz targets require successful numeric refinement and cache hits instead of accepting two matching errors.",
            "Documented fallible approximation, rounding and comparison APIs, cancellation/cache behavior, precision-ceiling limits and migration from 0.13.1.",
            "Formatted migrated include! fragments explicitly and removed redundant Result wrappers flagged by strict linting.",
        ],
        "retained_failed_checks": ["typed-api-prototype-v12-clippy", "typed-api-prototype-v13-fmt",
                                   "typed-api-prototype-v18-stack-clippy", "typed-api-prototype-v21-fuzz-fmt"],
        "test_coverage": "Latest executed full scalar suite remains v11: 974 unit/integration tests and 28 doctests. The present frozen source has not yet run the full scalar/downstream tests or final adversarial replays.",
        "requirements_before_admission": [
            "Repair and regress serialization variant indices: four deferred variants were inserted before existing variants, despite the append-only ordinal contract.",
            "Reproduce and repair the negative-sqrt constructor's positive sign fact; audit remaining domain/precision/cancellation paths.",
            "Implement actual scalar work/coefficient bounds; a fixed fractional precision ceiling is not a total work or memory bound.",
            "Validate scalar feature variants, full affected downstream tests, native-prototype compatibility, and every original report workload.",
            "Freeze and replay all original and expanded failures; retain proportionate timing/allocation/size evidence before applying runtime code.",
            "Complete all broader property, target and future-work scope and the full pre-push matrix. No push has been attempted.",
        ],
    }
    data["artifacts_sha256"] = {
        str(path.relative_to(EVIDENCE)): digest(path)
        for path in sorted(EVIDENCE.rglob("*"))
        if path.is_file() and "__pycache__" not in path.parts
    }
    out.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({key: data[key] for key in ["status", "source_files",
          "archived_changed_sources_verified", "main_guarded_files", "original_worktree_entries_preserved"]}))


if __name__ == "__main__":
    main()
