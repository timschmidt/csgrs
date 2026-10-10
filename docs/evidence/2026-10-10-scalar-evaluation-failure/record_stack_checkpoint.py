#!/usr/bin/env python3
"""Verify and retain a frozen downstream checkpoint without runtime admission."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
from parse_test_results import parse_phase

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def main():
    out = EVIDENCE / sys.argv[1]
    assert not out.exists()
    previous = read(EVIDENCE / "typed-api-prototype-v24-state.json")
    assert all(digest(EVIDENCE / name) == value
               for name, value in previous["artifacts_sha256"].items())
    record = EVIDENCE / sys.argv[2] / "source-hashes.json.gz"
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
    expected = read(EVIDENCE.parent / "2026-10-10-artifact-numeric-keys/retention.json")[
        "staged_verification"]["preexisting_status"]
    status = subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT, text=True).splitlines()
    remainder = [line for line in status if not line[3:].startswith(str(EVIDENCE.relative_to(ROOT)))
                 and line[3:] != "docs/properties-status.json"]
    assert remainder == expected
    validations = {}
    for phase in sys.argv[3:]:
        path = EVIDENCE / phase
        result = read(path / "result.json")
        assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
        validations[phase] = {
            "command": read(path / "command.json")["command"], "result": result,
            "source_hashes_sha256": digest(path / "source-hashes.json.gz"),
            "tests": parse_phase(path),
        }
    before = json.loads(gzip.decompress((EVIDENCE / previous["source_record"]).read_bytes()))
    data = {
        "status": "isolated domain/serialization and downstream validation checkpoint; no runtime admission",
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
        "changes": [
            "Append the four new deferred-node variants after every existing serialization index. A serializer-boundary regression observes Serde's actual derived indices; both full node inventories retain all 64 cases in matching order.",
            "Negative square-root construction retains Unknown sign/zero facts. Rational -1, rational -4 and negative pi reject evaluation as SqrtNegative without publishing a root approximation cache.",
            "Square-root kernels reject a sample below -1 as SqrtNegative and accept the -1 zero-enclosure endpoint. The sin(1/2)-1/2 regression first panics in BigInt::sqrt on v33, then passes at three precisions; nine legal zero-cache endpoint cases also pass.",
            "Real::asin preserves an existing Invalid domain certificate before requesting cancellable numeric refinement, repairing the original lattice representation/abort matrix without changing its assertions.",
            "Updated cancellation/cache documentation and preserved the README version label required by its existing metadata tests.",
        ],
        "retained_failures": {
            "v25": "859 scalar unit tests pass; the new ordinal and negative-root fact regressions fail before their repairs.",
            "v28": "The full five-crate run stops on the lattice domain-certificate failure after curve and DRC pass. See typed-api-prototype-v28-partial-results.json.",
            "v30": "Predicate serialization inventory still expected the four deferred variants in the middle; fixtures were reordered without deleting any case.",
            "v31": "The pi-22/7 root fixture fails its unknown-sign precondition; this is a fixture assumption failure, not evidence of a numeric panic.",
            "v32": "The short exact test filter selects zero tests. It provides no correctness coverage.",
            "v33": "The replacement sin(1/2)-1/2 fixture reaches the integer-square-root panic.",
            "v35": "The README metadata test requires the literal Version label; restored without changing the assertion.",
        },
        "partial_stack_record": "typed-api-prototype-v28-partial-results.json",
        "counting_note": "Cargo's top-level tests/doctests and inherited child-process harness runs are counted separately. The older v11 count is audited in typed-api-prototype-v11-count-audit.json; its raw logs remain unchanged.",
        "remaining_requirements": [
            "Continue the explicit-signal, series-admission error-propagation and raw serialized-domain checks in domain-resource-source-review.json. Only the refined-negative-sqrt hypothesis has been exercised here.",
            "Implement actual scalar work/coefficient bounds; the fixed fractional precision ceiling does not bound total work, allocations or elapsed time.",
            "Repeat the complete curve/DRC suites on the final source: their v28 passes precede the asin and refined-root repairs. Existing ignored cases remain open.",
            "Freeze and replay original and expanded failures, validate the independent native prototype combination, and retain proportionate timing/allocation/size and original full-board report evidence before runtime admission.",
            "Existing ignored tests remain open and are listed explicitly above. The broader property, target and crate future-work scope and full pre-push gate remain required. No push has been attempted.",
        ],
    }
    data["artifacts_sha256"] = {
        str(path.relative_to(EVIDENCE)): digest(path)
        for path in sorted(EVIDENCE.rglob("*"))
        if path.is_file() and "__pycache__" not in path.parts
    }
    out.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"status": data["status"], "source_files": len(source),
                      "changed_sources": len(changed), "validated_phases": len(validations)}))


if __name__ == "__main__":
    main()
