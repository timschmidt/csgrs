#!/usr/bin/env python3
"""Finish sequential validation of the frozen concurrent-publication candidate."""
import gzip
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
from parse_test_results_v2 import parse_phase

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]
EXPECTED_SOURCE = "e07fb6d0e04157499f88311758b5bdf5843664223fa1715a4fd261b43ea2eb11"


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def completed(phase):
    path = EVIDENCE / phase
    result = read(path / "result.json")
    assert result["exit_code"] == 0, (phase, result)
    assert result["source_unchanged"] and result["main_unchanged"], phase
    assert digest(path / "source-hashes.json.gz") == EXPECTED_SOURCE, phase
    return path


def summary(phase, name):
    result = parse_phase(completed(phase))
    assert result["top_level_failed"] == result["child_process_failed"] == 0
    out = EVIDENCE / name
    if out.exists():
        assert read(out) == result, out
    else:
        out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"phase": phase, "passed": result["top_level_passed"],
                      "child_passed": result["child_process_passed"],
                      "ignored": result["top_level_ignored"]}), flush=True)


def validate(phase, *arguments):
    if (EVIDENCE / phase).exists():
        path = completed(phase)
        expected_command = list(arguments[1:]) if arguments[0] == "--exec" else ["cargo", *arguments]
        assert read(path / "command.json")["command"] == expected_command, phase
        print(f"Reusing terminal passing phase: {phase}", flush=True)
        return path
    subprocess.run([sys.executable, str(EVIDENCE / "validate_candidate.py"),
                    phase, *arguments], cwd=ROOT, check=True)
    # The retained runner records Cargo failures without returning them itself.
    return completed(phase)


def main():
    stack = completed("typed-api-prototype-v203-stack-tests")
    summary(stack.name, "typed-api-prototype-v203-test-summary.json")
    binaries = re.findall(r"Running unittests src/lib\.rs \(([^)\n]*/hyperreal-[^)]+)\)",
                          (stack / "stderr.log").read_text())
    assert len(binaries) == 1, binaries
    binary = Path(binaries[0])
    binary_hash = digest(binary)

    validate("typed-api-prototype-v204-default-tests", "test", "--release", "-p", "hyperreal")
    summary("typed-api-prototype-v204-default-tests", "typed-api-prototype-v204-test-summary.json")
    validate("typed-api-prototype-v205-minimal-clippy", "clippy", "--release", "-p", "hyperreal",
             "--all-targets", "--no-default-features", "--", "-D", "warnings")
    validate("typed-api-prototype-v206-workspace-fmt", "fmt", "--all", "--", "--check")

    source = json.loads(gzip.decompress((stack / "source-hashes.json.gz").read_bytes()))
    original = json.loads(gzip.decompress(
        (EVIDENCE / "candidate-preparation/source-hashes.json.gz").read_bytes()))
    changed_rust = sorted(name for name, value in source.items()
                          if name.endswith(".rs") and original.get(name) != value)
    print(f"Checking {len(changed_rust)} changed Rust files from the frozen source map.", flush=True)
    validate("typed-api-prototype-v207-included-fmt", "--exec", "rustfmt", "--edition", "2024",
             "--config-path", "crates/.rustfmt.toml", "--config", "skip_children=true",
             "--check", *changed_rust)

    # No builds, tests or other measurements overlap either capped workload.
    for phase, selected in [
        ("typed-api-prototype-v208-publication-cap", "cache_publication_tests"),
        ("typed-api-prototype-v209-fractional-cap", "fractional_separation_tests"),
    ]:
        assert digest(binary) == binary_hash
        validate(phase, "--exec", sys.executable, str(EVIDENCE / "run_limited_unit_test.py"),
                 str(binary), binary_hash, f"computable::node::{selected}::",
                 "--nocapture", "--test-threads=1")

    build = "typed-api-prototype-v210-replay-build"
    replay = "typed-api-prototype-v211-replay"
    boundary = "typed-api-prototype-v212-precision-replay"
    validate(build, "build", "--release", "-p", "hyperreal", "-p", "hypercurve", "--all-features")
    subprocess.run([sys.executable, str(EVIDENCE / "reproduce_candidate.py"), build, replay],
                   cwd=ROOT, check=True)
    result = read(EVIDENCE / replay / "summary.json")
    assert result["source_hashes_sha256"] == EXPECTED_SOURCE and result["guards_unchanged"]
    assert result["ordinary_cases"] == 400
    assert result["invalid_first_contracts"] == result["invalid_retries"] == 0
    assert result["typed_interruption_errors"] == 300
    assert len(result["extremes"]) == 6 and all(row["valid"] for row in result["extremes"])
    assert len(result["cache"]) == 9 and all(row["valid"] for row in result["cache"])
    assert all(row["exit_code"] == 0 for row in result["commands"])

    subprocess.run([sys.executable, str(EVIDENCE / "replay_precision.py"), build, replay, boundary],
                   cwd=ROOT, check=True)
    result = read(EVIDENCE / boundary / "summary.json")
    assert result["source_hashes_sha256"] == EXPECTED_SOURCE and result["guards_unchanged"]
    assert result["cases"] == result["valid_cases"] == 60
    assert not result["nonzero_exit_cases"] and not result["invalid_numeric_cases"]
    print("All final-source publication checkpoint validations passed.", flush=True)


if __name__ == "__main__":
    main()
