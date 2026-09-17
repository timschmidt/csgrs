#!/usr/bin/env python3
"""Check that baseline qualification is complete and bound to current inputs.

This checks the assessment record and its evidence identity. It does not infer
performance equivalence or replace the source, proof and measurement audits.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

BASELINE = "a2da8e2b5de9a1a4653d3662f2f05cb6533fd7ef"
PRIORITIES = ["exactness", "completeness", "performance", "memory_use", "binary_size", "code_size"]
ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def qualification_inputs(root):
    """Include implementation, proof, test, workload and build/check definitions.

    Exclude the assessment record itself and generated measurement artifacts to
    avoid a self-referential hash. Evidence artifacts are checked separately.
    """
    patterns = [
        "src/**/*", "tests/**/*", "benches/**/*", "examples/**/*",
        "fuzz/fuzz_targets/**/*", "scripts/*.py", "scripts/*.sh",
        ".github/workflows/*.yml", ".github/workflows/*.yaml", "verification/*",
        ".cargo/**/*", "rust-toolchain", "rust-toolchain.toml",
        "Cargo.toml", "Cargo.lock", "build.rs", "fuzz/Cargo.toml", "fuzz/Cargo.lock",
        "promoted_slow_offenders.txt", "slow_performers.txt",
    ]
    sources = {path for pattern in patterns for path in root.glob(pattern)
               if path.is_file() and path != root / "verification/acceptance.json"
               and not {"target", "__pycache__", ".git"}.intersection(path.relative_to(root).parts)}
    return {str(path.relative_to(root)): sha256(path) for path in sorted(sources)}


def check_acceptance(root):
    report = json.loads((root / "verification/acceptance.json").read_text())
    if not isinstance(report, dict):
        raise RuntimeError("Baseline qualification must be an assessment object")
    if report.get("schema") != 1 or report.get("baseline") != BASELINE:
        raise RuntimeError("Baseline qualification has an invalid schema or reference commit")
    if report.get("priority_order") != PRIORITIES:
        raise RuntimeError("Baseline qualification does not preserve the ordered priorities")
    if report.get("status") != "accepted":
        raise RuntimeError("Baseline qualification is pending; empirical acceptance is not established")
    if report.get("unresolved") != []:
        raise RuntimeError("Baseline qualification has unresolved items or omits their ledger")
    inputs = qualification_inputs(root)
    if not inputs or report.get("candidate_sources") != inputs:
        raise RuntimeError("Baseline qualification is stale or does not cover the current source inputs")
    evidence = report.get("evidence")
    if not isinstance(evidence, dict) or not evidence:
        raise RuntimeError("Baseline qualification has no source-bound evidence artifacts")
    for name, expected in evidence.items():
        path = (root / name).resolve()
        if Path(name).is_absolute() or not path.is_relative_to(root.resolve()) or not path.is_file():
            raise RuntimeError(f"Invalid baseline evidence path: {name}")
        if sha256(path) != expected:
            raise RuntimeError(f"Baseline evidence artifact changed: {name}")
    assessments = report.get("assessments")
    if not isinstance(assessments, dict) or set(assessments) != set(PRIORITIES):
        raise RuntimeError("Baseline qualification must assess all six priorities")
    for priority in PRIORITIES:
        assessment = assessments[priority]
        if not isinstance(assessment, dict) or assessment.get("status") != "qualified":
            raise RuntimeError(f"Baseline assessment is incomplete: {priority}")
        rationale = assessment.get("rationale")
        references = assessment.get("evidence")
        if not isinstance(rationale, str) or not rationale.strip():
            raise RuntimeError(f"Baseline assessment has no rationale: {priority}")
        if not isinstance(references, list) or not references or any(
            not isinstance(name, str) or name not in evidence for name in references
        ):
            raise RuntimeError(f"Baseline assessment has missing evidence: {priority}")
    if qualification_inputs(root) != inputs:
        raise RuntimeError("Qualification inputs changed while checking baseline acceptance")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sources", action="store_true", help="print current qualification input hashes")
    args = parser.parse_args()
    if args.sources:
        print(json.dumps(qualification_inputs(ROOT), indent=2))
    else:
        check_acceptance(ROOT)
        print("Baseline assessment and evidence identity checks passed; review the recorded measurement conclusions.")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, ValueError, TypeError) as error:
        sys.exit(f"error: {error}")
