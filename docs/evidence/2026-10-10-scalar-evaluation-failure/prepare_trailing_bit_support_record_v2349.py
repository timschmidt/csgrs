"""Correct the evidence reader's function-versus-verification-unit assumption."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    previous = E / "record_trailing_bit_support_v2348.py"
    runner = E / "record_trailing_bit_support_v2349.py"
    failure = E / "trailing-bit-support-reader-failure-v2348.json"
    assert not runner.exists() and not failure.exists()
    assert not (E / "trailing-bit-support-milestone-v2348.json").exists()
    report = E / "trailing-bit-factorization-v2348/stdout.json"
    proof = json.loads(report.read_text())["verification-results"]
    assert proof["verified"] == 16 and proof["errors"] == 0 and proof["success"]
    text = previous.read_text()
    changes = [
        ('"trailing-bit-support-milestone-v2348.json"', '"trailing-bit-support-milestone-v2349.json"'),
        ('for name, expected in [("trailing-bit-support-v2346", 0),\n                           ("trailing-bit-factorization-v2347", 1),\n                           ("trailing-bit-factorization-v2348", 0)]:',
         'for name, expected, units in [("trailing-bit-support-v2346", 0, 4),\n                                  ("trailing-bit-factorization-v2347", 1, 0),\n                                  ("trailing-bit-factorization-v2348", 0, 16)]:'),
        ('assert proof["verified"] == (4 if expected == 0 else 0)', 'assert proof["verified"] == units'),
        ('passes four units and emits no warnings', 'passes 16 verification units across four proof functions and emits no warnings'),
        ('These are two separate four-unit support targets, not eight new retained production proofs.',
         'These are separate support targets with 4 and 16 verification units, including auxiliary assertion obligations; they add no retained production proof coverage.'),
        ('"phases": phases,', '"phases": phases, "reader_correction": "trailing-bit-support-reader-failure-v2348.json",\n        "reader_correction_sha256": sha(E / "trailing-bit-support-reader-failure-v2348.json"),'),
    ]
    for before, after in changes:
        assert text.count(before) == 1, before
        text = text.replace(before, after)
    ast.parse(text)
    runner.write_text(text)
    failure.write_text(json.dumps({"status": "Preserved evidence-reader failure; proof results are unchanged",
        "failed_runner": previous.name, "failed_runner_sha256": sha(previous),
        "observed_exit_code": 1, "observed_failure": "AssertionError at line 24: expected four verification units for a four-function target; the actual successful target reports sixteen units.",
        "actual_proof_report": str(report.relative_to(E)), "actual_proof_report_sha256": sha(report),
        "actual_verification_results": proof, "corrected_runner": runner.name,
        "corrected_runner_sha256": sha(runner), "preparation_sha256": sha(Path(__file__)),
        "scope": "The verifier also counts auxiliary assertion obligations. Correct the reader to the actual per-target counts 4, 0 and 16. No theorem, gate, source, result or retained coverage changes, and no proof rerun is necessary."}, indent=2) + "\n")
    print(json.dumps({"corrected_unit_counts": [4, 0, 16], "proof_reruns": 0}))


if __name__ == "__main__":
    main()
