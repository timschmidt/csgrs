#!/usr/bin/env python3
"""Retain deterministic publication failures and their exact workload coverage."""
import hashlib
import json
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
from parse_test_results_v2 import parse_phase

EVIDENCE = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def phase(version):
    matches = [p for p in EVIDENCE.glob(f"typed-api-prototype-v{version}-*") if p.is_dir()]
    assert len(matches) == 1, (version, matches)
    return matches[0]


def lines(version, pattern):
    return re.findall(pattern, (phase(version) / "stdout.log").read_text(), re.M)


def main():
    out = EVIDENCE / sys.argv[1]
    assert not out.exists()
    runs = {}
    for version in range(191, 201):
        path = phase(version)
        result = json.loads((path / "result.json").read_text())
        assert result["source_unchanged"] and result["main_unchanged"]
        runs[path.name] = {
            "result": result, "source_record_sha256": digest(path / "source-hashes.json.gz"),
            "tests": parse_phase(path),
            "failed_test_names": lines(version, r"^test (.+) \.\.\. FAILED$"),
        }
    sign_failures = lines(191, r"^expected=.*$")
    assert len(sign_failures) == 5
    assert all("retained=None, later=None" in line for line in sign_failures)
    bound_failures = {}
    for block in re.split(r"\n---- ", (phase(194) / "stdout.log").read_text())[1:]:
        name = block.split(" stdout ----", 1)[0]
        bound_failures[name] = re.findall(r"^case=.*$", block, re.M)
    expected_counts = {
        "numeric_magnitude_exactness_respects_closed_error_interval_endpoints": 240,
        "numeric_magnitude_combines_with_a_structural_sign_certificate": 32,
        "delayed_bound_cannot_erase_an_independent_exact_magnitude_proof": 8,
    }
    assert {name.rsplit("::", 1)[-1]: len(rows) for name, rows in bound_failures.items()} == expected_counts
    reuse_failures = lines(198, r"^expected=.*$")
    assert len(reuse_failures) == 5
    assert runs[phase(198).name]["tests"]["top_level_failed"] == 2
    diagnostic = (phase(197) / "stderr.log").read_text()
    assert "missing metadata: depth=3, a=-3, b=1" in diagnostic
    assert "immediate=Some(Positive), numeric=None, exact=None" in diagnostic
    assert runs[phase(199).name]["tests"]["top_level_passed"] == 922
    assert runs[phase(199).name]["tests"]["top_level_failed"] == 0
    assert runs[phase(200).name]["tests"]["top_level_passed"] == 6
    assert runs[phase(200).name]["tests"]["top_level_failed"] == 0
    data = {
        "status": "Monotone certificate publication and retained-proof reuse repaired in an isolated candidate; production admission pending",
        "helper_sha256": digest(Path(__file__)),
        "contract_decision": "p4-contract-decision-v173.json",
        "comparison_base": "typed-api-prototype-v190-state.json",
        "reproduced_failures": {
            "lost_algebraic_signs": {"count": 5, "lines": sign_failures},
            "bound_and_numeric_certificate_failures": bound_failures,
            "ignored_bound_signs": {"count": 5, "lines": reuse_failures},
            "ignored_nonzero_without_orientation": {"count": 1, "phase": phase(198).name},
            "original_deep_inverse_identity": {
                "unit_phase": phase(196).name, "unit_passed": 919, "unit_failed": 1,
                "diagnostic_phase": phase(197).name,
                "diagnostic_lines": re.findall(r"^(?:missing metadata:|inverse child:).*$", diagnostic, re.M),
                "finding": "A positive bound certificate was retained while exact_sign returned None. The consumer did not reuse that available proof.",
            },
        },
        "new_coverage": {
            "new_test_functions_since_v190": 7,
            "delayed_unknown_sign_schedules": 5,
            "sign_oracles": ["0", "7/9", "-7/9", "2^-2000", "-2^-2000"],
            "exact_remainder_graph": "96 raw additions over pi, then subtraction of pi + 96 - expected",
            "delayed_bound_schedules": 8,
            "retained_bound_sign_queries": 5,
            "nonzero_without_sign_metadata_cases": 1,
            "signed_half_functions": ["atan", "asinh", "asin", "atanh"],
            "numeric_magnitude_queries": 32,
            "numeric_magnitude_precisions": [-16, -128],
            "query_orders": ["approximation first", "structural sign first"],
            "closed_interval_centers": 105,
            "closed_interval_precisions": [-2147483648, -65, 0, 65, 2147483647],
            "closed_interval_observations": 525,
            "delayed_coarse_after_fine_evaluations": 4,
            "coarse_precision": -4, "fine_precision": -128,
            "retained_original_deep_inverse_identities": 84,
            "retained_original_perturbations": 336,
        },
        "repairs": [
            "Sign publication rechecks retained valid evidence after every CAS failure. Explicit destructive resets are test-only.",
            "Bound publication combines known sign and magnitude fields, preserves exact magnitudes, and preserves unrelated metadata bits. Existing inexact planning hints are retained until an exact proof supersedes them.",
            "Separated one-unit intervals certify an exact MSD only when both closed endpoints have the same representable exponent. Trailing-bit inspection avoids allocating endpoint integers.",
            "Root and descendant sign queries reuse retained bound sign certificates; reciprocal separation metadata accepts a proved nonzero denominator without requiring its orientation.",
            "Test-only scheduling hooks enforce delayed Unknown, delayed magnitude and delayed coarse numeric publication. They are absent from production builds.",
        ],
        "runs": runs,
        "limits": [
            "The numerical certificate assumes the successful approximation obeys its one-unit contract; arbitrary raw/serialized private-kernel convergence still needs independent validation.",
            "The tests cover the retained schedules and exact interval cases; they do not establish all schedules, a formal memory-model proof, or a global resource bound.",
            "The public meaning and propagation of inexact magnitude information remains a separate audit; this record does not claim all such fields are mathematically ordered bounds.",
            "Final-source full-stack validation, capped runs and 475 replays are recorded separately in the checkpoint.",
            "All 39 properties, 14 targets and crate future work remain in scope; no production admission or push is claimed.",
        ],
    }
    out.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"runs": len(runs), "lost_signs": 5, "lost_magnitudes": 8,
                      "missing_combined_magnitudes": 32, "missing_exact_interval_certificates": 240,
                      "ignored_bound_signs": 5, "ignored_nonzero_proofs": 1,
                      "closed_interval_observations": 525}))


if __name__ == "__main__":
    main()
