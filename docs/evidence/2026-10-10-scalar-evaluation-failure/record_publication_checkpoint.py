#!/usr/bin/env python3
"""Derive the publication checkpoint metadata from terminal retained results."""
import hashlib
import json
from pathlib import Path
import re

EVIDENCE = Path(__file__).resolve().parent
SOURCE_HASH = "e07fb6d0e04157499f88311758b5bdf5843664223fa1715a4fd261b43ea2eb11"


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(name, value):
    out = EVIDENCE / name
    assert not out.exists(), out
    out.write_text(json.dumps(value, indent=2) + "\n")


def capped(phase, selected):
    path = EVIDENCE / phase
    result = read(path / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert digest(path / "source-hashes.json.gz") == SOURCE_HASH
    stdout = (path / "stdout.log").read_text()
    receipt = json.loads(stdout.splitlines()[-1])
    assert receipt["exit_code"] == 0 and not receipt["wall_timeout"]
    counts = re.findall(r"^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;", stdout, re.M)
    assert counts == [(str(selected), "0", "0")], (phase, counts)
    return receipt | {"selected_tests": selected, "passed": selected, "failed": 0, "ignored": 0,
                      "phase_result": result, "source_record_sha256": SOURCE_HASH}


def warning_record():
    old = EVIDENCE / "typed-api-prototype-v179-stack-tests/stderr.log"
    new = EVIDENCE / "typed-api-prototype-v203-stack-tests/stderr.log"
    warnings = lambda path: [line for line in path.read_text().splitlines()
                             if line.startswith("warning:")]
    lines = warnings(new)
    assert warnings(old) == lines and len(lines) == 14
    clippy = {}
    for phase in ["typed-api-prototype-v202-stack-clippy", "typed-api-prototype-v205-minimal-clippy"]:
        path = EVIDENCE / phase
        assert read(path / "result.json")["exit_code"] == 0
        assert digest(path / "source-hashes.json.gz") == SOURCE_HASH
        assert not warnings(path / "stderr.log")
        clippy[phase] = {"stderr_sha256": digest(path / "stderr.log"), "warning_lines": 0}
    return {
        "status": "Cargo example-output filename warnings unchanged from the previous retained five-crate run",
        "baseline_phase": old.parent.name, "current_phase": new.parent.name,
        "baseline_stderr_sha256": digest(old), "current_stderr_sha256": digest(new),
        "identical_warning_lines": lines, "warning_count": len(lines), "strict_clippy": clippy,
        "scope": "No Cargo manifests or example names/paths changed. Both strict Clippy configurations pass without warning lines.",
    }


def main():
    regression = read(EVIDENCE / "publication-regression-coverage-v200.json")
    stack = read(EVIDENCE / "typed-api-prototype-v203-test-summary.json")
    default = read(EVIDENCE / "typed-api-prototype-v204-test-summary.json")
    for summary in [stack, default]:
        assert summary["top_level_failed"] == summary["child_process_failed"] == 0
        assert summary["child_process_passed"] == 2
    assert stack["top_level_passed"] == 5707 and stack["top_level_ignored"] == 33
    assert default["top_level_passed"] == 929 and default["top_level_ignored"] == 0
    assert default["per_crate"]["hyperreal"]["unit_and_integration"] == 906
    assert default["per_crate"]["hyperreal"]["doctests"] == 23

    replays = {}
    phase = "typed-api-prototype-v211-replay"
    path = EVIDENCE / phase / "summary.json"
    replay = read(path)
    assert replay["source_hashes_sha256"] == SOURCE_HASH and replay["guards_unchanged"]
    assert replay["ordinary_cases"] == 400 and replay["typed_interruption_errors"] == 300
    assert replay["invalid_first_contracts"] == replay["invalid_retries"] == 0
    assert len(replay["extremes"]) == 6 and all(row["valid"] for row in replay["extremes"])
    assert len(replay["cache"]) == 9 and all(row["valid"] for row in replay["cache"])
    assert all(row["exit_code"] == 0 for row in replay["commands"])
    replays["original"] = {
        "phase": phase, "summary_sha256": digest(path), "ordinary_cases": 400,
        "invalid_first_contracts": 0, "invalid_retries": 0, "typed_interruption_errors": 300,
        "extreme_cases": 6, "valid_extreme_cases": 6, "cache_cases": 9, "valid_cache_cases": 9,
        "source_hashes_sha256": SOURCE_HASH, "address_space_cap_mib": 192,
        "per_case_wall_timeout_seconds": 3,
    }
    phase = "typed-api-prototype-v212-precision-replay"
    path = EVIDENCE / phase / "summary.json"
    replay = read(path)
    assert replay["source_hashes_sha256"] == SOURCE_HASH and replay["guards_unchanged"]
    assert replay["cases"] == replay["valid_cases"] == 60
    assert not replay["nonzero_exit_cases"] and not replay["invalid_numeric_cases"]
    replays["adjacent_precision_boundaries"] = {
        "phase": phase, "summary_sha256": digest(path), "cases": 60, "valid_cases": 60,
        "source_hashes_sha256": SOURCE_HASH, "address_space_cap_mib": 192,
        "per_case_wall_timeout_seconds": 3,
    }

    caps = {phase: capped(phase, count) for phase, count in [
        ("typed-api-prototype-v208-publication-cap", 6),
        ("typed-api-prototype-v209-fractional-cap", 5),
    ]}
    format_command = read(EVIDENCE / "typed-api-prototype-v207-included-fmt/command.json")["command"]
    included_rust_count = len(format_command[format_command.index("--check") + 1:])
    assert included_rust_count == 69
    warnings = warning_record()
    metadata = {
        "previous_checkpoint": "typed-api-prototype-v190-state.json",
        "status": "isolated monotone concurrent certificate publication and proof-reuse repair; no runtime admission",
        "contract_decision": "p4-contract-decision-v173.json", "contract_commit": "8bf62fa3f",
        "changes": [
            "Atomic sign publication preserves valid proofs across delayed Unknown writers and rechecks every CAS retry. Destructive replacement remains available only for test fixtures.",
            "Atomic bounds combine known sign/nonzero information with independent magnitude evidence, preserve exact magnitude certificates and unrelated metadata bits, and keep existing inexact planning hints until an exact proof supersedes them.",
            "Separated one-unit numerical intervals publish an exact MSD when both closed endpoints share the same representable exponent. Trailing-bit inspection avoids allocating endpoint integers.",
            "Root and descendant sign queries reuse retained bound sign certificates. Reciprocal algebraic metadata requires nonzero status without unnecessarily requiring its orientation.",
            "Seven new tests retain five delayed sign schedules, eight delayed bound schedules, four coarse/fine schedules, 32 numerical magnitude queries, 525 closed interval observations and independent proof-reuse controls.",
            "The original 84 deep inverse identities and 336 perturbations remain unchanged. The metadata regression exposed by stronger bounds, its diagnostic source and both minimized failures remain archived.",
            "Final five-crate/default/minimal/format checks, eleven selected capped tests and all 475 original/precision replay cases pass on the same frozen source. Main runtime and original worktree entries remain preserved.",
        ],
        "retained_regression_record": "publication-regression-coverage-v200.json",
        "retained_failures": {
            "v191_delayed_sign_publication": {"failed_tests": 1, "lost_sign_certificates": 5},
            "v194_bound_publication": {"passed_tests": 1, "failed_tests": 3,
                "lost_exact_magnitudes": 8, "missed_numeric_magnitude_additions": 32,
                "missed_exact_closed_interval_certificates": 240, "closed_interval_observations": 525},
            "v196_original_deep_identity": {"passed_tests": 919, "failed_tests": 1},
            "v197_diagnostic": {"failed_tests": 1, "depth": 3, "a": -3, "b": 1,
                "finding": "A retained positive bound with no numeric cache was ignored by exact_sign."},
            "v198_bound_proof_reuse": {"failed_tests": 2, "ignored_bound_signs": 5,
                "ignored_nonzero_without_orientation": 1},
        },
        "new_matrix_coverage": regression["new_coverage"] | {"scalar_all_feature_unit_tests": 923},
        "capped_unit_regressions": caps,
        "test_summaries": {
            "five_crate": "typed-api-prototype-v203-test-summary.json",
            "five_crate_top_level_passed": stack["top_level_passed"],
            "five_crate_child_passed": stack["child_process_passed"],
            "five_crate_ignored": stack["top_level_ignored"],
            "default_scalar": "typed-api-prototype-v204-test-summary.json",
            "default_scalar_top_level_passed": default["top_level_passed"],
            "default_scalar_child_passed": default["child_process_passed"],
        },
        "included_rust_fragments_checked": included_rust_count,
        "validation_orchestration_correction": "publication-validation-count-v207.json",
        "final_source_replays": replays,
        "coverage_limits": [
            "P4 remains in progress. The retained deterministic publication schedules do not cover all concurrent public-query interleavings, serialization orders or all certificate combinations.",
            "Inexact magnitude fields remain unranked planning hints internally, while public documentation calls them conservative bounds. Source-derived rational counterexamples and a candidate upper-bound contract are recorded for the next independent regression/producer/consumer audit; no general inexact-bound correctness claim is made.",
            "The existing Verus magnitude model constrains exact MSDs only. This checkpoint does not claim new Verus verification or a formal atomic memory-model proof.",
            "Numerical exact-MSD certificates rely on the successful approximation one-unit contract. Mathematical domain certificates do not establish arbitrary raw/serialized private-kernel convergence; fine-kernel ranges and deferred exact trig poles remain open.",
            "Local structural visit/depth and precision ceilings do not account for aggregate proof, coefficient, allocation and evaluation work.",
            "The eleven capped tests pass under 192 MiB address space, 2/3-second CPU limits and a four-second wall timeout. These scoped workloads are not a matched performance comparison or a global resource guarantee.",
            "Fourteen Cargo example-output filename collision warnings match v179 exactly. Both strict Clippy configurations have no warning lines; Cargo manifests and CI configuration are unchanged.",
            "Independent native-prototype combination, complete board reports, proportionate timing/allocation/size evidence, Verus and the full pre-push workflow matrix remain required for their respective admission/completion claims.",
            "The candidate is isolated and unadmitted. No push was requested or attempted; all 39 properties, 14 targets and crate future work remain in scope.",
        ],
        "remaining_source_review": "publication-resource-followup-v203.json",
        "inherited_source_review": "domain-resource-followup-v155.json",
        "domain_inventory": "domain-inventory-v101.json",
        "inherited_warning_record": "typed-api-prototype-v213-inherited-warnings.json",
        "remaining_requirements": [
            "Execute exact-rational regressions for the public inexact magnitude contract, then repair and audit every producer, consumer, merge and overflow case while retaining valid stronger facts.",
            "Continue independent raw/serialized fine-kernel and deferred tangent/cotangent pole regressions, all cache/certificate combinations, serialization and remaining constructor/fact/algebra paths.",
            "Continue aggregate proof/evaluation, coefficient, allocation and graph accounting without reducing original workloads.",
            "Combine independently with the native prototype and retain full-board report/timing/allocation/size evidence before production admission; replay scalar failures after subsequent runtime changes.",
            "All 39 properties, 14 targets, crate future work and full pre-push verification remain required.",
        ],
    }
    write("typed-api-prototype-v213-inherited-warnings.json", warnings)
    write("typed-api-prototype-v213-metadata.json", metadata)
    print(json.dumps({"five_crate_passed": stack["top_level_passed"],
                      "default_scalar_passed": default["top_level_passed"],
                      "selected_capped_tests": sum(row["passed"] for row in caps.values()),
                      "replay_cases": 475, "inherited_warnings": warnings["warning_count"]}))


if __name__ == "__main__":
    main()
