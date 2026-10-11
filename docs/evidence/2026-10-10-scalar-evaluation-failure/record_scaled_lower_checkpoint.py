#!/usr/bin/env python3
"""Derive the scaled lower-exponent checkpoint from terminal validation records."""
import gzip
import hashlib
import json
from pathlib import Path
import re

EVIDENCE = Path(__file__).resolve().parent
SOURCE_HASH = "34117a772649183fdf14792e74c8f667903442b977970398aca650afdf78afa0"


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(name, value):
    path = EVIDENCE / name
    assert not path.exists(), path
    path.write_text(json.dumps(value, indent=2) + "\n")


def completed(phase):
    path = EVIDENCE / phase
    result = read(path / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert digest(path / "source-hashes.json.gz") == SOURCE_HASH
    return path


def capped(phase, count):
    path = completed(phase)
    text = (path / "stdout.log").read_text()
    receipt = json.loads(text.splitlines()[-1])
    assert receipt["exit_code"] == 0 and not receipt["wall_timeout"]
    assert re.findall(r"^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;", text, re.M) == [(str(count), "0", "0")]
    return receipt | {"selected_tests": count, "passed": count, "failed": 0, "ignored": 0,
                      "phase_result": read(path / "result.json"), "source_record_sha256": SOURCE_HASH}


def main():
    summaries = {name: read(EVIDENCE / f"typed-api-prototype-v{version}-test-summary.json")
                 for name, version in [("debug_scalar", 254), ("five_crate", 255), ("default_scalar", 256)]}
    for result in summaries.values():
        assert result["top_level_failed"] == result["child_process_failed"] == 0
    assert summaries["debug_scalar"]["top_level_passed"] == 930
    assert summaries["five_crate"]["top_level_passed"] == 5714
    assert summaries["five_crate"]["top_level_ignored"] == 33
    assert summaries["five_crate"]["child_process_passed"] == 2
    assert summaries["default_scalar"]["top_level_passed"] == 935
    assert summaries["default_scalar"]["child_process_passed"] == 2
    assert summaries["default_scalar"]["per_crate"]["hyperreal"]["unit_and_integration"] == 912
    assert summaries["default_scalar"]["per_crate"]["hyperreal"]["doctests"] == 23

    baseline = EVIDENCE / "typed-api-prototype-v246-scaled-lower-regression"
    baseline_result = read(baseline / "result.json")
    assert baseline_result["exit_code"] == 101
    assert baseline_result["source_unchanged"] and baseline_result["main_unchanged"]
    failures = re.findall(r"^sign=.*magnitude: None }$", (baseline / "stdout.log").read_text(), re.M)
    assert len(failures) == 12
    assert len(set(failures)) == 12
    repaired = EVIDENCE / "typed-api-prototype-v247-scaled-lower-repair"
    result = read(repaired / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    coverage = {
        "status": "Confirmed scaled-view certificate weakening repaired in the isolated candidate",
        "contract_decision": "p4-contract-decision-v173.json",
        "source_record_sha256": SOURCE_HASH,
        "baseline_phase": baseline.name, "baseline_stdout_sha256": digest(baseline / "stdout.log"),
        "baseline_failed_tests": 1, "baseline_weakened_observations": len(failures),
        "focused_repair_phase": repaired.name,
        "coverage": {
            "compact_base_exponent": -2147483648, "base_signs": [-1, 1],
            "rational_scales": ["1/2", "1/4", "3/8", "3/4"],
            "stronger_certificate_exactness": [False, True],
            "valid_publication_sequences": 16, "underflowing_sequences": 12, "control_sequences": 4,
            "public_fact_observations_including_clones_and_round_trips": 64,
            "clone_checks": 16, "cbor_round_trips": 16, "typed_precision_rejections": 16,
            "existing_rounding_boundary_cases_retained": 84, "zero_controls_retained": 1,
        },
        "repair_argument": "The bit-length difference plus signed offset is computed before narrowing. A mathematical upper exponent below i32::MIN implies that i32::MIN is still a valid coarser upper bound. Clamping clears the binade-preservation flag. Exponents above i32::MAX have no representable upper certificate and continue to return None. The original exact/floor query remains checked without clamping.",
        "workload_preservation": [
            "The original 16 fixtures retain both signs, three underflowing scales, one control and both valid stronger proof forms. Public multiplication shares the underlying Computable node, confirmed by the before-publication facts.",
            "The compact Offset graph denotes sign * 2^i32::MIN without materializing a 2^31-bit integer. Both incoming certificates independently follow from that exact denotation.",
            "The previous 84 rounding cases plus zero are retained; only their lower-overflow expected upper certificate changes. All earlier graph, cancellation, publication and precision workloads remain present.",
        ],
        "limits": [
            "Test-private publication installs independently valid certificates; this is not evidence of a public numerical cache schedule reaching i32::MIN. Numerical evaluation at that precision continues to return OutOfRange.",
            "The test exercises the public Real scaling/fact path, clones and cache-free reconstructed graphs. It does not establish every public cache schedule or concurrent interleaving.",
            "Lower clamping is added to rational upper-exponent conversion for scaled Real facts. It is not a claim that every BoundInfo transform retains every possible coarse exponent at a machine boundary.",
        ],
    }

    proof = completed("typed-api-prototype-v250-scaled-denotation-verus")
    proof_data = read(proof / "proof-reports/evidence.json")
    results = proof_data["verification_results"]
    assert results["success"] and results["verified"] == 638 and results["errors"] == 0
    assert results["is-verifying-entire-crate"] and "--no-cheating" in proof_data["command"]
    scope = proof_data["scope"]
    assert len(scope["verified_executable_contracts"]) == 96
    assert len(scope["verified_model_theorems"]) == 127
    assert len(scope["obligations"]) == 13 and all(row["status"] != "complete" for row in scope["obligations"])
    guard = completed("typed-api-prototype-v251-proof-rejections")
    guard_data = read(guard / "proof-checks/tooling.json")
    assert guard_data["exit_code"] == 0 and guard_data["tool_unchanged"]
    assert guard_data["verus_invocations"] == 217
    assert guard_data["baseline_successes"] == 4 and guard_data["rejected_invocations"] == 213
    calls = [json.loads(line) for line in gzip.decompress((guard / "proof-checks/verus-invocations.jsonl.gz").read_bytes()).splitlines()]
    assert len(calls) == 217 and all("--no-cheating" in call["command"] for call in calls)
    assert len(re.findall(r"^Rejected ", (guard / "stdout.log").read_text(), re.M)) == 213
    acceptance = completed("typed-api-prototype-v252-proof-acceptance-tests")
    acceptance_text = (acceptance / "stderr.log").read_text()
    acceptance_count = int(re.search(r"Ran (\d+) tests in", acceptance_text)[1])
    assert acceptance_count == 10 and acceptance_text.rstrip().endswith("OK")
    caps = {phase: capped(phase, count) for phase, count in [
        ("typed-api-prototype-v260-magnitude-cap", 5),
        ("typed-api-prototype-v261-publication-cap", 7),
        ("typed-api-prototype-v262-fractional-cap", 5),
        ("typed-api-prototype-v263-scaled-lower-cap", 1),
    ]}
    replays = {}
    for name, phase in [("original", "typed-api-prototype-v265-replay"),
                        ("adjacent_precision_boundaries", "typed-api-prototype-v266-precision-replay")]:
        path = EVIDENCE / phase / "summary.json"
        data = read(path)
        assert data["source_hashes_sha256"] == SOURCE_HASH and data["guards_unchanged"]
        if name == "original":
            assert data["ordinary_cases"] == 400 and data["typed_interruption_errors"] == 300
            assert data["invalid_first_contracts"] == data["invalid_retries"] == 0
            assert len(data["extremes"]) == 6 and all(row["valid"] for row in data["extremes"])
            assert len(data["cache"]) == 9 and all(row["valid"] for row in data["cache"])
            assert all(row["exit_code"] == 0 for row in data["commands"])
        else:
            assert data["cases"] == data["valid_cases"] == 60
            assert not data["nonzero_exit_cases"] and not data["invalid_numeric_cases"]
        replays[name] = {"phase": phase, "summary_sha256": digest(path),
                         "cases": 415 if name == "original" else 60,
                         "address_space_cap_mib": 192, "per_case_wall_timeout_seconds": 3}
    warnings = lambda path: [line for line in path.read_text().splitlines() if line.startswith("warning:")]
    old = EVIDENCE / "typed-api-prototype-v234-stack-tests/stderr.log"
    new = completed("typed-api-prototype-v255-stack-tests") / "stderr.log"
    assert warnings(old) == warnings(new) and len(warnings(new)) == 14
    for phase in ["typed-api-prototype-v253-stack-clippy", "typed-api-prototype-v257-minimal-clippy"]:
        assert not warnings(completed(phase) / "stderr.log")
    fmt = read(completed("typed-api-prototype-v259-included-fmt") / "command.json")["command"]
    rust_count = len(fmt[fmt.index("--check") + 1:])
    assert rust_count == 78
    metadata = {
        "previous_checkpoint": "typed-api-prototype-v245-state.json",
        "implementation_commit": "b6f0636da",
        "implementation_record": "scaled-lower-implementation-v254.json",
        "status": "isolated scaled-view lower-exponent certificate retention repair; no runtime admission",
        "contract_decision": "p4-contract-decision-v173.json", "contract_commit": "8bf62fa3f",
        "changes": [
            "Rational upper-exponent conversion preserves a coarse upper bound at i32::MIN on lower overflow and records that the exponent was clamped.",
            "Scaled Real facts preserve exactness only when the rational scale is a power of two and the composed exponent is representable without clamping. Exact/floor queries and numeric precision errors retain their previous contract.",
            "The formerly uncompiled 16-case draft is now a deterministic regression, with clones, CBOR reconstruction and typed precision controls added after confirming 12 weakened facts.",
            "The included production helper has a full-width Verus contract. Two conditional denotation lemmas and eight new defect mutations cover upper monotonicity, lower clamping, exactness and upper overflow.",
        ],
        "retained_regression_record": "scaled-lower-regression-coverage-v267.json",
        "retained_failures": {"v246_scaled_lower_regression": {"failed_tests": 1, "weakened_observations": 12}},
        "new_matrix_coverage": coverage["coverage"],
        "formal_validation": {
            "phase": proof.name, "verification_units": 638, "errors": 0,
            "listed_production_contracts": 96, "listed_model_theorems": 127,
            "open_obligation_groups": 13, "rejection_phase": guard.name,
            "rejected_mutants": 213, "passing_guard_baselines": 4, "new_rejections": 8,
            "acceptance_and_dependency_guard_tests": acceptance_count,
            "proof_runner_sha256": digest(proof / "proof-reports/runner.py"),
            "guard_invocations_sha256": digest(guard / "proof-checks/verus-invocations.jsonl.gz"),
            "limits": "The checked production exponent conversion preserves incoming certificates conditionally. The actual BigInt bit-length/alignment comparison and rational ceiling producer, graph/domain inputs, atomic CAS loops, private-kernel convergence and full empirical acceptance remain open; --require-complete is not claimed.",
        },
        "test_summaries": {name: {"record": f"typed-api-prototype-v{version}-test-summary.json",
                                  "passed": summaries[name]["top_level_passed"],
                                  "child_passed": summaries[name]["child_process_passed"],
                                  "ignored": summaries[name]["top_level_ignored"]}
                           for name, version in [("debug_scalar", 254), ("five_crate", 255), ("default_scalar", 256)]},
        "capped_unit_regressions": caps, "final_source_replays": replays,
        "included_rust_fragments_checked": rust_count,
        "inherited_warnings": {"baseline_phase": old.parent.name, "current_phase": new.parent.name,
                               "identical_warning_lines": warnings(new), "strict_clippy_warning_lines": 0},
        "coverage_limits": coverage["limits"] + [
            "The capped tests are scoped correctness/resource controls, not matched performance comparisons or a total resource guarantee.",
            "Main runtime and all 49 original worktree entries are preserved; the scalar candidate remains isolated and unadmitted.",
        ],
        "remaining_source_review": "scaled-lower-resource-followup-v267.json",
        "inherited_source_review": "domain-resource-followup-v155.json",
        "remaining_requirements": [
            "Continue source-derived raw/serialized fine-kernel and deferred tangent/cotangent pole regressions with independent enclosures and unchanged constructor-valid controls.",
            "Continue public query, serialization and concurrent certificate compatibility schedules, aggregate proof/evaluation/coefficient/allocation accounting, and implementation-connected proof obligations.",
            "Combine independently with the native prototype; retain full-board reports and proportionate timing/allocation/size evidence before production admission. Replay scalar failures after further runtime changes.",
            "All 39 properties, 14 targets, crate future work and full pre-push verification remain required. No push is requested or claimed.",
        ],
    }
    write("scaled-lower-regression-coverage-v267.json", coverage)
    write("typed-api-prototype-v267-metadata.json", metadata)
    print(json.dumps({"five_crate_passed": 5714, "default_scalar_passed": 935,
                      "debug_scalar_passed": 930, "verification_units": 638,
                      "rejected_mutants": 213, "capped_tests": 18, "replays": 475}))


if __name__ == "__main__":
    main()
