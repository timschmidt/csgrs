#!/usr/bin/env python3
"""Derive the upper-magnitude checkpoint from retained terminal evidence."""
import gzip
import hashlib
import json
from pathlib import Path
import re

EVIDENCE = Path(__file__).resolve().parent
SOURCE_HASH = "eb10458e49d2e3fac4501f0af9a9b7dbf9402b378cb9ad926feb6990a1a7719a"


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
                 for name, version in [("debug_scalar", 233), ("five_crate", 234), ("default_scalar", 235)]}
    for result in summaries.values():
        assert result["top_level_failed"] == result["child_process_failed"] == 0
    assert summaries["debug_scalar"]["top_level_passed"] == 929
    assert summaries["five_crate"]["top_level_passed"] == 5713
    assert summaries["five_crate"]["top_level_ignored"] == 33
    assert summaries["five_crate"]["child_process_passed"] == 2
    assert summaries["default_scalar"]["top_level_passed"] == 935
    assert summaries["default_scalar"]["child_process_passed"] == 2
    assert summaries["default_scalar"]["per_crate"]["hyperreal"]["unit_and_integration"] == 912
    assert summaries["default_scalar"]["per_crate"]["hyperreal"]["doctests"] == 23

    baseline = EVIDENCE / "typed-api-prototype-v215-magnitude-upper-regressions/stdout.log"
    sections = dict(re.findall(r"---- computable::node::magnitude_upper_bound_tests::(\w+) stdout ----\n(.*?)(?=\n----|\nfailures:|\Z)", baseline.read_text(), re.S))
    failures = {}
    for name, body in sections.items():
        lines = [line for line in body.splitlines() if line and not line.startswith(("thread ", "note:"))]
        failures[name] = len(lines)
        if name.startswith("public_rational_graph"):
            graph = {"invalid_facts": sum(": exact_msd=" in line for line in lines),
                     "weakened_facts": sum(": weakened " in line for line in lines),
                     "missing_available_facts": sum("missing available" in line for line in lines)}
    assert sorted(failures.values()) == [28, 80, 100, 152]
    assert graph == {"invalid_facts": 130, "weakened_facts": 14, "missing_available_facts": 8}
    coverage = {
        "status": "Six new deterministic regression tests pass in the isolated candidate",
        "contract_decision": "p4-contract-decision-v173.json",
        "source_record_sha256": SOURCE_HASH,
        "baseline_phase": baseline.parent.name, "baseline_stdout_sha256": digest(baseline),
        "baseline_failed_tests": 4, "baseline_observation_failures": failures,
        "baseline_graph_failure_classes": graph,
        "coverage": {
            "exact_rational_graphs": 64, "query_orders_per_graph": 2,
            "precisions": [-4, -64, -256], "exact_sample_error_checks": 384,
            "graph_fact_observations_with_serde": 704, "graph_strengthening_checks": 320,
            "cbor_round_trips_with_serde": 128, "cbor_recursion_limit": 1024,
            "scaled_radical_values": 30, "scaled_radical_cold_warm_observations": 60,
            "numeric_centers": 105, "numeric_precisions": [-2147483648, -65, 0, 65, 2147483647],
            "numeric_closed_interval_observations": 525,
            "valid_proof_permutations": 240, "publication_prefix_observations": 960,
            "rounded_scale_boundary_cases": 84, "rounded_scale_zero_controls": 1,
            "delayed_loose_vs_tight_upper_publication_schedules": 8,
        },
        "workload_preservation": [
            "Both signed 96-addition graphs are retained in full. The first v214 attempt hit JSON's default recursion limit; v215 uses CBOR's explicit 1024 limit without shrinking the graph.",
            "The graph matrix includes exact cancellation, inverse, square, both offsets, product signs and rational roots. Each query order builds a fresh fixture collection; related fixtures deliberately share child nodes within that collection.",
            "The v217 full unit run retained eight old assertions expecting the former estimates. Their final assertions enforce the upper-bound contract while preserving every original input, including the 32-term atan and 740-term asin/atanh workloads and their MPFR oracles.",
            "The previous 84 deep inverse identities, 336 perturbations, six publication tests and proof-reuse controls remain unchanged in scope.",
        ],
        "limits": [
            "Two named query orders and eight delayed writer schedules are not all possible interleavings.",
            "Decoded graphs have fact checks before/after successful approximation; the 384 independent one-unit numeric oracle checks apply to the original graph queries.",
            "These exact-rational and rational-square oracles cover the listed matrices, not every graph, magnitude producer, domain or private kernel.",
        ],
    }

    proof = completed("typed-api-prototype-v230-magnitude-denotation-verus")
    proof_data = read(proof / "proof-reports/evidence.json")
    results = proof_data["verification_results"]
    assert results["success"] and results["verified"] == 635 and results["errors"] == 0
    assert results["is-verifying-entire-crate"] and "--no-cheating" in proof_data["command"]
    scope = proof_data["scope"]
    assert len(scope["verified_executable_contracts"]) == 95
    assert len(scope["verified_model_theorems"]) == 125
    assert len(scope["obligations"]) == 13 and all(row["status"] != "complete" for row in scope["obligations"])
    guard = completed("typed-api-prototype-v229-proof-rejections")
    guard_data = read(guard / "proof-checks/tooling.json")
    assert guard_data["exit_code"] == 0 and guard_data["tool_unchanged"]
    assert guard_data["verus_invocations"] == 209
    assert guard_data["baseline_successes"] == 4 and guard_data["rejected_invocations"] == 205
    calls = [json.loads(line) for line in gzip.decompress((guard / "proof-checks/verus-invocations.jsonl.gz").read_bytes()).splitlines()]
    assert len(calls) == 209 and all("--no-cheating" in call["command"] for call in calls)
    assert len(re.findall(r"^Rejected ", (guard / "stdout.log").read_text(), re.M)) == 205
    acceptance = completed("typed-api-prototype-v231-proof-acceptance-tests")
    acceptance_text = (acceptance / "stderr.log").read_text()
    acceptance_count = int(re.search(r"Ran (\d+) tests in", acceptance_text)[1])
    assert acceptance_text.rstrip().endswith("OK")

    caps = {phase: capped(phase, count) for phase, count in [
        ("typed-api-prototype-v239-magnitude-cap", 5),
        ("typed-api-prototype-v240-publication-cap", 7),
        ("typed-api-prototype-v241-fractional-cap", 5),
    ]}
    replays = {}
    for name, phase in [("original", "typed-api-prototype-v243-replay"),
                        ("adjacent_precision_boundaries", "typed-api-prototype-v244-precision-replay")]:
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
    old = EVIDENCE / "typed-api-prototype-v179-stack-tests/stderr.log"
    new = completed("typed-api-prototype-v234-stack-tests") / "stderr.log"
    assert warnings(old) == warnings(new) and len(warnings(new)) == 14
    for phase in ["typed-api-prototype-v232-stack-clippy", "typed-api-prototype-v236-minimal-clippy"]:
        assert not warnings(completed(phase) / "stderr.log")
    fmt = read(completed("typed-api-prototype-v238-included-fmt") / "command.json")["command"]
    rust_count = len(fmt[fmt.index("--check") + 1:])
    assert rust_count == 76

    metadata = {
        "previous_checkpoint": "typed-api-prototype-v213-state.json",
        "status": "isolated certified upper-magnitude propagation and monotone merge repair; no runtime admission",
        "contract_decision": "p4-contract-decision-v173.json", "contract_commit": "8bf62fa3f",
        "changes": [
            "Every available MagnitudeBits exponent now certifies abs(value) < 2^(msd+1); exact_msd also certifies 2^msd <= abs(value). Smaller inexact exponents are stronger upper bounds.",
            "Square/product/same-sign addition account for carries; reciprocal upper bounds require an input lower-bound certificate. Whole exponent arithmetic is checked before narrowing.",
            "Numerical closed upper endpoints and non-dyadic Real scaling include their carries, including rounding before the final i32 range check.",
            "Atomic publication retains the minimum compatible upper exponent, preserves exact certificates, merges independent sign information and retries against the latest atomic word.",
            "The Verus denotation now constrains inexact exponents and proves conditional propagation/export. The stale cache mask model is corrected for the finite-real bit, with a preservation lemma and guard cases.",
            "Six new tests retain full original inputs; strict Clippy, debug scalar units, five-crate/default tests, formatting, 17 capped regressions and 475 original/precision replays pass on one frozen source.",
        ],
        "retained_regression_record": "magnitude-regression-coverage-v245.json",
        "retained_failures": {"v215_upper_bound_regressions": failures,
                              "v217_old_estimate_assertions": {"passed": 919, "failed": 8},
                              "v228_mutation_anchor_after_formatting": {"rejected_mutants": 139, "passing_baselines": 2, "finding": "Formatting split the new inverse mutation anchor; no incorrect proof was accepted. The run stopped before the remaining cases and is not a passing validation."},
                              "v219_to_v224_proof_preparation_and_development": "Terminal failures and exact source archives remain retained, including the stale finite-real mask and four incomplete nonlinear proof steps."},
        "new_matrix_coverage": coverage["coverage"],
        "formal_validation": {
            "phase": proof.name, "verification_units": 635, "errors": 0,
            "listed_production_contracts": 95, "listed_model_theorems": 125,
            "open_obligation_groups": 13, "rejection_phase": guard.name,
            "rejected_mutants": 205, "passing_guard_baselines": 4,
            "new_rejections": 13, "acceptance_and_dependency_guard_tests": acceptance_count,
            "adapted_existing_guards": "Seven stale source anchors were updated for the new contract. The former square/product tests treated accepting inexact inputs as a defect; upper-bound propagation now permits those inputs, so their replacement mutations omit available upper bounds and must violate the complete executable contracts. All 54 bound guard slots remain, with ten additional magnitude cases and three additional finite-real cache cases.",
            "proof_runner_sha256": digest(proof / "proof-reports/runner.py"),
            "guard_invocations_sha256": digest(guard / "proof-checks/verus-invocations.jsonl.gz"),
            "limits": "Conditional bound transforms and cache-word expressions are proved. BigInt endpoint production, rational scale rounding, graph/domain inputs, atomic CAS loops, arbitrary kernel convergence and full empirical acceptance remain open; --require-complete is not claimed.",
        },
        "test_summaries": {name: {"record": f"typed-api-prototype-v{version}-test-summary.json",
                                  "passed": summaries[name]["top_level_passed"],
                                  "child_passed": summaries[name]["child_process_passed"],
                                  "ignored": summaries[name]["top_level_ignored"]}
                           for name, version in [("debug_scalar", 233), ("five_crate", 234), ("default_scalar", 235)]},
        "capped_unit_regressions": caps, "final_source_replays": replays,
        "included_rust_fragments_checked": rust_count,
        "inherited_warnings": {"baseline_phase": old.parent.name, "current_phase": new.parent.name,
                               "identical_warning_lines": warnings(new), "strict_clippy_warning_lines": 0},
        "coverage_limits": coverage["limits"] + [
            "The capped tests are scoped correctness/resource controls, not matched performance comparisons or a total resource guarantee.",
            "Finite-real certificates do not establish private-kernel fine-precision convergence. Deferred trig poles and aggregate work/allocation budgets remain open.",
            "Main runtime and all 49 original worktree entries are preserved; the scalar candidate remains isolated and unadmitted.",
        ],
        "remaining_source_review": "magnitude-resource-followup-v245.json",
        "inherited_source_review": "domain-resource-followup-v155.json",
        "remaining_requirements": [
            "Execute the scaled-view lower-exponent exhaustion witness in magnitude-resource-followup-v245.json; a stronger base certificate may make the transformed field unavailable even while its earlier coarse upper bound remains valid. This is a source-derived follow-up, not executed coverage.",
            "Continue source-derived raw/serialized fine-kernel and deferred tangent/cotangent pole regressions with independent enclosures and unchanged full-resolution controls.",
            "Continue public query, serialization and concurrent certificate compatibility schedules, aggregate proof/evaluation/coefficient/allocation accounting, and implementation-connected proof obligations.",
            "Combine independently with the native prototype; retain full-board reports and proportionate timing/allocation/size evidence before production admission. Replay scalar failures after further runtime changes.",
            "All 39 properties, 14 targets, crate future work and full pre-push verification remain required. No push is requested or claimed.",
        ],
    }
    write("magnitude-regression-coverage-v245.json", coverage)
    write("typed-api-prototype-v245-metadata.json", metadata)
    print(json.dumps({"five_crate_passed": 5713, "default_scalar_passed": 935,
                      "debug_scalar_passed": 929, "verification_units": 635,
                      "rejected_mutants": 205, "capped_tests": 17, "replays": 475}))


if __name__ == "__main__":
    main()
