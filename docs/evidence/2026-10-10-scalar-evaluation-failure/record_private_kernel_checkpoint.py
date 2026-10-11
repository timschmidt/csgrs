#!/usr/bin/env python3
"""Derive the private-kernel checkpoint only from terminal guarded records."""
import hashlib
import json
from pathlib import Path
import re

EVIDENCE = Path(__file__).resolve().parent
SOURCE_HASH = "d872075a2118b941bfbdddbfe6615893494d641cfd5fab72428c18d6bfdf43be"


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
    output = (path / "stdout.log").read_text()
    receipt = json.loads(output.splitlines()[-1])
    assert receipt["exit_code"] == 0 and not receipt["wall_timeout"]
    assert re.findall(r"^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;", output, re.M) == [(str(count), "0", "0")]
    return receipt | {"passed": count, "phase_result": read(path / "result.json")}


def main():
    summaries = {name: read(EVIDENCE / f"typed-api-prototype-v{version}-test-summary.json")
                 for name, version in [("debug_scalar", 283), ("five_crate", 284), ("default_scalar", 285)]}
    for result in summaries.values():
        assert result["top_level_failed"] == result["child_process_failed"] == 0
    assert summaries["debug_scalar"]["top_level_passed"] == 937
    assert summaries["five_crate"]["top_level_passed"] == 5721
    assert summaries["five_crate"]["top_level_ignored"] == 33
    assert summaries["five_crate"]["child_process_passed"] == 2
    assert summaries["default_scalar"]["top_level_passed"] == 942
    assert summaries["default_scalar"]["child_process_passed"] == 2
    assert summaries["default_scalar"]["per_crate"]["hyperreal"]["unit_and_integration"] == 919
    assert summaries["default_scalar"]["per_crate"]["hyperreal"]["doctests"] == 23
    baseline = read(EVIDENCE / "typed-api-prototype-v271-private-kernel-baseline/summary.json")
    assert baseline["cases"] == 78 and baseline["incomplete_cases"] == 16
    assert baseline["unexpected_result_shapes"] == 21
    coarse = EVIDENCE / "typed-api-prototype-v272-asinh-coarse-regression"
    assert read(coarse / "result.json")["exit_code"] == 101
    assert len(re.findall(r"^kind=[12], input=.*sample=0$", (coarse / "stdout.log").read_text(), re.M)) == 12
    proof = read(EVIDENCE / "private-kernel-proof-reuse-v300.json")
    assert proof["source_record_sha256"] == SOURCE_HASH
    assert proof["verified_units"] == 638 and proof["errors"] == 0
    assert proof["reused_rejected_mutants"] == 213 and proof["new_rejection_invocations"] == 0

    caps = {phase: capped(phase, count) for phase, count in [
        ("typed-api-prototype-v289-private-kernel-cap", 7),
        ("typed-api-prototype-v290-magnitude-cap", 5),
        ("typed-api-prototype-v291-publication-cap", 7),
        ("typed-api-prototype-v292-fractional-cap", 5),
        ("typed-api-prototype-v293-scaled-lower-cap", 1),
    ]}
    replays = {}
    for phase in ["typed-api-prototype-v295-replay", "typed-api-prototype-v296-precision-replay",
                  "typed-api-prototype-v297-private-kernel-replay", "typed-api-prototype-v298-private-trig-replay"]:
        path = EVIDENCE / phase / "summary.json"
        data = read(path)
        assert data["source_hashes_sha256"] == SOURCE_HASH and data["guards_unchanged"]
        replays[phase] = {"summary_sha256": digest(path)}
        if phase.endswith("v295-replay"):
            assert data["ordinary_cases"] == 400 and data["typed_interruption_errors"] == 300
            assert data["invalid_first_contracts"] == data["invalid_retries"] == 0
            assert len(data["extremes"]) == 6 and all(row["valid"] for row in data["extremes"])
            assert len(data["cache"]) == 9 and all(row["valid"] for row in data["cache"])
            assert all(row["exit_code"] == 0 for row in data["commands"])
            replays[phase]["cases"] = 415
        elif phase.endswith("v296-precision-replay"):
            assert data["cases"] == data["valid_cases"] == 60
            assert not data["nonzero_exit_cases"] and not data["invalid_numeric_cases"]
            replays[phase]["cases"] = 60
        elif phase.endswith("v297-private-kernel-replay"):
            assert data["cases"] == 78 and data["incomplete_cases"] == data["unexpected_result_shapes"] == 0
            replays[phase]["cases"] = 78
            replays[phase]["coverage"] = "Termination and typed-domain result shapes; accuracy assessed separately by directed MPFR unit tests."
        else:
            assert data["cases"] == 600 and data["numeric_samples"] == 2400
            assert data["incomplete_cases"] == data["invalid_numeric_samples"] == data["typed_errors"] == 0
            replays[phase]["cases"] = 600
            replays[phase]["numeric_observations"] = 2400
    warnings = lambda path: [line for line in path.read_text().splitlines() if line.startswith("warning:")]
    old_warnings = warnings(EVIDENCE / "typed-api-prototype-v255-stack-tests/stderr.log")
    new_warnings = warnings(completed("typed-api-prototype-v284-stack-tests") / "stderr.log")
    assert old_warnings == new_warnings and len(new_warnings) == 14
    for phase in ["typed-api-prototype-v282-stack-clippy", "typed-api-prototype-v286-minimal-clippy"]:
        assert not warnings(completed(phase) / "stderr.log")
    fmt = read(completed("typed-api-prototype-v288-included-fmt") / "command.json")["command"]
    rust_count = len(fmt[fmt.index("--check") + 1:])
    assert rust_count == 80
    coverage = {
        "status": "Confirmed inverse-series convergence/coarse-value and exact-pole defects repaired in the isolated candidate",
        "contract_decision": "p4-contract-decision-v173.json",
        "source_record_sha256": SOURCE_HASH,
        "baseline_phases": {
            "v270": "All 600 moderate trig cases and 2400 exact-series enclosure observations passed; no defect inferred from small-argument comments.",
            "v271": "78 convergence/domain cases: raw atan/asinh at +/-2 and +/-8 hit CPU caps; exact tangent poles hit caps; cotangent poles returned Exhausted. 16 incomplete cases, 21 unexpected shapes. Finite samples were not credited with numeric accuracy.",
            "v272": "The independent coarse asinh regression recorded 12 one-unit enclosure violations.",
            "harness_failures": {"v268": "rustfmt", "v269": "probe denominator type", "v273": "missing Ordering qualification in first repair"},
        },
        "new_unit_coverage": {
            "test_functions": 7,
            "inverse_families": ["PrescaledAtan", "PrescaledAsinh", "AsinhRational"],
            "inverse_inputs_per_family": 13,
            "inverse_numeric_enclosure_checks": 1092,
            "coarse_asinh_enclosure_checks": 54,
            "asinh_i32_max_precision_zero_controls": 18,
            "exact_pole_inputs_across_three_families": 19,
            "exact_pole_typed_error_checks": 456,
            "nearby_pole_inputs_across_three_families": 14,
            "nearby_pole_numeric_enclosure_checks": 336,
            "views": ["shared clone", "original shared cache", "public constructor/control", "CBOR reconstruction"],
            "independent_oracle": "1024-bit directed MPFR endpoints converted to exact rationals; every returned sample must enclose both endpoints within its one-unit contract. Tangent/cotangent oracles include directed pi input bounds and monotonicity on the chosen pole-adjacent interval.",
            "observed_stop_sweep_fixtures": {"inverse_reductions": 6, "trig_domain_admission_or_pole": 12},
            "exact_pi_lattice_recognizer_checks": 594,
            "retained_zero_fact_checks": 4,
            "extreme_shift_recognizer_unknown_controls": 2,
            "undefined_operand_annihilator_control": 1,
        },
        "repair_argument": [
            "The atan and asinh Taylor kernels require a proved half-unit input bound, using existing upper certificates, bounded leaf comparison, or a rough sample with its one-unit error. Larger atan arguments use existing range reduction.",
            "Larger asinh inputs use odd symmetry and the positive logarithm identity. Coarse requests coarsen a complete unit-scale approximation instead of assuming the globally unbounded function fits below two.",
            "Once operands are proved finite real, bounded rational pi-multiple identities prove exact sine/cosine zeros. Publishing the exact zero sign strengthens later facts and prevents futile nonzero-domain refinement at recognized poles.",
        ],
        "workload_preservation": "All original v270/v271 cases, constructor-valid controls, full 192000-bit inverse series, signed 96-addition graphs, 740-term domain fixtures and prior cache/publication/scaling fixtures remain. The final source replays every original 475 scalar/precision case plus the 78 private-kernel and 600 moderate trig cases.",
        "limits": [
            "The 600 moderate trig cases do not establish arbitrary-argument convergence, accuracy or work bounds for every prescaled trig kernel. Larger serialized arguments and other private variants remain open.",
            "The exact-pole recognizer is sufficient, not complete: coefficient/depth/work limits and unrecognized algebraic forms leave a domain question unresolved. Its extreme-shift controls test only bounded recognition, not complete numerical evaluation of those graphs.",
            "The new runtime helpers and trig identities are not part of the Verus target. The unchanged conditional target and negative guards do not prove these implementations or all input graphs.",
            "Resource caps are scoped correctness/termination controls, not matched performance evidence, aggregate budgets, exhaustive cache schedules or production admission.",
        ],
    }
    metadata = {
        "previous_checkpoint": "typed-api-prototype-v267-state.json",
        "implementation_commits": ["20431a43a", "f26f3d080"],
        "implementation_records": ["private-kernel-inverse-repair-v277.json", "private-kernel-pole-implementation-v283.json"],
        "status": "isolated inverse-kernel and exact-pole repairs; no runtime admission",
        "contract_decision": "p4-contract-decision-v173.json", "contract_commit": "8bf62fa3f",
        "changes": coverage["repair_argument"],
        "retained_regression_record": "private-kernel-regression-coverage-v300.json",
        "new_matrix_coverage": coverage["new_unit_coverage"],
        "formal_validation": {key: proof[key] for key in [
            "fresh_verification_phase", "fresh_acceptance_phase", "verified_units", "errors",
            "listed_production_contracts", "listed_model_theorems", "open_obligation_groups",
            "reused_rejection_phase", "reused_rejected_mutants", "reused_passing_guard_baselines",
            "new_rejection_invocations", "fresh_acceptance_tests", "limits"]},
        "formal_reuse_record": "private-kernel-proof-reuse-v300.json",
        "test_summaries": {name: {"record": f"typed-api-prototype-v{version}-test-summary.json",
                                  "passed": summaries[name]["top_level_passed"],
                                  "child_passed": summaries[name]["child_process_passed"],
                                  "ignored": summaries[name]["top_level_ignored"]}
                           for name, version in [("debug_scalar", 283), ("five_crate", 284), ("default_scalar", 285)]},
        "capped_unit_regressions": caps, "final_source_replays": replays,
        "included_rust_fragments_checked": rust_count,
        "inherited_warnings": {"identical_cargo_filename_warning_lines": new_warnings, "strict_clippy_warning_lines": 0},
        "coverage_limits": coverage["limits"],
        "remaining_source_review": "private-kernel-resource-followup-v300.json",
        "remaining_requirements": [
            "Continue full-range serialized fine-kernel tests with independent enclosures and unchanged public controls; cover bounded-recognition misses without treating Unknown as invalidity.",
            "Continue loose-upper-bound planning cost, public concurrent/reconstructed fact compatibility, shared resource accounting and implementation-connected proofs; all thirteen whole-crate obligation groups stay open.",
            "Combine independently with the native prototype; retain full-board report and proportionate timing/allocation/size evidence before runtime admission. Preserve all original failures and workloads.",
            "All 39 properties, 14 targets, crate future work and pre-push verification remain in scope. Main runtime and all 49 original worktree entries are preserved. No push is requested or claimed.",
        ],
    }
    write("private-kernel-regression-coverage-v300.json", coverage)
    write("typed-api-prototype-v300-metadata.json", metadata)
    print(json.dumps({"five_crate_passed": 5721, "default_scalar_passed": 942,
                      "debug_scalar_passed": 937, "capped_tests": 25,
                      "scalar_precision_replays": 475, "private_kernel_replays": 78,
                      "moderate_trig_replays": 600}))


if __name__ == "__main__":
    main()
