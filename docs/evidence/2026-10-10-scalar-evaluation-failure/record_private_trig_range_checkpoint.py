#!/usr/bin/env python3
"""Derive the trig-range checkpoint from terminal guarded validations."""
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import re
import sys

EVIDENCE = Path(__file__).resolve().parent
SOURCE_HASH = "7a923346333383717557886f5ac8d669245e116ffab8f0e83f50c20a33d80bd5"


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
    assert len(sys.argv) == 2, "implementation commit required"
    summaries = {name: read(EVIDENCE / f"typed-api-prototype-v{version}-test-summary.json")
                 for name, version in [("debug_scalar", 312), ("five_crate", 313), ("default_scalar", 314)]}
    for result in summaries.values():
        assert result["top_level_failed"] == result["child_process_failed"] == 0
    assert summaries["debug_scalar"]["top_level_passed"] == 942
    assert summaries["five_crate"]["top_level_passed"] == 5726
    assert summaries["five_crate"]["top_level_ignored"] == 33
    assert summaries["five_crate"]["child_process_passed"] == 2
    assert summaries["default_scalar"]["top_level_passed"] == 947
    assert summaries["default_scalar"]["child_process_passed"] == 2
    assert summaries["default_scalar"]["per_crate"]["hyperreal"]["unit_and_integration"] == 924
    assert summaries["default_scalar"]["per_crate"]["hyperreal"]["doctests"] == 23

    baseline = read(EVIDENCE / "typed-api-prototype-v301-private-trig-wide-baseline/summary.json")
    assert baseline["cases"] == 384 and baseline["incomplete_cases"] == 24
    assert baseline["numeric_samples"] == 1440
    assert baseline["invalid_numeric_samples"] == baseline["typed_errors"] == 0
    baseline_rows = read(EVIDENCE / "typed-api-prototype-v301-private-trig-wide-baseline/receipts.json")
    failed = [row for row in baseline_rows if not row["complete"]]
    assert len(failed) == 24 and all(row["mode"] == "serialized" and row["input"] in {"-1048576", "1048576"}
                                     and row["command"]["exit_code"] == -24 for row in failed)
    proof = read(EVIDENCE / "private-trig-range-proof-reuse-v330.json")
    assert proof["source_record_sha256"] == SOURCE_HASH
    assert proof["verified_units"] == 638 and proof["errors"] == 0
    assert proof["reused_rejected_mutants"] == 213 and proof["new_rejection_invocations"] == 0
    caps = {phase: capped(phase, count) for phase, count in [
        ("typed-api-prototype-v318-private-kernel-cap", 7),
        ("typed-api-prototype-v319-magnitude-cap", 5),
        ("typed-api-prototype-v320-publication-cap", 7),
        ("typed-api-prototype-v321-fractional-cap", 5),
        ("typed-api-prototype-v322-scaled-lower-cap", 1),
        ("typed-api-prototype-v323-trig-range-cap", 5),
    ]}
    replays = {}
    for phase in ["typed-api-prototype-v325-replay", "typed-api-prototype-v326-precision-replay",
                  "typed-api-prototype-v327-private-kernel-replay", "typed-api-prototype-v328-private-trig-replay",
                  "typed-api-prototype-v329-private-trig-wide-replay"]:
        path = EVIDENCE / phase / "summary.json"
        data = read(path)
        assert data["source_hashes_sha256"] == SOURCE_HASH and data["guards_unchanged"]
        replays[phase] = {"summary_sha256": digest(path)}
        if phase.endswith("v325-replay"):
            assert data["ordinary_cases"] == 400 and data["typed_interruption_errors"] == 300
            assert data["invalid_first_contracts"] == data["invalid_retries"] == 0
            assert len(data["extremes"]) == 6 and all(row["valid"] for row in data["extremes"])
            assert len(data["cache"]) == 9 and all(row["valid"] for row in data["cache"])
            assert all(row["exit_code"] == 0 for row in data["commands"])
            replays[phase]["cases"] = 415
        elif phase.endswith("v326-precision-replay"):
            assert data["cases"] == data["valid_cases"] == 60
            assert not data["nonzero_exit_cases"] and not data["invalid_numeric_cases"]
            replays[phase]["cases"] = 60
        elif phase.endswith("v327-private-kernel-replay"):
            assert data["cases"] == 78 and data["incomplete_cases"] == data["unexpected_result_shapes"] == 0
            replays[phase]["cases"] = 78
            replays[phase]["coverage"] = "Termination and typed-domain result shapes; accuracy assessed separately by directed MPFR unit tests."
        else:
            count, samples = (384, 1536) if "wide" in phase else (600, 2400)
            assert data["cases"] == count and data["numeric_samples"] == samples
            assert data["incomplete_cases"] == data["invalid_numeric_samples"] == data["typed_errors"] == 0
            replays[phase] |= {"cases": count, "numeric_observations": samples}
    assert sum(row["cases"] for row in replays.values()) == 1537
    old_wide = EVIDENCE / "typed-api-prototype-v301-private-trig-wide-baseline"
    new_wide = EVIDENCE / "typed-api-prototype-v329-private-trig-wide-replay"
    for name in ["probe.rs", "reproduce.py", "oracles.json"]:
        assert digest(old_wide / name) == digest(new_wide / name), name
    final_rows = read(new_wide / "receipts.json")
    case_keys = ["family", "mode", "input", "precision", "oracle_key"]
    cap_keys = ["timeout_seconds", "address_space_cap_mib", "cpu_seconds"]
    assert len(baseline_rows) == len(final_rows) == 384
    for old, new in zip(baseline_rows, final_rows):
        assert all(old[key] == new[key] for key in case_keys)
        assert all(old["command"][key] == new["command"][key] for key in cap_keys)
        assert new["complete"] and new["invalid_numeric_samples"] == new["typed_errors"] == 0
    warnings = lambda path: [line for line in path.read_text().splitlines() if line.startswith("warning:")]
    old_warnings = warnings(EVIDENCE / "typed-api-prototype-v284-stack-tests/stderr.log")
    new_warnings = warnings(completed("typed-api-prototype-v313-stack-tests") / "stderr.log")
    assert old_warnings == new_warnings and len(new_warnings) == 14
    for phase in ["typed-api-prototype-v311-stack-clippy", "typed-api-prototype-v315-minimal-clippy"]:
        assert not warnings(completed(phase) / "stderr.log")
    fmt = read(completed("typed-api-prototype-v317-included-fmt") / "command.json")["command"]
    rust_count = len(fmt[fmt.index("--check") + 1:])
    assert rust_count == 81
    cache_schedules = 0
    for kind in range(4):
        divisor = 64 if kind < 2 else 16
        for numerator in [-513, -512, -511, -193, -192, -191, 191, 192, 193, 511, 512, 513]:
            for rounding in range(-1, 2):
                if abs(Fraction(numerator // divisor + rounding) - Fraction(numerator, divisor)) <= 1:
                    cache_schedules += 4
    assert cache_schedules == 448
    changes = [
        "Generic and rational sin/cos kernels establish their reduced range at demand time. Larger serialized arguments use the existing reducers; all global coarse bounds remain available.",
        "Sinc/cosc establish a three-quarter-unit Taylor range from a valid rough interval. A sample outside that certificate separates the denominator from zero and permits a fallible reduced ordinary quotient.",
        "Trig magnitude planning uses checked bit-length and exponent addition. Optional rational power-of-two expansion checks the existing 1048576-bit numeric precision ceiling before allocation and uses unsigned_abs for i32::MIN; inability to materialize a coefficient leaves the exact Offset graph intact.",
        "Nested negation reduction propagates observed cancellation through the fallible reducer. Deferred sin/cos answer globally bounded coarse requests after operand existence has been established, avoiding needless reduction at extreme scales.",
    ]
    limits = [
        "These tests cover the retained finite matrices, not arbitrary trig arguments, all serialized variants, every cache schedule or a universal series error/work proof.",
        "Extreme-offset controls establish typed construction, retained input sign facts, globally bounded coarse values and invalid-descendant rejection. Fine numerical evaluation and all symbolic simplifications at oversized scales remain unproved. Optional coefficient recognition now declines shifts beyond the numeric precision ceiling; a compressed symbolic certificate may be needed to preserve completeness there before admission.",
        "Coefficient expansion limits and per-search limits do not form a shared proof, arithmetic, allocation or time budget. A maximum permitted power-of-two coefficient occupies 1048577 bits, not 4096 bits.",
        "The new runtime helpers, range guards and transcendental identities are outside the unchanged conditional Verus target. The 213 negative guards are reused with identical inputs, not rerun; all thirteen whole-crate proof groups remain open.",
        "Scoped process caps and correctness replays are not matched performance evidence or runtime admission. Native-prototype combination and full-board timing/allocation/size evidence remain required.",
    ]
    coverage = {
        "status": "Repaired reproduced serialized trig work failures, extreme-offset overflow/allocation and swallowed cancellation in the isolated candidate",
        "contract_decision": "p4-contract-decision-v173.json", "source_record_sha256": SOURCE_HASH,
        "baseline": {"phase": "typed-api-prototype-v301-private-trig-wide-baseline", "cases": 384,
                     "serialized_cpu_cap_failures": 24, "completed_numeric_observations": 1440,
                     "invalid_completed_numeric_observations": 0,
                     "interpretation": "All public controls completed. Unreduced entire series can eventually converge; this baseline demonstrates cost failures, not mathematical divergence or wrong completed samples."},
        "wide_replay_comparison": {
            "probe_harness_and_oracle_bytes_identical": True,
            "case_order_parameters_and_caps_identical": True,
            "original_cpu_capped_cases_now_complete": 24,
            "all_cases_complete_with_valid_numeric_observations": 384,
        },
        "intermediate_failures": {
            "v302": "Debug overflow adding an extreme Offset exponent in known_msd_for_trig_reduction.",
            "v303": "After checked MSD arithmetic, the release range test passed before the extreme-offset test aborted on an enormous allocation.",
            "v304": "Debug backtrace identified negation of i32::MIN in power_of_two_rational through pi_acos_linear_form while reducing sine of a shifted half.",
            "v305": "Three tests passed; negative Add wrapped in Negate exposed an observed stop swallowed by a nested public trig constructor (sinc, -1048576, poll 16).",
            "v306": "All four then-present regressions passed after fallible negation reduction.",
            "v307": "All five regressions passed after adding coarse extreme-scale and retained-fact controls.",
        },
        "new_unit_coverage": {
            "test_functions": 5, "trig_families": 6, "signed_inputs_per_family": 33,
            "views": ["shared clone", "original shared cache", "public control", "CBOR reconstruction"],
            "precision_schedule": [1, 0, -4, -64, -256, -4], "numeric_enclosure_checks": 4752,
            "valid_boundary_cache_schedules": cache_schedules, "boundary_enclosure_checks": cache_schedules * 4,
            "observed_stop_sweep_fixtures": 28, "extreme_offset_typed_reduction_controls": 4,
            "oversized_scale_retained_fact_inputs": 8, "global_coarse_numeric_controls": 32,
            "undefined_descendant_controls": 2,
            "oracle": "1024-bit directed MPFR sin/cos endpoints converted to exact rationals, with exact signed division for sinc and exact (1-cos)/x^2 for cosc. Both endpoints must fit the returned one-unit interval.",
        },
        "changes": changes, "limits": limits,
        "workload_preservation": "All original v301 cases, public controls, moderate trig cases, private inverse/pole cases, signed 96-addition graphs, 740-term domain fixtures, full 192000-bit inverse series and cache/publication fixtures remain. Every retained 1537-case replay is rerun on the frozen final source.",
    }
    metadata = {
        "previous_checkpoint": "typed-api-prototype-v300-state.json",
        "implementation_commits": [sys.argv[1]],
        "implementation_records": ["private-trig-range-implementation-v312.json"],
        "status": "isolated trig range, extreme-offset and cancellation repairs; no runtime admission",
        "contract_decision": "p4-contract-decision-v173.json", "contract_commit": "8bf62fa3f",
        "changes": changes, "retained_regression_record": "private-trig-range-regression-coverage-v330.json",
        "new_matrix_coverage": coverage["new_unit_coverage"],
        "formal_validation": {key: proof[key] for key in [
            "fresh_verification_phase", "fresh_acceptance_phase", "verified_units", "errors",
            "listed_production_contracts", "listed_model_theorems", "open_obligation_groups",
            "reused_rejection_phase", "reused_rejected_mutants", "reused_passing_guard_baselines",
            "new_rejection_invocations", "fresh_acceptance_tests", "limits"]},
        "formal_reuse_record": "private-trig-range-proof-reuse-v330.json",
        "test_summaries": {name: {"record": f"typed-api-prototype-v{version}-test-summary.json",
                                  "passed": summaries[name]["top_level_passed"],
                                  "child_passed": summaries[name]["child_process_passed"],
                                  "ignored": summaries[name]["top_level_ignored"]}
                           for name, version in [("debug_scalar", 312), ("five_crate", 313), ("default_scalar", 314)]},
        "capped_unit_regressions": caps, "final_source_replays": replays,
        "included_rust_fragments_checked": rust_count,
        "inherited_warnings": {"identical_cargo_filename_warning_lines": new_warnings, "strict_clippy_warning_lines": 0},
        "coverage_limits": limits, "remaining_source_review": "private-trig-range-resource-followup-v330.json",
        "remaining_requirements": [
            "Test compressed wide/deep exact-pole and dyadic identities with finite controls; Unknown must not be mistaken for falsehood or credited as preserving all prior symbolic completeness.",
            "Continue composed planning with loose upper bounds, concurrent/reconstructed fact compatibility, shared resource accounting and implementation-connected proofs. All thirteen whole-crate proof groups remain open.",
            "Combine independently with the native prototype; preserve full-board reports and proportionate timing/allocation/size evidence before runtime admission.",
            "All 39 properties, 14 targets, crate future work and pre-push verification remain in scope. Main runtime and all 49 original worktree entries remain preserved. No push is requested or claimed.",
        ],
    }
    write("private-trig-range-regression-coverage-v330.json", coverage)
    write("typed-api-prototype-v330-metadata.json", metadata)
    print(json.dumps({"five_crate_passed": 5726, "default_scalar_passed": 947,
                      "debug_scalar_passed": 942, "capped_tests": 30, "replay_cases": 1537}))


if __name__ == "__main__":
    main()
