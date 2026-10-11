#!/usr/bin/env python3
"""Record terminal validation of compressed periodic angles with residuals."""
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

EVIDENCE = Path(__file__).resolve().parent
SOURCE_HASH = "ef53fadb5763cc8623b5176bd5b3ebb278772aeac3c4aaa9dfbed6e8130b0f06"
SOURCE_RECORD = "typed-api-prototype-v378-trig-residual-verus/source-hashes.json.gz"


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(name, data):
    path = EVIDENCE / name
    assert not path.exists(), path
    path.write_text(json.dumps(data, indent=2) + "\n")


def completed(phase):
    path = EVIDENCE / phase
    result = read(path / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert digest(path / "source-hashes.json.gz") == SOURCE_HASH
    return path


def main():
    assert len(sys.argv) == 2, "implementation commit required"
    summaries = {}
    for key, version, passed, child, ignored in [
        ("debug_scalar", 381, 953, 0, 0), ("five_crate", 382, 5737, 2, 33),
        ("default_scalar", 383, 958, 2, 0),
    ]:
        path = EVIDENCE / f"typed-api-prototype-v{version}-test-summary.json"
        data = read(path)
        assert data["top_level_passed"] == passed and data["child_process_passed"] == child
        assert data["top_level_failed"] == data["child_process_failed"] == 0
        assert data["top_level_ignored"] == ignored
        summaries[key] = {"record": path.name, "passed": passed, "child_passed": child, "ignored": ignored}
    scalar = read(EVIDENCE / summaries["default_scalar"]["record"])["per_crate"]["hyperreal"]
    assert scalar["unit_and_integration"] == 935 and scalar["doctests"] == 23

    caps = {}
    for version, suffix, count in [(387, "private-kernel", 7), (388, "magnitude", 5),
                                  (389, "publication", 7), (390, "fractional", 5),
                                  (391, "scaled-lower", 1), (392, "trig-range", 5),
                                  (393, "trig-lattice", 5), (394, "trig-residual", 6)]:
        phase = f"typed-api-prototype-v{version}-{suffix}-cap"
        path = completed(phase)
        log = (path / "stdout.log").read_text()
        receipt = json.loads(log.splitlines()[-1])
        assert receipt["exit_code"] == 0 and not receipt["wall_timeout"]
        assert re.findall(r"^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;", log, re.M) == [(str(count), "0", "0")]
        caps[phase] = receipt | {"passed": count, "phase_result": read(path / "result.json")}

    replays = {}
    for version, suffix, count in [(396, "replay", 415), (397, "precision-replay", 60),
                                  (398, "private-kernel-replay", 78), (399, "private-trig-replay", 600),
                                  (400, "private-trig-wide-replay", 384), (401, "symbolic-trig-scale-replay", 168),
                                  (402, "symbolic-trig-residual-replay", 456)]:
        phase = f"typed-api-prototype-v{version}-{suffix}"
        path = EVIDENCE / phase / "summary.json"
        data = read(path)
        assert data["source_hashes_sha256"] == SOURCE_HASH and data["guards_unchanged"]
        if version == 396:
            assert data["ordinary_cases"] == 400 and data["typed_interruption_errors"] == 300
            assert data["invalid_first_contracts"] == data["invalid_retries"] == 0
            assert len(data["extremes"]) == 6 and all(row["valid"] for row in data["extremes"])
            assert len(data["cache"]) == 9 and all(row["valid"] for row in data["cache"])
            assert all(row["exit_code"] == 0 for row in data["commands"])
        else:
            assert data["cases"] == count
            if version == 397:
                assert data["valid_cases"] == 60 and not data["nonzero_exit_cases"] and not data["invalid_numeric_cases"]
            elif version == 398:
                assert data["incomplete_cases"] == data["unexpected_result_shapes"] == 0
            elif version == 401:
                assert data["incomplete_cases"] == data["invalid_samples"] == 0
                assert data["valid_samples"] == 672
            else:
                assert data["incomplete_cases"] == data["invalid_numeric_samples"] == data["typed_errors"] == 0
                assert data["numeric_samples"] == count * 4
        replays[phase] = {"summary_sha256": digest(path), "cases": count}
    assert sum(row["cases"] for row in replays.values()) == 2161

    baseline = EVIDENCE / "typed-api-prototype-v366-symbolic-trig-residual-baseline"
    final = EVIDENCE / "typed-api-prototype-v402-symbolic-trig-residual-replay"
    old, new = read(baseline / "receipts.json"), read(final / "receipts.json")
    assert len(old) == len(new) == 456
    assert sum(not row["complete"] for row in old) == 36
    assert all(row["command"]["exit_code"] == -24 for row in old if not row["complete"])
    errors = Counter(sample["error"] for row in old for sample in row["samples"] if sample["error"])
    assert errors == {"Exhausted": 448, "OutOfRange": 448}
    assert sum(sample["numeric"] for row in old for sample in row["samples"]) == 819
    assert not any(row["invalid_numeric_samples"] for row in old)
    for name in ["probe.rs", "reproduce.py", "oracles.json"]:
        assert digest(baseline / name) == digest(final / name), name
    for a, b in zip(old, new):
        assert all(a[key] == b[key] for key in ["shape", "shift", "pi_coefficient", "residual", "family", "mode", "oracle_key"])
        assert a["command"]["command"][1:] == b["command"]["command"][1:]
        assert all(a["command"][key] == b["command"][key] for key in ["address_space_cap_mib", "cpu_seconds", "timeout_seconds"])
        assert b["complete"] and len(b["samples"]) == 4 and not b["typed_errors"] and not b["invalid_numeric_samples"]

    previous = read(EVIDENCE / "typed-api-prototype-v365-state.json")
    sources = json.loads(gzip.decompress((EVIDENCE / SOURCE_RECORD).read_bytes()))
    before = json.loads(gzip.decompress((EVIDENCE / previous["source_record"]).read_bytes()))
    changed = {name: {"before": before.get(name), "after": value} for name, value in sources.items() if before.get(name) != value}
    assert len(changed) == 6
    assert all(before[name] == sources[name] for name in before if "test" in name)
    for old_version, new_version, suffix in [(359, 396, "replay"), (361, 398, "private-kernel-replay"),
                                           (362, 399, "private-trig-replay"), (363, 400, "private-trig-wide-replay"),
                                           (364, 401, "symbolic-trig-scale-replay")]:
        a = EVIDENCE / f"typed-api-prototype-v{old_version}-{suffix}"
        b = EVIDENCE / f"typed-api-prototype-v{new_version}-{suffix}"
        for name in ["probe.rs", "reproduce.py"]:
            if (a / name).is_file():
                assert digest(a / name) == digest(b / name), (suffix, name)

    proof = read(EVIDENCE / "trig-residual-proof-reuse-v403.json")
    assert proof["source_record_sha256"] == SOURCE_HASH
    assert proof["verified_units"] == 638 and proof["errors"] == 0
    assert proof["reused_rejected_mutants"] == 213 and proof["new_rejection_invocations"] == 0
    warnings = lambda p: [line for line in p.read_text().splitlines() if line.startswith("warning:")]
    inherited = warnings(completed("typed-api-prototype-v382-stack-tests") / "stderr.log")
    assert inherited == warnings(EVIDENCE / "typed-api-prototype-v346-stack-tests/stderr.log") and len(inherited) == 14
    for phase in ["typed-api-prototype-v380-stack-clippy", "typed-api-prototype-v384-minimal-clippy"]:
        assert not warnings(completed(phase) / "stderr.log")
    fmt = read(completed("typed-api-prototype-v386-included-fmt") / "command.json")["command"]
    rust_count = len(fmt[fmt.index("--check") + 1:])
    assert rust_count == 85

    changes = [
        "Refactor the prior bounded total-arithmetic linear pi form without weakening half-pi lattice recognition. Keep checked binary exponents separate from bounded rational coefficients.",
        "Reduce twice the pi coefficient modulo four, admitting bounded negative exponents; reconstruct only bounded rational residuals. Exact nearest-quadrant arithmetic leaves a pi coefficient in [-1/4,1/4).",
        "Construct sine/cosine by exact angle addition and quadrant identities. Canonical small pi arguments are fixed points, and rational residuals use dedicated rational nodes so medium and large residuals cannot cycle through generic reduction.",
        "Use the rewrite in public fallible trig construction and private sine/cosine kernels, including raw nodes reached through flattened wrappers or tangent/cotangent quotients. Finite-domain validation, observed-stop propagation and numeric publication remain authoritative.",
    ]
    limits = [
        "The form still has local limits: 4096-bit coefficients/residuals, 16 live terms, depth 128 and 256 visits/products. Oversized rational residuals, extreme negative scales, unsupported symbolic or partial factors and deeper/wider forms may decline recognition. This is not a shared evaluator resource bound.",
        "The new normalization, modular residue, angle-addition and termination arguments are not connected to the conditional Verus target. Its 638 units and unchanged 213 negative guards do not prove the new runtime code; thirteen whole-crate obligation groups remain open.",
        "Capped correctness replays are not matched performance or runtime admission. Loose-upper-bound planning, shared arithmetic/proof/coefficient/allocation/time budgets, public concurrent/reconstructed fact schedules, native-prototype combination and full-board timing/allocation/size evidence remain required.",
    ]
    unit_coverage = {
        "test_functions": 6, "base_symbolic_inputs": 57, "trig_families": 4,
        "views": ["shared clone", "shared original", "public constructor", "CBOR reconstruction"],
        "precision_schedule": [0, -8, -256, -8],
        "base_directed_mpfr_enclosure_checks": 3648,
        "independent_exact_small_form_checks": 1183,
        "wrapper_and_large_cancellation_enclosure_checks": 192,
        "medium_residual_enclosure_checks": 2304,
        "canonical_fixed_point_controls": 3, "bounded_unknown_controls": 5,
        "invalid_descendant_typed_error_checks": 96, "observed_stop_sweep_fixtures": 12,
        "retained_sign_checks": "Every numerical check compares any before/after definite sign and verifies it against the independent enclosure; Unknown may strengthen.",
    }
    coverage = {
        "status": "Reproduced periodic-residual incompleteness repaired in the isolated candidate",
        "contract_decision": "p4-contract-decision-v173.json", "source_record_sha256": SOURCE_HASH,
        "baseline": {"phase": baseline.name, "commit": "4a0c0323c", "cases": 456, "cpu_cap_failures": 36,
                     "unexpected_typed_samples": dict(errors), "returned_numeric_samples": 819,
                     "invalid_returned_numeric_samples": 0,
                     "interpretation": "Completion/resource failures, with no wrong returned numeric sample observed."},
        "final_replay": {"phase": final.name, "cases": 456, "valid_numeric_observations": 1824,
                         "probe_harness_oracles_parameters_order_and_caps_preserved": True},
        "intermediate_failures": {
            "v367": "Compile error from using an unavailable Rational shift method; replaced with exact multiplication by one half.",
            "v368_v369": "New unit oracles reversed the parity of powers of two modulo six and parsed human mixed-fraction display as machine fractions. Corrected the table and imported exact numerator/denominator parts. The frozen external process oracle was already correct and unchanged.",
            "v370": "All five initial tests passed, before the wider debug suite exposed a reducer cycle.",
            "v375": "An unchanged coarse-cache boundary test overflowed its stack: generic reduction of medium rational residuals could re-enter the symbolic rewrite. The incomplete suite is retained, not counted as passing validation.",
            "v376": "After dedicated rational nodes removed that cycle and a sixth medium-residual regression was added, all 43 trig-filtered debug tests passed. All final-source validations were then rerun.",
        },
        "new_unit_coverage": unit_coverage, "changes": changes, "limits": limits,
        "workload_preservation": "All 1705 prior replay cases, complete 456-case residual baseline, earlier test sources, signed 96-addition graphs, 740-term domain fixtures, 192000-bit inverse series and cache/publication regressions are retained.",
    }
    scopes = {
        "node/trig_lattice.rs": "Complete shared bounded form, normalization, residue/reconstruction methods and retained lattice mapping.",
        "node/trig_periodic.rs": "Complete exact angle-addition rewrite, canonical fixed point, dedicated rational leaves, quadrant mapping and stop checks.",
        "node/exp_trig.rs": "New symbolic branches in fallible sin/cos/tan and neighboring reducers; half-pi quotient helpers and small-range checks read. No whole-file audit claimed.",
        "approximation/trig.rs": "Sine/cosine entry guards, tangent/cotangent dispatch, dedicated rational kernels and half-pi reduction helpers read in context. No whole-file audit claimed.",
        "node/trig_periodic_tests.rs": "Complete six-test file, independent directed/exact oracles, sign monotonicity, reconstructed/shared views, medium cycles, invalid operands, wrappers and stop sweeps.",
        "node/domain_validation.rs": "Existing finite-domain traversal and trig denominator checks read to verify the new rewrite cannot bypass required validation; source unchanged.",
    }
    reviewed = {"crates/hyperreal/src/computable/" + name: {"sha256": sources["crates/hyperreal/src/computable/" + name], "scope": scope} for name, scope in scopes.items()}
    followup = {
        "status": "Scoped source review; executed coverage is in the regression and state records",
        "previous_review": "trig-lattice-resource-followup-v365.json", "source_record": SOURCE_RECORD,
        "source_record_sha256": SOURCE_HASH, "reviewed_source_slices": reviewed,
        "resolved_witness": coverage["baseline"],
        "remaining_work": [
            "Extend preserved periodic/domain evidence to unsupported finite residuals/factors, deeper/wider forms, other private variants and nested cancellation; do not discard existing available certificates or fallback paths.",
            "Connect the bounded extraction, modular arithmetic, rational reconstruction, identities and termination arguments to implementation proofs while retaining all thirteen open whole-crate groups.",
            "Continue composed ln/exp/inverse-hyperbolic planning under loose magnitude bounds and public concurrent/reconstructed fact schedules. Preserve the P4 strengthening decision and the entire retained corpus.",
            "Establish shared budgets and matched cost evidence, combine the native prototype and replay unchanged full-board reports before runtime admission; complete all properties, targets and crate future work.",
        ], "limits": limits, "unadmitted": True,
        "scope_preserved": "All 39 properties, 14 targets, crate future work and 49 original worktree entries remain. P4 permits stronger retained proofs, never contradictory or weaker available facts.",
    }
    metadata = {
        "previous_checkpoint": "typed-api-prototype-v365-state.json", "implementation_commits": [sys.argv[1]],
        "baseline_commit": "4a0c0323c", "implementation_records": ["trig-residual-implementation-v381.json"],
        "status": "isolated compressed periodic-residual repair; no runtime admission",
        "contract_decision": "p4-contract-decision-v173.json", "contract_commit": "8bf62fa3f",
        "changes": changes, "retained_regression_record": "trig-residual-regression-coverage-v403.json",
        "new_matrix_coverage": unit_coverage,
        "formal_validation": {key: proof[key] for key in ["fresh_verification_phase", "fresh_acceptance_phase", "verified_units", "errors", "listed_production_contracts", "listed_model_theorems", "open_obligation_groups", "reused_rejection_phase", "reused_rejected_mutants", "reused_passing_guard_baselines", "new_rejection_invocations", "fresh_acceptance_tests", "limits"]},
        "formal_reuse_record": "trig-residual-proof-reuse-v403.json", "test_summaries": summaries,
        "capped_unit_regressions": caps, "final_source_replays": replays,
        "included_rust_fragments_checked": rust_count,
        "inherited_warnings": {"identical_cargo_filename_warning_lines": inherited, "strict_clippy_warning_lines": 0},
        "coverage_limits": limits, "remaining_source_review": "trig-residual-resource-followup-v403.json",
        "remaining_requirements": limits + ["All 39 properties, 14 targets, crate future work and pre-push verification remain required. Main runtime and all 49 original worktree entries are preserved. No push is requested or claimed."],
    }
    write("trig-residual-regression-coverage-v403.json", coverage)
    write("trig-residual-resource-followup-v403.json", followup)
    write("typed-api-prototype-v403-metadata.json", metadata)
    print(json.dumps({"five_crate_passed": 5737, "default_scalar_passed": 958,
                      "debug_scalar_passed": 953, "capped_tests": 41, "replay_cases": 2161}))


if __name__ == "__main__":
    main()
