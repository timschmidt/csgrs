#!/usr/bin/env python3
"""Record only terminal evidence for the compressed trig lattice candidate."""
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

EVIDENCE = Path(__file__).resolve().parent
SOURCE_HASH = "3381f7d3041d457756034ff8be6081e5859cb1ef77566e64c89d5319cf121d6d"


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


def main():
    assert len(sys.argv) == 2, "implementation commit required"
    summaries = {}
    for name, version, passed, child, ignored in [
        ("debug_scalar", 345, 947, 0, 0), ("five_crate", 346, 5731, 2, 33),
        ("default_scalar", 347, 952, 2, 0),
    ]:
        path = EVIDENCE / f"typed-api-prototype-v{version}-test-summary.json"
        data = read(path)
        assert data["top_level_passed"] == passed and data["child_process_passed"] == child
        assert data["top_level_failed"] == data["child_process_failed"] == 0
        assert data["top_level_ignored"] == ignored
        summaries[name] = {"record": path.name, "passed": passed, "child_passed": child, "ignored": ignored}
    scalar = read(EVIDENCE / summaries["default_scalar"]["record"])["per_crate"]["hyperreal"]
    assert scalar["unit_and_integration"] == 929 and scalar["doctests"] == 23
    caps = {}
    for version, suffix, count in [(351, "private-kernel", 7), (352, "magnitude", 5),
                                  (353, "publication", 7), (354, "fractional", 5),
                                  (355, "scaled-lower", 1), (356, "trig-range", 5),
                                  (357, "trig-lattice", 5)]:
        phase = f"typed-api-prototype-v{version}-{suffix}-cap"
        path = completed(phase)
        log = (path / "stdout.log").read_text()
        receipt = json.loads(log.splitlines()[-1])
        assert receipt["exit_code"] == 0 and not receipt["wall_timeout"]
        assert re.findall(r"^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;", log, re.M) == [(str(count), "0", "0")]
        caps[phase] = receipt | {"passed": count, "phase_result": read(path / "result.json")}
    replays = {}
    for version, suffix, count in [(359, "replay", 415), (360, "precision-replay", 60),
                                   (361, "private-kernel-replay", 78), (362, "private-trig-replay", 600),
                                   (363, "private-trig-wide-replay", 384), (364, "symbolic-trig-scale-replay", 168)]:
        phase = f"typed-api-prototype-v{version}-{suffix}"
        path = EVIDENCE / phase / "summary.json"
        data = read(path)
        assert data["source_hashes_sha256"] == SOURCE_HASH and data["guards_unchanged"]
        if version == 359:
            assert data["ordinary_cases"] == 400 and data["typed_interruption_errors"] == 300
            assert data["invalid_first_contracts"] == data["invalid_retries"] == 0
            assert len(data["extremes"]) == 6 and all(row["valid"] for row in data["extremes"])
            assert len(data["cache"]) == 9 and all(row["valid"] for row in data["cache"])
            assert all(row["exit_code"] == 0 for row in data["commands"])
        else:
            assert data["cases"] == count
            if version == 360:
                assert data["valid_cases"] == 60 and not data["nonzero_exit_cases"] and not data["invalid_numeric_cases"]
            elif version == 361:
                assert data["incomplete_cases"] == data["unexpected_result_shapes"] == 0
            elif version == 364:
                assert data["incomplete_cases"] == data["invalid_samples"] == 0
                assert data["valid_samples"] == 672
            else:
                assert data["incomplete_cases"] == data["invalid_numeric_samples"] == data["typed_errors"] == 0
                assert data["numeric_samples"] == count * 4
        replays[phase] = {"summary_sha256": digest(path), "cases": count}
    assert sum(row["cases"] for row in replays.values()) == 1705

    baseline = EVIDENCE / "typed-api-prototype-v331-symbolic-trig-scale-baseline"
    final = EVIDENCE / "typed-api-prototype-v364-symbolic-trig-scale-replay"
    old, new = read(baseline / "receipts.json"), read(final / "receipts.json")
    assert len(old) == len(new) == 168
    assert sum(not row["complete"] for row in old) == 14
    assert all(row["command"]["exit_code"] == -24 for row in old if not row["complete"])
    errors = Counter(record["error"] for row in old for record in row["records"]
                     if record["stage"] == "sample" and not record["valid"])
    assert errors == {"Exhausted": 184, "OutOfRange": 112}
    assert not any(record["numeric"] and not record["valid"] for row in old for record in row["records"]
                   if record["stage"] == "sample")
    for name in ["probe.rs", "reproduce.py"]:
        assert digest(baseline / name) == digest(final / name)
    for a, b in zip(old, new):
        assert all(a[key] == b[key] for key in ["shape", "shift", "index", "family", "mode"])
        assert a["records"][0] == b["records"][0]
        assert all(a["command"][key] == b["command"][key]
                   for key in ["address_space_cap_mib", "cpu_seconds", "timeout_seconds"])
        assert b["complete"] and b["valid_samples"] == 4 and b["invalid_samples"] == 0
    proof = read(EVIDENCE / "trig-lattice-proof-reuse-v365.json")
    assert proof["source_record_sha256"] == SOURCE_HASH
    assert proof["verified_units"] == 638 and proof["errors"] == 0
    assert proof["reused_rejected_mutants"] == 213 and proof["new_rejection_invocations"] == 0
    warnings = lambda path: [line for line in path.read_text().splitlines() if line.startswith("warning:")]
    inherited = warnings(completed("typed-api-prototype-v346-stack-tests") / "stderr.log")
    assert inherited == warnings(EVIDENCE / "typed-api-prototype-v313-stack-tests/stderr.log") and len(inherited) == 14
    for phase in ["typed-api-prototype-v344-stack-clippy", "typed-api-prototype-v348-minimal-clippy"]:
        assert not warnings(completed(phase) / "stderr.log")
    fmt = read(completed("typed-api-prototype-v350-included-fmt") / "command.json")["command"]
    rust_count = len(fmt[fmt.index("--check") + 1:])
    assert rust_count == 83
    changes = [
        "A bounded exact form retains rational coefficients and binary exponents separately for total arithmetic over rational/pi leaves. Equal exponents combine, remove powers of two and carry, without expanding enormous binary shifts.",
        "After rational residuals cancel, modular exponentiation computes twice the pi coefficient modulo four. An integral residue proves an exact half-pi lattice point; unsupported syntax or exhausted limits returns Unknown.",
        "Public trig constructors reuse the exact identity, numerical queries retain exact sign facts and valid enclosures, and domain checks recognize the resulting exact zeros before attempting nonzero refinement. Existing recognizers remain as fallbacks.",
        "All operands are visited even when multiplied by zero; unsupported partial functions cannot be erased. The original extreme-shift and coarse-value fixtures remain, with stronger exact identities accepted under the same one-unit contract.",
    ]
    limits = [
        "Recognition covers bounded linear pi forms with total rational arithmetic: 4096-bit stored coefficients, 16 live terms, depth 128 and a shared 256-unit visit/product budget inside this recognizer. These bounds are not aggregate evaluator budgets or proofs of all symbolic completeness.",
        "General non-lattice periodic reduction, extreme scales plus nonzero residuals, unsupported partial/symbolic factors, oversized coefficients and deeper/wider equivalent forms remain open. Nearby finite MPFR controls use binary exponents 1, 8 and 16, not i32::MAX.",
        "The implementation and its modular/irrationality arguments are not yet connected to the conditional Verus target. The fresh target has 638 units; 213 negative guards reuse identical inputs. All thirteen whole-crate proof groups remain open.",
        "Process caps and correctness replays do not establish matched performance or runtime admission. Shared budgets, public concurrent/reconstructed schedules, native-prototype combination and full-board timing/allocation/size evidence remain required.",
    ]
    coverage = {
        "status": "Reproduced symbolic lattice incompleteness repaired in the isolated candidate",
        "contract_decision": "p4-contract-decision-v173.json", "source_record_sha256": SOURCE_HASH,
        "baseline": {"phase": baseline.name, "cases": 168, "cpu_cap_failures": 14,
                     "unexpected_typed_samples": dict(errors), "invalid_returned_numeric_samples": 0,
                     "interpretation": "Exact values and poles failed to complete or returned resource/range errors; no wrong returned number was observed."},
        "final_replay": {"phase": final.name, "cases": 168, "valid_observations": 672,
                         "probe_harness_oracles_cases_and_caps_preserved": True},
        "intermediate_failures": {
            "v333": "Four new tests passed. A new test incorrectly required coarse sample zero for exact value one; corrected to the one-unit contract.",
            "v339": "946 of 947 debug tests passed. The older v330 extreme-scale test had the same coarse-zero assumption after cosine became exactly one; its identical inputs now allow either valid sample for that independently known identity.",
            "v334": "All five new tests passed after correcting the new coarse assertion.",
            "v340": "All 37 trig-filtered tests passed after correcting the old coarse assertion.",
        },
        "new_unit_coverage": {
            "test_functions": 5, "base_symbolic_inputs": 21, "signed_inputs": 42,
            "trig_families": 4, "views": ["shared clone", "shared original", "public constructor", "CBOR reconstruction"],
            "precision_schedule": [1, 0, -8, -128, -8, 2147483647],
            "lattice_numeric_enclosure_checks": 3024, "lattice_typed_pole_checks": 1008,
            "retained_sign_fact_checks": 504, "independent_integer_modular_oracle_checks": 12675,
            "carry_cancellation_queries": 28, "invalid_descendant_checks": 24,
            "bounded_unknown_controls": 3, "observed_stop_sweep_fixtures": 16,
            "nearby_finite_directed_mpfr_enclosure_checks": 192,
        },
        "changes": changes, "limits": limits,
        "workload_preservation": "All 1537 previous replay cases, the complete 168-case new baseline, signed 96-addition graphs, 740-term domain fixtures, 192000-bit inverse series, and all earlier cache/publication and private-kernel fixtures remain.",
    }
    metadata = {
        "previous_checkpoint": "typed-api-prototype-v330-state.json", "implementation_commits": [sys.argv[1]],
        "implementation_records": ["trig-lattice-implementation-v345.json"],
        "status": "isolated compressed trig lattice repair; no runtime admission",
        "contract_decision": "p4-contract-decision-v173.json", "contract_commit": "8bf62fa3f",
        "changes": changes, "retained_regression_record": "trig-lattice-regression-coverage-v365.json",
        "new_matrix_coverage": coverage["new_unit_coverage"],
        "formal_validation": {key: proof[key] for key in [
            "fresh_verification_phase", "fresh_acceptance_phase", "verified_units", "errors",
            "listed_production_contracts", "listed_model_theorems", "open_obligation_groups",
            "reused_rejection_phase", "reused_rejected_mutants", "reused_passing_guard_baselines",
            "new_rejection_invocations", "fresh_acceptance_tests", "limits"]},
        "formal_reuse_record": "trig-lattice-proof-reuse-v365.json", "test_summaries": summaries,
        "capped_unit_regressions": caps, "final_source_replays": replays,
        "included_rust_fragments_checked": rust_count,
        "inherited_warnings": {"identical_cargo_filename_warning_lines": inherited, "strict_clippy_warning_lines": 0},
        "coverage_limits": limits, "remaining_source_review": "trig-lattice-resource-followup-v365.json",
        "remaining_requirements": limits + [
            "All 39 properties, 14 targets, crate future work and pre-push verification remain in scope. Main runtime and all 49 original worktree entries are preserved. No push is requested or claimed."],
    }
    source_record = "typed-api-prototype-v342-trig-lattice-verus/source-hashes.json.gz"
    sources = json.loads(gzip.decompress((EVIDENCE / source_record).read_bytes()))
    scopes = {
        "trig_lattice.rs": "Complete sparse form, normalized carries, total grammar, local budgets, modular residue and exact trig integer mapping.",
        "approximation_queries.rs": "New exact trig branch after finite-real validation and cache lookup; retained exact sign publication and fallible numeric publication. Surrounding entry/domain/publication slice read in context.",
        "trig_domain.rs": "Complete zero recognizer, with the new half-pi certificate before unchanged bounded rational/pi fallback.",
        "exp_trig.rs": "New lattice branches in fallible sin/cos/tan constructors and neighboring retained reducers; no new whole-file audit claimed.",
        "trig_lattice_tests.rs": "Complete five-test file, including independent integer and directed MPFR oracles, shared/CBOR views, cancellation, invalid descendants and unchanged full baseline inputs.",
        "private_kernel_domain_tests.rs": "Existing extreme-shift recognizer controls: MAX now certifies sine zero, MIN stays Unknown; all original inputs retained.",
        "private_trig_range_tests.rs": "Existing eight extreme-scale fact/coarse controls: exact positive-turn cosine permits valid coarse centers zero or one; all other coarse and undefined-descendant controls retained.",
    }
    reviewed = {"crates/hyperreal/src/computable/node/" + name: {
        "sha256": sources["crates/hyperreal/src/computable/node/" + name], "scope": scope}
        for name, scope in scopes.items()}
    followup = {
        "status": "Scoped source review; executed coverage is in the regression and state records",
        "previous_review": "private-trig-range-resource-followup-v330.json",
        "source_record": source_record, "source_record_sha256": SOURCE_HASH,
        "reviewed_source_slices": reviewed, "resolved_witness": coverage["baseline"],
        "remaining_work": [
            "Extend periodic reduction to finite non-lattice residuals at extreme scales; compare exact independent controls without shrinking the 168 current cases.",
            "Cover wider/deeper equivalent forms, other private variants and nested cancellation. Optional coefficient expansion must not weaken available certificates; keep all existing fallback recognizers.",
            "Connect the implementation's normalization, modular residue, finite-domain and publication arguments to proofs; retain all thirteen whole-crate groups as open until their actual obligations are discharged.",
            "Continue loose-upper-bound planning, shared resource accounting and public concurrent/reconstructed fact schedules; combine the native prototype and retain full-board correctness/timing/allocation/size evidence before runtime admission.",
        ],
        "limits": limits, "unadmitted": True,
        "scope_preserved": "All 39 properties, 14 targets, crate future work and 49 original worktree entries remain. P4 permits stronger retained proofs, never contradictory or weaker available facts.",
    }
    write("trig-lattice-regression-coverage-v365.json", coverage)
    write("trig-lattice-resource-followup-v365.json", followup)
    write("typed-api-prototype-v365-metadata.json", metadata)
    print(json.dumps({"five_crate_passed": 5731, "default_scalar_passed": 952,
                      "debug_scalar_passed": 947, "capped_tests": 35, "replay_cases": 1705}))


if __name__ == "__main__":
    main()
