#!/usr/bin/env python3
"""Record terminal validation of periodic angles with finite opaque operands."""
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

EVIDENCE = Path(__file__).resolve().parent
SOURCE_HASH = "c1ff04a5d1de65bb08126580d94f51656221c4c7be84022695ded1890b09c001"
SOURCE_RECORD = "typed-api-prototype-v409-trig-operand-verus/source-hashes.json.gz"


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
    implementation = read(EVIDENCE / "trig-operand-implementation-v412.json")
    assert implementation["source_record_sha256"] == SOURCE_HASH
    summaries = {}
    for key, version, passed, child, ignored in [
        ("debug_scalar", 412, 959, 0, 0), ("five_crate", 413, 5743, 2, 33),
        ("default_scalar", 414, 964, 2, 0),
    ]:
        path = EVIDENCE / f"typed-api-prototype-v{version}-test-summary.json"
        data = read(path)
        assert data["top_level_passed"] == passed and data["child_process_passed"] == child
        assert data["top_level_failed"] == data["child_process_failed"] == 0
        assert data["top_level_ignored"] == ignored
        summaries[key] = {"record": path.name, "passed": passed, "child_passed": child, "ignored": ignored}
    scalar = read(EVIDENCE / summaries["default_scalar"]["record"])["per_crate"]["hyperreal"]
    assert scalar["unit_and_integration"] == 941 and scalar["doctests"] == 23

    caps = {}
    for version, suffix, count in [(418, "private-kernel", 7), (419, "magnitude", 5),
                                  (420, "publication", 7), (421, "fractional", 5),
                                  (422, "scaled-lower", 1), (423, "trig-range", 5),
                                  (424, "trig-lattice", 5), (425, "trig-residual", 6),
                                  (426, "trig-operand", 6)]:
        phase = f"typed-api-prototype-v{version}-{suffix}-cap"
        path = completed(phase)
        log = (path / "stdout.log").read_text()
        receipt = json.loads(log.splitlines()[-1])
        assert receipt["exit_code"] == 0 and not receipt["wall_timeout"]
        assert re.findall(r"^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;", log, re.M) == [(str(count), "0", "0")]
        assert receipt["address_space_bytes"] == 192 * 1024 * 1024
        assert receipt["cpu_seconds_soft_hard"] == [2, 3] and receipt["wall_seconds_limit"] == 4
        caps[phase] = receipt | {"passed": count, "phase_result": read(path / "result.json")}
    assert sum(row["passed"] for row in caps.values()) == 47

    replays = {}
    for version, suffix, count in [(428, "replay", 415), (429, "precision-replay", 60),
                                  (430, "private-kernel-replay", 78), (431, "private-trig-replay", 600),
                                  (432, "private-trig-wide-replay", 384), (433, "symbolic-trig-scale-replay", 168),
                                  (434, "symbolic-trig-residual-replay", 456), (435, "symbolic-trig-nonrational-replay", 384)]:
        phase = f"typed-api-prototype-v{version}-{suffix}"
        path = EVIDENCE / phase / "summary.json"
        data = read(path)
        assert data["source_hashes_sha256"] == SOURCE_HASH and data["guards_unchanged"]
        if version == 428:
            assert data["ordinary_cases"] == 400 and data["typed_interruption_errors"] == 300
            assert data["invalid_first_contracts"] == data["invalid_retries"] == 0
            assert len(data["extremes"]) == 6 and all(row["valid"] for row in data["extremes"])
            assert len(data["cache"]) == 9 and all(row["valid"] for row in data["cache"])
            assert all(row["exit_code"] == 0 for row in data["commands"])
        else:
            assert data["cases"] == count
            if version == 429:
                assert data["valid_cases"] == 60 and not data["nonzero_exit_cases"] and not data["invalid_numeric_cases"]
            elif version == 430:
                assert data["incomplete_cases"] == data["unexpected_result_shapes"] == 0
            elif version == 433:
                assert data["incomplete_cases"] == data["invalid_samples"] == 0 and data["valid_samples"] == 672
            else:
                assert data["incomplete_cases"] == data["invalid_numeric_samples"] == data["typed_errors"] == 0
                assert data["numeric_samples"] == count * 4
        replays[phase] = {"summary_sha256": digest(path), "cases": count}
    assert sum(row["cases"] for row in replays.values()) == 2545

    baseline = EVIDENCE / "typed-api-prototype-v404-symbolic-trig-nonrational-baseline"
    final = EVIDENCE / "typed-api-prototype-v435-symbolic-trig-nonrational-replay"
    old, new = read(baseline / "receipts.json"), read(final / "receipts.json")
    assert len(old) == len(new) == 384
    assert sum(not row["complete"] for row in old) == 36
    assert all(row["command"]["exit_code"] == -24 for row in old if not row["complete"])
    errors = Counter(sample["error"] for row in old for sample in row["samples"] if sample["error"])
    assert errors == {"Exhausted": 384, "OutOfRange": 672}
    assert sum(sample["numeric"] for row in old for sample in row["samples"]) == 374
    assert not any(row["invalid_numeric_samples"] for row in old)
    for name in ["probe.rs", "reproduce.py", "oracles.json"]:
        assert digest(baseline / name) == digest(final / name), name
    for a, b in zip(old, new):
        assert all(a[key] == b[key] for key in ["shape", "shift", "pi_coefficient", "residual", "family", "mode", "oracle_key"])
        assert a["command"]["command"][1:] == b["command"]["command"][1:]
        assert all(a["command"][key] == b["command"][key] for key in ["address_space_cap_mib", "cpu_seconds", "timeout_seconds"])
        assert b["complete"] and len(b["samples"]) == 4 and not b["typed_errors"] and not b["invalid_numeric_samples"]

    previous = read(EVIDENCE / "typed-api-prototype-v403-state.json")
    sources = json.loads(gzip.decompress((EVIDENCE / SOURCE_RECORD).read_bytes()))
    before = json.loads(gzip.decompress((EVIDENCE / previous["source_record"]).read_bytes()))
    changed = {name: {"before": before.get(name), "after": value} for name, value in sources.items() if before.get(name) != value}
    assert len(changed) == 9 and changed == implementation["changed_sources"]
    assert all(before[name] == sources[name] for name in before if "test" in name)
    for old_version, new_version, suffix in [(396, 428, "replay"), (398, 430, "private-kernel-replay"),
                                           (399, 431, "private-trig-replay"), (400, 432, "private-trig-wide-replay"),
                                           (401, 433, "symbolic-trig-scale-replay"), (402, 434, "symbolic-trig-residual-replay")]:
        a = EVIDENCE / f"typed-api-prototype-v{old_version}-{suffix}"
        b = EVIDENCE / f"typed-api-prototype-v{new_version}-{suffix}"
        for name in ["probe.rs", "reproduce.py", "oracles.json"]:
            if (a / name).is_file():
                assert digest(a / name) == digest(b / name), (suffix, name)

    proof = read(EVIDENCE / "trig-operand-proof-reuse-v436.json")
    assert proof["source_record_sha256"] == SOURCE_HASH
    assert proof["verified_units"] == 638 and proof["errors"] == 0
    assert proof["reused_rejected_mutants"] == 213 and proof["new_rejection_invocations"] == 0
    warnings = lambda p: [line for line in p.read_text().splitlines() if line.startswith("warning:")]
    inherited = warnings(completed("typed-api-prototype-v413-stack-tests") / "stderr.log")
    assert inherited == warnings(EVIDENCE / "typed-api-prototype-v382-stack-tests/stderr.log") and len(inherited) == 14
    for phase in ["typed-api-prototype-v411-stack-clippy", "typed-api-prototype-v415-minimal-clippy"]:
        assert not warnings(completed(phase) / "stderr.log")
    fmt = read(completed("typed-api-prototype-v417-included-fmt") / "command.json")["command"]
    rust_count = len(fmt[fmt.index("--check") + 1:])
    assert rust_count == 88

    changes = [
        "Extend periodic decomposition with retained opaque operands and existing bounded rational certificates. Multiply only by recognized scalars; keep nonlinear products intact. Combine equal atoms conditionally without erasing their original domain obligations.",
        "Preserve existing exact algebraic constructor branches and total pi/rational reduction. New public fallback nodes retain the original graph; private kernels validate its finite domain before using a reduced remainder.",
        "Approximate the finite remainder divided by pi/2 to choose a nearby integer. Subtract its independently enclosed pi multiple at guarded precision and evaluate the bounded sine/cosine series directly; do not reconstruct an equivalent trig node that could undo reduction.",
        "Recognize exact conditional lattice cancellation only after original-domain validation, including tangent/cotangent poles. Existing finite facts, observed-stop propagation and numeric publication remain authoritative.",
    ]
    limits = implementation["limits"]
    coverage = {
        "status": "Reproduced finite-operand periodic failures repaired in the isolated candidate",
        "contract_decision": "p4-contract-decision-v173.json", "source_record_sha256": SOURCE_HASH,
        "baseline": implementation["baseline"] | {"commit": "fd601ecd7", "unexpected_typed_samples": dict(errors),
            "invalid_returned_numeric_samples": 0, "interpretation": "Completion/resource failures; no wrong returned numeric sample observed."},
        "final_replay": {"phase": final.name, "cases": 384, "valid_numeric_observations": 1536,
                         "probe_harness_oracles_parameters_order_and_caps_preserved": True},
        "intermediate_failures": implementation["intermediate_failures"],
        "new_unit_coverage": implementation["coverage"] | {"test_functions": 6,
            "views": ["shared clone", "shared original", "public constructor", "CBOR reconstruction"],
            "precision_schedule": [0, -8, -256, -8],
            "retained_sign_checks": "Every numerical check preserves a previous definite sign and verifies it against an independent enclosure; Unknown may strengthen."},
        "changes": changes, "limits": limits,
        "workload_preservation": "All 2161 prior replay cases and test sources, the 384-case baseline, signed 96-addition graphs, 740-term domain fixtures, 192000-bit inverse series and cache/publication regressions remain.",
    }
    scopes = {
        "node/trig_operands.rs": "Complete conditional extraction, coefficient normalization, nonlinear atoms, cancellation, residue reconstruction and domain-qualified lattice query.",
        "approximation/trig_operands.rs": "Complete direct quotient, guarded subtraction, bounded-series call, quadrant mapping and original-domain validation.",
        "node/exp_trig.rs": "New public fallback placement after existing algebraic/rational certificates; raw original graph retained. Neighboring reducer paths read; no whole-file audit claimed.",
        "approximation/trig.rs": "New sine/cosine kernel entry branches, shared numerical series, and neighboring tangent/cotangent quotient paths read; no whole-file audit claimed.",
        "node/trig_lattice.rs": "Existing exact integer evaluation hook and new conditional lattice recognition after original-domain validation.",
        "node/trig_domain.rs": "Complete domain-qualified zero recognizer; conditional cancellation added to sine/cosine lattice queries.",
        "node/trig_operands_tests.rs": "Complete six-test file, directed independent enclosures, monotone signs, reconstruction, nonlinear products, wrappers, invalid descendants, poles, algebraic certificates and stops.",
        "node/domain_validation.rs": "Existing child-first finite-domain traversal and trig denominator checks; unchanged source.",
        "node/primitive_constructors.rs": "Existing structural comparison walks immutable DAGs beyond a 64-step fast path; atom comparison is not covered by the form's local visit budget. Unchanged source.",
        "node/structural_analysis.rs": "Existing bounded rational certificate uses 4096-bit values, depth 32 and 64 visits; reads immutable structure. Unchanged source.",
    }
    followup = {
        "status": "Scoped source review; executed coverage is in the regression and state records",
        "previous_review": "trig-residual-resource-followup-v403.json", "source_record": SOURCE_RECORD,
        "source_record_sha256": SOURCE_HASH,
        "reviewed_source_slices": {"crates/hyperreal/src/computable/" + name:
            {"sha256": sources["crates/hyperreal/src/computable/" + name], "scope": scope} for name, scope in scopes.items()},
        "resolved_witness": coverage["baseline"],
        "numerical_argument": "Let q=x/(pi/2), a=approx(q,-2), n=round(a/4). The one-unit approximation and rounding contracts give |q-n|<=3/4, hence |x-n*pi/2|<=3*pi/8<1.5. At work=p-7 and b=bits(n), the pi request work-b-4 contributes <1/32 work unit after multiplication by n/2. Operand error, inner half-unit rounding and final scaling by 1/8 keep the residual within one unit at p-4. The existing bounded-series kernel receives that sample. This is a source-level argument, not an implementation-connected formal proof.",
        "remaining_work": [
            "Prioritize shared resource accounting for structural atom comparison, original domain traversal and nested arithmetic. Local extraction limits alone do not bound these costs; budget exhaustion must retain available facts and return typed uncertainty without partial publication.",
            "Use nested periodic operands, wider/deeper equivalent forms, extreme negative scales, other private variants and concurrent/reconstructed cache schedules as adversarial coverage for that shared contract, while retaining existing available facts.",
            "Connect conditional extraction, exact modular residues, domain-preserving cancellation, numerical quotient/subtraction, series error and termination to implementation proofs; all thirteen whole-crate groups remain open.",
            "Continue composed ln/exp/inverse-hyperbolic planning under loose magnitude bounds. Establish matched cost evidence, combine the native prototype and replay unchanged full-board reports before admission; retain all properties, targets and crate future work.",
        ], "limits": limits, "unadmitted": True,
        "scope_preserved": "All 39 properties, 14 targets, crate future work and 49 original worktree entries remain. P4 permits stronger retained proofs, never contradictory or weaker available facts.",
    }
    metadata = {
        "previous_checkpoint": "typed-api-prototype-v403-state.json", "implementation_commits": [sys.argv[1]],
        "baseline_commit": "fd601ecd7", "implementation_records": ["trig-operand-implementation-v412.json"],
        "status": "isolated periodic-angle finite-operand repair; no runtime admission",
        "contract_decision": "p4-contract-decision-v173.json", "contract_commit": "8bf62fa3f",
        "changes": changes, "retained_regression_record": "trig-operand-regression-coverage-v436.json",
        "new_matrix_coverage": coverage["new_unit_coverage"],
        "formal_validation": {key: proof[key] for key in ["fresh_verification_phase", "fresh_acceptance_phase", "verified_units", "errors", "listed_production_contracts", "listed_model_theorems", "open_obligation_groups", "reused_rejection_phase", "reused_rejected_mutants", "reused_passing_guard_baselines", "new_rejection_invocations", "fresh_acceptance_tests", "limits"]},
        "formal_reuse_record": "trig-operand-proof-reuse-v436.json", "test_summaries": summaries,
        "capped_unit_regressions": caps, "final_source_replays": replays,
        "included_rust_fragments_checked": rust_count,
        "inherited_warnings": {"identical_cargo_filename_warning_lines": inherited, "strict_clippy_warning_lines": 0},
        "coverage_limits": limits, "remaining_source_review": "trig-operand-resource-followup-v436.json",
        "remaining_requirements": limits + ["All 39 properties, 14 targets, crate future work and pre-push verification remain required. Main runtime and all 49 original worktree entries are preserved. No push is requested or claimed."],
    }
    write("trig-operand-regression-coverage-v436.json", coverage)
    write("trig-operand-resource-followup-v436.json", followup)
    write("typed-api-prototype-v436-metadata.json", metadata)
    print(json.dumps({"five_crate_passed": 5743, "default_scalar_passed": 964,
                      "debug_scalar_passed": 959, "capped_tests": 47, "replay_cases": 2545}))


if __name__ == "__main__":
    main()
