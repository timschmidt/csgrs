#!/usr/bin/env python3
"""Record terminal validation of caller-owned cooperative evaluation budgets."""
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import re
import sys

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]
SOURCE_HASH = "c436bdefd3bfa8f3ab8c928f6edfa4288dd83e3fad2782d02568bc33224ee11e"
SOURCE_RECORD = "typed-api-prototype-v475-budget-verus/source-hashes.json.gz"


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
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"], phase
    assert digest(path / "source-hashes.json.gz") == SOURCE_HASH, phase
    return path


def case_commands(path):
    if (path / "commands.json").is_file():
        rows = read(path / "commands.json")
        return [row for row in rows if row["name"].startswith(("case-", "extreme-", "cache-"))]
    return read(path / "receipts.json")


def command_contract(row):
    # Replace only the isolated executable directory; preserve every argument,
    # case order and resource cap, including timeout's wrapper arguments.
    command = [re.sub(r"/target/typed-api-prototype-v\d+-[^/]+/", "/target/PHASE/", part)
               for part in row["command"]]
    return {"name": row["name"], "command": command,
            **{key: row.get(key) for key in ["timeout_seconds", "address_space_cap_mib", "cpu_seconds"]}}


def main():
    assert len(sys.argv) == 3, "implementation and classification-correction commits required"
    implementation = read(EVIDENCE / "evaluation-budget-implementation-v446.json")
    correction = read(EVIDENCE / "evaluation-budget-api-classification-v474.json")
    assert correction["source_record_sha256"] == SOURCE_HASH
    assert correction["runtime_source_record_sha256"] == implementation["source_record_sha256"]
    completed(correction["focused_validation"])
    orchestration = read(EVIDENCE / "evaluation-budget-orchestration-v504.json")
    assert orchestration["exit_code"] == 0 and orchestration["source_record_sha256"] == SOURCE_HASH
    assert orchestration["helper_sha256"] == digest(EVIDENCE / "validate_evaluation_budget_checkpoint_v3.py")
    assert orchestration["log_sha256"] == digest(EVIDENCE / "evaluation-budget-orchestration-v504-resumed.log")
    bookkeeping = read(EVIDENCE / orchestration["resumed_from"])
    assert bookkeeping["exit_code"] == 1 and bookkeeping["source_record_sha256"] == SOURCE_HASH
    assert bookkeeping["helper_sha256"] == digest(EVIDENCE / "validate_evaluation_budget_checkpoint_v2.py")
    assert bookkeeping["log_sha256"] == digest(EVIDENCE / "evaluation-budget-orchestration-v504.log")
    old_helper = (EVIDENCE / "validate_evaluation_budget_checkpoint_v2.py").read_text()
    correction_from = 'assert data["cases"] == 488 and data["numeric_samples"] == data["observations"] == 1824'
    correction_to = 'assert data["cases"] == 456 and data["numeric_samples"] == data["observations"] == 1824'
    assert old_helper.count(correction_from) == 1
    assert old_helper.replace(correction_from, correction_to) == (EVIDENCE / "validate_evaluation_budget_checkpoint_v3.py").read_text()
    summaries = {}
    for key, version, passed, child, ignored in [
        ("debug_scalar", 478, 966, 0, 0), ("five_crate", 479, 5750, 2, 33),
        ("default_scalar", 480, 971, 2, 0),
    ]:
        path = EVIDENCE / f"typed-api-prototype-v{version}-test-summary.json"
        data = read(path)
        assert data["top_level_passed"] == passed and data["child_process_passed"] == child
        assert data["top_level_failed"] == data["child_process_failed"] == 0
        assert data["top_level_ignored"] == ignored
        summaries[key] = {"record": path.name, "passed": passed, "child_passed": child, "ignored": ignored}
    scalar = read(EVIDENCE / summaries["default_scalar"]["record"])["per_crate"]["hyperreal"]
    assert scalar["unit_and_integration"] == 948 and scalar["doctests"] == 23

    caps = {}
    for version, suffix, count in [(484, "private-kernel", 7), (485, "magnitude", 5),
                                  (486, "publication", 7), (487, "fractional", 5),
                                  (488, "scaled-lower", 1), (489, "trig-range", 5),
                                  (490, "trig-lattice", 5), (491, "trig-residual", 6),
                                  (492, "trig-operand", 6), (493, "evaluation-budget", 7)]:
        phase = f"typed-api-prototype-v{version}-{suffix}-cap"
        path = completed(phase)
        log = (path / "stdout.log").read_text()
        receipt = json.loads(log.splitlines()[-1])
        assert receipt["exit_code"] == 0 and not receipt["wall_timeout"]
        assert re.findall(r"^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;", log, re.M) == [(str(count), "0", "0")]
        assert receipt["address_space_bytes"] == 192 * 1024 * 1024
        assert receipt["cpu_seconds_soft_hard"] == [2, 3] and receipt["wall_seconds_limit"] == 4
        caps[phase] = receipt | {"passed": count, "phase_result": read(path / "result.json")}
    assert sum(row["passed"] for row in caps.values()) == 54

    replays = {}
    for version, suffix, count in [(495, "replay", 415), (496, "precision-replay", 60),
                                  (497, "private-kernel-replay", 78), (498, "private-trig-replay", 600),
                                  (499, "private-trig-wide-replay", 384), (500, "symbolic-trig-scale-replay", 168),
                                  (501, "symbolic-trig-residual-replay", 456), (502, "symbolic-trig-nonrational-replay", 384),
                                  (503, "evaluation-budget-replay", 384)]:
        phase = f"typed-api-prototype-v{version}-{suffix}"
        path = EVIDENCE / phase / "summary.json"
        data = read(path)
        assert data["source_hashes_sha256"] == SOURCE_HASH and data["guards_unchanged"]
        if version == 495:
            assert data["ordinary_cases"] == 400 and data["typed_interruption_errors"] == 300
            assert data["invalid_first_contracts"] == data["invalid_retries"] == 0
            assert len(data["extremes"]) == 6 and all(row["valid"] for row in data["extremes"])
            assert len(data["cache"]) == 9 and all(row["valid"] for row in data["cache"])
            assert all(row["exit_code"] == 0 for row in data["commands"])
        else:
            assert data["cases"] == count
            if version == 496:
                assert data["valid_cases"] == 60 and not data["nonzero_exit_cases"] and not data["invalid_numeric_cases"]
            elif version == 497:
                assert data["incomplete_cases"] == data["unexpected_result_shapes"] == 0
            elif version == 500:
                assert data["incomplete_cases"] == data["invalid_samples"] == 0 and data["valid_samples"] == 672
            else:
                assert data["incomplete_cases"] == data["invalid_numeric_samples"] == data["typed_errors"] == 0
                assert data["numeric_samples"] == count * 4
        replays[phase] = {"summary_sha256": digest(path), "cases": count}
        if version < 503:
            old = EVIDENCE / f"typed-api-prototype-v{version - 67}-{suffix}"
            new = path.parent
            for name in ["probe.rs", "candidate_probe.rs", "cache_candidate_probe.rs", "reproduce.py", "oracles.json"]:
                if (old / name).is_file():
                    assert digest(old / name) == digest(new / name), (suffix, name)
            assert [command_contract(row) for row in case_commands(old)] == [
                command_contract(row) for row in case_commands(new)], suffix
    assert sum(row["cases"] for row in replays.values()) == 2929

    unbudgeted = EVIDENCE / "typed-api-prototype-v502-symbolic-trig-nonrational-replay"
    budgeted = EVIDENCE / "typed-api-prototype-v503-evaluation-budget-replay"
    assert digest(unbudgeted / "oracles.json") == digest(budgeted / "oracles.json")
    old, new = read(unbudgeted / "receipts.json"), read(budgeted / "receipts.json")
    assert len(old) == len(new) == 384
    assert [command_contract(row) for row in case_commands(unbudgeted)] == [
        command_contract(row) for row in case_commands(budgeted)]
    cold = Counter()
    work_units = []
    for a, b in zip(old, new):
        assert all(a[key] == b[key] for key in ["shape", "shift", "pi_coefficient", "residual", "family", "mode", "oracle_key"])
        assert b["complete"] and len(b["samples"]) == 4 and not b["typed_errors"] and not b["invalid_numeric_samples"]
        assert [sample["precision"] for sample in a["samples"]] == [sample["precision"] for sample in b["samples"]]
        empty, warm = b["empty_budget"], b["warm_budget"]
        assert empty["consumed"] == warm["empty_consumed"] == 0
        assert empty["error"] in [None, "Exhausted"]
        assert empty["numeric"] == (empty["error"] is None)
        assert warm["consumed"] + warm["remaining"] == 1_000_000 and warm["enclosed"]
        if warm["before_sign"] is not None:
            assert warm["after_sign"] == warm["before_sign"]
        cold[empty["error"] or "Numeric"] += 1
        work_units.append(warm["consumed"])
    final_budget = read(budgeted / "summary.json")
    assert final_budget["warm_zero_budget_checks"] == 384
    assert final_budget["cold_zero_budget_exhaustions"] == cold["Exhausted"]

    previous = read(EVIDENCE / "typed-api-prototype-v436-state.json")
    sources = json.loads(gzip.decompress((EVIDENCE / SOURCE_RECORD).read_bytes()))
    before = json.loads(gzip.decompress((EVIDENCE / previous["source_record"]).read_bytes()))
    assert set(before) <= set(sources)
    changed = {name: {"before": before.get(name), "after": value} for name, value in sources.items() if before.get(name) != value}
    inventory = "crates/hyperreal/tests/gmp_api_coverage.rs"
    assert len(changed) == 20 and changed == implementation["changed_sources"] | correction["changed_sources"]
    assert all(before[name] == sources[name] for name in before if "test" in name and name != inventory)
    runtime_snapshot = json.loads(gzip.decompress((EVIDENCE / implementation["source_record"]).read_bytes()))
    assert sources.keys() == runtime_snapshot.keys()
    runtime_delta = {name: {"before": runtime_snapshot[name], "after": value}
                     for name, value in sources.items() if runtime_snapshot[name] != value}
    assert runtime_delta == correction["changed_sources"] and list(runtime_delta) == [inventory]
    new_inventory = (EVIDENCE / SOURCE_RECORD).parent / "inputs" / inventory
    added_classifications = (
        "    // Caller-owned cooperative work limits and their accounting have no\n"
        "    // matching GMP/MPFR operation. Budgeted approximation/sign calls retain\n"
        "    // the existing numerical contract; exhaustion is typed uncertainty.\n"
        "    \"approx_with_budget\",\n    \"sign_until_with_budget\",\n"
        "    \"consumed\",\n    \"remaining\",\n"
    )
    assert new_inventory.read_text().count(added_classifications) == 1
    # The preparation inherited this file from the separate native prototype,
    # so main's user-owned version is not its baseline. Removing exactly the
    # four new classifications must recover the retained full-file hash.
    prior_inventory = new_inventory.read_text().replace(added_classifications, "").encode()
    assert hashlib.sha256(prior_inventory).hexdigest() == before[inventory]
    for name in ["quadratic_tower.rs", "radical_tower.rs"]:
        relative = "crates/hyperreal/src/computable/node/" + name
        old_path = (EVIDENCE / previous["source_record"]).parent / "inputs" / relative
        if not old_path.is_file():
            old_path = ROOT / relative
        assert digest(old_path) == before[relative]
        new_path = (EVIDENCE / SOURCE_RECORD).parent / "inputs" / relative
        marker = "#[cfg(test)]"
        expected_sections = int(name == "quadratic_tower.rs")
        assert old_path.read_text().count(marker) == new_path.read_text().count(marker) == expected_sections
        assert old_path.read_text().partition(marker)[2] == new_path.read_text().partition(marker)[2]

    proof = read(EVIDENCE / "evaluation-budget-proof-reuse-v504.json")
    assert proof["source_record_sha256"] == SOURCE_HASH
    assert proof["verified_units"] == 638 and proof["errors"] == 0
    assert proof["reused_rejected_mutants"] == 213 and proof["new_rejection_invocations"] == 0
    assert len(proof["proof_and_guard_inputs_sha256"]) == 44
    warnings = lambda p: [line for line in p.read_text().splitlines() if line.startswith("warning:")]
    inherited = warnings(completed("typed-api-prototype-v479-stack-tests") / "stderr.log")
    assert inherited == warnings(EVIDENCE / "typed-api-prototype-v413-stack-tests/stderr.log") and len(inherited) == 14
    for phase in ["typed-api-prototype-v477-stack-clippy", "typed-api-prototype-v481-minimal-clippy"]:
        assert not warnings(completed(phase) / "stderr.log")
    fmt = read(completed("typed-api-prototype-v483-included-fmt") / "command.json")["command"]
    formatted = fmt[fmt.index("--check") + 1:]
    original = json.loads(gzip.decompress((EVIDENCE / "candidate-preparation/source-hashes.json.gz").read_bytes()))
    assert formatted == sorted(name for name, value in sources.items() if name.endswith(".rs") and original.get(name) != value)
    cost = read(EVIDENCE / implementation["cost_record"])
    assert cost["source_record_sha256"] == implementation["source_record_sha256"]
    assert cost["probe_and_harness_identical"] and cost["checksums_and_iterations_preserved"]
    assert len(cost["cases"]) == 6
    cost_before = EVIDENCE / cost["baseline_phase"]
    cost_after = EVIDENCE / cost["candidate_phase"]
    assert digest(cost_before / "summary.json") == cost["baseline_summary_sha256"]
    assert digest(cost_after / "summary.json") == cost["candidate_summary_sha256"]
    before_cost, after_cost = read(cost_before / "summary.json"), read(cost_after / "summary.json")
    assert before_cost["processes"] == after_cost["processes"] == 54
    assert before_cost["guards_unchanged"] and after_cost["guards_unchanged"]
    for name in ["probe.rs", "reproduce.py"]:
        assert digest(cost_before / name) == digest(cost_after / name)
    for name, case in cost["cases"].items():
        assert case["before"] == before_cost["cases"][name]
        assert case["after"] == after_cost["cases"][name]
        assert len(case["before"]["elapsed_ns"]) == len(case["after"]["elapsed_ns"]) == 9
        assert case["before"]["iterations"] == case["after"]["iterations"]
        assert case["before"]["checksums"] == case["after"]["checksums"]
        assert case["median_ratio"] == case["after"]["median_ns"] / case["before"]["median_ns"]

    limits = implementation["limits"] + [
        "The per-allowance sweep is scoped to sqrt(2)+ln(7/5); global constant caches may already be warm. It is not a kernel-by-kernel exhaustive interruption matrix.",
        "External budget replay construction and structural_facts calls occur outside the explicit evaluation allowance. The 384 zero-budget queries check retained root approximations and signs; they do not meter the entire program.",
        "The 78 private-kernel replay cases check expected result shapes, not full numerical accuracy. Cost probes are six local micro-workloads, not allocation, binary-size or full-board admission evidence.",
    ]
    coverage = {
        "status": "Shared cooperative evaluation budget validated in the isolated candidate; no runtime admission",
        "contract_decision": "p4-contract-decision-v173.json", "source_record_sha256": SOURCE_HASH,
        "changes": implementation["changes"] + [correction["correction"]], "public_api": implementation["public_api"],
        "retained_intermediate_failure": {"phase": correction["failure_phase"], "exit_code": 101,
            "correction_record": "evaluation-budget-api-classification-v474.json", "guard_assertions_unchanged": True},
        "new_unit_coverage": implementation["unit_coverage"],
        "external_budget_replay": {"phase": budgeted.name, "cases": 384,
            "valid_generous_budget_numeric_observations": 1536, "warm_zero_budget_checks": 384,
            "cold_zero_budget_outcomes": dict(cold), "shared_allowance_per_case": 1_000_000,
            "observed_consumed_work_unit_range": [min(work_units), max(work_units)],
            "precision_schedule": [0, -8, -256, -8],
            "prior_corpus_fixtures_oracles_arguments_order_and_caps_preserved": True,
            "views": ["public construction", "serialized reconstruction"]},
        "cost_record": implementation["cost_record"], "cost_scope": cost["scope"],
        "workload_preservation": "All 2545 prior replay cases and numerical regression tests remain unchanged, including the 192000-bit series, signed 96-addition graphs, 740-term domain fixtures and prior cache/publication regressions. The API inventory adds four policy/accounting classifications; its assertions are unchanged. Existing quadratic-tower inline tests are byte-identical; radical_tower.rs has no inline test section.",
        "limits": limits,
    }
    scopes = {
        "budget.rs": "Complete public allowance, atomic subtraction, nested TLS request scope, duplicate-ancestor handling, sticky failure and unwind cleanup.",
        "node/evaluation_budget.rs": "Complete budgeted approximation/sign entry points and propagation helpers; prior immediate root signs are checked before starting work.",
        "node/evaluation_budget_tests.rs": "Complete seven-test file: allowance sweep, successive requests, retained high-precision sign, structural exhaustion, negative-cache retry, atomic sharing and nested/unwinding scopes.",
        "node/primitive_constructors.rs": "Changed structural-comparison traversal: fallible work callback, preserved exact comparator, bounded fast path and iterative fallback. Other constructor paths were not audited wholesale.",
        "node/approximation_queries.rs": "Changed root cache/int entry, work-stack and sign/refinement checkpoints; cache hits remain available without a fresh debit. No whole-file audit claimed.",
        "node/structural_analysis.rs": "Changed budgeted bounds/sign/filter visits and publication guards; interrupted optional searches do not publish Unknown. No whole-file audit claimed.",
        "node/quadratic_tower.rs": "Changed graph-visit debit and reuse of cached bases before charging; also read Quad operations and square-root/shift coefficient helpers. Algebraic coefficient operations remain unmetered; no whole-file audit claimed.",
        "node/radical_tower.rs": "Changed graph-visit debit and suppression of permanent decline after budget exhaustion; also read RadicalElement and RadicalTower algebraic operations through rational_square_root. Individual algebraic operations remain unmetered; test bodies were compared for preservation, not re-audited.",
        "approximation.rs": "Changed numerical should_stop import aliases the shared debit while retaining existing cancellation polling; individual kernel bodies were not re-audited wholesale.",
        "node/domain_certificates.rs": "Changed recursive certificate-visit debit after retained finite-domain lookup.",
        "node/domain_validation.rs": "Changed request-entry and traversal/domain-check debits; retained domain evidence survives interrupted work.",
        "node/trig_operands.rs": "Changed extraction visits and fallible atom equality; exhausted comparison declines extraction rather than declaring inequality.",
        "node/trig_lattice.rs": "Changed bounded extraction visit debit; modular arithmetic still has separate local size limits.",
        "node/trig_periodic.rs": "Changed shared work/cancellation checkpoints before and after reduction.",
    }
    followup = {
        "status": "Scoped source review; executed coverage is in the regression and state records",
        "previous_review": "trig-operand-resource-followup-v436.json", "source_record": SOURCE_RECORD,
        "source_record_sha256": SOURCE_HASH,
        "reviewed_source_slices": {"crates/hyperreal/src/computable/" + name:
            {"sha256": sources["crates/hyperreal/src/computable/" + name], "scope": scope} for name, scope in scopes.items()},
        "source_level_argument": "Atomic checked subtraction prevents an individual allowance from wrapping or overspending. Each distinct active allowance is charged, and any observed failure marks the request chain. Duplicate reentry preserves intervening limits. RAII restores the prior thread-local frame. Failure latches are request-local, so later calls can reuse established facts and numeric caches. Partial debits before another ancestor declines are conservative and are not rolled back. These are source-level arguments and tested examples, not implementation-connected formal proofs.",
        "resource_gap_candidates_from_read_slices": [
            {"path": "crates/hyperreal/src/computable/node/radical_tower.rs",
             "operations": ["RadicalElement::scale", "RadicalElement::add", "RadicalElement::neg", "RadicalTower::mul", "RadicalTower::sign", "RadicalTower::inverse"],
             "finding": "Recursive element arithmetic and clones can perform many coefficient operations inside one graph visit. mul checks the resulting term/coefficient limit after computing the product. Metering must decline conservatively before expensive work while preserving successful certificates."},
            {"path": "crates/hyperreal/src/computable/node/quadratic_tower.rs",
             "operations": ["Quad::mul", "Quad::inverse", "Quad::sign", "square_parts", "sqrt_quad", "shifted_coefficient"],
             "finding": "Product sums, exact ordering, GCD/square extraction and coefficient cloning remain outside the cooperative debit. shifted_coefficient bounds only new shift expansion, intentionally preserving already represented wide coefficients and cancellation."},
            {"path": "crates/hyperreal/src/computable/node/primitive_constructors.rs",
             "operations": ["Computable::internal_structural_eq", "Computable::budgeted_structural_eq"],
             "finding": "Only periodic-atom equality currently uses the fallible budgeted comparator. The legacy exact API must retain exact equality semantics; further callers need an explicit Unknown path before they can adopt bounded comparison."},
        ],
        "remaining_work": [
            "Complete resource accounting: map unmetered algebraic helpers, legacy exact structural comparisons, arbitrary-precision coefficient operations, allocation and time. The new checkpoint counter is a shared cooperative allowance, not a bound on these remaining costs.",
            "Exercise kernel-specific budget boundaries and public concurrent/reconstructed query schedules with independent numerical/domain certificates. Keep all previous adversarial fixtures and facts; Unknown may strengthen, but no available proof may weaken.",
            "Connect atomic allowance accounting, nested scope restoration, sticky failure and non-poisoning publication to implementation proofs, alongside extraction/modular/domain/series obligations. All thirteen whole-crate groups remain open.",
            "Continue composed ln/exp/inverse-hyperbolic planning under loose magnitude bounds. Retain scoped timing overhead evidence, then establish allocation/size/whole-application costs, combine the native prototype and replay unchanged full-board reports before admission.",
        ], "limits": limits, "unadmitted": True,
        "scope_preserved": "All 39 properties, 14 targets, crate future work and 49 original worktree entries remain. P4 permits stronger retained proofs, never contradictory or weaker available facts.",
    }
    metadata = {
        "previous_checkpoint": "typed-api-prototype-v436-state.json", "implementation_commits": sys.argv[1:],
        "cost_baseline_commit": "71f39c6df", "implementation_records": ["evaluation-budget-implementation-v446.json", "evaluation-budget-api-classification-v474.json"],
        "status": "isolated shared cooperative evaluation budgets; no runtime admission",
        "contract_decision": "p4-contract-decision-v173.json", "contract_commit": "8bf62fa3f",
        "changes": coverage["changes"], "retained_regression_record": "evaluation-budget-regression-coverage-v504.json",
        "retained_intermediate_failure": coverage["retained_intermediate_failure"],
        "orchestration_count_correction": {"failed_record": orchestration["resumed_from"],
            "passing_record": "evaluation-budget-orchestration-v504.json",
            "reason": "Phase renumbering accidentally changed the residual corpus count from 456 to 488. The continuation restores 456, keeps the same 1824 observations, reuses terminal passing validations and preserves all fixtures, runtime source and failed-helper evidence."},
        "new_matrix_coverage": coverage["new_unit_coverage"], "external_budget_replay": coverage["external_budget_replay"],
        "formal_validation": {key: proof[key] for key in ["fresh_verification_phase", "fresh_acceptance_phase", "verified_units", "errors", "listed_production_contracts", "listed_model_theorems", "open_obligation_groups", "reused_rejection_phase", "reused_rejected_mutants", "reused_passing_guard_baselines", "new_rejection_invocations", "fresh_acceptance_tests", "limits"]},
        "formal_reuse_record": "evaluation-budget-proof-reuse-v504.json", "test_summaries": summaries,
        "capped_unit_regressions": caps, "final_source_replays": replays,
        "included_rust_fragments_checked": len(formatted), "cost_record": implementation["cost_record"],
        "cost_reuse": {"measured_source_record_sha256": cost["source_record_sha256"],
            "final_source_record_sha256": SOURCE_HASH, "only_subsequent_change": inventory,
            "all_runtime_and_benchmark_source_bytes_identical": True},
        "inherited_warnings": {"identical_cargo_filename_warning_lines": inherited, "strict_clippy_warning_lines": 0},
        "coverage_limits": limits, "remaining_source_review": "evaluation-budget-resource-followup-v504.json",
        "remaining_requirements": limits + ["All 39 properties, 14 targets, crate future work and pre-push verification remain required. Main runtime and all 49 original worktree entries are preserved. No push is requested or claimed."],
    }
    write("evaluation-budget-regression-coverage-v504.json", coverage)
    write("evaluation-budget-resource-followup-v504.json", followup)
    write("typed-api-prototype-v504-metadata.json", metadata)
    print(json.dumps({"five_crate_passed": 5750, "default_scalar_passed": 971,
                      "debug_scalar_passed": 966, "capped_tests": 54, "replay_cases": 2929,
                      "included_rust_files": len(formatted), "cold_zero_budget_outcomes": dict(cold)}))


if __name__ == "__main__":
    main()
