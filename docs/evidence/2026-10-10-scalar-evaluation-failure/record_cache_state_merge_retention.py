"""Retain compatible cached-bound merging only after all frozen runtime gates."""
from collections import Counter
import copy
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
from parse_test_results_v3 import parse_phase
from record_domain_sign_retention_v3 import compact_summary, direct_test_counts

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
SOURCE_HASH = "555cf4a48df68e021119c107412824b5879baf48e252c6fb1c8d2bc22baf1c59"
read = lambda path: json.loads(path.read_text())
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def checked_phase(name):
    path = E / name
    result = read(path / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert sha(path / "source-hashes.json.gz") == SOURCE_HASH
    return result


def main():
    outputs = [E / name for name in ["cache-state-merge-retention-v1999.json",
        "typed-api-prototype-v1999-metadata.json", "cache-state-merge-ledger-update-v1999.json"]]
    assert all(not path.exists() for path in outputs)
    proof = read(E / "cache-state-merge-proof-validation-v1926.json")
    broad = read(E / "cache-state-merge-broad-validation-v1935.json")
    completed = read(E / broad["orchestration_record"])
    costs = read(E / "cache-state-merge-cost-review-v1943.json")
    plan_path = E / "cache-state-merge-replay-protocol-v1944.json"
    plan = read(plan_path)
    replay = read(E / "cache-state-merge-replay-validation-v1997.json")
    queries = read(E / "cache-state-merge-domain-query-comparison-v1982.json")
    comparison = read(E / "cache-state-merge-deterministic-replay-comparison-v1998.json")
    facts = read(E / "cache-state-merge-public-fact-audit-v1998.json")
    for item in [proof, broad, costs, plan, replay, queries, comparison, facts]:
        assert item["source_record_sha256"] == SOURCE_HASH
    assert completed["complete"] and len(completed["phases"]) == 9
    assert replay["complete"] and len(replay["results"]) == 54
    for item in [replay, comparison, facts]:
        assert item["protocol_sha256"] == sha(plan_path)
    assert replay["runner_sha256"] == sha(E / "run_cache_state_replays.py")
    assert sha(E / proof["retained_runtime_checkpoint"]) == proof["retained_runtime_checkpoint_sha256"]
    assert proof["verified_units"] == 651 and proof["errors"] == 0
    assert proof["listed_production_contracts"] == 98 and proof["listed_model_theorems"] == 135
    assert proof["open_obligation_groups"] == 13 and proof["fresh_acceptance_tests"] == 10
    assert proof["fresh_rejected_mutants"] == 247 and proof["fresh_passing_guard_baselines"] == 4
    assert proof["reused_rejected_mutants"] == 0
    for name, digest in proof["prior_experiments_sha256"].items():
        assert sha(E / name) == digest
    rejected = read(E / "bound-merge-cost-rejection-v1857.json")
    assert not rejected["runtime_retained"]
    for name, diagnostic in proof["compile_only_diagnostics"].items():
        assert sha(E / name / "summary.json") == diagnostic["summary_sha256"]
        assert diagnostic["timed_processes"] == 0
    for key in ["fresh_verification_phase", "fresh_acceptance_phase", "fresh_rejection_phase"]:
        checked_phase(proof[key])
    assert len(proof["proof_and_guard_inputs_sha256"]) == 44
    for name, digest in proof["proof_and_guard_inputs_sha256"].items():
        assert sha(SOURCE / name) == digest
    assert sha(E / proof["proposal_patch"]) == proof["proposal_patch_sha256"]
    subprocess.run(["git", "apply", "--reverse", "--check", str(E / proof["proposal_patch"])],
                   cwd=SOURCE, check=True)

    validations = {}
    for name, row in completed["phases"].items():
        assert checked_phase(name) == row["result"]
        tests = parse_phase(E / name)
        assert tests == broad["test_summaries"][name]
        validations[name] = {"result": row["result"], "tests": tests}
    expected_tests = {1927: (1074, 1, 0), 1932: (5858, 3, 33),
                      1933: (1059, 3, 0), 1934: (1074, 1, 0)}
    for version, expected in expected_tests.items():
        matches = [row["tests"] for name, row in validations.items()
                   if name.startswith(f"typed-api-prototype-v{version}-")]
        assert len(matches) == 1
        counts = matches[0]
        assert tuple(counts[key] for key in ["top_level_passed", "child_process_passed", "top_level_ignored"]) == expected
        assert counts["top_level_failed"] == counts["child_process_failed"] == 0
    assert broad["included_rustfmt_files"] == 108 and broad["ignored_test_names_unchanged"]
    assert len(broad["inherited_stack_warning_lines"]) == 14 and broad["strict_clippy_warning_lines"] == 0

    caps = {}
    for row in replay["results"]:
        if row["kind"] != "cap":
            continue
        name = row["phase"]
        assert checked_phase(name) == row["result"]
        cap = row["cap"]
        raw = [line for line in (E / name / "stdout.log").read_text().splitlines()
               if line.startswith('{"command":')]
        assert raw and json.loads(raw[-1]) == cap
        assert cap["exit_code"] == 0 and not cap["wall_timeout"]
        assert cap["address_space_bytes"] == 192 * 1024 * 1024
        assert cap["cpu_seconds_soft_hard"] == [2, 3] and cap["wall_seconds_limit"] == 4
        assert sha(Path(cap["command"][0])) == cap["binary_sha256"]
        counts = direct_test_counts(E / name)
        assert counts == direct_test_counts(E / row["baseline_phase"])
        caps[name] = {**row, "test_counts": counts}
    assert len(caps) == 39
    cap_counts = {key: sum(row["test_counts"][key] for row in caps.values())
                  for key in ["top_level_passed", "child_process_passed"]}
    assert cap_counts == {"top_level_passed": 156, "child_process_passed": 1}

    summaries = {}
    replay_counts = Counter()
    for row in replay["results"]:
        if row["kind"] == "cap":
            continue
        path = E / row["phase"]
        assert row["exit_code"] == 0 and sha(path / "summary.json") == row["summary_sha256"]
        summary = read(path / "summary.json")
        assert summary["source_hashes_sha256"] == SOURCE_HASH and summary["guards_unchanged"]
        count = row["expected_cases"]
        if count == 415:
            assert summary["ordinary_cases"] == 400 and summary["typed_interruption_errors"] == 300
            assert len(summary["extremes"]) == 6 and len(summary["cache"]) == 9
            assert all(item["valid"] for item in summary["extremes"] + summary["cache"])
            assert all(item["exit_code"] == 0 for item in summary["commands"])
        else:
            assert summary["cases"] == count
        for key, value in summary.items():
            if key.startswith("invalid_") or key in {"incomplete_cases", "unexpected_result_shapes", "nonzero_exit_cases", "typed_errors"}:
                assert not value, (path.name, key, value)
        summaries[path.name] = {"kind": row["kind"], "cases": count,
            "summary_sha256": sha(path / "summary.json"), "summary": compact_summary(summary)}
        replay_counts[row["kind"]] += count
    assert replay_counts == {"numerical": 4129, "public": 648}
    assert comparison["compared_cases"] == {"numerical": 4129, "public_deterministic": 570, "concurrent_setup_only": 78}
    assert not comparison["differences"] and not queries["differences"]
    assert queries["observed_counts"]["public"] == 1584
    assert queries["observed_counts"]["public_success"] == 453
    assert queries["observed_counts"]["public_exhausted"] == 1131
    public = facts["reports"]["public"]["observed_counts"]
    composed = facts["reports"]["composed"]["observed_counts"]
    assert public["joined_final_fact_comparisons"] == 6144
    assert public["race_worker_samples"] + public["race_worker_exhausted"] == 4608
    assert composed["root_fact_retention_comparisons"] == 85176
    assert composed["child_fact_retention_comparisons"] == 9360
    assert composed["events:cached"] == 13104
    cost_controller_path = E / "cache-state-merge-cost-validation-v1943.json"
    cost_controller = read(cost_controller_path)
    cost_milestone = read(E / "cache-state-merge-cost-milestone-v1943.json")
    assert cost_controller["complete"] and costs["local_timing_and_process_gates_passed"]
    assert not costs["persistent_above_five_percent_increases"]
    assert cost_milestone["cost_controller_sha256"] == sha(cost_controller_path)
    assert cost_milestone["cost_record_sha256"] == sha(E / "cache-state-merge-cost-review-v1943.json")
    assert cost_milestone["source_record_sha256"] == cost_controller["source_record_sha256"] == SOURCE_HASH
    assert subprocess.check_output(["git", "show", "HEAD:" + str((E / "cache-state-merge-cost-review-v1943.json").relative_to(ROOT))], cwd=ROOT) == (E / "cache-state-merge-cost-review-v1943.json").read_bytes()
    repeats = sum(row["successful_processes"] for row in costs["followups"])
    processes, preludes = 1404 + repeats, 936 + repeats
    assert costs["successful_cost_processes_including_standalone"] == processes
    assert costs["separate_cpu_preparation_processes"] == preludes and costs["original_process_caps_preserved"]
    assert costs["original_run"]["complete_fixtures"] == 52
    assert costs["original_run"]["successful_processes"] == 936 and costs["original_run"]["failed_attempts"] == 0
    assert all(row["failed_attempts"] == 0 for row in costs["followups"])
    assert all(row["non_timing_outputs_identical"] for row in costs["families"].values())
    assert all(case["median_percent_change"] <= 5 for row in costs["followups"]
               for cases in row["cases"].values() for case in cases)

    source_record = "typed-api-prototype-v1929-cache-state-merge-release-build/source-hashes.json.gz"
    assert sha(E / source_record) == SOURCE_HASH
    sources = json.loads(gzip.decompress((E / source_record).read_bytes()))
    previous = read(E / proof["retained_runtime_checkpoint"])
    before_sources = json.loads(gzip.decompress((E / previous["source_record"]).read_bytes()))
    assert sources.keys() == before_sources.keys() and len(sources) == 24746
    changed = {name: value for name, value in sources.items() if before_sources[name] != value}
    assert changed == proof["changed_sources_sha256"] and len(changed) == 6
    for name, value in sources.items():
        assert sha(SOURCE / name) == value, name

    limits = [
        "The proof includes the actual production BoundCache::merge body, including the uninitialized cache case. Ordinary Rust calls that same outlined selector. Two valid bounds for the same real value are compatible; merging them preserves denotation, all available certificates and accepted domain predicates. Producer soundness and independent real existence remain premises. The atomic load/CAS loop, caller ordering and graph semantics remain separate obligations; all thirteen whole-crate groups remain open.",
        "Ten acceptance/dependency tests and 247 rejected mutations plus four passing baselines do not satisfy the still-pending fixed pre-Verus acceptance report. The outlined extraction has no failed proof attempt. The earlier BoundInfo extraction failed its timing gate; its rejection, the unsupported Option::or probe and the compile-only variants remain immutable.",
        "The nine broad commands cover scalar configurations and the five-crate consumer stack, not every workspace/CI job. Thirty-three ignored tests and fourteen inherited Cargo example-filename warnings remain; both strict Clippy configurations have zero warnings.",
        "The 39 direct-binary caps and 648 public schedules retain 192 MiB, CPU 2/3-second and four-second wall limits. Numerical corpora preserve their individual original limits; the base and precision corpora use a three-second wall limit without the later CPU guard. The 78 private-kernel cases check result shapes, so 4129 is not a count of independent numerical oracle checks.",
        "All deterministic observations remain identical, including samples, typed errors, work, facts and recorded allocations. Only duration fields and known executable phase paths are normalized. The 78 concurrent public cases compare setup/oracle/process contracts separately from schedule-dependent outcomes.",
        "The public fact audit checks observed worker/root sequences and joined final snapshots. Thread stress is not exhaustive interleaving verification. Composed child snapshots start at the first serialized observation; the frozen Rust probe separately checks the initial child state. Exact-rational flags are checked for retention, not general recognition soundness.",
        "The 1584 domain queries include 453 successes and 1131 Exhausted answers, unchanged from v1836. The zero-Rust-allocation observation applies to 56 private certificate queries. The 48 endpoint/invalid/upper-only controls, 36 fine-retry schedules and 144 cached retry queries remain unchanged; these are nested in the nine domain cap phases, not additional executable processes.",
        "All fresh standalone, original paired and prescribed family-repeat processes pass their original caps. The cost record preserves all initial/repeated timing observations and separate executable sizes. These measurements establish neither causal attribution, a uniform speedup nor whole-board performance/size parity.",
        "The local cost baseline remains v1836. Older original-cap failures at v1638/v1710, the rejected BoundInfo extraction's v1857 timing regressions, the earlier retained integer-control tradeoff and the fixed pre-Verus acceptance baseline are not erased or reset by this successful comparison.",
        "EvaluationBudget excludes aggregate bytes/time and unmetered algebraic helpers. Construction and structural inspection are outside the public query allowances. Reconstructed graphs discard sharing/caches while process-global constants may already be warm.",
        "This retains an isolated candidate only. Main retains its original scalar failure and all 49 preexisting worktree entries. Native combination, full-board admission, all 39 properties, 14 targets and crate future work remain in scope; no property/target status changes and no push follow from this checkpoint.",
    ]
    next_priority = (
        "Connect the actual cache-word decoder to the existing model, following cache-word-next-review-v1926.json. "
        "Continue implementation-connected review of bound producers, independent real existence "
        "and atomic publication, using the retained public and composed schedules as regression "
        "controls. Conditional merge and domain soundness do not discharge those prerequisites. "
        "Continue the remaining constructor/coefficient and exact-operation resource work before "
        "native combination and unchanged full-board admission."
    )
    record = {
        "status": "Retain the isolated proved outlined cache-state merge; main admission remains open",
        "source_record": source_record, "source_record_sha256": SOURCE_HASH,
        "previous_checkpoint": proof["retained_runtime_checkpoint"],
        "previous_checkpoint_sha256": proof["retained_runtime_checkpoint_sha256"],
        "baseline": proof["baseline"], "baseline_sha256": proof["baseline_sha256"],
        "production_patch": proof["proposal_patch"], "production_patch_sha256": proof["proposal_patch_sha256"],
        "prior_experiments_sha256": proof["prior_experiments_sha256"],
        "compile_only_diagnostics": proof["compile_only_diagnostics"],
        "next_production_model_review": "cache-word-next-review-v1926.json",
        "next_production_model_review_sha256": sha(E / "cache-word-next-review-v1926.json"),
        "changed_sources_sha256": changed,
        "proof_record": "cache-state-merge-proof-validation-v1926.json",
        "proof_record_sha256": sha(E / "cache-state-merge-proof-validation-v1926.json"),
        "conditional_proof": {key: proof[key] for key in ["verified_units", "errors", "listed_production_contracts",
            "listed_model_theorems", "open_obligation_groups", "fresh_acceptance_tests", "fresh_rejected_mutants",
            "fresh_passing_guard_baselines", "reused_rejected_mutants"]},
        "fresh_cargo_and_format_validations": validations,
        "fresh_capped_validations": caps, "capped_test_counts": cap_counts,
        "replay_orchestration": "cache-state-merge-replay-validation-v1997.json",
        "replays": summaries, "replay_cases": dict(replay_counts),
        "domain_observation_comparison": "cache-state-merge-domain-query-comparison-v1982.json",
        "deterministic_replay_comparison": "cache-state-merge-deterministic-replay-comparison-v1998.json",
        "public_fact_audit": "cache-state-merge-public-fact-audit-v1998.json",
        "public_fact_observations": facts["reports"],
        "cost_record": "cache-state-merge-cost-review-v1943.json",
        "cost_processes": processes, "separate_cpu_preludes": preludes,
        "initial_and_followup_percent": costs["initial_and_followup_percent"],
        "decision": "Keep the exact runtime bound selection and route it through the same pure BoundCache::merge body included by Verus. The extraction adds conditional denotation, compatibility, certificate-retention and domain-retention proofs without losing any observed successful query, fact, work allowance or allocation behavior in the frozen deterministic corpora. Original-cap costs and the predeclared repeat rule pass. This is the new isolated baseline; main runtime admission remains open.",
        "limits": limits, "remaining_source_review": "domain-bound-deferred-review-v1702.json",
        "remaining_source_review_scope": "Historical review of v1754. Its public/concurrent/composed coverage gaps now have finite replay evidence, and cached-bound domain decisions and compatible merging have conditional production proofs. Producer/existence/atomic/caller and resource obligations remain open.",
        "next_priority": next_priority,
    }
    evidence_names = [record["replay_orchestration"], record["domain_observation_comparison"],
        record["deterministic_replay_comparison"], record["public_fact_audit"], outputs[0].name,
        "typed-api-prototype-v1999-state.json"]
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before = json.loads(before_bytes)
    after = copy.deepcopy(before)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for name in evidence_names:
        path = str(E.relative_to(ROOT) / name)
        assert path not in p4["evidence"]
        p4["evidence"].append(path)
    update = (
        "The v1999 isolated checkpoint retains the proved compatible cached-bound merge "
        "after all frozen gates: 651 verified units, 247 rejected mutations, four passing guard "
        "baselines, ten acceptance tests, nine broad commands, 39 capped phases (156 top-level "
        "tests plus one child), 4129 numerical cases and 648 public reuse schedules. All 1724 "
        "domain observations and all deterministic numerical/public receipt fields remain "
        "identical apart from times and known phase paths. Available facts survive 6144 joined "
        "final comparisons, 85176 root comparisons and 9360 observed-child comparisons. "
        "Concurrent outcome counts remain schedule-dependent and are audited for invariants. "
        f"The {processes} original-cap cost processes and {preludes} CPU preludes pass, "
        "including all prescribed family repeats; older cost failures/tradeoff and the fixed pre-Verus "
        "baseline remain explicit. This changes only the isolated retained candidate. "
        "Incoming-bound soundness, independent existence, atomic publication, caller ordering, "
        "all thirteen whole-crate groups, main/native/full-board admission and the full "
        "39-property/14-target/crate-future-work scope remain open."
    )
    p4["remaining"] += " " + update
    old_checkpoint = "Continue the isolated fallible scalar evaluator from typed-api-prototype-v1836-state.json before production admission."
    assert after["next_actions"][0].startswith(old_checkpoint)
    after["next_actions"][0] = after["next_actions"][0].replace(old_checkpoint,
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v1999-state.json before production admission.", 1)
    old_pending = "Next run all capped/numerical/public replays and receipt/fact audits before deciding retention."
    assert after["next_actions"][0].count(old_pending) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(old_pending,
        "All capped/numerical/public replays and receipt/fact audits pass; the extraction is retained in v1999. " + next_priority, 1)
    after["next_actions"][0] += " " + update
    unchanged = copy.deepcopy(after)
    original_p4 = next(row for row in before["properties"] if row["id"] == "P4")
    compare_p4 = next(row for row in unchanged["properties"] if row["id"] == "P4")
    compare_p4["evidence"], compare_p4["remaining"] = original_p4["evidence"], original_p4["remaining"]
    unchanged["next_actions"][0] = before["next_actions"][0]
    assert unchanged == before
    metadata = {
        "previous_checkpoint": proof["retained_runtime_checkpoint"], "status": record["status"],
        "contract_decision": "p4-contract-decision-v173.json",
        "accepted_cache_rule": "Warming may strengthen Unknown into Certified and reuse higher-precision proofs at zero work; no contradiction or weakening of an available certificate.",
        "evidence_commits": ["dec6ce44ffc109de6f8f52d2ce253cec1cae5dbf", subprocess.check_output(["git", "log", "-1", "--format=%H", "--", str((E / "cache-state-merge-broad-milestone-v1935.json").relative_to(ROOT))], cwd=ROOT, text=True).strip(), subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()],
        "retention_record": outputs[0].name,
        "production_patch": record["production_patch"], "production_patch_sha256": record["production_patch_sha256"],
        "fresh_validation_phases": {"cargo_and_format": 9, "separately_verified_capped_binary": 39},
        "cap_record_scope": "The retention record checks source identity, actual cap receipts and direct-binary nested counts for all 39 capped phases. Pass only the nine Cargo/format phases to the checkpoint Cargo parser.",
        "fresh_stack_counts": {"top_level_passed": 5858, "child_process_passed": 3, "ignored": 33},
        "fresh_scalar_counts": {"top_level_passed": 1074, "child_process_passed": 1},
        "fresh_replay_cases": dict(replay_counts), "conditional_proof_scope": record["proof_record"],
        "conditional_proof": record["conditional_proof"],
        "cost_record": record["cost_record"], "original_cost_guard_failures_preserved": True,
        "coverage_limits": limits, "main_runtime_admitted": False,
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
        "remaining_source_review": record["remaining_source_review"],
        "remaining_source_review_scope": record["remaining_source_review_scope"],
        "remaining_requirements": [next_priority,
            "Preserve the frozen public, composed, private, deterministic-publication and numerical corpora; their finite coverage does not replace producer/existence/atomic proofs or kernel interruption coverage.",
            "Continue bounded-query constructors, lazy/scaled Laurent and quadratic-tower coefficient copies, unshared symbolic growth and construction-time comparison without changing ordinary exactness or the eight-term recognition limit. Both prior acos and both atan allocation variants remain rejected.",
            "Trace direct Real inverse-trig/radical/tower reachability from public Computable allowances, keeping payload diagnostics distinct from native Real contracts. Measure smooth-prime logarithm factoring, power-of-two extraction, exact-operation CPU and native memory on unchanged full-size fixtures.",
            "All thirteen whole-crate proof groups and the fixed pre-Verus acceptance baseline remain open. Native-prototype combination and unchanged full-board admission are required before main runtime changes.",
            "Continue all 39 properties, 14 targets and crate future work. Preserve the original scalar failure, 49 preexisting worktree entries, older cost failures and retained integer tradeoff; no completion status changes at this checkpoint."],
    }
    outputs[0].write_text(json.dumps(record, indent=2) + "\n")
    metadata["retention_record_sha256"] = sha(outputs[0])
    outputs[1].write_text(json.dumps(metadata, indent=2) + "\n")
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    outputs[2].write_text(json.dumps({
        "before_sha256": hashlib.sha256(before_bytes).hexdigest(), "after_sha256": sha(ledger),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "retention_record_sha256": sha(outputs[0]), "metadata_sha256": sha(outputs[1]),
        "retained_runtime_checkpoint": "typed-api-prototype-v1999-state.json", "main_runtime_admitted": False,
    }, indent=2) + "\n")
    print(json.dumps({"status": record["status"], "broad_commands": len(validations),
        "capped_phases": len(caps), "capped_tests": cap_counts, "replay_cases": dict(replay_counts),
        "changed_source_files": len(changed), "source_files_checked": len(sources)}))


if __name__ == "__main__":
    main()
