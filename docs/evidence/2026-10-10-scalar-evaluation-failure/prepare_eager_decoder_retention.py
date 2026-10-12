"""Prepare a retention recorder with unchanged correctness audits and scoped cost evidence."""
import ast
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    parent = E / "record_domain_predicate_retention.py"
    target = E / "record_eager_decoder_retention.py"
    preparation = E / "eager-bound-decoder-retention-preparation-v2335.json"
    assert not target.exists() and not preparation.exists()
    original = parent.read_text()
    # Keep the complete broad/cap/replay/receipt/fact validation loops, changing
    # only the four broad phase numbers. Cost and proof provenance are distinct.
    begin = original.index("    validations = {}\n")
    end = original.index('    assert costs["successful_cost_processes_including_standalone"]', begin)
    audit = original[begin:end].replace("{1764: (1074, 1, 0), 1769: (5858, 3, 33),\n                      1770: (1059, 3, 0), 1771: (1074, 1, 0)}",
        "{2268: (1074, 1, 0), 2273: (5858, 3, 33),\n                      2274: (1059, 3, 0), 2275: (1074, 1, 0)}")
    assert audit != original[begin:end]
    prefix = '''"""Retain the isolated eager decoder only after its frozen correctness gates."""
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
SOURCE_HASH = "cc51e3938c1cb6d772778a4683b7412812e234e69cfb1550d6b6e14120d264ad"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def checked_phase(name):
    path = E / name
    result = read(path / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert sha(path / "source-hashes.json.gz") == SOURCE_HASH
    return result


def main():
    outputs = [E / n for n in ["eager-bound-decoder-retention-v2335.json",
        "typed-api-prototype-v2335-metadata.json", "eager-bound-decoder-ledger-update-v2335.json"]]
    assert all(not p.exists() for p in outputs)
    proof_path = E / "eager-bound-decoder-broad-milestone-v2279.json"
    proof = read(proof_path)
    assert all(sha(E / n) == h for n, h in proof["inputs_sha256"].items())
    broad = read(E / "eager-bound-decoder-broad-validation-v2276.json")
    completed = read(E / broad["orchestration_record"])
    costs_path = E / "eager-bound-decoder-computation-review-v2279.json"
    costs = read(costs_path)
    plan_path = E / "eager-bound-decoder-replay-protocol-v2280.json"
    plan = read(plan_path)
    replay = read(E / "eager-bound-decoder-replay-validation-v2333.json")
    queries = read(E / "eager-bound-decoder-domain-query-comparison-v2318.json")
    comparison = read(E / "eager-bound-decoder-deterministic-replay-comparison-v2334.json")
    facts = read(E / "eager-bound-decoder-public-fact-audit-v2334.json")
    for item in [proof, broad, costs, plan, replay, queries, comparison, facts]:
        assert item["source_record_sha256"] == SOURCE_HASH
    assert completed["complete"] and len(completed["phases"]) == 9
    assert replay["complete"] and len(replay["results"]) == 54
    for item in [replay, comparison, facts]:
        assert item["protocol_sha256"] == sha(plan_path)
    assert replay["runner_sha256"] == sha(E / "run_eager_bound_decoder_replays.py")
    assert sha(E / proof["retained_runtime_checkpoint"]) == proof["retained_runtime_checkpoint_sha256"]
    assert proof["verified_units"] == 655 and proof["listed_production_contracts"] == 98
    assert proof["listed_model_theorems"] == 135 and proof["open_obligation_groups"] == 13
    assert proof["fresh_acceptance_tests"] == 10 and proof["fresh_rejected_mutations"] == 245
    assert proof["fresh_passing_guard_baselines"] == 4
    for phase in ["typed-api-prototype-v2260-eager-bound-decoder-verus",
                  "typed-api-prototype-v2264-eager-bound-decoder-acceptance"]:
        checked_phase(phase)
    verification = read(E / "typed-api-prototype-v2260-eager-bound-decoder-verus/proof-reports/verus.json")
    result = verification["verification-results"]
    assert result["success"] and result["verified"] == 655 and result["errors"] == 0
    assert result["is-verifying-entire-crate"] and not result["encountered-error"] and not result["encountered-vir-error"]
    for row in proof["guard_groups"]:
        checked_phase(row["phase"])
        base = E / row["phase"] / "proof-checks"
        assert sha(base / "tooling.json") == row["tooling_sha256"]
        assert sha(base / "verus-invocations.jsonl.gz") == row["invocation_log_sha256"]
    baseline_path = E / "eager-bound-decoder-baseline-v2257/protocol.json"
    baseline = read(baseline_path)
    patch = E / "eager-bound-decoder-baseline-v2257/proposal.patch"
    application = read(E / "eager-bound-decoder-baseline-v2257/application.json")
    assert application["source_record_sha256"] == SOURCE_HASH
    assert sha(patch) == baseline["proposal_patch_sha256"]
    subprocess.run(["git", "apply", "--reverse", "--check", str(patch)], cwd=SOURCE, check=True)
'''
    suffix = '''
    assert sha(costs_path) == plan["cost_qualification_sha256"]
    assert costs["qualified_for_fresh_replays"]
    assert costs["method"] == "unchanged-computation-with-reviewed-diagnostic-metadata"
    assert not costs["original_full_file_identity_rule_satisfied"]
    assert costs["historical_matching_successful_processes"] == 1017
    assert costs["fresh_timing_processes"] == costs["historical_receipts_relabelled_as_candidate_runs"] == 0
    assert not costs["fresh_timing_gate_passed"] and not costs["all_historical_original_caps_pass"]
    assert len(costs["historical_failures"]) == 2
    for failure in costs["historical_failures"]:
        assert failure["original_gate_still_failed"] and failure["failure"]["exit_code"] == -24
        assert sha(E / failure["record"]) == failure["record_sha256"]
    for name, value in costs["static_inputs_sha256"].items():
        assert sha(E / name) == value
    for name, value in costs["historical_artifacts_sha256"].items():
        assert sha(E / name) == value
    binaries = read(E / "eager-bound-decoder-binary-comparison-v2262/summary.json")
    assert binaries["source_record_sha256"] == SOURCE_HASH and binaries["builds_passed"] == 7
    assert not binaries["all_probe_binaries_identical"]
    for row in binaries["results"]:
        assert sha(Path(row["binary"])) == row["binary_sha256"]
    source_record = "typed-api-prototype-v2261-eager-bound-decoder-release-build/source-hashes.json.gz"
    assert sha(E / source_record) == SOURCE_HASH
    sources = json.loads(gzip.decompress((E / source_record).read_bytes()))
    previous = read(E / proof["retained_runtime_checkpoint"])
    old_sources = json.loads(gzip.decompress((E / previous["source_record"]).read_bytes()))
    assert sources.keys() == old_sources.keys() and len(sources) == 24746
    changed = {n: h for n, h in sources.items() if old_sources[n] != h}
    assert changed == proof["changed_sources_sha256"] == baseline["candidate_sources_sha256"] and len(changed) == 5
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    guard = json.loads(gzip.decompress((E / "candidate-preparation/main-hashes.json.gz").read_bytes()))
    assert len(guard) == 1167 and all(sha(ROOT / n) == h for n, h in guard.items())
    limits = [
        "The actual decoder is proved for every u64 and five connected lemmas cover encoding round trips and field updates. Incoming certificate truth, independent existence, actual BigInt metadata, approximation kernels, callers and concurrent publication remain separate obligations; all thirteen whole-crate groups stay open.",
        "The 655 verified units, 245 rejected mutations, four positive baselines and ten acceptance tests describe the conditional target. They do not satisfy the pending fixed pre-Verus acceptance report or a whole-crate proof.",
        "The nine runtime commands cover scalar configurations and five consumer crates, not complete repository CI. The 33 ignored cases and 14 inherited Cargo filename warnings are unchanged; both strict Clippy configurations have no warnings.",
        "The 39 capped commands cover 156 top-level tests and one child under 192 MiB, CPU 2/3-second and four-second wall caps. The 648 public schedules retain those limits. Numerical replay limits are inherited unchanged, including older three-second wall limits without the later CPU guard. The 78 private-kernel cases check result shapes; 4129 is not a count of independent numerical oracles.",
        "Deterministic samples, typed errors, work, facts and recorded allocations agree. Only measured durations and known executable paths are normalized. The 78 concurrent public cases compare setup/oracle/process contracts, with a separate observed-fact audit; thread stress is not exhaustive interleaving verification.",
        "All seven frozen probe instruction/load layouts match v1836. The complete files differ in reviewed diagnostic/ELF/private-symbol metadata, and panic line numbers can change. The 194-reference audit is not exhaustive indirect-pointer analysis or a whole-program equivalence theorem. This evidence applies to these frozen compiler/target/build inputs.",
        "The separately frozen unchanged-computation method supports differential cost qualification. Its 1017 matching historical successes remain older v1836 executions; no fresh timing workload or new timing pass is claimed. Both later v1836 algebraic SIGXCPU results at v2100/v2247 remain failures of the same retained program under original caps. The method does not repair them or guarantee absolute resource bounds.",
        "Every earlier rejected variant, unresolved timing increase, original-cap failure, the retained integer-control tradeoff and the fixed pre-Verus baseline remain archived. No failed complete-file or measured-cost gate is relabeled passed. This retention uses the separate v2279 method and the new fresh correctness gates.",
        "EvaluationBudget excludes aggregate bytes/time and unmetered exact algebraic work. Construction and structural inspection are outside query allowances. Reconstructed graphs discard sharing/caches while shared process constants may be warm. Exact-rational flags are checked for retention, not general recognition soundness.",
        "Main retains the original scalar failure and all 49 original worktree entries. This is isolated retention only; native combination, full-board admission, all 39 properties, 14 targets, crate future work and complete pre-push verification remain in scope. No completion status or push follows.",
    ]
    next_priority = (
        "Continue from the retained eager decoder by connecting numerical bit-pattern decisions "
        "and checked exponent arithmetic to closed-interval bound certificates. Preserve metadata "
        "short-circuiting and distinguish conditional model bridges from verified production "
        "callers and dependency storage. Continue producer truth, independent existence, atomic "
        "publication and exact-operation resource work; the repeated baseline algebraic CPU "
        "failures, fixed pre-Verus baseline and main/native/full-board requirements remain open."
    )
    record = {
        "status": "Retain the isolated proved eager bound decoder; main admission and resource obligations remain open",
        "source_record": source_record, "source_record_sha256": SOURCE_HASH,
        "previous_checkpoint": proof["retained_runtime_checkpoint"],
        "previous_checkpoint_sha256": proof["retained_runtime_checkpoint_sha256"],
        "baseline": str(baseline_path.relative_to(E)), "baseline_sha256": sha(baseline_path),
        "production_patch": str(patch.relative_to(E)), "production_patch_sha256": sha(patch),
        "changed_sources_sha256": changed, "proof_record": proof_path.name, "proof_record_sha256": sha(proof_path),
        "conditional_proof": {"verified_units": 655, "errors": 0, "listed_production_contracts": 98,
            "listed_model_theorems": 135, "open_obligation_groups": 13, "fresh_acceptance_tests": 10,
            "fresh_rejected_mutants": 245, "fresh_passing_guard_baselines": 4, "reused_rejected_mutants": 0},
        "fresh_cargo_and_format_validations": validations, "fresh_capped_validations": caps,
        "capped_test_counts": cap_counts, "replay_orchestration": "eager-bound-decoder-replay-validation-v2333.json",
        "replays": summaries, "replay_cases": dict(replay_counts),
        "domain_observation_comparison": "eager-bound-decoder-domain-query-comparison-v2318.json",
        "deterministic_replay_comparison": "eager-bound-decoder-deterministic-replay-comparison-v2334.json",
        "public_fact_audit": "eager-bound-decoder-public-fact-audit-v2334.json", "public_fact_observations": facts["reports"],
        "cost_record": costs_path.name, "cost_record_sha256": sha(costs_path), "cost_method": costs["method"],
        "fresh_timing_processes": 0, "historical_matching_successful_processes": 1017,
        "later_baseline_cpu_failures_preserved": costs["historical_failures"],
        "original_complete_file_identity_gate_passed": False, "fresh_timing_gate_passed": False,
        "decision": "Keep the actual all-word decoder and connected proof lemmas after all frozen correctness, replay and fact gates. The separately frozen v2279 static computation review supplies proportionate differential-cost evidence for this proof extension, while inheriting the unchanged program's known resource failures. There is no main admission, fresh timing pass, absolute cap guarantee or property completion.",
        "limits": limits, "next_priority": next_priority,
        "remaining_source_review": "numerical-bound-metadata-review-v2279-v2.json",
        "remaining_source_review_sha256": sha(E / "numerical-bound-metadata-review-v2279-v2.json"),
    }
    ledger = ROOT / "docs/properties-status.json"
    before_bytes = ledger.read_bytes()
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == before_bytes
    before, after = json.loads(before_bytes), json.loads(before_bytes)
    assert len(after["properties"]) == 39 and len(after["targets"]) == 14
    p4 = next(row for row in after["properties"] if row["id"] == "P4")
    for name in [record["replay_orchestration"], record["domain_observation_comparison"],
        record["deterministic_replay_comparison"], record["public_fact_audit"], outputs[0].name,
        "typed-api-prototype-v2335-state.json", record["remaining_source_review"]]:
        value = str(E.relative_to(ROOT) / name)
        assert value not in p4["evidence"]
        p4["evidence"].append(value)
    update = (
        "The v2335 isolated checkpoint retains the proved eager bound decoder: 655 conditional "
        "units, 245 rejected mutations, four positive baselines, ten acceptance checks, nine "
        "runtime commands, 39 capped phases with 156 top-level tests plus one child, 4129 "
        "numerical cases and 648 public schedules. All 1724 domain observations and all "
        "deterministic non-timing receipts agree. Facts survive 6144 joined-final, 85176 root "
        "and 9360 observed-child comparisons; concurrent outcomes remain schedule-dependent. "
        "Seven instruction-identical probes and the separately frozen diagnostic-metadata "
        "review qualify differential cost evidence. The 1017 supporting successes remain "
        "historical; both later same-baseline CPU failures stay failed. Whole-file identity "
        "and fresh timing gates are not passed or relabeled. Producer truth, dependency "
        "storage, independent existence, atomic/caller ordering, all thirteen whole-crate "
        "groups, resource work and main/native/full-board admission remain open. All 39 "
        "properties, 14 targets and crate future work keep their requirements and statuses."
    )
    p4["remaining"] += " " + update
    old_prefix = "Continue the isolated fallible scalar evaluator from typed-api-prototype-v1836-state.json before production admission."
    assert after["next_actions"][0].startswith(old_prefix)
    after["next_actions"][0] = after["next_actions"][0].replace(old_prefix,
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v2335-state.json before production admission.", 1)
    assert after["next_actions"][0].count(proof["next_priority"]) == 1
    after["next_actions"][0] = after["next_actions"][0].replace(proof["next_priority"], next_priority, 1)
    after["next_actions"][0] += " " + update
    same = copy.deepcopy(after)
    original_p4 = next(row for row in before["properties"] if row["id"] == "P4")
    modified_p4 = next(row for row in same["properties"] if row["id"] == "P4")
    modified_p4["evidence"], modified_p4["remaining"] = original_p4["evidence"], original_p4["remaining"]
    same["next_actions"][0] = before["next_actions"][0]
    assert same == before
    metadata = {"previous_checkpoint": proof["retained_runtime_checkpoint"], "status": record["status"],
        "contract_decision": "p4-contract-decision-v173.json",
        "accepted_cache_rule": "Warming may strengthen Unknown into Certified and reuse higher-precision proofs at zero work; no contradiction or weakening of an available certificate.",
        "evidence_commits": ["467569e05b0e8475df089e83f4e2ac63ba62102a", "4e50b4aba045e2eed88033d71590a957de7cfc6b"],
        "retention_record": outputs[0].name, "production_patch": record["production_patch"],
        "production_patch_sha256": sha(patch), "fresh_validation_phases": {"cargo_and_format": 9, "separately_verified_capped_binary": 39},
        "cap_record_scope": "All direct-binary cap receipts and nested counts are checked in the retention record; pass only nine Cargo/format phases to the checkpoint parser.",
        "fresh_stack_counts": {"top_level_passed": 5858, "child_process_passed": 3, "ignored": 33},
        "fresh_scalar_counts": {"top_level_passed": 1074, "child_process_passed": 1},
        "fresh_replay_cases": dict(replay_counts), "conditional_proof_scope": proof_path.name,
        "conditional_proof": record["conditional_proof"], "cost_record": costs_path.name,
        "cost_method": costs["method"], "fresh_timing_processes": 0,
        "original_cost_guard_failures_preserved": True, "coverage_limits": limits,
        "main_runtime_admitted": False, "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
        "remaining_source_review": record["remaining_source_review"],
        "remaining_requirements": [next_priority,
            "Preserve every original full-size regression, corpus, process cap, failure and retained tradeoff. Thread stress and conditional lemmas do not replace graph/producer/existence/atomic proofs.",
            "Continue constructor/coefficient copies, exact-operation CPU/memory use and unshared expression growth before native combination and unchanged full-board admission.",
            "All thirteen whole-crate groups and the fixed pre-Verus baseline remain open. Complete the whole properties.md plan and repository pre-push checks; no property/target completion follows from this isolated checkpoint."]}
    outputs[0].write_text(json.dumps(record, indent=2) + "\\n")
    metadata["retention_record_sha256"] = sha(outputs[0])
    outputs[1].write_text(json.dumps(metadata, indent=2) + "\\n")
    ledger.write_text(json.dumps(after, indent=2) + "\\n")
    outputs[2].write_text(json.dumps({"before_sha256": hashlib.sha256(before_bytes).hexdigest(),
        "after_sha256": sha(ledger), "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "retention_record_sha256": sha(outputs[0]), "metadata_sha256": sha(outputs[1]),
        "retained_runtime_checkpoint": "typed-api-prototype-v2335-state.json", "main_runtime_admitted": False}, indent=2) + "\\n")
    print(json.dumps({"status": record["status"], "broad_commands": 9, "capped_phases": 39,
        "capped_tests": cap_counts, "replay_cases": dict(replay_counts), "changed_source_files": len(changed)}))


if __name__ == "__main__":
    main()
'''
    code = prefix + audit + suffix
    ast.parse(code)
    target.write_text(code)
    preparation.write_text(json.dumps({"predecessor": parent.name, "predecessor_sha256": sha(parent),
        "runner": target.name, "runner_sha256": sha(target),
        "reused_audit_block_sha256": hashlib.sha256(audit.encode()).hexdigest(),
        "audit_change": "Only four broad phase numbers change in the complete broad/cap/replay/receipt/fact validation block. Proof provenance uses the current 655-unit split-guard milestone. Cost qualification uses the separately frozen v2279 computation review while requiring both later baseline failures to remain failed. New source/ledger/checkpoint metadata names v2335.",
        "execution": "Prepared only; all replay and fact gates must complete before this recorder can run."}, indent=2) + "\n")
    print(json.dumps({"prepared": target.name, "runtime_retained": False}))


if __name__ == "__main__":
    main()
