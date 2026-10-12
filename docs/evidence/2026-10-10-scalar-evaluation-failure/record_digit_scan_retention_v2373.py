"""Retain the conditional digit-scan proof while preserving runtime and scope limits."""
import copy
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E / "digit-scan-retention-v2373.json"
    metadata_path = E / "typed-api-prototype-v2373-metadata.json"
    ledger_record = E / "digit-scan-ledger-update-v2373.json"
    assert all(not p.exists() for p in [out, metadata_path, ledger_record])
    previous_path = E / "typed-api-prototype-v2358-state.json"
    assert sha(previous_path) == "5af0c6067447ffcf6f531b7b3cb206b83532225b99f6c8a1f66cbca7698bc991"
    previous = read(previous_path)
    audit_path = E / "digit-scan-input-audit-v2372.json"
    audit = read(audit_path)
    assert audit["previous_checkpoint_sha256"] == sha(previous_path)
    assert audit["runtime_source_files_changed"] == 0 and audit["historical_runtime_evidence_applicable"]
    assert audit["fresh_rejected_mutations"] == 19 and audit["reused_rejected_mutations"] == 267
    assert audit["total_applicable_rejected_mutations"] == 286 and audit["total_applicable_positive_baselines"] == 6
    assert audit["fresh_acceptance_tests"] == 10 and audit["included_format_files"] == 111
    assert len(audit["later_baseline_cpu_failures_preserved"]) == 2
    record = E / audit["source_record"]
    assert sha(record) == audit["source_record_sha256"]
    sources = json.loads(gzip.decompress(record.read_bytes()))
    assert len(sources) == 24749 and all(sha(SOURCE / n) == h for n, h in sources.items())
    assert audit["conditional_proof"] == {"verified_units": 716, "errors": 0,
        "production_contracts": 98, "model_theorems": 160, "open_obligation_groups": 13}
    conditional = {"verified_units": 716, "errors": 0, "listed_production_contracts": 98,
        "listed_model_theorems": 160, "open_obligation_groups": 13, "fresh_acceptance_tests": 10,
        "fresh_rejected_mutants": 19, "fresh_passing_guard_baselines": 1,
        "reused_rejected_mutants": 267, "reused_passing_guard_baselines": 5,
        "total_applicable_rejected_mutants": 286, "total_applicable_passing_guard_baselines": 6}
    limits = [
        "Seventeen added conditional model lemmas cover radix-parametric multiword denotation, empty/all-maximal storage, normalized zero and bit length, first-nonzero/nonmaximal digit premises, and the public u32 leading-zero contract. They connect to the actual retained power-of-two/all-ones flag lemmas. The model uses natural digits and unbounded counts; actual dependency digit views, normalization, iterators, u64 count range and BigInt sign/magnitude imports remain open.",
        "The full conditional target passes 716 units, zero errors, 98 production contracts and 160 model theorems. Nineteen new negatives and one baseline pass, with ten acceptance tests and workspace/111-input formatting. The 267 old negatives/five baselines retain their original identities under v2372: all 272 temporary entrypoints reconstruct exactly from unchanged generators and original paths. The audit inherits v2357 and its explicit v2344 import-order correction; old runs are not relabelled as fresh.",
        "Six verification files change, including a new model and guard module. Ordinary runtime, build, test corpus and probe sources remain unchanged. Historical v2335 runtime, cap, cost and replay evidence remains applicable with all original limits and failures. No fresh runtime, cost or replay process is credited.",
        "The standalone generic probe passes 25 units and the connected probe passes 418 units including existing imported proofs; neither count is added to retained coverage. The failed int-to-nat proposal, unavailable closed-spec reveal and incorrect relative launcher path remain archived. The correction uses public pinned bit-position contracts without introducing trusted assumptions. Final entrypoint sorting and guard layout preserve module membership and the Python AST; the complete proof and fresh guards execute on that final source."
    ] + previous["coverage_limits"][4:]
    next_priority = (
        "Connect the generic digit denotation and scan lemmas to typed u32/u64 dependency storage "
        "and actual normalization/first-match iterator behavior. Discharge machine-width metadata "
        "ranges and BigInt sign/magnitude imports, retaining the producer's early Unknown and "
        "short-circuit trailing-zero scan. Continue approximation truth, independent real existence, "
        "caller and atomic publication proofs. Exact-operation resource failures, fixed-baseline "
        "and main/native/full-board requirements remain open.")
    summary = {
        "status": "Retain the isolated conditional digit-scan models; runtime remains v2335",
        "runner_sha256": sha(Path(__file__)), "previous_checkpoint": previous_path.name,
        "previous_checkpoint_sha256": sha(previous_path),
        "retained_runtime_checkpoint": previous["retained_runtime_checkpoint"],
        "source_record": audit["source_record"], "source_record_sha256": sha(record),
        "changed_sources": audit["changed_sources"], "runtime_source_files_changed": 0,
        "proposal_protocol": "digit-scan-baseline-v2363/protocol.json",
        "proposal_protocol_sha256": sha(E / "digit-scan-baseline-v2363/protocol.json"),
        "final_layout_protocol": "digit-scan-layout-v2366/protocol.json",
        "final_layout_protocol_sha256": sha(E / "digit-scan-layout-v2366/protocol.json"),
        "support_probes": audit["support_probes"], "launcher_failure_preserved": audit["launcher_failure_preserved"],
        "input_audit": audit_path.name, "input_audit_sha256": sha(audit_path),
        "conditional_proof": conditional, "fresh_validation_record": "digit-scan-validation-v2371.json",
        "fresh_validation_record_sha256": sha(E / "digit-scan-validation-v2371.json"),
        "historical_runtime_retention": previous["historical_runtime_retention"],
        "historical_runtime_retention_sha256": previous["historical_runtime_retention_sha256"],
        "historical_runtime_evidence_applicable": True, "historical_runs_relabelled_as_fresh": 0,
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "fresh_replay_cases": 0,
        "original_whole_file_and_timing_failures_preserved": True,
        "later_baseline_cpu_failures_preserved": audit["later_baseline_cpu_failures_preserved"],
        "coverage_limits": limits, "next_priority": next_priority, "main_runtime_admitted": False,
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
        "checkpoint_requirement": "Verify every v2358 artifact, current source, main guard and original worktree entries with record_checkpoint_v4.py before creating v2373.",
    }
    out.write_text(json.dumps(summary, indent=2) + "\n")
    metadata = {"previous_checkpoint": previous_path.name, "status": summary["status"],
        "retained_runtime_checkpoint": summary["retained_runtime_checkpoint"],
        "contract_decision": previous["contract_decision"], "accepted_cache_rule": previous["accepted_cache_rule"],
        "evidence_commits": previous["evidence_commits"] + [subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()],
        "retention_record": out.name, "retention_record_sha256": sha(out),
        "input_audit": audit_path.name, "input_audit_sha256": sha(audit_path), "conditional_proof": conditional,
        "fresh_validation_record": summary["fresh_validation_record"],
        "historical_runtime_retention": summary["historical_runtime_retention"],
        "historical_runtime_retention_sha256": summary["historical_runtime_retention_sha256"],
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "fresh_replay_cases": 0,
        "coverage_limits": limits, "main_runtime_admitted": False,
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
        "remaining_requirements": [next_priority, *previous["remaining_requirements"][1:]]}
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
    ledger = ROOT / "docs/properties-status.json"
    before_hash = sha(ledger)
    before = read(ledger)
    after = copy.deepcopy(before)
    entry = next(x for x in after["properties"] if x["id"] == "P4")
    for name in [out.name, "typed-api-prototype-v2373-state.json", audit_path.name]:
        path = str((E / name).relative_to(ROOT))
        assert path not in entry["evidence"]
        entry["evidence"].append(path)
    paragraph = (
        " The v2373 isolated proof checkpoint adds seventeen conditional digit-scan lemmas: "
        "716 units, zero errors, 98 production contracts and 160 model theorems. Nineteen fresh "
        "negative controls and one baseline pass; the 267 prior negatives/five baselines remain "
        "applicable under exact input and generated-entrypoint audits, giving 286 and six without "
        "relabelling old executions. Ten acceptance tests and workspace/111-input formatting pass. "
        "Six verification files change; ordinary runtime and historical runtime/cost/replay inputs "
        "remain unchanged. The model proves generic multiword factor and flag connections and the "
        "u32 leading-zero bound. Actual dependency storage and iterator behavior, machine count "
        "ranges, sign imports, producers, existence and atomic callers remain open. The type, "
        "closed-spec and launcher failures stay archived. Both baseline CPU failures, thirteen "
        "whole-crate groups and all property/target/native/full-board requirements remain unchanged.")
    entry["remaining"] += paragraph
    old_prefix = "Continue the isolated fallible scalar evaluator from typed-api-prototype-v2358-state.json"
    assert after["next_actions"][0].startswith(old_prefix)
    after["next_actions"][0] = after["next_actions"][0].replace(old_prefix,
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v2373-state.json", 1) + paragraph
    same = copy.deepcopy(after)
    current = next(x for x in same["properties"] if x["id"] == "P4")
    old = next(x for x in before["properties"] if x["id"] == "P4")
    current["evidence"], current["remaining"] = old["evidence"], old["remaining"]
    same["next_actions"][0] = before["next_actions"][0]
    assert same == before
    ledger.write_text(json.dumps(after, indent=2) + "\n")
    ledger_record.write_text(json.dumps({"before_sha256": before_hash, "after_sha256": sha(ledger),
        "changed_fields": ["P4.evidence", "P4.remaining", "next_actions[0]"],
        "all_39_property_and_14_target_requirements_and_statuses_preserved": True,
        "retention_record_sha256": sha(out), "metadata_sha256": sha(metadata_path),
        "retained_runtime_checkpoint": summary["retained_runtime_checkpoint"],
        "main_runtime_admitted": False}, indent=2) + "\n")
    print(json.dumps({"status": summary["status"], "verified_units": 716,
        "total_applicable_mutations": 286, "runtime_source_files_changed": 0, "checkpoint_pending": True}))


if __name__ == "__main__":
    main()
