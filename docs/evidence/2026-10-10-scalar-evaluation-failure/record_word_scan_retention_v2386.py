"""Retain the typed-word proof checkpoint with explicit conditional boundaries."""
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
    out = E / "word-scan-retention-v2386.json"
    metadata_path = E / "typed-api-prototype-v2386-metadata.json"
    ledger_record = E / "word-scan-ledger-update-v2386.json"
    assert all(not p.exists() for p in [out, metadata_path, ledger_record])
    previous_path = E / "typed-api-prototype-v2373-state.json"
    assert sha(previous_path) == "163bf723328ab72df0a56025f9a55960eda13eb4c6767669965b61e66333508a"
    previous = read(previous_path)
    audit_path = E / "word-scan-input-audit-v2385.json"
    audit = read(audit_path)
    assert audit["previous_checkpoint_sha256"] == sha(previous_path)
    assert audit["runtime_source_files_changed"] == 0 and audit["historical_runtime_evidence_applicable"]
    assert audit["fresh_rejected_mutations"] == 20 and audit["reused_rejected_mutations"] == 286
    assert audit["total_applicable_rejected_mutations"] == 306 and audit["total_applicable_positive_baselines"] == 7
    assert audit["fresh_acceptance_tests"] == 10 and audit["included_format_files"] == 112
    assert len(audit["later_baseline_cpu_failures_preserved"]) == 2
    record = E / audit["source_record"]
    assert sha(record) == audit["source_record_sha256"]
    sources = json.loads(gzip.decompress(record.read_bytes()))
    assert len(sources) == 24751 and all(sha(SOURCE / n) == h for n, h in sources.items())
    assert audit["conditional_proof"] == {"verified_units": 736, "errors": 0,
        "production_contracts": 98, "model_theorems": 178, "open_obligation_groups": 13}
    conditional = {"verified_units": 736, "errors": 0, "listed_production_contracts": 98,
        "listed_model_theorems": 178, "open_obligation_groups": 13, "fresh_acceptance_tests": 10,
        "fresh_rejected_mutants": 20, "fresh_passing_guard_baselines": 1,
        "reused_rejected_mutants": 286, "reused_passing_guard_baselines": 6,
        "total_applicable_rejected_mutants": 306, "total_applicable_passing_guard_baselines": 7}
    limits = [
        "Eighteen conditional typed-word lemmas connect u32/u64 sequence views to retained generic digit denotation and native word-factor helpers. They prove normalized bit lengths, both scan outcomes and zero-suffix truncation. The u64 denotation agrees with the actual verified limb prefix. First-match, normalization and aggregate count-range premises remain explicit; actual dependency storage imports, iterator execution and machine range guarantees are still open.",
        "The full conditional target passes 736 units, zero errors, 98 production contracts and 178 model theorems. Twenty new negatives and one baseline pass, with ten acceptance tests and workspace/112-input formatting. The 286 old negatives/six baselines retain original execution identities under v2385: all 292 temporary entrypoints reconstruct exactly from unchanged generators and original paths. This inherits v2372/v2357/v2344, including the explicit import-order correction; old runs are not relabelled as fresh.",
        "Six verification files change, including a new model and guard module. All ordinary runtime, build, test corpus and probe sources remain unchanged. Historical v2335 runtime, cap, cost and replay evidence remains applicable with every original limit and failure. No fresh runtime, cost or replay process is credited.",
        "The typed-word support probe passes 438 units including existing imports, not 438 new theorems. The initial cast syntax, public/private visibility and root open-spec visibility failures remain archived; the passing probe uses the intended module scope. A nonunique mutation anchor was rejected before source application and corrected in a separate frozen preparation. No mathematical theorem, gate or historical outcome was weakened."
    ] + previous["coverage_limits"][4:]
    next_priority = (
        "Prove actual dependency normalization and first-match iterator execution against the typed "
        "word views, then connect BigUint storage and BigInt sign/magnitude to the bound producer. "
        "Discharge aggregate machine count-range premises, preserving early Unknown and the lazy "
        "trailing-zero scan. Continue approximation truth, independent real existence, caller and "
        "atomic publication proofs. Exact-operation resource failures, fixed-baseline and "
        "main/native/full-board requirements remain open.")
    summary = {
        "status": "Retain the isolated conditional typed-word scan models; runtime remains v2335",
        "runner_sha256": sha(Path(__file__)), "previous_checkpoint": previous_path.name,
        "previous_checkpoint_sha256": sha(previous_path),
        "retained_runtime_checkpoint": previous["retained_runtime_checkpoint"],
        "source_record": audit["source_record"], "source_record_sha256": sha(record),
        "changed_sources": audit["changed_sources"], "runtime_source_files_changed": 0,
        "proposal_protocol": "word-scan-baseline-v2379/protocol.json",
        "proposal_protocol_sha256": sha(E / "word-scan-baseline-v2379/protocol.json"),
        "support_probes": audit["support_probes"], "preparation_failure": audit["preparation_failure"],
        "preparation_failure_sha256": audit["preparation_failure_sha256"],
        "input_audit": audit_path.name, "input_audit_sha256": sha(audit_path),
        "conditional_proof": conditional, "fresh_validation_record": "word-scan-validation-v2384.json",
        "fresh_validation_record_sha256": sha(E / "word-scan-validation-v2384.json"),
        "historical_runtime_retention": previous["historical_runtime_retention"],
        "historical_runtime_retention_sha256": previous["historical_runtime_retention_sha256"],
        "historical_runtime_evidence_applicable": True, "historical_runs_relabelled_as_fresh": 0,
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "fresh_replay_cases": 0,
        "original_whole_file_and_timing_failures_preserved": True,
        "later_baseline_cpu_failures_preserved": audit["later_baseline_cpu_failures_preserved"],
        "coverage_limits": limits, "next_priority": next_priority, "main_runtime_admitted": False,
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
        "checkpoint_requirement": "Verify every v2373 artifact, current source, main guard and original worktree entries with record_checkpoint_v4.py before creating v2386.",
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
    for name in [out.name, "typed-api-prototype-v2386-state.json", audit_path.name]:
        path = str((E / name).relative_to(ROOT))
        assert path not in entry["evidence"]
        entry["evidence"].append(path)
    paragraph = (
        " The v2386 isolated proof checkpoint adds eighteen conditional typed-word scan lemmas: "
        "736 units, zero errors, 98 production contracts and 178 model theorems. Twenty fresh "
        "negative controls and one baseline pass; 286 prior negatives/six baselines remain applicable "
        "under exact input and generated-entrypoint audits, giving 306 and seven without relabelling "
        "old executions. Ten acceptance tests and workspace/112-input formatting pass. Six verification "
        "files change; ordinary runtime and historical runtime/cost/replay inputs remain unchanged. "
        "Both word widths connect to the generic scans, and the u64 denotation matches the actual "
        "verified limb prefix. Zero-suffix truncation preserves value. Actual dependency storage and "
        "iterator execution, count-range premises, sign imports, producers, existence and atomic "
        "callers remain open. All failed support/preparation attempts, both baseline CPU failures, "
        "thirteen whole-crate groups and every property/target/native/full-board requirement remain.")
    entry["remaining"] += paragraph
    old_prefix = "Continue the isolated fallible scalar evaluator from typed-api-prototype-v2373-state.json"
    assert after["next_actions"][0].startswith(old_prefix)
    after["next_actions"][0] = after["next_actions"][0].replace(old_prefix,
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v2386-state.json", 1) + paragraph
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
    print(json.dumps({"status": summary["status"], "verified_units": 736,
        "total_applicable_mutations": 306, "runtime_source_files_changed": 0, "checkpoint_pending": True}))


if __name__ == "__main__":
    main()
