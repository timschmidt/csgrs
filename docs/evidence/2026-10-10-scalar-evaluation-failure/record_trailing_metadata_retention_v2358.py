"""Retain the conditional trailing-bit models with explicit inherited evidence."""
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
    out = E / "trailing-metadata-retention-v2358.json"
    metadata_path = E / "typed-api-prototype-v2358-metadata.json"
    ledger_record = E / "trailing-metadata-ledger-update-v2358.json"
    assert all(not p.exists() for p in [out, metadata_path, ledger_record])
    previous_path = E / "typed-api-prototype-v2345-state.json"
    previous = read(previous_path)
    assert sha(previous_path) == "a75220d56a289d2e83e36a426fb6f1f04b808e684b0889e5d360924408d940b9"
    audit_path = E / "trailing-metadata-input-audit-v2357.json"
    audit = read(audit_path)
    assert audit["previous_checkpoint_sha256"] == sha(previous_path)
    assert audit["runtime_source_files_changed"] == 0 and audit["historical_runtime_evidence_applicable"]
    assert audit["fresh_rejected_mutations"] == 12 and audit["reused_rejected_mutations"] == 255
    assert audit["total_applicable_rejected_mutations"] == 267 and audit["total_applicable_positive_baselines"] == 5
    assert audit["fresh_acceptance_tests"] == 10 and audit["included_format_files"] == 109
    assert len(audit["later_baseline_cpu_failures_preserved"]) == 2
    record = E / audit["source_record"]
    assert sha(record) == audit["source_record_sha256"]
    sources = json.loads(gzip.decompress(record.read_bytes()))
    assert len(sources) == 24747 and all(sha(SOURCE / n) == h for n, h in sources.items())
    assert audit["conditional_proof"] == {"verified_units": 682, "errors": 0,
        "production_contracts": 98, "model_theorems": 143, "open_obligation_groups": 13}
    conditional = {"verified_units": 682, "errors": 0, "listed_production_contracts": 98,
        "listed_model_theorems": 143, "open_obligation_groups": 13, "fresh_acceptance_tests": 10,
        "fresh_rejected_mutants": 12, "fresh_passing_guard_baselines": 1,
        "reused_rejected_mutants": 255, "reused_passing_guard_baselines": 4,
        "total_applicable_rejected_mutants": 267, "total_applicable_passing_guard_baselines": 5}
    limits = [
        "Six new conditional model lemmas prove 32/64-bit word-plus-one factorizations and the power-of-two/all-ones decisions from bit length and positive odd factors. The u64 zero-factor proof is reused from the actual verified helper. BigUint storage/import, normalized multiword scans, length ranges, BigInt sign/magnitude imports and the actual bound producer remain open.",
        "The full conditional target passes 682 units, 98 production contracts and 143 model theorems. Twelve new negatives and one baseline pass, with ten fresh acceptance checks and workspace/109-input formatting. The 255 old negatives/four baselines remain applicable under the v2357 disjoint-input audit, which inherits the explicit v2344 import-order correction. They are not relabelled as fresh current-source executions.",
        "Five verification files change, including one new guard module. All ordinary runtime, build, test corpus and probe sources are unchanged. Historical v2335 runtime, cap, cost and replay evidence retains its original identities and limits; no new runtime/cost/replay process is credited.",
        "The standalone support probes and their name-resolution failure remain separate from retained coverage. The reader's function-versus-unit count failure and the first nonunique mutation-anchor preparation also remain archived. Their corrections do not change theorem statements, gate criteria, historical outcomes or runtime source."
    ] + previous["coverage_limits"][4:]
    next_priority = (
        "Connect the proved word factors and flag decisions to normalized little-endian digit scans "
        "for both dependency widths, including empty/all-maximal storage and first nonzero/nonmaximal "
        "digits. Discharge actual storage, iterator, sign/magnitude import and u64 metadata range "
        "obligations while preserving the producer's short-circuit evaluation. Continue approximation "
        "truth, independent existence, caller and atomic publication proofs; all exact-operation "
        "resource failures, fixed-baseline and main/native/full-board requirements remain open.")
    summary = {"status": "Retain the isolated conditional trailing-bit metadata models; runtime remains v2335",
        "runner_sha256": sha(Path(__file__)), "previous_checkpoint": previous_path.name,
        "previous_checkpoint_sha256": sha(previous_path),
        "retained_runtime_checkpoint": previous["retained_runtime_checkpoint"],
        "source_record": audit["source_record"], "source_record_sha256": sha(record),
        "changed_sources": audit["changed_sources"], "runtime_source_files_changed": 0,
        "proposal_protocol": "trailing-metadata-baseline-v2351/protocol.json",
        "proposal_protocol_sha256": sha(E / "trailing-metadata-baseline-v2351/protocol.json"),
        "preparation_failure": "trailing-metadata-preparation-failure-v2350.json",
        "input_audit": audit_path.name, "input_audit_sha256": sha(audit_path),
        "conditional_proof": conditional, "fresh_validation_record": "trailing-metadata-validation-v2356.json",
        "fresh_validation_record_sha256": sha(E / "trailing-metadata-validation-v2356.json"),
        "historical_runtime_retention": previous["historical_runtime_retention"],
        "historical_runtime_retention_sha256": previous["historical_runtime_retention_sha256"],
        "historical_runtime_evidence_applicable": True, "historical_runs_relabelled_as_fresh": 0,
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "fresh_replay_cases": 0,
        "original_whole_file_and_timing_failures_preserved": True,
        "later_baseline_cpu_failures_preserved": audit["later_baseline_cpu_failures_preserved"],
        "coverage_limits": limits, "next_priority": next_priority, "main_runtime_admitted": False,
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
        "checkpoint_requirement": "Verify every v2345 artifact, the current source, main guard and original worktree entries with record_checkpoint_v4.py before creating v2358."}
    out.write_text(json.dumps(summary, indent=2) + "\n")
    metadata = {"previous_checkpoint": previous_path.name, "status": summary["status"],
        "retained_runtime_checkpoint": summary["retained_runtime_checkpoint"],
        "contract_decision": previous["contract_decision"], "accepted_cache_rule": previous["accepted_cache_rule"],
        "evidence_commits": previous["evidence_commits"] + ["b5fef35e7ae76323ff4f5bcffbb2d334305211fb",
            subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()],
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
    for name in [out.name, "typed-api-prototype-v2358-state.json", audit_path.name]:
        path = str((E / name).relative_to(ROOT))
        assert path not in entry["evidence"]
        entry["evidence"].append(path)
    paragraph = (
        " The v2358 isolated proof checkpoint adds six conditional trailing-bit metadata lemmas: "
        "682 units, zero errors, 98 production contracts and 143 model theorems. Twelve fresh "
        "negative controls and one positive baseline pass; 255 prior negatives and four baselines "
        "remain applicable under the disjoint-input audit, giving 267 and five without relabelling "
        "old executions. Ten acceptance tests and workspace/109-input formatting pass. Five "
        "verification files change; ordinary runtime source and all historical runtime/cost/replay "
        "inputs remain unchanged. The model connects word-plus-one factors and bit lengths to "
        "power-of-two/all-ones flags. Actual normalized multiword scans, storage and sign imports, "
        "metadata ranges, producers, existence and atomic callers remain open. The setup and reader "
        "failures stay archived. Both baseline CPU failures, thirteen whole-crate groups and all "
        "property/target/native/full-board requirements remain unchanged.")
    entry["remaining"] += paragraph
    old_prefix = "Continue the isolated fallible scalar evaluator from typed-api-prototype-v2345-state.json"
    assert after["next_actions"][0].startswith(old_prefix)
    after["next_actions"][0] = after["next_actions"][0].replace(old_prefix,
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v2358-state.json", 1) + paragraph
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
    print(json.dumps({"status": summary["status"], "verified_units": 682,
        "total_applicable_mutations": 267, "runtime_source_files_changed": 0, "checkpoint_pending": True}))


if __name__ == "__main__":
    main()
