"""Retain verification-only Cargo binding and complete scalar source catalogs."""
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
    out = E / "dependency-binding-retention-v2440.json"
    metadata_path = E / "typed-api-prototype-v2440-metadata.json"
    ledger_record = E / "dependency-binding-ledger-update-v2440.json"
    assert all(not p.exists() for p in [out, metadata_path, ledger_record])
    previous_path = E / "typed-api-prototype-v2386-state.json"
    assert sha(previous_path) == "6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9"
    previous = read(previous_path)
    audit_path = E / "dependency-binding-input-audit-v2439.json"
    audit = read(audit_path)
    assert audit["previous_checkpoint_sha256"] == sha(previous_path)
    assert audit["runtime_source_files_changed"] == 0 and audit["historical_runtime_evidence_applicable"]
    assert audit["fresh_rejected_mutations"] == 0 and audit["reused_rejected_mutations"] == 306
    assert audit["total_applicable_positive_baselines"] == 7 and audit["distinct_candidate_tests"] == 26
    assert audit["fresh_dependency_identity_tests"] == 19 and audit["fresh_acceptance_entrypoint_tests"] == 26
    assert audit["fresh_source_catalog_tests"] == 13 and audit["included_format_files"] == 112
    assert len(audit["later_baseline_cpu_failures_preserved"]) == 2
    record = E / audit["source_record"]
    assert sha(record) == audit["source_record_sha256"]
    sources = json.loads(gzip.decompress(record.read_bytes()))
    assert len(sources) == 24751 and all(sha(SOURCE / n) == h for n, h in sources.items())
    assert audit["conditional_proof"] == {"verified_units": 736, "errors": 0,
        "production_contracts": 98, "model_theorems": 178, "open_obligation_groups": 13}
    conditional = {"verified_units": 736, "errors": 0, "listed_production_contracts": 98,
        "listed_model_theorems": 178, "open_obligation_groups": 13,
        "fresh_dependency_identity_tests": 19, "fresh_acceptance_entrypoint_tests": 26,
        "distinct_candidate_tests": 26, "fresh_rejected_mutants": 0, "fresh_passing_guard_baselines": 0,
        "reused_rejected_mutants": 306, "reused_passing_guard_baselines": 7,
        "total_applicable_rejected_mutants": 306, "total_applicable_passing_guard_baselines": 7}
    limits = [
        "Three verification infrastructure/documentation files change; all Rust bodies, contracts, models, runtime/build inputs and mutation generators remain unchanged. Proof dependencies must belong to the actual production root closure and match canonical manifest paths and registry identity. Local and registry sources include all file types; binaries, lock/manifests and Cargo configurations are checked around fresh resolution. The dependency proof boundary remains transparent Sign only, with no assumption about arithmetic implementations.",
        "The fresh full conditional target passes 736 units, zero errors, 98 production contracts and 178 model theorems. Nineteen standalone dependency tests and the 26-test acceptance entry point pass; the latter includes the same nineteen tests, so there are 26 distinct candidate tests. Workspace and 112-input formatting pass. All 306 historical negative controls and seven baselines retain their original identities: unchanged Rust/generators, exact reconstruction of 313 temporary entrypoints, and unchanged dependency source/binary maps and Verus arguments. No guard is relabelled as newly executed.",
        "Support qualification includes nineteen preview dependency tests, two actual Cargo fixtures with nine Cargo processes, eight actual-workspace dependency bindings and thirteen source-catalog tests. A path mismatch is rejected before a build even when dependency bytes agree. The new candidate wrapper discovers introduced scalar files of every type, preserves original workspace inputs and catches additions/deletions/symlinks after a command. It does not claim discovery of new files outside the scalar subtree. The old count reader failed after successful tests; its frozen failure and corrected 26-test interpretation are retained.",
        "The v2397/v2423 support previews, including final shared BigUint source, verified conditional metadata bodies for both limb widths, mutation controls and native differential tests, remain separate from SOURCE and retained proof counts. No vendor runtime is applied. Actual imported BigUint/BigInt identity and caller refinement, normalization and machine count-range premises, approximation truth, independent existence and concurrent publication remain open. Historical v2335 runtime/cap/cost/replay evidence remains applicable under exact unchanged source and binary audits; no fresh runtime, timing or replay process is credited."
    ] + previous["coverage_limits"][4:]
    next_priority = (
        "Connect the qualified shared BigUint metadata bodies to the actual Cargo dependency and "
        "Verus imports, then prove BigInt sign/magnitude and bound-producing callers. Preserve "
        "normalization/allocation policy, early Unknown and lazy trailing-zero scans. Discharge "
        "explicit count-range premises and qualify any runtime change against original correctness, "
        "resource, cost, memory and size gates before admission. Continue approximation truth, "
        "independent existence and atomic publication; both baseline CPU failures and all "
        "main/native/full-board requirements remain open.")
    frontiers = {n: sha(E / n) for n in ["storage-contract-frontier-v2397.json",
                                        "biguint-integration-frontier-v2423.json"]}
    summary = {
        "status": "Retain isolated dependency/source binding infrastructure; runtime remains v2335",
        "runner_sha256": sha(Path(__file__)), "previous_checkpoint": previous_path.name,
        "previous_checkpoint_sha256": sha(previous_path),
        "retained_runtime_checkpoint": previous["retained_runtime_checkpoint"],
        "source_record": audit["source_record"], "source_record_sha256": sha(record),
        "changed_sources": audit["changed_sources"], "runtime_source_files_changed": 0,
        "proposal_protocol": "dependency-binding-baseline-v2432/protocol.json",
        "proposal_protocol_sha256": sha(E / "dependency-binding-baseline-v2432/protocol.json"),
        "input_audit": audit_path.name, "input_audit_sha256": sha(audit_path),
        "conditional_proof": conditional, "fresh_validation_record": audit["fresh_validation_record"],
        "fresh_validation_record_sha256": audit["fresh_validation_record_sha256"],
        "support_qualification": audit["support_qualification"], "prior_support_frontiers_sha256": frontiers,
        "candidate_wrapper": "validate_candidate_v3.py", "candidate_wrapper_sha256": sha(E / "validate_candidate_v3.py"),
        "failed_reader_preserved": audit["failed_reader_preserved"],
        "failed_reader_sha256": audit["failed_reader_sha256"],
        "historical_runtime_retention": previous["historical_runtime_retention"],
        "historical_runtime_retention_sha256": previous["historical_runtime_retention_sha256"],
        "historical_runtime_evidence_applicable": True, "historical_runs_relabelled_as_fresh": 0,
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "fresh_replay_cases": 0,
        "original_whole_file_and_timing_failures_preserved": True,
        "later_baseline_cpu_failures_preserved": audit["later_baseline_cpu_failures_preserved"],
        "coverage_limits": limits, "next_priority": next_priority, "main_runtime_admitted": False,
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
        "checkpoint_requirement": "Verify every v2386 artifact, all subsequent support artifacts, current source, main guard and original worktree entries with record_checkpoint_v4.py before creating v2440."
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
    for name in [out.name, "typed-api-prototype-v2440-state.json", audit_path.name]:
        path = str((E / name).relative_to(ROOT))
        assert path not in entry["evidence"]
        entry["evidence"].append(path)
    paragraph = (
        " The v2440 checkpoint strengthens verification dependency identity and source catalogs. "
        "Proof dependencies must match the actual production root closure and canonical paths; "
        "all source types, binaries and Cargo configuration are checked around resolution. "
        "The fresh full proof remains 736 units, zero errors, 98 contracts and 178 model theorems. "
        "Nineteen identity tests and the 26-test acceptance entry point pass with overlap counted "
        "explicitly; workspace/112-input formatting passes. Thirteen catalog tests and actual "
        "Cargo matching/mismatched-path fixtures qualify the infrastructure. All 306 earlier "
        "negative controls and seven baselines remain applicable under exact unchanged-input "
        "and 313-entrypoint reconstruction audits. Three verification files change; no Rust or "
        "runtime input changes. BigUint previews remain separate support evidence. The failed "
        "test-count reader, original resource/cost failures, thirteen whole-crate groups and "
        "every property/target/native/full-board requirement remain open or preserved.")
    entry["remaining"] += paragraph
    old_prefix = "Continue the isolated fallible scalar evaluator from typed-api-prototype-v2386-state.json"
    assert after["next_actions"][0].startswith(old_prefix)
    after["next_actions"][0] = after["next_actions"][0].replace(old_prefix,
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v2440-state.json", 1) + paragraph
    same = copy.deepcopy(after)
    current = next(x for x in same["properties"] if x["id"] == "P4")
    old_entry = next(x for x in before["properties"] if x["id"] == "P4")
    current["evidence"], current["remaining"] = old_entry["evidence"], old_entry["remaining"]
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
        "applicable_historical_mutations": 306, "runtime_source_files_changed": 0,
        "checkpoint_pending": True}))


if __name__ == "__main__":
    main()
