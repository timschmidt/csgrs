"""Retain the conditional metadata bridge without relabelling runtime evidence."""
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
    output = E / "approximation-metadata-retention-v2345.json"
    metadata_path = E / "typed-api-prototype-v2345-metadata.json"
    ledger_record = E / "approximation-metadata-ledger-update-v2345.json"
    assert all(not p.exists() for p in [output, metadata_path, ledger_record])
    previous_path = E / "typed-api-prototype-v2335-state.json"
    assert sha(previous_path) == "7d840a2160d3e48baa67a3cc4ca2388f6270b141960bc0441f2df888fade7228"
    previous = read(previous_path)
    audit_path = E / "approximation-metadata-input-audit-v2344/summary.json"
    audit = read(audit_path)
    assert audit["historical_runtime_evidence_applicable"] and audit["runtime_source_files_changed"] == 0
    assert audit["retained_checkpoint_sha256"] == sha(previous_path)
    assert audit["fresh_rejected_mutations"] == 255 and audit["fresh_positive_baselines"] == 4
    assert len(audit["new_negative_controls"]) == audit["acceptance_tests"] == 10
    assert not audit["original_whole_file_gate_passed"] and not audit["fresh_timing_gate_passed"]
    assert len(audit["later_baseline_cpu_failures_preserved"]) == 2
    record = E / audit["source_record"]
    assert sha(record) == audit["source_record_sha256"]
    sources = json.loads(gzip.decompress(record.read_bytes()))
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    proof = audit["conditional_proof"]
    assert proof == {"verified_units": 662, "errors": 0, "production_contracts": 98,
                     "model_theorems": 137, "open_obligation_groups": 13}
    conditional = {"verified_units": 662, "errors": 0, "listed_production_contracts": 98,
        "listed_model_theorems": 137, "open_obligation_groups": 13,
        "acceptance_tests": 10, "rejected_mutants": 255, "passing_guard_baselines": 4,
        "guard_source_scope": "Fresh v2338/v2339 runs before two README count fixes and one import reorder. The v2344 audit reconstructs all 133 affected invocation inputs, verifies identical imported paths and mutation commutation, and preserves original execution identities."}
    next_priority = (
        "Connect the actual normalized BigInt/BigUint metadata and sign imports to the conditional "
        "bit-length/all-ones/power-of-two premises, preserving the producer's short-circuit evaluation. "
        "Continue valid approximation production, independent real existence, caller preconditions "
        "and atomic publication. Exact-operation CPU/memory failures, the fixed pre-Verus baseline, "
        "all thirteen whole-crate groups and main/native/full-board admission remain open.")
    inherited = read(E / "typed-api-prototype-v2335-metadata.json")
    limits = [
        "The two new model lemmas prove closed one-unit interval endpoints and a valid bound from explicit metadata premises and actual exponent/constructor contracts. They cover both signs and an unavailable exponent. They do not verify the BigInt metadata producer, actual caller, approximation validity, real existence or atomic publication.",
        "The final conditional target verifies 662 units, 98 production contracts and 137 model theorems. All 255 negative mutations, four passing baselines and ten acceptance tests passed before an import-order/count correction. The audit verifies identical function bodies, specifications, guard scripts, dependencies and tools, and reconstructs every affected invocation under the commuting import reorder. Thirteen whole-crate groups and fixed-baseline acceptance remain open.",
        "Only four verification files differ from v2335. The first included-file formatting check failed on one import order; that result remains archived. The correction also fixes two stale README counts. The corrected full proof and both formatting commands pass. No ordinary runtime, build, corpus or probe source changes.",
        "The v2335 runtime tests, strict Clippy/build checks, 39 capped phases, 4129 numerical cases and 648 public schedules remain historical evidence, applicable through the v2344 input audit. No new runtime, cost or replay processes are credited. The seven frozen probe executables and three linked libraries remain byte-identical to v2335."
    ] + inherited["coverage_limits"][2:]
    summary = {
        "status": "Retain the isolated conditional approximation-metadata bridge; runtime stays v2335",
        "runner_sha256": sha(Path(__file__)), "previous_checkpoint": previous_path.name,
        "previous_checkpoint_sha256": sha(previous_path), "retained_runtime_checkpoint": previous_path.name,
        "source_record": audit["source_record"], "source_record_sha256": sha(record),
        "changed_sources": audit["changed_sources"], "runtime_source_files_changed": 0,
        "proposal_protocol": "approximation-metadata-baseline-v2336/protocol.json",
        "proposal_protocol_sha256": sha(E / "approximation-metadata-baseline-v2336/protocol.json"),
        "documentation_correction": "approximation-metadata-count-correction-v2342/protocol.json",
        "documentation_correction_sha256": sha(E / "approximation-metadata-count-correction-v2342/protocol.json"),
        "input_audit": str(audit_path.relative_to(E)), "input_audit_sha256": sha(audit_path),
        "conditional_proof": conditional,
        "proof_phases": ["typed-api-prototype-v2337-approximation-metadata-verus",
                         "typed-api-prototype-v2343-approximation-metadata-final-verus"],
        "fresh_guard_and_format_validation": "approximation-metadata-validation-v2341.json",
        "fresh_guard_and_format_validation_sha256": sha(E / "approximation-metadata-validation-v2341.json"),
        "final_passing_format_phase": "typed-api-prototype-v2344-approximation-metadata-final-format",
        "final_passing_format_result_sha256": sha(E / "typed-api-prototype-v2344-approximation-metadata-final-format/result.json"),
        "original_import_order_format_failure_preserved": True,
        "historical_runtime_retention": "eager-bound-decoder-retention-v2335.json",
        "historical_runtime_retention_sha256": sha(E / "eager-bound-decoder-retention-v2335.json"),
        "historical_runtime_evidence_applicable": True, "historical_runs_relabelled_as_fresh": 0,
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "fresh_replay_cases": 0,
        "old_whole_file_and_timing_failures_preserved": True,
        "later_baseline_cpu_failures_preserved": audit["later_baseline_cpu_failures_preserved"],
        "coverage_limits": limits, "next_priority": next_priority,
        "main_runtime_admitted": False,
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
        "checkpoint_requirement": "record_checkpoint_v4.py must verify every previous checkpoint artifact, current source, main guard and original worktree entry before creating v2345."
    }
    output.write_text(json.dumps(summary, indent=2) + "\n")
    metadata = {
        "previous_checkpoint": previous_path.name, "status": summary["status"],
        "retained_runtime_checkpoint": previous_path.name,
        "contract_decision": inherited["contract_decision"], "accepted_cache_rule": inherited["accepted_cache_rule"],
        "evidence_commits": inherited["evidence_commits"] + [subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()],
        "retention_record": output.name, "retention_record_sha256": sha(output),
        "input_audit": str(audit_path.relative_to(E)), "input_audit_sha256": sha(audit_path),
        "conditional_proof": conditional, "historical_runtime_retention": summary["historical_runtime_retention"],
        "historical_runtime_retention_sha256": summary["historical_runtime_retention_sha256"],
        "fresh_guard_and_format_validation": summary["fresh_guard_and_format_validation"],
        "original_import_order_format_failure_preserved": True,
        "final_passing_format_phase": summary["final_passing_format_phase"],
        "final_passing_format_result_sha256": summary["final_passing_format_result_sha256"],
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "fresh_replay_cases": 0,
        "coverage_limits": limits, "main_runtime_admitted": False,
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
        "remaining_requirements": [next_priority, *inherited["remaining_requirements"][1:]]
    }
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
    ledger = ROOT / "docs/properties-status.json"
    before_hash = sha(ledger)
    before = read(ledger)
    after = copy.deepcopy(before)
    entry = next(x for x in after["properties"] if x["id"] == "P4")
    for name in [output.name, "typed-api-prototype-v2345-state.json", str(audit_path.relative_to(E))]:
        path = str((E / name).relative_to(ROOT))
        assert path not in entry["evidence"]
        entry["evidence"].append(path)
    paragraph = (
        " The v2345 isolated proof checkpoint adds two conditional numerical-metadata lemmas: "
        "662 verified units, zero errors, 98 production contracts and 137 model theorems. "
        "All 255 mutations, four positive baselines and ten acceptance tests pass. The first "
        "included-file format check fails on one import order and stays archived. The corrected "
        "full proof and both formatting checks pass. Two README counts are also corrected; "
        "the input audit reconstructs every affected guard under the identical-name import "
        "reorder, preserving the original executions and their applicability. Four verification files "
        "change and no runtime source changes. Historical v2335 tests, caps, costs and replays "
        "remain applicable under unchanged source/build/probe inputs; none are relabelled as "
        "fresh executions. The seven frozen probes and three libraries still match v2335. "
        "Actual dependency metadata, approximation producers, independent existence and atomic "
        "callers remain unproved. Both baseline CPU failures, thirteen whole-crate groups and "
        "every property/target/native/full-board requirement remain open.")
    entry["remaining"] += paragraph
    old_prefix = "Continue the isolated fallible scalar evaluator from typed-api-prototype-v2335-state.json"
    assert after["next_actions"][0].startswith(old_prefix)
    after["next_actions"][0] = after["next_actions"][0].replace(old_prefix,
        "Continue the isolated fallible scalar evaluator from typed-api-prototype-v2345-state.json", 1) + paragraph
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
        "retention_record_sha256": sha(output), "metadata_sha256": sha(metadata_path),
        "retained_runtime_checkpoint": previous_path.name, "main_runtime_admitted": False}, indent=2) + "\n")
    print(json.dumps({"status": summary["status"], "proof_units": 662,
        "runtime_source_files_changed": 0, "checkpoint_pending": True}))


if __name__ == "__main__":
    main()
