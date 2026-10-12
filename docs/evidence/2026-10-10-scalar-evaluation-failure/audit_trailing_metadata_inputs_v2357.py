"""Bind the new model guards and inherited proof/runtime evidence to current inputs."""
import ast
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    out = E / "trailing-metadata-input-audit-v2357.json"
    assert not out.exists()
    previous_path = E / "typed-api-prototype-v2345-state.json"
    assert sha(previous_path) == "a75220d56a289d2e83e36a426fb6f1f04b808e684b0889e5d360924408d940b9"
    previous = read(previous_path)
    old = snapshot(E / previous["source_record"])
    phase = E / "typed-api-prototype-v2352-trailing-metadata-verus"
    record = phase / "source-hashes.json.gz"
    current = snapshot(record)
    assert sha(record) == "57fe3139119d43abbdb76bcbee47c1f78245107a397d9c3672132d9d0eadd979"
    assert len(old) == 24746 and len(current) == 24747 and not (old.keys() - current.keys())
    assert all(sha(SOURCE / n) == h for n, h in current.items())
    prefix = "crates/hyperreal/verification/"
    changed = {n: {"before": old.get(n), "after": h} for n, h in current.items() if old.get(n) != h}
    assert set(changed) == {prefix + n for n in ["bit_length_model.rs", "test_bit_metadata_rejections.py",
                          "test_rejections.py", "scope.json", "README.md"]}
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    proof = read(phase / "proof-reports/evidence.json")
    assert read(phase / "result.json")["exit_code"] == 0
    assert proof["verification_results"]["verified"] == 682 and proof["verification_results"]["errors"] == 0
    assert len(proof["scope"]["verified_model_theorems"]) == 143
    old_proof = E / "typed-api-prototype-v2343-approximation-metadata-final-verus/proof-reports"
    old_scope = read(old_proof / "evidence.json")["scope"]
    assert proof["scope"]["verified_executable_contracts"] == old_scope["verified_executable_contracts"]
    assert len(proof["scope"]["verified_executable_contracts"]) == 98
    assert proof["scope"]["obligations"] == old_scope["obligations"] and len(old_scope["obligations"]) == 13
    assert read(phase / "proof-reports/dependencies.json") == read(old_proof / "dependencies.json")
    dependencies = read(phase / "proof-reports/dependencies.json")
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    tooling = read(phase / "proof-reports/tooling.json")
    assert tooling["tool_unchanged"] and tooling["tool_sha256"] == read(old_proof / "tooling.json")["tool_sha256"]

    validation_path = E / "trailing-metadata-validation-v2356.json"
    validation = read(validation_path)
    assert validation["complete"] and len(validation["phases"]) == 4
    for name, row in validation["phases"].items():
        assert row["result"] == read(E / name / "result.json")
        assert row["result"]["exit_code"] == 0 and row["result"]["source_unchanged"] and row["result"]["main_unchanged"]
        assert sha(E / name / "source-hashes.json.gz") == sha(record)
    assert read(E / "trailing-metadata-validation-protocol-v2356.json")["included_format_files"] == 109
    acceptance_text = (E / "typed-api-prototype-v2354-trailing-metadata-acceptance/stderr.log").read_text()
    assert "Ran 10 tests" in acceptance_text and "\nOK\n" in acceptance_text
    fresh_guard = E / "typed-api-prototype-v2353-trailing-metadata-rejections/proof-checks"
    fresh_tool = read(fresh_guard / "tooling.json")
    assert fresh_tool["exit_code"] == 0 and fresh_tool["tool_unchanged"]
    assert fresh_tool["tool_sha256"] == tooling["tool_sha256"]
    calls = [json.loads(line) for line in gzip.open(fresh_guard / "verus-invocations.jsonl.gz", "rt")]
    assert len(calls) == 13 and calls[0]["exit_code"] == 0 and all(c["exit_code"] != 0 for c in calls[1:])
    script = SOURCE / prefix / "test_bit_metadata_rejections.py"
    tree = ast.parse(script.read_text())
    mutations = ast.literal_eval(next(n.value for n in tree.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "MUTATIONS" for t in n.targets)))
    assert len(mutations) == 12
    model = (SOURCE / prefix / "bit_length_model.rs").read_text()
    assert calls[0]["rust_inputs_sha256"]["bit_length_model.rs"] == hashlib.sha256(model.encode()).hexdigest()
    mutation_rows = []
    for index, (name, before, after) in enumerate(mutations, 1):
        start = model.index("proof fn " + name + "(")
        end = model.index("\n}\n", start) + 3
        assert model[start:end].count(before) == 1
        mutated = model[:start] + model[start:end].replace(before, after) + model[end:]
        digest = hashlib.sha256(mutated.encode()).hexdigest()
        assert calls[index]["rust_inputs_sha256"]["bit_length_model.rs"] == digest
        mutation_rows.append({"name": name, "invocation_index": index, "mutated_source_sha256": digest})

    inherited_path = E / "approximation-metadata-input-audit-v2344/summary.json"
    assert sha(inherited_path) == previous["artifacts_sha256"][str(inherited_path.relative_to(E))]
    inherited = read(inherited_path)
    old_guards = E / "typed-api-prototype-v2339-approximation-metadata-rejections/proof-checks"
    old_calls = [json.loads(line) for line in gzip.open(old_guards / "verus-invocations.jsonl.gz", "rt")]
    assert sha(old_guards / "verus-invocations.jsonl.gz") == inherited["guard_log_sha256"]
    assert len(old_calls) == 259 and sum(c["exit_code"] == 0 for c in old_calls) == 4
    assert read(old_guards / "tooling.json")["tool_sha256"] == tooling["tool_sha256"]
    changed_rust = {Path(n).name for n in changed if n.endswith(".rs")}
    assert changed_rust == {"bit_length_model.rs"}
    assert all(not changed_rust.intersection(Path(n).name for n in c["rust_inputs_sha256"]) for c in old_calls)
    retained_entrypoint = E / "trailing-metadata-baseline-v2351/retained" / prefix / "test_rejections.py"
    entrypoint = (SOURCE / prefix / "test_rejections.py").read_text()
    additions = ["from test_bit_metadata_rejections import main as reject_bit_metadata_mutations  # noqa: E402\n",
                 "    reject_bit_metadata_mutations()\n"]
    restored = entrypoint
    for addition in additions:
        assert restored.count(addition) == 1
        restored = restored.replace(addition, "")
    assert restored == retained_entrypoint.read_text()
    assert hashlib.sha256(restored.encode()).hexdigest() == old[prefix + "test_rejections.py"]
    assert all(old.get(n) == h for n, h in current.items() if n.endswith(".py") and n not in changed)
    assert inherited["fresh_rejected_mutations"] == 255 and inherited["fresh_positive_baselines"] == 4
    assert inherited["historical_runtime_evidence_applicable"]
    for key in ["runtime_libraries_sha256", "frozen_probe_binaries_sha256"]:
        assert all(sha(Path(n)) == h for n, h in inherited[key].items())
    for key in ["frozen_probe_sources_sha256", "retained_runtime_records_sha256"]:
        assert all(sha(E / n) == h for n, h in inherited[key].items())
    support_commit = read(Path("/tmp/csgrs-trailing-bit-v2349-stage-preparation.json"))
    assert all(sha(ROOT / n) == h for n, h in support_commit["paths_sha256"].items())
    assert all(sha(SOURCE / n) == h for n, h in current.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    out.write_text(json.dumps({"status": "Fresh metadata guards pass; disjoint old guards and unchanged runtime evidence remain applicable",
        "runner_sha256": sha(Path(__file__)), "previous_checkpoint": previous_path.name,
        "previous_checkpoint_sha256": sha(previous_path), "source_record": str(record.relative_to(E)),
        "source_record_sha256": sha(record), "source_files_verified": len(current),
        "main_files_verified": len(guard), "changed_sources": changed, "runtime_source_files_changed": 0,
        "conditional_proof": {"verified_units": 682, "errors": 0, "production_contracts": 98,
                              "model_theorems": 143, "open_obligation_groups": 13},
        "fresh_validation_record_sha256": sha(validation_path), "fresh_rejected_mutations": 12,
        "fresh_positive_baselines": 1, "fresh_guard_log_sha256": sha(fresh_guard / "verus-invocations.jsonl.gz"),
        "fresh_mutations": mutation_rows, "fresh_acceptance_tests": 10, "included_format_files": 109,
        "inherited_input_audit": str(inherited_path.relative_to(E)), "inherited_input_audit_sha256": sha(inherited_path),
        "reused_rejected_mutations": 255, "reused_positive_baselines": 4,
        "total_applicable_rejected_mutations": 267, "total_applicable_positive_baselines": 5,
        "old_guard_reuse_basis": "None of the 259 prior actual invocation input sets imports bit_length_model.rs, the only changed Rust file. Every old guard module and all its Rust inputs remain as covered by v2344, including that audit's explicit import-order correction. Removing exactly the new entrypoint import and call restores its prior bytes. The new module defines its own independent guard group; its two top-level interpreter settings repeat the existing guard setup, and it invokes no proof at import time. Existing calls retain their original identities and rejection criteria.",
        "old_entrypoint_sha256": old[prefix + "test_rejections.py"],
        "current_entrypoint_sha256": current[prefix + "test_rejections.py"],
        "unchanged_proof_dependencies_sha256": sha(phase / "proof-reports/dependencies.json"),
        "historical_runtime_evidence_applicable": True, "unchanged_frozen_probe_binaries": 7,
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "fresh_replay_cases": 0,
        "prior_support_commit_files_verified": len(support_commit["paths_sha256"]),
        "original_whole_file_and_timing_failures_preserved": True,
        "later_baseline_cpu_failures_preserved": inherited["later_baseline_cpu_failures_preserved"],
        "scope": "The six lemmas and twelve controls extend the conditional bit-count model only. BigUint normalization/storage, actual multiword scans, metadata ranges and imports, approximation producers, independent real existence, callers and concurrent publication remain open. No new runtime work or historical process relabelling occurs; every previous scope/resource/main/native/full-board limitation remains. The next checkpoint must still verify all previous artifacts."}, indent=2) + "\n")
    print(json.dumps({"verified_units": 682, "fresh_negative_controls": 12,
        "reused_negative_controls": 255, "total_negative_controls": 267,
        "runtime_source_files_changed": 0, "source_files_verified": len(current)}))


if __name__ == "__main__":
    main()
