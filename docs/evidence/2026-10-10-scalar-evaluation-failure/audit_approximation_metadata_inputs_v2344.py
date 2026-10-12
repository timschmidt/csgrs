"""Bind the metadata proofs to unchanged runtime and guard inputs."""
import ast
import gzip
import hashlib
import json
from pathlib import Path
import shlex

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
TARGET = ROOT / "target/scalar-evaluation-candidate/target"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    out = E / "approximation-metadata-input-audit-v2344"
    assert not out.exists()
    retained_path = E / "typed-api-prototype-v2335-state.json"
    assert sha(retained_path) == "7d840a2160d3e48baa67a3cc4ca2388f6270b141960bc0441f2df888fade7228"
    retained = read(retained_path)
    old_record = E / retained["source_record"]
    old = snapshot(old_record)
    phase = E / "typed-api-prototype-v2343-approximation-metadata-final-verus"
    record = phase / "source-hashes.json.gz"
    current = snapshot(record)
    assert len(current) == 24746 and current.keys() == old.keys()
    assert all(sha(SOURCE / n) == h for n, h in current.items())
    assert sha(record) == read(E / "approximation-metadata-count-correction-v2342/application.json")["source_record_sha256"]
    changed = {n: {"before": old[n], "after": h} for n, h in current.items() if old[n] != h}
    expected = {"crates/hyperreal/verification/" + n for n in
                ["bound_denotation.rs", "test_bound_rejections.py", "scope.json", "README.md"]}
    assert set(changed) == expected
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    result = read(phase / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    proof = read(phase / "proof-reports/evidence.json")
    assert proof["verification_results"]["verified"] == 662 and proof["verification_results"]["errors"] == 0
    scope = proof["scope"]
    assert len(scope["verified_executable_contracts"]) == 98 and len(scope["verified_model_theorems"]) == 137
    assert len(scope["obligations"]) == 13 and all(r["status"] != "complete" for r in scope["obligations"])
    prior_scope = read(E / "approximation-metadata-baseline-v2336/retained/crates/hyperreal/verification/scope.json")
    assert scope["verified_executable_contracts"] == prior_scope["verified_executable_contracts"]
    assert scope["obligations"] == prior_scope["obligations"]
    old_proof = E / "typed-api-prototype-v2260-eager-bound-decoder-verus/proof-reports"
    dependencies = read(phase / "proof-reports/dependencies.json")
    assert dependencies == read(old_proof / "dependencies.json")
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    tool = read(phase / "proof-reports/tooling.json")
    assert tool["tool_unchanged"] and tool["tool_sha256"] == read(old_proof / "tooling.json")["tool_sha256"]

    checks_path = E / "approximation-metadata-validation-v2341.json"
    checks = read(checks_path)
    assert not checks["complete"] and len(checks["phases"]) == 4
    for name, row in checks["phases"].items():
        assert row["result"]["exit_code"] == (1 if name.endswith("included-fmt") else 0)
        assert row["result"]["source_unchanged"] and row["result"]["main_unchanged"]
        assert read(E / name / "result.json") == row["result"]
        inputs = snapshot(E / name / "source-hashes.json.gz")
        assert inputs.keys() == current.keys()
        assert {n for n in current if current[n] != inputs[n]} == {
            "crates/hyperreal/verification/README.md", "crates/hyperreal/verification/bound_denotation.rs"}
    formatting = E / "typed-api-prototype-v2344-approximation-metadata-final-format"
    format_result = read(formatting / "result.json")
    assert format_result["exit_code"] == 0 and format_result["source_unchanged"] and format_result["main_unchanged"]
    assert sha(formatting / "source-hashes.json.gz") == sha(record)
    format_output = (formatting / "stdout.log").read_text()
    assert format_output.count("passed after correction") == 2
    guards = E / "typed-api-prototype-v2339-approximation-metadata-rejections/proof-checks"
    guard_tool = read(guards / "tooling.json")
    assert guard_tool["exit_code"] == 0 and guard_tool["tool_unchanged"]
    assert guard_tool["tool_sha256"] == tool["tool_sha256"]
    calls = [json.loads(line) for line in gzip.open(guards / "verus-invocations.jsonl.gz", "rt")]
    assert len(calls) == 259 and sum(c["exit_code"] == 0 for c in calls) == 4
    assert guard_tool["rejected_invocations"] == 255 and guard_tool["baseline_successes"] == 4
    assert all(not any(n.endswith(("README.md", "scope.json")) for n in c["rust_inputs_sha256"]) for c in calls)
    controls = None
    for node in ast.parse((E / "prepare_approximation_metadata_candidate.py").read_text()).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "MUTATIONS" for t in node.targets):
            controls = ast.literal_eval(node.value)
    assert controls is not None and len(controls) == 10
    model_name = "crates/hyperreal/verification/bound_denotation.rs"
    model = (SOURCE / model_name).read_text()
    old_model = (guards.parent / "inputs" / model_name).read_text()
    correction = read(E / "approximation-metadata-count-correction-v2342/protocol.json")
    old_import, new_import = correction["old_import"], correction["new_import"]
    assert old_model.count(old_import) == model.count(new_import) == 1
    assert old_model.replace(old_import, new_import) == model
    def imported_names(statement):
        prefix, group = statement.split("{", 1)
        assert prefix == "use vstd::arithmetic::power2::" and group.endswith("};")
        return sorted(n.strip() for n in group[:-2].split(","))
    assert imported_names(old_import) == imported_names(new_import)
    assert len(set(imported_names(old_import))) == 3
    model_mutations = []
    for node in ast.parse((SOURCE / "crates/hyperreal/verification/test_bound_rejections.py").read_text()).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "MODEL_MUTATIONS" for t in node.targets):
            model_mutations = ast.literal_eval(node.value)
        elif isinstance(node, ast.AugAssign) and isinstance(node.target, ast.Name) and node.target.id == "MODEL_MUTATIONS":
            model_mutations += ast.literal_eval(node.value)
    assert len(model_mutations) == 39
    variants = {hashlib.sha256(old_model.encode()).hexdigest(): ("unmutated model", old_model, model)}
    for label, before, after in model_mutations:
        assert old_model.count(before) == model.count(before) == 1
        previous_variant = old_model.replace(before, after)
        current_variant = model.replace(before, after)
        assert previous_variant.count(old_import) == current_variant.count(new_import) == 1
        assert previous_variant.replace(old_import, new_import) == current_variant
        digest = hashlib.sha256(previous_variant.encode()).hexdigest()
        assert digest not in variants
        variants[digest] = (label, previous_variant, current_variant)
    affected_calls = []
    for index, call in enumerate(calls):
        files = [(n, h) for n, h in call["rust_inputs_sha256"].items() if n.endswith("bound_denotation.rs")]
        if files:
            assert len(files) == 1 and files[0][1] in variants
            label, previous_variant, current_variant = variants[files[0][1]]
            affected_calls.append({"invocation_index": index, "file": files[0][0], "variant": label,
                "actual_executed_input_sha256": files[0][1],
                "current_equivalent_input_sha256": hashlib.sha256(current_variant.encode()).hexdigest()})
    assert len(affected_calls) == 133
    negative_controls = []
    for name, before, after in controls:
        assert old_model.count(before) == 1
        digest = hashlib.sha256(old_model.replace(before, after).encode()).hexdigest()
        matches = [i for i, c in enumerate(calls) if any(n.endswith("bound_denotation.rs") and h == digest
                   for n, h in c["rust_inputs_sha256"].items())]
        assert len(matches) == 1 and calls[matches[0]]["exit_code"] != 0
        negative_controls.append({"name": name, "invocation_index": matches[0], "mutated_source_sha256": digest})
    acceptance = E / "typed-api-prototype-v2338-approximation-metadata-acceptance"
    acceptance_output = (acceptance / "stderr.log").read_text()
    assert "Ran 10 tests" in acceptance_output and "\nOK\n" in acceptance_output

    binaries = read(E / "eager-bound-decoder-binary-comparison-v2262/summary.json")
    library_hashes = {n: row["candidate_sha256"] for n, row in binaries["libraries"].items()}
    assert all(sha(Path(n)) == h for n, h in library_hashes.items())
    binary_hashes = {row["binary"]: row["binary_sha256"] for row in binaries["results"]}
    assert len(binary_hashes) == 7 and all(sha(Path(n)) == h for n, h in binary_hashes.items())
    probe_hashes = {}
    for row in binaries["results"]:
        probe = next(Path(n) for n in row["command"] if str(n).endswith("probe.rs"))
        name = str(probe.relative_to(E))
        assert sha(probe) == retained["artifacts_sha256"][name]
        probe_hashes[name] = sha(probe)
    depfiles = [TARGET / n for n in ["release/deps/hyperreal-74ad6915c70c2b5a.d",
                "release/deps/hyperreal-c4ce5bbc6487bbcc.d", "debug/deps/hyperreal-563f75cd5535d588.d"]]
    dependency_rows = []
    for path in depfiles:
        text = path.read_text()
        inputs = shlex.split(text.splitlines()[0].split(": ", 1)[1])
        assert inputs and not set(inputs).intersection(changed)
        assert all(n in current and old[n] == current[n] for n in inputs)
        assert all(n not in text for n in expected)
        dependency_rows.append({"file": str(path), "sha256": sha(path),
            "source_inputs": inputs, "changed_inputs": []})

    review_path = E / "eager-bound-decoder-retention-v2335.json"
    review = read(review_path)
    records = {review_path.name, review["cost_record"], review["replay_orchestration"],
               review["domain_observation_comparison"], review["deterministic_replay_comparison"],
               review["public_fact_audit"], "eager-bound-decoder-candidate-validation-v2276.json"}
    record_hashes = {n: sha(E / n) for n in sorted(records)}
    assert all(h == retained["artifacts_sha256"][n] for n, h in record_hashes.items())
    assert not review["fresh_timing_gate_passed"] and not review["original_complete_file_identity_gate_passed"]
    assert review["fresh_timing_processes"] == 0 and len(review["later_baseline_cpu_failures_preserved"]) == 2
    assert review["replay_cases"] == {"numerical": 4129, "public": 648}
    assert len(review["fresh_cargo_and_format_validations"]) == 9 and len(review["fresh_capped_validations"]) == 39
    assert all(sha(SOURCE / n) == h for n, h in current.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    out.mkdir()
    for i, path in enumerate(depfiles):
        (out / f"runtime-dependencies-{i}.d.gz").write_bytes(gzip.compress(path.read_bytes(), mtime=0))
    (out / "summary.json").write_text(json.dumps({
        "status": "Unchanged runtime inputs and a checked import-only proof correction support evidence reuse",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint": retained_path.name,
        "retained_checkpoint_sha256": sha(retained_path), "source_record": str(record.relative_to(E)),
        "source_record_sha256": sha(record), "previous_source_record_sha256": sha(old_record),
        "changed_sources": changed, "runtime_source_files_changed": 0,
        "source_files_verified": len(current), "main_files_verified": len(main_guard),
        "conditional_proof": {"verified_units": 662, "errors": 0, "production_contracts": 98,
                              "model_theorems": 137, "open_obligation_groups": 13},
        "guard_validation_sha256": sha(checks_path), "guard_log_sha256": sha(guards / "verus-invocations.jsonl.gz"),
        "fresh_guard_invocations": 259, "fresh_rejected_mutations": 255, "fresh_positive_baselines": 4,
        "new_negative_controls": negative_controls, "acceptance_tests": 10,
        "post_guard_difference": "Two README counts and the order of three distinct imports in one use statement changed. Every function body, specification, Python guard, manifest, lockfile and tool input is identical. Both formatting commands were rerun successfully after preserving the first import-order failure.",
        "guard_import_correction": {"old_import": old_import, "new_import": new_import,
            "identical_imported_paths": imported_names(old_import), "reconstructed_model_variants": len(variants),
            "affected_actual_invocations": affected_calls,
            "basis": "The use statement only introduces the same three distinct names from the same path; no alias, visibility, attribute or executable expression changes. Reordering it commutes exactly with all 39 model mutations. All 133 affected actual invocation hashes map to these reconstructed variants. The corrected complete proof passes; the historical negative runs are not relabelled as current-source executions."},
        "fresh_final_format_phase": formatting.name,
        "fresh_final_format_result_sha256": sha(formatting / "result.json"),
        "earlier_format_failure_preserved": "typed-api-prototype-v2341-approximation-metadata-included-fmt",
        "unchanged_proof_dependencies_sha256": sha(phase / "proof-reports/dependencies.json"),
        "unchanged_verifier_files": len(tool["tool_sha256"]), "runtime_libraries_sha256": library_hashes,
        "frozen_probe_binaries_sha256": binary_hashes, "frozen_probe_sources_sha256": probe_hashes,
        "compiler_dependency_records": dependency_rows, "retained_runtime_records_sha256": record_hashes,
        "historical_runtime_evidence_applicable": True, "historical_runs_relabelled_as_fresh": 0,
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "fresh_replay_cases": 0,
        "original_whole_file_gate_passed": False, "fresh_timing_gate_passed": False,
        "later_baseline_cpu_failures_preserved": review["later_baseline_cpu_failures_preserved"],
        "scope": "This proof-only extension changes four verification inputs and no ordinary runtime/build/test/probe source. The three compiler dependency records and unchanged libraries/seven frozen probes corroborate that boundary for the recorded builds. Runtime tests, costs, caps and replays remain historical v2335 evidence with their original identities, ignored cases, warnings and limitations. The checkpoint retention guard must still hash-check all v2335 artifacts. No new timing, resource, main/native, full-board or whole-crate claim follows."
    }, indent=2) + "\n")
    print(json.dumps({"source_files_verified": len(current), "changed_verification_files": 4,
        "runtime_source_files_changed": 0, "negative_mutations": 255, "new_controls": 10,
        "unchanged_frozen_probes": 7, "runtime_reuse_applicable": True}))


if __name__ == "__main__":
    main()
