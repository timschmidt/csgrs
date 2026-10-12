"""Audit fresh typed-word controls and disjoint prior proof/runtime inputs."""
import ast
import gzip
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
import audit_digit_scan_inputs_v2372 as prior

E, ROOT, SOURCE, PREFIX = prior.E, prior.ROOT, prior.SOURCE, prior.PREFIX
sha, digest, read, snapshot, invocations = prior.sha, prior.digest, prior.read, prior.snapshot, prior.invocations


def main():
    out = E / "word-scan-input-audit-v2385.json"
    assert not out.exists()
    previous_path = E / "typed-api-prototype-v2373-state.json"
    assert sha(previous_path) == "163bf723328ab72df0a56025f9a55960eda13eb4c6767669965b61e66333508a"
    previous = read(previous_path)
    old = snapshot(E / previous["source_record"])
    phase = E / "typed-api-prototype-v2380-word-scan-verus"
    record = phase / "source-hashes.json.gz"
    current = snapshot(record)
    assert sha(record) == "67e3d14b5d7a319778d56b701498e97225c16134eca65bb14a7586c727f2fb5a"
    assert len(old) == 24749 and len(current) == 24751 and not old.keys() - current.keys()
    assert all(sha(SOURCE / n) == h for n, h in current.items())
    changed = {n: {"before": old.get(n), "after": h} for n, h in current.items() if old.get(n) != h}
    assert set(changed) == {PREFIX + n for n in ["lib.rs", "word_scan_model.rs", "test_word_scan_rejections.py",
                          "test_rejections.py", "scope.json", "README.md"]}
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    proof = read(phase / "proof-reports/evidence.json")
    assert read(phase / "result.json")["exit_code"] == 0
    assert proof["verification_results"]["verified"] == 736 and proof["verification_results"]["errors"] == 0
    old_proof = E / "typed-api-prototype-v2367-digit-scan-final-verus/proof-reports"
    old_scope = read(old_proof / "evidence.json")["scope"]
    scope = proof["scope"]
    assert scope["verified_executable_contracts"] == old_scope["verified_executable_contracts"]
    assert len(scope["verified_executable_contracts"]) == 98
    assert len(scope["verified_model_theorems"]) == 178 and len(old_scope["verified_model_theorems"]) == 160
    assert scope["verified_model_theorems"][:160] == old_scope["verified_model_theorems"]
    assert scope["obligations"] == old_scope["obligations"] and len(scope["obligations"]) == 13
    dependencies = read(phase / "proof-reports/dependencies.json")
    assert dependencies == read(old_proof / "dependencies.json")
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    tooling = read(phase / "proof-reports/tooling.json")
    assert tooling["tool_unchanged"] and tooling["tool_sha256"] == read(old_proof / "tooling.json")["tool_sha256"]
    validation_path = E / "word-scan-validation-v2384.json"
    validation = read(validation_path)
    assert validation["complete"] and len(validation["phases"]) == 4
    for name, row in validation["phases"].items():
        assert row["result"] == read(E / name / "result.json")
        assert row["result"]["exit_code"] == 0 and row["result"]["source_unchanged"] and row["result"]["main_unchanged"]
        assert sha(E / name / "source-hashes.json.gz") == sha(record)
    assert read(E / "word-scan-validation-protocol-v2384.json")["included_format_files"] == 112
    acceptance = (E / "typed-api-prototype-v2382-word-scan-acceptance/stderr.log").read_text()
    assert "Ran 10 tests" in acceptance and "\nOK\n" in acceptance
    fresh = E / "typed-api-prototype-v2381-word-scan-rejections/proof-checks"
    fresh_tool = read(fresh / "tooling.json")
    assert fresh_tool["exit_code"] == 0 and fresh_tool["tool_unchanged"]
    assert fresh_tool["tool_sha256"] == tooling["tool_sha256"]
    calls = invocations(fresh / "verus-invocations.jsonl.gz")
    assert len(calls) == 21 and calls[0]["exit_code"] == 0 and all(c["exit_code"] != 0 for c in calls[1:])
    tree = ast.parse((SOURCE / PREFIX / "test_word_scan_rejections.py").read_text())
    mutations = ast.literal_eval(next(n.value for n in tree.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "MUTATIONS" for t in n.targets)))
    assert len(mutations) == 20
    model = (SOURCE / PREFIX / "word_scan_model.rs").read_text()
    assert calls[0]["rust_inputs_sha256"]["word_scan_model.rs"] == digest(model)
    rows = []
    for index, (name, before, after) in enumerate(mutations, 1):
        start = model.index("proof fn " + name + "(")
        end = model.index("\n}\n", start) + 3
        assert model[start:end].count(before) == 1
        mutated = model[:start] + model[start:end].replace(before, after) + model[end:]
        assert calls[index]["rust_inputs_sha256"]["word_scan_model.rs"] == digest(mutated)
        assert any(m in calls[index]["stderr"] for m in ["postcondition not satisfied", "assertion failed",
                                                        "requires not satisfied", "precondition not satisfied"])
        rows.append({"function": name, "invocation_index": index, "mutated_source_sha256": digest(mutated)})
    inherited_path = E / "digit-scan-input-audit-v2372.json"
    assert sha(inherited_path) == previous["artifacts_sha256"][inherited_path.name]
    inherited = read(inherited_path)
    assert inherited["total_applicable_rejected_mutations"] == 286
    assert inherited["total_applicable_positive_baselines"] == 6
    old_logs = [E / n for n in inherited["prior_guard_logs_sha256"]]
    old_logs.append(E / "typed-api-prototype-v2368-digit-scan-rejections/proof-checks/verus-invocations.jsonl.gz")
    old_calls = []
    for log in old_logs:
        assert sha(log) == previous["artifacts_sha256"][str(log.relative_to(E))]
        assert read(log.parent / "tooling.json")["tool_sha256"] == tooling["tool_sha256"]
        old_calls += invocations(log)
    assert len(old_calls) == 292 and sum(c["exit_code"] == 0 for c in old_calls) == 6
    assert all("word_scan_model.rs" not in c["rust_inputs_sha256"] for c in old_calls)
    entrypoint = (SOURCE / PREFIX / "test_rejections.py").read_text()
    for addition in ["from test_word_scan_rejections import main as reject_word_scan_mutations  # noqa: E402\n",
                     "    reject_word_scan_mutations()\n"]:
        assert entrypoint.count(addition) == 1
        entrypoint = entrypoint.replace(addition, "")
    assert digest(entrypoint) == old[PREFIX + "test_rejections.py"]
    assert all(old.get(n) == h for n, h in current.items() if n.endswith(".py") and n not in changed)
    assert sha(Path(prior.__file__)) == previous["artifacts_sha256"][Path(prior.__file__).name]
    constructors = prior.generated_guard_entrypoints()
    name = "test_digit_scan_rejections.py"
    tree = ast.parse((SOURCE / PREFIX / name).read_text())
    entry_calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute) and n.func.attr == "write_text"
        and isinstance(n.func.value, ast.BinOp) and isinstance(n.func.value.op, ast.Div)
        and isinstance(n.func.value.right, ast.Constant) and n.func.value.right.value == "lib.rs"]
    assert len(entry_calls) == 1
    constructors.append((name, compile(ast.Expression(entry_calls[0].args[0]), name, "eval")))
    generated_counts = {n: 0 for n, _ in constructors}
    for call in old_calls:
        root = Path(call["command"][-1]).parent
        locals_ = {"root": root, "modules": ["computable_bounds", "computable_cache", "magnitude_model",
                                             "integer_approximation_model", "real_approximation_model"]}
        matched = [n for n, code in constructors if digest(eval(code, {"__builtins__": {}, **locals_}, locals_))
                   == call["rust_inputs_sha256"]["lib.rs"]]
        assert len(matched) == 1
        generated_counts[matched[0]] += 1
    runtime_path = E / "approximation-metadata-input-audit-v2344/summary.json"
    assert sha(runtime_path) == previous["artifacts_sha256"][str(runtime_path.relative_to(E))]
    runtime = read(runtime_path)
    assert runtime["historical_runtime_evidence_applicable"] and inherited["historical_runtime_evidence_applicable"]
    for key in ["runtime_libraries_sha256", "frozen_probe_binaries_sha256"]:
        assert all(sha(Path(n)) == h for n, h in runtime[key].items())
    for key in ["frozen_probe_sources_sha256", "retained_runtime_records_sha256"]:
        assert all(sha(E / n) == h for n, h in runtime[key].items())
    support_rows = {}
    for version, expected_exit in [(2374, 1), (2375, 1), (2376, 1), (2377, 0)]:
        path = E / f"word-scan-probe-v{version}"
        result = read(path / "result.json")
        assert result["exit_code"] == expected_exit and result["source_unchanged"] and result["tooling_unchanged"]
        for field, filename in [("protocol_sha256", "protocol.json"), ("stdout_sha256", "stdout.json"), ("stderr_sha256", "stderr.log")]:
            assert result[field] == sha(path / filename)
        support_rows[path.name] = {"result_sha256": sha(path / "result.json"), "exit_code": expected_exit,
                                  "verification_results": read(path / "stdout.json")["verification-results"]}
    assert support_rows["word-scan-probe-v2377"]["verification_results"]["verified"] == 438
    preparation_failure = E / "word-scan-preparation-failure-v2378.json"
    assert not read(preparation_failure)["source_applied"]
    assert all(sha(SOURCE / n) == h for n, h in current.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    out.write_text(json.dumps({
        "status": "Typed word-scan proofs and controls pass; unchanged prior proof/runtime inputs remain applicable",
        "runner_sha256": sha(Path(__file__)), "prior_audit_helper_sha256": sha(Path(prior.__file__)),
        "previous_checkpoint": previous_path.name, "previous_checkpoint_sha256": sha(previous_path),
        "source_record": str(record.relative_to(E)), "source_record_sha256": sha(record),
        "source_files_verified": len(current), "main_files_verified": len(guard), "changed_sources": changed,
        "runtime_source_files_changed": 0,
        "conditional_proof": {"verified_units": 736, "errors": 0, "production_contracts": 98,
                              "model_theorems": 178, "open_obligation_groups": 13},
        "fresh_validation_record_sha256": sha(validation_path), "fresh_rejected_mutations": 20,
        "fresh_positive_baselines": 1, "fresh_guard_log_sha256": sha(fresh / "verus-invocations.jsonl.gz"),
        "fresh_mutations": rows, "fresh_acceptance_tests": 10, "included_format_files": 112,
        "inherited_input_audit": inherited_path.name, "inherited_input_audit_sha256": sha(inherited_path),
        "reused_rejected_mutations": 286, "reused_positive_baselines": 6,
        "total_applicable_rejected_mutations": 306, "total_applicable_positive_baselines": 7,
        "reconstructed_prior_temporary_entrypoints": generated_counts,
        "prior_guard_logs_sha256": {str(p.relative_to(E)): sha(p) for p in old_logs},
        "old_guard_reuse_basis": "All previous model and production Rust sources and guard generators are unchanged from v2373. Each of 292 temporary entrypoints reconstructs exactly from its unchanged generator and original path. None imports the new model or changed full-target lib.rs. Removing the new guard import and call restores the prior test entrypoint. The v2372/v2357/v2344 audits and all original execution identities remain in force.",
        "unchanged_proof_dependencies_sha256": sha(phase / "proof-reports/dependencies.json"),
        "historical_runtime_evidence_applicable": True, "unchanged_frozen_probe_binaries": 7,
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "fresh_replay_cases": 0,
        "support_probes": support_rows, "preparation_failure": preparation_failure.name,
        "preparation_failure_sha256": sha(preparation_failure),
        "original_whole_file_and_timing_failures_preserved": True,
        "later_baseline_cpu_failures_preserved": inherited["later_baseline_cpu_failures_preserved"],
        "scope": "Eighteen conditional typed-word lemmas and twenty controls. The 64-bit denotation agrees with the actual verified limb prefix; both native widths supply the generic scan factors, normalized bit lengths and zero-suffix truncation identity. Real dependency storage and iterator execution, machine count range premises, sign imports, approximation truth, independent existence, callers and atomic publication remain open. No runtime evidence is relabelled, and no main/native/property/target admission follows.",
    }, indent=2) + "\n")
    print(json.dumps({"verified_units": 736, "fresh_negative_controls": 20, "reused_negative_controls": 286,
        "total_negative_controls": 306, "reconstructed_prior_entrypoints": len(old_calls), "source_files_verified": len(current)}))


if __name__ == "__main__":
    main()
