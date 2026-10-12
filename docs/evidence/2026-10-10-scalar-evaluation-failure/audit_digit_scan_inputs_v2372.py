"""Bind generic scan proofs and fresh guards to unchanged prior proof/runtime inputs."""
import ast
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PREFIX = "crates/hyperreal/verification/"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
digest = lambda s: hashlib.sha256(s.encode()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def invocations(path):
    return [json.loads(line) for line in gzip.open(path, "rt")]


def generated_guard_entrypoints():
    """Read the unchanged constructors of temporary lib.rs, not the full target."""
    constructors = []
    names = ["test_rejections.py", "test_structural_rejections.py", "test_bound_rejections.py",
             "test_cache_rejections.py", "test_bit_metadata_rejections.py"]
    for name in names:
        text = (SOURCE / PREFIX / name).read_text()
        tree = ast.parse(text)
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute) and n.func.attr == "write_text"
            and isinstance(n.func.value, ast.BinOp)
            and isinstance(n.func.value.op, ast.Div)
            and isinstance(n.func.value.right, ast.Constant) and n.func.value.right.value == "lib.rs"]
        assert len(calls) == 1, name
        constructors.append((name, compile(ast.Expression(calls[0].args[0]), name, "eval")))
    return constructors


def main():
    out = E / "digit-scan-input-audit-v2372.json"
    assert not out.exists()
    previous_path = E / "typed-api-prototype-v2358-state.json"
    assert sha(previous_path) == "5af0c6067447ffcf6f531b7b3cb206b83532225b99f6c8a1f66cbca7698bc991"
    previous = read(previous_path)
    old = snapshot(E / previous["source_record"])
    phase = E / "typed-api-prototype-v2367-digit-scan-final-verus"
    record = phase / "source-hashes.json.gz"
    current = snapshot(record)
    assert sha(record) == "a83cff31d79bb8bbbb5100829d14a5683ca081410433e95b32d22460a062a730"
    assert len(old) == 24747 and len(current) == 24749 and not old.keys() - current.keys()
    assert all(sha(SOURCE / n) == h for n, h in current.items())
    changed = {n: {"before": old.get(n), "after": h} for n, h in current.items() if old.get(n) != h}
    assert set(changed) == {PREFIX + n for n in ["lib.rs", "digit_scan_model.rs", "test_digit_scan_rejections.py",
                          "test_rejections.py", "scope.json", "README.md"]}
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    proof = read(phase / "proof-reports/evidence.json")
    assert read(phase / "result.json")["exit_code"] == 0
    assert proof["verification_results"]["verified"] == 716 and proof["verification_results"]["errors"] == 0
    old_proof = E / "typed-api-prototype-v2352-trailing-metadata-verus/proof-reports"
    old_scope = read(old_proof / "evidence.json")["scope"]
    assert proof["scope"]["verified_executable_contracts"] == old_scope["verified_executable_contracts"]
    assert len(old_scope["verified_executable_contracts"]) == 98
    assert len(proof["scope"]["verified_model_theorems"]) == 160
    assert len(old_scope["verified_model_theorems"]) == 143
    assert proof["scope"]["verified_model_theorems"][:143] == old_scope["verified_model_theorems"]
    assert proof["scope"]["obligations"] == old_scope["obligations"] and len(old_scope["obligations"]) == 13
    dependencies = read(phase / "proof-reports/dependencies.json")
    assert dependencies == read(old_proof / "dependencies.json")
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    tooling = read(phase / "proof-reports/tooling.json")
    assert tooling["tool_unchanged"] and tooling["tool_sha256"] == read(old_proof / "tooling.json")["tool_sha256"]

    validation_path = E / "digit-scan-validation-v2371.json"
    validation = read(validation_path)
    assert validation["complete"] and len(validation["phases"]) == 4
    for name, row in validation["phases"].items():
        assert row["result"] == read(E / name / "result.json")
        assert row["result"]["exit_code"] == 0 and row["result"]["source_unchanged"] and row["result"]["main_unchanged"]
        assert sha(E / name / "source-hashes.json.gz") == sha(record)
    assert read(E / "digit-scan-validation-protocol-v2371.json")["included_format_files"] == 111
    acceptance = (E / "typed-api-prototype-v2369-digit-scan-acceptance/stderr.log").read_text()
    assert "Ran 10 tests" in acceptance and "\nOK\n" in acceptance
    fresh = E / "typed-api-prototype-v2368-digit-scan-rejections/proof-checks"
    fresh_tool = read(fresh / "tooling.json")
    assert fresh_tool["exit_code"] == 0 and fresh_tool["tool_unchanged"]
    assert fresh_tool["tool_sha256"] == tooling["tool_sha256"]
    calls = invocations(fresh / "verus-invocations.jsonl.gz")
    assert len(calls) == 20 and calls[0]["exit_code"] == 0 and all(c["exit_code"] != 0 for c in calls[1:])
    tree = ast.parse((SOURCE / PREFIX / "test_digit_scan_rejections.py").read_text())
    mutations = ast.literal_eval(next(n.value for n in tree.body if isinstance(n, ast.Assign)
        and any(isinstance(t, ast.Name) and t.id == "MUTATIONS" for t in n.targets)))
    assert len(mutations) == 19
    model = (SOURCE / PREFIX / "digit_scan_model.rs").read_text()
    assert calls[0]["rust_inputs_sha256"]["digit_scan_model.rs"] == digest(model)
    rows = []
    for index, (name, before, after) in enumerate(mutations, 1):
        start = model.index("proof fn " + name + "(")
        end = model.index("\n}\n", start) + 3
        assert model[start:end].count(before) == 1
        mutated = model[:start] + model[start:end].replace(before, after) + model[end:]
        assert calls[index]["rust_inputs_sha256"]["digit_scan_model.rs"] == digest(mutated)
        assert any(m in calls[index]["stderr"] for m in ["postcondition not satisfied", "assertion failed",
                                                        "requires not satisfied", "precondition not satisfied"])
        rows.append({"function": name, "invocation_index": index, "mutated_source_sha256": digest(mutated)})

    inherited_path = E / "trailing-metadata-input-audit-v2357.json"
    assert sha(inherited_path) == previous["artifacts_sha256"][inherited_path.name]
    inherited = read(inherited_path)
    assert inherited["total_applicable_rejected_mutations"] == 267
    assert inherited["total_applicable_positive_baselines"] == 5
    old_logs = [E / "typed-api-prototype-v2339-approximation-metadata-rejections/proof-checks/verus-invocations.jsonl.gz",
                E / "typed-api-prototype-v2353-trailing-metadata-rejections/proof-checks/verus-invocations.jsonl.gz"]
    old_calls = []
    for log in old_logs:
        assert sha(log) == previous["artifacts_sha256"][str(log.relative_to(E))]
        assert read(log.parent / "tooling.json")["tool_sha256"] == tooling["tool_sha256"]
        old_calls += invocations(log)
    assert len(old_calls) == 272 and sum(c["exit_code"] == 0 for c in old_calls) == 5
    assert all("digit_scan_model.rs" not in c["rust_inputs_sha256"] for c in old_calls)
    entrypoint = (SOURCE / PREFIX / "test_rejections.py").read_text()
    for addition in ["from test_digit_scan_rejections import main as reject_digit_scan_mutations  # noqa: E402\n",
                     "    reject_digit_scan_mutations()\n"]:
        assert entrypoint.count(addition) == 1
        entrypoint = entrypoint.replace(addition, "")
    assert digest(entrypoint) == old[PREFIX + "test_rejections.py"]
    assert all(old.get(n) == h for n, h in current.items() if n.endswith(".py") and n not in changed)
    constructors = generated_guard_entrypoints()
    generated_counts = {n: 0 for n, _ in constructors}
    for call in old_calls:
        root = Path(call["command"][-1]).parent
        locals_ = {"root": root, "modules": ["computable_bounds", "computable_cache", "magnitude_model",
                                             "integer_approximation_model", "real_approximation_model"]}
        matched = []
        for name, code in constructors:
            text = eval(code, {"__builtins__": {}, **locals_}, locals_)
            if digest(text) == call["rust_inputs_sha256"]["lib.rs"]:
                matched.append(name)
        assert len(matched) == 1, call["command"]
        generated_counts[matched[0]] += 1
    runtime_path = E / inherited["inherited_input_audit"]
    assert sha(runtime_path) == inherited["inherited_input_audit_sha256"]
    runtime = read(runtime_path)
    assert runtime["historical_runtime_evidence_applicable"]
    for key in ["runtime_libraries_sha256", "frozen_probe_binaries_sha256"]:
        assert all(sha(Path(n)) == h for n, h in runtime[key].items())
    for key in ["frozen_probe_sources_sha256", "retained_runtime_records_sha256"]:
        assert all(sha(E / n) == h for n, h in runtime[key].items())
    support_rows = {}
    for version, expected_exit in [(2359, 0), (2360, 1), (2361, 1), (2362, 0)]:
        name = "digit-scan-model-probe-v2359" if version == 2359 else f"digit-scan-connections-probe-v{version}"
        path = E / name
        result = read(path / "result.json")
        assert result["exit_code"] == expected_exit and result["source_unchanged"] and result["tooling_unchanged"]
        for field, filename in [("protocol_sha256", "protocol.json"), ("stdout_sha256", "stdout.json"), ("stderr_sha256", "stderr.log")]:
            assert result[field] == sha(path / filename)
        support_rows[name] = {"result_sha256": sha(path / "result.json"), "exit_code": expected_exit,
                              "verification_results": read(path / "stdout.json")["verification-results"]}
    assert support_rows["digit-scan-model-probe-v2359"]["verification_results"]["verified"] == 25
    assert support_rows["digit-scan-connections-probe-v2362"]["verification_results"]["verified"] == 418
    assert read(E / "typed-api-prototype-v2364-digit-scan-verus/result.json")["exit_code"] == 2
    assert read(E / "typed-api-prototype-v2365-digit-scan-verus/result.json")["exit_code"] == 0
    assert all(sha(SOURCE / n) == h for n, h in current.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    out.write_text(json.dumps({
        "status": "Generic scan proofs and fresh controls pass; prior disjoint guards and runtime inputs remain applicable",
        "runner_sha256": sha(Path(__file__)), "previous_checkpoint": previous_path.name,
        "previous_checkpoint_sha256": sha(previous_path), "source_record": str(record.relative_to(E)),
        "source_record_sha256": sha(record), "source_files_verified": len(current), "main_files_verified": len(guard),
        "changed_sources": changed, "runtime_source_files_changed": 0,
        "conditional_proof": {"verified_units": 716, "errors": 0, "production_contracts": 98,
                              "model_theorems": 160, "open_obligation_groups": 13},
        "fresh_validation_record_sha256": sha(validation_path), "fresh_rejected_mutations": 19,
        "fresh_positive_baselines": 1, "fresh_guard_log_sha256": sha(fresh / "verus-invocations.jsonl.gz"),
        "fresh_mutations": rows, "fresh_acceptance_tests": 10, "included_format_files": 111,
        "inherited_input_audit": inherited_path.name, "inherited_input_audit_sha256": sha(inherited_path),
        "reused_rejected_mutations": 267, "reused_positive_baselines": 5,
        "total_applicable_rejected_mutations": 286, "total_applicable_positive_baselines": 6,
        "reconstructed_prior_temporary_entrypoints": generated_counts,
        "prior_guard_logs_sha256": {str(p.relative_to(E)): sha(p) for p in old_logs},
        "old_guard_reuse_basis": "All prior model and production Rust inputs and guard generators are unchanged from v2358. Each of the 272 captured temporary lib.rs hashes reconstructs exactly from its unchanged generator and original temporary path; none imports the new module or the changed full-target entrypoint. Removing exactly the new test entrypoint import and call restores its old bytes. Reuse inherits v2357 and its explicit v2344 import-order audit, preserving original invocation identities.",
        "unchanged_proof_dependencies_sha256": sha(phase / "proof-reports/dependencies.json"),
        "historical_runtime_evidence_applicable": True, "unchanged_frozen_probe_binaries": 7,
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "fresh_replay_cases": 0,
        "support_probes": support_rows, "launcher_failure_preserved": "typed-api-prototype-v2364-digit-scan-verus",
        "original_whole_file_and_timing_failures_preserved": True,
        "later_baseline_cpu_failures_preserved": inherited["later_baseline_cpu_failures_preserved"],
        "scope": "Seventeen conditional model lemmas and nineteen new controls. Radix-parametric denotation, first-match premises and the public u32 bit contract now connect to the retained flag decisions. Actual dependency storage/views, normalization, iterators, machine count range, sign imports, approximation truth, independent existence, callers and concurrent publication remain open. No runtime execution, historical relabelling, main/native admission or property/target completion follows.",
    }, indent=2) + "\n")
    print(json.dumps({"verified_units": 716, "fresh_negative_controls": 19,
        "reused_negative_controls": 267, "total_negative_controls": 286,
        "reconstructed_prior_entrypoints": len(old_calls), "source_files_verified": len(current)}))


if __name__ == "__main__":
    main()
