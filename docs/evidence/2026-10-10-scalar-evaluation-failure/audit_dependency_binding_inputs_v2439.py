"""Qualify proof/runtime evidence reuse after verification-only binding changes."""
import ast
import gzip
import hashlib
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
from audit_digit_scan_inputs_v2372 import generated_guard_entrypoints, invocations

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PREFIX = "crates/hyperreal/verification/"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
digest = lambda s: hashlib.sha256(s.encode()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    out = E / "dependency-binding-input-audit-v2439.json"
    assert not out.exists()
    previous_path = E / "typed-api-prototype-v2386-state.json"
    assert sha(previous_path) == "6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9"
    previous = read(previous_path)
    old = snapshot(E / previous["source_record"])
    phase = E / "typed-api-prototype-v2433-dependency-binding-verus"
    record = phase / "source-hashes.json.gz"
    current = snapshot(record)
    assert sha(record) == "650385802de02964ad26d0f60ea407294d451b2a538f1de86f94074c1821e236"
    assert len(old) == len(current) == 24751 and old.keys() == current.keys()
    changed = {n: {"before": old[n], "after": h} for n, h in current.items() if old[n] != h}
    assert set(changed) == {"crates/hyperreal/scripts/verus_dependencies.py",
                            PREFIX + "test_dependency_identity.py", PREFIX + "README.md"}
    assert all(sha(SOURCE / n) == h for n, h in current.items())
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    proof = read(phase / "proof-reports/evidence.json")
    result = read(phase / "result.json")
    assert result["exit_code"] == 0 and result["source_unchanged"] and result["main_unchanged"]
    assert result["source_catalog_rechecked"] and result["source_snapshot_error"] is None
    old_reports = (E / previous["source_record"]).parent / "proof-reports"
    old_proof = read(old_reports / "evidence.json")
    assert proof["scope"] == old_proof["scope"]
    assert proof["verification_results"]["verified"] == 736 and proof["verification_results"]["errors"] == 0
    scope = proof["scope"]
    assert len(scope["verified_executable_contracts"]) == 98
    assert len(scope["verified_model_theorems"]) == 178 and len(scope["obligations"]) == 13
    dependencies = read(phase / "proof-reports/dependencies.json")
    old_dependencies = read(old_reports / "dependencies.json")
    for key in ["sources", "artifacts", "verus_arguments"]:
        assert dependencies[key] == old_dependencies[key], key
    package_map = lambda d: {p["id"]: p for p in d["packages"]}
    assert package_map(dependencies) == package_map(old_dependencies)
    assert len(dependencies["packages"]) == len(dependencies["bindings"]) == 8
    for package in dependencies["packages"]:
        parent = Path(package["manifest_path"]).parent
        assert all(sha(parent / n) == h for n, h in dependencies["sources"][package["id"]].items())
    assert len(dependencies["artifacts"]) == 19
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    tooling = read(phase / "proof-reports/tooling.json")
    assert tooling["tool_unchanged"]
    assert tooling["tool_sha256"] == read(old_reports / "tooling.json")["tool_sha256"]

    validation_path = E / "dependency-binding-validation-v2438.json"
    validation = read(validation_path)
    assert validation["complete"] and len(validation["phases"]) == 4
    for name, row in validation["phases"].items():
        assert row["result"] == read(E / name / "result.json")
        assert row["result"]["exit_code"] == 0
        assert row["result"]["source_unchanged"] and row["result"]["main_unchanged"]
        assert row["result"]["source_catalog_rechecked"] and row["result"]["source_snapshot_error"] is None
        assert sha(E / name / "source-hashes.json.gz") == sha(record)
    for version, name, count in [(2434, "dependency-identity-tests", 19),
                                  (2435, "dependency-binding-acceptance", 26)]:
        stderr = (E / f"typed-api-prototype-v{version}-{name}/stderr.log").read_text()
        assert f"Ran {count} tests" in stderr and "\nOK\n" in stderr
    correction = read(E / "dependency-binding-validation-correction-v2438.json")
    assert correction["dependency_identity_tests"] == 19
    assert correction["acceptance_only_tests"] == 7 and correction["acceptance_entrypoint_tests"] == 26
    assert not read(E / "dependency-binding-validation-v2434.json")["complete"]
    assert len(correction["remaining_runs"][1]["command"][8:]) == 112

    inherited_path = E / "word-scan-input-audit-v2385.json"
    assert sha(inherited_path) == previous["artifacts_sha256"][inherited_path.name]
    inherited = read(inherited_path)
    assert inherited["total_applicable_rejected_mutations"] == 306
    assert inherited["total_applicable_positive_baselines"] == 7
    logs = [E / n for n in inherited["prior_guard_logs_sha256"]]
    logs.append(E / "typed-api-prototype-v2381-word-scan-rejections/proof-checks/verus-invocations.jsonl.gz")
    assert len(logs) == 4
    calls = []
    for path in logs:
        assert sha(path) == previous["artifacts_sha256"][str(path.relative_to(E))]
        assert read(path.parent / "tooling.json")["tool_sha256"] == tooling["tool_sha256"]
        calls += invocations(path)
    assert len(calls) == 313 and sum(c["exit_code"] == 0 for c in calls) == 7
    constructors = generated_guard_entrypoints()
    for name in ["test_digit_scan_rejections.py", "test_word_scan_rejections.py"]:
        tree = ast.parse((SOURCE / PREFIX / name).read_text())
        writes = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute) and n.func.attr == "write_text"
            and isinstance(n.func.value, ast.BinOp) and isinstance(n.func.value.op, ast.Div)
            and isinstance(n.func.value.right, ast.Constant) and n.func.value.right.value == "lib.rs"]
        assert len(writes) == 1, name
        constructors.append((name, compile(ast.Expression(writes[0].args[0]), name, "eval")))
    counts = {n: 0 for n, _ in constructors}
    for call in calls:
        locals_ = {"root": Path(call["command"][-1]).parent,
                   "modules": ["computable_bounds", "computable_cache", "magnitude_model",
                               "integer_approximation_model", "real_approximation_model"]}
        matched = [name for name, code in constructors
            if digest(eval(code, {"__builtins__": {}, **locals_}, locals_)) == call["rust_inputs_sha256"]["lib.rs"]]
        assert len(matched) == 1, call["command"]
        counts[matched[0]] += 1
    assert counts == {"test_rejections.py": 104, "test_structural_rejections.py": 22,
        "test_bound_rejections.py": 91, "test_cache_rejections.py": 42,
        "test_bit_metadata_rejections.py": 13, "test_digit_scan_rejections.py": 20,
        "test_word_scan_rejections.py": 21}
    runtime_path = E / "approximation-metadata-input-audit-v2344/summary.json"
    assert sha(runtime_path) == previous["artifacts_sha256"][str(runtime_path.relative_to(E))]
    runtime = read(runtime_path)
    assert runtime["historical_runtime_evidence_applicable"]
    for key in ["runtime_libraries_sha256", "frozen_probe_binaries_sha256"]:
        assert all(sha(Path(n)) == h for n, h in runtime[key].items())
    for key in ["frozen_probe_sources_sha256", "retained_runtime_records_sha256"]:
        assert all(sha(E / n) == h for n, h in runtime[key].items())

    support = {}
    paths = ["dependency-binding-units-v2425/result.json", "dependency-binding-cargo-v2427/result.json",
             "candidate-snapshot-validation-v2430.json", "workspace-dependency-binding-v2431/result.json"]
    for name in paths:
        support[name] = {"sha256": sha(E / name)}
    units, cargo, catalog, workspace = [read(E / n) for n in paths]
    assert units["exit_code"] == 0 and units["nineteen_tests_passed"]
    assert cargo["complete"] and all(r["gate_passed"] for r in cargo["runs"])
    assert [(r["name"], r["expected_acceptance"], len(r["calls"])) for r in cargo["runs"]] == [
        ("matched", True, 7), ("split", False, 2)]
    assert catalog["thirteen_tests_passed"] and catalog["retained_source_snapshot_identical"]
    assert len(workspace["packages"]) == len(workspace["bindings"]) == 8
    assert workspace["source_unchanged"] and workspace["main_unchanged"]
    support_commits = {}
    for version, stem in [(2397, "storage-contract"), (2423, "biguint-integration")]:
        path = Path(f"/tmp/csgrs-{stem}-v{version}-stage-preparation.json")
        stage = read(path)
        assert all(sha(ROOT / n) == h for n, h in stage["paths_sha256"].items())
        support_commits[stem] = {"preparation_sha256": sha(path),
                                "committed_files_rechecked": len(stage["paths_sha256"])}
    assert all(sha(SOURCE / n) == h for n, h in current.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    out.write_text(json.dumps({
        "status": "Dependency binding and complete scalar source catalogs pass; unchanged proof/runtime inputs support historical reuse",
        "runner_sha256": sha(Path(__file__)), "previous_checkpoint": previous_path.name,
        "previous_checkpoint_sha256": sha(previous_path), "source_record": str(record.relative_to(E)),
        "source_record_sha256": sha(record), "source_files_verified": len(current), "main_files_verified": len(guard),
        "changed_sources": changed, "runtime_source_files_changed": 0,
        "conditional_proof": {"verified_units": 736, "errors": 0, "production_contracts": 98,
                              "model_theorems": 178, "open_obligation_groups": 13},
        "fresh_validation_record": validation_path.name, "fresh_validation_record_sha256": sha(validation_path),
        "fresh_dependency_identity_tests": 19, "fresh_acceptance_entrypoint_tests": 26,
        "distinct_candidate_tests": 26, "acceptance_includes_identity_tests": True,
        "included_format_files": 112, "failed_reader_preserved": "dependency-binding-validation-v2434.json",
        "failed_reader_sha256": sha(E / "dependency-binding-validation-v2434.json"),
        "reader_correction_sha256": sha(E / "dependency-binding-validation-correction-v2438.json"),
        "inherited_input_audit": inherited_path.name, "inherited_input_audit_sha256": sha(inherited_path),
        "fresh_rejected_mutations": 0, "fresh_positive_baselines": 0,
        "reused_rejected_mutations": 306, "reused_positive_baselines": 7,
        "total_applicable_rejected_mutations": 306, "total_applicable_positive_baselines": 7,
        "reconstructed_prior_temporary_entrypoints": counts,
        "prior_guard_logs_sha256": {str(p.relative_to(E)): sha(p) for p in logs},
        "old_guard_reuse_basis": "All Rust inputs and guard generators are unchanged from v2386. The 313 original temporary entrypoints reconstruct exactly. Dependency package maps, source maps, all nineteen artifact bytes and all four Verus arguments match the previous full proof; package list order alone differs. The fresh full proof uses the stronger binding helper. Original mutant validation and execution identities remain inherited through v2385/v2372/v2357/v2344; no mutation run is relabelled as fresh.",
        "proof_dependencies_sha256": sha(phase / "proof-reports/dependencies.json"),
        "prior_proof_dependencies_sha256": sha(old_reports / "dependencies.json"),
        "unchanged_dependency_packages": 8, "unchanged_dependency_artifacts": 19,
        "support_qualification": support, "prior_support_commits_rechecked": support_commits,
        "fresh_preview_identity_tests": 19, "fresh_source_catalog_tests": 13,
        "actual_cargo_fixtures": 2, "fixture_cargo_processes": 9,
        "actual_workspace_proof_bindings": 8,
        "historical_runtime_input_audit_sha256": sha(runtime_path),
        "historical_runtime_evidence_applicable": True, "unchanged_frozen_probe_binaries": 7,
        "unchanged_runtime_libraries": 3, "historical_runs_relabelled_as_fresh": 0,
        "fresh_runtime_test_processes": 0, "fresh_cost_processes": 0, "fresh_replay_cases": 0,
        "original_whole_file_and_timing_failures_preserved": True,
        "later_baseline_cpu_failures_preserved": inherited["later_baseline_cpu_failures_preserved"],
        "scope": "Verification infrastructure only. Proof dependencies must match the production root closure, exact registry identity and canonical manifest path; local sources, all file types, binaries and configuration are checked around resolution. Candidate source discovery covers original workspace files plus introduced scalar files, rejecting scalar source symlinks. The dependency model remains transparent Sign only. No vendor runtime application, new Rust theorem, arithmetic assumption, fresh runtime qualification, main/native admission or property/target completion follows. BigUint shared-source previews remain separate support evidence; all thirteen whole-crate groups remain open."
    }, indent=2) + "\n")
    print(json.dumps({"verified_units": 736, "distinct_candidate_tests": 26,
        "reused_negative_controls": 306, "reconstructed_prior_entrypoints": len(calls),
        "source_files_verified": len(current), "runtime_source_files_changed": 0}))


if __name__ == "__main__":
    main()
