"""Record real-crate export, Cargo facade import and proof-erased execution support."""
import collections
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    out = E / "biguint-crate-frontier-v2449.json"
    assert not out.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    original = snapshot(E / "candidate-preparation/source-hashes.json.gz")
    spec = importlib.util.spec_from_file_location("catalog_v2449", E / "validate_candidate_v3.py")
    catalog = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(catalog)
    assert catalog.source_hashes(SOURCE, original) == sources and len(sources) == 24751
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    export_rows = {}
    for version, expected_exit in [(2441, 1), (2442, 0)]:
        path = E / f"biguint-crate-export-v{version}"
        result, protocol = read(path / "result.json"), read(path / "protocol.json")
        assert result["exit_code"] == expected_exit
        assert result["source_unchanged"] and result["main_unchanged"] and result["tooling_unchanged"]
        for key, name in [("protocol_sha256", "protocol.json"), ("stdout_sha256", "stdout.json"), ("stderr_sha256", "stderr.log")]:
            assert result[key] == sha(path / name)
        vendor = Path(protocol["vendor"])
        assert all(sha(vendor / n) == h for n, h in protocol["vendor_files_sha256"].items())
        assert sha(path / "lib.rs") == sha(vendor / "src/lib.rs")
        assert protocol["changed_files"] == ["src/lib.rs"]
        assert all(sha(E / protocol["preview"] / n) == h for n, h in protocol["preview_files_sha256"].items())
        for name, value in protocol["vendor_files_sha256"].items():
            if name != "src/lib.rs":
                assert sha(E / protocol["preview"] / "vendor/num-bigint" / name) == value
        artifacts = vendor.parents[1] / "artifacts"
        assert all(sha(artifacts / n) == h for n, h in result["emitted_artifacts_sha256"].items())
        assert "--no-cheating" in protocol["command"] and "--compile" in protocol["command"]
        if expected_exit == 0:
            assert result["verification_results"]["verified"] == 450 and result["verification_results"]["errors"] == 0
            assert not (path / "stderr.log").read_text()
        export_rows[path.name] = {"result_sha256": sha(path / "result.json"), "exit_code": expected_exit,
                                 "verification_results": result["verification_results"]}
    cargo_rows = {}
    for version in [2444, 2445]:
        path = E / f"biguint-cargo-export-v{version}"
        result, protocol = read(path / "result.json"), read(path / "protocol.json")
        assert result["gate_passed"] and result["error"] is None
        assert result["source_unchanged"] and result["main_unchanged"] and result["fixture_unchanged"]
        assert result["protocol_sha256"] == sha(path / "protocol.json")
        compiler_path = path / "compiler/compiler-result.json"
        compiler, invocation = read(compiler_path), read(path / "compiler/compiler-invocation.json")
        assert result["compiler_result"] == compiler and result["compiler_result_sha256"] == sha(compiler_path)
        assert compiler["verification_results"]["verified"] == 450 and compiler["verification_results"]["errors"] == 0
        assert compiler["stdout_sha256"] == sha(path / "compiler/compiler.stdout.json")
        assert compiler["stderr_sha256"] == sha(path / "compiler/compiler.stderr.log")
        assert compiler["compiler_invocation_sha256"] == sha(path / "compiler/compiler-invocation.json")
        assert invocation["verus_command"][6:] == invocation["cargo_arguments"]
        assert invocation["verus_command"][1:5] == ["--no-cheating", "--output-json", "--compile", "--export"]
        forbidden = ["--no-verify", "--no-erasure-check", "--no-lifetime", "--verify-module", "--verify-only-module"]
        assert all(flag not in invocation["verus_command"] for flag in forbidden)
        fixture = Path(protocol["work"])
        assert all(sha(fixture / n) == h for n, h in protocol["work_files_sha256"].items())
        assert sha(fixture / "artifacts/num_bigint.vir") == compiler["export_sha256"] == result["export_sha256"]
        dependencies_path = path / "dependency-reports/dependencies.json"
        dependencies = read(dependencies_path)
        assert result["dependency_report_sha256"] == sha(dependencies_path)
        assert len(dependencies["packages"]) == len(dependencies["bindings"]) == 8
        for package in dependencies["packages"]:
            root = Path(package["manifest_path"]).parent
            assert all(sha(root / n) == h for n, h in dependencies["sources"][package["id"]].items())
        assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
        selected = [p for p in dependencies["packages"] if p["name"] == "num-bigint"]
        assert len(selected) == 1 and selected[0]["source"] is None
        assert Path(selected[0]["manifest_path"]) == fixture / "vendor/num-bigint/Cargo.toml"
        assert dependencies["verus_arguments"] == result["verus_arguments"]
        diagnostics = [json.loads(line) for line in (path / "compiler/compiler.stderr.log").read_text().splitlines()]
        warnings = collections.Counter(d["message"] for d in diagnostics if d.get("level") == "warning")
        if version == 2445:
            assert not warnings and compiler["warnings"] == 0
            assert compiler["inherited_jobserver_descriptors"] == [3, 5]
            assert "-D warnings" in protocol["environment"]["RUSTFLAGS"]
        else:
            assert sum(warnings.values()) == 510
        cargo_rows[path.name] = {"result_sha256": sha(path / "result.json"),
            "compiler_result_sha256": sha(compiler_path), "dependency_report_sha256": sha(dependencies_path),
            "verified_units_including_imports": 450, "bound_packages": 8,
            "warning_diagnostics_including_summaries": sum(warnings.values()), "warning_messages": dict(warnings),
            "warning_free_gate_passed": not warnings, "exported_metadata_sha256": result["export_sha256"]}

    import_rows = {}
    for version, name, complete in [(2443, "biguint-import", True),
                                     (2446, "biguint-facade-import", False), (2447, "biguint-facade-import", True)]:
        path = E / f"{name}-v{version}"
        result, protocol = read(path / "result.json"), read(path / "protocol.json")
        assert result["protocol_sha256"] == sha(path / "protocol.json")
        assert result["complete"] == complete and result["source_unchanged"] and result["main_unchanged"]
        base = (path / "baseline.rs").read_text()
        replacements = {"baseline": None, "bits-zero": ("    value.bits()", "    0"),
                        "zeros-none": ("    value.trailing_zeros()", "    None"),
                        "ones-zero": ("    value.trailing_ones()", "    0")}
        for job in protocol["jobs"]:
            assert "--no-cheating" in job["command"]
            source = path / (job["name"] + ".rs")
            assert sha(source) == sha(Path(job["path"])) == job["source_sha256"]
            change = replacements[job["name"]]
            if change:
                assert base.count(change[0]) == 1 and source.read_text() == base.replace(*change)
            else:
                assert source.read_text() == base
        for row in result["runs"]:
            assert row["stdout_sha256"] == sha(path / (row["name"] + ".stdout.json"))
            assert row["stderr_sha256"] == sha(path / (row["name"] + ".stderr.log"))
        if complete:
            assert len(result["runs"]) == 4 and all(row["gate_passed"] for row in result["runs"])
            assert result["runs"][0]["exit_code"] == 0
            assert result["runs"][0]["verification_results"]["verified"] == 3
            assert not (path / "baseline.stderr.log").read_text()
            for row in result["runs"][1:]:
                assert row["exit_code"] == 1 and row["proof_failure"]
                assert not row["verification_results"]["encountered-vir-error"]
                assert row["verification_results"]["errors"] == 1
        else:
            assert len(result["runs"]) == 1 and not result["runs"][0]["proof_failure"]
        if version == 2447:
            assert "use num::bigint::BigUint;" in base
            command = protocol["jobs"][0]["command"]
            external = [command[i + 1] for i, arg in enumerate(command) if arg == "--extern"]
            assert len(external) == 1 and external[0].startswith("num=")
        import_rows[path.name] = {"result_sha256": sha(path / "result.json"), "complete": complete,
            "executed_jobs": len(result["runs"]), "passing_baselines": int(complete),
            "real_proof_failures": 3 if complete else 0, "verified_caller_functions": 3 if complete else 0}

    native_path = E / "biguint-cargo-runtime-v2448"
    native, native_protocol = read(native_path / "result.json"), read(native_path / "protocol.json")
    assert native["complete"] and len(native["runs"]) == 2 and all(r["gate_passed"] for r in native["runs"])
    assert native["protocol_sha256"] == sha(native_path / "protocol.json")
    assert native["fresh_dependency_corpus_executions"] == 1 and native["fresh_scalar_runtime_tests"] == 0
    for row in native["runs"]:
        assert row["exit_code"] == 0
        assert row["stdout_sha256"] == sha(native_path / (row["name"] + ".stdout.log"))
        assert row["stderr_sha256"] == sha(native_path / (row["name"] + ".stderr.log"))
        assert not (native_path / (row["name"] + ".stderr.log")).read_text()
    assert sha(Path(native_protocol["commands"][1]["command"][0])) == native["binary_sha256"]
    assert sha(native_path / "execute.stdout.log") == native_protocol["expected_corpus_stdout_sha256"]
    observations = dict(item.split("=", 1) for item in (native_path / "execute.stdout.log").read_text().split())
    assert observations == {"cases": "67137", "paired_values": "170970", "helper_width_checks": "134274", "diagnostic_checksum": "f8af72a05fcd32ff"}
    assert all(sha(Path(n)) == h for n, h in native_protocol["dependency_artifacts_sha256"].items())
    assert catalog.source_hashes(SOURCE, original) == sources
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == (ROOT / "docs/properties-status.json").read_bytes()
    out.write_text(json.dumps({
        "status": "Actual BigUint crate export, Cargo num facade import and proof-erased corpus pass in isolated support fixtures",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint": checkpoint_path.name,
        "retained_checkpoint_sha256": sha(checkpoint_path), "retained_source_record_sha256": checkpoint["source_record_sha256"],
        "source_files_rechecked": len(sources), "main_files_rechecked": len(main_guard),
        "candidate_sources_changed": False, "source_catalog_unchanged": True, "main_runtime_admitted": False,
        "retained_verified_units": 736, "new_retained_proof_units": 0, "whole_crate_obligation_groups_open": 13,
        "crate_export_attempts": export_rows, "cargo_export_attempts": cargo_rows, "import_attempts": import_rows,
        "native_runtime_result_sha256": sha(native_path / "result.json"), "native_corpus": observations,
        "fresh_dependency_corpus_executions": 1, "fresh_scalar_runtime_tests": 0,
        "verified_native_width": 64, "final_export_verified_units_including_existing_models": 450,
        "selected_metadata_executable_functions": 9, "new_export_units_including_loops_and_constant": 12,
        "existing_imported_units": 438, "passing_import_paths": 2,
        "import_caller_functions_per_path": 3, "import_negative_controls_per_path": 3,
        "distinct_caller_body_mutations": 3, "fresh_import_positive_baselines": 2,
        "fresh_import_negative_executions": 6, "final_cargo_compiler_warnings": 0,
        "unretained_failures_preserved": ["biguint-crate-export-v2441", "biguint-facade-import-v2446"],
        "warning_bearing_build_preserved": "biguint-cargo-export-v2444",
        "scope": "The actual complete num-bigint crate root, private BigUint representation and public inherent methods are compiled; only annotated metadata bodies and imported models are verified. Cargo builds one coherent num dependency graph and binds its local BigUint source to the isolated fixture's production root. Importing only num as a direct Rust external crate successfully resolves the exported transitive BigUint proof identity. A native execution of the unchanged independent oracle corpus agrees with the pinned upstream library after proof erasure. SOURCE still uses its retained registry dependency and is unchanged. No whole-num-bigint verification, new retained scalar theorem, production dependency admission or scalar runtime/cost/memory/size qualification is claimed.",
        "remaining": [
            "Apply a reviewable source/Cargo integration only after preserving the fixed scalar correctness, resource, cost, memory and size gates. The final proof metadata currently belongs to an isolated fixture, not SOURCE.",
            "Connect actual BigInt sign/magnitude and bound-producing callers to the imported BigUint contracts; normalization and aggregate count-range premises remain explicit. Preserve early Unknown, metadata short-circuiting, lazy trailing-zero scans and existing allocation policy.",
            "Make proof-model placement and Cargo/Verus wiring suitable for the repository and publication, then qualify supported features, native and 32-bit targets, MSRV and packaging. The actual crate export here covers host u64 limbs; the earlier shared-source proof covers both helper widths separately.",
            "Continue approximation truth, independent real existence, atomic publication and exact-operation resource work. Both historical baseline CPU failures, all thirteen whole-crate groups, 39 properties, 14 targets, crate future work, main/native/full-board admission and complete pre-push verification remain open."
        ],
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True
    }, indent=2) + "\n")
    print(json.dumps({"actual_crate_verified_units_including_models": 450, "warning_free_cargo_build": True,
        "import_paths": 2, "fresh_negative_import_executions": 6, "native_cases": 67137,
        "source_unchanged": True, "new_retained_proof_units": 0}))


if __name__ == "__main__":
    main()
