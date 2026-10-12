"""Record actual BigInt getter contracts, import controls and native evidence."""
import ast
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
digest = lambda s: hashlib.sha256(s.encode()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def assert_logs(path, row, stem):
    assert row["stdout_sha256"] == sha(path / (stem + ".stdout.json"))
    assert row["stderr_sha256"] == sha(path / (stem + ".stderr.log"))


def main():
    out = E / "bigint-metadata-frontier-v2457.json"
    assert not out.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    original = snapshot(E / "candidate-preparation/source-hashes.json.gz")
    spec = importlib.util.spec_from_file_location("catalog_v2457", E / "validate_candidate_v3.py")
    catalog = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(catalog)
    assert catalog.source_hashes(SOURCE, original) == sources and len(sources) == 24751
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    previous_frontier = E / "biguint-crate-frontier-v2449.json"
    assert sha(previous_frontier) == "368ac57f18492c1455ea44bd5c0381f435dff4311005df4ff739b93e0c619a40"
    previous_stage = read(Path("/tmp/csgrs-biguint-crate-v2449-stage-preparation.json"))
    assert len(previous_stage["paths_sha256"]) == 103
    assert all(sha(ROOT / n) == h for n, h in previous_stage["paths_sha256"].items())

    base = E / "bigint-metadata-export-v2450"
    result, protocol = read(base / "result.json"), read(base / "protocol.json")
    assert result["exit_code"] == 0 and result["verification_results"]["verified"] == 453
    assert result["verification_results"]["errors"] == 0 and not (base / "stderr.log").read_text()
    assert result["protocol_sha256"] == sha(base / "protocol.json")
    assert result["stdout_sha256"] == sha(base / "stdout.json") and result["stderr_sha256"] == sha(base / "stderr.log")
    vendor = Path(protocol["vendor"])
    assert all(sha(vendor / n) == h for n, h in protocol["vendor_files_sha256"].items())
    assert [n for n, h in protocol["vendor_files_sha256"].items() if protocol["previous_vendor_files_sha256"].get(n) != h] == ["src/bigint.rs"]
    bigint = (vendor / "src/bigint.rs").read_text()
    assert sha(vendor / "src/bigint.rs") == sha(base / "bigint.rs")
    tree = ast.parse((E / "probe_bigint_metadata_v2450.py").read_text())
    constants = {n.targets[0].id: ast.literal_eval(n.value) for n in tree.body
                 if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
                 and n.targets[0].id in {"GHOST", "SIGN", "MAGNITUDE"}}
    restored = bigint
    reversals = [(constants["GHOST"], ""),
        (constants["SIGN"], "    #[inline]\n    pub fn sign(&self) -> Sign {\n        self.sign\n    }"),
        (constants["MAGNITUDE"], "    #[inline]\n    pub fn magnitude(&self) -> &BigUint {\n        &self.data\n    }"),
        ("#[cfg_attr(verus_keep_ghost, verus_verify)]\npub enum Sign {", "pub enum Sign {"),
        ("#[cfg_attr(verus_keep_ghost, verus_verify)]\npub struct BigInt {", "pub struct BigInt {")]
    for before, after in reversals:
        assert restored.count(before) == 1
        restored = restored.replace(before, after)
    assert digest(restored) == protocol["previous_vendor_files_sha256"]["src/bigint.rs"]
    functions = read(base / "stdout.json")["func-details"]
    selected = [n for n in functions if n.startswith("num_bigint::bigint::")]
    assert set(selected) == {"num_bigint::bigint::Sign::clone", *[
        "num_bigint::bigint::BigInt::" + n for n in ["sign", "magnitude", "metadata_sign", "metadata_magnitude",
                                                    "metadata_storage_bits", "metadata_normalized", "metadata_value"]]}
    tool = Path(protocol["command"][0]).parent
    assert all(sha(tool / n) == h for n, h in protocol["tool_sha256"].items())
    assert all(sha(Path(n)) == h for n, h in protocol["dependency_artifacts_sha256"].items())
    assert all(sha(vendor.parents[1] / "artifacts" / n) == h for n, h in result["emitted_artifacts_sha256"].items())

    guard_original = E / "bigint-metadata-guards-v2451"
    guard_final = E / "bigint-metadata-guards-v2453"
    original_protocol, original_result = read(guard_original / "protocol.json"), read(guard_original / "result.json")
    guards = read(guard_final / "result.json")
    assert not original_result["complete"] and len(original_result["runs"]) == 1
    assert guards["complete"] and len(guards["runs"]) == 8 and all(r["gate_passed"] for r in guards["runs"])
    assert guards["protocol_sha256"] == sha(guard_final / "protocol.json")
    assert (guard_original / "baseline.stderr.log").read_text() == "note: verifying module bigint\n\n"
    assert_logs(guard_original, original_result["runs"][0], "baseline")
    assert guards["runs"][0]["original_phase"] == guard_original.name
    assert guards["runs"][0]["verification_results"]["verified"] == 3
    assert len(original_protocol["mutations"]) == 7
    for index, (name, kind, marker, before, after) in enumerate(original_protocol["mutations"], 1):
        start = bigint.index(marker)
        end = bigint.index("\n    }", start) + len("\n    }")
        assert bigint[start:end].count(before) == 1
        mutated = bigint[:start] + bigint[start:end].replace(before, after) + bigint[end:]
        job = original_protocol["jobs"][index]
        assert job["name"] == name and job["kind"] == kind
        assert digest(mutated) == job["vendor_files_sha256"]["src/bigint.rs"]
        assert sha(guard_original / "inputs" / (name + ".bigint.rs")) == digest(mutated)
        assert all(sha(Path(job["vendor"]) / n) == h for n, h in job["vendor_files_sha256"].items())
        row = guards["runs"][index]
        assert row["name"] == name and row["exit_code"] == 1 and row["real_proof_failure"]
        assert not row["verification_results"]["encountered-vir-error"] and row["verification_results"]["errors"] > 0
        assert_logs(guard_final, row, name)

    cargo = E / "bigint-cargo-export-v2454"
    cargo_result, cargo_protocol = read(cargo / "result.json"), read(cargo / "protocol.json")
    assert cargo_result["gate_passed"] and cargo_result["error"] is None
    compiler = read(cargo / "compiler/compiler-result.json")
    assert cargo_result["compiler_result_sha256"] == sha(cargo / "compiler/compiler-result.json")
    assert compiler["verification_results"]["verified"] == 453 and compiler["verification_results"]["errors"] == 0
    assert compiler["warnings"] == 0 and compiler["inherited_jobserver_descriptors"] == [3, 5]
    assert compiler["stdout_sha256"] == sha(cargo / "compiler/compiler.stdout.json")
    assert compiler["stderr_sha256"] == sha(cargo / "compiler/compiler.stderr.log")
    assert not any(json.loads(line).get("level") == "warning" for line in (cargo / "compiler/compiler.stderr.log").read_text().splitlines())
    invocation = read(cargo / "compiler/compiler-invocation.json")
    assert invocation["verus_command"][6:] == invocation["cargo_arguments"]
    assert invocation["verus_command"][1:5] == ["--no-cheating", "--output-json", "--compile", "--export"]
    dependencies = read(cargo / "dependency-reports/dependencies.json")
    assert cargo_result["dependency_report_sha256"] == sha(cargo / "dependency-reports/dependencies.json")
    assert len(dependencies["packages"]) == len(dependencies["bindings"]) == 8
    fixture = Path(cargo_protocol["work"])
    assert all(sha(fixture / n) == h for n, h in cargo_protocol["work_files_sha256"].items())
    assert all(sha(fixture / "vendor/num-bigint" / n) == h for n, h in protocol["vendor_files_sha256"].items())
    assert sha(fixture / "artifacts/num_bigint.vir") == cargo_result["export_sha256"] == compiler["export_sha256"]
    for package in dependencies["packages"]:
        directory = Path(package["manifest_path"]).parent
        assert all(sha(directory / n) == h for n, h in dependencies["sources"][package["id"]].items())
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())

    caller_path = E / "bigint-facade-import-v2455"
    callers, caller_protocol = read(caller_path / "result.json"), read(caller_path / "protocol.json")
    assert callers["complete"] and len(callers["runs"]) == 5 and all(r["gate_passed"] for r in callers["runs"])
    assert callers["runs"][0]["verification_results"]["verified"] == 2
    assert not (caller_path / "baseline.stderr.log").read_text()
    caller = (E / "bigint-import-caller-v2455.rs").read_text()
    replacements = {"baseline": None,
        "sign-reversed": ("        Sign::Minus => -1,", "        Sign::Minus => 1,"),
        "magnitude-bits-zero": ("    let magnitude = value.magnitude();\n    magnitude.bits()", "    let _magnitude = value.magnitude();\n    0"),
        "missing-normalization": ("requires value.metadata_normalized(), value.metadata_storage_bits() <= u64::MAX,", "requires value.metadata_storage_bits() <= u64::MAX,"),
        "missing-count-bound": ("requires value.metadata_normalized(), value.metadata_storage_bits() <= u64::MAX,", "requires value.metadata_normalized(),")}
    for job, row in zip(caller_protocol["jobs"], callers["runs"], strict=True):
        assert job["name"] == row["name"]
        change = replacements[job["name"]]
        content = caller if change is None else caller.replace(*change)
        assert job["source_sha256"] == digest(content) == sha(Path(job["path"]))
        assert sha(caller_path / (job["name"] + ".rs")) == digest(content)
        assert_logs(caller_path, row, job["name"])
        external = [job["command"][i + 1] for i, arg in enumerate(job["command"]) if arg == "--extern"]
        assert len(external) == 1 and external[0].startswith("num=")
        if change is not None:
            assert caller.count(change[0]) == 1
            assert row["exit_code"] == 1 and row["proof_failure"]
            assert not row["verification_results"]["encountered-vir-error"] and row["verification_results"]["errors"] == 1

    native_path = E / "bigint-cargo-runtime-v2456"
    native, native_protocol = read(native_path / "result.json"), read(native_path / "protocol.json")
    assert native["complete"] and len(native["runs"]) == 3 and all(r["gate_passed"] for r in native["runs"])
    assert native["protocol_sha256"] == sha(native_path / "protocol.json")
    assert native["fresh_dependency_corpus_executions"] == 1 and native["fresh_scalar_runtime_tests"] == 0
    for row in native["runs"]:
        assert row["exit_code"] == 0
        assert row["stdout_sha256"] == sha(native_path / (row["name"] + ".stdout.log"))
        assert row["stderr_sha256"] == sha(native_path / (row["name"] + ".stderr.log"))
        assert not (native_path / (row["name"] + ".stderr.log")).read_text()
    assert sha(Path(native_protocol["commands"][-1]["command"][0])) == native["binary_sha256"]
    assert sha(Path(native_protocol["harness"])) == native_protocol["harness_sha256"]
    observations = dict(item.split("=", 1) for item in (native_path / "execute.stdout.log").read_text().split())
    assert observations["fixtures"] == "197388" and observations["paired_values"] == "789552"
    assert all(sha(Path(n)) == h for n, h in native_protocol["dependency_artifacts_sha256"].items())
    assert catalog.source_hashes(SOURCE, original) == sources
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == (ROOT / "docs/properties-status.json").read_bytes()
    records = [base / "result.json", guard_original / "result.json", guard_final / "result.json",
               cargo / "result.json", caller_path / "result.json", native_path / "result.json"]
    out.write_text(json.dumps({
        "status": "Actual BigInt sign/magnitude contracts, Cargo callers and focused native corpus pass in isolated support fixtures",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint": checkpoint_path.name,
        "retained_checkpoint_sha256": sha(checkpoint_path), "retained_source_record_sha256": checkpoint["source_record_sha256"],
        "previous_support_frontier_sha256": sha(previous_frontier), "previous_support_artifacts_rechecked": 103,
        "source_files_rechecked": len(sources), "main_files_rechecked": len(main_guard),
        "candidate_sources_changed": False, "source_catalog_unchanged": True, "main_runtime_admitted": False,
        "retained_verified_units": 736, "new_retained_proof_units": 0, "whole_crate_obligation_groups_open": 13,
        "result_records_sha256": {str(p.relative_to(E)): sha(p) for p in records},
        "actual_export_verified_units_including_existing_models": 453,
        "existing_export_units": 450, "new_verified_getter_bodies": 2, "new_generated_clone_units": 1,
        "actual_bigint_proof_functions_and_views": sorted(selected), "closed_representation_views": 5,
        "runtime_source_expressions_restored_exactly": True,
        "module_positive_baselines": 1, "module_executable_negative_controls": 2,
        "module_representation_negative_controls": 5, "module_negative_controls_reconstructed": 7,
        "import_caller_functions": 2, "import_positive_baselines": 1,
        "import_executable_negative_controls": 2, "import_missing_premise_negative_controls": 2,
        "cargo_bound_packages": 8, "cargo_compiler_warnings": 0,
        "native_corpus": observations, "native_result_sha256": sha(native_path / "result.json"),
        "fresh_dependency_corpus_executions": 1, "fresh_scalar_runtime_tests": 0,
        "failed_reader_preserved": "bigint-metadata-guards-v2451/result.json",
        "failed_reader_correction": "bigint-metadata-guards-v2453/protocol.json",
        "unexecuted_superseded_cargo_driver": "probe_bigint_cargo_export_v2452.py",
        "scope": "The actual dependency Sign enum and private BigInt fields support closed sign, magnitude, storage-count, normalization and signed-value views. The existing getters return the same runtime expressions. Their numerical sign and absolute-value conclusions require normalized magnitude storage and sign/zero consistency. The native-width full export verifies two getters and generated Sign clone in addition to the earlier 450 units. Two importing callers use only the Cargo num facade to prove numerical sign and absolute-value bit length; omitting normalization or count range is rejected. A new independent signed-word oracle covers focused public construction, negation and cancellation samples after proof erasure. This does not prove all constructors, arithmetic, equality/ordering/hash implementations, scalar bound-producing callers, or the original approximation's truth. SOURCE still uses its retained dependency and is unchanged; no retained proof count, runtime/cost gate or property status is promoted.",
        "remaining": [
            "Connect the actual bound_from_approx producer to these imported views and the retained interval/bit-factor lemmas, preserving early Unknown, lazy trailing-zero scans and metadata short-circuiting.",
            "Discharge or explicitly guard normalization, sign consistency and aggregate count-range premises at actual callers. A borrowed getter proof does not establish invariants for all unverified constructors and arithmetic.",
            "Prepare repository-appropriate model placement, Cargo/Verus wiring, publication/MSRV and cross-target qualification. Admit any scalar dependency/runtime change only after the original correctness, resource, cost, memory and size gates.",
            "Continue approximation truth, independent existence, atomic publication and exact-operation resource work. Both baseline CPU failures, thirteen whole-crate groups, all 39 properties, 14 targets, crate future work, main/native/full-board admission and complete pre-push verification remain open."
        ],
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True
    }, indent=2) + "\n")
    print(json.dumps({"actual_export_units_including_models": 453, "new_getters": 2,
        "module_negative_controls": 7, "import_negative_controls": 4, "cargo_warnings": 0,
        "native_fixtures": 197388, "native_pairs": 789552, "source_unchanged": True}))


if __name__ == "__main__":
    main()
