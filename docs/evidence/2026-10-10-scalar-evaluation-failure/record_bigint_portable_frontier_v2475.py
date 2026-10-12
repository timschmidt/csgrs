"""Audit portable dependency, archive, imported producer and unchanged native corpus evidence."""
import difflib
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tomllib

sys.dont_write_bytecode = True
E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob("*")) if p.is_file()}


def logs(path, row, stdout=None, stderr=None):
    assert row["stdout_sha256"] == sha(path / (stdout or (row["name"] + ".stdout.log")))
    assert row["stderr_sha256"] == sha(path / (stderr or (row["name"] + ".stderr.log")))


def main():
    out = E / "bigint-portable-frontier-v2475.json"
    assert not out.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    original_sources = snapshot(E / "candidate-preparation/source-hashes.json.gz")
    spec = importlib.util.spec_from_file_location("catalog_v2475", E / "validate_candidate_v3.py")
    catalog = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(catalog)
    assert catalog.source_hashes(SOURCE, original_sources) == sources and len(sources) == 24751
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    previous_frontier = E / "approximation-bound-frontier-v2465.json"
    assert sha(previous_frontier) == "412a8b592719c35568ae19096e26b9e044907766b340515e4b772ef961258a9d"
    support_count = 0
    for name, count in [("biguint-crate-v2449", 103), ("bigint-metadata-v2457", 91), ("approximation-bound-v2465", 102)]:
        paths = read(Path("/tmp/csgrs-" + name + "-stage-preparation.json"))["paths_sha256"]
        assert len(paths) == count and all(sha(ROOT / n) == h for n, h in paths.items())
        support_count += count
    prepared_path = E / "bigint-portable-package-v2466"
    prepared = read(prepared_path / "protocol.json")
    assert sha(prepared_path / "protocol.json") == "10e88be4c24ab80a0b5a1926f2434800838d5c5067f0d555cb215b4d030d2126"
    portable = Path(prepared["vendor"])
    assert files(portable) == prepared["vendor_files_sha256"] and len(files(portable)) == 74
    assert {n for n, h in prepared["base_vendor_files_sha256"].items() if prepared["vendor_files_sha256"][n] != h} == {"src/lib.rs", "Cargo.toml"}
    assert len(prepared["copied_model_files"]) == 17
    for name, record in prepared["copied_model_files"].items():
        assert sha(portable / name) == record["sha256"] == sources[record["source"]]
        assert sha(SOURCE / record["source"]) == record["sha256"]
    assert sha(Path(prepared["external_license_source"])) == prepared["external_license_sha256"] == sha(portable / "verification/LICENSE-HYPERREAL")
    assert tomllib.loads((portable / "Cargo.toml").read_text())["package"]["license"] == "Apache-2.0"
    patches = []
    for name, original_name in [("src/lib.rs", "lib.rs"), ("Cargo.toml", "Cargo.toml")]:
        before = (prepared_path / "original" / original_name).read_text()
        after = (portable / name).read_text()
        assert sha(prepared_path / "original" / original_name) == prepared["base_vendor_files_sha256"][name]
        patches.append("".join(difflib.unified_diff(before.splitlines(True), after.splitlines(True), fromfile="a/" + name, tofile="b/" + name, n=0)))
    assert "".join(patches) == (prepared_path / "portability.patch").read_text()
    root = (portable / "src/lib.rs").read_text()
    scalar = SOURCE / "crates/hyperreal"
    restored = root.replace("../verification/verified/mod.rs", str(scalar / "src/verified/mod.rs"))
    for name in ["magnitude_model", "bit_length_model", "digit_scan_model", "word_scan_model"]:
        restored = restored.replace("../verification/" + name + ".rs", str(scalar / "verification" / (name + ".rs")))
    assert restored == (prepared_path / "original/lib.rs").read_text()
    assert str(ROOT) not in root

    cargo_path = E / "bigint-portable-cargo-v2467"
    cargo, cargo_protocol = read(cargo_path / "result.json"), read(cargo_path / "protocol.json")
    assert cargo["gate_passed"] and cargo["error"] is None and cargo["bound_packages"] == 8
    assert cargo["protocol_sha256"] == sha(cargo_path / "protocol.json")
    compiler = read(cargo_path / "compiler/compiler-result.json")
    assert compiler["verification_results"]["verified"] == 453 and compiler["verification_results"]["errors"] == 0
    assert compiler["warnings"] == 0 and compiler["inherited_jobserver_descriptors"] == [3, 5]
    assert cargo["compiler_result_sha256"] == sha(cargo_path / "compiler/compiler-result.json")
    logs(cargo_path / "compiler", compiler, "compiler.stdout.json", "compiler.stderr.log")
    invocation = read(cargo_path / "compiler/compiler-invocation.json")
    assert invocation["verus_command"][6:] == invocation["cargo_arguments"]
    dependencies = read(cargo_path / "dependency-reports/dependencies.json")
    assert cargo["dependency_report_sha256"] == sha(cargo_path / "dependency-reports/dependencies.json")
    fixture = Path(cargo_protocol["work"])
    assert all(sha(fixture / n) == h for n, h in cargo_protocol["work_files_sha256"].items())
    assert files(fixture / "vendor/num-bigint") == prepared["vendor_files_sha256"]
    assert sha(fixture / "artifacts/num_bigint.vir") == cargo["export_sha256"] == compiler["export_sha256"]
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    for package in dependencies["packages"]:
        directory = Path(package["manifest_path"]).parent
        assert all(sha(directory / n) == h for n, h in dependencies["sources"][package["id"]].items())
    tool = Path(invocation["verus_command"][0]).parent
    assert all(sha(tool / n) == h for n, h in cargo_protocol["tool_sha256"].items())

    initial_path = E / "bigint-portable-validation-v2468"
    resumed_path = E / "bigint-portable-validation-v2471"
    initial, initial_protocol = read(initial_path / "result.json"), read(initial_path / "protocol.json")
    resumed, resumed_protocol = read(resumed_path / "result.json"), read(resumed_path / "protocol.json")
    assert not initial["complete"] and len(initial["runs"]) == 6
    assert [r["name"] for r in initial["runs"][:5]] == ["resolve", "workspace-msrv", "workspace-msrv-no-std", "i686-no-std", "wasm32-no-std"]
    assert all(r["gate_passed"] and r["warnings"] == 0 for r in initial["runs"][:5])
    assert initial["runs"][-1]["exit_code"] == 101 and not initial["runs"][-1]["gate_passed"]
    assert "failed to download `quickcheck v1.0.3`" in (initial_path / "clippy-all-features.stderr.log").read_text()
    assert not resumed["complete"] and len(resumed["runs"]) == 2
    assert resumed["runs"][0]["gate_passed"] and resumed["runs"][0]["warnings"] == 0
    assert resumed["runs"][1]["exit_code"] == 101 and not resumed["runs"][1]["gate_passed"]
    assert "reserved file name Cargo.toml.orig" in (resumed_path / "package.stderr.log").read_text()
    assert resumed_protocol["jobs"] == initial_protocol["jobs"][5:]
    for path, result in [(initial_path, initial), (resumed_path, resumed)]:
        assert result["protocol_sha256"] == sha(path / "protocol.json")
        for row in result["runs"]:
            logs(path, row)
    assert [r["run"] for r in resumed["reused_successful_runs"]] == initial["runs"][:5]
    fetch_path = E / "bigint-portable-fetch-v2470"
    fetch = read(fetch_path / "result.json")
    assert fetch["gate_passed"] and fetch["exit_code"] == 0 and fetch["locked_inputs_unchanged"]
    logs(fetch_path, fetch, "stdout.log", "stderr.log")
    assert resumed_protocol["fetch_result_sha256"] == sha(fetch_path / "result.json")
    locked = read(initial_path / "locked-inputs.json")
    assert files(Path(initial_protocol["work"]) / "num-bigint") == locked
    assert locked.keys() - prepared["vendor_files_sha256"].keys() == {"Cargo.lock"}
    assert all(locked[n] == h for n, h in prepared["vendor_files_sha256"].items())
    assert sha(initial_path / "Cargo.lock") == locked["Cargo.lock"]

    package_path = E / "bigint-portable-repackage-v2473"
    final_path = E / "bigint-portable-repackage-v2474"
    package_protocol = read(package_path / "protocol.json")
    packaged, final_protocol = read(final_path / "result.json"), read(final_path / "protocol.json")
    assert not (package_path / "result.json").exists()
    package_inputs = package_protocol["vendor_files_sha256"]
    assert package_inputs.keys() == locked.keys()
    assert [n for n, h in package_inputs.items() if locked[n] != h] == ["Cargo.toml"]
    work = Path(package_protocol["work"])
    assert files(work / "num-bigint") == package_inputs
    manifest_before = (portable / "Cargo.toml").read_text()
    manifest_after = (package_path / "Cargo.toml").read_text()
    assert manifest_after == manifest_before.replace('exclude = [\n', 'exclude = [\n    "/Cargo.toml.orig",\n    "/.cargo_vcs_info.json",\n')
    assert sha(package_path / "Cargo.toml") == package_inputs["Cargo.toml"]
    patch = "".join(difflib.unified_diff(manifest_before.splitlines(True), manifest_after.splitlines(True),
        fromfile="a/Cargo.toml", tofile="b/Cargo.toml", n=0))
    assert patch == (package_path / "packaging-manifest.patch").read_text()
    assert packaged["complete"] and len(packaged["runs"]) == 2 and all(r["gate_passed"] and r["warnings"] == 0 for r in packaged["runs"])
    assert packaged["protocol_sha256"] == sha(final_path / "protocol.json")
    assert final_protocol["previous_protocol_sha256"] == sha(package_path / "protocol.json")
    assert final_protocol["jobs"] == package_protocol["jobs"][1:]
    assert packaged["runs"][-1]["verification_results"]["verified"] == 453
    assert packaged["runs"][-1]["verification_results"]["errors"] == 0
    for row in packaged["runs"]:
        logs(final_path, row)
    package_record = read(final_path / "package-record.json")
    assert packaged["package_record_sha256"] == final_protocol["package_record_sha256"] == sha(final_path / "package-record.json")
    assert package_record["integrity_gate_passed"] and not package_record["original_process_exit_code_separately_recorded"]
    archive = package_path / "num-bigint-0.4.6.crate"
    assert sha(archive) == package_record["archive_sha256"] == sha(work / "target/package/num-bigint-0.4.6.crate")
    with tarfile.open(archive) as package:
        archived = {str(Path(entry.name).relative_to("num-bigint-0.4.6")):
            hashlib.sha256(package.extractfile(entry).read()).hexdigest() for entry in package.getmembers() if entry.isfile()}
    assert archived == package_record["unpacked_files_sha256"] == files(Path(package_record["unpacked"]))
    assert len(archived) == 73 and package_inputs.keys() - archived.keys() == {".cargo_vcs_info.json", ".gitignore"}
    assert {n for n in archived if archived[n] != package_inputs[n]} == {"Cargo.toml", "Cargo.toml.orig"}
    assert sha(work / "proof-artifacts/num_bigint.vir") == packaged["export_sha256"]

    producer_path = E / "approximation-bound-portable-proof-v2469"
    producer, producer_protocol = read(producer_path / "result.json"), read(producer_path / "protocol.json")
    assert producer["exit_code"] == 0 and producer["verification_results"]["verified"] == 737
    assert producer["verification_results"]["errors"] == 0 and not (producer_path / "stderr.log").read_bytes()
    assert producer["protocol_sha256"] == sha(producer_path / "protocol.json")
    logs(producer_path, producer, "stdout.json", "stderr.log")
    assert producer_protocol["exported_metadata_sha256"] == cargo["export_sha256"]
    assert producer_protocol["cargo_export_result_sha256"] == sha(cargo_path / "result.json")
    native_path = E / "approximation-bound-portable-native-v2472"
    native, native_protocol = read(native_path / "result.json"), read(native_path / "protocol.json")
    assert native["complete"] and len(native["runs"]) == 3 and all(r["gate_passed"] for r in native["runs"])
    assert native["protocol_sha256"] == sha(native_path / "protocol.json")
    assert native["fresh_isolated_producer_corpus_executions"] == 1 and native["fresh_scalar_runtime_tests"] == 0
    for row in native["runs"]:
        logs(native_path, row)
    binary = Path(native_protocol["commands"][-1]["command"][0])
    assert sha(binary) == native["binary_sha256"]
    for name, h in native_protocol["work_inputs_sha256"].items():
        assert sha(native_path / "inputs" / name) == sha(binary.parent / name) == h
    old_native = read(E / "approximation-bound-native-v2464/protocol.json")
    assert native_protocol["work_inputs_sha256"] == old_native["work_inputs_sha256"]
    assert native_protocol["harness_sha256"] == old_native["harness_sha256"]
    assert native_protocol["instrumentation_sha256"] == old_native["instrumentation_sha256"]
    assert native_protocol["shared_source_sha256"] == old_native["shared_source_sha256"]
    assert all(sha(Path(n)) == h for n, h in native_protocol["dependency_artifacts_sha256"].items())
    observations = native["runs"][-1]["observations"]
    assert observations["queries"] == "2775612" and observations["diagnostic_checksum"] == "83aad051019b1358"
    assert native["runs"][-1]["stdout_sha256"] == sha(E / "approximation-bound-native-v2464/execute.stdout.log")
    assert catalog.source_hashes(SOURCE, original_sources) == sources
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == (ROOT / "docs/properties-status.json").read_bytes()
    records = [cargo_path / "result.json", initial_path / "result.json", fetch_path / "result.json",
        resumed_path / "result.json", final_path / "result.json", producer_path / "result.json", native_path / "result.json"]
    result = {
        "status": "The dependency proof package is self-contained, builds across the selected targets, and preserves the imported producer proof and native oracle",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint": checkpoint_path.name,
        "retained_checkpoint_sha256": sha(checkpoint_path), "retained_source_record_sha256": checkpoint["source_record_sha256"],
        "previous_frontier_sha256": sha(previous_frontier), "previous_support_artifacts_rechecked": support_count,
        "source_files_rechecked": len(sources), "main_files_rechecked": len(main_guard),
        "candidate_sources_changed": False, "source_catalog_unchanged": True, "main_runtime_admitted": False,
        "retained_verified_units": 736, "new_retained_proof_units": 0, "whole_crate_obligation_groups_open": 13,
        "reviewable_package_base": str(portable.relative_to(E)),
        "reviewable_package_manifest_overlay": str((package_path / "Cargo.toml").relative_to(E)),
        "packaging_lockfile": str((initial_path / "Cargo.lock").relative_to(E)),
        "copied_ghost_files": 17, "portable_source_files": 74, "packaged_files": 73,
        "runtime_method_bytes_changed": False, "ghost_model_bytes_changed": False,
        "cargo_export_verified_units_including_existing_models": 453, "cargo_bound_packages": 8, "cargo_compiler_warnings": 0,
        "packaged_export_verified_units_including_existing_models": 453, "packaged_compiler_warnings": 0,
        "scalar_preview_verified_units": 737, "new_scalar_producer_bodies_in_preview": 1,
        "native_compile_checks": ["Rust 1.95 std", "Rust 1.95 no_std", "i686-unknown-linux-musl no_std", "wasm32-unknown-unknown no_std", "unpacked crate on Rust 1.98.1"],
        "all_feature_library_clippy_passed": True, "rust_1_60_tested": False, "cross_target_execution_claimed": False,
        "native_corpus": observations, "fresh_isolated_producer_corpus_executions": 1, "fresh_scalar_runtime_tests": 0,
        "archive_sha256": sha(archive), "archive_integrity_record_sha256": sha(final_path / "package-record.json"),
        "result_records_sha256": {str(p.relative_to(E)): sha(p) for p in records},
        "failures_preserved": ["bigint-portable-validation-v2468/result.json", "bigint-portable-validation-v2471/result.json",
            "bigint-portable-repackage-v2473/protocol.json (post-package reader stops before result.json)"],
        "scope": "Seventeen ghost-only files retain their qualified scalar bytes inside the dependency package, with relative root modules and explicit provenance/license metadata. The actual Cargo-bound num graph verifies and compiles the native dependency at 453 units with zero warnings. Focused MSRV/no_std/cross-target compilation and all-feature library Clippy pass. Cargo reserved-file and .gitignore-reader failures are preserved and corrected in separate records; the existing 73-file archive retains every Rust/model/license/provenance/lock byte, and its relocated root builds and verifies 453 units. The exact shared scalar producer still verifies in the 737-unit full preview and a fresh relink of the unchanged endpoint-oracle corpus preserves its checksum and call selection. These counts overlap and are not added or promoted to retained proof coverage. No SOURCE or main implementation changes occur.",
        "remaining": [
            "Wire the portable package and packaging-only manifest overlay into the actual scalar candidate with matching production/proof Cargo paths, locks and an audited Verus wrapper. The fixture package remains local and unpublished; root patches do not establish verified downstream registry builds, and upstream Rust 1.60 remains untested.",
            "Discharge or explicitly guard normalization, sign consistency and aggregate count-range premises at actual producers/constructors/arithmetic. The shared producer proof remains conditional on those premises and the approximation's one-unit truth contract.",
            "Qualify every candidate dependency/runtime edit under the original correctness, resource, cost, memory and size gates before retention. Continue independent existence, atomic publication and exact-operation resource work.",
            "Both baseline CPU failures, thirteen whole-crate groups, all 39 properties, 14 targets, crate future work, main/native/full-board admission and complete pre-push verification remain open or preserved."
        ],
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True
    }
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"cargo_units": 453, "packaged_units": 453, "scalar_preview_units": 737,
        "packaged_files": 73, "native_queries": 2775612, "source_unchanged": True, "frontier_sha256": sha(out)}))


if __name__ == "__main__":
    main()
