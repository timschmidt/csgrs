"""Audit and record the shared bound-producer preview without admitting runtime changes."""
import difflib
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
digest = lambda text: hashlib.sha256(text.encode()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob("*")) if p.is_file()}


def logs(path, row, name, extension="json"):
    assert row["stdout_sha256"] == sha(path / (name + ".stdout." + extension))
    assert row["stderr_sha256"] == sha(path / (name + ".stderr.log"))


def main():
    out = E / "approximation-bound-frontier-v2465.json"
    assert not out.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    original_sources = snapshot(E / "candidate-preparation/source-hashes.json.gz")
    spec = importlib.util.spec_from_file_location("catalog_v2465", E / "validate_candidate_v3.py")
    catalog = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(catalog)
    assert catalog.source_hashes(SOURCE, original_sources) == sources and len(sources) == 24751
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    previous_frontier = E / "bigint-metadata-frontier-v2457.json"
    assert sha(previous_frontier) == "84e63a914c81951aeb4984a681d0b158c153177cd545c7884f1f349beb036d17"
    support_count = 0
    for manifest, count in [("biguint-crate-v2449", 103), ("bigint-metadata-v2457", 91)]:
        paths = read(Path("/tmp/csgrs-" + manifest + "-stage-preparation.json"))["paths_sha256"]
        assert len(paths) == count and all(sha(ROOT / n) == h for n, h in paths.items())
        support_count += count

    preview = E / "approximation-bound-candidate-v2460"
    shared_name = "candidate/crates/hyperreal/src/computable/node/approximation_bound.rs"
    proposal = (preview / shared_name).read_text()
    for version in [2458, 2460]:
        path = E / f"approximation-bound-candidate-v{version}"
        protocol = read(path / "protocol.json")
        assert all(sha(path / n) == h for n, h in protocol["preview_files_sha256"].items())
        assert sha(path / shared_name) == protocol["proposal_sha256"] == sha(E / f"approximation-bound-shared-proposal-v{version}.rs")
        patches = []
        for name in protocol["candidate_source_paths"]:
            new = (path / "candidate" / name).read_text()
            old_path = path / "original" / name
            old = old_path.read_text() if old_path.exists() else ""
            if old_path.exists():
                assert sha(old_path) == sources[name]
            else:
                assert name not in sources
            patches.append("".join(difflib.unified_diff(old.splitlines(True), new.splitlines(True),
                fromfile="a/" + name, tofile="b/" + name, n=0)))
        assert "".join(patches) == (path / "candidate.patch").read_text()
        structural_name = "crates/hyperreal/src/computable/node/structural_analysis.rs"
        structural = (SOURCE / structural_name).read_text()
        begin = structural.index("    fn bound_from_approx(")
        end = structural.index("    fn rational_kernel_sign(", begin)
        assert structural[begin:end] == (path / "original-method.rs").read_text()
        assert structural[:begin] + structural[end:] == (path / "candidate" / structural_name).read_text()
        node_name = "crates/hyperreal/src/computable/node.rs"
        marker = 'include!("node/structural_analysis.rs");'
        assert (SOURCE / node_name).read_text().replace(marker, marker + '\ninclude!("node/approximation_bound.rs");') == (path / "candidate" / node_name).read_text()
    initial = (E / "approximation-bound-shared-proposal-v2458.rs").read_text()
    before = "assert((trailing_zeros == Some(magnitude_bits - 1))"
    after = "assert((trailing_zeros == Some((magnitude_bits - 1) as u64))"
    assert initial.count(before) == 1 and initial.replace(before, after) == proposal

    base_path = E / "approximation-bound-proof-v2461"
    base, base_protocol = read(base_path / "result.json"), read(base_path / "protocol.json")
    assert base["exit_code"] == 0 and base["verification_results"]["verified"] == 737
    assert base["verification_results"]["errors"] == 0 and base["verification_results"]["is-verifying-entire-crate"]
    assert base["protocol_sha256"] == sha(base_path / "protocol.json")
    assert base["stdout_sha256"] == sha(base_path / "stdout.json")
    assert base["stderr_sha256"] == sha(base_path / "stderr.log") and not (base_path / "stderr.log").read_bytes()
    assert sha(preview / "protocol.json") == base_protocol["preview_protocol_sha256"]
    old_proof = read((E / checkpoint["source_record"]).parent / "proof-reports/verus.json")
    new_proof = read(base_path / "stdout.json")
    old_names = {k.removeprefix("hyperreal_proofs::") for k in old_proof["func-details"] if k.startswith("hyperreal_proofs::")}
    new_names = {k.removeprefix("approximation_bound_preview::") for k in new_proof["func-details"] if k.startswith("approximation_bound_preview::")}
    assert new_names - old_names == {"computable_bounds::Computable::bound_from_approx", "computable_bounds::approximation_bound_expected"}
    assert old_names <= new_names and len(old_names) == 472
    failed = E / "approximation-bound-proof-v2459"
    failed_result = read(failed / "result.json")
    assert failed_result["exit_code"] == 1 and failed_result["verification_results"]["verified"] == 0
    assert failed_result["protocol_sha256"] == sha(failed / "protocol.json")
    assert failed_result["stdout_sha256"] == sha(failed / "stdout.json")
    assert failed_result["stderr_sha256"] == sha(failed / "stderr.log")

    frozen_path = E / "approximation-bound-guards-v2462"
    guards_path = E / "approximation-bound-guards-v2463"
    frozen = read(frozen_path / "protocol.json")
    early = read(frozen_path / "result.json")
    guards, guards_protocol = read(guards_path / "result.json"), read(guards_path / "protocol.json")
    assert not early["complete"] and len(early["runs"]) == 1 and early["runs"][0]["exit_code"] == 255
    assert not early["runs"][0]["verification_results"]
    logs(frozen_path, early["runs"][0], "baseline")
    assert guards["complete"] and len(guards["runs"]) == 13 and all(r["gate_passed"] for r in guards["runs"])
    assert guards["protocol_sha256"] == sha(guards_path / "protocol.json")
    assert guards_protocol["frozen_protocol_sha256"] == sha(frozen_path / "protocol.json")
    assert guards_protocol["incomplete_result_sha256"] == sha(frozen_path / "result.json")
    assert len(frozen["mutations"]) == 12
    baseline_files = files(preview)
    expected_sources = [("baseline", "baseline", proposal)]
    for name, kind, before, after in frozen["mutations"]:
        assert proposal.count(before) == 1
        expected_sources.append((name, kind, proposal.replace(before, after)))
    for old_job, job, row, (name, kind, text) in zip(frozen["jobs"], guards_protocol["jobs"], guards["runs"], expected_sources, strict=True):
        assert old_job["name"] == job["name"] == row["name"] == name
        assert old_job["kind"] == job["kind"] == row["kind"] == kind
        assert job["command"] == ["--verify-only-module" if a == "--verify-module" else a for a in old_job["command"]]
        assert job["command"][:-5] == base_protocol["command"][:-1]
        assert job["command"][-5:-1] == ["--verify-only-module", "computable_bounds", "--verify-function", "bound_from_approx"]
        work = Path(job["work"])
        assert job["work_files_sha256"] == old_job["work_files_sha256"] == files(work)
        assert sha(work / shared_name) == digest(text) == sha(frozen_path / "inputs" / (name + ".rs"))
        assert (work / "computable_bounds.rs").read_text() == (preview / "computable_bounds.rs").read_text().replace(str(preview / shared_name), str(work / shared_name))
        assert (work / "proof.rs").read_text() == (preview / "proof.rs").read_text().replace(str(preview / "computable_bounds.rs"), str(work / "computable_bounds.rs"))
        assert job["work_files_sha256"].keys() == baseline_files.keys()
        assert all(h == baseline_files[n] for n, h in job["work_files_sha256"].items() if n not in {shared_name, "proof.rs", "computable_bounds.rs"})
        logs(guards_path, row, name)
        if kind == "baseline":
            assert row["exit_code"] == 0 and row["verification_results"]["verified"] == 1 and row["verification_results"]["errors"] == 0
            assert row["informational_stderr_only"]
        else:
            assert row["exit_code"] == 1 and row["real_proof_failure"]
            assert row["verification_results"]["errors"] == 1 and not row["verification_results"]["encountered-vir-error"]

    cargo = E / "bigint-cargo-export-v2454"
    cargo_result, cargo_protocol = read(cargo / "result.json"), read(cargo / "protocol.json")
    dependencies = read(cargo / "dependency-reports/dependencies.json")
    fixture = Path(cargo_protocol["work"])
    assert cargo_result["gate_passed"] and cargo_result["compiler_result"]["warnings"] == 0
    assert all(sha(fixture / n) == h for n, h in cargo_protocol["work_files_sha256"].items())
    assert sha(fixture / "artifacts/num_bigint.vir") == cargo_result["export_sha256"] == base_protocol["exported_metadata_sha256"]
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    for package in dependencies["packages"]:
        directory = Path(package["manifest_path"]).parent
        assert all(sha(directory / n) == h for n, h in dependencies["sources"][package["id"]].items())
    tool = Path(base_protocol["command"][0]).parent
    assert all(sha(tool / n) == h for n, h in base_protocol["tool_sha256"].items())

    native_path = E / "approximation-bound-native-v2464"
    native, native_protocol = read(native_path / "result.json"), read(native_path / "protocol.json")
    assert native["complete"] and len(native["runs"]) == 3 and all(r["gate_passed"] for r in native["runs"])
    assert native["protocol_sha256"] == sha(native_path / "protocol.json")
    assert native["fresh_isolated_producer_corpus_executions"] == 1 and native["fresh_scalar_runtime_tests"] == 0
    for row in native["runs"]:
        assert row["exit_code"] == 0
        logs(native_path, row, row["name"], "log")
        assert not (native_path / (row["name"] + ".stderr.log")).read_bytes()
    binary = Path(native_protocol["commands"][-1]["command"][0])
    assert sha(binary) == native["binary_sha256"]
    for name, h in native_protocol["work_inputs_sha256"].items():
        assert sha(native_path / "inputs" / name) == sha(binary.parent / name) == h
    assert sha(E / "approximation-bound-native-harness-v2464.rs") == native_protocol["harness_sha256"]
    assert sha(E / "approximation-bound-instrumentation-v2464.rs") == native_protocol["instrumentation_sha256"]
    assert sha(preview / shared_name) == native_protocol["shared_source_sha256"]
    excerpts = {}
    for name, record in native_protocol["excerpts"].items():
        excerpts[name] = Path(record["source"]).read_text()[record["start"]:record["end"]]
        assert digest(excerpts[name]) == record["excerpt_sha256"]
    inputs = native_path / "inputs"
    assert (inputs / "bound.rs").read_text() == "type Precision = i32;\n" + excerpts["bound_enum"] + "\nimpl BoundInfo {\n" + excerpts["bound_constructor"] + "}\n"
    assert (inputs / "word.rs").read_text() == excerpts["word_function"]
    assert (inputs / "original.rs").read_text() == "impl Computable {\n" + (preview / "original-method.rs").read_text().removesuffix("\n") + "}\n"
    assert (inputs / "modules.rs").read_text().count('include!("' + str(preview / shared_name) + '");') == 2
    assert all(sha(Path(n)) == h for n, h in native_protocol["dependency_artifacts_sha256"].items())
    observations = dict(item.split("=", 1) for item in (native_path / "execute.stdout.log").read_text().split())
    assert all(observations[k] == v for k, v in native_protocol["expected_counts"].items())
    assert observations == native["runs"][-1]["observations"]
    assert observations["diagnostic_checksum"] == "83aad051019b1358"
    assert catalog.source_hashes(SOURCE, original_sources) == sources
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    assert subprocess.check_output(["git", "show", "HEAD:docs/properties-status.json"], cwd=ROOT) == (ROOT / "docs/properties-status.json").read_bytes()
    result_paths = [failed / "result.json", base_path / "result.json", frozen_path / "result.json",
        guards_path / "result.json", native_path / "result.json"]
    result = {
        "status": "The shared approximation-bound producer passes conditional proof, twelve mutation controls and an independent native oracle in isolated previews",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint": checkpoint_path.name,
        "retained_checkpoint_sha256": sha(checkpoint_path), "retained_source_record_sha256": checkpoint["source_record_sha256"],
        "previous_support_frontier_sha256": sha(previous_frontier), "previous_support_artifacts_rechecked": support_count,
        "source_files_rechecked": len(sources), "main_files_rechecked": len(main_guard),
        "candidate_sources_changed": False, "source_catalog_unchanged": True, "main_runtime_admitted": False,
        "retained_verified_units": 736, "new_retained_proof_units": 0, "whole_crate_obligation_groups_open": 13,
        "preview_full_verified_units": 737, "preview_new_producer_bodies": 1, "preview_new_specifications": 1,
        "existing_scalar_proof_functions_and_views_preserved": len(old_names),
        "producer_positive_baselines": 1, "producer_negative_controls_reconstructed": 12,
        "producer_wrong_behavior_controls": 7, "producer_weakened_output_controls": 3, "producer_missing_premise_controls": 2,
        "actual_cargo_dependency_export_units_including_existing_models": 453,
        "native_corpus": observations, "fresh_isolated_producer_corpus_executions": 1, "fresh_scalar_runtime_tests": 0,
        "shared_proof_and_native_source_sha256": sha(preview / shared_name), "native_source_excerpts_reconstructed": True,
        "result_records_sha256": {str(p.relative_to(E)): sha(p) for p in result_paths},
        "failures_preserved": ["approximation-bound-proof-v2459/result.json", "approximation-bound-guards-v2462/result.json"],
        "corrections": ["approximation-bound-correction-v2460.json", "approximation-bound-guards-v2463/protocol.json"],
        "scope": "The prepared three-file extraction shares one associated-function body between the node facade preview and full scalar proof wrapper. Actual Cargo-imported BigInt sign/magnitude and BigUint bit metadata connect to the retained interval lemmas. Under explicit normalized representation and u64 count-range premises, the proof fixes every returned bound field and establishes soundness for every real satisfying the approximation's one-unit error contract. Returning Unknown or discarding an available exact exponent is rejected. Independent native endpoint arithmetic agrees with the original and proposed producers; separate counted adapters preserve metadata call selection, including the lazy zero scan. This does not prove approximation truth, caller premises, whole-node representation, atomic publication or scalar runtime admission. Imported and local verified counts are separate and not added to retained coverage.",
        "remaining": [
            "Prepare repository-appropriate dependency model placement and Cargo/Verus import wiring for the complete candidate, including publication, MSRV and supported targets. Production/proof dependencies must resolve to the same physical package; the preview export cannot substitute silently for the retained registry dependency.",
            "Discharge or explicitly enforce normalization, sign consistency and aggregate count-range premises at actual constructors, arithmetic and producer callers, and establish the original approximation's truth independently of its cached bound.",
            "Qualify any candidate dependency/runtime change under the original correctness, resource, cost, memory and size gates before retention; the focused native producer corpus supplies no substitute timing or full scalar gate.",
            "Continue independent existence, atomic publication and exact-operation resource work. Both baseline CPU failures, thirteen whole-crate groups, all 39 properties, 14 targets, crate future work, main/native/full-board admission and complete pre-push verification remain open."
        ],
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True
    }
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"full_preview_units": 737, "producer_negative_controls": 12,
        "native_queries": 2775612, "source_unchanged": True, "frontier_sha256": sha(out)}))


if __name__ == "__main__":
    main()
