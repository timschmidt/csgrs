"""Audit the guarded shared-source BigUint preview and its native qualification."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import tarfile

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def audit_guards(version):
    directory = E / f"biguint-metadata-guards-v{version}"
    protocol = read(directory / "protocol.json")
    result = read(directory / "result.json")
    preview = E / protocol["preview"]
    assert result["complete"] and result["protocol_sha256"] == sha(directory / "protocol.json")
    assert len(protocol["runs"]) == len(result["runs"]) == 21
    assert all(result[k] for k in ["source_unchanged", "main_unchanged", "preview_unchanged", "tooling_unchanged"])
    assert all(sha(preview / n) == h for n, h in protocol["preview_files_sha256"].items())
    for row, observed in zip(protocol["runs"], result["runs"]):
        case = directory / row["directory"]
        assert observed == read(case / "result.json") and observed["gate_passed"]
        assert all(sha(case / n) == h for n, h in row["files_sha256"].items())
        assert observed["stdout_sha256"] == sha(case / "stdout.json")
        assert observed["stderr_sha256"] == sha(case / "stderr.log")
        entry = (preview / "proof.rs").read_text()
        mutation = row["mutation"]
        if mutation:
            original_path = preview / mutation["file"]
            original = original_path.read_text()
            before, after = mutation["before"], mutation["after"]
            if mutation["function"] == "WIDTH":
                assert original.count(before) == 1
                changed = original.replace(before, after)
            else:
                anchor = "fn " + mutation["function"] + "("
                assert original.count(anchor) == 1
                start = original.index(anchor)
                indent = "    " if original_path.name == "metadata_impl.rs" else ""
                end = original.index("\n" + indent + "}\n", start) + len(indent) + 3
                assert original[start:end].count(before) == 1
                changed = original[:start] + original[start:end].replace(before, after) + original[end:]
            changed_path = case / original_path.name
            assert changed_path.read_text() == changed
            start = entry.index(f"mod biguint_u{mutation['width']} {{")
            end = entry.index("\n}\n", start) + 3
            assert entry[start:end].count(str(original_path)) == 1
            entry = entry[:start] + entry[start:end].replace(str(original_path), str(changed_path)) + entry[end:]
            assert observed["exit_code"] != 0 and observed["verification_results"]["errors"] > 0
            assert observed["verification_results"]["verified"] > 0
            assert not observed["verification_results"]["encountered-vir-error"]
        else:
            assert observed["exit_code"] == 0 and observed["verification_results"]["verified"] == 462
        assert (case / "probe.rs").read_text() == entry
    return {"protocol_sha256": sha(directory / "protocol.json"), "result_sha256": sha(directory / "result.json"),
            "negative_controls": 20, "positive_baselines": 1, "mutated_inputs_reconstructed": True}


def main():
    out = E / "biguint-integration-frontier-v2423.json"
    assert not out.exists()
    checkpoint_path = E / "typed-api-prototype-v2386-state.json"
    assert sha(checkpoint_path) == "6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    previous = E / "storage-contract-frontier-v2397.json"
    assert sha(previous) == "2fbbaf6524f8f6c0c0f3a70f895f43deaec335a2e89814419127b0a5cbdfdf2d"
    partial = read(E / "biguint-metadata-preparation-failure-v2399.json")
    assert all(sha(E / n) == h for n, h in partial["failed_partial_files_sha256"].items())
    previews = {}
    archive = Path("/tmp/csgrs-properties-cargo/registry/cache/index.crates.io-1949cf8c6b5b557f/num-bigint-0.4.6.crate")
    assert sha(archive) == "a5e44f723f1133c9deac646763579fdb3ac745e418f2a7af9cd0c431da1f20b9"
    with tarfile.open(archive) as source_archive:
        originals = {m.name.removeprefix("num-bigint-0.4.6/"): source_archive.extractfile(m).read()
                     for m in source_archive.getmembers()}
    original_hashes = {n: hashlib.sha256(data).hexdigest() for n, data in originals.items()}
    assert len(originals) == 51
    for version in [2400, 2402, 2404, 2406, 2413]:
        preview = E / f"biguint-metadata-candidate-v{version}"
        protocol = read(preview / "protocol.json")
        assert protocol["upstream_files_sha256"] == original_hashes
        assert all(sha(preview / n) == h for n, h in protocol["preview_files_sha256"].items())
        assert protocol["upstream_patch_sha256"] == sha(preview / "upstream.patch")
        vendor = preview / "vendor/num-bigint"
        changed = [n for n, h in original_hashes.items() if sha(vendor / n) != h]
        assert changed == protocol["modified_upstream_files"]
        patch = "".join("".join(difflib.unified_diff(originals[n].decode().splitlines(True),
            (vendor / n).read_text().splitlines(True), fromfile=f"upstream/{n}", tofile=f"candidate/{n}")) for n in changed)
        assert (preview / "upstream.patch").read_text() == patch
        # The full allocation policy is deliberately left unchanged and unproved.
        original = originals["src/biguint.rs"].decode()
        modified = (vendor / "src/biguint.rs").read_text()
        start = original.index("    fn normalize(&mut self) {")
        end = original.index("    /// Returns a normalized", start)
        old_normalize = original[start:end]
        start = modified.index("    fn normalize(&mut self) {")
        end = modified.index("    /// Returns a normalized", start)
        new_normalize = modified[start:end]
        old_trim = "            let len = self.data.iter().rposition(|&d| d != 0).map_or(0, |i| i + 1);\n            self.data.truncate(len);"
        assert new_normalize == old_normalize.replace(old_trim, "            metadata::trim_zero_suffix(&mut self.data);")
        assert sha(vendor / "LICENSE-MIT") == original_hashes["LICENSE-MIT"]
        assert sha(vendor / "LICENSE-APACHE") == original_hashes["LICENSE-APACHE"]
        previews[preview.name] = {"protocol_sha256": sha(preview / "protocol.json"),
            "modified_upstream_files": changed, "allocation_policy_unchanged": True,
            "licenses_preserved": True, "shared_source_files": protocol["added_files"]}
    proofs = {}
    for version, expected_exit, diagnostic in [
        (2401, 1, "Self is not supported in reveal/hide"),
        (2403, 1, "which is ignored because"),
        (2405, 1, "cannot read const with mode exec"),
        (2407, 0, ""), (2414, 0, ""),
    ]:
        phase = E / f"biguint-metadata-proof-v{version}"
        result = read(phase / "result.json")
        protocol = read(phase / "protocol.json")
        assert result["exit_code"] == expected_exit
        assert all(result[k] for k in ["source_unchanged", "main_unchanged", "preview_unchanged", "tooling_unchanged"])
        assert diagnostic in (phase / "stderr.log").read_text()
        for key, name in [("protocol_sha256", "protocol.json"), ("stdout_sha256", "stdout.json"), ("stderr_sha256", "stderr.log")]:
            assert result[key] == sha(phase / name)
        assert all(sha(E / protocol["preview"] / n) == h for n, h in protocol["preview_files_sha256"].items())
        if expected_exit == 0:
            assert result["verification_results"]["verified"] == 462 and result["verification_results"]["errors"] == 0
            assert not (phase / "stderr.log").read_text()
        proofs[phase.name] = {"result_sha256": sha(phase / "result.json"), "exit_code": expected_exit,
            "verification_results": result["verification_results"], "diagnostic": diagnostic}
    guard_runs = {f"biguint-metadata-guards-v{version}": audit_guards(version) for version in [2408, 2415]}
    failed_resolution = E / "biguint-runtime-preparation-v2409"
    assert not read(failed_resolution / "resolution-result.json")["complete"]
    assert "multiple workspace roots found" in (failed_resolution / "harness.stderr.log").read_text()
    failed_clippy = E / "biguint-runtime-v2412-vendor-clippy"
    assert read(failed_clippy / "result.json")["exit_code"] == 101
    phases = {}
    for version, kind in [(2417, "vendor-clippy"), (2418, "vendor-tests"), (2419, "vendor-no-std"),
                          (2420, "harness-debug"), (2421, "harness-release"), (2422, "shared-fmt")]:
        phase = E / f"biguint-runtime-v{version}-{kind}"
        result = read(phase / "result.json")
        assert result["exit_code"] == 0 and not result["warning_lines"]
        assert all(result[k] for k in ["source_unchanged", "main_unchanged", "work_unchanged", "preview_unchanged", "dependencies_unchanged"])
        for key, name in [("protocol_sha256", "protocol.json"), ("stdout_sha256", "stdout.log"), ("stderr_sha256", "stderr.log")]:
            assert result[key] == sha(phase / name)
        phases[phase.name] = {"result_sha256": sha(phase / "result.json"), "result": result}
    debug = phases["biguint-runtime-v2420-harness-debug"]["result"]
    release = phases["biguint-runtime-v2421-harness-release"]["result"]
    assert debug["harness"] == release["harness"]
    assert int(debug["harness"]["cases"]) == 67137
    assert int(debug["harness"]["helper_width_checks"]) == 134274
    assert all(sha(Path(r["binary"])) == r["binary_sha256"] for r in [debug, release])
    prep_path = E / "biguint-runtime-preparation-v2416"
    prepared = read(prep_path / "result.json")
    work = Path(prepared["work"])
    assert all(sha(work / n) == h for n, h in prepared["work_files_sha256"].items())
    assert all(sha(Path(n).parent / child) == h for n, children in prepared["dependency_sources_sha256"].items() for child, h in children.items())
    # These runtime inputs must be the exact amended preview sources, even
    # though their Cargo roots and lockfiles are separate qualification inputs.
    final_preview = E / "biguint-metadata-candidate-v2413"
    for path in (final_preview / "vendor/num-bigint").rglob("*"):
        if path.is_file():
            assert sha(work / path.relative_to(final_preview)) == sha(path)
    test_rows = phases["biguint-runtime-v2418-vendor-tests"]["result"]["test_summaries"]
    assert test_rows and all(row[1] == 0 for row in test_rows)
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    data = {
        "status": "Shared-source BigUint preview verified, guarded and checked in ordinary Rust; not applied to SOURCE",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint": checkpoint_path.name,
        "retained_checkpoint_sha256": sha(checkpoint_path), "retained_source_record_sha256": checkpoint["source_record_sha256"],
        "previous_frontier_sha256": sha(previous), "source_files_verified": len(sources), "main_files_verified": len(guard),
        "candidate_sources_changed": False, "main_sources_changed": False, "main_runtime_admitted": False,
        "retained_verified_units": 736, "retained_production_contracts": 98, "retained_model_theorems": 178,
        "retained_negative_controls": 306, "retained_positive_baselines": 7, "whole_crate_obligation_groups_open": 13,
        "new_retained_proof_units": 0, "new_retained_model_theorems": 0, "new_retained_production_contracts": 0,
        "final_preview": final_preview.name, "previews": previews, "proof_phases": proofs, "guard_runs": guard_runs,
        "final_preview_verified_units_including_existing_imports": 462,
        "final_preview_executable_functions": 18, "final_preview_executable_width_constants": 2,
        "final_preview_negative_controls": 20, "final_preview_positive_baselines": 1,
        "runtime_preparation_sha256": sha(prep_path / "result.json"), "native_phases": phases,
        "native_upstream_test_summaries": test_rows, "native_upstream_tests_passed": sum(row[0] for row in test_rows),
        "native_corpus_per_build": debug["harness"], "native_corpus_builds": ["debug", "release"],
        "native_biguint_host": prepared["host"], "native_shared_helper_widths": [32, 64],
        "failed_formatter_preparation_sha256": sha(E / "biguint-metadata-preparation-failure-v2399.json"),
        "failed_workspace_resolution_sha256": sha(failed_resolution / "resolution-result.json"),
        "failed_strict_clippy_sha256": sha(failed_clippy / "result.json"),
        "fresh_scalar_runtime_commands": 0, "fresh_fixed_baseline_cost_commands": 0,
        "findings": [
            "The production preview and Verus compile the same private Vec-backed BigUint declaration, three metadata method bodies and six helpers for each native width. Public ghost views are defined from the actual field. No layout cast, shadow executable implementation or local trusted metadata assertion is used.",
            "The final proof verifies under explicit normalized-storage and u64 aggregate-count premises. Twenty actual executable mutations are rejected with proof errors, including width constants, scans, truncation, count arithmetic and each public method. Both the initial and amended guard runs are retained; only the amended run qualifies the final preview.",
            "The original normalize guard and exact capacity/shrink_to_fit block are unchanged. Existing licenses and archive identities are retained. Scoped legacy-intrinsic lint allowances and two scratch-vector clear calls are explicitly disclosed; the remaining dependency arithmetic and allocator implementation are not proved by this milestone.",
            "Ordinary Cargo resolves the patched path dependency and locked registry original as distinct packages. Strict library Clippy, original library/integration tests, no_std checking, shared-file formatting and the independent bit-vector/differential corpus pass. Debug and release corpus summaries match.",
            "SOURCE, the scalar producer's lazy evaluation order, main files and all retained coverage counts remain unchanged. This isolated native corpus is not scalar fixed-baseline correctness, cost, resource, memory or size admission."
        ],
        "next_action": "Integrate the verified shared files into a frozen SOURCE candidate only after extending snapshots to every vendor file and binding the proof/production Cargo path package identities. Preserve actual dependency implementation provenance, discharge the storage normalization/count-range and BigInt/producer caller premises, and run the original fixed-baseline scalar correctness/cost/resource/memory/size gates before retaining a runtime change. Standalone publication and old-compiler/cross-target compatibility remain explicit open work.",
        "all_39_properties_14_targets_and_crate_future_work_in_scope": True,
        "limits": "Retained runtime v2335 and proof checkpoint v2386 remain authoritative. All thirteen whole-crate groups remain open. Preserve every prior failure, original resource cap, pre-Verus baseline and all 49 original worktree entries. No property, target, backend, whole-crate or main/native/full-board completion is claimed.",
    }
    out.write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"final_preview_verified_units": 462, "preview_executable_functions": 18,
        "preview_negative_controls": 20, "native_upstream_tests_passed": data["native_upstream_tests_passed"],
        "native_corpus": data["native_corpus_per_build"], "source_changed": False, "retained_verified_units": 736}))


if __name__ == "__main__":
    main()
