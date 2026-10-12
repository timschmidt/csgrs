"""Exclude inherited generated manifests before locally repackaging the unchanged source."""
import difflib
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import time
import tomllib

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
INITIAL = E / "bigint-portable-validation-v2468"
PREVIOUS = E / "bigint-portable-validation-v2471"
PREPARED = E / "bigint-portable-package-v2466"
CARGO = E / "bigint-portable-cargo-v2467"
OUT = E / "bigint-portable-repackage-v2473"
WORK = Path("/tmp/csgrs-bigint-portable-repackage-v2473")
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob("*")) if p.is_file()}


def main():
    assert not OUT.exists() and not WORK.exists()
    previous = read(PREVIOUS / "result.json")
    initial = read(INITIAL / "protocol.json")
    assert not previous["complete"] and len(previous["runs"]) == 2
    assert previous["runs"][0]["gate_passed"] and previous["runs"][0]["name"] == "clippy-all-features"
    assert previous["runs"][1]["exit_code"] == 101 and previous["runs"][1]["name"] == "package"
    assert (PREVIOUS / "package.stderr.log").read_text() == "error: invalid inclusion of reserved file name Cargo.toml.orig in package source\n"
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    prepared = read(PREPARED / "protocol.json")
    portable = Path(prepared["vendor"])
    cargo_protocol = read(CARGO / "protocol.json")
    dependencies = read(CARGO / "dependency-reports/dependencies.json")
    tool = Path(read(CARGO / "wrapper-config.json")["verus"]).parent
    old_work = Path(initial["work"])
    old_vendor = old_work / "num-bigint"
    old_inputs = read(INITIAL / "locked-inputs.json")
    assert files(old_vendor) == old_inputs
    assert files(portable) == prepared["vendor_files_sha256"]
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    assert all(sha(tool / n) == h for n, h in cargo_protocol["tool_sha256"].items())
    OUT.mkdir()
    WORK.mkdir()
    vendor = WORK / "num-bigint"
    shutil.copytree(old_vendor, vendor)
    before = (vendor / "Cargo.toml").read_text()
    marker = 'exclude = [\n'
    assert before.count(marker) == 1
    after = before.replace(marker, marker + '    "/Cargo.toml.orig",\n    "/.cargo_vcs_info.json",\n')
    old_manifest, new_manifest = tomllib.loads(before), tomllib.loads(after)
    assert new_manifest["package"]["exclude"][:2] == ["/Cargo.toml.orig", "/.cargo_vcs_info.json"]
    new_manifest["package"]["exclude"] = new_manifest["package"]["exclude"][2:]
    assert old_manifest == new_manifest
    (vendor / "Cargo.toml").write_text(after)
    (OUT / "Cargo.toml").write_text(after)
    (OUT / "packaging-manifest.patch").write_text("".join(difflib.unified_diff(
        before.splitlines(True), after.splitlines(True), fromfile="a/Cargo.toml", tofile="b/Cargo.toml", n=0)))
    current_inputs = files(vendor)
    assert current_inputs.keys() == old_inputs.keys()
    assert [n for n, h in current_inputs.items() if old_inputs[n] != h] == ["Cargo.toml"]
    archive = WORK / "target/package/num-bigint-0.4.6.crate"
    unpacked = WORK / "unpacked/num-bigint-0.4.6"
    artifacts = WORK / "proof-artifacts"
    artifacts.mkdir()
    environment = os.environ.copy()
    for name in initial["removed_environment_keys"]:
        environment.pop(name, None)
    overrides = initial["environment_overrides"] | {"CARGO_TARGET_DIR": str(WORK / "target")}
    environment.update(overrides)
    jobs = [{"name": job["name"], "command": [arg.replace(str(old_work), str(WORK)) for arg in job["command"]]}
        for job in initial["jobs"][6:]]
    assert [job["name"] for job in jobs] == ["package", "packaged-build", "packaged-proof"]
    protocol = {"status": "Frozen packaging-only correction before three unchanged relocated checks",
        "runner_sha256": sha(Path(__file__)), "initial_protocol_sha256": sha(INITIAL / "protocol.json"),
        "previous_result_sha256": sha(PREVIOUS / "result.json"), "old_locked_inputs_sha256": sha(INITIAL / "locked-inputs.json"),
        "work": str(WORK), "vendor_files_sha256": current_inputs, "jobs": jobs,
        "environment_overrides": overrides, "packaging_manifest_sha256": sha(OUT / "Cargo.toml"),
        "packaging_patch_sha256": sha(OUT / "packaging-manifest.patch"),
        "correction": "The upstream archive contains Cargo.toml.orig and inherited VCS metadata. Keep both files in the source/provenance copy but exclude them from Cargo's package input; Cargo regenerates its own normalized/original manifests. Clone the complete prior locked source first, changing only the package exclude list. No dependency, feature, Rust declaration, proof, or lockfile changes. The previous reserved-file packaging failure remains failed. Native compile/Clippy and Cargo-export evidence refer to the prior source; their executable/model inputs are identical, not fresh executions here.",
        "scope": initial["scope"]}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    unpacked_inputs = None
    rows = []
    for job in jobs:
        start = time.monotonic()
        run = subprocess.run(job["command"], cwd=WORK, env=environment, capture_output=True, text=True)
        stdout, stderr = OUT / (job["name"] + ".stdout.log"), OUT / (job["name"] + ".stderr.log")
        stdout.write_text(run.stdout)
        stderr.write_text(run.stderr)
        warnings = len(re.findall(r"(?m)^warning(?:\[|:)", run.stderr))
        verification = None
        if job["name"] == "packaged-proof":
            try:
                verification = json.loads(run.stdout).get("verification-results", {})
                diagnostics = [json.loads(line) for line in run.stderr.splitlines()]
                warnings += sum(item.get("level") == "warning" for item in diagnostics)
            except json.JSONDecodeError:
                verification = {}
        passed = run.returncode == 0 and warnings == 0
        if verification is not None:
            passed = passed and verification.get("verified") == 453 and verification.get("errors") == 0
        if job["name"] == "package" and passed:
            assert archive.is_file()
            (OUT / archive.name).write_bytes(archive.read_bytes())
            with tarfile.open(archive) as package:
                entries = package.getmembers()
                assert all(entry.isfile() or entry.isdir() for entry in entries)
                assert len({entry.name for entry in entries}) == len(entries)
                package.extractall(WORK / "unpacked", filter="data")
            unpacked_inputs = files(unpacked)
            expected = set(current_inputs) - {".cargo_vcs_info.json", "Cargo.toml.orig"}
            assert expected <= unpacked_inputs.keys(), sorted(expected - unpacked_inputs.keys())
            permitted = {"Cargo.toml", "Cargo.toml.orig", "Cargo.lock", ".cargo_vcs_info.json"}
            assert all(unpacked_inputs[n] == h for n, h in current_inputs.items() if n not in permitted)
            assert set(unpacked_inputs) - set(current_inputs) <= permitted
            assert (unpacked / "Cargo.toml.orig").read_text() == after
            for relative in re.findall(r'#\[path = "([^"]+)"\]', (unpacked / "src/lib.rs").read_text()):
                resolved = (unpacked / "src" / relative).resolve()
                assert resolved.is_relative_to(unpacked.resolve()) and resolved.is_file()
            package_record = {"archive_sha256": sha(archive), "unpacked": str(unpacked),
                "unpacked_files_sha256": unpacked_inputs,
                "manifest_normalization_paths": [n for n, h in unpacked_inputs.items() if current_inputs.get(n) != h],
                "source_and_model_bytes_preserved": True, "module_paths_stay_inside_package": True,
                "generated_original_manifest_matches_corrected_input": True}
            (OUT / "package-record.json").write_text(json.dumps(package_record, indent=2) + "\n")
        row = {"name": job["name"], "exit_code": run.returncode, "seconds": time.monotonic() - start,
            "warnings": warnings, "verification_results": verification, "gate_passed": passed,
            "stdout_sha256": sha(stdout), "stderr_sha256": sha(stderr)}
        rows.append(row)
        assert files(vendor) == current_inputs and files(old_vendor) == old_inputs
        if unpacked_inputs is not None:
            assert files(unpacked) == unpacked_inputs
        assert files(portable) == prepared["vendor_files_sha256"]
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in main_guard.items())
        assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
        assert all(sha(tool / n) == h for n, h in cargo_protocol["tool_sha256"].items())
        result = {"protocol_sha256": sha(OUT / "protocol.json"), "runs": rows,
            "complete": len(rows) == len(jobs) and all(r["gate_passed"] for r in rows),
            "source_unchanged": True, "main_unchanged": True, "portable_source_unchanged": True,
            "old_packaging_inputs_unchanged": True, "dependency_artifacts_unchanged": True, "tooling_unchanged": True,
            "package_record_sha256": sha(OUT / "package-record.json") if (OUT / "package-record.json").exists() else None,
            "export_sha256": sha(artifacts / "num_bigint.vir") if (artifacts / "num_bigint.vir").exists() else None,
            "retained_coverage_changed": False, "runtime_admitted": False}
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        if not passed:
            break


if __name__ == "__main__":
    main()
