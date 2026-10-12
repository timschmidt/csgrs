"""Audit Cargo's omitted VCS files, then build and prove the existing unpacked archive."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tarfile
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PREVIOUS = E / "bigint-portable-repackage-v2473"
INITIAL = E / "bigint-portable-validation-v2468"
CARGO = E / "bigint-portable-cargo-v2467"
OUT = E / "bigint-portable-repackage-v2474"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob("*")) if p.is_file()}


def main():
    assert not OUT.exists() and not (PREVIOUS / "result.json").exists()
    previous = read(PREVIOUS / "protocol.json")
    initial = read(INITIAL / "protocol.json")
    work = Path(previous["work"])
    vendor = work / "num-bigint"
    unpacked = work / "unpacked/num-bigint-0.4.6"
    archive = work / "target/package/num-bigint-0.4.6.crate"
    assert sha(archive) == sha(PREVIOUS / archive.name)
    inputs = previous["vendor_files_sha256"]
    assert files(vendor) == inputs
    extracted = files(unpacked)
    assert inputs.keys() - extracted.keys() == {".cargo_vcs_info.json", ".gitignore"}
    assert not extracted.keys() - inputs.keys()
    assert {n for n in inputs.keys() & extracted.keys() if inputs[n] != extracted[n]} == {"Cargo.toml", "Cargo.toml.orig"}
    assert (unpacked / "Cargo.toml.orig").read_bytes() == (vendor / "Cargo.toml").read_bytes() == (PREVIOUS / "Cargo.toml").read_bytes()
    assert "    Packaged 73 files," in (PREVIOUS / "package.stderr.log").read_text()
    assert not re.search(r"(?m)^(?:warning|error)(?:\[|:)", (PREVIOUS / "package.stderr.log").read_text())
    with tarfile.open(archive) as package:
        members = package.getmembers()
        assert all(m.isfile() or m.isdir() for m in members)
        archive_sources = {str(Path(m.name).relative_to("num-bigint-0.4.6")):
            hashlib.sha256(package.extractfile(m).read()).hexdigest() for m in members if m.isfile()}
    assert archive_sources == extracted and len(extracted) == 73
    for relative in re.findall(r'#\[path = "([^"]+)"\]', (unpacked / "src/lib.rs").read_text()):
        path = (unpacked / "src" / relative).resolve()
        assert path.is_relative_to(unpacked.resolve()) and path.is_file()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    cargo_protocol = read(CARGO / "protocol.json")
    dependencies = read(CARGO / "dependency-reports/dependencies.json")
    tool = Path(read(CARGO / "wrapper-config.json")["verus"]).parent
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    assert all(sha(tool / n) == h for n, h in cargo_protocol["tool_sha256"].items())
    OUT.mkdir()
    jobs = previous["jobs"][1:]
    assert [j["name"] for j in jobs] == ["packaged-build", "packaged-proof"]
    package_record = {"archive_phase": PREVIOUS.name, "archive_sha256": sha(archive),
        "original_package_stdout_sha256": sha(PREVIOUS / "package.stdout.log"),
        "original_package_stderr_sha256": sha(PREVIOUS / "package.stderr.log"),
        "original_process_exit_code_separately_recorded": False,
        "original_success_branch_reached": True,
        "unpacked": str(unpacked), "unpacked_files_sha256": extracted,
        "source_and_model_bytes_preserved": True, "module_paths_stay_inside_package": True,
        "generated_original_manifest_matches_corrected_input": True,
        "omitted_input_files": [".cargo_vcs_info.json", ".gitignore"],
        "normalized_manifest_files": ["Cargo.toml", "Cargo.toml.orig"],
        "integrity_gate_passed": True}
    (OUT / "package-record.json").write_text(json.dumps(package_record, indent=2) + "\n")
    protocol = {"status": "Frozen corrected archive audit before two previously unattempted commands",
        "runner_sha256": sha(Path(__file__)), "previous_protocol_sha256": sha(PREVIOUS / "protocol.json"),
        "package_record_sha256": sha(OUT / "package-record.json"), "jobs": jobs,
        "correction": "Cargo created the archive and the runner reached its post-success integrity branch, but that reader incorrectly required Cargo to package .gitignore. The first reader stops at that assertion before writing a result; preserve it. Audit the existing archive byte-for-byte, allowing only omitted .gitignore/inherited VCS metadata and Cargo's two normalized manifests. Every Rust, model, license, provenance and lock byte remains required. Do not rerun the completed package operation or invent a separately recorded exit code. Run only the previously unattempted packaged build and proof commands.",
        "environment_overrides": previous["environment_overrides"], "scope": previous["scope"]}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    environment = os.environ.copy()
    for name in initial["removed_environment_keys"]:
        environment.pop(name, None)
    environment.update(previous["environment_overrides"])
    rows = []
    for job in jobs:
        start = time.monotonic()
        run = subprocess.run(job["command"], cwd=work, env=environment, capture_output=True, text=True)
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
        row = {"name": job["name"], "exit_code": run.returncode, "seconds": time.monotonic() - start,
            "warnings": warnings, "verification_results": verification, "gate_passed": passed,
            "stdout_sha256": sha(stdout), "stderr_sha256": sha(stderr)}
        rows.append(row)
        assert files(vendor) == inputs and files(unpacked) == extracted
        assert sha(archive) == package_record["archive_sha256"]
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in main_guard.items())
        assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
        assert all(sha(tool / n) == h for n, h in cargo_protocol["tool_sha256"].items())
        exported = work / "proof-artifacts/num_bigint.vir"
        result = {"protocol_sha256": sha(OUT / "protocol.json"), "runs": rows,
            "complete": len(rows) == len(jobs) and all(r["gate_passed"] for r in rows),
            "reused_archive_integrity_gate_passed": True, "source_unchanged": True, "main_unchanged": True,
            "packaging_inputs_unchanged": True, "dependency_artifacts_unchanged": True, "tooling_unchanged": True,
            "package_record_sha256": sha(OUT / "package-record.json"),
            "export_sha256": sha(exported) if exported.exists() else None,
            "retained_coverage_changed": False, "runtime_admitted": False}
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        if not passed:
            break


if __name__ == "__main__":
    main()
