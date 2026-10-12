"""Resume the unchanged packaging checks after fetching missing locked dependencies."""
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
PREVIOUS = E / "bigint-portable-validation-v2468"
PREPARED = E / "bigint-portable-package-v2466"
CARGO = E / "bigint-portable-cargo-v2467"
FETCH = E / "bigint-portable-fetch-v2470"
OUT = E / "bigint-portable-validation-v2471"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob("*")) if p.is_file()}


def main():
    assert not OUT.exists()
    previous, initial = read(PREVIOUS / "result.json"), read(PREVIOUS / "protocol.json")
    assert not previous["complete"] and len(previous["runs"]) == 6
    assert all(r["gate_passed"] for r in previous["runs"][:5])
    assert previous["runs"][-1]["name"] == "clippy-all-features" and previous["runs"][-1]["exit_code"] == 101
    fetch = read(FETCH / "result.json")
    assert fetch["gate_passed"] and fetch["locked_inputs_unchanged"]
    assert previous["protocol_sha256"] == sha(PREVIOUS / "protocol.json")
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
    work = Path(initial["work"])
    vendor = work / "num-bigint"
    current_inputs = read(PREVIOUS / "locked-inputs.json")
    assert files(vendor) == current_inputs
    assert files(portable) == prepared["vendor_files_sha256"]
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    assert all(sha(tool / n) == h for n, h in cargo_protocol["tool_sha256"].items())
    archive = work / "target/package/num-bigint-0.4.6.crate"
    unpacked = work / "unpacked/num-bigint-0.4.6"
    assert not archive.exists() and not unpacked.exists()
    environment = os.environ.copy()
    for name in initial["removed_environment_keys"]:
        environment.pop(name, None)
    environment.update(initial["environment_overrides"])
    jobs = initial["jobs"][5:]
    assert [job["name"] for job in jobs] == ["clippy-all-features", "package", "packaged-build", "packaged-proof"]
    for row in previous["runs"]:
        assert row["stdout_sha256"] == sha(PREVIOUS / (row["name"] + ".stdout.log"))
        assert row["stderr_sha256"] == sha(PREVIOUS / (row["name"] + ".stderr.log"))
    OUT.mkdir()
    protocol = {"status": "Frozen before resuming four unchanged commands after dependency cache preparation",
        "runner_sha256": sha(Path(__file__)), "previous_protocol_sha256": sha(PREVIOUS / "protocol.json"),
        "previous_result_sha256": sha(PREVIOUS / "result.json"), "fetch_result_sha256": sha(FETCH / "result.json"),
        "locked_inputs_sha256": sha(PREVIOUS / "locked-inputs.json"), "jobs": jobs,
        "environment_overrides": initial["environment_overrides"],
        "reused_successful_phases": [row["name"] for row in previous["runs"][:5]],
        "correction": "Populate missing locked optional dependencies, then rerun only the command that failed before compilation and run the three unattempted commands. All commands and input source/lock bytes remain exactly as frozen in v2468. Its failed Clippy attempt remains failed; its five successful phases are reused, not rerun or counted as fresh.",
        "scope": initial["scope"], "package_preparation": initial["package_preparation"]}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    unpacked_inputs = None
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
        if job["name"] == "package" and passed:
            assert archive.is_file()
            (OUT / archive.name).write_bytes(archive.read_bytes())
            with tarfile.open(archive) as package:
                entries = package.getmembers()
                assert all(entry.isfile() or entry.isdir() for entry in entries)
                assert len({entry.name for entry in entries}) == len(entries)
                package.extractall(work / "unpacked", filter="data")
            unpacked_inputs = files(unpacked)
            expected = set(prepared["vendor_files_sha256"]) - {".cargo_vcs_info.json"}
            assert expected <= unpacked_inputs.keys(), sorted(expected - unpacked_inputs.keys())
            permitted = {"Cargo.toml", "Cargo.toml.orig", "Cargo.lock", ".cargo_vcs_info.json"}
            assert all(unpacked_inputs[n] == h for n, h in current_inputs.items() if n not in permitted)
            assert set(unpacked_inputs) - set(current_inputs) <= permitted
            for relative in re.findall(r'#\[path = "([^"]+)"\]', (unpacked / "src/lib.rs").read_text()):
                resolved = (unpacked / "src" / relative).resolve()
                assert resolved.is_relative_to(unpacked.resolve()) and resolved.is_file()
            package_record = {"archive_sha256": sha(archive), "unpacked": str(unpacked),
                "unpacked_files_sha256": unpacked_inputs,
                "manifest_normalization_paths": [n for n, h in unpacked_inputs.items() if current_inputs.get(n) != h],
                "source_and_model_bytes_preserved": True, "module_paths_stay_inside_package": True}
            (OUT / "package-record.json").write_text(json.dumps(package_record, indent=2) + "\n")
        row = {"name": job["name"], "exit_code": run.returncode, "seconds": time.monotonic() - start,
            "warnings": warnings, "verification_results": verification, "gate_passed": passed,
            "stdout_sha256": sha(stdout), "stderr_sha256": sha(stderr)}
        rows.append(row)
        assert files(vendor) == current_inputs
        if unpacked_inputs is not None:
            assert files(unpacked) == unpacked_inputs
        assert files(portable) == prepared["vendor_files_sha256"]
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in main_guard.items())
        assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
        assert all(sha(tool / n) == h for n, h in cargo_protocol["tool_sha256"].items())
        result = {"protocol_sha256": sha(OUT / "protocol.json"), "runs": rows,
            "reused_successful_runs": [{"phase": PREVIOUS.name, "run": row} for row in previous["runs"][:5]],
            "complete": len(rows) == len(jobs) and all(r["gate_passed"] for r in rows),
            "source_unchanged": True, "main_unchanged": True, "portable_source_unchanged": True,
            "dependency_artifacts_unchanged": True, "tooling_unchanged": True,
            "locked_inputs_sha256": sha(PREVIOUS / "locked-inputs.json"),
            "package_record_sha256": sha(OUT / "package-record.json") if (OUT / "package-record.json").exists() else None,
            "export_sha256": sha(work / "proof-artifacts/num_bigint.vir") if (work / "proof-artifacts/num_bigint.vir").exists() else None,
            "retained_coverage_changed": False, "runtime_admitted": False}
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        if not passed:
            break


if __name__ == "__main__":
    main()
