"""Qualify relocated package inputs, host/32-bit builds and the packaged proof target."""
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

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PREPARED = E / "bigint-portable-package-v2466"
CARGO = E / "bigint-portable-cargo-v2467"
OUT = E / "bigint-portable-validation-v2468"
WORK = Path("/tmp/csgrs-bigint-portable-validation-v2468")
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob("*")) if p.is_file()}


def main():
    assert not OUT.exists() and not WORK.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    prepared = read(PREPARED / "protocol.json")
    portable = Path(prepared["vendor"])
    cargo, cargo_protocol = read(CARGO / "result.json"), read(CARGO / "protocol.json")
    assert cargo["gate_passed"] and cargo["compiler_result"]["verification_results"]["verified"] == 453
    assert cargo["compiler_result"]["warnings"] == 0
    dependencies = read(CARGO / "dependency-reports/dependencies.json")
    tool = Path(read(CARGO / "wrapper-config.json")["verus"]).parent
    assert files(portable) == prepared["vendor_files_sha256"]
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    assert all(sha(tool / n) == h for n, h in cargo_protocol["tool_sha256"].items())
    OUT.mkdir()
    WORK.mkdir()
    vendor = WORK / "num-bigint"
    shutil.copytree(portable, vendor)
    assert files(vendor) == prepared["vendor_files_sha256"]
    target = WORK / "target"
    archive = target / "package/num-bigint-0.4.6.crate"
    unpacked = WORK / "unpacked/num-bigint-0.4.6"
    artifacts = WORK / "proof-artifacts"
    artifacts.mkdir()
    manifest = str(vendor / "Cargo.toml")
    environment = os.environ.copy()
    for name in ["RUSTC_WRAPPER", "RUSTC_WORKSPACE_WRAPPER", "CARGO_ENCODED_RUSTFLAGS"]:
        environment.pop(name, None)
    overrides = {"CARGO_HOME": "/tmp/csgrs-properties-cargo", "CARGO_NET_OFFLINE": "true",
        "CARGO_TARGET_DIR": str(target), "CARGO_INCREMENTAL": "0", "CARGO_PROFILE_DEV_DEBUG": "0",
        "CARGO_TERM_COLOR": "never", "CCACHE_DIR": "/tmp/csgrs-properties-ccache", "RUSTFLAGS": "-D warnings"}
    environment.update(overrides)
    jobs = [
        {"name": "resolve", "command": ["cargo", "generate-lockfile", "--offline", "--manifest-path", manifest]},
        {"name": "workspace-msrv", "command": ["cargo", "+1.95.0", "check", "--offline", "--locked", "--lib", "--manifest-path", manifest]},
        {"name": "workspace-msrv-no-std", "command": ["cargo", "+1.95.0", "check", "--offline", "--locked", "--lib", "--no-default-features", "--manifest-path", manifest]},
        {"name": "i686-no-std", "command": ["cargo", "check", "--offline", "--locked", "--lib", "--no-default-features", "--target", "i686-unknown-linux-musl", "--manifest-path", manifest]},
        {"name": "wasm32-no-std", "command": ["cargo", "check", "--offline", "--locked", "--lib", "--no-default-features", "--target", "wasm32-unknown-unknown", "--manifest-path", manifest]},
        {"name": "clippy-all-features", "command": ["cargo", "clippy", "--offline", "--locked", "--lib", "--all-features", "--manifest-path", manifest, "--", "-D", "warnings"]},
        {"name": "package", "command": ["cargo", "package", "--offline", "--locked", "--allow-dirty", "--no-verify", "--manifest-path", manifest]},
        {"name": "packaged-build", "command": ["cargo", "+1.98.1", "check", "--offline", "--locked", "--lib", "--manifest-path", str(unpacked / "Cargo.toml")]},
    ]
    invocation = read(CARGO / "compiler/compiler-invocation.json")
    proof_command = invocation["verus_command"].copy()
    previous_vendor = str(Path(cargo_protocol["work"]) / "vendor/num-bigint/src/lib.rs")
    assert proof_command.count(previous_vendor) == 1
    proof_command[proof_command.index(previous_vendor)] = str(unpacked / "src/lib.rs")
    proof_command[proof_command.index("--export") + 1] = str(artifacts / "num_bigint.vir")
    proof_command[proof_command.index("--out-dir") + 1] = str(artifacts)
    jobs.append({"name": "packaged-proof", "command": proof_command})
    protocol = {"status": "Frozen before lock preparation, portable builds, local packaging and relocated proof",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"], "prepared_protocol_sha256": sha(PREPARED / "protocol.json"),
        "cargo_result_sha256": sha(CARGO / "result.json"), "cargo_protocol_sha256": sha(CARGO / "protocol.json"),
        "cargo_compiler_invocation_sha256": sha(CARGO / "compiler/compiler-invocation.json"),
        "original_vendor_files_sha256": prepared["vendor_files_sha256"], "work": str(WORK),
        "jobs": jobs, "environment_overrides": overrides, "removed_environment_keys": ["RUSTC_WRAPPER", "RUSTC_WORKSPACE_WRAPPER", "CARGO_ENCODED_RUSTFLAGS"],
        "dependency_artifacts_sha256": dependencies["artifacts"], "tool_sha256": cargo_protocol["tool_sha256"],
        "expected_package_paths": sorted(n for n in prepared["vendor_files_sha256"] if n != ".cargo_vcs_info.json"),
        "lock_preparation": "The first command may add only Cargo.lock to the copied vendor. Freeze that file before all later commands; no qualified source file may change.",
        "package_preparation": "After local cargo package, extract the archive with the data filter and require every source/model/license/provenance byte to match the frozen package input. Only Cargo manifest normalization, Cargo.toml.orig, generated Cargo.lock and Cargo VCS metadata may differ. Freeze the extracted complete source catalog before its build and proof.",
        "scope": "Run from a fresh /tmp tree outside the repository and its Cargo configuration hierarchy. Check the workspace's declared Rust 1.95, no_std, both native word-width selections via i686 and wasm32 cross compilation, and strict library Clippy with all optional features. Package locally without publishing, build the unpacked crate and verify/compile its actual root using the qualified Cargo compiler arguments with only source/export/output paths relocated. The cross checks do not execute target binaries or prove a full 32-bit crate; Rust 1.60 is not installed and is not credited. This is dependency packaging evidence, not downstream registry binding, scalar runtime admission, cost/memory/size or full repository CI."}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    current_inputs = prepared["vendor_files_sha256"].copy()
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
        if job["name"] == "resolve" and passed:
            current_inputs["Cargo.lock"] = sha(vendor / "Cargo.lock")
            (OUT / "Cargo.lock").write_bytes((vendor / "Cargo.lock").read_bytes())
            (OUT / "locked-inputs.json").write_text(json.dumps(current_inputs, indent=2) + "\n")
        if job["name"] == "package" and passed:
            assert archive.is_file()
            (OUT / archive.name).write_bytes(archive.read_bytes())
            with tarfile.open(archive) as package:
                entries = package.getmembers()
                assert all(entry.isfile() or entry.isdir() for entry in entries)
                assert len({entry.name for entry in entries}) == len(entries)
                package.extractall(WORK / "unpacked", filter="data")
            unpacked_inputs = files(unpacked)
            expected = set(prepared["vendor_files_sha256"]) - {".cargo_vcs_info.json"}
            assert expected <= unpacked_inputs.keys()
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
            "complete": len(rows) == len(jobs) and all(r["gate_passed"] for r in rows),
            "source_unchanged": True, "main_unchanged": True, "portable_source_unchanged": True,
            "dependency_artifacts_unchanged": True, "tooling_unchanged": True,
            "locked_inputs_sha256": sha(OUT / "locked-inputs.json") if (OUT / "locked-inputs.json").exists() else None,
            "package_record_sha256": sha(OUT / "package-record.json") if (OUT / "package-record.json").exists() else None,
            "export_sha256": sha(artifacts / "num_bigint.vir") if (artifacts / "num_bigint.vir").exists() else None,
            "retained_coverage_changed": False, "runtime_admitted": False}
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        if not passed:
            break


if __name__ == "__main__":
    main()
