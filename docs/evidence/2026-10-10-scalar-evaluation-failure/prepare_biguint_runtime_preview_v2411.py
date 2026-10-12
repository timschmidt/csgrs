"""Resolve an isolated ordinary-Rust qualification preview from frozen shared files."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import tomllib

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PREVIEW = E / "biguint-metadata-candidate-v2406"
OUT = E / "biguint-runtime-preparation-v2411"
WORK = ROOT / "target/biguint-metadata-runtime-preview-v2411"
TARGET = ROOT / "target/biguint-metadata-runtime-build-v2411"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    result = {}
    for directory, children, names in os.walk(root):
        children[:] = sorted(n for n in children if n not in {"target", ".git", "__pycache__"})
        for name in sorted(names):
            path = Path(directory) / name
            assert not path.is_symlink(), path
            result[str(path.relative_to(root))] = sha(path)
    return result


def main():
    assert not OUT.exists() and not WORK.exists() and not TARGET.exists()
    checkpoint_path = E / "typed-api-prototype-v2386-state.json"
    assert sha(checkpoint_path) == "6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    proof = E / "biguint-metadata-proof-v2407"
    controls = E / "biguint-metadata-guards-v2408"
    assert read(proof / "result.json")["exit_code"] == 0
    assert read(controls / "result.json")["complete"]
    assert len(read(controls / "result.json")["runs"]) == 21
    preview_files = read(proof / "protocol.json")["preview_files_sha256"]
    assert files(PREVIEW) == preview_files
    OUT.mkdir()
    shutil.copytree(PREVIEW / "vendor/num-bigint", WORK / "vendor/num-bigint")
    (WORK / "src").mkdir()
    harness = E / "biguint-runtime-harness-v2409.rs"
    (WORK / "src/main.rs").write_bytes(harness.read_bytes())
    manifest = '''[package]
name = "biguint-metadata-runtime-preview"
version = "0.0.0"
edition = "2021"
publish = false

[workspace]
resolver = "2"
exclude = ["vendor/num-bigint"]

[lints.rust]
unexpected_cfgs = { level = "warn", check-cfg = ['cfg(verus_keep_ghost)'] }

[dependencies]
patched = { package = "num-bigint", path = "vendor/num-bigint" }
upstream = { package = "num-bigint", version = "=0.4.6" }
'''
    (WORK / "Cargo.toml").write_text(manifest)
    (OUT / "harness.Cargo.toml").write_text(manifest)
    # Seed both resolutions from the retained production lock; Cargo may remove
    # unused packages but must retain identities of every production dependency.
    for path in [WORK / "Cargo.lock", WORK / "vendor/num-bigint/Cargo.lock"]:
        path.write_bytes((SOURCE / "Cargo.lock").read_bytes())
    before = files(WORK)
    env = os.environ | {
        "CARGO_HOME": "/tmp/csgrs-properties-cargo", "CARGO_NET_OFFLINE": "true",
        "CARGO_TARGET_DIR": str(TARGET), "CCACHE_DIR": "/tmp/csgrs-properties-ccache",
        "CARGO_TERM_COLOR": "never",
    }
    commands = [
        {"name": name, "command": ["cargo", "metadata", "--offline", "--format-version", "1", "--manifest-path", str(path)]}
        for name, path in [("harness", WORK / "Cargo.toml"), ("vendor", WORK / "vendor/num-bigint/Cargo.toml")]
    ]
    protocol = {
        "status": "Frozen before dependency resolution; no ordinary build or runtime execution yet",
        "runner_sha256": sha(Path(__file__)), "harness_sha256": sha(harness),
        "retained_checkpoint_sha256": sha(checkpoint_path), "source_record_sha256": checkpoint["source_record_sha256"],
        "preview": PREVIEW.name, "preview_files_sha256": preview_files, "work": str(WORK), "target": str(TARGET),
        "work_before_resolution_sha256": before, "commands": commands,
        "proof_result_sha256": sha(proof / "result.json"), "guard_result_sha256": sha(controls / "result.json"),
        "environment": {k: env[k] for k in ["CARGO_HOME", "CARGO_NET_OFFLINE", "CARGO_TARGET_DIR", "CCACHE_DIR", "CARGO_TERM_COLOR"]},
        "scope": "Isolated direct dependency comparison, not the scalar runtime or its fixed-baseline qualification. The patched package is copied byte-for-byte from the verified preview. The registry original remains separately resolved. Native public API tests run on the host width; direct shared helper tests exercise both widths. SOURCE, main and retained coverage remain unchanged.",
    }
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    results = []
    metadata = []
    for row in commands:
        started = time.monotonic()
        result = subprocess.run(row["command"], cwd=WORK, env=env, capture_output=True, text=True)
        (OUT / f"{row['name']}.metadata.json").write_text(result.stdout)
        (OUT / f"{row['name']}.stderr.log").write_text(result.stderr)
        results.append(row | {"exit_code": result.returncode, "seconds": time.monotonic() - started,
            "stdout_sha256": sha(OUT / f"{row['name']}.metadata.json"), "stderr_sha256": sha(OUT / f"{row['name']}.stderr.log")})
        (OUT / "resolution-result.json").write_text(json.dumps({"protocol_sha256": sha(OUT / "protocol.json"),
            "runs": results, "complete": len(results) == len(commands) and result.returncode == 0}, indent=2) + "\n")
        assert result.returncode == 0, result.stderr
        metadata.append(json.loads(result.stdout))
        print(json.dumps({"resolved": row["name"], "packages": len(metadata[-1]["packages"])}), flush=True)
    after = files(WORK)
    assert set(before) == set(after)
    assert [n for n in before if before[n] != after[n]] == ["Cargo.lock", "vendor/num-bigint/Cargo.lock"]
    for name, path in [("harness", WORK / "Cargo.lock"), ("vendor", WORK / "vendor/num-bigint/Cargo.lock")]:
        (OUT / f"{name}.Cargo.lock").write_bytes(path.read_bytes())
    identity = lambda p: (p["name"], p["version"], p.get("source"), p.get("checksum"))
    production = {identity(p) for p in tomllib.loads((SOURCE / "Cargo.lock").read_text())["package"]}
    for path in [WORK / "Cargo.lock", WORK / "vendor/num-bigint/Cargo.lock"]:
        for package in tomllib.loads(path.read_text())["package"]:
            if package.get("source") and package["name"] in {"num-bigint", "num-traits", "num-integer", "autocfg"}:
                assert identity(package) in production, (path, package)
    resolved_packages = {p["id"]: p for data in metadata for p in data["packages"]}
    biguint_packages = [p for p in resolved_packages.values() if p["name"] == "num-bigint"]
    assert len(biguint_packages) == 2
    assert any(p["source"] is None and Path(p["manifest_path"]) == WORK / "vendor/num-bigint/Cargo.toml" for p in biguint_packages)
    assert any(p["source"] == "registry+https://github.com/rust-lang/crates.io-index" and p["version"] == "0.4.6" for p in biguint_packages)
    dependency_sources = {p["manifest_path"]: files(Path(p["manifest_path"]).parent)
                          for p in resolved_packages.values() if p["source"] is not None}
    result = {
        "status": "Ordinary-Rust preview inputs and resolutions frozen before compilation",
        "protocol_sha256": sha(OUT / "protocol.json"), "work": str(WORK), "target": str(TARGET),
        "work_files_sha256": after, "resolved_packages": list(resolved_packages.values()),
        "dependency_sources_sha256": dependency_sources,
        "rustc_version": subprocess.check_output(["rustc", "--version", "--verbose"], text=True),
        "cargo_version": subprocess.check_output(["cargo", "--version"], text=True).strip(),
        "host": subprocess.check_output(["rustc", "--print", "host-tuple"], text=True).strip(),
        "source_unchanged": all(sha(SOURCE / n) == h for n, h in sources.items()),
        "main_unchanged": all(sha(ROOT / n) == h for n, h in guard.items()),
        "preview_unchanged": files(PREVIEW) == preview_files,
        "runtime_admitted": False, "retained_proof_coverage_changed": False,
    }
    assert result["source_unchanged"] and result["main_unchanged"] and result["preview_unchanged"]
    (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"frozen_work_files": len(after), "resolved_packages": len(resolved_packages),
        "registry_source_files": sum(len(v) for v in dependency_sources.values()), "runtime_admitted": False}))


if __name__ == "__main__":
    main()
