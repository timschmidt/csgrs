"""Run a frozen ordinary-Rust preview command without changing retained sources."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PREPARATION = E / "biguint-runtime-preparation-v2416"
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
    phase, kind = sys.argv[1:]
    assert re.fullmatch(r"biguint-runtime-v[0-9]+-[a-z-]+", phase)
    out = E / phase
    assert not out.exists()
    prepared = read(PREPARATION / "result.json")
    preparation_protocol = read(PREPARATION / "protocol.json")
    assert prepared["protocol_sha256"] == sha(PREPARATION / "protocol.json")
    work = Path(prepared["work"])
    assert files(work) == prepared["work_files_sha256"]
    assert all(files(Path(n).parent) == h for n, h in prepared["dependency_sources_sha256"].items())
    preview = E / preparation_protocol["preview"]
    assert files(preview) == preparation_protocol["preview_files_sha256"]
    checkpoint_path = E / "typed-api-prototype-v2386-state.json"
    assert sha(checkpoint_path) == "6dabcb25b03a7a8769c432b926b3c4a35288ed43fd172e7a46d27503d8f68cb9"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    vendor = str(work / "vendor/num-bigint/Cargo.toml")
    harness = str(work / "Cargo.toml")
    cargo = lambda operation, manifest, *arguments: ["cargo", operation, "--offline", "--locked", "--manifest-path", manifest, *arguments]
    commands = {
        "vendor-clippy": cargo("clippy", vendor, "--lib", "--", "-D", "warnings"),
        "vendor-tests": cargo("test", vendor, "--lib", "--tests"),
        "vendor-no-std": cargo("check", vendor, "--lib", "--no-default-features"),
        "harness-debug": cargo("run", harness),
        "harness-release": cargo("run", harness, "--release"),
        "shared-fmt": ["rustfmt", "--edition", "2021", "--check", "--config", "skip_children=true",
                       str(work / "src/main.rs"), *[str(work / "vendor/num-bigint/src/biguint" / name) for name in
                       ["representation.rs", "metadata_impl.rs", "metadata_u32.rs", "metadata_u64.rs"]]],
    }
    command = commands[kind]
    env = os.environ | preparation_protocol["environment"]
    out.mkdir()
    protocol = {
        "status": "Frozen before this ordinary-Rust preview validation",
        "runner_sha256": sha(Path(__file__)), "preparation": PREPARATION.name,
        "preparation_result_sha256": sha(PREPARATION / "result.json"), "kind": kind,
        "command": command, "cwd": str(work), "environment": preparation_protocol["environment"],
        "retained_checkpoint_sha256": sha(checkpoint_path), "source_record_sha256": checkpoint["source_record_sha256"],
        "scope": "Direct patched dependency validation and a deterministic bit-level/differential corpus. This does not run scalar fixed-baseline cost, memory or size qualification; it cannot admit a runtime change. Sources, resolved dependency inputs and prior preview evidence must remain byte-identical through the command.",
    }
    (out / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    start = time.monotonic()
    result = subprocess.run(command, cwd=work, env=env, capture_output=True, text=True)
    (out / "stdout.log").write_text(result.stdout)
    (out / "stderr.log").write_text(result.stderr)
    assert files(work) == prepared["work_files_sha256"]
    assert all(files(Path(n).parent) == h for n, h in prepared["dependency_sources_sha256"].items())
    assert files(preview) == preparation_protocol["preview_files_sha256"]
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    test_rows = re.findall(r"test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;", result.stdout)
    data = {
        "exit_code": result.returncode, "seconds": time.monotonic() - start,
        "protocol_sha256": sha(out / "protocol.json"), "stdout_sha256": sha(out / "stdout.log"),
        "stderr_sha256": sha(out / "stderr.log"), "kind": kind,
        "test_summaries": [[int(v) for v in row] for row in test_rows],
        "warning_lines": [line for line in result.stderr.splitlines() if line.startswith("warning:")],
        "source_unchanged": True, "main_unchanged": True, "work_unchanged": True,
        "preview_unchanged": True, "dependencies_unchanged": True,
        "retained_proof_coverage_changed": False, "runtime_admitted": False,
    }
    if kind.startswith("harness-") and result.returncode == 0:
        match = re.fullmatch(r"cases=(\d+) paired_values=(\d+) helper_width_checks=(\d+) diagnostic_checksum=([0-9a-f]{16})\n", result.stdout)
        assert match, result.stdout
        data["harness"] = dict(zip(["cases", "paired_values", "helper_width_checks", "diagnostic_checksum"], match.groups()))
        binary = Path(prepared["target"]) / ("release" if kind.endswith("release") else "debug") / "biguint-metadata-runtime-preview"
        data["binary"] = str(binary)
        data["binary_sha256"] = sha(binary)
    (out / "result.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps(data))
    sys.exit(0 if result.returncode == 0 else 1)


if __name__ == "__main__":
    main()
