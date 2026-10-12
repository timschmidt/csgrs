"""Fetch only the already locked native optional dependencies missing from the offline cache."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PREVIOUS = E / "bigint-portable-validation-v2468"
OUT = E / "bigint-portable-fetch-v2470"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    assert not OUT.exists()
    previous, protocol = read(PREVIOUS / "result.json"), read(PREVIOUS / "protocol.json")
    assert not previous["complete"] and len(previous["runs"]) == 6
    assert all(r["gate_passed"] for r in previous["runs"][:5])
    assert previous["runs"][-1]["name"] == "clippy-all-features" and previous["runs"][-1]["exit_code"] == 101
    assert "failed to download `quickcheck v1.0.3`" in (PREVIOUS / "clippy-all-features.stderr.log").read_text()
    work = Path(protocol["work"])
    vendor = work / "num-bigint"
    inputs = read(PREVIOUS / "locked-inputs.json")
    assert all(sha(vendor / n) == h for n, h in inputs.items())
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    OUT.mkdir()
    command = ["cargo", "fetch", "--locked", "--target", "x86_64-unknown-linux-gnu", "--manifest-path", str(vendor / "Cargo.toml")]
    overrides = {"CARGO_HOME": "/tmp/csgrs-properties-cargo", "CARGO_NET_OFFLINE": "false", "CARGO_TERM_COLOR": "never"}
    frozen = {"status": "Frozen before populating the cache from the unchanged lockfile",
        "runner_sha256": sha(Path(__file__)), "previous_result_sha256": sha(PREVIOUS / "result.json"),
        "previous_protocol_sha256": sha(PREVIOUS / "protocol.json"), "locked_inputs_sha256": sha(PREVIOUS / "locked-inputs.json"),
        "command": command, "environment_overrides": overrides,
        "scope": "The all-feature Clippy command stopped before compiling because quickcheck 1.0.3 was absent from the offline cache. Cargo may fetch native-target packages already pinned by the unchanged lockfile; no dependency version, package source, source code, prior result or verification input is changed. This cache preparation is not a passing Clippy result."}
    (OUT / "protocol.json").write_text(json.dumps(frozen, indent=2) + "\n")
    start = time.monotonic()
    run = subprocess.run(command, cwd=work, env=os.environ | overrides, capture_output=True, text=True)
    (OUT / "stdout.log").write_text(run.stdout)
    (OUT / "stderr.log").write_text(run.stderr)
    assert all(sha(vendor / n) == h for n, h in inputs.items())
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    result = {"exit_code": run.returncode, "seconds": time.monotonic() - start, "gate_passed": run.returncode == 0,
        "protocol_sha256": sha(OUT / "protocol.json"), "stdout_sha256": sha(OUT / "stdout.log"),
        "stderr_sha256": sha(OUT / "stderr.log"), "locked_inputs_unchanged": True, "source_unchanged": True,
        "main_unchanged": True, "retained_coverage_changed": False, "runtime_admitted": False}
    (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
