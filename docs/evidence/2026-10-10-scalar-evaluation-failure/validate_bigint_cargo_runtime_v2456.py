"""Qualify proof-erased BigInt metadata against independent signed word inputs."""
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
OUT = E / "bigint-cargo-runtime-v2456"
WORK = ROOT / "target/bigint-cargo-runtime-v2456"
EXPORT = E / "bigint-cargo-export-v2454"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    assert not OUT.exists() and not WORK.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in guard.items())
    export, protocol = read(EXPORT / "result.json"), read(EXPORT / "protocol.json")
    dependencies = read(EXPORT / "dependency-reports/dependencies.json")
    assert export["gate_passed"] and export["compiler_result"]["warnings"] == 0
    assert read(E / "bigint-facade-import-v2455/result.json")["complete"]
    old_dependencies = read((E / checkpoint["source_record"]).parent / "proof-reports/dependencies.json")
    all_artifacts = old_dependencies["artifacts"] | dependencies["artifacts"]
    assert all(sha(Path(n)) == h for n, h in all_artifacts.items())
    fixture = Path(protocol["work"])
    assert all(sha(fixture / n) == h for n, h in protocol["work_files_sha256"].items())
    harness = E / "bigint-runtime-harness-v2456.rs"
    OUT.mkdir()
    WORK.mkdir()
    upstream = [n for n in old_dependencies["artifacts"] if Path(n).name.startswith("libnum_bigint-") and n.endswith(".rlib")]
    assert len(upstream) == 1
    facade_argument = dependencies["verus_arguments"][1]
    assert facade_argument.startswith("num=")
    tool = Path(read(EXPORT / "wrapper-config.json")["verus"]).parent
    pin = read(SOURCE / "crates/hyperreal/verification/toolchain.json")
    binary = WORK / "corpus"
    commands = [
        {"name": "format", "command": ["rustfmt", "--edition", "2021", "--config", "skip_children=true", "--check", str(harness), str(E / "bigint-import-caller-v2455.rs")]},
        {"name": "compile", "command": ["rustc", "+" + pin["rust_toolchain"], "--edition=2021", "-D", "warnings",
            "--check-cfg", "cfg(verus_keep_ghost)", "--check-cfg", "cfg(verus_keep_ghost_body)",
            "--extern", "patched=" + facade_argument.removeprefix("num="), "--extern", "upstream=" + upstream[0],
            *dependencies["verus_arguments"][-2:], *old_dependencies["verus_arguments"][-2:],
            "-L", "dependency=" + str(tool), str(harness), "-o", str(binary)]},
        {"name": "execute", "command": [str(binary)]},
    ]
    frozen = {"status": "Frozen before formatting, compiling and executing the signed-word oracle corpus",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"], "commands": commands,
        "harness": str(harness), "harness_sha256": sha(harness),
        "cargo_export_result_sha256": sha(EXPORT / "result.json"),
        "dependency_report_sha256": sha(EXPORT / "dependency-reports/dependencies.json"),
        "dependency_artifacts_sha256": all_artifacts,
        "expected_corpus_counts": {"fixtures": "197388", "paired_values": "789552"},
        "scope": "One fresh native execution compares the Cargo num facade and its Verus-compiled BigInt with the pinned registry original. The independent oracle normalizes signed u32 word inputs without using either dependency. All u16 words and zero-padded multiword inputs cover three sign tags, from-slice/from-magnitude constructors, negation and cancellation. This is focused getter and proof-erasure qualification, not formal constructor invariants, scalar runtime admission, timings, memory, binary size or complete repository CI."}
    (OUT / "protocol.json").write_text(json.dumps(frozen, indent=2) + "\n")
    rows = []
    for job in commands:
        start = time.monotonic()
        run = subprocess.run(job["command"], cwd=ROOT, env=os.environ, capture_output=True, text=True)
        stdout, stderr = OUT / (job["name"] + ".stdout.log"), OUT / (job["name"] + ".stderr.log")
        stdout.write_text(run.stdout)
        stderr.write_text(run.stderr)
        passed = run.returncode == 0 and not run.stderr
        if job["name"] == "execute":
            observations = dict(item.split("=", 1) for item in run.stdout.split())
            passed = passed and all(observations.get(k) == v for k, v in frozen["expected_corpus_counts"].items())
        row = {"name": job["name"], "exit_code": run.returncode, "seconds": time.monotonic() - start,
               "gate_passed": passed, "stdout_sha256": sha(stdout), "stderr_sha256": sha(stderr)}
        rows.append(row)
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in guard.items())
        assert all(sha(fixture / n) == h for n, h in protocol["work_files_sha256"].items())
        assert sha(harness) == frozen["harness_sha256"]
        assert all(sha(Path(n)) == h for n, h in all_artifacts.items())
        result = {"protocol_sha256": sha(OUT / "protocol.json"), "runs": rows,
                  "complete": len(rows) == len(commands) and all(r["gate_passed"] for r in rows),
                  "binary_sha256": sha(binary) if binary.exists() else None,
                  "source_unchanged": True, "main_unchanged": True, "dependency_inputs_unchanged": True,
                  "corpus_source_unchanged_during_validation": True, "fresh_scalar_runtime_tests": 0, "fresh_dependency_corpus_executions": int(job["name"] == "execute"),
                  "retained_coverage_changed": False, "runtime_admitted": False}
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        if not passed:
            break


if __name__ == "__main__":
    main()
