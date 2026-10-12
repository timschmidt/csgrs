"""Run the frozen independent corpus through the verified Cargo num facade."""
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
OUT = E / "biguint-cargo-runtime-v2448"
WORK = ROOT / "target/biguint-cargo-runtime-v2448"
EXPORT = E / "biguint-cargo-export-v2445"
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
    assert read(E / "biguint-facade-import-v2447/result.json")["complete"]
    old_dependencies = read((E / checkpoint["source_record"]).parent / "proof-reports/dependencies.json")
    all_artifacts = old_dependencies["artifacts"] | dependencies["artifacts"]
    assert all(sha(Path(n)) == h for n, h in all_artifacts.items())
    fixture = Path(protocol["work"])
    assert all(sha(fixture / n) == h for n, h in protocol["work_files_sha256"].items())
    native = read(E / "biguint-runtime-preparation-v2416/result.json")
    native_root = Path(native["work"])
    assert all(sha(native_root / n) == h for n, h in native["work_files_sha256"].items())
    harness = native_root / "src/main.rs"
    assert sha(harness) == sha(E / "biguint-runtime-harness-v2409.rs")
    for width in [32, 64]:
        name = f"vendor/num-bigint/src/biguint/metadata_u{width}.rs"
        assert sha(native_root / name) == sha(fixture / name)
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
        {"name": "compile", "command": ["rustc", "+" + pin["rust_toolchain"], "--edition=2021", "-D", "warnings",
            "--check-cfg", "cfg(verus_keep_ghost)", "--check-cfg", "cfg(verus_keep_ghost_body)",
            "--extern", "patched=" + facade_argument.removeprefix("num="), "--extern", "upstream=" + upstream[0],
            *dependencies["verus_arguments"][-2:], *old_dependencies["verus_arguments"][-2:],
            "-L", "dependency=" + str(tool), str(harness), "-o", str(binary)]},
        {"name": "execute", "command": [str(binary)]},
    ]
    frozen = {"status": "Frozen before compiling and executing the unchanged oracle corpus",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"], "commands": commands,
        "harness": str(harness), "harness_sha256": sha(harness),
        "native_preparation_sha256": sha(E / "biguint-runtime-preparation-v2416/result.json"),
        "cargo_export_result_sha256": sha(EXPORT / "result.json"),
        "dependency_report_sha256": sha(EXPORT / "dependency-reports/dependencies.json"),
        "dependency_artifacts_sha256": all_artifacts,
        "expected_corpus_stdout_sha256": "f3598386985c93b4e77a64ada29cd6935ade0ca7a550d8d09323512386cc4acf",
        "scope": "One fresh native execution of the unchanged 67137-case corpus against the Cargo-built num facade and its Verus-compiled BigUint, plus the pinned registry original. Public metadata comparisons use an independent bit-vector oracle; direct shared helpers cover both limb widths. This qualifies proof-erased execution for this dependency build, not scalar runtime admission, timings, memory, binary size or complete repository CI."}
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
            passed = passed and sha(stdout) == frozen["expected_corpus_stdout_sha256"]
        row = {"name": job["name"], "exit_code": run.returncode, "seconds": time.monotonic() - start,
               "gate_passed": passed, "stdout_sha256": sha(stdout), "stderr_sha256": sha(stderr)}
        rows.append(row)
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in guard.items())
        assert all(sha(fixture / n) == h for n, h in protocol["work_files_sha256"].items())
        assert all(sha(native_root / n) == h for n, h in native["work_files_sha256"].items())
        assert all(sha(Path(n)) == h for n, h in all_artifacts.items())
        result = {"protocol_sha256": sha(OUT / "protocol.json"), "runs": rows,
                  "complete": len(rows) == len(commands) and all(r["gate_passed"] for r in rows),
                  "binary_sha256": sha(binary) if binary.exists() else None,
                  "source_unchanged": True, "main_unchanged": True, "dependency_inputs_unchanged": True,
                  "corpus_unchanged": True, "fresh_scalar_runtime_tests": 0, "fresh_dependency_corpus_executions": int(job["name"] == "execute"),
                  "retained_coverage_changed": False, "runtime_admitted": False}
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        if not passed:
            break


if __name__ == "__main__":
    main()
