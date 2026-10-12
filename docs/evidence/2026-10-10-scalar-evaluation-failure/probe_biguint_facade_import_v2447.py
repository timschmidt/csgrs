"""Verify callers through Cargo's num facade and its actual verified BigUint."""
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
OUT = E / "biguint-facade-import-v2447"
WORK = ROOT / "target/biguint-facade-import-v2447"
EXPORT = E / "biguint-cargo-export-v2445"
CALLER = E / "biguint-import-caller-v2446.rs"
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
    assert export["compiler_result"]["verification_results"]["verified"] == 450
    assert sha(EXPORT / "dependency-reports/dependencies.json") == export["dependency_report_sha256"]
    fixture = Path(protocol["work"])
    assert all(sha(fixture / n) == h for n, h in protocol["work_files_sha256"].items())
    metadata = fixture / "artifacts/num_bigint.vir"
    assert sha(metadata) == export["export_sha256"]
    assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
    tool = Path(read(EXPORT / "wrapper-config.json")["verus"]).parent
    assert all(sha(tool / n) == h for n, h in protocol["tool_sha256"].items())
    OUT.mkdir()
    WORK.mkdir()
    text = CALLER.read_text()
    direct = (E / "biguint-import-caller-v2443.rs").read_text()
    assert text == direct.replace("use num_bigint::BigUint;", "use num::bigint::BigUint;")
    changes = [("bits-zero", "    value.bits()", "    0"),
               ("zeros-none", "    value.trailing_zeros()", "    None"),
               ("ones-zero", "    value.trailing_ones()", "    0")]
    variants = [("baseline", text, 0)]
    for name, before, after in changes:
        assert text.count(before) == 1
        variants.append((name, text.replace(before, after), 1))
    jobs = []
    for name, content, expected_exit in variants:
        path = WORK / (name + ".rs")
        path.write_text(content)
        (OUT / path.name).write_text(content)
        command = [str(tool / "verus"), "--edition=2021", "--crate-type=lib",
                   "--crate-name", "biguint_facade_caller", "--no-cheating", "--output-json",
                   "-D", "warnings", "--check-cfg", "cfg(verus_keep_ghost)",
                   "--check-cfg", "cfg(verus_keep_ghost_body)",
                   "--import", "num_bigint=" + str(metadata),
                   *dependencies["verus_arguments"], str(path)]
        jobs.append({"name": name, "path": str(path), "source_sha256": sha(path),
                     "command": command, "expected_exit": expected_exit})
    frozen = {"status": "Frozen before num facade callers and proof controls",
        "runner_sha256": sha(Path(__file__)), "caller_sha256": sha(CALLER),
        "retained_checkpoint_sha256": sha(checkpoint_path), "source_record_sha256": checkpoint["source_record_sha256"],
        "cargo_export_protocol_sha256": sha(EXPORT / "protocol.json"),
        "cargo_export_result_sha256": sha(EXPORT / "result.json"),
        "dependency_report_sha256": sha(EXPORT / "dependency-reports/dependencies.json"),
        "exported_metadata_sha256": sha(metadata), "jobs": jobs,
        "scope": "The same three conditional callers now use num::bigint::BigUint from Cargo's complete production-bound dependency graph. Only num is passed as a direct Rust external crate. Exported num_bigint proof metadata must resolve to that transitive type. Three body substitutions must fail real proof obligations. These support results do not change retained proof counts, prove unannotated arithmetic, discharge normalization/count-range premises or admit scalar runtime changes."}
    (OUT / "protocol.json").write_text(json.dumps(frozen, indent=2) + "\n")
    rows = []
    for job in jobs:
        start = time.monotonic()
        run = subprocess.run(job["command"], cwd=ROOT, env=os.environ, capture_output=True, text=True)
        stdout, stderr = OUT / (job["name"] + ".stdout.json"), OUT / (job["name"] + ".stderr.log")
        stdout.write_text(run.stdout)
        stderr.write_text(run.stderr)
        try:
            parsed = json.loads(run.stdout).get("verification-results", {})
        except json.JSONDecodeError:
            parsed = {}
        proof_failure = any(message in run.stderr for message in ["postcondition not satisfied", "assertion failed",
                                                                 "precondition not satisfied", "requires not satisfied"])
        passed = run.returncode == 0 if job["expected_exit"] == 0 else run.returncode != 0 and proof_failure
        row = {"name": job["name"], "exit_code": run.returncode, "seconds": time.monotonic() - start,
               "verification_results": parsed, "gate_passed": passed, "proof_failure": proof_failure,
               "stdout_sha256": sha(stdout), "stderr_sha256": sha(stderr), "source_sha256": sha(Path(job["path"]))}
        rows.append(row)
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in guard.items())
        assert all(sha(fixture / n) == h for n, h in protocol["work_files_sha256"].items())
        assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
        assert sha(metadata) == export["export_sha256"]
        assert all(sha(tool / n) == h for n, h in protocol["tool_sha256"].items())
        assert all(sha(Path(j["path"])) == j["source_sha256"] for j in jobs)
        data = {"protocol_sha256": sha(OUT / "protocol.json"), "runs": rows,
                "complete": len(rows) == len(jobs) and all(r["gate_passed"] for r in rows),
                "source_unchanged": True, "main_unchanged": True, "export_inputs_unchanged": True,
                "tooling_unchanged": True, "caller_inputs_unchanged": True,
                "retained_coverage_changed": False, "runtime_admitted": False}
        (OUT / "result.json").write_text(json.dumps(data, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        if not passed:
            break


if __name__ == "__main__":
    main()
