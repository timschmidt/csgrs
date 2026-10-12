"""Accept the exact module-selection note and finish the seven frozen controls."""
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
OUT = E / "bigint-metadata-guards-v2453"
PRIOR = E / "bigint-metadata-guards-v2451"
BASE = E / "bigint-metadata-export-v2450"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    assert not OUT.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    prior_protocol, prior_result = read(PRIOR / "protocol.json"), read(PRIOR / "result.json")
    base_protocol = read(BASE / "protocol.json")
    tool = Path(base_protocol["command"][0]).parent
    assert not prior_result["complete"] and len(prior_result["runs"]) == 1
    baseline = prior_result["runs"][0]
    assert baseline["exit_code"] == 0 and baseline["verification_results"]["verified"] == 3
    assert baseline["verification_results"]["errors"] == 0
    assert (PRIOR / "baseline.stderr.log").read_text() == "note: verifying module bigint\n\n"
    assert baseline["stdout_sha256"] == sha(PRIOR / "baseline.stdout.json")
    assert baseline["stderr_sha256"] == sha(PRIOR / "baseline.stderr.log")
    for job in prior_protocol["jobs"]:
        assert all(sha(Path(job["vendor"]) / n) == h for n, h in job["vendor_files_sha256"].items())
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    OUT.mkdir()
    protocol = {"status": "Frozen before completing the unrun actual BigInt controls",
        "runner_sha256": sha(Path(__file__)), "prior_protocol_sha256": sha(PRIOR / "protocol.json"),
        "prior_result_sha256": sha(PRIOR / "result.json"), "prior_runner_sha256": sha(E / "check_bigint_metadata_guards_v2451.py"),
        "retained_checkpoint_sha256": sha(checkpoint_path), "source_record_sha256": checkpoint["source_record_sha256"],
        "reused_baseline": baseline, "remaining_jobs": prior_protocol["jobs"][1:],
        "reason": "The selected-module baseline verified all three intended units without warnings or errors. The reader required empty stderr and incorrectly rejected the exact informational module-selection note. Preserve the original successful execution and failed reader result; accept only that exact note and run the seven already frozen, previously unexecuted mutations. No source, theorem or negative-control condition changes."}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    rows = [baseline | {"gate_passed": True, "original_gate_passed": False,
                        "original_phase": PRIOR.name, "accepted_exact_module_note": True}]
    for job in prior_protocol["jobs"][1:]:
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
        passed = run.returncode != 0 and proof_failure and not parsed.get("encountered-vir-error", True)
        row = {"name": job["name"], "kind": job["kind"], "exit_code": run.returncode,
               "seconds": time.monotonic() - start, "verification_results": parsed,
               "gate_passed": passed, "real_proof_failure": proof_failure,
               "stdout_sha256": sha(stdout), "stderr_sha256": sha(stderr)}
        rows.append(row)
        for frozen in prior_protocol["jobs"]:
            assert all(sha(Path(frozen["vendor"]) / n) == h for n, h in frozen["vendor_files_sha256"].items())
        assert all(sha(Path(base_protocol["vendor"]) / n) == h for n, h in base_protocol["vendor_files_sha256"].items())
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in main_guard.items())
        assert all(sha(tool / n) == h for n, h in base_protocol["tool_sha256"].items())
        assert all(sha(Path(n)) == h for n, h in base_protocol["dependency_artifacts_sha256"].items())
        result = {"protocol_sha256": sha(OUT / "protocol.json"), "runs": rows,
            "complete": len(rows) == len(prior_protocol["jobs"]) and all(r["gate_passed"] for r in rows),
            "source_unchanged": True, "main_unchanged": True, "base_unchanged": True, "tooling_unchanged": True,
            "retained_coverage_changed": False, "runtime_admitted": False}
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        if not passed:
            break


if __name__ == "__main__":
    main()
