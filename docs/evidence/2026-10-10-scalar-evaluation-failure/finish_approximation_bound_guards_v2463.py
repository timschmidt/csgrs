"""Use the required exact-module selector for the already frozen producer controls."""
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
FROZEN = E / "approximation-bound-guards-v2462"
OUT = E / "approximation-bound-guards-v2463"
PREVIEW = E / "approximation-bound-candidate-v2460"
BASE = E / "approximation-bound-proof-v2461"
CARGO = E / "bigint-cargo-export-v2454"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def files(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob("*")) if p.is_file()}


def main():
    assert not OUT.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    frozen, incomplete = read(FROZEN / "protocol.json"), read(FROZEN / "result.json")
    assert not incomplete["complete"] and len(incomplete["runs"]) == 1
    assert incomplete["runs"][0]["exit_code"] == 255 and not incomplete["runs"][0]["verification_results"]
    assert (FROZEN / "baseline.stderr.log").read_text() == "Error: When using --verify-function, use --verify-only-module instead of --verify-module\n"
    assert sha(FROZEN / "protocol.json") == incomplete["protocol_sha256"]
    assert sha(BASE / "result.json") == frozen["base_result_sha256"]
    preview = read(PREVIEW / "protocol.json")
    base_protocol = read(BASE / "protocol.json")
    cargo, cargo_protocol = read(CARGO / "result.json"), read(CARGO / "protocol.json")
    dependencies = read(CARGO / "dependency-reports/dependencies.json")
    fixture = Path(cargo_protocol["work"])
    metadata = fixture / "artifacts/num_bigint.vir"
    tool = Path(base_protocol["command"][0]).parent
    jobs = []
    for job in frozen["jobs"]:
        assert job["command"].count("--verify-module") == 1
        command = ["--verify-only-module" if arg == "--verify-module" else arg for arg in job["command"]]
        jobs.append(job | {"command": command})

    def audit():
        assert all(files(Path(j["work"])) == j["work_files_sha256"] for j in jobs)
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in main_guard.items())
        assert all(sha(PREVIEW / n) == h for n, h in preview["preview_files_sha256"].items())
        assert all(sha(fixture / n) == h for n, h in cargo_protocol["work_files_sha256"].items())
        assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
        assert sha(metadata) == cargo["export_sha256"]
        assert all(sha(tool / n) == h for n, h in base_protocol["tool_sha256"].items())

    audit()
    OUT.mkdir()
    protocol = {"status": "Frozen command correction before running the unchanged thirteen producer controls",
        "runner_sha256": sha(Path(__file__)), "frozen_protocol_sha256": sha(FROZEN / "protocol.json"),
        "incomplete_result_sha256": sha(FROZEN / "result.json"), "jobs": jobs,
        "correction": "The pinned verifier requires --verify-only-module with --verify-function. Change that one selector in every frozen command; no input bytes, proof premises, obligations or gate criteria change. The initial command failed before proof and earns zero proof credit.",
        "baseline_allowed_stderr": frozen["baseline_allowed_stderr"], "scope": frozen["scope"]}
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
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
            "precondition not satisfied", "requires not satisfied", "possible arithmetic underflow/overflow"])
        informational_only = all(not line.strip() or line.startswith("note: verifying ") for line in run.stderr.splitlines())
        if job["kind"] == "baseline":
            passed = run.returncode == 0 and parsed.get("verified") == 1 and parsed.get("errors") == 0 and informational_only
        else:
            passed = run.returncode != 0 and proof_failure and parsed.get("errors", 0) > 0 and not parsed.get("encountered-vir-error", True)
        row = {"name": job["name"], "kind": job["kind"], "exit_code": run.returncode,
            "seconds": time.monotonic() - start, "verification_results": parsed,
            "gate_passed": passed, "real_proof_failure": proof_failure, "informational_stderr_only": informational_only,
            "stdout_sha256": sha(stdout), "stderr_sha256": sha(stderr)}
        rows.append(row)
        audit()
        result = {"protocol_sha256": sha(OUT / "protocol.json"), "runs": rows,
            "complete": len(rows) == len(jobs) and all(r["gate_passed"] for r in rows),
            "source_unchanged": True, "main_unchanged": True, "preview_unchanged": True,
            "dependency_inputs_unchanged": True, "tooling_unchanged": True,
            "frozen_variant_inputs_unchanged": True, "retained_coverage_changed": False, "runtime_admitted": False}
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        if not passed:
            break


if __name__ == "__main__":
    main()
