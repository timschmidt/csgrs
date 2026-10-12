"""Reject contradictory, weakened and premise-free approximation-bound producers."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
PREVIEW = E / "approximation-bound-candidate-v2460"
BASE = E / "approximation-bound-proof-v2461"
CARGO = E / "bigint-cargo-export-v2454"
OUT = E / "approximation-bound-guards-v2462"
WORK = ROOT / "target/approximation-bound-guards-v2462"
SHARED = "candidate/crates/hyperreal/src/computable/node/approximation_bound.rs"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))

MUTATIONS = [
    ("early-threshold-too-small", "executable", "if magnitude_bits <= 1 {", "if magnitude_bits <= 0 {"),
    ("discard-certified-result", "weaker-output", "            result\n", "            BoundInfo::Unknown\n"),
    ("all-ones-reversed", "executable", "let reaches_next_binade = trailing_ones == magnitude_bits;", "let reaches_next_binade = trailing_ones != magnitude_bits;"),
    ("exponent-correction-reversed", "executable", "if reaches_next_binade { 0 } else { 1 },", "if reaches_next_binade { 1 } else { 0 },"),
    ("discard-precision-offset", "executable", "                false,\n                prec,", "                false,\n                0,"),
    ("discard-exact-exponent", "weaker-output", "BoundInfo::with_sign_msd(sign, msd, exact_msd)", "BoundInfo::with_sign_msd(sign, msd, false)"),
    ("discard-exponent", "weaker-output", "BoundInfo::with_sign_msd(sign, msd, exact_msd)", "BoundInfo::with_sign_msd(sign, None, exact_msd)"),
    ("force-positive-sign", "executable", "let sign = appr.sign();", "let sign = Sign::Plus;"),
    ("power-of-two-test-reversed", "executable", "trailing_zeros != Some(magnitude_bits - 1)", "trailing_zeros == Some(magnitude_bits - 1)"),
    ("exact-all-ones-reversed", "executable", "                && !reaches_next_binade;", "                && reaches_next_binade;"),
    ("missing-normalization", "premise", "requires appr.metadata_normalized(), appr.metadata_storage_bits() <= u64::MAX,", "requires appr.metadata_storage_bits() <= u64::MAX,"),
    ("missing-storage-bound", "premise", "requires appr.metadata_normalized(), appr.metadata_storage_bits() <= u64::MAX,", "requires appr.metadata_normalized(),"),
]


def files(root):
    return {str(p.relative_to(root)): sha(p) for p in sorted(root.rglob("*")) if p.is_file()}


def main():
    assert not OUT.exists() and not WORK.exists()
    checkpoint_path = E / "typed-api-prototype-v2440-state.json"
    assert sha(checkpoint_path) == "1647e9f980a5d12f117bcecb2a5ccc3c561018e342fce83b15f6ead13c550ef0"
    checkpoint = read(checkpoint_path)
    sources = snapshot(E / checkpoint["source_record"])
    main_guard = snapshot(E / "candidate-preparation/main-hashes.json.gz")
    preview = read(PREVIEW / "protocol.json")
    base, base_protocol = read(BASE / "result.json"), read(BASE / "protocol.json")
    assert base["exit_code"] == 0 and base["verification_results"]["verified"] == 737
    assert base["verification_results"]["errors"] == 0 and not (BASE / "stderr.log").read_bytes()
    assert sha(BASE / "protocol.json") == base["protocol_sha256"]
    assert sha(PREVIEW / "protocol.json") == base_protocol["preview_protocol_sha256"]
    cargo, cargo_protocol = read(CARGO / "result.json"), read(CARGO / "protocol.json")
    dependencies = read(CARGO / "dependency-reports/dependencies.json")
    fixture = Path(cargo_protocol["work"])
    metadata = fixture / "artifacts/num_bigint.vir"
    tool = Path(base_protocol["command"][0]).parent

    def audit():
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in main_guard.items())
        assert all(sha(PREVIEW / n) == h for n, h in preview["preview_files_sha256"].items())
        assert all(sha(fixture / n) == h for n, h in cargo_protocol["work_files_sha256"].items())
        assert all(sha(Path(n)) == h for n, h in dependencies["artifacts"].items())
        assert sha(metadata) == cargo["export_sha256"]
        assert all(sha(tool / n) == h for n, h in base_protocol["tool_sha256"].items())

    audit()
    original = (PREVIEW / SHARED).read_text()
    variants = [("baseline", "baseline", original)]
    for name, kind, before, after in MUTATIONS:
        assert original.count(before) == 1, name
        variants.append((name, kind, original.replace(before, after)))
    OUT.mkdir()
    (OUT / "inputs").mkdir()
    WORK.mkdir()
    jobs = []
    for name, kind, source in variants:
        copy = WORK / name
        shutil.copytree(PREVIEW, copy)
        (copy / SHARED).write_text(source)
        bounds = (copy / "computable_bounds.rs").read_text()
        assert bounds.count(str(PREVIEW / SHARED)) == 1
        (copy / "computable_bounds.rs").write_text(bounds.replace(str(PREVIEW / SHARED), str(copy / SHARED)))
        root = (copy / "proof.rs").read_text()
        assert root.count(str(PREVIEW / "computable_bounds.rs")) == 1
        (copy / "proof.rs").write_text(root.replace(str(PREVIEW / "computable_bounds.rs"), str(copy / "computable_bounds.rs")))
        (OUT / "inputs" / (name + ".rs")).write_text(source)
        command = base_protocol["command"][:-1] + ["--verify-module", "computable_bounds",
            "--verify-function", "bound_from_approx", str(copy / "proof.rs")]
        jobs.append({"name": name, "kind": kind, "work": str(copy), "work_files_sha256": files(copy), "command": command})
    protocol = {"status": "Frozen before one positive and twelve negative producer controls",
        "runner_sha256": sha(Path(__file__)), "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"],
        "base_result_sha256": sha(BASE / "result.json"), "base_protocol_sha256": sha(BASE / "protocol.json"),
        "preview_protocol_sha256": sha(PREVIEW / "protocol.json"), "mutations": MUTATIONS, "jobs": jobs,
        "baseline_allowed_stderr": "Only blank lines and lines starting with 'note: verifying '",
        "scope": "The complete 737-unit preview passed before these controls. Select only the actual new associated function; keep all other production/model bodies and the actual Cargo BigInt/BigUint import unchanged. Seven executable errors, three weakened outputs and two removed representation/range premises must fail real proof obligations. Proof annotations remain present so these controls qualify the full implementation plus annotations, not a claim about minimal postconditions. No SOURCE edit, approximation-existence proof, constructor-invariant proof, whole-node proof, runtime admission or retained-coverage increase occurs."}
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
        assert all(files(Path(j["work"])) == j["work_files_sha256"] for j in jobs)
        audit()
        result = {"protocol_sha256": sha(OUT / "protocol.json"), "runs": rows,
            "complete": len(rows) == len(jobs) and all(r["gate_passed"] for r in rows),
            "source_unchanged": True, "main_unchanged": True, "preview_unchanged": True,
            "dependency_inputs_unchanged": True, "tooling_unchanged": True,
            "retained_coverage_changed": False, "runtime_admitted": False}
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        if not passed:
            break


if __name__ == "__main__":
    main()
