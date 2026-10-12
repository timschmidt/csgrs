"""Reject executable and representation-view mutations in the actual BigInt module."""
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
BASE = E / "bigint-metadata-export-v2450"
OUT = E / "bigint-metadata-guards-v2451"
WORK = ROOT / "target/bigint-metadata-guards-v2451"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
snapshot = lambda p: json.loads(gzip.decompress(p.read_bytes()))

MUTATIONS = [
    ("sign-always-zero", "executable", "    pub fn sign(", "        self.sign\n", "        Sign::NoSign\n"),
    ("sign-reversed", "executable", "    pub fn sign(", "        self.sign\n", "        match self.sign { Sign::Minus => Sign::Plus, Sign::Plus => Sign::Minus, Sign::NoSign => Sign::NoSign }\n"),
    ("magnitude-view-offset", "representation", "    pub closed spec fn metadata_magnitude(", "self.data.metadata_value()", "self.data.metadata_value() + 1"),
    ("storage-view-offset", "representation", "    pub closed spec fn metadata_storage_bits(", "self.data.metadata_storage_bits()", "self.data.metadata_storage_bits() + 1"),
    ("negative-denotation-reversed", "representation", "    pub closed spec fn metadata_value(", "Sign::Minus => -(self.data.metadata_value() as int),", "Sign::Minus => self.data.metadata_value() as int,"),
    ("missing-sign-consistency", "representation", "    pub closed spec fn metadata_normalized(", "(self.sign == Sign::NoSign <==> self.data.metadata_value() == 0)", "true"),
    ("missing-magnitude-normalization", "representation", "    pub closed spec fn metadata_normalized(", "self.data.metadata_normalized()\n            && ", ""),
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
    assert all(sha(SOURCE / n) == h for n, h in sources.items())
    assert all(sha(ROOT / n) == h for n, h in main_guard.items())
    base_result, base_protocol = read(BASE / "result.json"), read(BASE / "protocol.json")
    assert base_result["exit_code"] == 0 and base_result["verification_results"]["verified"] == 453
    vendor = Path(base_protocol["vendor"])
    assert files(vendor) == base_protocol["vendor_files_sha256"]
    tool = Path(base_protocol["command"][0]).parent
    assert all(sha(tool / n) == h for n, h in base_protocol["tool_sha256"].items())
    original = (vendor / "src/bigint.rs").read_text()
    variants = [("baseline", "baseline", original)]
    for name, kind, marker, before, after in MUTATIONS:
        start = original.index(marker)
        end = original.index("\n    }", start) + len("\n    }")
        body = original[start:end]
        assert body.count(before) == 1, name
        variants.append((name, kind, original[:start] + body.replace(before, after) + original[end:]))
    OUT.mkdir()
    (OUT / "inputs").mkdir()
    WORK.mkdir()
    prefix = []
    skip = False
    for argument in base_protocol["command"][:-1]:
        if skip:
            skip = False
            continue
        if argument in {"--export", "--out-dir"}:
            skip = True
            continue
        if argument != "--compile":
            prefix.append(argument)
    jobs = []
    for name, kind, text in variants:
        copy = WORK / name
        shutil.copytree(vendor, copy)
        (copy / "src/bigint.rs").write_text(text)
        (OUT / "inputs" / (name + ".bigint.rs")).write_text(text)
        inputs = files(copy)
        assert inputs.keys() == base_protocol["vendor_files_sha256"].keys()
        assert all(h == base_protocol["vendor_files_sha256"][n] for n, h in inputs.items() if n != "src/bigint.rs")
        jobs.append({"name": name, "kind": kind, "vendor": str(copy), "vendor_files_sha256": inputs,
                     "command": prefix + ["--verify-module", "bigint", str(copy / "src/lib.rs")]})
    protocol = {"status": "Frozen before eight actual-module proof controls",
        "runner_sha256": sha(Path(__file__)), "base_protocol_sha256": sha(BASE / "protocol.json"),
        "base_result_sha256": sha(BASE / "result.json"), "retained_checkpoint_sha256": sha(checkpoint_path),
        "source_record_sha256": checkpoint["source_record_sha256"], "mutations": MUTATIONS, "jobs": jobs,
        "scope": "The complete annotated baseline already passes 453 units. This fresh baseline and seven controls verify the actual bigint module, including the two getters and generated Sign clone. Two controls alter executable sign behavior; five alter denotation, magnitude/range views or the normalized predicate. Every rejection must be a real failed proof obligation, not a frontend error. Other source modules are byte-identical to the fully proved baseline. No runtime, SOURCE, main or retained-coverage change follows."}
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
                                                                 "precondition not satisfied", "requires not satisfied"])
        if job["kind"] == "baseline":
            passed = run.returncode == 0 and parsed.get("verified") == 3 and not run.stderr
        else:
            passed = run.returncode != 0 and proof_failure and not parsed.get("encountered-vir-error", True)
        row = {"name": job["name"], "kind": job["kind"], "exit_code": run.returncode,
               "seconds": time.monotonic() - start, "verification_results": parsed,
               "gate_passed": passed, "real_proof_failure": proof_failure,
               "stdout_sha256": sha(stdout), "stderr_sha256": sha(stderr)}
        rows.append(row)
        assert files(Path(job["vendor"])) == job["vendor_files_sha256"]
        assert files(vendor) == base_protocol["vendor_files_sha256"]
        assert all(sha(SOURCE / n) == h for n, h in sources.items())
        assert all(sha(ROOT / n) == h for n, h in main_guard.items())
        assert all(sha(tool / n) == h for n, h in base_protocol["tool_sha256"].items())
        assert all(sha(Path(n)) == h for n, h in base_protocol["dependency_artifacts_sha256"].items())
        result = {"protocol_sha256": sha(OUT / "protocol.json"), "runs": rows,
            "complete": len(rows) == len(jobs) and all(r["gate_passed"] for r in rows),
            "source_unchanged": True, "main_unchanged": True, "base_unchanged": True, "tooling_unchanged": True,
            "retained_coverage_changed": False, "runtime_admitted": False}
        (OUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(row), flush=True)
        if not passed:
            break


if __name__ == "__main__":
    main()
