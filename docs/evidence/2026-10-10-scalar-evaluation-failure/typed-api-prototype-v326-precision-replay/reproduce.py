"""Retain adjacent precision-boundary failures beyond the original endpoint cases."""
import gzip
import hashlib
import json
from pathlib import Path
import resource
import subprocess
import time
import sys

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / sys.argv[3]
OUT.mkdir()
SOURCE = ROOT / "target/scalar-evaluation-candidate/source"
BUILD = EVIDENCE / sys.argv[1]
PROBE = ROOT / "target" / sys.argv[2] / "probe"
HASHES = json.loads(gzip.decompress((BUILD / "source-hashes.json.gz").read_bytes()))
GUARD = json.loads(gzip.decompress((EVIDENCE / "candidate-preparation/main-hashes.json.gz").read_bytes()))
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
probe_hash = sha(PROBE)


def verify():
    assert sha(PROBE) == probe_hash
    assert all(sha(SOURCE / p) == h for p, h in HASHES.items())
    assert all(sha(ROOT / p) == h for p, h in GUARD.items())


def cap():
    resource.setrlimit(resource.RLIMIT_AS, (192 * 1024 * 1024, 192 * 1024 * 1024))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


(OUT / "reproduce.py").write_bytes(Path(__file__).read_bytes())
verify()
rows = []
for family in ["one", "rational", "pi", "e", "sqrt_two", "ln_two", "sin_one", "cos_one", "pi_plus_e", "pi_times_e"]:
    for precision in [-(2**31) + 1, -(2**31) + 2, -(2**31) + 16, 2**31 - 2, 2**31 - 3, 2**31 - 17]:
        name = f"case-{len(rows)}"
        command = ["timeout", "--kill-after=2s", "3", str(PROBE), family, "control", "approx", str(precision)]
        start = time.monotonic()
        with (OUT / (name + ".stdout")).open("w") as stdout, (OUT / (name + ".stderr")).open("w") as stderr:
            result = subprocess.run(command, cwd=ROOT, stdout=stdout, stderr=stderr, preexec_fn=cap)
        receipts = [json.loads(line) for line in (OUT / (name + ".stdout")).read_text().splitlines()]
        # Every family is positive and lies in (1/2, 10). At the positive
        # boundary, 0 or 1 obeys the one-unit contract. At the negative
        # boundary a <=4096-bit integer cannot approximate the scaled value.
        def valid(receipt):
            if receipt["error"] is not None:
                return receipt["result"] is None
            if precision > 0:
                return receipt["result"] in ["0", "1"]
            return receipt["result"] is None and receipt["bits"] > 2**31 - 20

        rows.append({
            "name": name, "family": family, "precision": precision,
            "command": command, "exit_code": result.returncode,
            "seconds": time.monotonic() - start, "timeout_seconds": 3,
            "address_space_cap_mib": 192, "receipts": receipts,
            "valid": result.returncode == 0 and len(receipts) == 2 and all(map(valid, receipts)),
        })
        (OUT / "receipts.json").write_text(json.dumps(rows, indent=2) + "\n")
    verify()
    print(family, "completed", flush=True)
verify()
summary = {
    "status": "exploratory precision audit of unadmitted prototype",
    "source_record": str(BUILD / "command.json"),
    "source_hashes_sha256": sha(BUILD / "source-hashes.json.gz"),
    "probe_sha256": probe_hash, "guards_unchanged": True,
    "cases": len(rows), "valid_cases": sum(x["valid"] for x in rows),
    "nonzero_exit_cases": [x["name"] for x in rows if x["exit_code"] != 0],
    "invalid_numeric_cases": [x["name"] for x in rows if x["exit_code"] == 0 and not x["valid"]],
    "artifacts_sha256": {p.name: sha(p) for p in OUT.iterdir() if p.is_file()},
}
(OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print({k: summary[k] for k in ["cases", "valid_cases", "nonzero_exit_cases", "invalid_numeric_cases"]}, flush=True)
