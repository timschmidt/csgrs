"""Freeze the current-main source plus the unchanged index in new temporary storage."""
import gzip
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
V = Path("/dev/shm/csgrs-circle-support-consumers-v1")
S = V / "candidate-source-v1"
O = V / "preparation-v1"
OLD = W.parent / "circle-support-corpus"
BASE = "67a6ba54f1bf25f0ef495972238bba0a0102137e"


def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()


def read(p):
    return json.loads(Path(p).read_text())


def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2) + "\n")


assert subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip() == BASE
subprocess.run(["git", "diff", "--quiet"], cwd=ROOT, check=True)
subprocess.run(["git", "diff", "--cached", "--quiet"], cwd=ROOT, check=True)
assert V.is_dir() and V.stat().st_uid == os.getuid() and set(p.name for p in V.iterdir()) == {"write-control"}
parent = read(ROOT / "docs/evidence/2026-10-17-circle-support-corpus/checkpoint-v1.json")
assert all(sha(ROOT / p) == h for p, h in parent["evidence_files_sha256"].items())
assert all(sha(ROOT / p) == h for p, h in parent["implementation_sha256"].items())
with gzip.open(OLD / "preparation-v1/main-input-hashes.json.gz", "rt") as f:
    previous_main = json.load(f)
with gzip.open(OLD / "preparation-v2/source-hashes.json.gz", "rt") as f:
    previous_candidate = json.load(f)
main = {p: sha(ROOT / p) for p in previous_main}
changed_main = {p for p, h in main.items() if previous_main[p] != h}
assert changed_main == set(parent["implementation_sha256"]) | {"docs/properties-status.json"}
proposal = read(OLD / "preparation-v2/result.json")["proposal_sha256"]
assert all(sha(OLD / "candidate-source-v2" / p) == h for p, h in previous_candidate.items())
O.mkdir(exist_ok=False)
S.mkdir(exist_ok=False)
for p in main:
    dest = S / p
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / p, dest)
for p, h in proposal.items():
    dest = S / p
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(OLD / "candidate-source-v2" / p, dest)
    assert sha(dest) == h
current = {p: sha(S / p) for p in set(main) | set(proposal)}
assert set(current) == set(previous_candidate)
differences = {p: {"previous": previous_candidate[p], "current": h}
               for p, h in current.items() if previous_candidate[p] != h}
assert set(differences) == {"docs/properties-status.json"}
with (O / "format.stdout").open("wb") as out, (O / "format.stderr").open("wb") as err:
    result = subprocess.run(["cargo", "fmt", "--all", "--", "--check"], cwd=S, stdout=out, stderr=err)
assert result.returncode == 0
for name, value in [("main-input-hashes", main), ("source-hashes", current)]:
    with gzip.open(O / (name + ".json.gz"), "wt") as f:
        json.dump(value, f, sort_keys=True)
dump(O / "environment.json", {
    "quota": subprocess.check_output(["quota", "-w", "-v", "-p", "--show-mntpoint", "--hide-device", "-f", "/dev/shm"], text=True),
    "filesystems": subprocess.check_output(["df", "-B1", str(ROOT), "/dev/shm", "/tmp", "/run/user/1000"], text=True),
    "mount": subprocess.check_output(["findmnt", "-T", str(V), "-o", "TARGET,SOURCE,FSTYPE,OPTIONS"], text=True),
    "memory": Path("/proc/meminfo").read_text(),
    "rustc": subprocess.check_output(["rustc", "-vV"], text=True),
    "cargo": subprocess.check_output(["cargo", "-V"], text=True),
})
dump(O / "result.json", {
    "complete": True, "base_commit": BASE, "source": str(S), "storage_root": str(V),
    "source_inputs": len(current), "source_hashes_sha256": sha(O / "source-hashes.json.gz"),
    "main_input_hashes_sha256": sha(O / "main-input-hashes.json.gz"),
    "proposal_sha256": proposal, "changes_from_qualified_e40_candidate": differences,
    "implementation_and_tests_identical_to_e40_candidate": True,
    "format_exit_code": result.returncode, "main_sources_modified": False, "cache_admitted": False,
    "input_sha256": {str(p): sha(p) for p in [
        Path(__file__), W / "contract-v1.md", V / "write-control",
        OLD / "preparation-v2/result.json", OLD / "preparation-v2/source-hashes.json.gz",
        ROOT / "docs/evidence/2026-10-17-circle-support-corpus/checkpoint-v1.json",
    ]},
})
assert all(sha(ROOT / p) == h for p, h in main.items())
print(json.dumps({"source_inputs": len(current), "only_difference_from_e40": list(differences),
                  "storage_root": str(V), "main_sources_modified": False}))
