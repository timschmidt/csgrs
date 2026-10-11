#!/usr/bin/env python3
"""Run algebraic budget checks and preserved replay/cost workloads sequentially."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[2]
EXPECTED = "8cf89a1bf55963e29f8aa939eefdd964b5754e7a56bd0c156f718b777cac81e8"
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def validate(phase, *arguments):
    subprocess.run([sys.executable, str(EVIDENCE / "validate_candidate_v2.py"), phase, *arguments], cwd=ROOT, check=True)
    path = EVIDENCE / phase
    data = read(path / "result.json")
    assert data["exit_code"] == 0 and data["source_unchanged"] and data["main_unchanged"], (phase, data)
    assert sha(path / "source-hashes.json.gz") == EXPECTED


def replay(script, *arguments):
    subprocess.run([sys.executable, str(EVIDENCE / script), *arguments], cwd=ROOT, check=True)
    data = read(EVIDENCE / arguments[1] / "summary.json")
    assert data["source_hashes_sha256"] == EXPECTED and data["guards_unchanged"]


validate("typed-api-prototype-v513-stack-clippy", "clippy", "--release", "-p", "hyperreal", "-p", "hyperlimit", "-p", "hyperlattice", "-p", "hypercurve", "-p", "hyperdrc", "--all-targets", "--all-features", "--", "-D", "warnings")
build = "typed-api-prototype-v514-replay-build"
validate(build, "build", "--release", "-p", "hyperreal", "-p", "hypercurve", "--all-features")
for phase, mode in [("typed-api-prototype-v515-algebraic-budget-replay", "replay"), ("typed-api-prototype-v516-algebraic-cost", "cost")]:
    replay("reproduce_algebraic_budget_v2.py", build, phase, "typed-api-prototype-v512-algebraic-budget-tests", mode)
replay("reproduce_evaluation_budget_cost.py", build, "typed-api-prototype-v517-default-evaluation-cost")
print("Algebraic budget focused validation, external replay and two cost corpora passed.", flush=True)
