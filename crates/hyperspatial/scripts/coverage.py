#!/usr/bin/env python3
"""Measure instrumented Hyperspatial production sources with rustup LLVM tools.

All library and integration tests run with all features. Dependencies, tests,
benchmarks, examples, and doctests are outside this denominator. Stable Rust
does not instrument branch coverage here; zero branch entries mean unmeasured.
"""

import json
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile

crate = Path(__file__).resolve().parents[1]
root = crate.parents[1]
rustc = subprocess.check_output(["rustc", "-vV"], text=True)
host = next(line.split(": ", 1)[1] for line in rustc.splitlines() if line.startswith("host:"))
sysroot = Path(subprocess.check_output(["rustc", "--print", "sysroot"], text=True).strip())
llvm = sysroot / "lib/rustlib" / host / "bin"
if not (llvm / "llvm-cov").exists() or not (llvm / "llvm-profdata").exists():
    raise SystemExit("Install coverage tools with: rustup component add llvm-tools-preview")
target = Path(os.environ.get("CARGO_TARGET_DIR", root / "target/hyperspatial-coverage")).resolve()
target.mkdir(parents=True, exist_ok=True)
environment = os.environ.copy()
environment["CARGO_TARGET_DIR"] = str(target)
environment["RUSTFLAGS"] = environment.get("RUSTFLAGS", "") + " -C instrument-coverage"
arguments = ["cargo", "test", "--locked", "-p", "hyperspatial", "--all-features", "--lib", "--tests"]
with tempfile.TemporaryDirectory(prefix="profiles-", dir=target) as profiles:
    environment["LLVM_PROFILE_FILE"] = str(Path(profiles) / "%p-%m.profraw")
    artifacts = subprocess.check_output(
        arguments + ["--no-run", "--message-format=json"], cwd=root, env=environment, text=True,
    )
    executables = set()
    for line in artifacts.splitlines():
        event = json.loads(line)
        if event.get("reason") == "compiler-artifact" and event.get("executable") and event["profile"]["test"]:
            executables.add(event["executable"])
    if not executables:
        raise SystemExit("Cargo produced no instrumented test executables")
    subprocess.run(arguments, cwd=root, env=environment, check=True)
    raw = sorted(Path(profiles).glob("*.profraw"))
    if not raw:
        raise SystemExit("Tests produced no instrumented profiles")
    merged = target / "hyperspatial.profdata"
    subprocess.run([str(llvm / "llvm-profdata"), "merge", "-sparse", *map(str, raw), "-o", str(merged)], check=True)

objects = sorted(executables)
cov_args = [objects[0], f"--instr-profile={merged}"]
for executable in objects[1:]:
    cov_args.extend(["--object", executable])
export = json.loads(subprocess.check_output(
    [str(llvm / "llvm-cov"), "export", "--summary-only", *cov_args], text=True,
))
files = {}
for dataset in export["data"]:
    for file in dataset["files"]:
        path = Path(file["filename"]).resolve()
        if path.is_relative_to(crate / "src"):
            files[str(path.relative_to(crate))] = file["summary"]
if not files:
    raise SystemExit("LLVM exported no Hyperspatial production source coverage")
totals = {}
for metric in ["lines", "regions", "functions"]:
    count = sum(file[metric]["count"] for file in files.values())
    covered = sum(file[metric]["covered"] for file in files.values())
    totals[metric] = {"count": count, "covered": covered, "percent": 100 * covered / count if count else None}
report = {
    "rustc": rustc, "features": "all-features", "profile": "test",
    "scope": "crates/hyperspatial/src; library and integration test executions",
    "excluded": ["dependencies", "tests", "benchmarks", "examples", "doctests"],
    "branch_coverage": "unmeasured (stable Rust instrumentation)",
    "files": files, "totals": totals,
    "source_sha256": {
        str(path.relative_to(crate)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted((crate / "src").rglob("*.rs"))
    },
}
output = target / "coverage.json"
output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
print(json.dumps(totals, indent=2))
print(f"Full production denominator and per-file results: {output}")
if totals["lines"]["percent"] < 95:
    raise SystemExit("Production line coverage is below the 95% gate; the 100% target remains open")
