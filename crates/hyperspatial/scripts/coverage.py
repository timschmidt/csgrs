#!/usr/bin/env python3
"""Measure instrumented Hyperspatial production sources with rustup LLVM tools.

Hyperspatial library and integration tests run with all features. A separate
denominator measures the new Hyperreal inverse's production executable lines,
using its own integration and boundary tests as well as downstream executions.
Stable Rust does not instrument branch coverage here.
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
native_arguments = ["cargo", "test", "--locked", "-p", "hyperreal", "--no-default-features",
                    "--features", "serde,dispatch-trace"]
commands = [arguments,
            native_arguments + ["--test", "inverse_isometric_latitude", "--test", "negative_exp_tail", "--test", "elliptic_integrals"],
            native_arguments + ["--lib"]]
with tempfile.TemporaryDirectory(prefix="profiles-", dir=target) as profiles:
    environment["LLVM_PROFILE_FILE"] = str(Path(profiles) / "%p-%m.profraw")
    executables = set()
    for command in commands:
        artifacts = subprocess.check_output(
            command + ["--no-run", "--message-format=json"], cwd=root, env=environment, text=True,
        )
        for line in artifacts.splitlines():
            event = json.loads(line)
            if event.get("reason") == "compiler-artifact" and event.get("executable") and event["profile"]["test"]:
                executables.add(event["executable"])
        subprocess.run(command, cwd=root, env=environment, check=True)
    if not executables:
        raise SystemExit("Cargo produced no instrumented test executables")
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
    "commands": commands,
    "source_sha256": {
        str(path.relative_to(crate)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted((crate / "src").rglob("*.rs"))
    },
}
# As in Hyperreal's coverage script, exclude a trailing inline test module
# from this physical executable-line denominator. Executions from unit tests
# still cover production code. This is not whole-Hyperreal coverage.
native_sources = [root / "crates/hyperreal/src/computable/approximation/inverse_isometric_latitude.rs",
                  root / "crates/hyperreal/src/real/arithmetic/isometric_latitude.rs"]
native_lines = {str(p): {"count": 0, "covered": 0,
                        "test_boundary": next((i for i, line in enumerate(p.read_text().splitlines(), 1)
                                               if line.strip() == "#[cfg(test)]"), None)}
                for p in native_sources}
annotated = subprocess.check_output(
    [str(llvm / "llvm-cov"), "show", *cov_args, *map(str, native_sources),
     "--show-instantiations=false", "--show-line-counts-or-regions"], text=True)
active = None
for line in annotated.splitlines():
    if line.endswith(":") and line[:-1] in native_lines:
        active = native_lines[line[:-1]]
        continue
    fields = line.split("|", 2)
    if active is None or len(fields) != 3 or not fields[0].strip().isdigit():
        continue
    number = int(fields[0].strip())
    count = fields[1].strip()
    if not count or (active["test_boundary"] is not None and number >= active["test_boundary"]):
        continue
    active["count"] += 1
    active["covered"] += count != "0"
native_total = {metric: sum(row[metric] for row in native_lines.values())
                for metric in ("count", "covered")}
if not native_total["count"] or any(not row["count"] for row in native_lines.values()):
    raise SystemExit("LLVM reported no native inverse executable lines")
native_total["percent"] = 100 * native_total["covered"] / native_total["count"]
report["native_inverse"] = {
    "scope": "only the two new Hyperreal implementation files; physical executable lines, excluding trailing test modules",
    "regions_functions_branches": "not measured separately for this production-only subset",
    "files": {str(Path(p).relative_to(root)): row for p, row in native_lines.items()},
    "physical_executable_lines": native_total,
    "source_sha256": {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in native_sources},
}
# The repaired exponential paths predate this crate. Report their complete
# function denominators separately, without rebranding this as whole-native
# coverage or changing the existing Hyperspatial/inverse gates.
exp_ranges = {}
for path, start_marker, end_marker in [
    (root / "crates/hyperreal/src/computable/approximation/exp_sqrt.rs",
     "fn exp(", "fn expm1("),
    (root / "crates/hyperreal/src/real/arithmetic/elementary_functions.rs",
     "    pub fn exp(", "    /// Raise two to this Real exponent."),
]:
    lines = path.read_text().splitlines()
    start = next(i for i, line in enumerate(lines, 1) if line.startswith(start_marker))
    end = next(i for i, line in enumerate(lines, 1) if i > start and line.startswith(end_marker))
    exp_ranges[str(path)] = {"start": start, "end_exclusive": end, "count": 0, "covered": 0}
exp_annotated = subprocess.check_output(
    [str(llvm / "llvm-cov"), "show", *cov_args, *exp_ranges,
     "--show-instantiations=false", "--show-line-counts-or-regions"], text=True)
active = None
for line in exp_annotated.splitlines():
    if line.endswith(":") and line[:-1] in exp_ranges:
        active = exp_ranges[line[:-1]]
        continue
    fields = line.split("|", 2)
    if active is None or len(fields) != 3 or not fields[0].strip().isdigit():
        continue
    number, count = int(fields[0].strip()), fields[1].strip()
    if count and active["start"] <= number < active["end_exclusive"]:
        active["count"] += 1
        active["covered"] += count != "0"
if any(not row["count"] for row in exp_ranges.values()):
    raise SystemExit("LLVM reported no repaired exponential function lines")
report["native_exponential"] = {
    "scope": "complete Computable exp approximation and Real::exp functions; physical executable lines",
    "regions_functions_branches": "not measured separately for this subset",
    "files": {str(Path(p).relative_to(root)): row for p, row in exp_ranges.items()},
    "source_sha256": {str(Path(p).relative_to(root)): hashlib.sha256(Path(p).read_bytes()).hexdigest()
                      for p in exp_ranges},
}
output = target / "coverage.json"
elliptic_sources = [root / "crates/hyperreal/src" / part / "elliptic_integrals.rs"
                    for part in ("computable/approximation", "computable/node", "real/arithmetic")]
elliptic_files = {
    str(Path(file["filename"]).resolve().relative_to(root)): file["summary"]
    for dataset in export["data"] for file in dataset["files"]
    if Path(file["filename"]).resolve() in elliptic_sources
}
if len(elliptic_files) != len(elliptic_sources):
    raise SystemExit("LLVM did not report every native elliptic implementation file")
elliptic_totals = {metric: {key: sum(file[metric][key] for file in elliptic_files.values())
                           for key in ("count", "covered")}
                   for metric in ("lines", "regions", "functions")}
for row in elliptic_totals.values():
    row["percent"] = 100 * row["covered"] / row["count"] if row["count"] else None
report["native_elliptic"] = {
    "scope": "three native elliptic implementation files; shared dispatch and structural matching excluded",
    "files": elliptic_files, "totals": elliptic_totals,
    "source_sha256": {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in elliptic_sources},
}
output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
print(json.dumps(totals, indent=2))
print("Native inverse production lines:", json.dumps(native_total))
print(f"Full production denominator and per-file results: {output}")
if totals["lines"]["percent"] < 95:
    raise SystemExit("Production line coverage is below the 95% gate; the 100% target remains open")
if native_total["percent"] < 95:
    raise SystemExit("Native inverse production line coverage is below the 95% gate")
if elliptic_totals["lines"]["percent"] < 95:
    raise SystemExit("Native elliptic production line coverage is below the 95% gate")
