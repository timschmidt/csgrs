#!/usr/bin/env python3
"""Build and compare the pinned, separate-process validated ODE adapters."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import statistics
import subprocess
import time

from validate import CASES, validate

CAPD_REVISION = "37057e5899bccaa6cd5701057b7e6ea5e232ef8b"


def validate_samples(rows, solver, cases, iterations, samples):
    """Require every completed case/mode/sample exactly once before aggregation."""
    expected = {(case, mode, sample) for case in cases
                for mode in ("reuse", "fresh") for sample in range(samples)}
    assert len(rows) == len(expected), "missing/duplicate timing records"
    assert {(r["case"], r["mode"], r["sample"]) for r in rows} == expected
    for row in rows:
        assert row["kind"] == "timing" and row["solver"] == solver
        assert type(row["sample"]) is int and type(row["iterations"]) is int
        assert row["iterations"] == iterations
        assert type(row["total_ns"]) is int and row["total_ns"] > 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capd-source", required=True, type=Path)
    parser.add_argument("--capd-build", required=True, type=Path)
    parser.add_argument("--cargo-target", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--cpu", type=int, default=min(os.sched_getaffinity(0)))
    parser.add_argument("--iterations", type=int, default=100)
    parser.add_argument("--samples", type=int, default=5)
    args = parser.parse_args()
    assert 1 <= args.iterations <= 100000 and 1 <= args.samples <= 100
    assert args.cpu in os.sched_getaffinity(0)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    here = Path(__file__).resolve().parent
    repo = here.parent.parent
    commands = []

    def run(label, command, env=None, timeout=600):
        start = time.monotonic()
        with (output / f"{label}.log").open("w") as log:
            result = subprocess.run(command, cwd=repo, env=env, stdout=log,
                                    stderr=subprocess.STDOUT, timeout=timeout)
        commands.append({"label": label, "command": list(map(str, command)),
                         "cwd": str(repo), "seconds": time.monotonic() - start,
                         "exit_code": result.returncode})
        (output / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
        assert result.returncode == 0, f"{label}: see {output / (label + '.log')}"

    revision = subprocess.check_output(
        ["git", "-C", str(args.capd_source), "rev-parse", "HEAD"], text=True).strip()
    assert revision == CAPD_REVISION
    assert not subprocess.check_output(["git", "-C", str(args.capd_source), "diff", "HEAD"])
    flags = shlex.split(subprocess.check_output(
        [str(args.capd_build.resolve() / "bin/capd-config"), "--cflags", "--libs"], text=True))
    assert "-frounding-math" in flags and "-D__USE_NATIVE__" in flags
    # Keep upstream compiler options; mark only external headers as system headers.
    flags = ["-isystem" + flag[2:] if flag.startswith("-I") else flag for flag in flags]
    capd = args.cargo_target.resolve() / "capd-comparison"
    env = os.environ | {"CARGO_TARGET_DIR": str(args.cargo_target.resolve()),
                        "CARGO_BUILD_JOBS": "16", "CARGO_NET_OFFLINE": "true"}
    run("hyper-build", ["cargo", "+1.95.0", "build", "--locked", "--release",
                        "--manifest-path", str(here / "Cargo.toml")], env=env)
    run("capd-build", ["g++", "-Wall", "-Wextra", "-Werror", str(here / "capd.cpp"),
                       *flags, "-o", str(capd)])
    hyper = args.cargo_target.resolve() / "release/validated-dynamics-comparison"
    executables = {"hyperdynamics": hyper, "capd": capd}
    metadata = {"capd_revision": revision, "cpu": args.cpu, "platform": platform.platform(),
                "source_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, cwd=repo).strip(),
                "environment": {k: env[k] for k in ("CARGO_TARGET_DIR", "CARGO_BUILD_JOBS", "CARGO_NET_OFFLINE")},
                "source_sha256": {str(p.relative_to(here)): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in here.rglob("*") if p.is_file() and "__pycache__" not in p.parts},
                "binaries": {name: {"path": str(p), "bytes": p.stat().st_size,
                                     "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                             for name, p in executables.items()}}
    for tool, command in [("rustc", ["rustc", "+1.95.0", "-Vv"]), ("g++", ["g++", "--version"]),
                          ("cpu_info", ["lscpu"])]:
        metadata[tool] = subprocess.check_output(command, text=True)
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    accepted = {}
    for name, binary in executables.items():
        run(name + "-preflight", [str(binary)])
        rows = [json.loads(line) for line in (output / (name + "-preflight.log")).read_text().splitlines()]
        accepted[name] = rows
        (output / (name + "-validation.json")).write_text(json.dumps(validate(rows), indent=2) + "\n")
    for process in ("rustc", "cc1plus"):
        assert subprocess.run(["pgrep", "-x", process], stdout=subprocess.DEVNULL).returncode == 1
    timing = []
    for block, name in enumerate(("hyperdynamics", "capd", "capd", "hyperdynamics")):
        label = f"{block}-{name}"
        run(label, ["setarch", platform.machine(), "-R", "taskset", "-c", str(args.cpu),
                    str(executables[name]), str(args.iterations), str(args.samples)])
        rows = [json.loads(line) for line in (output / (label + ".log")).read_text().splitlines()]
        certificates = [row for row in rows if row["kind"] == "certificate"]
        assert certificates == accepted[name], "measurement changed certified outputs"
        validate(certificates)
        samples = [row | {"block": block} for row in rows if row["kind"] == "timing"]
        validate_samples(samples, name, [r["case"] for r in accepted[name]], args.iterations, args.samples)
        timing.extend(samples)
    (output / "raw-timing.json").write_text(json.dumps(timing, indent=2) + "\n")
    summary = []
    for name in executables:
        for case in [r["case"] for r in accepted[name]]:
            for mode in ("reuse", "fresh"):
                values = [r["total_ns"] / r["iterations"] for r in timing
                          if (r["solver"], r["case"], r["mode"]) == (name, case, mode)]
                summary.append({"solver": name, "case": case, "mode": mode,
                                "samples": len(values), "median_ns": statistics.median(values),
                                "min_ns": min(values), "max_ns": max(values)})
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Validated all {len(CASES) * len(executables)} outcomes; saved {len(timing)} timing samples to {output}")


if __name__ == "__main__":
    main()
