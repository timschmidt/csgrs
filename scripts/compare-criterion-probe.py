#!/usr/bin/env python3
"""Retain alternating Criterion runs of two existing benchmark executables.

This compares selected workloads. It preserves Criterion samples and estimates;
it does not add exact-output assertions to the benchmark being executed.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("filter")
    parser.add_argument("output", type=Path)
    parser.add_argument("--pairs", type=int, default=5)
    args = parser.parse_args()
    if args.pairs <= 0:
        parser.error("pairs must be positive")
    programs = {"before": args.baseline.resolve(), "after": args.candidate.resolve()}
    args.output.mkdir(parents=True, exist_ok=False)
    cpu = min(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None
    settings = [args.filter, "--bench", "--sample-size", "30", "--warm-up-time", "0.2", "--measurement-time", "0.5", "--noplot"]
    rows = []
    expected_names = None

    def pin_cpu():
        os.sched_setaffinity(0, {cpu})

    for pair in range(args.pairs):
        order = ["before", "after"] if pair % 2 == 0 else ["after", "before"]
        for implementation in order:
            directory = args.output / f"{pair:02d}-{implementation}"
            directory.mkdir()
            with (directory / "process.log").open("w") as stream:
                subprocess.run(
                    [str(programs[implementation]), *settings],
                    env=dict(os.environ, CRITERION_HOME=str((directory / "criterion").resolve()), HYPERSOLVE_SKIP_BENCHMARK_REPORTS="1"),
                    stdout=stream, stderr=subprocess.STDOUT, check=True,
                    preexec_fn=pin_cpu if cpu is not None else None,
                )
            measurements = []
            for identity_file in sorted(directory.glob("criterion/**/new/benchmark.json")):
                identity = json.loads(identity_file.read_text())
                estimates = json.loads(identity_file.with_name("estimates.json").read_text())
                measurements.append({"workload": identity["full_id"], "mean_ns": estimates["mean"]["point_estimate"]})
            names = [item["workload"] for item in measurements]
            if not names:
                raise RuntimeError(f"no benchmark measurements in {directory}")
            if expected_names is None:
                expected_names = names
            if names != expected_names:
                raise RuntimeError(f"workload selection changed in {directory}")
            rows.extend(dict(item, pair=pair, implementation=implementation) for item in measurements)
            print(f"Completed {pair + 1}/{args.pairs}: {implementation}", flush=True)

    summary = {}
    for name in expected_names:
        medians = {implementation: statistics.median(
            row["mean_ns"] for row in rows if row["workload"] == name and row["implementation"] == implementation
        ) for implementation in programs}
        medians["change_percent"] = (medians["after"] / medians["before"] - 1) * 100
        summary[name] = medians
    metadata = {
        "scope": "Selected Criterion workloads; median of each process's mean estimate, in nanoseconds.",
        "limitations": "Existing benchmark setup assertions only; exact-output tests are separate. No statistical guarantee or full-suite conclusion follows from these process medians.",
        "pairs": args.pairs,
        "arguments": settings,
        "cpu_affinity": cpu,
        "host": {"system": platform.system(), "release": platform.release(), "machine": platform.machine()},
        "rustc": subprocess.check_output(["rustc", "--version"], text=True).strip(),
        "executables": {name: {"sha256": hashlib.sha256(program.read_bytes()).hexdigest(), "bytes": program.stat().st_size} for name, program in programs.items()},
        "measurements": rows,
        "median_ns_per_operation": summary,
        "files": {str(path.relative_to(args.output)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(args.output.rglob("*")) if path.is_file()},
    }
    (args.output / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
