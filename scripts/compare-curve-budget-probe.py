#!/usr/bin/env python3
"""Pair two bezier_algebraic_parameter executables and verify their reported evidence.

This probe measures the unbudgeted hot paths. Its checksums/counters do not
replace exact-output tests, allocation evidence, or the competitive suite.
"""

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import statistics
import subprocess


ROW = re.compile(
    r"(?P<name>\w+): (?P<iterations>\d+) iterations in "
    r"(?P<elapsed>[0-9.]+)(?P<unit>ns|µs|ms|s) \([^\n]+?/iter\), (?P<evidence>.+)"
)
NANOSECONDS = {"ns": 1, "µs": 1_000, "ms": 1_000_000, "s": 1_000_000_000}


def parse(output):
    rows = []
    names = set()
    for line in output.splitlines():
        match = ROW.fullmatch(line)
        if match is None or match["name"] in names:
            raise ValueError(f"unexpected or duplicate benchmark row: {line!r}")
        names.add(match["name"])
        rows.append({
            "workload": match["name"],
            "iterations": int(match["iterations"]),
            "elapsed_ns": float(match["elapsed"]) * NANOSECONDS[match["unit"]],
            "evidence": match["evidence"],
        })
    if not rows:
        raise ValueError("benchmark produced no workload rows")
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--samples", type=int, default=11)
    parser.add_argument("--warmup", type=int, default=2)
    args = parser.parse_args()
    if args.samples <= 0 or args.warmup < 0:
        parser.error("samples must be positive and warmup nonnegative")
    programs = {"before": args.baseline.resolve(), "after": args.candidate.resolve()}
    args.output.mkdir(parents=True, exist_ok=True)
    cpu = min(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None

    def pin_cpu():
        os.sched_setaffinity(0, {cpu})

    expected = None
    rows = []
    raw = []
    for pair in range(args.warmup + args.samples):
        order = ["before", "after"] if pair % 2 == 0 else ["after", "before"]
        for implementation in order:
            output = subprocess.check_output(
                [str(programs[implementation])], text=True,
                preexec_fn=pin_cpu if cpu is not None else None,
            )
            filename = f"{pair:02d}-{implementation}.log"
            (args.output / filename).write_text(output)
            raw.append({"log": filename, "sha256": hashlib.sha256(output.encode()).hexdigest()})
            measured = parse(output)
            evidence = [(row["workload"], row["iterations"], row["evidence"]) for row in measured]
            if expected is None:
                expected = evidence
            if evidence != expected:
                raise RuntimeError(f"workload or reported evidence changed in {filename}")
            if pair >= args.warmup:
                rows.extend(dict(row, implementation=implementation, sample=pair-args.warmup) for row in measured)

    with (args.output / "samples.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    medians = {}
    for workload, _, _ in expected:
        values = {}
        for implementation in programs:
            values[implementation] = statistics.median(
                row["elapsed_ns"] / row["iterations"] for row in rows
                if row["implementation"] == implementation and row["workload"] == workload
            )
        values["change_percent"] = (values["after"] / values["before"] - 1) * 100
        medians[workload] = values
    metadata = {
        "scope": "Existing unbudgeted curve parameter benchmark; all reported counts and evidence must match.",
        "limitations": "No active-budget throughput, allocator, full serialized geometry, or competitive-library comparison is claimed.",
        "warmup_pairs": args.warmup,
        "sample_pairs": args.samples,
        "cpu_affinity": cpu,
        "host": {"system": platform.system(), "release": platform.release(), "machine": platform.machine()},
        "rustc": subprocess.check_output(["rustc", "--version"], text=True).strip(),
        "executables": {name: {
            "sha256": hashlib.sha256(program.read_bytes()).hexdigest(),
            "bytes": program.stat().st_size,
        } for name, program in programs.items()},
        "reported_evidence": expected,
        "median_ns_per_operation": medians,
        "raw_logs": raw,
    }
    (args.output / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(medians, indent=2))


if __name__ == "__main__":
    main()
