#!/usr/bin/env python3
"""Compare warm sphere-constructor retention using two mesh_pipeline binaries.

This focused probe validates triangle counts only. Exact geometry and certainty
are covered separately by tests/solid_cache.rs; this is not a competitive suite.
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--samples", type=int, default=15)
    parser.add_argument("--warmup", type=int, default=2)
    parser.add_argument("--iterations", type=int, default=1_000_000)
    args = parser.parse_args()
    if args.samples <= 0 or args.warmup < 0 or args.iterations <= 0:
        parser.error("samples and iterations must be positive; warmup must be nonnegative")

    programs = {"before": args.baseline.resolve(), "after": args.candidate.resolve()}
    settings = {
        "CSGRS_CONSTRUCTION_BENCH_ITERS": str(args.iterations),
        "CSGRS_BUFFER_BENCH_ITERS": "1",
        "CSGRS_BOOLEAN_BENCH_ITERS": "0",
        "CSGRS_HULL_BENCH_ITERS": "0",
        "CSGRS_RAW_HULL_BENCH_ITERS": "0",
        "CSGRS_MINKOWSKI_BENCH_ITERS": "0",
    }
    environment = dict(os.environ, **settings)
    # Use the same permitted CPU for both children so migration between cores
    # cannot masquerade as a difference in this nanosecond-scale workload.
    cpu = min(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None
    def pin_cpu():
        os.sched_setaffinity(0, {cpu})

    rows = []
    for sample in range(args.warmup + args.samples):
        order = ["before", "after"] if sample % 2 == 0 else ["after", "before"]
        for name in order:
            output = subprocess.check_output(
                [str(programs[name])], env=environment, text=True,
                preexec_fn=pin_cpu if cpu is not None else None,
            )
            match = re.search(
                r"mesh_construction iterations=(\d+) polygons=(\d+) elapsed_us=(\d+)",
                output,
            )
            if match is None:
                raise RuntimeError(f"{name}: missing mesh_construction row")
            iterations, triangles, elapsed = map(int, match.groups())
            if iterations != args.iterations or triangles != 528 * iterations:
                raise RuntimeError(f"{name}: constructor workload or triangle count changed")
            if sample >= args.warmup:
                rows.append([name, sample - args.warmup, iterations, triangles, elapsed])

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["implementation", "sample", "iterations", "triangles", "elapsed_us"])
        writer.writerows(rows)
    metadata = {
        "scope": "Warm sphere(10, 24, 12) constructor probe; each process includes one cold miss.",
        "correctness": "Triangle counts only; exact row and certainty checks are separate tests.",
        "environment": settings,
        "samples": args.samples,
        "warmup_process_pairs": args.warmup,
        "cpu_affinity": cpu,
        "host": {"system": platform.system(), "release": platform.release(), "machine": platform.machine()},
        "rustc": subprocess.check_output(["rustc", "--version"], text=True).strip(),
        "executables": {},
        "median_ns_per_operation": {},
    }
    for name, program in programs.items():
        metadata["executables"][name] = {
            "sha256": hashlib.sha256(program.read_bytes()).hexdigest(),
            "bytes": program.stat().st_size,
        }
        metadata["median_ns_per_operation"][name] = statistics.median(
            row[4] / row[2] * 1000 for row in rows if row[0] == name
        )
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata["median_ns_per_operation"], indent=2))


if __name__ == "__main__":
    main()
