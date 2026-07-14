#!/usr/bin/env python3
"""Validate and summarize benchmark CSV files without third-party packages."""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

FIELDS = [
    "engine",
    "suite",
    "benchmark",
    "case",
    "sample",
    "iterations",
    "elapsed_ns",
    "work_units",
    "output_size",
    "checksum",
]


def load(paths: list[Path]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for path in paths:
        with path.open(newline="", encoding="utf-8") as source:
            reader = csv.DictReader(source)
            if reader.fieldnames != FIELDS:
                raise ValueError(f"{path}: expected fields {FIELDS}, got {reader.fieldnames}")
            for line, row in enumerate(reader, 2):
                for field in FIELDS[4:]:
                    try:
                        int(row[field])
                    except ValueError as error:
                        raise ValueError(f"{path}:{line}: {field} is not an integer") from error
                if int(row["iterations"]) < 1:
                    raise ValueError(f"{path}:{line}: iterations must be positive")
                rows.append(row)
    if not rows:
        raise ValueError("no benchmark samples found")
    return rows


def summarize(rows: list[dict[str, str]]) -> list[dict[str, object]]:
    groups: dict[tuple[str, str, str, str], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        key = (row["suite"], row["benchmark"], row["case"], row["engine"])
        groups[key].append(row)

    summaries: list[dict[str, object]] = []
    for (suite, benchmark, case, engine), samples in sorted(groups.items()):
        times = [int(row["elapsed_ns"]) / int(row["iterations"]) for row in samples]
        outputs = [int(row["output_size"]) / int(row["iterations"]) for row in samples]
        checksums = {int(row["checksum"]) for row in samples}
        if len(checksums) != 1:
            raise ValueError(
                f"nondeterministic checksum for {engine}/{suite}/{benchmark}/{case}: "
                f"{sorted(checksums)}"
            )
        summaries.append(
            {
                "suite": suite,
                "benchmark": benchmark,
                "case": case,
                "engine": engine,
                "samples": len(samples),
                "median_ns": statistics.median(times),
                "min_ns": min(times),
                "max_ns": max(times),
                "output_size": statistics.median(outputs),
            }
        )
    return summaries


def markdown(summaries: list[dict[str, object]]) -> str:
    baselines = {
        (row["suite"], row["benchmark"], row["case"]): float(row["median_ns"])
        for row in summaries
        if row["engine"] == "csgrs"
    }
    lines = [
        "| Suite | Benchmark | Case | Engine | Samples | Median ns/op | vs csgrs | Output/op |",
        "|---|---|---|---|---:|---:|---:|---:|",
    ]
    for row in summaries:
        key = (row["suite"], row["benchmark"], row["case"])
        baseline = baselines.get(key)
        ratio = "—" if baseline is None else f"{float(row['median_ns']) / baseline:.3f}×"
        lines.append(
            f"| {row['suite']} | {row['benchmark']} | {row['case']} | "
            f"{row['engine']} | {row['samples']} | {float(row['median_ns']):.0f} | "
            f"{ratio} | {float(row['output_size']):.0f} |"
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        result = markdown(summarize(load(args.csv)))
    except (OSError, ValueError) as error:
        print(f"benchmark summary error: {error}", file=sys.stderr)
        return 1
    if args.output:
        args.output.write_text(result, encoding="utf-8")
    else:
        print(result, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
