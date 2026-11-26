#!/usr/bin/env python3
"""Check CSV structure/repeatability and report median per-iteration timings.

The checks do not establish geometry correctness, matched timing boundaries,
certainty, complete workload coverage or provenance. Legacy rows become warm.
Engine parity covers only workload keys present in the supplied files, and
checksums are compared within groups without normalization by iteration count.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

# Required ordered header; numerical fields start at sample, not at elapsed_ns.
FIELDS = [
    "engine",
    "temperature",
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
# Accepted historical header, interpreted as warm regardless of actual cache state.
LEGACY_FIELDS = [field for field in FIELDS if field != "temperature"]

# Exact engine-label set required for every observed portable workload key.
CROSS_KERNEL_ENGINES = {
    "csgrs",
    "cgal-epeck",
    "opencascade-double-tight",
}
# Only these suites participate in engine parity and the cold/warm comparison table.
CROSS_KERNEL_SUITES = {"kernel", "precision", "corpus"}


def load(paths: list[Path]) -> list[dict[str, str]]:
    """Load ordered current/legacy CSV headers, temperature and integer fields.

    Iterations must be positive; other numeric fields may be negative or zero.
    Requires at least one row overall, but accepts empty individual files with
    valid headers. Does not reject duplicate samples or extra row columns.
    Short rows can raise TypeError when int(None) is attempted, outside the CLI's
    handled error set. File/CSV errors otherwise follow the underlying readers.
    """
    rows: list[dict[str, str]] = []
    for path in paths:
        with path.open(newline="", encoding="utf-8") as source:
            reader = csv.DictReader(source)
            legacy = reader.fieldnames == LEGACY_FIELDS
            if reader.fieldnames != FIELDS and not legacy:
                raise ValueError(f"{path}: expected fields {FIELDS}, got {reader.fieldnames}")
            for line, row in enumerate(reader, 2):
                if legacy:
                    row["temperature"] = "warm"
                if row["temperature"] not in {"cold", "warm"}:
                    raise ValueError(
                        f"{path}:{line}: temperature must be cold or warm"
                    )
                for field in FIELDS[5:]:
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
    """Group by temperature/workload/engine and compute per-iteration statistics.

    Divides elapsed time and output size by iterations, not work_units. Requires
    identical raw accumulated checksums per group, so different iteration plans
    may fail despite identical per-iteration output. Duplicate sample indices
    are retained as samples. Returns sorted groups with median/min/max time;
    these statistics do not compare geometry or certify workload completion.
    """
    groups: dict[tuple[str, str, str, str, str], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        key = (
            row["temperature"],
            row["suite"],
            row["benchmark"],
            row["case"],
            row["engine"],
        )
        groups[key].append(row)

    summaries: list[dict[str, object]] = []
    for (temperature, suite, benchmark, case, engine), samples in sorted(groups.items()):
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
                "temperature": temperature,
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


def validate_engine_parity(summaries: list[dict[str, object]]) -> tuple[int, int]:
    """Require the exact engine set for each observed portable-suite key.

    Returns distinct workload and temperature counts, not a proof that their
    Cartesian product is present. Missing workloads absent from every engine
    are invisible; no portable rows returns (0, 0). Set membership does not
    detect duplicate engine rows or compare samples, output sizes or receipts.
    """
    engines_by_workload: dict[
        tuple[object, object, object, object], set[object]
    ] = defaultdict(set)
    for row in summaries:
        if row["suite"] in CROSS_KERNEL_SUITES:
            key = (
                row["temperature"],
                row["suite"],
                row["benchmark"],
                row["case"],
            )
            engines_by_workload[key].add(row["engine"])

    failures = []
    for key, engines in sorted(engines_by_workload.items()):
        missing = CROSS_KERNEL_ENGINES - engines
        unexpected = engines - CROSS_KERNEL_ENGINES
        if missing or unexpected:
            failures.append(
                f"{'/'.join(map(str, key))}: missing={sorted(missing)}, "
                f"unexpected={sorted(unexpected)}"
            )
    if failures:
        raise ValueError("cross-kernel engine parity failed: " + "; ".join(failures))
    temperatures = {key[0] for key in engines_by_workload}
    workloads = {(key[1], key[2], key[3]) for key in engines_by_workload}
    return len(workloads), len(temperatures)


def markdown(
    summaries: list[dict[str, object]],
    parity: tuple[int, int] | None = None,
) -> str:
    """Render median timing/output tables and ratios to matching csgrs rows.

    The cold/warm table needs all six engine/temperature values for a workload.
    Other rows render even without a csgrs baseline. Zero baseline medians cause
    division errors; caller-supplied labels are not Markdown-escaped. Optional
    parity counts are displayed as a product without verifying a complete grid.
    """
    baselines = {
        (
            row["temperature"],
            row["suite"],
            row["benchmark"],
            row["case"],
        ): float(row["median_ns"])
        for row in summaries
        if row["engine"] == "csgrs"
    }
    lines = []
    if parity is not None:
        parity_workloads, parity_temperatures = parity
        workload_label = "workload" if parity_workloads == 1 else "workloads"
        temperature_label = (
            "temperature" if parity_temperatures == 1 else "temperatures"
        )
        lines.extend(
            [
                f"Cross-kernel parity: **{parity_workloads} {workload_label} × "
                f"{parity_temperatures} {temperature_label} × "
                f"{len(CROSS_KERNEL_ENGINES)} engines**.",
                "",
            ]
        )
    by_temperature = {
        (
            row["temperature"],
            row["suite"],
            row["benchmark"],
            row["case"],
            row["engine"],
        ): float(row["median_ns"])
        for row in summaries
        if row["suite"] in CROSS_KERNEL_SUITES
    }
    workloads = sorted(
        {
            (row["suite"], row["benchmark"], row["case"])
            for row in summaries
            if row["suite"] in CROSS_KERNEL_SUITES
        }
    )
    complete_temperature_rows = []
    for suite, benchmark, case in workloads:
        key_values = {
            (temperature, engine): by_temperature.get(
                (temperature, suite, benchmark, case, engine)
            )
            for temperature in ("cold", "warm")
            for engine in CROSS_KERNEL_ENGINES
        }
        if all(value is not None for value in key_values.values()):
            complete_temperature_rows.append(
                (suite, benchmark, case, key_values)
            )
    if complete_temperature_rows:
        lines.extend(
            [
                "Ratios above 1× mean the comparison engine is slower than csgrs. "
                "CSGRS speedup is cold time divided by warm time.",
                "",
                "| Suite | Benchmark | Case | CSGRS cold ns | CSGRS warm ns | CSGRS speedup | CGAL/csgrs cold | CGAL/csgrs warm | OCCT/csgrs cold | OCCT/csgrs warm |",
                "|---|---|---|---:|---:|---:|---:|---:|---:|---:|",
            ]
        )
        for suite, benchmark, case, values in complete_temperature_rows:
            csgrs_cold = values[("cold", "csgrs")]
            csgrs_warm = values[("warm", "csgrs")]
            cgal_cold = values[("cold", "cgal-epeck")]
            cgal_warm = values[("warm", "cgal-epeck")]
            occt_cold = values[("cold", "opencascade-double-tight")]
            occt_warm = values[("warm", "opencascade-double-tight")]
            lines.append(
                f"| {suite} | {benchmark} | {case} | {csgrs_cold:.0f} | "
                f"{csgrs_warm:.0f} | {csgrs_cold / csgrs_warm:.3f}× | "
                f"{cgal_cold / csgrs_cold:.3f}× | "
                f"{cgal_warm / csgrs_warm:.3f}× | "
                f"{occt_cold / csgrs_cold:.3f}× | "
                f"{occt_warm / csgrs_warm:.3f}× |"
            )
        lines.append("")
    lines.extend(
        [
            "| Temperature | Suite | Benchmark | Case | Engine | Samples | Median ns/op | vs csgrs | Output/op |",
            "|---|---|---|---|---|---:|---:|---:|---:|",
        ]
    )
    for row in summaries:
        key = (
            row["temperature"],
            row["suite"],
            row["benchmark"],
            row["case"],
        )
        baseline = baselines.get(key)
        ratio = "—" if baseline is None else f"{float(row['median_ns']) / baseline:.3f}×"
        lines.append(
            f"| {row['temperature']} | {row['suite']} | {row['benchmark']} | {row['case']} | "
            f"{row['engine']} | {row['samples']} | {float(row['median_ns']):.0f} | "
            f"{ratio} | {float(row['output_size']):.0f} |"
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    """Parse paths/options, summarize and print or overwrite the selected report.

    Read/validation OSError and ValueError return status 1 with a diagnostic.
    Output writes occur outside that handler; TypeError, division errors and
    other failures can escape with a traceback. No parent directory is created,
    atomic report installation performed or raw input modified.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--require-engine-parity", action="store_true")
    args = parser.parse_args()
    try:
        summaries = summarize(load(args.csv))
        parity = (
            validate_engine_parity(summaries) if args.require_engine_parity else None
        )
        result = markdown(summaries, parity)
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
