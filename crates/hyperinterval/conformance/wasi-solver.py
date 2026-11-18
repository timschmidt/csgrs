#!/usr/bin/env python3
"""Run the solver's interval acceptance suites and Horner migration comparisons on WASI."""

import argparse
import json
import os
from pathlib import Path

from wasi_libtest import RUNNER, TARGET, build_tests, compare_dispatch, completed, execute, test_names

# These integration suites run in full with their normal feature set.
SUITES = (
    "expression_interval", "interval_box_radius", "interval_quadratic_extrema",
    "jacobian_matrix", "krawczyk_affine_oracle", "krawczyk_affine_residuals",
    "krawczyk_certificates", "krawczyk_multivariate_certainty", "program_jacobian_trace",
    "real_interval_enclosures", "real_interval_migration_contract", "trivariate_arithmetic",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", default="stable")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    cargo = ["cargo", f"+{args.toolchain}", "test", "--locked", "--release", "-p", "hypersolve", "--all-features"]
    selection = ["--lib", *(arg for suite in SUITES for arg in ("--test", suite))]
    guests = build_tests([*cargo, "--target", TARGET], selection, os.environ)
    hosts = build_tests(cargo, selection, os.environ)
    report = {"guest": {}, "dispatch_windows": {}}

    for suite in SUITES:
        guest = ["node", str(RUNNER), guests[suite]]
        expected = test_names([hosts[suite]])
        if not expected or test_names(guest) != expected:
            raise SystemExit(f"Empty or unequal native and WASI test inventories for {suite}")
        log = output / f"{suite}.log"
        run = execute([*guest, "--test-threads=1"], log)
        if run.returncode or not completed(run.stdout, expected):
            raise SystemExit(f"Incomplete or failed WASI {suite} test suite. See {log}")
        report["guest"][suite] = sorted(expected)

    migrations = [
        ("algebraic::interval_migration::rational_horner", 1404),
        ("root_isolation::interval_migration::rational_horner_exclusion", 2808),
    ]
    for suite, expected in migrations:
        case = f"{suite}_exact_differential"
        log = output / f"{case.replace('::', '.')}.log"
        run = execute(["node", str(RUNNER), guests["hypersolve"], "--exact", case], log)
        if run.returncode or not completed(run.stdout, {case}):
            raise SystemExit(f"Missing or failed exact Horner test. See {log}")
        report["guest"].setdefault("hypersolve", []).append(case)
        case = f"{suite}_dispatch_differential"
        report["dispatch_windows"][case] = compare_dispatch(
            guests["hypersolve"], hosts["hypersolve"], case, "HYPERSOLVE_HORNER_BACKEND", "HORNER", expected, output)
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"{sum(map(len, report['guest'].values()))} solver tests passed in WASI. "
          f"{sum(report['dispatch_windows'].values())} Horner dispatch windows matched native and WASI.")


if __name__ == "__main__":
    main()
