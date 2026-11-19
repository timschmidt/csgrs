#!/usr/bin/env python3
"""Check the Hyperpath test inventory and replay the shipped schedule on WASI Preview 1."""

import argparse
import json
import os
from pathlib import Path
import subprocess

from wasi_libtest import ROOT, RUNNER, TARGET, build_tests, completed, execute, test_names


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", default="stable")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    cargo = ["cargo", f"+{args.toolchain}"]
    options = ["--locked", "--release", "-p", "hyperpath", "--all-features"]
    selection = ["--lib", "--tests"]
    tests = [*cargo, "test", *options]
    hosts = build_tests(tests, selection, os.environ)
    guests = build_tests(tests, [*selection, "--target", TARGET], os.environ)
    if set(hosts) != set(guests) or "schedule_equivalence" not in hosts:
        raise SystemExit("Missing schedule oracle or unequal native and WASI suite inventories")

    # Use the same property seed and suite-defined case counts on both targets.
    seed = "20261119"
    host_env = {key: value for key, value in os.environ.items() if not key.startswith("PROPTEST_")}
    host_env["PROPTEST_RNG_SEED"] = seed
    report = {}
    for suite, binary in sorted(guests.items()):
        guest = ["node", str(RUNNER), "--env", "PROPTEST_RNG_SEED", seed, binary]
        names = test_names([hosts[suite]])
        if not names or names != test_names(guest):
            raise SystemExit(f"Empty or unequal native and WASI test inventories in {suite}")
        for target, command, env in [("native", [hosts[suite]], host_env), ("wasi", guest, None)]:
            log = output / f"{suite}-{target}.log"
            run = execute([*command, "--test-threads=1"], log, env)
            if run.returncode or not completed(run.stdout, names):
                raise SystemExit(f"Incomplete {target} test run. See {log}")
        report[suite] = sorted(names)

    example = [*cargo, "run", "--quiet", *options, "--example", "certified_schedule"]
    native = subprocess.check_output(example, cwd=ROOT)
    runner = f"target.{TARGET}.runner={json.dumps(['node', str(RUNNER)])}"
    replay = subprocess.check_output([*example, "--target", TARGET, "--config", runner], cwd=ROOT)
    if not native or replay != native:
        raise SystemExit("Native and WASI schedule example transcripts differ")
    (output / "certified_schedule.log").write_bytes(replay)
    (output / "report.json").write_text(json.dumps({"seed": seed, "suites": report}, indent=2) + "\n")
    print(f"{sum(map(len, report.values()))} Hyperpath tests and the shipped schedule passed on WASI.")


if __name__ == "__main__":
    main()
