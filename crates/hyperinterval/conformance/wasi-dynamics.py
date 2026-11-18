#!/usr/bin/env python3
"""Run Hyperdynamics numerical suites and fresh-process source replays on WASI Preview 1."""

import argparse
import json
import os
from pathlib import Path
import sys
import tempfile

from wasi_libtest import RUNNER, TARGET, build_tests, completed, execute, test_names

THREAD_DROP = ("flow_noise_witness", "closure::unique_history_drops_iteratively_with_a_retained_intermediate_state")
REPLAY_DIRECTORY = "CSGRS_FLOW_NOISE_REPLAY_DIR"


def replay_child(args):
    """Execute the original child with the exact source supplied by its native parent."""
    guest = os.environ["DYNAMICS_WASI_GUEST"]
    keys = [key for key in os.environ if key == REPLAY_DIRECTORY
            or (key.startswith("HYPERDYNAMICS_") and key.endswith("_REPLAY"))]
    if len(keys) != 1 or "--exact" not in args:
        raise SystemExit("Expected one replay input and an exact child test")
    key = keys[0]
    case = args[args.index("--exact") + 1]
    options = ["--env", key, os.environ[key]]
    if key == REPLAY_DIRECTORY:
        options = ["--env", key, "/replay", "--replay-dir", os.environ[key]]
    # The native tests collect subprocess stdout via a pipe. WASI needs a regular
    # file while writing large evidence rows, then the parent can read them.
    with tempfile.TemporaryDirectory() as temp:
        run = execute(["node", str(RUNNER), *options, guest, *args], Path(temp) / "child.log")
    if run.returncode or not completed(run.stdout, {case}):
        raise SystemExit(run.stdout)
    # Single-thread libtest can put the first evidence row after the test label.
    prefix = f"test {case} ... "
    for line in run.stdout.splitlines():
        print(line.removeprefix(prefix) if line.startswith(prefix) else line)
    with Path(os.environ["DYNAMICS_WASI_RECEIPT"]).open("a") as receipt:
        print(case, file=receipt)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", default="stable")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    cargo = ["cargo", f"+{args.toolchain}", "test", "--locked", "--release", "-p", "hyperdynamics", "--all-features"]
    select = ["--lib", "--tests"]
    guest_env = os.environ.copy()
    guest_env.pop("HYPERDYNAMICS_REPLAY_DRIVER", None)
    guests = build_tests([*cargo, "--target", TARGET], select, guest_env)
    host_env = guest_env | {"HYPERDYNAMICS_REPLAY_DRIVER": str(Path(__file__).resolve())}
    hosts = build_tests(cargo, select, host_env)
    if set(guests) != set(hosts):
        raise SystemExit("Native and guest executable inventories differ")
    report = {"guest": {}, "fresh_replays": {}, "host_threads": []}
    for suite, guest in sorted(guests.items()):
        command = ["node", str(RUNNER), guest]
        names = test_names(command)
        if names != test_names([hosts[suite]]) or (not names and suite != "hyperdynamics"):
            raise SystemExit(f"Empty or unequal native/WASI test inventories for {suite}")
        replays = {name for name in names if "fresh_process" in name}
        thread = {THREAD_DROP[1]} if suite == THREAD_DROP[0] else set()
        direct = names - replays - thread
        skip = [arg for name in sorted(replays | thread) for arg in ("--skip", name)]
        log = output / f"{suite}.log"
        run = execute([*command, "--exact", "--test-threads=1", *skip], log)
        if run.returncode or not completed(run.stdout, direct):
            raise SystemExit(f"Incomplete WASI {suite} suite. See {log}")
        report["guest"][suite] = sorted(direct)
        for case in sorted(replays):
            stem = suite + "." + case.replace("::", ".")
            receipt = output / (stem + ".children")
            env = os.environ | {"DYNAMICS_WASI_GUEST": guest, "DYNAMICS_WASI_RECEIPT": str(receipt)}
            log = output / (stem + ".log")
            with tempfile.TemporaryDirectory(dir=output) as temp:
                run = execute([hosts[suite], "--exact", case, "--nocapture"], log, env | {"TMPDIR": temp})
            if run.returncode or not completed(run.stdout, {case}):
                raise SystemExit(f"Fresh WASI replay differs from its native parent. See {log}")
            calls = receipt.read_text().splitlines() if receipt.is_file() else []
            if set(calls) != {case}:
                raise SystemExit(f"Missing or incorrect guest replay receipt for {case}")
            report["fresh_replays"].setdefault(suite, {})[case] = len(calls)
        for case in thread:
            log = output / "wasi-thread-drop.log"
            run = execute([*command, "--exact", case, "--nocapture"], log)
            if not run.returncode or "kind: Unsupported" not in run.stdout:
                raise SystemExit(f"Unexpected WASI thread creation outcome. See {log}")
            log = output / "native-thread-drop.log"
            run = execute([hosts[suite], "--exact", case], log)
            if run.returncode or not completed(run.stdout, {case}):
                raise SystemExit(f"Native small-stack destruction failed. See {log}")
            report["host_threads"].append(case)
    if not report["fresh_replays"] or report["host_threads"] != [THREAD_DROP[1]]:
        raise SystemExit("Missing fresh replay or thread-destruction coverage")
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"{sum(map(len, report['guest'].values()))} direct WASI tests, "
          f"{sum(map(len, report['fresh_replays'].values()))} fresh WASI replay cases, "
          f"{len(report['host_threads'])} native thread test.")


if __name__ == "__main__":
    if "--exact" in sys.argv[1:]:
        replay_child(sys.argv[1:])
    else:
        main()
