"""Build and run Cargo libtest executables in isolated WASI Preview 1 guests."""

import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]
RUNNER = ROOT / "crates/hyperlattice-interval/conformance/run-wasi-test.mjs"
TARGET = "wasm32-wasip1"


def build_tests(cargo, selection, env):
    output = subprocess.check_output([*cargo, *selection, "--no-run", "--message-format=json-render-diagnostics"],
                                     env=env, cwd=ROOT, text=True)
    executables = {}
    for row in map(json.loads, output.splitlines()):
        if row.get("executable") and row["profile"]["test"]:
            name = row["target"]["name"]
            if name in executables:
                raise SystemExit(f"Duplicate test executable for {name}")
            executables[name] = row["executable"]
    return executables


def test_names(command):
    listing = subprocess.check_output([*command, "--list"], cwd=ROOT, text=True, stderr=subprocess.PIPE)
    return set(re.findall(r"^(\S+): test$", listing, re.MULTILINE))


def completed(transcript, expected):
    return (f"test result: ok. {len(expected)} passed; 0 failed; 0 ignored;" in transcript
            and set(re.findall(r"^test (\S+) \.\.\. ", transcript, re.MULTILINE)) == expected)


def execute(command, log, env=None):
    # Node's WASI stdout can return EAGAIN on a full pipe. Write large trace
    # transcripts directly to a regular file so guest println! completes.
    with log.open("w") as stream:
        result = subprocess.run(command, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT)
    return subprocess.CompletedProcess(command, result.returncode, log.read_text())


def run_replay_child(guest, args, case, options, receipt):
    """Run the guest with a regular stdout file, then forward its evidence to the native parent."""
    with tempfile.TemporaryDirectory() as temp:
        run = execute(["node", str(RUNNER), *options, guest, *args], Path(temp) / "child.log")
    if run.returncode or not completed(run.stdout, {case}):
        raise SystemExit(run.stdout)
    # On single-thread libtest the first evidence row can follow the test label.
    lines = [line.removeprefix(f"test {case} ... ") for line in run.stdout.splitlines()]
    for line in lines:
        print(line)
    with receipt.open("a") as stream:
        print(case, file=stream)
    return lines


def compare_dispatch(guest, native, case, variable, marker, expected, output):
    """Compare full ordered results and counters across implementations and targets."""
    prefix = f"{marker} TraceSnapshot "
    baseline = None
    for backend in ("legacy", "shared"):
        runs = [
            ("native", [native], os.environ | {variable: backend}),
            ("wasi", ["node", str(RUNNER), "--env", variable, backend, guest], None),
        ]
        for target, command, env in runs:
            log = output / f"{case.replace('::', '.')}-{target}-{backend}.log"
            run = execute([*command, "--exact", case, "--test-threads=1", "--nocapture"], log, env)
            if run.returncode or not completed(run.stdout, {case}):
                raise SystemExit(f"{case} did not complete. See {log}")
            # Single-thread libtest prints the test name and first row on one line.
            lines = (line.removeprefix(f"test {case} ... ") for line in run.stdout.splitlines())
            rows = [line for line in lines if line.startswith(prefix)]
            if len(rows) != expected:
                raise SystemExit(f"Expected {expected} {marker} rows, found {len(rows)} in {log}")
            if baseline is None:
                baseline = rows
                continue
            for index, (old, new) in enumerate(zip(baseline, rows)):
                if old != new:
                    raise SystemExit(f"{case} row {index + 1} differs in {log}\nnative legacy: {old}\nobserved: {new}")
    return expected
