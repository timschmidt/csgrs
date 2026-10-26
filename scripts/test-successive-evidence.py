#!/usr/bin/env python3
"""Exercise an actual attributed benchmark, including a killed child process.

Usage: test-successive-evidence.py --binary /tmp/build/successive_operations \
           --output /tmp/evidence-check [--baseline /tmp/old-rows.tsv]
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


DRIVER = Path(__file__).with_name("successive-operations.py").resolve()


def rows(path):
    return [
        line.split("\t") for line in path.read_text().splitlines()
        if line and not line.startswith(("#", "chain\t"))
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    binary = args.binary.resolve()
    build = json.loads(subprocess.check_output([str(binary), "--build-info"], text=True))
    checkout = output / "new-checkout"
    checkout.mkdir()
    subprocess.run(["git", "init", "-q", str(checkout)], check=True)
    subprocess.run([
        "git", "-C", str(checkout), "-c", "user.name=Evidence test", "-c",
        "user.email=evidence@example.invalid", "commit", "-q", "--allow-empty",
        "-m", "unrelated runtime checkout",
    ], check=True)
    revision = subprocess.check_output(["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True).strip()
    assert revision != build["source"]["revision"]
    moved = checkout / "moved-binary"
    shutil.copy2(binary, moved)
    replacement_tools = output / "runtime-tools"
    replacement_tools.mkdir()
    for name in ["git", "rustc"]:
        tool = replacement_tools / name
        tool.write_text("#!/bin/sh\nexit 99\n")
        tool.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = str(replacement_tools) + os.pathsep + environment["PATH"]

    def command(destination, *arguments):
        return [sys.executable, str(DRIVER), "run", "--binary", str(moved), "--output",
                str(destination), "--timeout", "120", "--", *arguments]

    for mode in ["validated", "untouched"]:
        destination = output / mode
        arguments = ["4-gon drills"] if mode == "validated" else ["--untouched", "4-gon drills"]
        subprocess.run(command(destination, *arguments), cwd=checkout, env=environment, check=True)
        manifest = json.loads((destination / "run.json").read_text())
        assert manifest["status"] == "completed"
        assert manifest["build"] == build
        assert manifest["binary_sha256"] == hashlib.sha256(moved.read_bytes()).hexdigest()
        assert manifest["rows"] == 16 and manifest["outcomes"] == {"certified": 16}
        header = next(line for line in (destination / "rows.tsv").read_text().splitlines() if line.startswith("# build\t"))
        assert json.loads(header.split("\t", 1)[1]) == build
        measured = rows(destination / "rows.tsv")
        assert all(row[3] == ("yes" if mode == "validated" else "-") for row in measured)
        if mode == "validated" and args.baseline:
            # Geometry, receipts, representation sizes, operation allocation
            # traffic and calls stay identical. Timing and live process memory
            # include different evidence-collection overhead and are separate.
            before = rows(args.baseline)
            assert len(before) == len(measured)
            for old, new in zip(before, measured):
                assert old[:11] == new[:11], (old, new)
                assert (old[14], old[17]) == (new[14], new[17]), (old, new)

    for mode in ["killed", "driver-terminated"]:
        destination = output / mode
        with (output / f"{mode}.log").open("w") as log:
            driver = subprocess.Popen(command(destination, "32-gon drills"), cwd=checkout, env=environment,
                                      stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 90
                while time.monotonic() < deadline:
                    journal = destination / "rows.tsv"
                    if journal.exists() and rows(journal):
                        manifest = json.loads((destination / "run.json").read_text())
                        assert driver.poll() is None
                        assert "# completed\n" not in journal.read_text()
                        if mode == "killed":
                            os.killpg(manifest["pid"], signal.SIGKILL)
                        else:
                            driver.terminate()
                        break
                    if driver.poll() is not None:
                        raise AssertionError("probe ended before the interruption test")
                    time.sleep(0.01)
                else:
                    raise AssertionError("no complete row before the interruption deadline")
                assert driver.wait(timeout=30) != 0
            finally:
                if driver.poll() is None:
                    state = destination / "run.json"
                    if state.exists():
                        pid = json.loads(state.read_text()).get("pid")
                        if pid:
                            try:
                                os.killpg(pid, signal.SIGKILL)
                            except ProcessLookupError:
                                pass
                    driver.terminate()
                    driver.wait(timeout=15)
        manifest = json.loads((destination / "run.json").read_text())
        assert manifest["status"] == "interrupted", manifest
        expected_signal = signal.SIGKILL if mode == "killed" else signal.SIGTERM
        assert manifest["process_returncode"] == -expected_signal
        assert manifest["rows"] >= 1 and not manifest["completion_marker"]
        assert all(row[2:4] == ["certified", "yes"] for row in rows(destination / "rows.tsv"))
    timed = output / "timed-out"
    timeout_command = command(timed, "32-gon drills")
    timeout_command[timeout_command.index("--timeout") + 1] = "0.1"
    result = subprocess.run(timeout_command, cwd=checkout, env=environment, check=False)
    assert result.returncode == 124
    state = json.loads((timed / "run.json").read_text())
    assert state["status"] == "timeout" and not state["completion_marker"]
    print("passed: moved binary, replaced runtime tools, both validation modes, preserved rows, child/driver termination, timeout")


if __name__ == "__main__":
    main()
