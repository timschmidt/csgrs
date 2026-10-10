#!/usr/bin/env python3
"""Run an already-built, hash-matched scalar unit test with hard resource caps."""
import hashlib
import json
from pathlib import Path
import resource
import subprocess
import sys
import time


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (192 * 1024 * 1024, 192 * 1024 * 1024))
    resource.setrlimit(resource.RLIMIT_CPU, (2, 3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def main():
    executable = Path(sys.argv[1]).resolve()
    expected_hash = sys.argv[2]
    assert digest(executable) == expected_hash
    command = [str(executable), *sys.argv[3:]]
    started = time.monotonic()
    timed_out = False
    try:
        result = subprocess.run(command, preexec_fn=limits, timeout=4)
        code = result.returncode
    except subprocess.TimeoutExpired:
        timed_out = True
        code = 124
    assert digest(executable) == expected_hash
    print(json.dumps({
        "command": command, "binary_sha256": expected_hash,
        "helper_sha256": digest(Path(__file__)),
        "address_space_bytes": 192 * 1024 * 1024,
        "cpu_seconds_soft_hard": [2, 3], "wall_seconds_limit": 4,
        "exit_code": code, "wall_timeout": timed_out,
        "seconds": time.monotonic() - started,
    }), flush=True)
    sys.exit(code if code >= 0 else 128 - code)


if __name__ == "__main__":
    main()
