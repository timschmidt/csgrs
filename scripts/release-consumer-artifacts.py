"""Select the required release smoke executables from Cargo's build messages."""

import json
import os
from pathlib import Path
import sys


def executables(messages, manifest, names):
    paths = {}
    for message in messages:
        if (message["reason"] != "compiler-artifact"
                or message["manifest_path"] != str(manifest)
                or "bin" not in message["target"]["kind"]):
            continue
        name = message["target"]["name"]
        if name in names and message["executable"]:
            if name in paths:
                raise ValueError(f"multiple executable artifacts for {name}")
            paths[name] = message["executable"]
    missing = [name for name in names if name not in paths]
    if missing:
        raise ValueError(f"missing executable artifacts: {', '.join(missing)}")
    return [paths[name] for name in names]


if __name__ == "__main__":
    try:
        paths = executables(map(json.loads, sys.stdin), Path(sys.argv[1]).resolve(), sys.argv[2:])
    except ValueError as error:
        sys.exit(str(error))
    for path in paths:
        sys.stdout.buffer.write(os.fsencode(path) + b"\0")
