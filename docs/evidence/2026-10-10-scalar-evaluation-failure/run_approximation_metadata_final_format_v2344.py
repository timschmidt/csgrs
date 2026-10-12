"""Repeat both formatting commands after the import-order correction."""
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
SOURCE = E.parents[2] / "target/scalar-evaluation-candidate/source"

for phase in ["typed-api-prototype-v2340-approximation-metadata-workspace-fmt",
              "typed-api-prototype-v2341-approximation-metadata-included-fmt"]:
    command = json.loads((E / phase / "command.json").read_text())["command"]
    print(json.dumps({"unchanged_command_from": phase, "command": command}), flush=True)
    subprocess.run(command, cwd=SOURCE, check=True)
    print(phase, "passed after correction", flush=True)
