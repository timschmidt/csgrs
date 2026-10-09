"""Run the changed demo's validation with frozen Hypercurve Rust sources.

From the repository root: python3 reproduce_validation.py OUTPUT
These checks do not replace the repository-wide pre-push workflow matrix.
"""
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

root = Path.cwd()
output = Path(sys.argv[1]).resolve()
output.mkdir(parents=True, exist_ok=False)
demo = root / "crates/hypercurve/examples/hypercurve_ui"


def sources():
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted((root / "crates/hypercurve").rglob("*.rs"))
            if "target" not in p.parts}


frozen = sources()
commands = [
    ("workspace-fmt", root, ["cargo", "fmt", "--all", "--", "--check"]),
    ("demo-fmt", demo, ["cargo", "fmt", "--", "--check"]),
    ("demo-clippy", demo, ["cargo", "clippy", "--locked", "--all-targets", "--", "-D", "warnings"]),
    ("demo-tests", demo, ["cargo", "test", "--locked", "--no-fail-fast"]),
    ("demo-native", demo, ["cargo", "build", "--release", "--locked", "--message-format=json"]),
    ("demo-wasm", demo, ["cargo", "build", "--release", "--locked", "--target", "wasm32-unknown-unknown"]),
]
status = []
for name, cwd, command in commands:
    assert sources() == frozen
    started = time.monotonic()
    with (output / f"{name}.log").open("w") as log:
        if name == "demo-native":
            with (output / "library-build.jsonl").open("w") as artifacts:
                code = subprocess.run(command, cwd=cwd, stdout=artifacts, stderr=log).returncode
        else:
            code = subprocess.run(command, cwd=cwd, stdout=log, stderr=subprocess.STDOUT).returncode
    status.append(dict(name=name, cwd=str(cwd), command=command, exit_code=code,
                       elapsed_seconds=time.monotonic() - started))
    (output / "status.json").write_text(json.dumps(status, indent=2) + "\n")
    (output / "sources.json").write_text(json.dumps(
        dict(source_hashes=frozen, unchanged=sources() == frozen), indent=2) + "\n")
    print(name, code, flush=True)
    assert sources() == frozen
    if code:
        raise SystemExit(code)
