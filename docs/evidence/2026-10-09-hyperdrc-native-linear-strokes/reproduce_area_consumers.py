"""Run DRC consumers after the repaired isolated validation finishes."""

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parent
VALIDATION = ROOT / "area-compaction-isolated-trial2"
FINAL = ROOT / "area-compaction-final-source"
SOURCE = Path("/tmp/drc-area-compaction-final/source")
TARGET = Path("/tmp/drc-area-compaction-final/target")
OUT = ROOT / "area-compaction-consumers-trial2"
previous = json.loads((VALIDATION / "status.json").read_text())
assert previous[-1]["name"] == "sign-probe"
assert all(command["exit_code"] == 0 for command in previous)
final_status = json.loads((FINAL / "status.json").read_text())
assert final_status[-1]["name"] == "real-clippy"
assert all(command["exit_code"] == 0 for command in final_status)
source = json.loads((FINAL / "sources.json").read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
fixtures = json.loads((ROOT / "area-compaction-fixture-recovery/manifest.json").read_text())


def verify():
    assert all(digest(Path(name)) == value["sha256"] for name, value in fixtures["inputs"].items())
    assert all(digest(SOURCE / name) == value for name, value in source["source_hashes"].items())


verify()
OUT.mkdir()
shutil.copyfile(__file__, OUT / "reproduce.py")
ENV = os.environ | source["environment"]
status = []


def run(name, command):
    verify()
    start = time.monotonic()
    print("start", name, flush=True)
    with (OUT / (name + ".log")).open("w") as log:
        result = subprocess.run(command, cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=False)
    status.append({"name": name, "command": command, "exit_code": result.returncode, "wall_seconds": time.monotonic() - start})
    (OUT / "status.json").write_text(json.dumps(status, indent=2) + "\n")
    verify()
    print("finish", name, result.returncode, flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)


run("build-drc-tests", ["cargo", "test", "--locked", "--release", "-p", "hyperdrc", "--lib", "--no-run", "--message-format=json"])
executables = []
for line in (OUT / "build-drc-tests.log").read_text().splitlines():
    if not line.startswith("{"):
        continue
    event = json.loads(line)
    if event.get("reason") == "compiler-artifact" and event.get("executable") and event["target"]["name"] == "hyperdrc":
        executables.append(event["executable"])
assert len(executables) == 1
run("drc-tests", [executables[0], "--test-threads=2"])
run("drc-clippy", ["cargo", "clippy", "--locked", "--release", "-p", "hyperdrc", "--all-targets", "--", "-D", "warnings"])
manifest = {
    "scope": "Full default DRC library tests and strict release/all-target Clippy against the repaired isolated source; this is not the complete workspace/pre-push matrix.",
    "source_record": str(FINAL / "sources.json"),
    "source_unchanged": True,
    "fixture_recovery_record": str(ROOT / "area-compaction-fixture-recovery/manifest.json"),
    "executable": {"path": executables[0], "sha256": digest(Path(executables[0]))},
    "status": status,
}
(OUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
