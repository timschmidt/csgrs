"""Finalize only the API inventory over the fully validated runtime source."""

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import time

ROOT = Path(__file__).resolve().parent
BASE = Path("/tmp/drc-area-compaction-sign-fix")
WORK = Path("/tmp/drc-area-compaction-final")
OUT = ROOT / "area-compaction-final-source"
BASE_EVIDENCE = ROOT / "area-compaction-isolated-trial2"
SOURCE, TARGET = WORK / "source", WORK / "target"
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
prior = json.loads((BASE_EVIDENCE / "status.json").read_text())
assert prior[-1]["name"] == "sign-probe" and all(command["exit_code"] == 0 for command in prior)
base = json.loads((BASE_EVIDENCE / "sources.json").read_text())
assert all(digest(BASE / "source" / name) == value for name, value in base["source_hashes"].items())
if OUT.exists():
    assert (OUT / "copy-interruption.json").exists()
    assert not (OUT / "sources.json").exists() and not (OUT / "status.json").exists()
else:
    OUT.mkdir()
WORK.mkdir(exist_ok=True)
for folder in ["source", "target"]:
    (WORK / folder).mkdir(exist_ok=True)
    with (OUT / ("copy-" + folder + ".log")).open("w") as log:
        subprocess.run(["cp", "-a", "--reflink=auto", str(BASE / folder) + "/.", str(WORK / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
assert all(digest(SOURCE / name) == value for name, value in base["source_hashes"].items())
name = "crates/hyperreal/tests/gmp_api_coverage.rs"
path = SOURCE / name
text = path.read_text()
before = '    "compact_quadratic_tower",\n'
after = '    "compact_pi_laurent",\n' + before
assert text.count(before) == 1
path.write_text(text.replace(before, after))
initial = {name: digest(SOURCE / name) for name in base["source_hashes"]}
assert {key for key, value in initial.items() if value != base["source_hashes"][key]} == {name}
with tarfile.open(OUT / "source-delta.tar.gz", "w:gz") as archive:
    archive.add(path, arcname=name)
shutil.copyfile(__file__, OUT / "reproduce.py")
ENV = os.environ | base["environment"] | {"CARGO_TARGET_DIR": str(TARGET)}
record = {
    "scope": "One integration-test inventory entry; all other source, manifest, fixture and non-test executable code hashes exactly match the fully validated sign-retention candidate.",
    "base_source_record": str(BASE_EVIDENCE / "sources.json"),
    "base_validation_record": str(BASE_EVIDENCE / "status.json"),
    "only_modified_sources_from_runtime_validation": [name],
    "non_test_executable_source_unchanged": True,
    "source_hashes": initial,
    "unchanged": True,
    "environment": {key: ENV[key] for key in base["environment"]},
}
(OUT / "sources.json").write_text(json.dumps(record, indent=2) + "\n")
status = []
commands = [
    ("format", ["cargo", "fmt", "--all", "--", "--check"]),
    ("real-all-target-tests", ["cargo", "test", "--locked", "--release", "-p", "hyperreal", "--all-features", "--tests", "--", "--test-threads=2"]),
    ("real-clippy", ["cargo", "clippy", "--locked", "--release", "-p", "hyperreal", "--all-features", "--all-targets", "--", "-D", "warnings"]),
]
for label, command in commands:
    assert all(digest(SOURCE / key) == value for key, value in initial.items())
    start = time.monotonic()
    print("start", label, flush=True)
    with (OUT / (label + ".log")).open("w") as log:
        result = subprocess.run(command, cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=False)
    status.append({"name": label, "command": command, "exit_code": result.returncode, "wall_seconds": time.monotonic() - start})
    (OUT / "status.json").write_text(json.dumps(status, indent=2) + "\n")
    assert all(digest(SOURCE / key) == value for key, value in initial.items())
    print("finish", label, result.returncode, flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)
assert all(digest(BASE / "source" / key) == value for key, value in base["source_hashes"].items())
print("API inventory and scalar integration validation complete", flush=True)
