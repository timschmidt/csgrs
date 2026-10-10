"""Freeze the native-stroke prototype after exact bounding-box initialization admission."""

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import time

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
BASE = EVIDENCE / "native-area-compaction-gate"
OUT = EVIDENCE / "native-bbox-initial-pair-gate"
OUT.mkdir()
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
base = json.loads((BASE / "sources.json").read_text())
paths = set(base["source_hashes"])
for package in ["hyperreal", "hyperlimit", "hyperlattice", "hypersolve", "hypercurve", "hyperdrc", "hypermesh", "hyperpath", "hyperphysics", "hypertri"]:
    for path in (ROOT / "crates" / package).rglob("*"):
        if path.is_file() and "target" not in path.parts and (path.suffix == ".rs" or path.name in ["Cargo.toml", "Cargo.lock"] or "fixtures" in path.parts):
            paths.add(str(path.relative_to(ROOT)))
initial = {name: digest(ROOT / name) for name in sorted(paths)}
changes = [name for name, value in initial.items() if base["source_hashes"].get(name) != value]
assert set(changes) == {"crates/hypercurve/src/bbox.rs", "crates/hypercurve/tests/bbox_initial_pair.rs"}
validated = json.loads((EVIDENCE / "bbox-initial-pair-candidate/sources.json").read_text())
assert all(validated["source_hashes"].get(name) == value for name, value in initial.items())
focused = json.loads((EVIDENCE / "diagnostics-38.json").read_text())
assert focused["format_and_strict_clippy_passed"]
assert sum(focused["tests_passed"].values()) == 29
with tarfile.open(OUT / "source-delta.tar.gz", "w:gz") as archive:
    for name in changes:
        archive.add(ROOT / name, arcname=name)
ENV = os.environ | {"CARGO_HOME": "/tmp/csgrs-properties-cargo", "CCACHE_DIR": "/tmp/csgrs-properties-ccache", "CARGO_NET_OFFLINE": "true", "CARGO_TARGET_DIR": str(ROOT / "target")}
record = {
    "scope": "Current complete native-stroke prototype plus admitted exact bounding-box initialization. Original unchanged full-board runtime gate. Twenty-nine focused native-candidate tests and strict Clippy are reused only after all 1152 recorded main source/Cargo/fixture hashes match their validated candidate counterparts; these tests are not rerun. Not broad native admission or a matched application speedup measurement.",
    "reused_focused_validation": {"record": "diagnostics-38.json", "source_record": "bbox-initial-pair-candidate/sources.json", "source_record_sha256": digest(EVIDENCE / "bbox-initial-pair-candidate/sources.json"), "tests_passed": focused["tests_passed"], "all_recorded_main_sources_identical": True},
    "head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
    "base_source_record": str(BASE / "sources.json"),
    "base_source_record_sha256": digest(BASE / "sources.json"),
    "source_hashes": initial,
    "delta_files": changes,
    "source_delta_sha256": digest(OUT / "source-delta.tar.gz"),
    "unchanged": True,
    "environment": {key: ENV.get(key) for key in ["CARGO_HOME", "CCACHE_DIR", "CARGO_NET_OFFLINE", "CARGO_TARGET_DIR", "RUSTFLAGS", "CARGO_PROFILE_RELEASE_DEBUG"]},
}
(OUT / "sources.json").write_text(json.dumps(record, indent=2) + "\n")
shutil.copyfile(__file__, OUT / "reproduce.py")
status = []


def verify():
    assert all(digest(ROOT / name) == value for name, value in initial.items())


def run(name, command):
    verify()
    print("start", name, flush=True)
    start = time.monotonic()
    with (OUT / (name + ".log")).open("w") as log:
        result = subprocess.run(command, cwd=ROOT, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=False)
    status.append({"name": name, "command": command, "exit_code": result.returncode, "wall_seconds": time.monotonic() - start})
    (OUT / "status.json").write_text(json.dumps(status, indent=2) + "\n")
    verify()
    print("finish", name, result.returncode, flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)


run("compiler", ["rustc", "-vV"])
run("format", ["cargo", "fmt", "--all", "--", "--check"])
run("build-board-smoke", ["cargo", "test", "--locked", "--release", "-p", "hyperdrc", "--lib", "--no-run", "--message-format=json"])
executables = []
for line in (OUT / "build-board-smoke.log").read_text().splitlines():
    if line.startswith("{"):
        event = json.loads(line)
        if event.get("reason") == "compiler-artifact" and event.get("executable") and event["target"]["name"] == "hyperdrc":
            executables.append(event["executable"])
assert len(executables) == 1
(OUT / "binary.json").write_text(json.dumps({"path": executables[0], "sha256": digest(Path(executables[0]))}, indent=2) + "\n")
run("full-board-smoke", ["/usr/bin/time", "-v", "-o", str(OUT / "full-board-smoke-time.log"), executables[0], "zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite", "--exact", "--test-threads=1", "--nocapture"])
log = (OUT / "full-board-smoke.log").read_text()
assert "skipping complex project fixture test" not in log and log.count("hyperdrc: finished emit report") == 2
print("Original native-prototype board gate passed", flush=True)
