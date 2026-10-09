#!/usr/bin/env python3
"""Freeze and run the public forty-capsule expression diagnostic.

This is a correctness diagnosis, not a performance/admission measurement.
Run from a tree matching the retained ownership source archive plus the
recorded Point2 documentation/tests. No production files may change mid-run.
"""

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import time


ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / (sys.argv[1] if len(sys.argv) > 1 else "area-compaction-scalar-trial1")
BASE = EVIDENCE / "normal-form-ownership-candidate"
PROBE = EVIDENCE / (sys.argv[2] if len(sys.argv) > 2 else "area_scalar_compaction_probe.rs")
BINARY = Path("/tmp/drc-area-scalar-compaction-probe")
ENV = os.environ | {
    "CARGO_HOME": "/tmp/csgrs-properties-cargo",
    "CCACHE_DIR": "/tmp/csgrs-properties-ccache",
    "CARGO_NET_OFFLINE": "true",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


OUT.mkdir()
baseline = json.loads((BASE / "sources.json").read_text())["source_hashes"]
extra = {"crates/hypercurve/src/point/equality_tests.rs", "crates/hyperreal/src/real/arithmetic/pi_laurent_compaction.rs"}
paths = sorted(set(baseline) | extra)
frozen = {name: digest(ROOT / name) for name in paths}
changed = [name for name in paths if frozen[name] != baseline.get(name)]
assert changed == sorted(extra | {
    "crates/hypercurve/src/point.rs", "crates/hyperreal/src/computable/node/algebra.rs", "crates/hyperreal/src/real/arithmetic.rs",
}), changed
with tarfile.open(OUT / "source-delta.tar.gz", "w:gz") as archive:
    for name in changed:
        archive.add(ROOT / name, arcname=name)
source = {
    "source_hashes": frozen,
    "base_archive": str((BASE / "sources.tar.gz").relative_to(EVIDENCE)),
    "base_archive_sha256": digest(BASE / "sources.tar.gz"),
    "delta_files": changed,
    "delta_archive_sha256": digest(OUT / "source-delta.tar.gz"),
    "probe_sha256": digest(PROBE),
    "unchanged": False,
}
write("sources.json", source)
status = {
    "scope": "experimental scalar compaction; focused tests and public expression diagnosis; not admission",
    "environment": {key: ENV[key] for key in ["CARGO_HOME", "CCACHE_DIR", "CARGO_NET_OFFLINE"]},
    "commands": [],
}


def check_sources():
    assert all(digest(ROOT / name) == value for name, value in frozen.items())
    assert digest(PROBE) == source["probe_sha256"]


def run(name, argv, timeout=900):
    check_sources()
    start = time.monotonic()
    print(name, flush=True)
    with (OUT / f"{name}.log").open("w") as stdout, (OUT / f"{name}.stderr").open("w") as stderr:
        try:
            result = subprocess.run(argv, cwd=ROOT, env=ENV, stdout=stdout, stderr=stderr, timeout=timeout, check=False)
            code = result.returncode
        except subprocess.TimeoutExpired:
            code = "timeout"
    check_sources()
    status["commands"].append({"name": name, "argv": argv, "exit_code": code, "wall_seconds": time.monotonic() - start})
    write("status.json", status)
    source["unchanged"] = True
    write("sources.json", source)
    if code != 0:
        print(f"{name}: {code}; see {OUT}", flush=True)
        sys.exit(1)


run("compiler", ["rustc", "-vV"])
run("format", ["cargo", "fmt", "--all", "--", "--check"])
run("scalar-tests", ["cargo", "test", "--locked", "--release", "-p", "hyperreal", "--lib", "pi_laurent_compaction_tests", "--", "--test-threads=1"])
run("build", ["cargo", "build", "--locked", "--release", "-p", "hypercurve", "--features", "triangulation,hyperreal/default,hyperreal/serde", "--message-format=json"])
libraries = {}
for line in (OUT / "build.log").read_text().splitlines():
    event = json.loads(line)
    if event.get("reason") != "compiler-artifact":
        continue
    name = event["target"]["name"]
    if name not in {"hypercurve", "hyperreal", "hyperlimit"}:
        continue
    path = next(Path(path) for path in event["filenames"] if path.endswith(".rlib"))
    libraries[name] = {"path": str(path), "sha256": digest(path), "features": event["features"]}
assert len(libraries) == 3, libraries.keys()
write("libraries.json", libraries)
argv = ["rustc", "--edition=2024", "-O", "-D", "warnings", str(PROBE), "-o", str(BINARY)]
for name, info in sorted(libraries.items()):
    argv.extend(["--extern", name + "=" + info["path"]])
for path in sorted({str(Path(info["path"]).parent) for info in libraries.values()}):
    argv.extend(["-L", "dependency=" + path])
run("compile-probe", argv)
write("binary.json", {"path": str(BINARY), "bytes": BINARY.stat().st_size, "sha256": digest(BINARY)})
run("probe", [str(BINARY)])
print("complete", flush=True)
