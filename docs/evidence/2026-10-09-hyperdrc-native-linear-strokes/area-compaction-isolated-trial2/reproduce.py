"""Validate only Laurent compaction and loop-area consumers on admitted source."""

import difflib
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import time

REPO = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
BASE = Path("/tmp/drc-point-equality-regressions")
WORK = Path("/tmp/drc-area-compaction-sign-fix")
OUT = EVIDENCE / "area-compaction-isolated-trial2"
SOURCE, TARGET = WORK / "source", WORK / "target"
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
base_record = EVIDENCE / "point-equality-regressions/sources.json"
base = json.loads(base_record.read_text())
assert all(digest(BASE / "source" / name) == value for name, value in base["source_hashes"].items())
WORK.mkdir()
OUT.mkdir()
for folder in ["source", "target"]:
    subprocess.run(["cp", "-a", "--reflink=auto", str(BASE / folder if folder == "source" else Path("/tmp/drc-area-compaction-isolated/target")), str(WORK / folder)], check=True)
copied = [
    "crates/hyperreal/src/computable/node/algebra.rs",
    "crates/hyperreal/src/real/arithmetic.rs",
    "crates/hyperreal/src/real/arithmetic/pi_laurent_compaction.rs",
    "crates/hypercurve/tests/contour_area_normal_form.rs",
]
for name in copied:
    shutil.copyfile(REPO / name, SOURCE / name)
contour_name = "crates/hypercurve/src/contour.rs"
path = SOURCE / contour_name
text = path.read_text()
before = "    Ok(Some(area))\n}\n\nfn compute_contour_signed_x_first_moment"
after = "    // Reduce a complete integral before it is added to independent loops.\n    // Unsupported forms retain their exact expression and existing behavior.\n    Ok(Some(area.compact_pi_laurent().unwrap_or(area)))\n}\n\nfn compute_contour_signed_x_first_moment"
assert text.count(before) == 1 and after in (REPO / contour_name).read_text()
path.write_text(text.replace(before, after))
region_name = "crates/hypercurve/src/bezier_region.rs"
path = SOURCE / region_name
text = path.read_text()
before = "        Ok(Classification::Decided(Some(total)))"
after = "        Ok(Classification::Decided(Some(\n            total.compact_pi_laurent().unwrap_or(total),\n        )))"
for marker in ["\nfn boundary_area_moments<'a>(", "\nfn validate_retained_boundary_loop("]:
    prefix, suffix = text.split(marker, 1)
    left, found, right = prefix.rpartition(before)
    assert found and right == "\n    }\n}\n"
    text = left + after + right + marker + suffix
assert text.count("compact_pi_laurent") == 2
path.write_text(text)
changes = sorted(copied + [contour_name, region_name])
for name in changes:
    if name in base["source_hashes"]:
        assert subprocess.check_output(["git", "show", "HEAD:" + name], cwd=REPO) == (BASE / "source" / name).read_bytes()
paths = sorted(set(base["source_hashes"]) | set(changes))
hashes = lambda: {name: digest(SOURCE / name) for name in paths}
initial = hashes()
assert {name for name in paths if initial[name] != base["source_hashes"].get(name)} == set(changes)
with tarfile.open(OUT / "source-delta.tar.gz", "w:gz") as archive:
    for name in changes:
        archive.add(SOURCE / name, arcname=name)
patch = []
for name in changes:
    old = (BASE / "source" / name).read_text() if (BASE / "source" / name).exists() else ""
    patch.extend(difflib.unified_diff(old.splitlines(keepends=True), (SOURCE / name).read_text().splitlines(keepends=True), fromfile="a/" + name, tofile="b/" + name))
(OUT / "source-delta.patch").write_text("".join(patch))
shutil.copyfile(__file__, OUT / "reproduce.py")
ENV = os.environ | {
    "CARGO_HOME": "/tmp/csgrs-properties-cargo",
    "CARGO_TARGET_DIR": str(TARGET),
    "CCACHE_DIR": "/tmp/csgrs-properties-ccache",
    "CARGO_NET_OFFLINE": "true",
    "RUSTDOCFLAGS": "-D warnings",
}
record = {
    "scope": "Isolated Laurent compaction and three loop-integral consumers; no other native-stroke prototypes.",
    "production_baseline_revision": "45fcbd3e3",
    "base_source_record": str(base_record),
    "only_modified_sources": changes,
    "source_hashes": initial,
    "unchanged": True,
    "environment": {key: ENV[key] for key in ["CARGO_HOME", "CARGO_TARGET_DIR", "CCACHE_DIR", "CARGO_NET_OFFLINE", "RUSTDOCFLAGS"]},
}
(OUT / "sources.json").write_text(json.dumps(record, indent=2) + "\n")
status = []


def run(name, command):
    assert hashes() == initial
    start = time.monotonic()
    print("start", name, flush=True)
    with (OUT / (name + ".log")).open("w") as log:
        result = subprocess.run(command, cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, timeout=1800, check=False)
    status.append({"name": name, "command": command, "exit_code": result.returncode, "wall_seconds": time.monotonic() - start})
    (OUT / "status.json").write_text(json.dumps(status, indent=2) + "\n")
    record["unchanged"] = hashes() == initial
    (OUT / "sources.json").write_text(json.dumps(record, indent=2) + "\n")
    assert record["unchanged"]
    print("finish", name, result.returncode, flush=True)
    if result.returncode:
        raise SystemExit(result.returncode)


commands = [
    ("format", ["cargo", "fmt", "--all", "--", "--check"]),
    ("real-default", ["cargo", "test", "--locked", "--release", "-p", "hyperreal", "--lib", "--", "--test-threads=2"]),
    ("real-all-features", ["cargo", "test", "--locked", "--release", "-p", "hyperreal", "--all-features", "--lib", "--", "--test-threads=2"]),
    ("real-clippy", ["cargo", "clippy", "--locked", "--release", "-p", "hyperreal", "--all-features", "--all-targets", "--", "-D", "warnings"]),
    ("curve-tests", ["cargo", "test", "--locked", "--release", "-p", "hypercurve", "--features", "triangulation", "--lib", "--tests", "--", "--test-threads=2"]),
    ("curve-clippy", ["cargo", "clippy", "--locked", "--release", "-p", "hypercurve", "--features", "triangulation", "--all-targets", "--", "-D", "warnings"]),
    ("docs", ["cargo", "doc", "--locked", "--release", "-p", "hyperreal", "-p", "hypercurve", "--all-features", "--lib", "--no-deps"]),
    ("build-board-smoke", ["cargo", "test", "--locked", "--release", "-p", "hyperdrc", "--lib", "--no-run", "--message-format=json"]),
]
for name, command in commands:
    run(name, command)
executables = []
for line in (OUT / "build-board-smoke.log").read_text().splitlines():
    if not line.startswith("{"):
        continue
    event = json.loads(line)
    if event.get("reason") == "compiler-artifact" and event.get("executable") and event["target"]["name"] == "hyperdrc":
        executables.append(event["executable"])
assert len(executables) == 1
run("full-board-smoke", ["/usr/bin/time", "-v", "-o", str(OUT / "full-board-smoke-time.log"), executables[0], "zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite", "--exact", "--test-threads=1", "--nocapture"])
log = (OUT / "full-board-smoke.log").read_text()
assert "skipping complex project fixture test" not in log
assert log.count("hyperdrc: finished emit report") == 2
assert all(digest(BASE / "source" / name) == value for name, value in base["source_hashes"].items())
libraries = []
for line in (OUT / "build-board-smoke.log").read_text().splitlines():
    if not line.startswith("{"):
        continue
    event = json.loads(line)
    if event.get("reason") == "compiler-artifact" and event["target"]["name"] == "hyperreal":
        libraries.extend(Path(path) for path in event["filenames"] if path.endswith(".rlib"))
assert len(libraries) == 1
probe = EVIDENCE / "area_compaction_sign_probe.rs"
binary = Path("/tmp/drc-area-compaction-sign-fixed-probe")
record["sign_probe_sha256"] = digest(probe)
record["sign_probe_library"] = {"path": str(libraries[0]), "sha256": digest(libraries[0])}
run("compile-sign-probe", ["rustc", "--edition=2024", "-O", "-D", "warnings", str(probe), "--extern", "hyperreal=" + str(libraries[0]), "-L", "dependency=" + str(libraries[0].parent), "-o", str(binary)])
run("sign-probe", [str(binary)])
assert digest(probe) == record["sign_probe_sha256"]
assert digest(libraries[0]) == record["sign_probe_library"]["sha256"]
expected = [f"scale={scale}\tbefore=Some({sign})\timmediate=Some({sign})\tafter_0=Some({sign})\tafter_512=Some({sign})" for scale, sign in [(-7, "Negative"), (1, "Positive")]]
assert (OUT / "sign-probe.log").read_text().splitlines() == expected
print("isolated validation and sign retention complete", flush=True)
