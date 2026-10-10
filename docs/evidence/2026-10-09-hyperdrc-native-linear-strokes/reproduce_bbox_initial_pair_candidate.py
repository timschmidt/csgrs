"""Evaluate exact bounding-box initialization in a frozen isolated workspace."""

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
OUT = EVIDENCE / "bbox-initial-pair-candidate"
WORK = ROOT / "target/native-bbox-initial-pair"
SOURCE, TARGET = WORK / "source", WORK / "target"
BASE = ROOT / "target/native-normal-form-root-reuse"
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())
native = read(EVIDENCE / "native-area-compaction-gate/sources.json")
base = read(EVIDENCE / "normal-form-root-reuse-diagnostic/sources.json")
assert all(digest(ROOT / name) == value for name, value in native["source_hashes"].items())
assert all(digest(BASE / "source" / name) == value for name, value in base["source_hashes"].items())
OUT.mkdir()
WORK.mkdir()
for folder in ["source", "target"]:
    with (OUT / ("copy-" + folder + ".log")).open("w") as log:
        subprocess.run(["cp", "-a", "--reflink=auto", str(BASE / folder), str(WORK / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
old_helper = SOURCE / "crates/hyperreal/src/computable/node/normal_form_root_reuse_diagnostic.rs"
assert old_helper.is_file()
old_helper.unlink()
for name, value in native["source_hashes"].items():
    path = SOURCE / name
    path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROOT / name, path)
    assert digest(path) == value
bbox = "crates/hypercurve/src/bbox.rs"
regressions = "crates/hypercurve/tests/bbox_initial_pair.rs"
path = SOURCE / bbox
text = path.read_text()
before = """        let mut min_x = first.x().clone();
        let mut min_y = first.y().clone();
        let mut max_x = first.x().clone();
        let mut max_y = first.y().clone();
"""
after = """        let Some(second) = points.next() else {
            return Classification::Decided(Self::from_point(first.clone()));
        };
        // The initial bounds coincide. One certified comparison per axis
        // establishes both extrema; comparing the same pair twice adds no proof.
        let Some((mut min_x, mut max_x)) = ordered_coordinate_pair(first.x(), second.x()) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some((mut min_y, mut max_y)) = ordered_coordinate_pair(first.y(), second.y()) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
"""
assert text.count(before) == 1
text = text.replace(before, after)
before = "fn include_coordinate(min: &mut Real, max: &mut Real, value: &Real) -> Option<()> {"
after = """fn ordered_coordinate_pair(first: &Real, second: &Real) -> Option<(Real, Real)> {
    Some(match compare_reals(second, first, &CurveContext::STRICT)? {
        Ordering::Less => (second.clone(), first.clone()),
        Ordering::Equal => (first.clone(), first.clone()),
        Ordering::Greater => (first.clone(), second.clone()),
    })
}

""" + before
assert text.count(before) == 1
path.write_text(text.replace(before, after))
shutil.copyfile(EVIDENCE / "bbox_initial_pair_regressions.rs", SOURCE / regressions)
ENV = os.environ | {"CARGO_HOME": "/tmp/csgrs-properties-cargo", "CCACHE_DIR": "/tmp/csgrs-properties-ccache", "CARGO_NET_OFFLINE": "true", "CARGO_TARGET_DIR": str(TARGET)}
with (OUT / "prepare-format.log").open("w") as log:
    subprocess.run(["cargo", "fmt", "--all"], cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
paths = (set(base["source_hashes"]) | set(native["source_hashes"]) | {regressions}) - {"crates/hyperreal/src/computable/node/normal_form_root_reuse_diagnostic.rs"}
expected = base["source_hashes"] | native["source_hashes"]
initial = {name: digest(SOURCE / name) for name in sorted(paths)}
assert {name for name, value in initial.items() if expected.get(name) != value} == {bbox, regressions}
with tarfile.open(OUT / "source-delta.tar.gz", "w:gz") as archive:
    for name in [bbox, regressions]:
        archive.add(SOURCE / name, arcname=name)
record = {
    "scope": "Only initial bounding-box extrema are computed from one certified comparison per axis. No scalar cache, proof limit or public signature changes. Main production Rust is untouched.",
    "native_source_record": str(EVIDENCE / "native-area-compaction-gate/sources.json"),
    "fallback_workspace_record": str(EVIDENCE / "normal-form-root-reuse-diagnostic/sources.json"),
    "only_changed_sources": [bbox, regressions],
    "source_hashes": initial,
    "source_unchanged": True,
    "main_production_unchanged": True,
    "environment": {key: ENV.get(key) for key in ["CARGO_HOME", "CCACHE_DIR", "CARGO_NET_OFFLINE", "CARGO_TARGET_DIR", "RUSTFLAGS", "CARGO_PROFILE_RELEASE_DEBUG"]},
}
(OUT / "sources.json").write_text(json.dumps(record, indent=2) + "\n")
shutil.copyfile(__file__, OUT / "reproduce.py")
status = []


def verify():
    assert all(digest(SOURCE / name) == value for name, value in initial.items())
    assert all(digest(ROOT / name) == value for name, value in native["source_hashes"].items())


def run(name, command, allow_failure=False):
    verify()
    print("start", name, flush=True)
    start = time.monotonic()
    with (OUT / (name + ".log")).open("w") as log:
        result = subprocess.run(command, cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=False)
    status.append({"name": name, "command": command, "exit_code": result.returncode, "wall_seconds": time.monotonic() - start})
    (OUT / "status.json").write_text(json.dumps(status, indent=2) + "\n")
    verify()
    print("finish", name, result.returncode, flush=True)
    if result.returncode and not allow_failure:
        raise SystemExit(result.returncode)


run("format", ["cargo", "fmt", "--all", "--", "--check"])
features = "triangulation,hyperreal/default,hyperreal/serde"
run("bbox-unit", ["cargo", "test", "--locked", "--release", "-p", "hypercurve", "--features", features, "--lib", "bbox::tests", "--", "--test-threads=1"])
run("bbox-public", ["cargo", "test", "--locked", "--release", "-p", "hypercurve", "--features", features, "--test", "hypercurve_bbox", "--test", "bbox_initial_pair", "--", "--test-threads=1"])
run("curve-regressions", ["cargo", "test", "--locked", "--release", "-p", "hypercurve", "--features", features, "--test", "contour_area_normal_form", "--test", "native_linear_stroke", "--", "--test-threads=1"])
run("strict-clippy", ["cargo", "clippy", "--locked", "--release", "-p", "hypercurve", "--features", features, "--all-targets", "--", "-D", "warnings"])
run("build-library", ["cargo", "build", "--locked", "--release", "-p", "hypercurve", "--features", features, "--message-format=json"])
print("Bounding-box initial-pair candidate passed focused correctness and lint gates", flush=True)
