"""Count repeated covered reductions without changing production Rust sources."""

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
OUT = EVIDENCE / "normal-form-reuse-diagnostic"
WORK = ROOT / "target/native-normal-form-reuse"
SOURCE, TARGET = WORK / "source", WORK / "target"
BASE = Path("/tmp/drc-area-compaction-final")
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())
native = read(EVIDENCE / "native-area-compaction-gate/sources.json")
base = read(EVIDENCE / "area-compaction-final-source/sources.json")
assert all(digest(ROOT / name) == value for name, value in native["source_hashes"].items())
assert all(digest(BASE / "source" / name) == value for name, value in base["source_hashes"].items())
OUT.mkdir()
WORK.mkdir()
for folder in ["source", "target"]:
    with (OUT / ("copy-" + folder + ".log")).open("w") as log:
        subprocess.run(["cp", "-a", "--reflink=auto", str(BASE / folder), str(WORK / folder)], stdout=log, stderr=subprocess.STDOUT, check=True)
for name, value in native["source_hashes"].items():
    path = SOURCE / name
    path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROOT / name, path)
    assert digest(path) == value
algebra = "crates/hyperreal/src/computable/node/algebra.rs"
helper = "crates/hyperreal/src/computable/node/normal_form_reuse_diagnostic.rs"
path = SOURCE / algebra
text = path.read_text()
start = text.index("    fn pi_laurent_polynomial_with_memo(")
end = text.index("    pub(crate) fn bounded_laurent_rational", start)
body = text[start:end]
before = "            return value.clone();\n        }\n        if budget == 0 || *remaining_work == 0 {\n            return None;"
after = "            normal_form_reuse_diagnostic::memo_hit();\n            return value.clone();\n        }\n        let diagnostic_work_before = *remaining_work;\n        if budget == 0 || *remaining_work == 0 {\n            normal_form_reuse_diagnostic::budget_denial();\n            return None;"
assert body.count(before) == 1
body = body.replace(before, after)
before = "        if shared {\n            memo.get_or_insert_with"
after = "        normal_form_reuse_diagnostic::computed(self, shared, diagnostic_work_before - *remaining_work, result.as_ref());\n        if shared {\n            memo.get_or_insert_with"
assert body.count(before) == 1
body = body.replace(before, after)
path.write_text('include!("normal_form_reuse_diagnostic.rs");\n\n' + text[:start] + body + text[end:])
shutil.copyfile(EVIDENCE / "normal_form_reuse_diagnostic.rs", SOURCE / helper)
ENV = os.environ | {"CARGO_HOME": "/tmp/csgrs-properties-cargo", "CCACHE_DIR": "/tmp/csgrs-properties-ccache", "CARGO_NET_OFFLINE": "true", "CARGO_TARGET_DIR": str(TARGET)}
with (OUT / "prepare-format.log").open("w") as log:
    subprocess.run(["cargo", "fmt", "--all"], cwd=SOURCE, env=ENV, stdout=log, stderr=subprocess.STDOUT, check=True)
paths = set(base["source_hashes"]) | set(native["source_hashes"]) | {helper}
expected = base["source_hashes"] | native["source_hashes"]
initial = {name: digest(SOURCE / name) for name in sorted(paths)}
assert {name for name, value in initial.items() if expected.get(name) != value} == {algebra, helper}
with tarfile.open(OUT / "instrumentation.tar.gz", "w:gz") as archive:
    for name in [algebra, helper]:
        archive.add(SOURCE / name, arcname=name)
record = {
    "scope": "Diagnostic-only counters added to an isolated complete native-prototype source. No persistent certificate cache, mathematical operation, proof budget or decision branch is changed. Main production Rust remains untouched.",
    "native_source_record": str(EVIDENCE / "native-area-compaction-gate/sources.json"),
    "fallback_workspace_record": str(EVIDENCE / "area-compaction-final-source/sources.json"),
    "only_instrumented_sources": [algebra, helper],
    "source_hashes": initial,
    "source_unchanged": True,
    "main_production_unchanged": True,
    "environment": {key: ENV.get(key) for key in ["CARGO_HOME", "CCACHE_DIR", "CARGO_NET_OFFLINE", "CARGO_TARGET_DIR", "RUSTFLAGS", "CARGO_PROFILE_RELEASE_DEBUG"]},
    "identity_accounting": "A per-thread window of at most 16,384 nodes uses Weak references to prevent address reuse without increasing Arc strong counts. Windows clear on the next distinct node after reaching capacity. Repeated covered reductions are a windowed lower-bound observation, not a simulated cache hit rate or proof that budgets/semantics would remain unchanged under caching.",
    "approximation_kinds": ["Add", "Multiply", "Negate", "Offset", "Inverse", "Square", "Sqrt", "LinearCombination3", "other"],
    "row_fields": ["computed", "covered", "repeated_identity", "repeated_covered", "shared_covered", "repeated_shared_covered", "inclusive_work_units", "repeated_covered_inclusive_work_units"],
    "coverage_definition": "A computed polynomial is covered when no coefficient contains opaque atoms. Memo hits and depth/node-budget denials are counted separately. Work-unit sums are inclusive and can double count descendants; they are not CPU savings.",
    "limitations": "Instrumentation changes allocation lifetimes and timing, so neither timings nor allocation/RSS totals admit an optimization. Current source has no Arc::get_mut or Arc::make_mut path for these nodes; weak references do not alter the strong-count memoization decision. Full fixtures, exact formulas, assertions and the original board gate remain unchanged.",
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
run("scalar-regressions", ["cargo", "test", "--locked", "--release", "-p", "hyperreal", "--lib", "compact_pi_laurent", "--", "--test-threads=1"])
run("curve-regressions", ["cargo", "test", "--locked", "--release", "-p", "hypercurve", "--features", "triangulation,hyperreal/default,hyperreal/serde", "--test", "contour_area_normal_form", "--test", "native_linear_stroke", "--", "--test-threads=1"])
run("build-board-smoke", ["cargo", "test", "--locked", "--release", "-p", "hyperdrc", "--lib", "--no-run", "--message-format=json"])
executables = []
for line in (OUT / "build-board-smoke.log").read_text().splitlines():
    if line.startswith("{"):
        event = json.loads(line)
        if event.get("reason") == "compiler-artifact" and event.get("executable") and event["target"]["name"] == "hyperdrc":
            executables.append(event["executable"])
assert len(executables) == 1
(OUT / "binary.json").write_text(json.dumps({"path": executables[0], "sha256": digest(Path(executables[0]))}, indent=2) + "\n")
run("full-board-smoke", [executables[0], "zz_complex_project_performance_tests::kicad_board_completes_smoke_check_suite", "--exact", "--test-threads=1", "--nocapture"], allow_failure=True)
log = (OUT / "full-board-smoke.log").read_text()
assert "skipping complex project fixture test" not in log and log.count("hyperdrc: finished emit report") == 2
assert "NORMAL_FORM_REUSE_DIAGNOSTIC {" in log
print("Full-fixture reuse diagnostic complete; timing is not admission evidence", flush=True)
