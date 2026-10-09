"""Admit the exact-area completeness repair with measured resource tradeoffs."""

import difflib
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "area-compaction-admission"
OUT.mkdir(exist_ok=True)
REPLAY = ROOT / "area-compaction-production-replay"
COST = ROOT / "area-compaction-cost"
REVISION = "da123e54b"
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
validation = read(ROOT / "diagnostics-31.json")
consumer = read(ROOT / "diagnostics-32.json")
cost = read(COST / "manifest.json")
replay = read(REPLAY / "manifest.json")
source = Path("/tmp/drc-area-compaction-final/source")
baseline = Path("/tmp/drc-point-equality-regressions/source")
assert cost["source_unchanged"] and len(cost["commands"]) == 28
assert all(command["exit_code"] == 0 for command in cost["commands"])
assert len(cost["trials"]) == 20 and len(cost["summaries"]) == 108
assert consumer["capability_rows"]["historical_deltas"] == 0
assert validation["board_gate"]["passed"]
for variant in replay["variants"].values():
    hashes = read(Path(variant["source_record"]))["source_hashes"]
    assert all(digest(Path(variant["source_directory"]) / name) == value for name, value in hashes.items())
    assert all(digest(Path(name)) == value["sha256"] for name, value in variant["libraries"].items())
assert all(digest(Path(name)) == value["sha256"] for name, value in replay["additional_fixture_inputs"].items())
for binary in cost["binaries"].values():
    assert digest(Path(binary["path"])) == binary["sha256"]
assert digest(COST / "probe.rs") == cost["probe_sha256"] == digest(ROOT / "area_compaction_cost_probe.rs")
for trial in cost["trials"]:
    count = 128 if trial["mode"] == "time" else 64
    assert len(trial["rows"]) == 63
    witnesses = [row for row in trial["rows"] if "witness" in row]
    assert len(witnesses) == 9 and all(row["all_areas_exact"] and row["count"] == count for row in witnesses)
    assert all(row["iterations"] == count for row in trial["rows"] if "name" in row)
for summary in cost["summaries"]:
    for variant, fields in summary["variants"].items():
        rows = [next(row for row in trial["rows"] if row.get("name") == summary["name"]) for trial in cost["trials"] if trial["variant"] == variant and trial["mode"] == summary["mode"]]
        assert len(rows) == 5
        for field, values in fields.items():
            assert values["samples"] == [row[field] for row in rows]
            assert values["median"] == statistics.median(values["samples"])
            if summary["mode"] == "memory":
                assert len(set(values["samples"])) == 1

# Independently compare the workspace package inputs used by the DRC build
# with committed Git blobs. Historical audit copies, fuzz workspaces, demo
# hosting configuration and benchmark-result TOML are outside these checks.
package_roots = set()
for line in (REPLAY / "candidate-library-build.log").read_text().splitlines():
    if line.startswith("{"):
        event = json.loads(line)
        path = event.get("manifest_path", "")
        if event.get("reason") == "compiler-artifact" and path.startswith(str(source) + "/"):
            package_roots.add(str(Path(path).parent.relative_to(source)))
assert len(package_roots) == 11


def included(name):
    for package in package_roots:
        prefix = "" if package == "." else package + "/"
        if name in [prefix + suffix for suffix in ["Cargo.toml", "Cargo.lock", "build.rs"]]:
            return True
        if name.endswith(".rs") and any(name.startswith(prefix + folder + "/") for folder in ["src", "tests", "benches", "examples"]):
            return True
    return False


blobs = {}
for entry in subprocess.check_output(["git", "ls-tree", "-r", "-z", REVISION]).split(b"\0"):
    if not entry:
        continue
    meta, name = entry.split(b"\t")
    name = name.decode()
    if included(name):
        mode, kind, sha = meta.decode().split()
        assert kind == "blob"
        blobs[name] = sha
changes = set(validation["only_modified_sources"])
new = {"crates/hypercurve/tests/contour_area_normal_form.rs", "crates/hyperreal/src/real/arithmetic/pi_laurent_compaction.rs"}
audit = {}
for variant, directory in [("baseline", baseline), ("candidate", source)]:
    differences = []
    for name, sha in blobs.items():
        data = (directory / name).read_bytes()
        actual = hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()
        if actual != sha:
            differences.append(name)
    assert set(differences) == (set() if variant == "baseline" else changes - new)
    audit[variant] = {"checked_files": len(blobs), "missing": [], "different_from_revision": differences}
head_audit = {"revision": REVISION, "package_roots": sorted(package_roots), "selection": "Cargo.toml/Cargo.lock/build.rs and tracked Rust under src/tests/benches/examples for the eleven workspace packages in the DRC build graph", "variants": audit}
(OUT / "committed-source-audit.json").write_text(json.dumps(head_audit, indent=2) + "\n")

patch = []
with tarfile.open(OUT / "source-delta.tar.gz", "w:gz") as archive:
    for name in sorted(changes):
        before = "" if name in new else subprocess.check_output(["git", "show", REVISION + ":" + name], text=True)
        after = (source / name).read_text()
        assert before != after
        patch.append(f"diff --git a/{name} b/{name}\n")
        if name in new:
            patch.append("new file mode 100644\n")
        patch.extend(difflib.unified_diff(before.splitlines(keepends=True), after.splitlines(keepends=True), fromfile="/dev/null" if name in new else "a/" + name, tofile="b/" + name))
        archive.add(source / name, arcname=name)
(OUT / "source-delta.patch").write_text("".join(patch))
shutil.copyfile(__file__, OUT / "reproduce.py")
selected_names = [
    "capsule-quadratic-length-pi-contour-first",
    "capsule-quadratic-length-pi-region-first",
    "capsule-huge-translation-contour-first",
    "capsule-huge-translation-region-first",
    "circle-pi-region-warm",
    "rectangle-pi-region-first",
]
selected = [row for row in cost["summaries"] if row["name"] in selected_names]
binary_deltas = {mode: cost["binaries"]["candidate-" + mode]["bytes"] - cost["binaries"]["baseline-" + mode]["bytes"] for mode in ["time", "memory"]}
record = {
    "decision": "keep as a scoped completeness repair, with the measured first-query cost increase explicitly accepted",
    "scope": "The seven-file scalar/area/regression/API-inventory patch only. Three loop-area consumers compact their own result through the existing bounded scalar recognizer. No global aggregation rebalance, proof-budget increase, layout change, persistent cache, or wider native-stroke change is retained.",
    "correctness": "The unchanged forty translated capsule fixture and independent forty-contour/material-minus-hole regressions now prove their analytic total exactly. Both independent tests returned Unknown on admitted production. Reversal, rebuilt boundaries and both policies retain certified answers. The original lost-sign candidate is rejected; valid sign proofs survive reconstruction in the retained repair.",
    "priority_decision": "Representational completeness takes priority over first-query performance. Compaction costs up to 33.785 additional microseconds per first contour query in this matrix (huge rational translation), while capsule region query medians and requested allocations decrease substantially and retained results shrink. Some circle/rectangle query costs also increase. All existing output witnesses and the original full-board gate are preserved. This is not a uniform optimization or a broad speedup claim.",
    "validation": {"scalar_and_curve": validation["tests"], "drc": consumer["consumer_tests"], "strict_clippy": validation["strict_clippy"] + ["hyperdrc release all targets"], "strict_docs": validation["strict_docs"], "board_gate": validation["board_gate"]},
    "output_preservation": {"capability_rows": consumer["capability_rows"], "reports": consumer["complete_reports"]},
    "measurement_scope": cost["scope"],
    "cost_rows": {"operations": 54, "analytic_witnesses": 9, "fresh_processes_per_variant_per_mode": 5, "timing_iterations_per_row": 128, "allocation_iterations_per_row": 64, "all_allocation_samples_repeat_exactly": True},
    "selected_costs": selected,
    "executable_size_growth_bytes": binary_deltas,
    "timing_limitations": "There is substantial variation even in unchanged construction controls. Per-operation medians describe these samples; no statistical significance, causal construction speedup, competitive advantage, application speedup, total resource bound, or globally cold scalar state is established. Requested allocation accounting excludes allocator metadata and native C heaps. Retained/peak counters are per measured operation, not a sum of application peak memory.",
    "frontier": "Opaque coefficients, exhausted existing depth/node/term budgets and the general exp/log integer-power representation return None. The caller retains its original exact value in those cases. Existing ignored curve tests and the wider native-stroke runtime gate remain open.",
    "only_modified_sources": sorted(changes),
    "final_source_record": str(ROOT / "area-compaction-final-source/sources.json"),
    "fixture_recovery_record": replay["fixture_recovery_record"],
    "committed_source_audit": head_audit,
    "prior_evidence": ["diagnostics-28.json", "diagnostics-29.json", "diagnostics-30.json", "diagnostics-31.json", "diagnostics-32.json"],
    "limitations": "No property or plan target is completed by this scoped repair. This is not the exhaustive workspace/pre-push matrix. The separate native-stroke prototype remains unadmitted; its current combined runtime must be measured separately.",
}
assert binary_deltas == {"time": 4640, "memory": 4592}
paths = [p for directory in [COST, OUT] for p in sorted(directory.iterdir()) if p.is_file()]
paths += [ROOT / name for name in ["reproduce_area_cost.py", "area_compaction_cost_probe.rs", "analyze_area_admission.py"]]
record["artifacts"] = {str(path.relative_to(ROOT)): {"bytes": path.stat().st_size, "sha256": digest(path)} for path in paths}
(ROOT / "diagnostics-33.json").write_text(json.dumps(record, indent=2) + "\n")
print(f"Admission verified: seven source files, {len(blobs)} committed build-graph inputs, exact outputs, and measured resource tradeoffs.")
