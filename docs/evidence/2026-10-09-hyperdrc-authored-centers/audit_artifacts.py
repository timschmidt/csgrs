"""Verify retained evidence, source reconstruction, output bytes and goal scope.

Run at the recorded candidate checkout after all validation and cost jobs exit.
This audit does not rerun compiled tests. Runtime binaries are checked when
present; their original hashes remain recorded for later reconstruction.
"""
import ast
import gzip
import hashlib
import json
from pathlib import Path
import re
import statistics
import subprocess

record = Path(__file__).resolve().parent
load = lambda path: json.loads(path.read_bytes())
digest = lambda data: hashlib.sha256(data).hexdigest()
info = lambda path: dict(bytes=Path(path).stat().st_size, sha256=digest(Path(path).read_bytes()))
manifest = load(record / "manifest.json")
base = manifest["base_revision"]
pattern = re.compile(r"^test result: (ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;", re.M)

def results(path, passed, failed=0):
    rows = pattern.findall(path.read_text())
    assert rows, path
    total = [sum(int(row[index]) for row in rows) for index in [1, 2, 3]]
    assert total == [passed, failed, 0], (path, total)
    return dict(passed=passed, failed=failed, ignored=0)

source_counts = {}
for folder, names in [
    ("validation", ["fmt", "clippy", "test", "docs", "fixture-build"]),
    ("consumer-validation", ["fmt", "consumer-clippy", "consumer-drc", "consumer-curves"]),
]:
    steps = load(record / folder / "status.json")
    assert [s["name"] for s in steps] == names
    assert all(s["exit_code"] == 0 for s in steps)
    sources = load(record / folder / "sources.json")
    assert sources["unchanged"]
    for p, expected in sources["source_hashes"].items():
        assert info(p)["sha256"] == expected, p
    source_counts[folder] = len(sources["source_hashes"])
tests = {
    "drc": results(record / "validation/test.log", 1416),
    "consumer_drc": results(record / "consumer-validation/consumer-drc.log", 9),
    "consumer_curves": results(record / "consumer-validation/consumer-curves.log", 5),
    "focused": results(record / "candidate-expanded.log", 49),
    "initial_candidate": results(record / "candidate-initial.log", 33),
    "baseline": results(record / "baseline.log", 5, 28),
    "readme_after_prose": results(record / "readme-validation/readme.log", 2),
}
readme = load(record / "readme-validation/manifest.json")
assert readme["unchanged"] and readme["exit_code"] == 0
for path, expected in readme["source_hashes"].items():
    assert info(path)["sha256"] == expected
baseline = load(record / "baseline.json")
assert baseline["production_commit"] == base and baseline["exit_code"] == 101
assert baseline["unchanged"]
for path, expected in baseline["source_hashes"].items():
    if path.endswith("/authored_center_regressions.rs"):
        data = (record / "baseline-regressions.rs").read_bytes()
    else:
        data = subprocess.check_output(["git", "show", f"{base}:{path}"])
        if path.endswith("/checks/mod.rs"):
            anchor = baseline["injection_anchor"].encode()
            assert data.count(anchor) == 1
            data = data.replace(anchor, anchor + baseline["test_module_registration"].encode())
    assert digest(data) == expected, path
assert len(baseline["source_hashes"]) == 122
for stage in ["candidate-initial", "candidate-expanded"]:
    run = load(record / f"{stage}.json")
    assert run["unchanged"] and run["exit_code"] == 0
reused = load(record / "baseline-reports/reused-cost-manifest.json")
for p, expected in reused["source_hashes"].items():
    assert digest(subprocess.check_output(["git", "show", f"{base}:{p}"])) == expected
before = load(record / "baseline-reports/manifest.json")
assert before["exit_code"] == 0
assert before["linked_libraries"]["/tmp/drc-authored-centers-baseline-libraries/libhyperdrc.rlib"]["sha256"] == "f015697d84c81ddd7a00a683b33f7e30b587cfd063a85774f40ed61e93aca781"
assert before["driver"] == info(record / "full_report_replay.rs")
for label, row in before["reports"].items():
    raw = gzip.decompress((record / f"baseline-reports/{label}.json.gz").read_bytes())
    assert len(raw) == row["bytes"] and digest(raw) == row["sha256"]

for path, row in load(record / "log-storage.json").items():
    archive = (record / row["raw_archive"]).read_bytes()
    raw = gzip.decompress(archive)
    assert digest(archive) == row["archive_sha256"]
    assert len(raw) == row["raw_bytes"] and digest(raw) == row["raw_sha256"]
    clean = b"\n".join(line.rstrip(b" \t\r") for line in raw.split(b"\n"))
    assert clean == (record / path).read_bytes()
    assert digest(clean) == row["display_sha256"]

snapshots = load(record / "snapshot-versions.json")
assert len(snapshots) == 6
for row in snapshots:
    old = subprocess.check_output(["git", "show", f"{base}:{row['path']}"])
    assert old.count(b'"check_version": 13') == row["changed_check_versions"] == 160
    assert Path(row["path"]).read_bytes() == old.replace(b'"check_version": 13', b'"check_version": 14')
    assert row["previous"] == 13 and row["current"] == 14 and row["all_other_bytes_unchanged"]

cost = load(record / "cost/manifest.json")
assert cost["source_unchanged"] and cost["fixture_exit_code"] == 0
assert cost["source_hashes"] == load(record / "validation/sources.json")["source_hashes"]
for key, name in [("probe_source", "operation"), ("affine_probe_source", "affine"), ("native_drill_probe_source", "native_drill"), ("ipc_probe_source", "ipc"), ("layer_order_probe_source", "layer_order"), ("point_consumer_probe_source", "point_consumer"), ("authored_center_probe_source", "authored_center")]:
    assert cost[key] == info(record / "cost" / f"{name}_probe.rs")
rows = load(record / "cost/rows.json")
assert dict(count=len(rows), **info(record / "cost/rows.json")) == cost["rows"] == manifest["cost"]["rows"]
assert len(rows) == 139
previous = (record.parent / "2026-10-09-hyperdrc-point-consumers/cost/rows.json").read_bytes()
assert (json.dumps(rows[:110], sort_keys=True, separators=(",", ":")) + "\n").encode() == previous
observations = []
for trial in range(1, 6):
    log = (record / f"cost/operation-{trial}.log").read_text()
    parsed = []
    for line in log.splitlines():
        if line.startswith("operation_"):
            name, iterations, nanoseconds, witness = line.split("\t")
            parsed.append(dict(name=name, iterations=int(iterations), ns_per_query=int(nanoseconds)/int(iterations), witness=witness))
    assert [{k: v for k, v in row.items() if k != "ns_per_query"} for row in parsed] == rows
    assert re.search(r"Exit status: 0\s*$", log)
    observations.append(dict(rows=parsed, peak_rss_kib=int(re.search(r"Maximum resident set size \(kbytes\): (\d+)", log)[1])))
assert observations == cost["observations"]
medians = {row["name"]: statistics.median(run["rows"][i]["ns_per_query"] for run in observations) for i, row in enumerate(rows)}
assert medians == cost["medians_ns"]
fixture = (record / "cost/fixture.log").read_text()
seconds = float(re.search(r"fixture_smoke_total ms=([\d.]+)", fixture)[1]) / 1000
assert seconds == manifest["cost"]["fixture_seconds"] < 120
assert re.search(r"Exit status: 0\s*$", fixture)
text_audit = load(record / "cost/fixture-report-audit.json")
assert text_audit["normalized_reports_unchanged"] and text_audit["ordered_check_counts_unchanged"]
assert text_audit["diff_lines"] == 0

subprocess.run(["python3", str(record / "audit_report_deltas.py"), str(record / "report-replay")], check=True, stdout=subprocess.PIPE)
delta_audit = load(record / "report-replay/baseline-delta-audit.json")
replay = load(record / "report-replay/manifest.json")
assert replay["source_unchanged"] and replay["source_hashes"] == cost["source_hashes"]
assert len(replay["trials"]) == 2 and all(t["exit_code"] == 0 for t in replay["trials"])
assert replay["cost_library_build"] == info(record / "cost/library-build.jsonl")
assert len(replay["reports"]) == 2
for label, data in replay["reports"].items():
    raw_reports = []
    for path, expected in data["compressed_artifacts"].items():
        assert info(record / "report-replay" / path) == expected
        raw_reports.append(gzip.decompress((record / "report-replay" / path).read_bytes()))
    assert len(raw_reports) == 2 and raw_reports[0] == raw_reports[1]
    raw = raw_reports[0]
    assert len(raw) == data["bytes"] and digest(raw) == data["sha256"]
    before = gzip.decompress((record / f"baseline-reports/{label}.json.gz").read_bytes())
    assert before.count(b'"check_version":13') == data["baseline_version_fields"] == 8
    expected = before.replace(b'"check_version":13', b'"check_version":14')
    changes = delta_audit["reports"][label]["diagnostic_changes"]
    for old, new in {(row["before"], row["after"]) for row in changes}:
        expected = expected.replace(json.dumps(old).encode(), json.dumps(new).encode())
    assert raw == expected
    assert data["identical_complete_bytes"]
    assert data["baseline_only_version_changed"] == (not changes)
    parsed = json.loads(raw)
    assert len(parsed["violations"]) == parsed["violation_count"] == data["violations"]
for path, expected in {**cost["binaries"], **replay["binaries"], **replay["linked_libraries"]}.items():
    if Path(path).exists():
        assert info(path) == expected, path

inventory = load(record / "constructor-inventory.json")
assert inventory["counts"]["base"]["CopperSpatialIndex::new"] == 54
assert inventory["counts"]["candidate"]["CopperSpatialIndex::new"] == 48

initial = load(record / "report-replay/initial-comparison.json")
assert initial["exit_code"] == 1
assert info(record / "report-replay/initial-replay-script.py")["sha256"] == initial["script_sha256"]
assert info(record / "report-replay/manifest.json")["sha256"] == initial["raw_gate_manifest_sha256"]

for path, expected in manifest["evidence_files"].items():
    assert info(record / path) == expected, path
for path, expected in manifest["other_input_hashes"].items():
    assert info(path)["sha256"] == expected, path
for path in record.glob("*.py"):
    ast.parse(path.read_text(), filename=str(path))
for path in record.rglob("*.json"):
    load(path)
old_status = json.loads(subprocess.check_output(["git", "show", f"{base}:docs/properties-status.json"]))
new_status = load(Path("docs/properties-status.json"))
assert old_status["scope"] == new_status["scope"]
for key, count in [("properties", 39), ("targets", 14)]:
    assert len(new_status[key]) == count
    assert [(r["id"], r["status"]) for r in old_status[key]] == [(r["id"], r["status"]) for r in new_status[key]]
assert [r["path"] for r in old_status["future_work"]["sources"]] == [r["path"] for r in new_status["future_work"]["sources"]]
result = dict(status="passed", test_results=tests, validated_source_counts=source_counts,
              baseline_reconstructed_source_count=122,
              original_110_cost_witnesses_preserved=True, all_139_cost_rows_stable=True,
              full_affected_check_reports_identical=True, baseline_version_fields_per_board=8,
              baseline_diagnostic_changes={"cparti-fpga":6,"hvp109a":0}, all_other_baseline_report_bytes_preserved=True,
              six_snapshots_only_version_changed=True, unchanged_fixture_seconds=seconds,
              full_39_property_14_target_future_work_scope_preserved=True)
(record / "artifact-audit.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
