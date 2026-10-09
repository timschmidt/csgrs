"""Audit retained logs, source reconstruction, report bytes, and ledger scope.

Run from the repository root. Compiled tests and timed jobs are not rerun here.
Runtime binaries remain under the recorded /tmp paths for the original audit;
reproduction on another host should first use the separate replay scripts.
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
file_digest = lambda path: digest(Path(path).read_bytes())
info = lambda path: dict(bytes=Path(path).stat().st_size, sha256=file_digest(path))
manifest = load(record / "manifest.json")
base = manifest["base_revision"]

source_counts = {}
for folder, names in [
    ("validation", ["fmt", "clippy", "test", "docs", "fixture-build"]),
    ("consumer-validation", ["fmt", "consumer-clippy", "consumer-drc", "consumer-curves"]),
]:
    steps = load(record / folder / "status.json")
    assert [step["name"] for step in steps] == names
    assert all(step["exit_code"] == 0 for step in steps)
    assert all((record / folder / (step["name"] + ".log")).is_file() for step in steps)
    sources = load(record / folder / "sources.json")
    assert sources["unchanged"]
    for path, expected in sources["source_hashes"].items():
        assert file_digest(path) == expected, path
    source_counts[folder] = len(sources["source_hashes"])

pattern = re.compile(r"^test result: (ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;", re.M)


def results(path, passed, failed=0):
    rows = pattern.findall(path.read_text())
    assert rows, path
    totals = [sum(int(row[index]) for row in rows) for index in [1, 2, 3]]
    assert totals == [passed, failed, 0], (path, totals)
    assert all(row[0] == ("FAILED" if int(row[2]) else "ok") for row in rows)
    return dict(passed=totals[0], failed=totals[1], ignored=totals[2])


test_counts = {
    "drc": results(record / "validation/test.log", 1320),
    "consumer_drc": results(record / "consumer-validation/consumer-drc.log", 9),
    "consumer_curves": results(record / "consumer-validation/consumer-curves.log", 5),
    "focused": results(record / "draft/focused.log", 7),
    "baseline": results(record / "baseline.log", 2, 5),
    "before_test_updates": results(record / "draft/validation-before-test-updates/test.log", 1313, 2),
}
baseline = load(record / "baseline.json")
assert baseline["production_commit"] == base and baseline["exit_code"] == 101
assert baseline["unchanged"] and baseline["passed"] == 2 and baseline["failed"] == 5
for path, expected in baseline["source_hashes"].items():
    if path.endswith("/layer_order_regressions.rs"):
        data = (record / "baseline-regressions.rs").read_bytes()
    else:
        data = subprocess.check_output(["git", "show", f"{base}:{path}"])
        if path.endswith("/checks/mod.rs"):
            anchor = b"pub mod layer;\n"
            assert data.count(anchor) == 1
            data = data.replace(anchor, anchor + baseline["test_module_registration"].encode())
    assert digest(data) == expected, path
assert len(baseline["source_hashes"]) == 117
assert load(record / "baseline-replay-audit.json")["all_baseline_hashes_match_base_plus_test_injection"]
before_manifest = load(record / "baseline-full-board/manifest.json")
assert before_manifest["source_hashes"] == baseline["source_hashes"] and before_manifest["source_unchanged"]
assert before_manifest["witness_source"] == info(record / "baseline_full_board_witness.rs")
storage = baseline["log_storage"]
archive = (record / storage["raw_archive"]).read_bytes()
raw = gzip.decompress(archive)
assert digest(archive) == storage["archive_sha256"]
assert len(raw) == storage["raw_bytes"] and digest(raw) == storage["raw_sha256"]
assert b"\n".join(line.rstrip(b" \t\r") for line in raw.split(b"\n")) == (record / storage["display_log"]).read_bytes()
assert file_digest(record / storage["display_log"]) == storage["display_sha256"]

snapshots = load(record / "snapshot-versions.json")
assert len(snapshots) == 6
for row in snapshots:
    old = subprocess.check_output(["git", "show", f"{base}:{row['path']}"])
    assert old.count(b'"check_version": 11') == row["changed_check_versions"] == 160
    assert Path(row["path"]).read_bytes() == old.replace(b'"check_version": 11', b'"check_version": 12')
    assert row["previous"] == 11 and row["current"] == 12 and row["all_other_bytes_unchanged"]

cost = load(record / "cost/manifest.json")
assert cost["source_unchanged"] and cost["fixture_exit_code"] == 0
assert cost["source_hashes"] == load(record / "validation/sources.json")["source_hashes"]
for key, filename in [
    ("probe_source", "operation_probe.rs"),
    ("affine_probe_source", "affine_probe.rs"),
    ("native_drill_probe_source", "native_drill_probe.rs"),
    ("ipc_probe_source", "ipc_probe.rs"),
    ("layer_order_probe_source", "layer_order_probe.rs"),
]:
    assert cost[key] == info(record / "cost" / filename)
for filename, expected in cost["binaries"].items():
    assert info(filename) == expected, filename
rows = load(record / "cost/rows.json")
assert dict(count=len(rows), **info(record / "cost/rows.json")) == cost["rows"] == manifest["cost"]["rows"]
assert len(rows) == 75
previous = (record.parent / "2026-10-08-hyperdrc-ipc-native/cost/rows.json").read_bytes()
assert (json.dumps(rows[:71], sort_keys=True, separators=(",", ":")) + "\n").encode() == previous
assert cost["previous_seventy_one_witness_rows_unchanged"]
observations = []
for trial in range(1, 6):
    text = (record / f"cost/operation-{trial}.log").read_text()
    parsed = []
    for line in text.splitlines():
        if line.startswith("operation_"):
            name, iterations, nanoseconds, witness = line.split("\t")
            parsed.append(dict(name=name, iterations=int(iterations), ns_per_query=int(nanoseconds)/int(iterations), witness=witness))
    assert [{k: v for k, v in row.items() if k != "ns_per_query"} for row in parsed] == rows
    assert re.search(r"Exit status: 0\s*$", text)
    observations.append(dict(rows=parsed, peak_rss_kib=int(re.search(r"Maximum resident set size \(kbytes\): (\d+)", text)[1])))
assert observations == cost["observations"]
medians = {row["name"]: statistics.median(run["rows"][index]["ns_per_query"] for run in observations) for index, row in enumerate(rows)}
assert medians == cost["medians_ns"]
assert {row["name"]: medians[row["name"]] for row in rows[71:]} == manifest["cost"]["layer_order_medians_ns"]
rss = [run["peak_rss_kib"] for run in observations]
assert [min(rss), max(rss)] == manifest["cost"]["peak_rss_kib"]
fixture = (record / "cost/fixture.log").read_text()
seconds = float(re.search(r"fixture_smoke_total ms=([\d.]+)", fixture)[1]) / 1000
assert seconds == manifest["cost"]["fixture_seconds"] < manifest["cost"]["fixture_gate_seconds"] == 120
assert int(re.search(r"Maximum resident set size \(kbytes\): (\d+)", fixture)[1]) == manifest["cost"]["fixture_peak_rss_kib"]
assert re.search(r"Exit status: 0\s*$", fixture)

replay = load(record / "report-replay/manifest.json")
assert replay["source_unchanged"] and replay["source_hashes"] == cost["source_hashes"]
assert all(trial["exit_code"] == 0 for trial in replay["trials"])
assert len(replay["trials"]) == 2
assert replay["baseline_witness"] == info(record / "baseline-full-board/witness.json")
assert replay["cost_library_build"] == info(record / "cost/library-build.jsonl")
for name, expected in replay["sources"].items():
    assert info(record / f"{name}.rs") == expected
for path, expected in {**replay["binaries"], **replay["linked_libraries"], **replay["archives"]}.items():
    assert info(path) == expected, path
assert len(replay["reports"]) == 3
for label, data in replay["reports"].items():
    raw_reports = []
    for path, expected in data["compressed_artifacts"].items():
        assert info(record / "report-replay" / path) == expected
        raw_reports.append(gzip.decompress((record / "report-replay" / path).read_bytes()))
    assert len(raw_reports) == 2 and raw_reports[0] == raw_reports[1]
    raw = raw_reports[0]
    assert len(raw) == data["bytes"] and digest(raw) == data["sha256"]
    parsed = json.loads(raw)
    assert len(parsed["violations"]) == parsed["violation_count"] == data["violations"]
    assert data["identical_complete_bytes"] and parsed["summary"] == data["summary"]
before = load(record / "baseline-full-board/witness.json")["input_order_variants"]
after = load(record / "report-replay/witness.json")["input_order_results"]
canonical = next(row for row in before if row["layers"] == ["B.Cu", "In1.Cu"])
assert after == [canonical, canonical]
assert replay["canonical_duplicate_matches_baseline_variant"]
runtime_cost = Path(manifest["runtime"]["cost"])
runtime_replay = Path(manifest["runtime"]["report_replay"])
subprocess.run(["python3", str(record / "audit_fixture_order_repair.py"), str(runtime_cost), str(runtime_replay)], check=True, capture_output=True)
for name in ["fixture-report-audit.json", "order-repair-audit.json", "fixture-findings.diff", "fixture-report.diff"]:
    assert (record / "cost" / name).read_bytes() == (runtime_cost / name).read_bytes()
order_audit = load(record / "cost/order-repair-audit.json")
assert order_audit["all_other_rendered_findings_unchanged_after_path_id_row_normalization"]
assert order_audit["duplicate_header_counts_geometry_and_message_unchanged"]
assert order_audit["candidate_complete_witness_equals_canonical_baseline_variant"]
text_audit = load(record / "cost/fixture-report-audit.json")
assert len(text_audit["candidate_check_counts"]) == 9 and text_audit["ordered_check_counts_unchanged"]

for path, expected in manifest["evidence_files"].items():
    assert info(record / path) == expected, path
for path, expected in manifest["other_input_hashes"].items():
    assert file_digest(path) == expected, path
for path in record.glob("*.py"):
    ast.parse(path.read_text(), filename=str(path))
for path in record.rglob("*.json"):
    load(path)

old_status = json.loads(subprocess.check_output(["git", "show", f"{base}:docs/properties-status.json"]))
new_status = load(Path("docs/properties-status.json"))
assert old_status["scope"] == new_status["scope"]
for group, count in [("properties", 39), ("targets", 14)]:
    assert len(old_status[group]) == len(new_status[group]) == count
    for old, new in zip(old_status[group], new_status[group], strict=True):
        for key in ["id", "requirement", "status"]:
            assert old[key] == new[key], (group, key, new["id"])
assert new_status["future_work"]["inventory_complete"] == old_status["future_work"]["inventory_complete"]
assert [(row["path"], row["status"]) for row in new_status["future_work"]["sources"]] == [(row["path"], row["status"]) for row in old_status["future_work"]["sources"]]
result = dict(scope="Retained-log, source reconstruction and artifact integrity audit; compiled tests and timed jobs are not rerun here.",
              base_revision=base, final_source_hashes_match_current_files=source_counts,
              baseline_base_plus_test_injection_hashes_match=117, test_counts=test_counts,
              snapshot_only_version_changes=dict(boards=6, fields_per_board=160, previous=11, current=12),
              cost_rows=cost["rows"], previous_seventy_one_rows_byte_identical=True,
              cost_logs_match_manifest_and_medians=True, cost_peak_rss_kib=rss, fixture_seconds=seconds,
              complete_three_report_bytes_identical_between_two_processes=True,
              full_board_candidate_matches_canonical_baseline_variant=True,
              full_fixture_text_audits_replayed=True, baseline_raw_log_storage_verified=True,
              all_evidence_file_hashes_match=True,
              all_39_property_and_14_target_requirements_and_statuses_preserved=True,
              future_work_source_statuses_preserved=True)
(record / "artifact-audit.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
