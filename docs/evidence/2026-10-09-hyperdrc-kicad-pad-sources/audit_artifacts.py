"""Check the collected evidence against frozen sources and complete raw reports.

Run from the repository root after build_manifest.py. This checks the retained
artifacts, not a fresh execution of the tests, benchmarks or independent oracle.
"""
from collections import Counter
from pathlib import Path
import ast
import gzip
import hashlib
import json
import re
import subprocess

record = Path(__file__).resolve().parent
root = record.parents[2]


def load(path):
    return json.loads(path.read_bytes())


def info(data):
    return dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())


def frozen(mapping):
    for path, digest in mapping.items():
        assert info((root / path).read_bytes())["sha256"] == digest, path


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def delta(before, after, path=""):
    if isinstance(before, dict) and isinstance(after, dict):
        return [row for key in sorted(before.keys() | after.keys())
                for row in delta(before.get(key), after.get(key), path + "/" + key)]
    if isinstance(before, list) and isinstance(after, list) and len(before) == len(after):
        return [row for i, (a, b) in enumerate(zip(before, after))
                for row in delta(a, b, path + "/" + str(i))]
    return [] if canonical(before) == canonical(after) else [dict(path=path, before=before, after=after)]


main = load(record / "manifest.json")
steps = load(record / "validation/status.json")
assert len(steps) == 8 and all(row["exit_code"] == 0 for row in steps)
assert [row["name"] for row in steps] == [
    "fmt", "drc-clippy", "drc-tests", "drc-docs", "consumer-clippy",
    "consumer-drc", "consumer-curves", "fixture-build",
]
sources = load(record / "validation/sources.json")
assert sources["unchanged"] and len(sources["source_hashes"]) == 540
frozen(sources["source_hashes"])
fixtures = load(record / "validation/fixtures.json")
assert fixtures["unchanged"] and len(fixtures["source_hashes"]) == 6
frozen(fixtures["source_hashes"])
for name, expected in [("drc-tests", 1597), ("consumer-drc", 9), ("consumer-curves", 5)]:
    rows = re.findall(r"test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;",
                      (record / "validation" / (name + ".log")).read_text())
    assert tuple(map(sum, zip(*(map(int, row) for row in rows)))) == (expected, 0, 0)

base = main["base_revision"]
snapshots = load(record / "snapshot-versions.json")
assert len(snapshots) == 6
for row in snapshots:
    before = subprocess.check_output(["git", "show", base + ":" + row["path"]], cwd=root)
    after = (root / row["path"]).read_bytes()
    token = b'"check_version": 19'
    assert before.count(token) == row["versions"] == 160
    assert before.replace(token, b'"check_version": 20') == after
    assert info(before)["sha256"] == row["previous_sha256"]
    assert info(after)["sha256"] == row["actual_sha256"]

cost = load(record / "cost/manifest.json")
cost_raw = (record / "cost/rows.json").read_bytes()
cost_rows = json.loads(cost_raw)
previous_cost = record.parent / "2026-10-09-hypercurve-retained-circular-area/cost/rows.json"
assert previous_cost.read_bytes() == cost_raw
assert len(cost_rows) == 231 and info(cost_raw) == {k: cost["rows"][k] for k in ["bytes", "sha256"]}
assert len(cost["observations"]) == 5 and cost["fixture_exit_code"] == 0
assert not cost["previous_row_deltas"] and cost["source_unchanged"]
for trial in cost["observations"]:
    assert [{k: row[k] for k in ["name", "iterations", "witness"]} for row in trial["rows"]] == cost_rows
assert main["cost"]["fixture_seconds"] < main["cost"]["fixture_gate_seconds"] == 120

source = load(record / "source-audit/manifest.json")
identities = {}
for name, variant in source["variants"].items():
    assert len(variant["trials"]) == 5
    identities[name] = load(record / "source-audit" / name / "rows.json")
    assert len(identities[name]) == 19
    for trial in variant["trials"]:
        assert trial["exit_code"] == 0
        assert all(row["iterations"] == 50 for row in trial["rows"])
        assert [{k: row[k] for k in ["name", "input", "iterations", "witness"]}
                for row in trial["rows"]] == identities[name]
assert [{k: row[k] for k in ["name", "input", "iterations"]} for row in identities["baseline"]] == [
    {k: row[k] for k in ["name", "input", "iterations"]} for row in identities["candidate"]]
assert sum(a["witness"] != b["witness"] for a, b in zip(identities["baseline"], identities["candidate"])) == source["deltas"] == 18
native_components = custom_placements = 0
for row in source["variants"]["candidate"]["complete_boards"]:
    result = row["result"]
    assert result["all_admitted"]
    assert result["wrong_anchor"] == result["wrong_geometry"] == result["missing_expected_rows"] == 0
    raw_result = record / "source-audit/candidate" / (row["label"] + ".json")
    assert info(raw_result.read_bytes()) == row["artifact"]
    assert result == {k: len(v) if isinstance(v, list) else v for k, v in load(raw_result).items()}
    assert info(Path(result["source"]).read_bytes())["sha256"] == result["input_sha256"]
    native_components += result["matched_native_components"]
    custom_placements += result["custom_placement_only"]
assert (native_components, custom_placements) == (2837, 17)

reports = load(record / "report-replay/manifest.json")
audit = load(record / "report-replay/finding-audit.json")
assert len(reports["trials"]) == 2 and all(row["exit_code"] == 0 for row in reports["trials"])
for label, row in reports["reports"].items():
    raw = gzip.decompress((record / "report-replay" / ("trial-1-" + label + ".json.gz")).read_bytes())
    assert raw == gzip.decompress((record / "report-replay" / ("trial-2-" + label + ".json.gz")).read_bytes())
    assert info(raw) == {k: row[k] for k in ["bytes", "sha256"]}
    before = load(Path(audit["boards"][label]["previous"])) if not audit["boards"][label]["previous"].endswith(".gz") else json.loads(gzip.decompress(Path(audit["boards"][label]["previous"]).read_bytes()))
    after = json.loads(raw)
    old, new = before["violations"], after["violations"]
    assert len(new) == row["violations"] == after["violation_count"]
    diff_raw = gzip.decompress((record / "report-replay" / (label + "-finding-deltas.json.gz")).read_bytes())
    assert info(diff_raw) == audit["boards"][label]["complete_deltas"]
    diff = json.loads(diff_raw)
    assert sorted([x["before"] for x in diff["stable"]] + [x["index"] for x in diff["removed"]]) == list(range(len(old)))
    assert sorted([x["after"] for x in diff["stable"]] + [x["index"] for x in diff["added"]]) == list(range(len(new)))
    assert all(canonical(old[x["before"]]) == canonical(new[x["after"]]) for x in diff["stable"])
    assert all(x["finding"] == old[x["index"]] for x in diff["removed"])
    assert all(x["finding"] == new[x["index"]] for x in diff["added"])
    assert Counter(canonical(x) for x in old if x["check"] == "geometry-uncertainty") == Counter(
        canonical(x) for x in new if x["check"] == "geometry-uncertainty")
    assert delta({k: v for k, v in before.items() if k != "violations"},
                 {k: v for k, v in after.items() if k != "violations"}) == load(record / "report-replay" / (label + "-metadata-deltas.json"))

drills = load(record / "drill-audit/manifest.json")
assert drills["source_unchanged"]
for row in drills["results"]:
    assert row["exit_code"] == 0 and row["result"]["complete_report_pair_multiset_matches"]
    assert info(gzip.decompress((record / "oracle-inputs" / ("drills-" + row["label"] + ".json.gz")).read_bytes())) == row["oracle"]
    assert row["report"] == {k: reports["reports"][row["label"]][k] for k in ["bytes", "sha256"]}
assert sum(row["result"]["exact_drills_matched"] for row in drills["results"]) == 1932
assert [row["result"]["exact_clearance_pairs"] for row in drills["results"]] == [0, 192]
for bundle in [cost, source, reports, drills]:
    assert bundle["source_unchanged"] and bundle["source_hashes"] == sources["source_hashes"]
for row in load(record / "unchanged-core-source-audit.json")["rows"]:
    assert not row["changed"]
    frozen(row["source_hashes"])

ledger = load(root / "docs/properties-status.json")
prior_ledger = json.loads(subprocess.check_output(["git", "show", base + ":docs/properties-status.json"], cwd=root))
for key, count in [("properties", 39), ("targets", 14)]:
    assert len(ledger[key]) == count
    assert [(x["id"], x["status"]) for x in ledger[key]] == [(x["id"], x["status"]) for x in prior_ledger[key]]
for path in record.rglob("*.py"):
    ast.parse(path.read_text(), filename=str(path))
for path in record.rglob("*.json"):
    if path.name != "artifact-audit.json":
        load(path)


def references(value):
    if isinstance(value, dict):
        for item in value.values():
            references(item)
    elif isinstance(value, list):
        for item in value:
            references(item)
    elif isinstance(value, str) and value.endswith((".json", ".log")) and " " not in value:
        assert value == "artifact-audit.json" or (record / value).is_file(), value


references(main)
assert not list(record.rglob("*.pyc"))
artifacts = {str(path.relative_to(record)): info(path.read_bytes()) for path in sorted(record.rglob("*"))
             if path.is_file() and path.name != "artifact-audit.json"}
result = dict(status="passed", scope="Audit of retained artifacts; no new test/oracle execution or full-plan completion claim.",
              source_files=540, snapshot_files=6, validation_gates=8, drc_tests=1597, selected_consumer_tests=14,
              preserved_cost_rows=231, loader_rows=19, changed_loader_witnesses=18,
              native_pad_components=2837, custom_placements=17, exact_drills=1932,
              report_counts={label: row["violations"] for label, row in reports["reports"].items()},
              artifacts=artifacts, inventory_excludes=["artifact-audit.json"])
(record / "artifact-audit.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps({k: v for k, v in result.items() if k != "artifacts"}, indent=2))
