"""Verify DRC consumers and unchanged complete production outputs."""

import gzip
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CONSUMERS = ROOT / "area-compaction-consumers-trial2"
REPLAY = ROOT / "area-compaction-production-replay"
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
consumers = read(CONSUMERS / "manifest.json")
replay = read(REPLAY / "manifest.json")
previous = read(ROOT / "normal-form-ownership-production-replay-corrected/manifest.json")
assert consumers["source_unchanged"] and replay["source_unchanged"]
assert len(consumers["status"]) == 3 and all(command["exit_code"] == 0 for command in consumers["status"])
assert all(command["exit_code"] == 0 for command in replay["commands"])
assert "test result: ok. 1559 passed; 0 failed; 0 ignored" in (CONSUMERS / "drc-tests.log").read_text()
assert digest(Path(consumers["executable"]["path"])) == consumers["executable"]["sha256"]
assert replay["historical_row_deltas"] == 0
assert read(REPLAY / "historical-row-deltas.json") == []
rows = read(REPLAY / "rows.json")
assert len(rows) == 231 and len(replay["cost_trials"]) == 10
for trial in replay["cost_trials"]:
    assert [{key: value for key, value in row.items() if key != "elapsed_ns"} for row in trial["rows"]] == rows
for name, variant in replay["variants"].items():
    source = read(Path(variant["source_record"]))
    assert all(digest(Path(variant["source_directory"]) / name) == value for name, value in source["source_hashes"].items())
    assert all(digest(Path(path)) == info["sha256"] for path, info in variant["libraries"].items())
    assert all(digest(Path(info["path"])) == info["sha256"] for info in variant["binaries"].values())
for board, info in replay["reports"].items():
    assert info["complete_bytes_identical_across_both_variants_and_repetitions"]
    for key in ["bytes", "sha256", "violation_count"]:
        assert info[key] == previous["reports"][board][key]
    for variant in ["baseline", "candidate"]:
        for index in [1, 2]:
            raw = (REPLAY / f"{variant}-reports-{index}" / (board + ".json")).read_bytes()
            assert gzip.decompress((REPLAY / f"{variant}-reports-{index}-{board}.json.gz").read_bytes()) == raw
            assert len(raw) == info["bytes"] and hashlib.sha256(raw).hexdigest() == info["sha256"]
record = {
    "decision": "consumer correctness and complete output-preservation gates passed; matched resource costs still pending",
    "scope": "Only the seven-file isolated Laurent reconstruction/area/test patch over admitted production; broader native-stroke changes remain excluded.",
    "previous_diagnostic": "diagnostics-31.json",
    "consumer_tests": {"passed": 1559, "failed": 0, "ignored": 0, "strict_clippy": "release/default features/all targets"},
    "capability_rows": {"rows": 231, "fresh_cpu_0_processes_per_variant": 5, "exact_witnesses_identical": True, "historical_deltas": 0},
    "complete_reports": replay["reports"],
    "report_trials": "Two fresh CPU-0-pinned processes per variant, each serializing all twelve checks for both full boards with the unchanged seven-record IPC input. Complete bytes match without normalization and equal the earlier admitted ownership baseline.",
    "only_modified_sources": replay["only_modified_sources"],
    "sources_and_libraries_verified": True,
    "fixture_recovery": "The initial run passed 1,558 tests and failed one file read because the inherited temporary snapshots omitted a committed fixture. The exact committed input is restored in every comparison workspace and the complete consumer suite is rerun; failed logs remain archived.",
    "limitations": "These are correctness/output checks and capability costs, not a broad speedup or full pre-push matrix. Focused matched construction/query timing, requested allocation counts, retained/peak bytes, and executable sizes still determine resource tradeoffs. The native-stroke prototype and all broader properties.md scope remain open.",
}
paths = [p for directory in [CONSUMERS, REPLAY, ROOT / "area-compaction-consumers", ROOT / "area-compaction-fixture-recovery"] for p in sorted(directory.rglob("*")) if p.is_file()]
paths += [ROOT / name for name in ["reproduce_area_consumers.py", "reproduce_area_production_replay.py", "analyze_area_production_replay.py"]]
record["artifacts"] = {str(path.relative_to(ROOT)): {"bytes": path.stat().st_size, "sha256": digest(path)} for path in paths}
(ROOT / "diagnostics-32.json").write_text(json.dumps(record, indent=2) + "\n")
print("Verified 1,559 consumer tests, strict Clippy, 231 unchanged rows, and all complete report bytes.")
