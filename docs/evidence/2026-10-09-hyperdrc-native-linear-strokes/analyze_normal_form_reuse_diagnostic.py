"""Check full-fixture, bounded-window expression-identity observations."""

import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / "normal-form-reuse-diagnostic"
SOURCE = ROOT / "target/native-normal-form-reuse/source"
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(OUT / "sources.json")
native = read(Path(source["native_source_record"]))
assert all(digest(SOURCE / name) == value for name, value in source["source_hashes"].items())
assert all(digest(ROOT / name) == value for name, value in native["source_hashes"].items())
with tarfile.open(OUT / "instrumentation.tar.gz") as archive:
    assert set(archive.getnames()) == set(source["only_instrumented_sources"])
    for name in archive.getnames():
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == source["source_hashes"][name]
status = read(OUT / "status.json")
assert len(status) == 5 and all(command["exit_code"] == 0 for command in status[:-1])
assert status[-1]["name"] == "full-board-smoke" and status[-1]["exit_code"] in [0, 101]
for name, count in [("scalar-regressions", 6), ("curve-regressions", 10)]:
    passed = re.findall(r"test result: ok\. (\d+) passed; 0 failed; 0 ignored", (OUT / (name + ".log")).read_text())
    assert sum(map(int, passed)) == count
binary = read(OUT / "binary.json")
assert digest(Path(binary["path"])) == binary["sha256"]
log = (OUT / "full-board-smoke.log").read_text()
assert "skipping complex project fixture test" not in log
assert log.count("hyperdrc: finished emit report") == 2
assert list(map(int, re.findall(r"finished run \((\d+) active finding\(s\)", log))) == [12, 197]
if status[-1]["exit_code"]:
    assert "complex project KiCad smoke suite took " in log and "s of worker time" in log
    assert "test result: FAILED. 0 passed; 1 failed; 0 ignored" in log
records = []
decoder = json.JSONDecoder()
marker = "NORMAL_FORM_REUSE_DIAGNOSTIC "
for line in log.splitlines():
    if marker in line:
        record, consumed = decoder.raw_decode(line.split(marker, 1)[1])
        assert record["capacity"] == 16_384 and len(record["rows"]) == 9
        assert all(len(row) == 8 for row in record["rows"])
        assert record["nodes_inserted"] == record["window_resets"] * record["capacity"] + record["final_window_entries"]
        assert sum(row[0] - row[2] for row in record["rows"]) == record["nodes_inserted"]
        for row in record["rows"]:
            assert 0 <= row[3] <= min(row[1], row[2]) <= row[0]
            assert 0 <= row[5] <= min(row[3], row[4]) <= row[1]
            assert row[6] >= row[0] and row[7] >= row[3]
        records.append(record)
assert records
rows = [[sum(record["rows"][kind][field] for record in records) for field in range(8)] for kind in range(9)]
by_kind = {kind: dict(zip(source["row_fields"], values)) for kind, values in zip(source["approximation_kinds"], rows)}
compound = {field: sum(row[index] for row in rows[:8]) for index, field in enumerate(source["row_fields"])}
observations = {
    "scope": "Completed thread-local records from the original full two-board fixture; no geometry fixture or assertion was reduced. Windowed expression identity is protected by Weak references, with no extra strong references and no cached polynomial values.",
    "received_thread_records": len(records),
    "raw_thread_records": records,
    "per_kind": by_kind,
    "compound_kinds": compound,
    "compound_covered_percent": 100 * compound["covered"] / compound["computed"],
    "compound_repeated_covered_percent_of_computations": 100 * compound["repeated_covered"] / compound["computed"],
    "compound_repeated_covered_percent_of_covered": 100 * compound["repeated_covered"] / compound["covered"] if compound["covered"] else 0,
    "memo_hits": sum(record["memo_hits"] for record in records),
    "budget_denials": sum(record["budget_denials"] for record in records),
    "window_resets": sum(record["window_resets"] for record in records),
    "identity_accounting": source["identity_accounting"],
    "coverage_definition": source["coverage_definition"],
    "limitations": "Only completed thread-local records are observed. Repeated covered values are candidate reuse opportunities, not measured cache hits or savings. Work totals overlap through descendants. A cache would still need proof-budget/cold-warm/concurrency/abort/serialization checks and matched cost/memory/size evidence. Timing and allocation lifetimes are perturbed by the diagnostic.",
}
(OUT / "observations.json").write_text(json.dumps(observations, indent=2) + "\n")
record = {
    "decision": "reuse diagnostic retained; no persistent certificate cache or other runtime optimization admitted",
    "source_files_verified": len(source["source_hashes"]),
    "main_production_source_unchanged": True,
    "instrumented_sources": source["only_instrumented_sources"],
    "focused_tests_passed": 16,
    "complete_fixture_counts": {"cparti-fpga": 12, "hvp109a": 197},
    "observations": {key: value for key, value in observations.items() if key != "raw_thread_records"},
    "ordinary_release_gate": "diagnostics-34.json",
    "sampled_profile": "diagnostics-35.json",
    "next_step": "Separate top-level query reuse from recursive subexpression reuse before choosing a cache location. The existing lazy evaluation cache can hold graph-free certificates without changing Node layout or serialization. Recursive reuse also needs explicit cold/warm frontier checks at depth, node and term limits; these counts do not establish that safety or a speedup.",
    "limitations": "The original native runtime gate remains failed. The diagnostic adds no production API/cache and makes no speedup, allocation, coverage-completion or native-admission claim. All broader properties.md scope remains open.",
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ["normal_form_reuse_diagnostic.rs", "reproduce_normal_form_reuse_diagnostic.py", "analyze_normal_form_reuse_diagnostic.py"]]
record["artifacts"] = {str(path.relative_to(EVIDENCE)): {"bytes": path.stat().st_size, "sha256": digest(path)} for path in paths}
(EVIDENCE / "diagnostics-36.json").write_text(json.dumps(record, indent=2) + "\n")
print("Observed compound computations:", compound["computed"])
print("Repeated covered compound computations:", compound["repeated_covered"])
print("Share of compound computations:", observations["compound_repeated_covered_percent_of_computations"])
