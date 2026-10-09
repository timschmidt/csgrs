"""Preserve the fully validated first source, rejected for a separate proof loss."""

import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "area-compaction-isolated-trial1"
WORK = Path("/tmp/drc-area-compaction-isolated/source")
BASE = Path("/tmp/drc-point-equality-regressions/source")
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
source = read(OUT / "sources.json")
base = read(ROOT / "point-equality-regressions/sources.json")
assert source["unchanged"]
assert all(digest(WORK / name) == value for name, value in source["source_hashes"].items())
assert all(digest(BASE / name) == value for name, value in base["source_hashes"].items())
changes = {name for name, value in source["source_hashes"].items() if base["source_hashes"].get(name) != value}
assert changes == set(source["only_modified_sources"]) and len(changes) == 6
with tarfile.open(OUT / "source-delta.tar.gz") as archive:
    assert set(archive.getnames()) == changes
    for name in changes:
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == source["source_hashes"][name]
status = read(OUT / "status.json")
assert len(status) == 9 and status[-1]["name"] == "full-board-smoke"
assert all(command["exit_code"] == 0 for command in status)
decision = read(OUT / "decision.json")
for name, expected in [("real-default", (767, 0, 0)), ("real-all-features", (854, 0, 0)), ("curve-tests", (2420, 0, 28))]:
    rows = re.findall(r"test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored", (OUT / (name + ".log")).read_text())
    assert tuple(sum(int(row[index]) for row in rows) for index in range(3)) == expected
counter = read(OUT / "sign-counterexample.json")
assert all(command["exit_code"] == 0 for command in counter["commands"])
assert counter["same_counterexample_as_main_trial"]
assert (OUT / "sign-counterexample.log").read_bytes() == (ROOT / "area-compaction-sign-diagnostic/probe.log").read_bytes()
time_log = (OUT / "full-board-smoke-time.log").read_text()
for label, expected in [("User time (seconds)", "56.28"), ("System time (seconds)", "0.82"), ("Maximum resident set size (kbytes)", "891032"), ("Exit status", "0")]:
    assert f"{label}: {expected}" in time_log
record = {
    "decision": "rejected as written; validation evidence retained for the proof-retention repair",
    "previous_diagnostic": "diagnostics-29.json",
    "scope": "Only the six intended scalar/curve/test source files over admitted production, without the pending native-stroke prototypes.",
    "validation": decision["validation"],
    "strict_clippy": ["hyperreal release all targets/all features", "hypercurve release all targets with triangulation"],
    "strict_docs": "hyperreal and hypercurve release libraries, all features, RUSTDOCFLAGS=-D warnings",
    "board_gate": decision["board_gate"],
    "counterexample": "The original near-zero sign-loss probe repeats identically against these isolated production libraries: both signs are known before reconstruction, then Unknown at precision 0 and -512 afterward. Existing suites passing does not admit this regression.",
    "source_files_verified": len(source["source_hashes"]),
    "source_delta_archive_verified": True,
    "only_modified_sources": sorted(changes),
    "repair_state": "A separate source/target tree is validating transfer of valid sign certificates plus the unchanged public counterexample. This first source/target stays frozen.",
    "limitations": "The 28 existing ignored curve cases remain open and are named in the logs. This is not the full workspace/pre-push workflow matrix. Worker CPU was not printed; reported timing is whole-process CPU, and no speedup is claimed. Historical report replay and matched cost runs remain pending. All broader plan scope remains open.",
}
record["artifacts"] = {str(path.relative_to(ROOT)): {"bytes": path.stat().st_size, "sha256": digest(path)} for path in sorted(OUT.iterdir()) if path.is_file()}
for name in ["reproduce_area_isolated.py", "analyze_area_isolated_trial1.py"]:
    path = ROOT / name
    record["artifacts"][name] = {"bytes": path.stat().st_size, "sha256": digest(path)}
(ROOT / "diagnostics-30.json").write_text(json.dumps(record, indent=2) + "\n")
print("Verified 4,041 test passes, 28 existing ignored cases, strict lints/docs, board gate, and sign-loss rejection.")
