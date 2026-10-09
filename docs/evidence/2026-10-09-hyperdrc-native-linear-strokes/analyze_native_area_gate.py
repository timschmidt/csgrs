"""Record the complete native-prototype gate after the exact-area admission."""

import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / "native-area-compaction-gate"
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())
source = read(OUT / "sources.json")
assert source["unchanged"]
assert all(digest(ROOT / name) == value for name, value in source["source_hashes"].items())
assert digest(Path(source["base_source_record"])) == source["base_source_record_sha256"]
assert digest(OUT / "source-delta.tar.gz") == source["source_delta_sha256"]
with tarfile.open(OUT / "source-delta.tar.gz") as archive:
    assert set(archive.getnames()) == set(source["delta_files"])
    for name in source["delta_files"]:
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == source["source_hashes"][name]
status = read(OUT / "status.json")
assert len(status) == 7 and all(command["exit_code"] == 0 for command in status[:-1])
assert status[-1]["name"] == "full-board-smoke" and status[-1]["exit_code"] == 101
counts = {}
for name, expected in [("scalar-composition", 6), ("curve-composition", 10), ("drc-native-strokes", 26)]:
    rows = re.findall(r"test result: ok\. (\d+) passed; 0 failed; 0 ignored", (OUT / (name + ".log")).read_text())
    counts[name] = sum(map(int, rows))
    assert counts[name] == expected
binary = read(OUT / "binary.json")
assert digest(Path(binary["path"])) == binary["sha256"]
log = (OUT / "full-board-smoke.log").read_text()
assert "skipping complex project fixture test" not in log
assert log.count("hyperdrc: finished emit report") == 2
worker = float(re.search(r"complex project KiCad smoke suite took ([\d.]+)s of worker time", log)[1])
assert worker == 207.576353903
findings = list(map(int, re.findall(r"finished run \((\d+) active finding\(s\)", log)))
assert findings == [12, 197]
checks = [{"check": name, "findings": int(count), "wall_seconds": float(seconds)} for name, count, seconds in re.findall(r"finished check ([^ ]+) \((\d+) finding\(s\), ([\d.]+)s\)", log)]
assert len(checks) == 8
time_log = (OUT / "full-board-smoke-time.log").read_text()
for label, expected in [("User time (seconds)", "206.34"), ("System time (seconds)", "1.57"), ("Elapsed (wall clock) time (h:mm:ss or m:ss)", "3:28.97"), ("Maximum resident set size (kbytes)", "1625476")]:
    assert f"{label}: {expected}" in time_log
record = {
    "decision": "broader native-stroke prototype remains unadmitted: original full-board runtime gate still fails",
    "scope": "Current frozen native prototype with the separately admitted exact-area/sign-proof repair; no production source edits during build, validation or measurement.",
    "admitted_area_revision": source["head"],
    "admitted_area_record": "diagnostics-33.json",
    "source_files_verified": len(source["source_hashes"]),
    "focused_tests": counts,
    "board_gate": {"passed": False, "worker_cpu_seconds": worker, "worker_cpu_limit_seconds": 120, "process_cpu_seconds": 207.91, "wall_seconds": 208.97, "peak_rss_kib": 1625476, "counts": {"cparti-fpga": 12, "hvp109a": 197}, "checks": checks},
    "comparison": "The earlier ownership-era native trial took 209.577392864 worker CPU seconds with the same 12/197 findings. These individual runs do not establish a causal speedup. Layer sanity remains about 94 seconds on CPArti, followed by about 69 seconds for copper-neck checks; the exact-area repair does not remove that bottleneck.",
    "next_step": "Inspect current native projection and retained-geometry costs within layer sanity and copper-neck work before choosing another performance candidate. Preserve exact source geometry, proof budgets, complete fixtures and the original gate.",
    "limitations": "Forty-two focused test invocations are not the complete native-prototype validation matrix. Final source/report oracle audits, matched native costs, readiness snapshots and broader properties.md scope remain open. No other runtime changes are admitted by this failed gate.",
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ["reproduce_native_area_gate.py", "analyze_native_area_gate.py"]]
record["artifacts"] = {str(path.relative_to(EVIDENCE)): {"bytes": path.stat().st_size, "sha256": digest(path)} for path in paths}
(EVIDENCE / "diagnostics-34.json").write_text(json.dumps(record, indent=2) + "\n")
print("Verified 42 focused passes and unchanged native gate failure at 207.576353903 worker CPU seconds.")
