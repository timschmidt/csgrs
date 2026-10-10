"""Record the unchanged native-prototype gate after bounding-box admission."""

import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / "native-bbox-initial-pair-gate"
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())
source = read(OUT / "sources.json")
assert source["unchanged"]
assert all(digest(ROOT / name) == value for name, value in source["source_hashes"].items())
assert digest(Path(source["base_source_record"])) == source["base_source_record_sha256"]
assert digest(OUT / "source-delta.tar.gz") == source["source_delta_sha256"]
with tarfile.open(OUT / "source-delta.tar.gz") as archive:
    assert set(archive.getnames()) == set(source["delta_files"])
    for name in archive.getnames():
        assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == source["source_hashes"][name]
prior = read(EVIDENCE / "bbox-initial-pair-candidate/sources.json")
assert all(prior["source_hashes"].get(name) == value for name, value in source["source_hashes"].items())
assert sum(source["reused_focused_validation"]["tests_passed"].values()) == 29
status = read(OUT / "status.json")
assert len(status) == 4 and all(command["exit_code"] == 0 for command in status[:-1])
assert status[-1]["name"] == "full-board-smoke"
passed = status[-1]["exit_code"] == 0
assert status[-1]["exit_code"] in [0, 101]
binary = read(OUT / "binary.json")
assert digest(Path(binary["path"])) == binary["sha256"]
log = (OUT / "full-board-smoke.log").read_text()
assert "skipping complex project fixture test" not in log
assert log.count("hyperdrc: finished emit report") == 2
worker = re.search(r"complex project KiCad smoke suite took ([\d.]+)s of worker time", log)
if not passed:
    assert worker and float(worker[1]) > 120
worker = float(worker[1]) if worker else None
findings = list(map(int, re.findall(r"finished run \((\d+) active finding\(s\)", log)))
assert findings == [12, 197]
checks = [{"check": name, "findings": int(count), "wall_seconds": float(seconds)} for name, count, seconds in re.findall(r"finished check ([^ ]+) \((\d+) finding\(s\), ([\d.]+)s\)", log)]
assert len(checks) == 8
time_log = (OUT / "full-board-smoke-time.log").read_text()
resource = lambda label: re.search(re.escape(label) + r": ([^\n]+)", time_log)[1]
user = float(resource("User time (seconds)"))
system = float(resource("System time (seconds)"))
record = {
    "decision": "broader native-stroke prototype remains unadmitted pending its full source/report/resource review" if passed else "broader native-stroke prototype remains unadmitted: original full-board runtime gate still fails",
    "scope": "Frozen native prototype with the separately admitted exact bounding-box initializer; no source edits, other builds or tests during the full-board measurement.",
    "admitted_bbox_revision": source["head"],
    "admitted_bbox_record": "diagnostics-41.json",
    "source_files_verified": len(source["source_hashes"]),
    "reused_focused_validation": source["reused_focused_validation"],
    "board_gate": {"passed": passed, "worker_cpu_seconds": worker, "worker_cpu_limit_seconds": 120, "process_cpu_seconds": round(user + system, 2), "wall_elapsed": resource("Elapsed (wall clock) time (h:mm:ss or m:ss)"), "peak_rss_kib": int(resource("Maximum resident set size (kbytes)")), "counts": {"cparti-fpga": 12, "hvp109a": 197}, "checks": checks},
    "comparison": "The earlier exact-area native trial took 207.576353903 worker CPU seconds with the same 12/197 findings. Individual gate runs do not establish a causal speedup; the paired production query measurements are recorded separately in diagnostics-41.",
    "next_step": "Reproduce the finite-input overflow concern found in the existing floating-point ray rejection filter before further optimization. Then evaluate the previously recorded redundant endpoint and boundary-loop equality work, preserving exact comparisons, typed uncertainty, validation order and the original board gate.",
    "limitations": "The 29 focused native-candidate passes and strict Clippy are reused after source-hash equality checks, not rerun. Full native validation, source/report oracle audits, matched native costs, readiness snapshots and broader properties.md scope remain open. A passing gate alone would not admit the broader prototype.",
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ["reproduce_native_bbox_gate.py", "analyze_native_bbox_gate.py"]]
record["artifacts"] = {str(path.relative_to(EVIDENCE)): {"bytes": path.stat().st_size, "sha256": digest(path)} for path in paths}
(EVIDENCE / "diagnostics-42.json").write_text(json.dumps(record, indent=2) + "\n")
print(json.dumps(record["board_gate"], indent=2))
