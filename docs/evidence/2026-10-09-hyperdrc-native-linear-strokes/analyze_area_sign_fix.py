"""Verify the repaired isolated runtime and final API-inventory validation."""

import hashlib
import json
from pathlib import Path
import re
import tarfile

ROOT = Path(__file__).resolve().parent
RUNTIME = ROOT / "area-compaction-isolated-trial2"
FINAL = ROOT / "area-compaction-final-source"
BASE = ROOT / "point-equality-regressions"
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())
runtime = read(RUNTIME / "sources.json")
final = read(FINAL / "sources.json")
base = read(BASE / "sources.json")
for directory, record in [
    (Path("/tmp/drc-point-equality-regressions/source"), base),
    (Path("/tmp/drc-area-compaction-sign-fix/source"), runtime),
    (Path("/tmp/drc-area-compaction-final/source"), final),
]:
    assert all(digest(directory / name) == value for name, value in record["source_hashes"].items())
changes = {name for name, value in runtime["source_hashes"].items() if base["source_hashes"].get(name) != value}
assert changes == set(runtime["only_modified_sources"]) and len(changes) == 6
inventory = "crates/hyperreal/tests/gmp_api_coverage.rs"
assert {name for name, value in final["source_hashes"].items() if runtime["source_hashes"].get(name) != value} == {inventory}
for directory, record, expected in [(RUNTIME, runtime, changes), (FINAL, final, {inventory})]:
    with tarfile.open(directory / "source-delta.tar.gz") as archive:
        assert set(archive.getnames()) == expected
        for name in expected:
            assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == record["source_hashes"][name]
statuses = {"runtime": read(RUNTIME / "status.json"), "final": read(FINAL / "status.json")}
assert len(statuses["runtime"]) == 11 and statuses["runtime"][-1]["name"] == "sign-probe"
assert len(statuses["final"]) == 3 and statuses["final"][-1]["name"] == "real-clippy"
assert all(command["exit_code"] == 0 for status in statuses.values() for command in status)


def test_counts(path):
    rows = re.findall(r"test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored", path.read_text())
    assert rows
    return {key: sum(int(row[index]) for row in rows) for index, key in enumerate(["passed", "failed", "ignored"])} | {"test_binaries": len(rows)}


counts = {name: test_counts(RUNTIME / (name + ".log")) for name in ["real-default", "real-all-features", "curve-tests"]}
for name, expected in [("real-default", 768), ("real-all-features", 855), ("curve-tests", 2420)]:
    assert counts[name]["passed"] == expected and counts[name]["failed"] == 0
assert counts["curve-tests"]["ignored"] == 28
counts["final-real-all-target-tests"] = test_counts(FINAL / "real-all-target-tests.log")
assert counts["final-real-all-target-tests"]["passed"] > 855
assert counts["final-real-all-target-tests"]["failed"] == 0
assert "test every_public_numeric_api_is_benchmarked_or_explicitly_classified ... ok" in (FINAL / "real-all-target-tests.log").read_text()
before = read(ROOT / "area-compaction-api-inventory/before.json")
assert before["commands"][-1]["exit_code"] == 101
assert '["compact_pi_laurent"]' in (ROOT / "area-compaction-api-inventory/before.log").read_text()
signs = (RUNTIME / "sign-probe.log").read_text().splitlines()
assert len(signs) == 2
for line, scale, sign in zip(signs, [-7, 1], ["Negative", "Positive"]):
    assert line.split("\t") == [f"scale={scale}"] + [f"{name}=Some({sign})" for name in ["before", "immediate", "after_0", "after_512"]]
assert digest(ROOT / "area_compaction_sign_probe.rs") == runtime["sign_probe_sha256"]
assert digest(Path(runtime["sign_probe_library"]["path"])) == runtime["sign_probe_library"]["sha256"]
time_log = (RUNTIME / "full-board-smoke-time.log").read_text()
for label, value in [("User time (seconds)", "56.53"), ("System time (seconds)", "0.80"), ("Elapsed (wall clock) time (h:mm:ss or m:ss)", "0:57.62"), ("Maximum resident set size (kbytes)", "891196"), ("Exit status", "0")]:
    assert f"{label}: {value}" in time_log
board_log = (RUNTIME / "full-board-smoke.log").read_text()
for count in [12, 201]:
    assert f"finished run ({count} active finding(s)" in board_log
record = {
    "decision": "sign-proof regression repaired and isolated validation passed; production replay and matched resource measurements remain admission gates",
    "scope": "Bounded scalar Laurent reconstruction, three loop-area consumers, independent full forty-capsule regressions, and one public-API inventory entry. No other provisional native-stroke source is present.",
    "previous_diagnostic": "diagnostics-30.json",
    "tests": counts,
    "validation_commands": statuses,
    "strict_clippy": ["hyperreal release all targets/all features", "hypercurve release all targets with triangulation"],
    "strict_docs": "hyperreal and hypercurve release libraries, all features, RUSTDOCFLAGS=-D warnings",
    "sign_counterexample": "The unchanged public near-zero counterexample now retains both original signs immediately and at precision 0 and -512. Only valid exact-sign certificates are transferred; no approximation cache or recognition-budget increase is introduced.",
    "api_inventory": "The original audit failed solely on the new public method. Adding its explicit no-GMP-analog classification repairs the audit; symbolic expression reconstruction has no matching numeric GMP/MPFR operation.",
    "board_gate": {"passed": True, "worker_cpu_limit_seconds": 120, "worker_cpu_seconds_printed": False, "process_user_cpu_seconds": 56.53, "process_system_cpu_seconds": 0.80, "process_wall_seconds": 57.62, "peak_rss_kib": 891196, "counts": {"cparti-fpga": 12, "hvp109a": 201}},
    "source_files_verified": len(final["source_hashes"]),
    "source_archives_verified": True,
    "final_source_difference_from_runtime_validation": [inventory],
    "only_modified_sources": sorted(changes | {inventory}),
    "limitations": "Twenty-eight existing ignored curve cases remain open. This is not the complete workspace or pre-push workflow matrix. General exp/log integer-power graphs remain unsupported by the existing Laurent recognizer. No speedup or broad plan completion is claimed. The separate native-stroke prototype remains unadmitted.",
}
paths = [p for directory in [RUNTIME, FINAL, ROOT / "area-compaction-api-inventory"] for p in sorted(directory.iterdir()) if p.is_file()]
paths += [ROOT / name for name in ["reproduce_area_isolated_sign_fix.py", "reproduce_area_final_source.py", "analyze_area_sign_fix.py"]]
record["artifacts"] = {str(path.relative_to(ROOT)): {"bytes": path.stat().st_size, "sha256": digest(path)} for path in paths}
(ROOT / "diagnostics-31.json").write_text(json.dumps(record, indent=2) + "\n")
print("Verified repaired signs, full scalar/curve validation, API inventory, board gate, and source archives.")
