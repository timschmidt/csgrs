"""Verify focused curve integration and the admitted-production counterexamples."""

import hashlib
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parent
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
read = lambda path: json.loads(path.read_text())
artifacts = {}
for index in [1, 2]:
    out = ROOT / f"area-compaction-curve-trial{index}"
    source = read(out / "sources.json")
    assert source["unchanged"] and len(source["source_hashes"]) == 1150
    base = ROOT / source["base_archive"]
    assert digest(base) == source["base_archive_sha256"]
    assert digest(out / "source-delta.tar.gz") == source["delta_archive_sha256"]
    reconstructed = {}
    for path in [base, out / "source-delta.tar.gz"]:
        with tarfile.open(path) as archive:
            for member in archive.getmembers():
                if member.isfile():
                    reconstructed[member.name] = hashlib.sha256(archive.extractfile(member).read()).hexdigest()
    assert reconstructed == source["source_hashes"]
    commands = read(out / "status.json")["commands"]
    assert [command["exit_code"] for command in commands] == ([0, 0, 101] if index == 1 else [0] * 6)
    for path in out.iterdir():
        if path.is_file():
            artifacts[str(path.relative_to(ROOT))] = {"bytes": path.stat().st_size, "sha256": digest(path)}
out = ROOT / "area-compaction-curve-trial2"
log = (out / "curve-tests.log").read_text()
for count in [2, 8]:
    assert f"test result: ok. {count} passed; 0 failed; 0 ignored;" in log
equal = "Decided { value: Equal, certainty: Exact, stage: Structural }"
log = (out / "probe.log").read_text()
for name in ["original", "original-compact-analytic", "original-loop-sum", "scalar_compacted"]:
    assert f"aggregate\t{name}\t{equal}" in log
baseline = ROOT / "area-compaction-baseline-regression"
prior = read(baseline / "manifest.json")
assert [command["exit_code"] for command in prior["commands"]] == [0, 101]
assert prior["probe_sha256"] == read(out / "sources.json")["source_hashes"]["crates/hypercurve/tests/contour_area_normal_form.rs"]
log = (baseline / "regression.log").read_text()
assert "0 passed; 2 failed; 0 ignored" in log
assert log.count("left: None") == log.count("right: Some(Equal)") == 2
for path in baseline.iterdir():
    if path.is_file():
        artifacts[str(path.relative_to(ROOT))] = {"bytes": path.stat().st_size, "sha256": digest(path)}
for name in ["reproduce_area_integration.py", "analyze_area_integration.py"]:
    path = ROOT / name
    artifacts[name] = {"bytes": path.stat().st_size, "sha256": digest(path)}
record = {
    "decision": "focused completeness improvement demonstrated; runtime changes not yet admitted",
    "previous_diagnostic": "diagnostics-27.json",
    "change": "Explicit bounded scalar reconstruction at complete mixed Contour2 integrals and both retained/native Bezier boundary-loop integral accumulators. No curve, source parameter, topology, or budget representation is replaced.",
    "original_fixture": "The unchanged forty translated diagonal capsules now prove strict exact aggregate equality. The regression additionally retains the original positive-sign and narrow enclosure checks.",
    "independent_regressions": "A direct analytic orthonormal-frame constructor, independent of stroke_path, builds all forty capsules. Tests cover individual reversal and cloned queries, full contour sums, material regions and boundary-path reconstruction under both policies, plus all forty capsules as holes in a rational rectangle.",
    "admitted_production_counterexamples": "Both new public regressions return Unknown at exact aggregate comparison against unchanged admitted production libraries; both pass with the candidate. The original sources, compiler command, library hashes and expected failures are retained.",
    "validation": {"focused_tests_passed": 10, "ignored": 0, "source_files_frozen_per_trial": 1150, "source_archives_reconstructed_and_verified": 2},
    "failure_preserved": "Trial 1 did not compile because the new test assumed From<LineSeg2/CircularArc2> for Segment2; corrected to explicit variants without changing geometry or assertions.",
    "next_step": "Complete isolated production validation, original board gate, historical exact-row/report replay and matched area timing/allocation/size evidence before retention.",
    "limitations": "No runtime admission or performance claim. The broader native-stroke prototype and its runtime gate remain pending. All 39 properties, 14 targets and crate future work remain in scope.",
    "artifacts": artifacts,
}
(ROOT / "diagnostics-28.json").write_text(json.dumps(record, indent=2) + "\n")
print("Verified full-fixture exactness, ten focused passes, and two admitted-production counterexamples.")
