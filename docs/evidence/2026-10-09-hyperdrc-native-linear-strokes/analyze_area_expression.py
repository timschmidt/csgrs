"""Validate the full-size capsule diagnosis without admitting production code."""

import hashlib
import json
from pathlib import Path
import tarfile

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
equal = "Decided { value: Equal, certainty: Exact, stage: Structural }"
unknown = "Unknown { needed: RealRefinement, stage: Undecided }"
artifacts = {}
for directory in ["area-expression-diagnostic", "area-expression-separated"]:
    out = EVIDENCE / directory
    source = read(out / "sources.json")
    assert source["unchanged"] and len(source["source_hashes"]) == 1148
    assert all(digest(ROOT / name) == value for name, value in source["source_hashes"].items())
    base = EVIDENCE / source["base_archive"]
    assert digest(base) == source["base_archive_sha256"]
    assert digest(out / "source-delta.tar.gz") == source["delta_archive_sha256"]
    with tarfile.open(out / "source-delta.tar.gz") as archive:
        assert sorted(archive.getnames()) == source["delta_files"]
        for name in archive.getnames():
            assert hashlib.sha256(archive.extractfile(name).read()).hexdigest() == source["source_hashes"][name]
    status = read(out / "status.json")
    assert [command["name"] for command in status["commands"]] == ["compiler", "build", "compile-probe", "probe"]
    assert [command["exit_code"] for command in status["commands"]] == [0, 0, 0, 101 if directory.endswith("diagnostic") else 0]
    for path in out.iterdir():
        if path.is_file():
            artifacts[str(path.relative_to(EVIDENCE))] = {"sha256": digest(path), "bytes": path.stat().st_size}

lines = (EVIDENCE / "area-expression-separated/probe.log").read_text().splitlines()
contours = [line for line in lines if line.startswith("contour\t")]
loops = [line for line in lines if line.startswith("loop\t")]
compacted = [line for line in lines if line.startswith("compact_loop\t")]
assert contours == [f"contour\t{index}\tedges=6\tarcs=4" for index in range(40)]
assert len(loops) == len(compacted) == 40
assert all(f"original={equal}\tquadratic_chord=true" in line for line in loops)
assert all(line == f"compact_loop\t{index}\toriginal={equal}\tanalytic={equal}" for index, line in enumerate(compacted))
arcs = [line for line in lines if line.startswith("arc\t")]
assert len(arcs) == 4
assert all(f"perpendicular={equal}" in line and f"sweep_half_pi={equal}" in line for line in arcs)
aggregate = dict(line.split("\t")[1:] for line in lines if line.startswith("aggregate\t"))
assert aggregate == {
    "original": unknown,
    "original-compact-analytic": unknown,
    "original-loop-sum": unknown,
    "separated": unknown,
    "compacted-count": "40",
    "compacted": equal,
    "compacted-balanced": equal,
    "analytic-identity": equal,
}
for name in ["area_expression_probe.rs", "area_expression_separated_probe.rs", "reproduce_area_expression.py", "analyze_area_expression.py"]:
    path = EVIDENCE / name
    artifacts[name] = {"sha256": digest(path), "bytes": path.stat().st_size}
record = {
    "decision": "diagnostic-only; no production change admitted",
    "scope": "The unchanged full forty-capsule independent area fixture in the pending native-stroke prototype.",
    "previous_diagnostic": "diagnostics-25.json",
    "contract": "Each radius-one capsule has area 2*sqrt(dx^2+dy^2)+pi. Directions (1,2), (2,3), (3,5) occur 14,13,13 times at original translations (10*i+pi,17). The independent compact oracle is 28*sqrt(5)+26*sqrt(13)+26*sqrt(34)+40*pi.",
    "initial_probe_failure": "A diagnostic assumption of two recovered semicircles was false. The native view has two lines and four quarter-circle arcs per capsule. Preserve its failed assertion and original source; it is not a production failure.",
    "result": {
        "original_individual_areas_exact": 40,
        "compacted_areas_equal_original_and_analytic": 40,
        "recovered_contour_edges": 6,
        "recovered_contour_arcs": 4,
        "aggregate_comparisons": aggregate,
        "quadratic_compaction": "The algebraic chord-minus-radial-cross sum reduces with the existing exact quadratic certificate to 2*sqrt(5), 2*sqrt(13), or 2*sqrt(34). Circular-sector terms remain exact Real expressions.",
    },
    "inference": "The uncompact aggregate exceeds optional symbolic recognition capacity; separating terms alone is insufficient. Reducing each complete loop before accumulation is a candidate fix. These probes do not isolate which depth or aggregate-visit limit is reached.",
    "next_step": "Evaluate explicit bounded scalar normal-form reconstruction at loop integration boundaries. Preserve existing budgets, exact carriers, abort semantics, unsupported forms and retained provenance; add independent scalar and full-fixture regressions before admission.",
    "validation": {"source_files_hashed_before_and_after_each_command": 1148, "production_source_unchanged": True, "features": read(EVIDENCE / "area-expression-separated/libraries.json")},
    "limitations": "Public read-only probes only, not a matched benchmark or production regression fix. The native-stroke prototype and original board runtime failure remain pending; no property or target is completed. All 39 properties, 14 targets and crate future work remain in scope.",
    "artifacts": artifacts,
}
(EVIDENCE / "diagnostics-26.json").write_text(json.dumps(record, indent=2) + "\n")
print("Verified: 40 exact individual areas; original aggregate Unknown; compacted aggregate exact.")
