"""Keep the post-area-repair profile separate from release admission timing."""

import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = Path(__file__).resolve().parent
OUT = EVIDENCE / "native-area-compaction-profile"
read = lambda path: json.loads(path.read_text())
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
profile = read(OUT / "manifest.json")
observed = read(OUT / "observations.json")
source = read(Path(profile["source_reference"]))
gate = read(EVIDENCE / "diagnostics-34.json")
assert profile["source_unchanged"] and profile["binary_unchanged"]
assert all(digest(ROOT / name) == value for name, value in source["source_hashes"].items())
assert digest(Path(profile["binary"])) == profile["binary_sha256"]
assert profile["record_exit_code"] == 101 and profile["script_exit_code"] == 0
assert all(command["exit_code"] == 0 for command in profile["commands"][1:])
assert observed["samples"] == 10493 and observed["lost_samples"] == 0
assert observed["empty_stack_period_percent"] == 0
log = (OUT / "record.log").read_text()
worker = float(re.search(r"suite took ([\d.]+)s of worker time", log)[1])
assert worker == 211.729944788
assert list(map(int, re.findall(r"finished run \((\d+) active finding\(s\)", log))) == [12, 197]
record = {
    "decision": "diagnostic profile retained; no new production optimization admitted",
    "scope": profile["scope"],
    "source_files_verified": len(source["source_hashes"]),
    "ordinary_release_gate": gate["board_gate"],
    "profiled_worker_cpu_seconds": worker,
    "profiled_timing_is_not_admission": True,
    "samples": observed["samples"],
    "lost_samples": observed["lost_samples"],
    "empty_stack_period_percent": observed["empty_stack_period_percent"],
    "unknown_leaf_period_percent": observed["unknown_leaf_period_percent"],
    "groups": observed["groups"],
    "normal_form_contexts": observed["normal_form_contexts"],
    "nearest_recovered_normal_form_callers": observed["nearest_recovered_non_scalar_normal_form_callers"][:8],
    "leading_leaf_symbols": observed["leaf_symbols"][:15],
    "interpretation": "The current profile still spends about half its sampled cycles within bounded polynomial normalization, distributed across comparison, zero, equality and sign queries. The explicit Laurent compaction path accounts for less than one percent in recovered stacks. Allocator routines and coefficient-vector cloning are prominent leaf costs. This suggests repeated symbolic proof work remains a useful target; it does not prove that a proposed cache or alternative container would improve the complete workload.",
    "next_step": "Measure repeated eligible closed-form computations by live expression-node identity in a separately instrumented diagnostic. Quantify the reuse opportunity before adding a persistent cache or changing memoization. Keep failed/opaque certificates, original proof budgets, source geometry and full fixtures explicit.",
    "limitations": "This changes code generation and adds sampling; timing is perturbed and cannot be compared as a release speedup. Cycle-period percentages are not call counts or wall-time percentages. Inclusive groups overlap. The decoder deliberately uses --no-inline, so wrappers such as layer_sanity can be absent from recovered ancestors; their small named group is not their total execution cost. Existing rejected Point2 shortcuts remain rejected. The native-prototype runtime gate and all broader plan scope remain open.",
}
paths = [path for path in sorted(OUT.iterdir()) if path.is_file()]
paths += [EVIDENCE / name for name in ["reproduce_native_area_profile_build.py", "reproduce_native_area_profile.py", "analyze_native_area_profile.py"]]
record["artifacts"] = {str(path.relative_to(EVIDENCE)): {"bytes": path.stat().st_size, "sha256": digest(path)} for path in paths}
(EVIDENCE / "diagnostics-35.json").write_text(json.dumps(record, indent=2) + "\n")
print("Verified 10,493 samples, zero losses, unchanged full sources, and diagnostic-only normalization attribution.")
