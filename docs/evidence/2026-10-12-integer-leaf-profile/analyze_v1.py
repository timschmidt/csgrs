"""Aggregate recorded cycle periods once per recovered symbol per sample."""
import collections
import gzip
import hashlib
import json
from pathlib import Path
import re

here = Path(__file__).resolve().parent / "profile-v1"
manifest = json.loads((here / "manifest.json").read_text())
assert hashlib.sha256(Path(manifest["binary"]).read_bytes()).hexdigest() == manifest["binary_sha256"]
with gzip.open(here / "stacks.stdout.gz", "rt") as src:
    blocks = src.read().strip().split("\n\n")
symbols = collections.Counter()
leaf = collections.Counter()
depths = collections.Counter()
groups = {
    "polynomial_normal_form": "pi_laurent_polynomial",
    "exact_sign": "::exact_sign",
    "quadratic_projection": "exact_quadratic_surd_parts",
    "closed_form_cache_read": "EvaluationCache>::pi_laurent_polynomial",
    "structural_facts": "::structural_facts",
    "orientation_predicate": "hyperlimit::predicates::orient::orient2",
    "native_contour_winding": "hypercurve::contour::contour_winding_number",
    "native_minor_arc_winding": "process_minor_arc_winding",
    "retained_boundary_side_ray": "loop_windings_from_boundary_side_ray",
    "conic_line_relation": "relation_to_line_with",
    "regularized_fragment_classification": "regularized_fragment_geometric_decision",
    "regularized_fragment_actions": "regularized_fragment_actions",
    "board_fixture": "kicad_board_completes_smoke_check_suite",
}
group_weights = collections.Counter()
normal_form_callers = collections.Counter()
total = empty = unknown_leaf = 0
times = []
for block in blocks:
    lines = block.splitlines()
    match = re.match(r"^\S+\s+\d+/\d+\s+([0-9.]+):\s+(\d+) cycles:u:", lines[0])
    assert match, lines[0]
    times.append(float(match[1]))
    period = int(match[2])
    total += period
    frames = []
    for line in lines[1:]:
        frame = re.match(r"\s+[a-f0-9]+ (.*?) \((.*)\)$", line)
        assert frame, line
        frames.append(frame[1])
    depths[len(frames)] += 1
    if frames:
        leaf[frames[0]] += period
    else:
        empty += period
    if not frames or frames[0] == "[unknown]":
        unknown_leaf += period
    for symbol in set(frames):
        symbols[symbol] += period
    for name, needle in groups.items():
        if any(needle in symbol for symbol in frames):
            group_weights[name] += period
    normal_form_frames = [i for i, symbol in enumerate(frames) if "pi_laurent_polynomial" in symbol]
    if normal_form_frames:
        for symbol in frames[max(normal_form_frames) + 1:]:
            if any(crate in symbol for crate in ("hypercurve::", "hyperlimit::", "hypersolve::")):
                normal_form_callers[symbol] += period
                break
report = gzip.decompress((here / "self-cost.stdout.gz").read_bytes()).decode()
assert int(re.search(r"Event count \(approx\.\): (\d+)", report)[1]) == total
lost = int(re.search(r"Total Lost Samples: (\d+)", report)[1])
observations = dict(
    scope="Diagnostic profile of a separate frame-pointer/line-table binary built from the frozen candidate. Percentages sum recorded userspace cycle periods, not wall time or call counts. Symbols are counted once per sample; inclusive categories overlap. Truncated chains can undercount ancestors.",
    samples=len(blocks), total_sample_period=total, lost_samples=lost,
    first_sample_time=min(times), last_sample_time=max(times),
    empty_stack_period_percent=100 * empty / total,
    unknown_leaf_period_percent=100 * unknown_leaf / total,
    depth_sample_counts=dict(sorted(depths.items())),
    groups={name: dict(symbol_substring=needle, period=group_weights[name], inclusive_percent=100 * group_weights[name] / total) for name, needle in groups.items()},
    nearest_recovered_non_scalar_normal_form_callers=[dict(symbol=s, period=n, total_profile_percent=100 * n / total) for s, n in normal_form_callers.most_common()],
    inclusive_symbols=[dict(symbol=s, period=n, inclusive_percent=100 * n / total) for s, n in symbols.most_common()],
    leaf_symbols=[dict(symbol=s, period=n, self_percent=100 * n / total) for s, n in leaf.most_common()],
    admission_limit="This build changes code generation for profiling. Its runtime is not release timing, a matched speedup, or evidence that the original 120-second admission gate passes.",
)
(here / "observations.json").write_text(json.dumps(observations, indent=2) + "\n")
print(len(blocks), "samples;", lost, "lost; empty weight", observations["empty_stack_period_percent"])
for name, row in observations["groups"].items():
    print(name, round(row["inclusive_percent"], 3))
for row in observations["inclusive_symbols"][:35]:
    print(round(row["inclusive_percent"], 3), row["symbol"][:250])
