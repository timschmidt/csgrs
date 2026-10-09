"""Retain weighted leaf samples and disclose failed DWARF caller recovery."""
import collections
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

here = Path(__file__).resolve().parent
manifest = json.loads((here / "manifest.json").read_text())
assert hashlib.sha256(Path(manifest["binary"]).read_bytes()).hexdigest() == manifest["binary_sha256"]
command = ["perf", "script", "-G", "-F", "comm,pid,tid,time,period,event,ip,sym,dso", "-i", str(here / "perf.data")]
result = subprocess.run(command, text=True, capture_output=True, env={**os.environ, "DEBUGINFOD_URLS": ""})
assert result.returncode == 0
with gzip.open(here / "leaves.txt.gz", "wt") as out:
    out.write(result.stdout)
(here / "leaves-stderr.log").write_text(result.stderr)
weights = collections.Counter()
samples = 0
for line in result.stdout.splitlines():
    if not line.strip():
        continue
    match = re.match(r"\s*\S+\s+\d+/\d+\s+[0-9.]+:\s+(\d+) cycles:u:\s+[a-f0-9]+ (.*?) \((.*)\)$", line)
    assert match, line
    samples += 1
    weights[match[2]] += int(match[1])
total = sum(weights.values())
report = (here / "self-cost.log").read_text()
assert int(re.search(r"Event count \(approx\.\): (\d+)", report)[1]) == total
lost = int(re.search(r"Total Lost Samples: (\d+)", report)[1])

with gzip.open(here / "stacks.txt.gz", "rt") as src:
    blocks = src.read().strip().split("\n\n")
empty_samples = empty_weight = stack_total = 0
depths = collections.Counter()
for block in blocks:
    lines = block.splitlines()
    match = re.match(r"^\S+\s+\d+/\d+\s+[0-9.]+:\s+(\d+) cycles:u:", lines[0])
    assert match, lines[0]
    period = int(match[1])
    depth = sum(bool(re.match(r"\s+[a-f0-9]+ .* \(.*\)$", line)) for line in lines[1:])
    depths[depth] += 1
    stack_total += period
    if not depth:
        empty_samples += 1
        empty_weight += period
assert stack_total == total and len(blocks) == samples
allocator_names = {
    "_int_malloc", "malloc", "cfree@GLIBC_2.2.5", "unlink_chunk.isra.0",
    "_int_free_chunk", "malloc_consolidate", "__libc_malloc2", "realloc",
    "__rustc::__rust_alloc", "__rustc::__rust_dealloc", "__rustc::__rust_realloc",
    "__rustc::__rust_no_alloc_shim_is_unstable_v2",
}
groups = {
    "polynomial_and_radical_symbols": [s for s in weights if any(x in s for x in ("PiLaurentPolynomial", "pi_laurent_polynomial", "RadicalCoefficient"))],
    "named_allocator_symbols": sorted(set(weights) & allocator_names),
}
observations = dict(
    scope="Separate sampled diagnostic. Weights are recorded userspace cycle periods, not wall time, call counts, allocations, or a matched speedup.",
    samples=samples, total_sample_period=total, lost_samples=lost,
    leaf_command=command, leaf_command_exit_code=result.returncode,
    leaf_unknown_percent=100 * weights["[unknown]"] / total,
    original_stack_export=dict(empty_samples=empty_samples, empty_period_percent=100 * empty_weight / total, depths=dict(sorted(depths.items()))),
    caller_limitation="The original DWARF export contains no frames for almost all weighted samples. The report's Children and Self columns agree and do not establish inclusive costs. Local replay yields some short/invalid chains; no caller percentages are claimed. A separate frame-pointer build is required.",
    groups={name: dict(symbols=syms, period=sum(weights[s] for s in syms), self_percent=100 * sum(weights[s] for s in syms) / total) for name, syms in groups.items()},
    leaf_symbols=[dict(symbol=s, period=n, self_percent=100 * n / total) for s, n in weights.most_common()],
    next_step="Recover reliable call chains before selecting a geometry-level optimization. Scalar polynomial reconstruction and allocator traffic are the observed leaf costs to investigate.",
)
(here / "observations.json").write_text(json.dumps(observations, indent=2) + "\n")
print(samples, "samples;", lost, "lost; empty stack weight", observations["original_stack_export"]["empty_period_percent"])
for name, group in observations["groups"].items():
    print(name, group["self_percent"])
