# Policy contract performance evidence

## Grammar cache policy

This directory records the focused constructor probe for the explicit
`solid::cache` retention policy. The baseline is revision
`0c5508b89`; the candidate extracts the five grammar caches into typed storage
with caller-visible entry limits, usage, clearing, and scoped policies.

The measured workload calls `sphere(10, 24, 12)` repeatedly with the default
policy. Each process includes one cold miss and otherwise exercises warm
primitive retention. It is not a cold-construction, allocation, Boolean, or
competitive-library benchmark. The probe checks the expected 528 triangles
per call. Exact coordinate/index equality, copy-on-write isolation, context
and certainty preservation, eviction, disabled retention, nested scopes, and
thread isolation are checked separately in
[`tests/solid_cache.rs`](../../../tests/solid_cache.rs).

| Run | Baseline median | Candidate median | Interpretation |
| --- | ---: | ---: | --- |
| `cache-constructor` | 36.202 ns/call | 36.293 ns/call | Final inline helper, same CPU; 0.25% difference, no speedup claim |
| `cache-constructor-factored` | 36.546 ns/call | 38.585 ns/call | Earlier helper without explicit inlining, unpinned exploratory run; not the retained variant |

Each CSV retains 15 alternating baseline/candidate pairs after two warmup
pairs, with one million calls per process. Its matching JSON records settings,
toolchain, host, executable size and SHA-256, and medians. The final run pins
both children to CPU 0. The exploratory run used a prior script without
affinity pinning; its difference cannot be attributed solely to inlining.
Executable sizes are whole benchmark artifacts, not library-size measurements.

Build and save the baseline benchmark executable before changing sources:

```sh
cargo bench --locked --no-default-features --bench mesh_pipeline --no-run
# Copy the executable printed by Cargo to a separate baseline path.
```

Build the candidate with the same command, save its executable separately,
then run from the repository root:

```sh
python3 scripts/compare-cache-probe.py /path/to/baseline /path/to/candidate \
  benchmarks/baselines/2026-10-08-properties/cache-constructor.csv
cargo test --locked --all-features --test solid_cache
cargo test --locked --no-default-features --test solid_cache
```

The retained binary was built after the cache changes and before the later
Hypercurve preview-policy correction. That correction does not participate in
this no-default-features sphere-constructor path. No broader performance or
memory conclusion follows from this probe.

## Curve work-budget accounting

[`curve-budget/manifest.json`](curve-budget/manifest.json) retains 11 alternating
baseline/candidate process pairs after two warmup pairs, pinned to CPU 0.
The baseline executable was built at `e225e6f69`, before budget instrumentation,
with `cargo bench --locked -p hypercurve --bench bezier_algebraic_parameter
--no-run`. The candidate uses the same build command and includes the shared
work counters with inlined accounting accessors. Exact executable hashes,
whole-benchmark sizes, raw stdout, per-sample timings, and every reported
checksum/count are retained. Both binaries report identical evidence on all
11 workloads, including the original quintic and deep boundary-root fixtures.

| Workload | Baseline median | Candidate median | Change |
| --- | ---: | ---: | ---: |
| Simple-root refined ordering | 16.212 µs | 16.390 µs | +1.10% |
| Repeated-root refined ordering | 64.632 µs | 66.092 µs | +2.26% |
| Five-root quintic isolation | 34.398 µs | 34.980 µs | +1.69% |
| Deep boundary-root isolation | 47.423 µs | 48.805 µs | +2.91% |

The final medians range from -3.74% to +2.91% across these workloads. This is
evidence of the cost of adding the explicit contract, not a speedup claim or
a statistical guarantee of a regression ceiling. The retained variant adds
7,048 bytes (0.060%) to this benchmark executable; that is not a measurement
of library binary size or allocations. Active-budget throughput and the full
competitive suite are not measured here. Exact curve results and retry
behavior are checked separately in
[`work_budget.rs`](../../../crates/hypercurve/tests/work_budget.rs).

The preceding variant without the inline attributes is retained in
[`curve-budget-initial/manifest.json`](curve-budget-initial/manifest.json).
Its deep boundary-root median was 5.28% slower than its paired baseline.
The inlined variant was retained to reduce the observed overhead. Both runs
use the same baseline binary and verify the same evidence; run-to-run timing
variation means the difference cannot be attributed wholly to inlining.

To reproduce, save each revision's executable separately and run:

```sh
python3 scripts/compare-curve-budget-probe.py /path/to/baseline /path/to/candidate \
  /path/to/new-evidence-directory
```
