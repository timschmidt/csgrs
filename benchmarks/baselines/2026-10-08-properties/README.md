# Grammar cache policy evidence

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
