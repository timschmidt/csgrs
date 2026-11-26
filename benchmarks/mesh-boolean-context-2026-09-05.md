# Mesh Boolean context measurements — 2026-09-05

This historical record reports completion of the sampled-cylinder subtraction
under `GeometryContext::APPROXIMATE_512` and `PredicateUndecided` under STRICT.
It does not establish the current revision's outcomes. The retained benchmark
assertions still require those distinct outcomes; a newly certified result would
fail those historical expectations.

Measured on an AMD Ryzen 7 5800X3D with `rustc 1.97.0 (2d8144b78 2026-07-07)`,
using the optimized benchmark profile and no default features:

```sh
cargo bench --locked --no-default-features --bench mesh_boolean_context
```

Each workload subtracts a radius-2, height-6 cylinder, translated to `(6, 6, -1)`,
from a 12×12×4 body. The 16-segment case has 76 input triangles; the 32-segment
case has 140. Construction is excluded. Each policy starts with fresh mesh
storage and runs five operations on the same inputs, retaining kernel facts
between samples. Scalar expression caches can already be warm.

| Cylinder segments | Policy | Completed | Output triangles | First operation (ms) | Median operation (ms) |
|---:|---|---:|---:|---:|---:|
| 16 | `STRICT` | 0/5 | — | 28.385 | 21.696 |
| 16 | `APPROXIMATE_512` | 5/5 | 144 | 379.730 | 230.502 |
| 32 | `STRICT` | 0/5 | — | 42.630 | 36.321 |
| 32 | `APPROXIMATE_512` | 5/5 | 272 | 728.902 | 816.220 |

Every completion in the recorded run reported `Approximate512Consumed`. Strict
timings measured the time until `PredicateUndecided` was returned. These are local timing
samples using workspace path dependencies, including concurrent development
changes in Hyperreal and Hypersolve.

The validation report for that run described four tests in
`tests/mesh_boolean_context.rs` covering all four Boolean operations under both
policies, certified rational results, empty intersections,
input errors, policy isolation after approximation, and the drilled body's
topology check and expected polygonal-cylinder volume. It also described 15 tests in
`tests/geometry_context.rs` covering context propagation through bounds, transforms,
primitives, triangulation, curves, imports, SDF sampling, and scalar/WASM adapters.
The all-feature suite passed 122 tests with one pre-existing ignored render
regression; the four Boolean tests also passed with no default features.
Formatting, all-feature Clippy for all targets with `-D warnings`, API
documentation generation, and both adversarial scripts passed.

Those counts and pass statements are historical, not checks executed by the
documentation audit. This record does not identify an exact source commit or retain
raw samples here, so it cannot alone reproduce or certify those historical results.
Strict failure latency and approximate completion latency measure different contracts.
