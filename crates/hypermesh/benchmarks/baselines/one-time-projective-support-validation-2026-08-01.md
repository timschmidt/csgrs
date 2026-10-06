# One-time projective support validation — 2026-08-01

This is Phase 7 checkpoint 26 of the workspace Hypermesh path-completeness
plan. The retained implementation is Hypermesh
`0a6607569c611cae0a973259541047720c2ad30b`, based on checkpoint 25
`d7dd9e492da6a4eeb8725d478cc7e7bf7e743a4e`. The scalar base remains
Hyperreal `a90fd36aca8df4aab4661c068f2b29961d657da2`.

## Outcome

The compact certified-convex projective input builder now validates each new
support plane once instead of reevaluating the same immutable plane-validity
predicate for every coplanar subdivision triangle. Exact axis-support reuse,
exact adjacent-support reuse, and inversion of an already validated support
preserve a non-zero normal. Newly supplied supports, newly constructed axis
supports, and new `Plane::from_points` supports still pass through the selected
operation `DecisionContext` before admission.

This removes 1.066% of generated-projective retired instructions and 9.766%
of dense-box instructions when both policies are paired. Task clock improves
0.40% and 7.01%, respectively. The retained-arrangement control cannot enter
the compact input path and remains neutral (-0.023% instructions and -0.026%
clock). Peak heap, allocation counts, output topology, call-graph size, and
public API are unchanged. Linked size is effectively neutral: canonical
native and optimized-WASM consumers move between -144 and +72 text/file bytes.

## Exactness and policy invariant

`Plane::decide_is_valid` proves that the support normal is non-zero. For every
support selected by the compact builder, one of these exhaustive cases holds:

1. a supplied support is stored uniquely and validated immediately;
2. a newly constructed exact axis support is validated immediately;
3. a newly constructed point support is validated immediately;
4. an existing exact axis support is immutable and was validated when stored;
5. an existing adjacent coplanar support is immutable and was validated when
   stored; or
6. an adjacent support is inverted, which negates its coefficients and
   algebraically preserves the already established non-zero-normal fact.

The change does not infer triangle validity from a binary64 hint and does not
weaken any topology predicate. Approximate positions remain scheduling hints;
support construction and reuse remain exact. The compact path is still
reachable only for exactly two inputs whose convexity facts have already been
certified, and it remains excluded when retained polygons replace source
triangles.

Under `STRICT`, an unresolved predicate remains a typed indeterminate result.
Under `APPROXIMATE_512`, only Hyperlimit's terminal 512-bit equality/sign
interpretation can resolve an otherwise unresolved predicate. The first
validation of every independently constructed support uses that immutable
policy context, so any authorized terminal consumption is still recorded.
Reuse adds no terminal, equality rule, cache, or policy branch.

All existing retryable compact failures still rebuild the ordinary full
polygon soup and continue through the previous full projective candidate and
complete general subdivision path. Non-retryable index, shape, arithmetic,
and degeneracy failures propagate unchanged. This is an internal execution
change, not a compatibility shim.

## Exact output gates

Both policies retain the checkpoint-25 certified output rows:

| Fixture | Input triangles | Output vertices | Output triangles |
| --- | ---: | ---: | ---: |
| Generated projective | 13,452 | 154 | 304 |
| Retained arrangement | 4,524 | 625 | 1,246 |
| Dense subdivided boxes | 6,144 | 27 | 50 |

The opt-in YeahRight gate passes union, intersection, difference, and symmetric
difference under both policies. Policy-paired meshes are exactly equal and
each result passes exact directed closure, exact nondegeneracy, and
polygon/immediate API agreement.

## Serialized CPU work

Checkpoint-25 and candidate repeated-operation executables were pinned to
logical CPU 9 in parent/candidate/candidate/parent order. Each process builds
the fixture once and repeats a complete immediate union. Retired instructions
are the primary deterministic retention gate; task clock and cycles are
reported as corroborating measurements.

| Fixture / policy | Repetitions | Task movement | Cycle movement | Instruction movement | Branch movement |
| --- | ---: | ---: | ---: | ---: | ---: |
| Generated / `STRICT` | 501 | +0.606% | +0.546% | -1.068% | -1.430% |
| Generated / `APPROXIMATE_512` | 501 | -1.410% | -1.321% | -1.065% | -1.427% |
| Dense boxes / `STRICT` | 10,001 | -7.916% | -7.920% | -9.769% | -12.833% |
| Dense boxes / `APPROXIMATE_512` | 10,001 | -6.107% | -6.107% | -9.764% | -12.827% |
| Retained / `STRICT` | 51 | +0.171% | +0.127% | -0.021% | -0.019% |
| Retained / `APPROXIMATE_512` | 51 | -0.224% | -0.299% | -0.025% | -0.025% |

The policy-paired movements are -0.402% task clock and -1.066% instructions
for generated projective input, -7.011% and -9.766% for dense boxes, and
-0.026% and -0.023% for the retained control. The strict generated clock row
is ordinary run-to-run noise: its deterministic instructions and branches
fall by the same amount as the approximate row, while the paired clock still
improves.

Raw task-clock and retired-instruction brackets are retained in the companion
TOML file. No clock claim depends on a single process.

## Large-fixture heap

Heaptrack covers fixture construction plus one complete immediate union. Both
policies give identical allocation and peak-heap rows, and every row is
exactly unchanged from checkpoint 25.

| Fixture | Allocations | Reconstructed temporaries | Peak heap |
| --- | ---: | ---: | ---: |
| Generated projective | 200,756 | 10,359 | 7.50 MiB |
| Dense boxes | 27,212 | 81 | 2.34 MiB |
| Retained arrangement | 454,005 | 28,735 | 11.67 MiB |

The six recordings are
`/tmp/hypermesh-support-once-{generated,boxes,retained}-{strict,approx}.gz.zst`.
RSS varied only within process-layout noise: 18.17–18.30 MiB generated,
11.22–11.23 MiB boxes, and 21.50–21.53 MiB retained.

## Linked code and call graph

The implementation changes 14 production lines and deletes one. It adds no
public API, compatibility shim, function, or duplicated policy path.

| Consumer | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| General release native text | 4,065,132 | 4,065,204 | +72 |
| Immediate release native text | 4,098,796 | 4,098,868 | +72 |
| General release WASM `wasm-opt -Oz` | 2,727,644 | 2,727,577 | -67 |
| Immediate release WASM `wasm-opt -Oz` | 2,742,692 | 2,742,625 | -67 |
| General size native text | 1,870,978 | 1,870,850 | -128 |
| Immediate size native text | 1,883,454 | 1,883,310 | -144 |
| General size WASM `wasm-opt -Oz` | 1,165,790 | 1,165,788 | -2 |
| Immediate size WASM `wasm-opt -Oz` | 1,176,169 | 1,176,167 | -2 |

The equal-length repeated executable shrinks 2,272 file bytes and 1,488 text
bytes. Its aggregate text/data/BSS grows 16 bytes because BSS layout moves
from 883 to 2,363 bytes; this does not represent allocated runtime storage.

The final call graph remains exactly 8,028 nodes / 19,820 edges for Hypermesh
and 19,718 / 39,444 across Hyperreal, Hyperlattice, Hyperlimit, Hypertri, and
Hypermesh. The change introduces no function node or call edge.

## Cycle profile

The final CPU-9 frame-pointer profile covers 501 strict generated unions,
9,190 samples, zero lost samples, and approximately 19.285 billion cycles.
Largest self owners are four-by-two signed-product summation 5.63%, compact
input construction 4.90%, six-by-two summation 4.53%, lossy rational export
4.52%, crossing-event splitting 3.97%, mixed-width GCD 3.15%, word GCD 2.85%,
allocator work 2.78%, exact normalization 2.48%, exact-rational coordinate
classification 2.38%, compact projective preparation 2.13%, rational filtering
1.98%, point evidence 1.87%, line sign 1.78%, the shared projective core 1.62%,
and `memmove` 1.57%.

Compact input construction sampled at 5.67% in checkpoint 25 and 4.90% here.
Sampling attribution varies, so the paired instruction counts are the
authoritative improvement measure.

## Competitive and historical controls

One CPU-9 Criterion session reports equivalent throughput workloads:

| Control | Hypermesh | Boolmesh | Manifold-rust |
| --- | ---: | ---: | ---: |
| Projective generated union | 6.1780–6.3906 ms (6.2400 center) | 740.00–745.81 us (742.27) | 654.50–660.11 us (656.40) |
| Dense exact-cell boxes | 732.02–738.51 us (735.02 center) | 6.7536–6.7978 ms (6.7775) | 4.2717–4.3181 ms (4.2870) |

The generated Criterion change is statistically neutral (+1.373%, p=0.31)
and its absolute center is slightly below checkpoint 25. Hypermesh remains
about 8.41x slower than Boolmesh and 9.51x slower than Manifold on that row.
The dense-box center improves 13.004%; Hypermesh is about 9.22x faster than
Boolmesh and 5.83x faster than Manifold. Competitors remain throughput
comparators, not exactness oracles.

The retained historical baseline remains 944.8 ms, 67.74 MiB, and 5,020,891
allocations. Current direct work is about 34.77 ms, 11.67 MiB, and 454,005
allocations: approximately 96.32%, 82.77%, and 90.96% below those directional
historical values. Fixture and implementation evolution make that a trend,
not a direct A/B.

## Validation

The final implementation passes:

- default, no-default, and all-feature Hypermesh test matrices;
- 1,059/1,059 default Hypermesh library tests under nightly AddressSanitizer;
- warning-denied all/no-default Clippy and rustdoc;
- all Hypermesh fuzz-bin checks and all-feature benchmark compilation;
- opt-in release exactness for every Boolean operation and both policies,
  polygon/immediate agreement, 3,360/13,440-triangle stress, and the complete
  11,894-triangle input-validation fixture;
- all-family dispatch tracing with zero unknown-fact, fallback, or abort events
  for the generated compact row;
- all six large-fixture Heaptrack recordings, serialized CPU counters,
  native/WASM consumer size controls, competitive Criterion controls, final
  frame-pointer profile, and the five-crate call graph; and
- formatting and diff checks.

The temporary repetition hook used to amortize process startup was removed
before final source validation. The approximately 56-minute full-resolution
rotated Boolean was not rerun: it enters the unchanged ordinary
non-certified-input path, so this certified-convex admission optimization
cannot execute for it.
