# Hypermesh exact projective fingerprint checkpoint

Date: 2026-07-31

Direct Hypermesh parent: `5fe308f0b6325577ab702eae51bc9bc0838f9844`

Core implementation: `a384d6ec12dc5d105d8e7e1c506acfdd2ec89b83`

Nested caller migration: `3e05079adfa83aa6eb1eeac28dca2e653bf71f56`

## Outcome

The retained 4,524-triangle YeahRight arrangement exposed an all-pairs
projective vertex-coincidence scan as the dominant post-closure cost. The
candidate replaces the quadratic scan for finite exact-rational points with an
exact inequality filter over the prime field modulo `2^61 - 1`.

On the direct-parent fixture, the candidate:

- reduces task clock by 69.78%;
- reduces instructions by 71.91%;
- reduces allocation calls by 82.38%;
- reduces temporary allocations by 57.68%;
- preserves the 12.70 MiB peak heap;
- produces the same 625-vertex, 1,246-triangle certified result under
  `STRICT` and `APPROXIMATE_512`.

This is a filter, not hash-based topology. A different field fingerprint proves
that two finite rational affine points cannot be equal. A matching fingerprint
only creates a candidate and still executes the existing policy-aware exact
equality path. Modular collisions therefore cannot merge distinct points.

## Exactness and path completeness

For a homogeneous rational point `(x, y, z, w)`, the filter reduces each
affine coordinate `x/w`, `y/w`, and `z/w` in the prime field. Rational equality
is preserved by this homomorphism whenever every required denominator and the
reduced weight are invertible. Therefore unequal field values are a sound proof
of rational inequality.

The filter deliberately declines to key a point when:

- any coordinate is not an exact rational;
- the actual homogeneous weight is zero;
- a rational denominator is zero modulo the prime; or
- the weight reduces to zero modulo the prime.

Every pair involving an unkeyed point retains the original all-pairs
policy-aware comparison. Within a matching fingerprint bucket, one
representative is retained for each exactly distinct collision class. No
`STRICT` or `APPROXIMATE_512` terminal is bypassed, and no field collision is
accepted as equality.

Regressions cover scale invariance, distinct affine values, a deliberate
modular collision, actual and modular-zero weights, modular denominators,
symbolic fallback, and collision-class merging under both policies. Both
policy runs remain `MeshCertainty::Certified` on the measured rational
fixtures, so `APPROXIMATE_512` is available but was not consumed.

## Benchmark fixture correction

The live YeahRight generator previously converted independently computed
per-face binary64 subdivision points into exact rationals and then attached an
unchecked certified-convex fact. Exact `try_certify_convex` correctly rejected
that mesh: adjacent nominally coplanar facets were not the same exact planes.
The unchecked fact sent invalid proof evidence into the projective fast path.

The generator now snaps the 107-vertex, 210-triangle convex hull to a `2^-40`
dyadic grid before subdivision. Coordinates are finite and bounded by 512, so
the three additional midpoint bits required by the eight-way stress fixture
remain within binary64's 53-bit significand. The generated mesh is then
explicitly certified through `try_certify_convex`; the old forwarding helper
was removed and every caller migrated.

The opt-in corpus now verifies that:

- the generated hull certifies with `MeshCertainty::Certified`;
- all competitor adapters receive the same fixture;
- every Hypermesh Boolean operation returns a boundaryless exact mesh;
- polygon and immediate triangle APIs agree; and
- the 3,360- and 13,440-hull-triangle stress cases complete.

No compatibility shim or unchecked fallback was added.

## Direct-parent runtime

The A/B uses the same repaired Hyperreal, Hyperlattice, Hyperlimit, and
Hypertri sources on both sides. Each row is the average of eleven serialized
runs pinned to CPU 8. The direct parent uses its then-canonical
`APPROXIMATE_512` probe; the candidate row uses the explicit
`APPROXIMATE_512` argument. Both runs report certified output.

| Counter | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| Task clock | 298.74 ms | 90.27 ms | -69.7831% |
| Cycles | 1,231,409,536 | 353,858,020 | -71.2640% |
| Instructions | 3,814,408,751 | 1,071,384,751 | -71.9122% |
| Branches | 710,945,009 | 181,216,551 | -74.5105% |
| Branch misses | 2,293,051 | 1,435,872 | -37.3816% |
| Cache misses | 3,022,310 | 1,432,070 | -52.6167% |

The candidate `STRICT` control is 90.27 ms and 1,071,399,344 instructions.
The 0.0014% instruction difference from the approximate-policy row is
non-semantic command/output overhead; both operation results are certified and
identical.

An alternative direct `[Rational; 3]` affine key reduced allocation calls by a
further 1,428 but required 1,078,090,000 instructions and 93.31 ms. The modular
filter is retained because runtime is the primary optimization criterion and
the allocation difference is only 0.11% of the candidate total.

## Large-fixture heap

Heaptrack includes fixture construction and the complete immediate operation.
Its displayed `M` units are reported below as MiB consistently with the prior
checkpoint. RSS includes profiler overhead and is informative rather than a
retained-memory gate.

| Fixture and policy | Allocations | Temporary | Peak heap | Heaptrack RSS | Output |
| --- | ---: | ---: | ---: | ---: | --- |
| Retained parent, default approximate | 7,122,814 | 409,198 | 12.70 MiB | 22.65 MiB | 625 v / 1,246 t |
| Retained candidate, `STRICT` | 1,254,723 | 173,192 | 12.70 MiB | 23.03 MiB | 625 v / 1,246 t |
| Retained candidate, `APPROXIMATE_512` | 1,254,723 | 173,192 | 12.70 MiB | 22.52 MiB | 625 v / 1,246 t |
| Boxes, 6,144 t, `STRICT` | 27,214 | 85 | 4.70 MiB | 13.02 MiB | 27 v / 50 t |
| Boxes, 6,144 t, `APPROXIMATE_512` | 27,214 | 85 | 4.70 MiB | 13.09 MiB | 27 v / 50 t |
| Dyadic YeahRight, 13,452 t, `STRICT` | 304,575 | 27,065 | 11.66 MiB | 23.65 MiB | 154 v / 304 t |
| Dyadic YeahRight, 13,452 t, `APPROXIMATE_512` | 304,575 | 27,065 | 11.66 MiB | 23.77 MiB | 154 v / 304 t |

The 6,144-triangle box control improves from 37,678,847 parent instructions to
35,894,595 (-4.73%) and from 27,477 to 27,214 allocations, while preserving
the 4.70 MiB peak heap. Its 6.00 versus 5.83 ms task-clock movement is
favorable but the row is short enough that instructions are the stronger
signal.

The 13,452-triangle generated fixture executes in 80.46 ms and 670,023,595
instructions under `STRICT`, versus 79.86 ms and 669,979,318 instructions
under `APPROXIMATE_512`. Both return `Certified`.

The separate ignored stress test intentionally retains two generated cases and
all competitor adapters at once. It reports 2,499,547 allocations, 284,833
temporary allocations, 81.35 MiB peak heap, 115.16 MiB heaptrack RSS, and
8.512 seconds. This is a combined multi-case/multi-engine retention ceiling,
not the heap cost of one Hypermesh operation.

## Historical and competitive controls

The repaired scalar/predicate stack already improves the direct parent from
the previous checkpoint's 1,770.68 ms and 41,757,532 allocations to 298.74 ms
and 7,122,814 allocations. The fingerprint candidate improves that repaired
parent again to 90.27 ms and 1,254,723 allocations.

The older frozen YeahRight row was 944.8 ms, 67.74 MiB peak heap, 5,020,891
allocations, and 82.5 MiB maximum RSS. That historical row materialized a
different polygon output, so it is directional rather than an A/B oracle. The
current 12.70 MiB retained-fixture heap remains 81.25% below its heap ceiling.

Criterion throughput controls, pinned to CPU 8, report:

| Union workload | Hypermesh | boolmesh | manifold-rust |
| --- | ---: | ---: | ---: |
| Overlapping 12-triangle boxes | 5.0826 us | 65.829 us | 57.834 us |
| 3,072-triangle boxes per operand | 1.8464 ms | 7.4557 ms | 4.3513 ms |
| Dyadic YeahRight 840-triangle hull + box | 13.160 ms | 0.75679 ms | 0.66567 ms |

Hypermesh is 12.95x and 11.38x faster on the small exact-cell row, and 4.04x
and 2.36x faster on the large exact-cell row. On the projective YeahRight row,
boolmesh and manifold-rust are 17.39x and 19.77x faster throughput references.
They do not provide Hypermesh's exact `Real`, policy, or certification
contract, so they are not correctness oracles.

The frozen small-box Hypermesh control is 4.917 us; the current 5.0826 us is a
3.37% linked-layout/dependency movement on a path that never calls the new
fingerprint filter. It remains explicit as a small-path recovery target.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. The filter adds
0.1649–0.3931% across the sixteen canonical rows. This is retained as a
measured performance-for-code-size tradeoff: the hard projective workload
executes 71.91% fewer instructions.

| Features | Consumer | Profile | Parent | Candidate | Movement |
| --- | --- | --- | ---: | ---: | ---: |
| Default | General native | Release | 4,011,916 | 4,019,124 | +0.1797% |
| Default | General WASM | Release | 2,694,255 | 2,698,772 | +0.1677% |
| Default | Immediate native | Release | 4,045,532 | 4,052,740 | +0.1782% |
| Default | Immediate WASM | Release | 2,709,293 | 2,713,870 | +0.1689% |
| Default | General native | Size | 1,838,682 | 1,843,786 | +0.2776% |
| Default | General WASM | Size | 1,139,247 | 1,143,391 | +0.3637% |
| Default | Immediate native | Size | 1,851,166 | 1,856,270 | +0.2757% |
| Default | Immediate WASM | Size | 1,150,212 | 1,154,361 | +0.3607% |
| All | General native | Release | 4,161,441 | 4,168,689 | +0.1742% |
| All | General WASM | Release | 2,777,507 | 2,782,113 | +0.1658% |
| All | Immediate native | Release | 4,195,321 | 4,202,553 | +0.1724% |
| All | Immediate WASM | Release | 2,792,926 | 2,797,532 | +0.1649% |
| All | General native | Size | 1,838,938 | 1,844,090 | +0.2802% |
| All | General WASM | Size | 1,169,182 | 1,173,778 | +0.3931% |
| All | Immediate native | Size | 1,851,350 | 1,856,486 | +0.2774% |
| All | Immediate WASM | Size | 1,180,380 | 1,184,552 | +0.3534% |

The nested size and fuzz lockfiles were stale after Hypermesh acquired its
Hypertri edge. They now contain that exact local package graph, and both pass
again under `--locked`.

## Caller migration

The Trunk UI was the remaining workspace caller of policy-free Hypermesh and
Hypergraphics APIs. It now owns one explicit `MeshContext` using
`APPROXIMATE_512`, calls the immediate `boolean_mesh` API, forwards the same
predicate policy to camera projection, and displays both the selected policy
and the aggregate result certainty. Its tests assert that the rational demo
operations remain certified. No forwarding API or compatibility layer was
introduced.

## Source and call graph

Across the two implementation commits, the checkpoint adds 351 lines and
removes 56, including tests, benchmark probes, UI migration, and three repaired
lockfiles.

Against the isolated direct parent, Hypermesh's source call graph moves from
7,924 nodes and 19,498 edges to 7,950 nodes and 19,555 edges: +26 nodes and
+57 edges. The five-crate Hyperreal/Hyperlattice/Hyperlimit/Hypertri/Hypermesh
graph contains 19,503 nodes and 38,951 edges. The added nodes are primarily the
four modular helpers, conservative fallback edges, and collision regressions.

## Validation

All completed successfully:

- default tests: 1,167 executed, 7 ignored;
- no-default tests: 1,167 executed, 7 ignored;
- all-feature tests: 1,169 executed, 7 ignored;
- all-target warning-denied Clippy with all and no default features;
- warning-denied all-feature rustdoc;
- every fuzz binary under the repaired locked graph;
- all benchmark targets and the dispatch-trace executable;
- native and `wasm32-unknown-unknown` size-profile builds;
- all default/all native/WASM release/size measurements;
- eight native UI tests and `trunk build --release --locked`;
- three opt-in YeahRight API/exactness tests and the two-size stress test;
- formatting and `git diff --check`.

## Reproduction

```sh
cargo test --locked --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo test --locked --all-features --no-fail-fast
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo clippy --locked --all-targets --no-default-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run
cargo bench --locked --bench dispatch_trace --features dispatch-trace

cargo build --locked --manifest-path benchmarks/size-harness/Cargo.toml --profile size
cargo build --locked --manifest-path benchmarks/size-harness/Cargo.toml \
  --profile size --target wasm32-unknown-unknown
./benchmarks/size-harness/measure.sh default
./benchmarks/size-harness/measure.sh all

YEAHRIGHT_BENCH=1 cargo test --locked --release --test competitive \
  yeahright_benchmark_inputs_reach_every_competitor \
  -- --ignored --exact --test-threads=1
YEAHRIGHT_BENCH=1 cargo test --locked --release --test competitive \
  yeahright_exact_hypermesh_outputs_remain_boundaryless_for_every_operation \
  -- --ignored --exact --test-threads=1
YEAHRIGHT_BENCH=1 cargo test --locked --release --test competitive \
  yeahright_polygon_and_triangle_immediate_apis_remain_consistent \
  -- --ignored --exact --test-threads=1
YEAHRIGHT_BENCH=1 cargo test --locked --release --test competitive \
  larger_yeahright_fixtures_expose_memory_pressure \
  -- --ignored --exact --test-threads=1

cargo build --locked --release --example large_mesh_heap_probe
heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe boxes-3072 strict
YEAHRIGHT_BENCH=1 heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe yeahright-8 strict

cd examples/hypermesh_ui
cargo test --locked
NO_COLOR=true trunk build --release --locked
```

The retained fixture is
`competitive/data/yeahright_boolean_hull.obj` from the prior checkpoint, with
SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`.
