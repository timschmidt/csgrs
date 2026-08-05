# Phase 17 checkpoint: shared demand-materialized exact source planes

Date: 2026-08-05

Status: accepted checkpoint; Phases 17 and 18 remain open

Hypermesh implementation: `7bdff130e8a1e137974a16fb59b35415e581fa64`

Measured parent: `7d836392643a2a786e57db2aa06de9c0cd2e955c`

This checkpoint removes one complete owned support/edge-plane set from every
source `ConvexPolygon`. One operation-local plane owner is shared by all faces
of a source mesh. Native source planes that already satisfy the retained
lossless binary32/binary64 contract reconstruct eagerly; general
`hyperreal::Real` source planes retain only independent empty support/edge
cells and construct exact planes when the complete candidate schedule first
reaches that face.

This is a representation and proof-scheduling rule. It has no fixture,
coordinate, triangle-count, operation, expected-result, policy-name, or
competitor branch. It introduces no compatibility surface and no alternate
Boolean engine.

## Representation and exactness

`ConvexPolygon` now owns one private plane carrier:

- an `Arc` containing support plus edge planes for standalone and derived
  polygons; or
- an `Arc<RetainedSourcePlanes>` plus checked `u32` face ID for a source
  triangle.

The source-plane owner does not repeat source positions or source triangle
rows. Exact positions and the checked three-index face descriptor remain in
the existing retained vertex cycle. A closure exposes those points only when
an empty `OnceLock` needs construction. Once both plane cells are populated,
the exact retained vertex cycle may be released without invalidating future
projective reconstruction.

The layout changes are:

| Carrier | Parent | Current |
| --- | ---: | ---: |
| `ConvexPolygon` | 312 bytes | 128 bytes |
| `PolygonPlanes` | — | 16 bytes |
| lazy source-face plane cells | — | 32 bytes |
| retained source vertex cycle | 24 bytes | 24 bytes |
| retained source extrema | 2 bytes | 2 bytes |

The 58.97% polygon-header reduction improves contiguous polygon storage. The
two independent cells are deliberate: a support-only consumer does not pay for
three edge planes, while an edge consumer initializes and shares the one
support plane it necessarily uses.

For a validated cyclic source triangle with
`n = (b - a) × (c - a)`, each edge normal is `(b - a) × n`, and
`((b - a) × n) · (c - a) = -(n · n)`. Therefore the opposite vertex is
strictly on the negative side for every nondegenerate source face. The source
constructor uses this exact algebraic identity instead of repeating an
orientation predicate. The general polygon constructor keeps the full
policy-aware predicate because it cannot assume source-triangle provenance.
Positive projective normalization preserves the identity.

Source validation still checks every vertex index and compact-ID conversion.
When a certified closed-PWN fact is unavailable, it also executes the central
policy-aware nondegeneracy predicate. Malformed compact/source row counts and
missing source-plane rows return typed errors. Tests cover empty-to-support-to-
edge materialization, native eager materialization, retained-vertex release,
the source orientation identity under both policies, and the compact carrier
sizes.

`STRICT` receives no approximate terminal from this change. Exact construction
does not make an equality decision, and all classifications remain owned by
the existing `DecisionContext`/Hyperlimit cascades. `APPROXIMATE_512` can still
terminate only in Hyperlimit and contributes to aggregate mesh certainty. All
large exact rows below remain `Certified` under both policies.

## Broad performance controls

The parent executable is the immediately preceding retained-source-bound
checkpoint. Counts are three CPU-11-pinned `perf stat` repetitions. All six
controls retire fewer instructions and branches:

| Fixture / workload | Parent instructions | Current instructions | Change | Branch change |
| --- | ---: | ---: | ---: | ---: |
| overlapping boxes, all four ×1000 | 4,395,187,899 | 4,355,684,489 | -0.899% | -1.073% |
| sparse multishell 512, all four ×5 | 3,223,622,434 | 3,185,472,797 | -1.183% | -1.232% |
| dense coplanar 32, all four ×1 | 10,511,779,980 | 10,485,189,178 | -0.253% | -0.056% |
| clipped voxel torus 33, all four ×3 | 2,833,311,667 | 2,806,713,518 | -0.939% | -0.905% |
| 2,049-bit rational boxes, union ×5 | 13,830,028,302 | 12,731,207,958 | -7.945% | -6.863% |
| full rotated YeahRight, intersection ×1 | 12,891,016,963 | 11,415,839,882 | -11.444% | -12.693% |

The general symbolic and wide-rational wins come from not constructing plane
expressions for faces that exact spatial and support facts never reach. Native
primitive workloads avoid the atomic lazy probe and benefit from the smaller
polygon carrier. No workload selects either route by name or size.

## Large-fixture heap matrix

All fifteen selectors ran under both `STRICT` and `APPROXIMATE_512`. Every
policy pair was byte-identical and `Certified`. Total process peak is lower in
fourteen rows and differs by only 64 bytes (0.0005%) in the remaining general
box row. The kernel-only boundary includes demand-created source planes that
the parent constructed before the boundary, so its increases on several rows
are an ownership-timing shift; the total peak is the comparable user-visible
quantity.

| Selector | Parent peak | Current peak | Peak change | Parent/current kernel peak |
| --- | ---: | ---: | ---: | ---: |
| boxes-3072 | 12,580,432 | 12,383,968 | -196,464 | 11,530,906 / 11,334,226 |
| boxes-3072-general | 12,630,032 | 12,630,096 | +64 | 12,033,434 / 12,033,530 |
| dense-coplanar-16 | 15,424,213 | 15,227,749 | -196,464 | 14,394,248 / 14,197,736 |
| dense-coplanar-32 | 61,449,461 | 60,663,173 | -786,288 | 57,423,408 / 56,637,072 |
| sparse-shells-512 | 12,658,579 | 12,527,651 | -130,928 | 11,696,422 / 11,565,446 |
| self-pwn-clusters-512 | 12,788,737 | 12,657,681 | -131,056 | 11,824,368 / 11,693,264 |
| wide-rational-64 | 17,861,502 | 17,861,062 | -440 | 17,243,378 / 17,256,946 |
| wide-rational-512 | 18,750,759 | 18,750,079 | -680 | 18,123,426 / 18,141,314 |
| wide-rational-2048 | 27,425,592 | 27,423,760 | -1,832 | 26,761,890 / 26,800,514 |
| voxel-torus-33 | 12,989,224 | 12,784,184 | -205,040 | 11,921,366 / 11,716,278 |
| voxel-torus-65 | 52,587,884 | 51,784,828 | -803,056 | 47,769,314 / 46,966,210 |
| YeahRight | 5,371,937 | 5,371,657 | -280 | 4,454,684 / 5,100,524 |
| YeahRight ×4 | 20,120,341 | 20,120,061 | -280 | 16,481,486 / 19,062,686 |
| YeahRight ×8 | 76,261,237 | 76,258,109 | -3,128 | 61,735,982 / 72,055,774 |
| full rotated YeahRight | 148,606,806 | 84,482,432 | -64,124,374 (-43.15%) | 141,483,412 / 77,359,070 |

The full row also removes 2,756,526 allocation calls, 548,872 reallocations,
and 122,692,904 bytes of cumulative allocation payload. Some smaller symbolic
rows make more small lazy-cache allocations even though their total live peaks
do not increase materially; those costs are retained in the ledger because
performance counters improve and the full real-world memory result is large.

## Heaptrack ownership on the full fixture

The fresh strict capture is
`target/phase17-lazy-shared-source-planes-full-strict.zst`. Its Heaptrack peak
is 84,556,160 bytes versus 148,680,534 bytes for the parent. Both exceed the
allocator-probe values above by the same 73,728-byte runtime allocation.

Demand-created planes are allocated while arrangement code is on the stack but
remain owned by the source soup. Reattributing those allocations gives the
comparable ownership picture:

| Live owner at peak | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| exact source soup/cache | 104,936,648 | 31,298,480 | -73,638,168 (-70.17%) |
| arrangement excluding source cache | 36,546,764 | 46,060,590 | +9,513,826 (+26.03%) |
| input/runtime outside those owners | about 7,197,122 | about 7,197,090 | effectively flat |
| total | 148,680,534 | 84,556,160 | -64,124,374 (-43.13%) |

The current source owner consists of 4,000,792 bytes allocated directly by
polygon-soup construction, 20,552,832 bytes of later-created support planes,
and 6,744,856 bytes of later-created edge cycles. Only 11,659 of 23,788 source
supports (49.01%) and 1,784 edge cycles (7.50%) materialize. Thus 12,129 source
faces never allocate any exact plane, and only 7.50% need the complete four-
plane carrier.

Raw nested current call-stack totals are 73,358,278 bytes below
`build_surface_arrangement`, 47,414,534 below `corefine_surface`, and
24,982,804 below pairwise intersection. They are not additive stage owners and
include the lazy source cache above. The exclusive arrangement increase is
reported rather than assigned to the source win; it makes arrangement/corefine
storage the next measured memory target.

## Historical and CGAL boundary

Fresh CPU-11-pinned full processes return the same certified exact-empty
result:

| Policy | Wall time | Maximum RSS |
| --- | ---: | ---: |
| `STRICT` | 1.12 s | 98,492 KiB |
| `APPROXIMATE_512` | 1.11 s | 98,316 KiB |

The strict row is 16.4% faster and uses 43.1% less RSS than the immediate
checkpoint's advisory 1.34 s / 173,040 KiB row. Against historical EMBER it is
about 2,958× faster and uses 70.1% less RSS. Historical CGAL EPECK remains far
ahead at 0.09 s / 15,516 KiB: current Hypermesh is still about 12.44× slower
and 6.35× larger on that pinned full-fixture boundary.

CGAL 6.0.3 EPECK was refreshed for 63 repetitions in each exact-OFF copy mode.
Hypermesh used 63 fresh CPU-pinned processes through `perf stat -r 63`, which
removed a frequency-bimodal orchestration artifact observed in an initial
sample. Every CGAL output is valid, closed, structurally valid, and matches
Hypermesh's topology and exact-volume oracle.

| Fixture | CGAL outside / inside | Hypermesh `STRICT` | Ratio | Hypermesh `APPROXIMATE_512` | Ratio |
| --- | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 112,972 / 122,712 ns | 471,827 ns | 4.176× | 474,327 ns | 4.199× |
| affine boxes | 376,124 / 383,553 ns | 873,540 ns | 2.322× | 869,970 ns | 2.313× |

Hypermesh itself improves slightly from the parent crossing/affine rows; the
crossing ratio grows because this fresh CGAL run is faster. Both per-case gaps
remain open. No small-case or competitor-specific path was introduced.

## Code and binary size

The restructuring adds 705 and removes 370 lines across five production/test
files. Much of the apparent growth is explicit carrier invariants and tests;
the normal release consumer shrinks:

| Feature/profile | Native text movement | Optimized WASM movement |
| --- | ---: | ---: |
| default release, general | -7,208 bytes (-0.361%) | -12,022 bytes (-0.849%) |
| default release, immediate | -7,208 bytes (-0.361%) | -12,033 bytes (-0.848%) |
| default size, general | +1,152 bytes (+0.106%) | +653 bytes (+0.096%) |
| default size, immediate | +1,168 bytes (+0.108%) | +655 bytes (+0.096%) |
| all-feature release, general | -8,124 bytes (-0.381%) | -13,939 bytes (-0.934%) |
| all-feature release, immediate | -8,124 bytes (-0.381%) | -13,957 bytes (-0.934%) |
| all-feature size, general | +936 bytes (+0.086%) | +404 bytes (+0.060%) |
| all-feature size, immediate | +920 bytes (+0.084%) | +405 bytes (+0.060%) |

The small size-profile growth is accepted under the performance-first
priority; it is not treated as a size win.

## Call graph and validation

The refreshed five-crate graphs contain 15,090 nodes / 25,152 edges for
production, 17,410 / 28,468 with tests and examples, and 21,456 / 34,569 with
all tests, examples, benches, and fuzz targets. There are zero production
EMBER, local-BSP, or segment-trace nodes. The source-plane construction carrier
has no direct edge to Hyperlimit because it constructs exact values and makes
no decision; all later classifications continue through the canonical policy
cascade. Hypercurve and HyperSolve are excluded from the audit and untouched.

Validation passed:

```sh
cargo fmt --all -- --check
cargo test --locked --all-features
cargo test --locked --lib --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-Dwarnings' cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo check --locked --benches --all-features
cargo check --locked --examples --all-features
benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```

The all-feature suite passes 195 tests with the six documented opt-in/manual
YeahRight tests ignored; the minimal library suite passes 147 tests. The exact
large-mesh heap matrix, performance controls, CGAL rows, and all three call
graphs also pass.

Phase 17 and Phase 18 remain open. The next memory work moves to the measured
surface-arrangement/corefinement owner, while competitive work continues from
the explicit per-case CGAL deficits. Any optimization must remain a clean
general exact algorithm, preserve every complete fallback, and exploit retained
Hyperreal facts rather than recognize corpus or benchmark identities.
