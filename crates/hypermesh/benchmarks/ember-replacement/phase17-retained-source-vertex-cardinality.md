# Phase 17 retained source-vertex cardinality

Date: 2026-08-05

Status: accepted checkpoint; Phases 17 and 18 remain open

Implementation: Hypermesh `f01a9fd34eb8701d65534f4d2d210fae2cf59cb4`

Direct parent/evidence: canonical graph-point transfer `c94b4fd3` / `df429775`

## Result

Surface corefinement used to discover its initial arrangement-point capacity by
walking every polygon, cloning each `ConstructionVertexIdentity`, and inserting
the identities into a temporary hash map. It then dropped that map and repeated
the same source identities through the final structural and numeric point
interners.

Source triangle validation already visits every checked dense position index.
It now uses one fallible dense usage bitset per mesh to count the distinct source
vertex IDs referenced by valid triangles. `PolygonSoup` retains that exact
scalar cardinality, the Boolean orchestrator passes it to the arrangement, and
corefinement adds it to the intersection graph's already exact construction
point count when sizing the final point arena. The temporary cloned-identity
hash pass is gone.

This is a general dataflow and ownership correction. It does not inspect a
fixture name, triangle count, coordinate width, operation, topology, output,
competitor, or policy. There is no thresholded production path, compatibility
shim, alternate Boolean engine, or changed topology algorithm. Independent
source indices remain distinct even when their numeric points coincide; unused
positions are correctly excluded; equal local indices in different meshes are
distinct source identities.

Two measured variants were rejected and removed. Reusing one usage vector
across meshes perturbed allocation order enough to regress the shallow-symbolic
retained-fact schedule. An inline 64-bit mask helped that one small row but made
every large/general control slightly worse and introduced an arbitrary size
threshold. The accepted implementation is the single clean dense-index
algorithm for every mesh.

## Exactness, completeness, and policy

- Triangle indices are bounds checked before indexing the usage bitset and
  retain the existing checked `u32` identity-domain conversion.
- The per-mesh count and aggregate count retain typed capacity failures. Arena
  capacity addition remains checked.
- Cardinality is allocation metadata only. It neither proves point equality nor
  changes structural-identity precedence, exact rational equality, general
  equality, triangulation, radial topology, winding, or output certification.
- `STRICT` still has no approximate terminal. `APPROXIMATE_512` can consume its
  terminal only through Hyperlimit's existing predicate/equality cascade, and
  aggregate certainty remains operation-local and observable.
- A focused regression covers an unused source position, coincident values with
  independent source IDs, and two separate mesh identity domains. It remains
  `Certified`.
- Every large-fixture policy pair is byte-identical and `Certified`; the
  depth-128 symbolic control remains `Approximate512Consumed` only where
  intentionally exercised.

## Deterministic retired-work measurements

The current release probe is compared against saved executable
`target/phase17-graph-point-transfer-competitive-probe`. Measurements are
CPU-11-pinned `perf stat` instruction and branch counts; topology summaries,
exact output, and certainty are identical.

| Fixture / workload | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| overlapping boxes, all four ×1000 | 3,945,579,644 | 3,929,346,315 | -0.4114% | 681,607,924 | 679,724,351 | -0.2763% |
| sparse 512 shells, all four ×5 | 2,725,368,948 | 2,705,837,159 | -0.7167% | 490,528,468 | 488,090,503 | -0.4970% |
| dense coplanar 32, all four ×1 | 10,073,784,698 | 10,062,938,392 | -0.1077% | 1,722,489,682 | 1,721,715,754 | -0.0449% |
| clipped voxel torus 33, all four ×3 | 2,645,220,971 | 2,635,280,439 | -0.3758% | 443,944,085 | 443,128,838 | -0.1836% |
| 2,049-bit wide boxes, union ×5 | 11,824,627,010 | 11,803,725,338 | -0.1768% | 2,177,768,433 | 2,175,095,419 | -0.1227% |
| full YeahRight, intersection ×1 | 10,394,701,591 | 10,381,550,482 | -0.1265% | 1,784,342,892 | 1,782,886,931 | -0.0816% |

All six broad controls retire work. Symbolic controls disclose the small
tradeoff rather than hiding it:

| Fixture / policy / workload | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change | Certainty |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| depth 1 / `STRICT` / all four ×20 | 4,285,247,881 | 4,288,171,815 | +0.0682% | 936,210,724 | 936,876,962 | +0.0712% | `Certified` |
| depth 128 / `APPROXIMATE_512` / all four ×5 | 1,505,351,329 | 1,504,946,380 | -0.0269% | 274,782,529 | 274,688,631 | -0.0342% | `Approximate512Consumed` |

The shallow row is a bounded open regression. It is retained because all six
general and large controls improve, the implementation is simpler, and both
rejected scheduling variants were worse across the complete control set.

The public polygon-soup boundary was measured separately so moving the count
into validation could not hide a preprocessing regression. Criterion intervals
move from 10.578–10.627 µs to 10.510–10.538 µs for a cube (estimated -0.485%)
and from 4.9521–4.9896 ms to 4.9301–4.9507 ms for 3,072 subdivided triangles
(estimated -0.590%). Both are small favorable/no-regression observations, not
claimed independent speedups.

## Large-fixture heap matrix

All fifteen selectors ran in fresh processes under both policies. Each policy
pair has the same exact output, certainty, peak, allocation calls, and allocated
bytes. Peaks are byte-identical to the direct parent; the removed preliminary
identity pass reduces allocation calls and cumulative byte traffic.

| Selector | Peak bytes | Parent/current calls | Parent/current cumulative bytes |
| --- | ---: | ---: | ---: |
| boxes-3072 | 12,383,968 | 19,228 / 19,219 | 16,571,814 / 16,337,262 |
| boxes-3072-general | 12,630,096 | 38,414 / 38,405 | 18,901,662 / 18,667,110 |
| dense-coplanar-16 | 14,893,071 | 276,943 / 276,934 | 66,315,962 / 66,081,410 |
| dense-coplanar-32 | 59,391,879 | 1,106,675 / 1,106,664 | 265,121,594 / 264,183,522 |
| sparse-shells-512 | 11,888,929 | 213,486 / 213,476 | 35,125,748 / 34,654,632 |
| self-PWN-clusters-512 | 12,284,773 | 215,480 / 215,470 | 35,721,550 / 35,250,438 |
| wide-rational-64 | 17,861,062 | 384,612 / 384,603 | 32,609,070 / 32,374,518 |
| wide-rational-512 | 18,750,079 | 1,295,426 / 1,295,417 | 104,896,158 / 104,661,606 |
| wide-rational-2048 | 27,423,760 | 1,567,490 / 1,567,481 | 385,301,742 / 385,067,190 |
| voxel-torus-33 | 12,784,184 | 28,309 / 28,300 | 23,174,148 / 22,939,728 |
| voxel-torus-65 | 51,784,828 | 102,730 / 102,719 | 105,626,263 / 104,688,451 |
| yeahright | 5,161,051 | 167,363 / 167,357 | 11,458,693 / 11,429,415 |
| yeahright-4 | 19,676,989 | 728,414 / 728,406 | 45,783,466 / 45,666,328 |
| yeahright-8 | 76,258,109 | 3,315,752 / 3,315,742 | 188,362,156 / 187,893,674 |
| yeahright-full-rotated | 59,440,454 | 9,926,708 / 9,926,697 | 588,769,229 / 587,830,239 |

The full row retains a 52,317,092-byte incremental Boolean-kernel peak and
1,610,293 reallocations. Its Heaptrack capture is
`target/phase17-source-vertex-cardinality-full-strict.zst.zst`; peak stacks are
`/tmp/hypermesh-source-vertex-cardinality-peak.stacks`. The exact stack sum is
59,514,182 bytes: 59,440,454 allocator bytes plus 73,728 profiler/process bytes,
identical to the parent. Final Heaptrack analysis reports 11,645,442 allocation
function calls, eleven fewer than the parent, and 2,669,073 temporary
allocations. The recorder's live summary used a different temporary-allocation
classification; the final `heaptrack_print` analysis is the recorded value.

## Historical and competitive boundaries

Fresh full processes preserve exact certified-empty output:

| Policy | Wall time | Maximum RSS |
| --- | ---: | ---: |
| `STRICT` | 1.02 s | 69,340 KiB |
| `APPROXIMATE_512` | 1.01 s | 69,192 KiB |

These one-process clocks and RSS maxima are noisy and do not override the stable
retired-work and requested-heap results. Against historical EMBER's 3,312.66 s
/ 329,352 KiB, current strict is about 3,248× faster and uses 78.95% less RSS.
Historical CGAL's 0.09 s / 15,516 KiB remains about 11.33× faster and uses
4.47× less RSS.

Pinned CGAL 6.0.3 EPECK exact-OFF values are unchanged. Hypermesh values are
medians of 63 fresh CPU-11-pinned processes per policy. Same-time parent
reruns show clock noise around the small deterministic improvement, so no wall
speedup is claimed:

| Fixture | CGAL outside / inside | Parent `STRICT` | Current `STRICT` | Current/CGAL | Parent `APPROXIMATE_512` | Current `APPROXIMATE_512` | Current/CGAL |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 112,972 / 122,712 ns | 416,941 ns | 425,591 ns | 3.767× | 417,111 ns | 414,841 ns | 3.672× |
| affine boxes | 376,124 / 383,553 ns | 803,865 ns | 806,035 ns | 2.143× | 807,984 ns | 811,834 ns | 2.159× |

Every CGAL gap remains open. No fixture was special-cased and no favorable row
is substituted for a losing case.

## Code, binary size, and call graph

The implementation changes three files by 106 insertions and 45 deletions,
including focused test helpers. Every one of the sixteen canonical consumer
artifacts shrinks:

| Configuration | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| default release native general text | 1,986,058 | 1,982,922 | -3,136 (-0.1579%) |
| default release native immediate text | 1,989,202 | 1,986,066 | -3,136 (-0.1577%) |
| default release optimized WASM general | 1,403,447 | 1,400,219 | -3,228 (-0.2300%) |
| default release optimized WASM immediate | 1,405,301 | 1,402,077 | -3,224 (-0.2294%) |
| default size native general text | 1,091,151 | 1,089,359 | -1,792 (-0.1642%) |
| default size native immediate text | 1,092,123 | 1,090,675 | -1,448 (-0.1326%) |
| default size optimized WASM general | 679,616 | 678,998 | -618 (-0.0909%) |
| default size optimized WASM immediate | 680,024 | 679,559 | -465 (-0.0684%) |
| all-feature release native general text | 2,121,287 | 2,118,223 | -3,064 (-0.1444%) |
| all-feature release native immediate text | 2,124,127 | 2,121,063 | -3,064 (-0.1442%) |
| all-feature release optimized WASM general | 1,477,270 | 1,474,049 | -3,221 (-0.2180%) |
| all-feature release optimized WASM immediate | 1,479,243 | 1,479,126 | -117 (-0.0079%) |
| all-feature size native general text | 1,093,391 | 1,091,959 | -1,432 (-0.1310%) |
| all-feature size native immediate text | 1,094,347 | 1,092,915 | -1,432 (-0.1309%) |
| all-feature size optimized WASM general | 679,649 | 679,187 | -462 (-0.0680%) |
| all-feature size optimized WASM immediate | 679,772 | 679,462 | -310 (-0.0456%) |

The regenerated five-crate graphs contain 15,119 nodes / 25,213 edges for
production, 17,439 / 28,529 with tests and examples, and 21,485 / 34,630 with
all tests, examples, benches, and fuzz targets. A precise namespace/name audit
finds zero EMBER, subdivision-engine, segment-trace, or local-BSP node. The
production graph has one `boolean -> build_surface_arrangement`, one
`build_surface_arrangement -> corefine_surface`, and one
`build_polygon_soup_internal -> validate_source_triangles` edge. Hypercurve and
HyperSolve are excluded and untouched.

## Validation and reproduction

The following pass on the accepted source:

```sh
cargo fmt --all -- --check
cargo test --locked
cargo test --locked --all-features
cargo test --locked --lib --no-default-features
cargo check --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-Dwarnings' cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo check --locked --benches --all-features
cargo check --locked --examples --all-features
benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```

The default suite passes 199 tests with six documented opt-in ignores. The
all-feature suite passes 200 tests with six ignores; the minimal library passes
152 tests. Fuzz targets, benches, examples, clippy, warning-denied rustdoc, and
both size matrices compile cleanly.

Call graphs were generated from the workspace root with:

```sh
tools/hyper-callgraph/target/release/hyper-callgraph --root . --out-dir hypermesh/target/callgraph-hypermesh-source-vertex-cardinality-production --crate-name hypermesh,hyperreal,hyperlimit,hyperlattice,hypertri --format json --per-library
tools/hyper-callgraph/target/release/hyper-callgraph --root . --out-dir hypermesh/target/callgraph-hypermesh-source-vertex-cardinality-tests --crate-name hypermesh,hyperreal,hyperlimit,hyperlattice,hypertri --include-tests --include-examples --format json --per-library
tools/hyper-callgraph/target/release/hyper-callgraph --root . --out-dir hypermesh/target/callgraph-hypermesh-source-vertex-cardinality-all --crate-name hypermesh,hyperreal,hyperlimit,hyperlattice,hypertri --include-tests --include-examples --include-bench --include-fuzz --format json --per-library
```

Phases 17 and 18 remain open for the external real-world corpus, remaining
per-case CGAL gaps, further measured arrangement/corefinement lifetime and
allocation reduction, and the final removal/policy/caller audit.
