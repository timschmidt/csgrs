# Phase 17 flat face-point schedules

Date: 2026-08-05

Status: accepted checkpoint; Phases 17 and 18 remain open

Implementation: Hypermesh `bdecdd0d11e00d368469625d6f411662c710b638`
and `c5be7425ae32d1fad07043e9260f1ef9cdb0bf23`

Direct parent/evidence: retained source-vertex cardinality `f01a9fd3` /
`8015bb87`

## Result

Changed source triangles used tree containers for every face-local projected
point even though operation-wide arrangement IDs are already canonical `u32`
values and all later consumers require ID order. The retained implementation
uses one fallibly sized vector for the initial ID set, sorts and deduplicates it
once, projects into a contiguous `(ID, ExactPoint)` schedule, batches newly
constructed crossings, and restores the same sorted order before incidence and
CDT. Binary search replaces the projected-point and global-to-local maps. The
three source boundary edges are an inline sorted array rather than two
temporary sets.

`bdecdd0d` first hoists each authored constraint's two invariant projected
endpoint lookups out of its candidate-point loop. `c5be7425` then removes the
face-local point `BTreeSet`, projected-point `BTreeMap`, local-ID `BTreeMap`,
and boundary comparison `BTreeSet`. The line-canonicalization and split-line
maps deliberately remain unchanged: their ordered minimum construction-line
selection is part of the exact arrangement recipe, not incidental lookup
storage.

This is one general flat data schedule. It does not inspect a fixture, size,
coordinate width, operation, topology, result, policy name, or competitor.
There is no threshold, compatibility shim, special Boolean path, or alternate
engine.

Two adjacent experiments were rejected and are absent. Adding eagerly retained
arbitrary-width rational line filters to every `Line2Orientation` increased
sparse/dense instructions about 18--20% because cheap affine evidence normally
settled the query; all Hyperlimit and Hypermesh changes for that variant were
removed. Scanning the new crossing vector on every insertion to suppress
duplicate projection clones helped sparse work slightly but added about 0.04%
dense instructions; it was also removed.

## Exactness, completeness, and policy

- The initial and final point orders are identical to the former ordered maps:
  ascending canonical arrangement ID. Crossing-pair discovery order and
  construction identity insertion order are unchanged.
- Every authored constraint pair is still tested. Every retained or newly
  constructed crossing enters the final sorted schedule, and duplicate IDs are
  removed only after the arena has already established their canonical numeric
  identity.
- Support-plane membership, source half-space containment, exact segment
  incidence, exact segment ordering, constrained triangulation, topology
  certification, and source orientation use the same predicates in the same
  logical order.
- Capacity arithmetic and every new vector reservation return typed errors.
  Missing projected endpoints and missing CDT-local endpoints now return typed
  `SurfaceArrangementFailed` errors instead of relying on indexing panics.
- `STRICT` remains exact-only. `APPROXIMATE_512` can terminate only through
  Hyperlimit's existing terminal and aggregate certainty remains observable.
  The depth-128 symbolic control still reports `Approximate512Consumed`; every
  large exact fixture remains `Certified` under both policies.

## Deterministic retired-work measurements

The current release probe is compared to the saved direct-parent executable
`target/phase17-hoisted-face-constraint-endpoints-competitive-probe`.
Measurements are CPU-11-pinned hardware instruction and branch counts. Exact
outputs, topology summaries, and certainty are identical.

| Fixture / workload | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| overlapping boxes, all four x1000 | 3,910,185,917 | 3,849,530,545 | -1.5512% | 675,524,777 | 661,560,339 | -2.0672% |
| sparse 512 shells, all four x5 | 2,696,443,482 | 2,659,192,875 | -1.3815% | 486,171,989 | 477,044,915 | -1.8773% |
| dense coplanar 32, all four x1 | 10,049,244,466 | 9,972,761,305 | -0.7611% | 1,719,804,728 | 1,701,385,195 | -1.0710% |
| clipped voxel torus 33, all four x3 | 2,627,255,581 | 2,623,875,601 | -0.1287% | 441,042,505 | 440,142,564 | -0.2040% |
| 2,049-bit wide boxes, union x5 | 11,803,578,487 | 11,798,148,370 | -0.0460% | 2,175,070,927 | 2,173,653,839 | -0.0652% |
| full YeahRight, intersection x1 | 10,361,948,680 | 10,343,754,512 | -0.1756% | 1,778,792,633 | 1,773,855,479 | -0.2776% |

The symbolic controls expose the only bounded tradeoff:

| Fixture / policy / workload | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change | Certainty |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| depth 1 / `STRICT` / all four x20 | 4,294,623,136 | 4,270,537,378 | -0.5608% | 940,199,286 | 932,953,984 | -0.7706% | `Certified` |
| depth 128 / `APPROXIMATE_512` / all four x5 | 1,504,922,893 | 1,504,973,121 | +0.0033% | 274,739,312 | 274,687,945 | -0.0187% | `Approximate512Consumed` |

The deep row adds 50,228 instructions across five complete four-operation
runs. It is retained because all broad exact rows and branches improve, the
face algorithm is simpler and contiguous, allocation traffic falls, and every
linked consumer shrinks materially.

## Large-fixture heap matrix

All fifteen selectors ran in fresh processes under both policies. Every pair
has byte-identical output, certainty, peak, allocation counts, and allocated
bytes. Peaks remain byte-identical to the direct parent; flat face schedules
remove tree-node allocations and cumulative traffic.

| Selector | Peak bytes | Parent/current calls | Parent/current cumulative bytes |
| --- | ---: | ---: | ---: |
| boxes-3072 | 12,383,968 | 19,219 / 17,179 | 16,337,262 / 15,798,758 |
| boxes-3072-general | 12,630,096 | 38,405 / 36,365 | 18,667,110 / 18,128,606 |
| dense-coplanar-16 | 14,893,071 | 276,934 / 252,358 | 66,081,410 / 59,814,506 |
| dense-coplanar-32 | 59,391,879 | 1,106,664 / 1,008,360 | 264,183,522 / 239,115,978 |
| sparse-shells-512 | 11,888,929 | 213,476 / 205,284 | 34,654,632 / 32,733,608 |
| self-PWN-clusters-512 | 12,284,773 | 215,470 / 207,278 | 35,250,438 / 33,329,414 |
| wide-rational-64 | 17,861,062 | 384,603 / 382,563 | 32,374,518 / 31,836,014 |
| wide-rational-512 | 18,750,079 | 1,295,417 / 1,293,377 | 104,661,606 / 104,123,102 |
| wide-rational-2048 | 27,423,760 | 1,567,481 / 1,565,441 | 385,067,190 / 384,528,686 |
| voxel-torus-33 | 12,784,184 | 28,300 / 27,728 | 22,939,728 / 22,801,432 |
| voxel-torus-65 | 51,784,828 | 102,719 / 101,579 | 104,688,451 / 104,411,211 |
| yeahright | 5,161,051 | 167,357 / 166,969 | 11,429,415 / 11,335,919 |
| yeahright-4 | 19,676,989 | 728,406 / 727,591 | 45,666,328 / 45,466,616 |
| yeahright-8 | 76,258,109 | 3,315,742 / 3,314,116 | 187,893,674 / 187,496,890 |
| yeahright-full-rotated | 59,440,454 | 9,926,697 / 9,917,399 | 587,830,239 / 586,087,151 |

The full row retains a 52,317,092-byte incremental kernel peak. Its
reallocations move 1,610,293 to 1,610,837 while allocation calls fall 9,298
and traffic falls 1,743,088 bytes. Heaptrack artifact
`target/phase17-flat-face-point-schedules-full-strict-direct.zst.zst` and peak
stacks `/tmp/hypermesh-flat-face-points-peak.stacks` sum to exactly 59,514,182
bytes, identical to the parent. Final analysis reports 11,636,688 allocation
function calls (8,754 fewer) and 2,669,123 temporary allocations.

## Historical and competitive boundaries

Pinned CGAL 6.0.3 EPECK exact-OFF values remain unchanged. Hypermesh values
are medians of 63 fresh CPU-11-pinned processes per policy:

| Fixture | CGAL outside / inside | Hypermesh `STRICT` | Ratio | Hypermesh `APPROXIMATE_512` | Ratio |
| --- | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 112,972 / 122,712 ns | 408,242 ns | 3.614x | 408,332 ns | 3.614x |
| affine boxes | 376,124 / 383,553 ns | 779,267 ns | 2.072x | 774,896 ns | 2.060x |

Both gaps improve but remain open. Full-process timing was frequency-bimodal:
the direct pinned `perf` run completed in 1.004 seconds, while a later five-run
`time` window ranged 1.75--2.39 seconds. No full-wall speedup is claimed from
that noisy window. RSS remains stable at 69,192--69,380 KiB. Historical EMBER
remains orders of magnitude slower and larger, while historical CGAL's 0.09 s
/ 15,516 KiB full row remains materially ahead. No favorable aggregate closes
any losing CGAL case.

## Code, binary size, and call graph

The two commits change one production file by 147 insertions and 64 deletions.
Most additions are fallible capacity and malformed-index handling. Against the
preceding recorded checkpoint, all sixteen canonical linked artifacts shrink:

| Configuration | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| default release native general text | 1,982,922 | 1,958,098 | -24,824 (-1.2519%) |
| default release native immediate text | 1,986,066 | 1,961,242 | -24,824 (-1.2499%) |
| default release optimized WASM general | 1,400,219 | 1,376,453 | -23,766 (-1.6973%) |
| default release optimized WASM immediate | 1,402,077 | 1,378,310 | -23,767 (-1.6951%) |
| default size native general text | 1,089,359 | 1,069,423 | -19,936 (-1.8301%) |
| default size native immediate text | 1,090,675 | 1,070,379 | -20,296 (-1.8609%) |
| default size optimized WASM general | 678,998 | 662,891 | -16,107 (-2.3722%) |
| default size optimized WASM immediate | 679,559 | 663,477 | -16,082 (-2.3665%) |
| all-feature release native general text | 2,118,223 | 2,093,367 | -24,856 (-1.1734%) |
| all-feature release native immediate text | 2,121,063 | 2,096,223 | -24,840 (-1.1711%) |
| all-feature release optimized WASM general | 1,474,049 | 1,450,176 | -23,873 (-1.6196%) |
| all-feature release optimized WASM immediate | 1,479,126 | 1,452,176 | -26,950 (-1.8220%) |
| all-feature size native general text | 1,091,959 | 1,071,663 | -20,296 (-1.8587%) |
| all-feature size native immediate text | 1,092,915 | 1,072,635 | -20,280 (-1.8556%) |
| all-feature size optimized WASM general | 679,187 | 663,109 | -16,078 (-2.3672%) |
| all-feature size optimized WASM immediate | 679,462 | 663,391 | -16,071 (-2.3653%) |

The regenerated five-crate graphs contain 15,140 nodes / 25,243 edges for
production, 17,460 / 28,559 with tests and examples, and 21,506 / 34,660 with
all tests, examples, benches, and fuzz targets. A precise namespace audit finds
zero removed EMBER, subdivision-engine, segment-trace, or local-BSP node. There
is one production `boolean -> build_surface_arrangement` edge and one
`build_surface_arrangement -> corefine_surface` edge. Hypercurve and HyperSolve
remain excluded and untouched.

## Validation and reproduction

The accepted source passes default 199 tests with six opt-in ignores,
all-feature 200 tests with six ignores, minimal 152 library tests, warning-
denied Clippy and rustdoc, fuzz/bench/example compilation, formatting, and both
size matrices. Principal commands:

```sh
cargo fmt --all -- --check
cargo test --locked
cargo test --locked --all-features
cargo test --locked --lib --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS=-Dwarnings cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo check --locked --benches --examples --all-features
benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all
```

Phases 17 and 18 remain open for the external real-world corpus, every
remaining per-case CGAL gap, further exact corefinement/scalar work, and the
final removal/policy/caller audit.
