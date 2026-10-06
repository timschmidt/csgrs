# Phase 17 flat face-constraint schedules

Date: 2026-08-05

Status: accepted checkpoint; Phases 17 and 18 remain open

Implementation: Hypermesh `36f9eef6`

Direct parent/evidence: flat face-point schedules `c5be7425` / `cc1cb281`

## Result

Changed source faces still had two tree-backed constraint carriers after their
point schedules became flat. The first `BTreeMap` canonicalized authored source
and intersection segments, then immediately copied them into a `Vec` for the
complete crossing-pair loop. The second retained a construction-line value for
every split edge even though line identity has completed its last use during
crossing-point construction; boundary comparison and Hypertri consume only the
unique endpoint pair.

The retained implementation fallibly sizes one authored vector, sorts it by
endpoint pair and construction identity, and deduplicates it so the first row
keeps the same canonical minimum line as the former map. Split edges use one
fallibly growing sorted vector with binary-search insertion. The already sorted
projected `(arrangement ID, ExactPoint)` schedule also replaces a duplicate
local-ID vector during CDT translation and output mapping.

This is one general exact face-arrangement schedule. It does not inspect a
fixture, size, coordinate width, operation, topology, result, policy name,
competitor, or benchmark. There is no threshold, compatibility shim, alternate
engine, or incomplete fast path.

## Exactness, completeness, and policy

- Authored constraints retain exactly the former `BTreeMap` order: ascending
  normalized endpoint pair. Sorting by line inside each endpoint group makes
  deduplication retain the same minimum `ConstructionEdgeIdentity`.
- Every authored pair still enters the exhaustive crossing loop. Line identity
  remains present through crossing classification and canonical intersection
  construction; it is discarded only when a split row has become a pure CDT
  endpoint constraint.
- Split-edge insertion is exact, sorted, and idempotent. Collinear overlap,
  T-junction, dense crossing-grid, boundary-only, and duplicate-line tests all
  traverse the same complete incidence and triangulation paths.
- Every capacity sum and reservation is checked. Failed authored/split/point/CDT
  allocations remain typed `CapacityOverflow`; absent endpoint translations
  remain typed `SurfaceArrangementFailed`.
- No predicate or certainty flow changes. `STRICT` remains exact-only.
  `APPROXIMATE_512` can terminate only in Hyperlimit, and the depth-128 symbolic
  control still reports `Approximate512Consumed`. All large exact rows remain
  `Certified` under both policies.

The new direct unit test proves canonical endpoint order, duplicate removal,
and minimum construction-line selection. Existing tests prove exhaustive dense
crossings, collinear overlaps/T-junctions, isolated contacts, policy-owned point
aliasing, topology certification, and all Boolean outputs.

## Deterministic retired-work measurements

The current release probe is compared to the saved direct-parent executable.
Measurements are CPU-11-pinned `perf stat -r 3` instruction and branch counts.
Outputs, topology summaries, and certainty are identical.

| Fixture / workload | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra, all four x1000 | 2,953,297,315 | 2,934,914,828 | -0.6224% | 506,896,740 | 503,109,540 | -0.7471% |
| affine boxes, all four x1000 | 6,968,349,413 | 6,921,011,726 | -0.6793% | 1,202,471,706 | 1,192,063,253 | -0.8656% |
| sparse 512 shells, all four x5 | 2,659,192,875 | 2,645,981,756 | -0.4968% | 477,044,915 | 474,518,418 | -0.5296% |
| dense coplanar 32, all four x1 | 9,972,761,305 | 9,925,271,397 | -0.4762% | 1,701,385,195 | 1,689,447,681 | -0.7016% |
| clipped voxel torus 33, all four x3 | 2,623,875,601 | 2,623,002,814 | -0.0333% | 440,142,564 | 439,963,737 | -0.0406% |
| 2,049-bit wide boxes, union x5 | 11,798,148,370 | 11,796,259,343 | -0.0160% | 2,173,653,839 | 2,173,181,388 | -0.0217% |
| full YeahRight instrumented kernel x1 | 10,607,826,071 | 10,602,455,746 | -0.0506% | 1,791,938,581 | 1,790,759,530 | -0.0658% |

The symbolic controls expose the only retained tradeoff:

| Fixture / policy / workload | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change | Certainty |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| depth 1 / `STRICT` / all four x20 | 4,270,537,378 | 4,271,806,687 | +0.0297% | 932,953,984 | 933,203,135 | +0.0267% | `Certified` |
| depth 128 / `APPROXIMATE_512` / all four x5 | 1,504,973,121 | 1,504,804,477 | -0.0112% | 274,687,945 | 274,671,204 | -0.0061% | `Approximate512Consumed` |

The shallow row adds about 63,466 instructions per complete four-output
operation. It is retained because all seven exact controls and the deep
terminal-policy control improve, allocation traffic falls broadly, and the
algorithm removes redundant ownership without eager scalar work.

A smaller ordered-insertion authored schedule was measured and completely
removed. It avoided the `RawConstraint` sort instantiation but cost 0.05--0.14%
instructions on crossing, sparse, and torus controls. The retained batch sort
has no size/count threshold and wins the primary performance objective.

## Large-fixture heap matrix

All fifteen selectors ran in fresh processes under both policies. Every policy
pair has byte-identical output, certainty, peak, allocation counts, reallocation
counts, and allocated bytes. Every peak is byte-identical to the parent.

| Selector | Peak bytes | Parent/current calls | Parent/current cumulative bytes |
| --- | ---: | ---: | ---: |
| boxes-3072 | 12,383,968 | 17,179 / 16,637 | 15,798,758 / 15,498,934 |
| boxes-3072-general | 12,630,096 | 36,365 / 35,823 | 18,128,606 / 17,828,782 |
| dense-coplanar-16 | 14,893,071 | 252,358 / 240,070 | 59,814,506 / 57,283,178 |
| dense-coplanar-32 | 59,391,879 | 1,008,360 / 959,208 | 239,115,978 / 228,990,666 |
| sparse-shells-512 | 11,888,929 | 205,284 / 201,188 | 32,733,608 / 31,502,760 |
| self-PWN-clusters-512 | 12,284,773 | 207,278 / 203,182 | 33,329,414 / 32,098,566 |
| wide-rational-64 | 17,861,062 | 382,563 / 382,021 | 31,836,014 / 31,536,190 |
| wide-rational-512 | 18,750,079 | 1,293,377 / 1,292,835 | 104,123,102 / 103,823,278 |
| wide-rational-2048 | 27,423,760 | 1,565,441 / 1,564,899 | 384,528,686 / 384,228,862 |
| voxel-torus-33 | 12,784,184 | 27,728 / 27,430 | 22,801,432 / 22,712,960 |
| voxel-torus-65 | 51,784,828 | 101,579 / 100,979 | 104,411,211 / 104,232,547 |
| yeahright | 5,161,051 | 166,969 / 166,768 | 11,335,919 / 11,276,143 |
| yeahright-4 | 19,676,989 | 727,591 / 727,161 | 45,466,616 / 45,338,168 |
| yeahright-8 | 76,258,109 | 3,314,116 / 3,313,269 | 187,496,890 / 187,244,826 |
| yeahright-full-rotated | 59,440,454 | 9,917,399 / 9,911,765 | 586,087,151 / 584,399,479 |

The full row retains a 52,317,092-byte incremental kernel peak. Calls fall by
5,634 and cumulative traffic by 1,687,672 bytes; reallocations rise by 2,062 to
1,612,899 because the fallible flat split vectors grow geometrically. Dense-32
removes 49,152 calls and 10,125,312 bytes of traffic.

Heaptrack artifact
`target/phase17-flat-face-constraint-schedules-full-strict.zst.zst` reports an
exact 59,514,182-byte peak, unchanged from the parent, 11,633,116 allocation
function calls (-3,572), and 2,669,159 temporary allocations (+36). Peak stacks
are `/tmp/hypermesh-flat-constraints-peak.stacks`.

## Historical and competitive boundaries

One advisory current full process reports 1.13 s / 68,864 KiB for `STRICT` and
1.08 s / 68,896 KiB for `APPROXIMATE_512`, with exact empty output and
`Certified` certainty. Host clocks were frequency-bimodal, so no full wall-time
speedup is claimed. Historical EMBER remains 3,312.66 s / 329,352 KiB; the
strict advisory row is about 2,932x faster and uses about 79.1% less RSS.
Historical CGAL remains 0.09 s / 15,516 KiB; it is still about 12.56x faster and
4.44x smaller by RSS.

CGAL 6.0.3 EPECK exact-OFF was refreshed for 63 internal repetitions per copy
mode. Hypermesh used 63 fresh CPU-11-pinned processes per policy. The absolute
window is recorded, but not used to infer this checkpoint's small wall delta:
same-time parent medians and current medians moved in opposite directions across
the two fixtures, while deterministic counters improved every exact row.

| Fixture | CGAL outside / inside | Hypermesh `STRICT` | Ratio to outside | Hypermesh `APPROXIMATE_512` | Ratio to outside |
| --- | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 87,104 / 136,461 ns | 518,134 ns | 5.948x | 484,877 ns | 5.567x |
| affine boxes | 337,037 / 339,646 ns | 1,109,593 ns | 3.292x | 1,193,138 ns | 3.540x |

Fresh Hypermesh ranges were 403,312--761,708 ns, 403,423--758,607 ns,
790,226--1,305,391 ns, and 779,007--1,437,281 ns respectively. Every competitive
gap remains open; no aggregate, historical win, or favorable clock sample is
substituted for a losing case.

## Code, binary size, and call graph

The implementation changes one production file by 100 insertions and 56
deletions, including typed failure handling and the direct invariant test.
Performance-priority release builds grow modestly because the canonical
`RawConstraint` sort instantiation remains; the rejected ordered-insertion
variant was smaller but measurably slower. All size-profile artifacts shrink.

| Configuration | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| default release native general text | 1,958,098 | 1,959,578 | +1,480 (+0.0756%) |
| default release native immediate text | 1,961,242 | 1,962,722 | +1,480 (+0.0755%) |
| default release optimized WASM general | 1,376,453 | 1,382,028 | +5,575 (+0.4050%) |
| default release optimized WASM immediate | 1,378,310 | 1,383,876 | +5,566 (+0.4038%) |
| default size native general text | 1,069,423 | 1,068,847 | -576 (-0.0539%) |
| default size native immediate text | 1,070,379 | 1,069,811 | -568 (-0.0531%) |
| default size optimized WASM general | 662,891 | 662,218 | -673 (-0.1015%) |
| default size optimized WASM immediate | 663,477 | 662,648 | -829 (-0.1250%) |
| all-feature release native general text | 2,093,367 | 2,095,063 | +1,696 (+0.0810%) |
| all-feature release native immediate text | 2,096,223 | 2,097,919 | +1,696 (+0.0809%) |
| all-feature release optimized WASM general | 1,450,176 | 1,455,985 | +5,809 (+0.4006%) |
| all-feature release optimized WASM immediate | 1,452,176 | 1,457,954 | +5,778 (+0.3979%) |
| all-feature size native general text | 1,071,663 | 1,071,095 | -568 (-0.0530%) |
| all-feature size native immediate text | 1,072,635 | 1,072,059 | -576 (-0.0537%) |
| all-feature size optimized WASM general | 663,109 | 662,267 | -842 (-0.1270%) |
| all-feature size optimized WASM immediate | 663,391 | 662,541 | -850 (-0.1281%) |

The regenerated five-crate graphs contain 15,141 nodes / 25,248 edges for
production, 17,461 / 28,564 with tests and examples, and 21,507 / 34,665 with
all tests, examples, benches, and fuzz targets. A precise namespace audit finds
zero removed EMBER, subdivision-engine, segment-trace, or local-BSP node. There
is exactly one `boolean -> build_surface_arrangement`, one
`build_surface_arrangement -> corefine_surface`, one
`corefine_face -> canonicalize_constraint_lines`, and one
`corefine_face -> insert_sorted_edge` edge. Hypercurve and HyperSolve remain
excluded and untouched.

## Validation and reproduction

The accepted source passes 200 default tests with six opt-in ignores, 201
all-feature tests with six ignores, and 153 minimal library tests. Warning-
denied Clippy and rustdoc, fuzz/bench/example compilation, formatting, both size
matrices, all fifteen two-policy large-heap rows, and all three call graphs pass.

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

Phases 17 and 18 remain open for external real-world corpus completion, every
remaining per-case CGAL gap, further exact arrangement/scalar work, and the
final removal/policy/caller audit.
