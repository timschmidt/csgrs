# Phase 17 shared exact source-vertex cycles

Captured 2026-08-05 on the established Ryzen 7 5800X3D / CPU 11 protocol.
Implementation commit
`df1d005c138f2dedb88f4c15a6d48f55368b173f`; the measured production parent
is `f646d8f008c69d1cd65fd3876480af443de435e0`. The intervening
`56ab9e983436c7e6f8365c05a219ed340ad04b02` commit contains evidence only and
has identical production code.

This is an accepted Phase 17 memory/performance checkpoint. It does not claim
CGAL EPECK parity, completion of the real-world fixture corpus, Phase 17
completion, or the Phase 18 audit.

## Result

Source polygons no longer clone three exact `Point3` values into a separately
allocated vertex cycle for every input triangle. One sized exact-position
owner is shared by all source faces of each mesh; every face retains only its
three checked `u32` source indices. Native meshes reuse their existing
`Arc<[Point3]>`. A raw borrowed view is copied once per source mesh because the
public returned `PolygonSoup` must remain owned after the borrow ends.

The representation remains general:

- standalone public triangles and quads retain their existing owned exact
  cycles;
- arrangement-derived polygons clear or own geometry according to their real
  topology instead of pretending to be source triangles;
- inversion shares the exact source owner and reverses only the three compact
  indices;
- source construction identity uses the same checked indices as the retained
  coordinates, so geometry and identity cannot diverge;
- no fixture name, coordinate, triangle count, expected result, operation,
  competitor, or benchmark state selects this path; and
- the representation adds no predicate or policy branch. Exact bounds,
  planes, candidate tests, and consuming predicates are unchanged.

This plays directly to Hyperreal's retained exact values: source coordinates
stay canonical and shareable, while later predicates still receive exact
`Real` references and can use their existing fact-driven schedules.

## Correctness and policy

The ownership test covers native owner reuse, one owned copy for a borrowed
view, checked compact indices, exact coordinate recovery, and inversion. The
complete all-feature suite and the minimal-feature library suite pass. Every
one of the 15
large-fixture allocator selectors produces a byte-identical `Certified` row
under `STRICT` and `APPROXIMATE_512`.

These particular fixtures never need terminal equality, so neither policy is
consumed. The existing policy suite separately proves that a genuinely
undecidable terminal equality remains exact under `STRICT`, terminates only in
Hyperlimit's 512-bit approximation under `APPROXIMATE_512`, and raises the
aggregate mesh certainty. The new ownership functions have no call-graph edge
to Hyperlimit and cannot bypass that rule.

## Retired work and wall-clock controls

The accepted parent and current release executables were run on the same pinned
CPU. All six broad controls retire less work. Counts are three-run `perf stat`
aggregates unless noted.

| Fixture / workload | Parent instructions | Current instructions | Change | Parent branches | Current branches | Change |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| overlapping boxes, all four ×1000 | 4,452,056,863 | 4,425,492,760 | -0.597% | 789,424,579 | 785,419,140 | -0.507% |
| sparse shells 512, all four ×5 | 3,322,239,063 | 3,293,359,807 | -0.869% | 620,081,466 | 614,655,893 | -0.875% |
| dense coplanar 32, all four ×1 | 10,608,848,804 | 10,573,675,522 | -0.332% | 1,825,317,571 | 1,813,449,321 | -0.650% |
| voxel torus 33, all four ×3 | 2,883,111,919 | 2,859,575,585 | -0.816% | 494,874,210 | 489,754,782 | -1.035% |
| wide rational 2048, union ×5 | 13,897,281,109 | 13,864,594,842 | -0.235% | 2,532,732,965 | 2,524,535,752 | -0.324% |
| full rotated YeahRight, intersection ×1 | 12,941,347,945 | 12,914,795,173 | -0.205% | 2,264,200,941 | 2,257,844,985 | -0.281% |

For affine boxes, a separate five-run counter mean falls from 3,803,469,517
to 3,790,674,490 instructions (-0.336%) and from 668,481,997 to 667,069,057
branches (-0.211%). Its counter-run elapsed mean improves from 412.758 ms to
405.504 ms (-1.76%).

Host frequency was visibly noisy. Interleaved 11-process parent/current
medians improve crossing octahedra by 2.74% under `STRICT` and 1.92% under
`APPROXIMATE_512`. The analogous affine medians move +2.99% and +6.10% even
though both retired work and the five-run elapsed mean improve. The checkpoint
therefore treats counters and the broad consistency of the heap/runtime data
as the stable signal rather than claiming an affine wall-time gain.

## Complete large-fixture allocator matrix

Every current row below is identical under `STRICT` and `APPROXIMATE_512` and
returns `Certified`. `Kernel peak` excludes the retained prepared-input
payload. `Added` is total allocated bytes over the operation. All deltas are
against the production parent.

| Selector | Current peak | Peak change | Current kernel peak | Kernel change | Alloc calls | Call change | Added bytes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| boxes-3072 | 14,349,904 | -2,703,296 (-15.852%) | 13,300,378 | -16.892% | 151,270 | -6,142 | 24,505,470 |
| boxes-3072-general | 14,399,504 | -2,260,320 (-13.567%) | 13,802,906 | -14.071% | 158,167 | -6,140 | 26,663,150 |
| dense-coplanar-16 | 17,046,229 | -2,703,296 (-13.688%) | 16,016,264 | -14.441% | 405,969 | -6,142 | 74,439,618 |
| dense-coplanar-32 | 67,937,525 | -10,813,376 (-13.731%) | 63,911,472 | -14.471% | 1,630,355 | -24,574 | 298,116,386 |
| sparse-shells-512 | 13,739,923 | -1,802,176 (-11.595%) | 12,777,766 | -12.361% | 436,499 | -4,094 | 47,059,244 |
| self-pwn-clusters-512 | 13,871,137 | -1,803,936 (-11.508%) | 12,906,768 | -12.263% | 437,008 | -4,098 | 47,572,222 |
| wide-rational-64 | 19,630,974 | -2,703,296 (-12.104%) | 19,012,850 | -12.448% | 635,832 | -6,142 | 45,640,558 |
| wide-rational-512 | 20,520,231 | -2,703,296 (-11.640%) | 19,892,898 | -11.964% | 1,635,498 | -6,142 | 124,470,758 |
| wide-rational-2048 | 29,195,064 | -2,703,296 (-8.475%) | 28,531,362 | -8.655% | 1,907,487 | -6,142 | 427,527,486 |
| voxel-torus-33 | 14,835,880 | -2,821,216 (-15.978%) | 13,768,022 | -17.006% | 171,173 | -6,410 | 31,754,892 |
| voxel-torus-65 | 59,816,684 | -11,043,936 (-15.585%) | 54,998,114 | -16.723% | 663,424 | -25,098 | 139,396,255 |
| yeahright | 5,596,865 | -374,816 (-6.277%) | 4,679,612 | -7.416% | 204,247 | -850 | 12,984,701 |
| yeahright-4 | 21,010,549 | -1,483,616 (-6.596%) | 17,371,694 | -7.868% | 861,695 | -3,370 | 51,324,786 |
| yeahright-8 | 80,135,413 | -5,918,816 (-6.878%) | 65,610,158 | -8.275% | 3,820,154 | -13,450 | 208,988,596 |
| yeahright-full-rotated | 154,886,838 | -10,466,656 (-6.330%) | 147,763,444 | -6.615% | 14,518,353 | -23,786 | 799,065,469 |

Reallocation counts are unchanged for all rows. For every selector, the drop
in total peak, kernel peak, and total added bytes is exactly the same number.
That is the expected signature of removing one independent source-cycle
allocation and its exact coordinate clones per source triangle; it is not a
shift into another stage.

## Stage-specific lifetime

Heaptrack's full rotated YeahRight peak confirms the allocator boundary:

| Live owner at peak | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| total | 165,427,230 | 154,960,566 | -10,466,664 (-6.33%) |
| polygon soup | 122,254,248 | 111,787,592 | -10,466,656 (-8.56%) |
| surface arrangement | 35,975,860 | 35,975,852 | -8 |
| corefinement | 32,732,060 | 32,732,220 | +160 |
| pairwise intersection | 2,853,604 | 2,853,604 | 0 |

The stable post-corefinement peak falls from 158,143,590 to 147,676,078
bytes. The source ownership reduction therefore stays live through output and
does not displace memory into arrangement, corefinement, or pairwise work.
The current capture is
`target/phase17-shared-source-vertices-full-strict.zst`.

## Historical and current CGAL boundary

A fresh current process for the 11,894-by-11,894 full rotated intersection
returns the exact empty result with `Certified` certainty in 1.52 seconds and
179,808 KiB maximum RSS. The paired parent process takes 1.55 seconds and
190,172 KiB. Against the established historical rows, current is roughly
2,179× faster than EMBER's 3,312.66 seconds and uses 45.4% less RSS than
EMBER's 329,352 KiB. The historical CGAL EPECK row remains substantially
ahead at 0.09 seconds and 15,516 KiB; those gaps stay open.

The pinned CGAL 6.0.3 EPECK executable was rerun for 21 repetitions over the
same exact OFF values in both copy modes. Every operation produced a valid,
closed, structurally valid mesh with the same topology and exact-volume oracle
as Hypermesh.

| Fixture | CGAL outside | CGAL inside | Hypermesh STRICT | Ratio to outside | Hypermesh APPROXIMATE_512 | Ratio to outside |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 196,616 ns | 197,326 ns | 415,138 ns | 2.111× | 434,740 ns | 2.211× |
| affine boxes | 392,123 ns | 559,072 ns | 872,033 ns | 2.224× | 885,614 ns | 2.259× |

These current-session medians are reported directly despite host variance.
Both exact small-case parity rows remain open; no competitor-specific path is
introduced to close them.

## Code, binary size, and call graph

The implementation changes 231 added and 53 removed lines, including its
ownership/inversion regression test. Against the production parent:

| Consumer/profile | Native text change | Optimized WASM change |
| --- | ---: | ---: |
| default release, general | -1,828 bytes (-0.092%) | +1,649 bytes (+0.117%) |
| default release, immediate | -1,828 bytes (-0.092%) | +1,624 bytes (+0.115%) |
| default size, general | +704 bytes (+0.065%) | +672 bytes (+0.099%) |
| default size, immediate | +712 bytes (+0.066%) | +446 bytes (+0.066%) |
| all-feature release, general | -2,468 bytes (-0.116%) | +1,806 bytes (+0.121%) |
| all-feature release, immediate | -2,468 bytes (-0.116%) | +1,773 bytes (+0.119%) |

Current all-feature release native text is 2,120,771/2,123,611 bytes and
optimized WASM is 1,490,213/1,492,170 bytes for general/immediate consumers.
Current all-feature size-profile native text is 1,084,567/1,085,523 bytes and
optimized WASM is 676,543/676,816 bytes.

The refreshed five-crate production graph has 14,949 nodes and 24,950 edges;
the matching examples/tests graph has 17,269 nodes and 28,266 edges. The full
examples/tests/benches/fuzz graph has 21,315 nodes and 34,367 edges. New
ownership helpers call only allocation, checked conversion/indexing, retained
geometry construction, and the pre-existing exact bounds/plane schedule. They
have no edge to Hyperlimit. The production graph contains one surface-
arrangement Boolean engine and no EMBER or local-BSP path.

## Validation and reproduction

```sh
cargo fmt --all -- --check
cargo test --locked --all-features
cargo test --locked --no-default-features --lib
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo check --locked --benches --all-features
cargo check --locked --examples --all-features

taskset -c 11 perf stat -r 3 -x, -e instructions:u,branches:u \
  target/release/examples/competitive_arrangement_probe \
  sparse_multishell_tetrahedra_512 all strict 5

env YEAHRIGHT_BENCH=1 \
  target/release/examples/large_mesh_kernel_heap_probe \
  yeahright-full-rotated strict

benchmarks/size-harness/measure.sh default
benchmarks/size-harness/measure.sh all

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir hypermesh/target/phase17-shared-source-vertices-callgraph-production \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --per-library --format json
```

The next memory work should follow the measured 111.79 MB polygon-soup owner,
particularly exact source planes and per-face bounds. Any replacement must
remain one clean representation/predicate schedule, preserve all exact paths,
and win broad controls rather than recognize a fixture.
