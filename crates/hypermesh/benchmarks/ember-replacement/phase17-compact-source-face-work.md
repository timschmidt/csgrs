# Phase 17 checkpoint: compact source-face arrangement work

Date: 2026-08-05

Status: accepted checkpoint; Phases 17 and 18 remain open

Hypermesh implementation: `cf07cbaf8fcd19bc4a3655a6386dd7c55af170e5`

Measured parent: `2e91a5177d2d2a66c35489e123e4a6f5b33c30e6`

The surface corefinement stage now stores the three point IDs of every source
triangle inline and retains only additional intersection constraints and
contacts. Source-boundary constraints are already implied by the triangle and
its canonical retained edge identities, so they are constructed only for a
face that actually reaches constrained triangulation. Face work is consumed in
source order and its local vectors are released as soon as that face finishes.

The arrangement point arena also drops a duplicate point-to-construction
identity vector. The structural identity map remains the canonical recipe-to-
point index and the numeric interner remains the complete aliasing route. A
direct construction test now proves that crossing split lines derive the
canonical support-plane triple instead of depending on a diagnostic duplicate
owner.

This is one general triangle-arrangement representation. There is no fixture,
coordinate, triangle-count, topology, operation, expected-output, policy, or
competitor branch. No compatibility shim or alternate Boolean path is added.

## Exactness, policy, and complete exits

`FaceWork` has one meaning for every source triangle:

1. its inline boundary is populated from the checked source vertex cycle;
2. pairwise events append only additional constraints or point contacts;
3. retained source-edge points are propagated from the source edge identity;
4. an unchanged face emits its source triangle directly; and
5. a changed face reconstructs its three source edges and enters the same exact
   PSLG/CDT schedule as before.

The source boundary, vertex identity cycle, and edge identity cycle must each
have exactly three aligned entries. Missing or malformed carriers return the
existing typed surface-arrangement error. Every capacity reserve remains
checked. Structural point identities still precede numeric equality, exact
rational contradictions remain rejected, and the general point interner still
promotes through `DecisionContext` and Hyperlimit when exact retained facts do
not decide equality.

No predicate or terminal was added locally. `STRICT` remains exact-only;
`APPROXIMATE_512` can decide only through Hyperlimit's terminal and updates the
aggregate certainty. The all-feature suite's terminal-policy, point-aliasing,
radial-equality, output-certification, symbolic-depth, and pairwise-contact
tests all pass.

## Broad performance controls

Saved parent and current default-feature release executables ran adjacently on
CPU 11 with three `perf stat` repetitions. Outputs and certainty are identical.
Instructions and branches improve on every broad exact workload:

| Fixture / workload | Parent instructions | Current instructions | Change | Branch change |
| --- | ---: | ---: | ---: | ---: |
| overlapping boxes, all four ×1000 | 4,016,548,370 | 3,988,598,107 | -0.6959% | -0.8534% |
| sparse multishell 512, all four ×5 | 2,770,857,207 | 2,750,323,478 | -0.7411% | -0.8747% |
| dense coplanar 32, all four ×1 | 10,303,958,112 | 10,289,216,560 | -0.1431% | -0.2828% |
| clipped voxel torus 33, all four ×3 | 2,666,940,145 | 2,646,100,131 | -0.7814% | -0.9407% |
| 2,049-bit rational boxes, union ×5 | 11,869,941,359 | 11,836,833,066 | -0.2789% | -0.3069% |
| full rotated YeahRight, intersection ×1 | 10,430,883,271 | 10,404,981,358 | -0.2483% | -0.2896% |

The improvement comes from smaller face rows, zero face-local allocations for
untouched faces, early row destruction, and removal of duplicate arrangement
identity ownership. Candidate discovery, exact predicates, triangulation,
radial topology, winding, and output certification are unchanged.

Two general symbolic controls make the cost of reconstructing source-edge
constraints on changed faces explicit:

| Symbolic workload | Parent/current instructions | Movement | Branch movement |
| --- | ---: | ---: | ---: |
| depth 1, all four ×20, `STRICT` | 4,288,944,771 / 4,295,790,760 | +0.1596% | +0.1526% |
| depth 128, all four ×5, `APPROXIMATE_512` | 1,505,496,051 / 1,505,498,946 | +0.0002% | +0.0023% |

Depth 1 remains `Certified`; depth 128 retains the same output and
`Approximate512Consumed`. The small changed-face cost is retained and reported
because every broad control improves and the general policy path remains
complete.

An experimental sparse changed-row indirection was rejected and fully removed.
It did not reduce the full-fixture peak, regressed dense-32 instructions about
0.125%, and saved only about 1.04 MB of cumulative payload churn. The accepted
layout is the simpler dense source-order schedule.

## Large-fixture heap matrix

All fifteen selectors ran under both policies. Each policy pair is byte-
identical and `Certified`. Total peak never increases; the full fixture falls
materially. Allocation calls and cumulative payload fall in every row.

| Selector | Parent/current peak | Parent/current allocations | Parent/current payload |
| --- | ---: | ---: | ---: |
| boxes-3072 | 12,383,968 / 12,383,968 | 31,262 / 19,227 | 17,557,542 / 16,571,238 |
| boxes-3072-general | 12,630,096 / 12,630,096 | 50,448 / 38,413 | 19,887,390 / 18,901,086 |
| dense-coplanar-16 | 14,893,071 / 14,893,071 | 283,087 / 276,942 | 66,660,634 / 66,303,658 |
| dense-coplanar-32 | 59,391,879 / 59,391,879 | 1,131,251 / 1,106,674 | 266,498,458 / 265,072,426 |
| sparse-shells-512 | 11,888,929 / 11,888,929 | 219,630 / 213,485 | 35,740,148 / 35,113,460 |
| self-PWN-clusters-512 | 12,284,773 / 12,284,773 | 221,632 / 215,479 | 36,336,670 / 35,709,262 |
| wide-rational-64 | 17,861,062 / 17,861,062 | 396,646 / 384,611 | 33,594,798 / 32,608,494 |
| wide-rational-512 | 18,750,079 / 18,750,079 | 1,307,460 / 1,295,425 | 105,881,886 / 104,895,582 |
| wide-rational-2048 | 27,423,760 / 27,423,760 | 1,579,524 / 1,567,489 | 386,287,470 / 385,301,166 |
| voxel-torus-33 | 12,784,184 / 12,784,184 | 41,003 / 28,308 | 24,189,972 / 23,173,108 |
| voxel-torus-65 | 51,784,828 / 51,784,828 | 152,672 / 102,729 | 109,622,215 / 105,624,183 |
| YeahRight | 5,161,051 / 5,161,051 | 168,979 / 167,362 | 11,588,357 / 11,457,989 |
| YeahRight ×4 | 19,676,989 / 19,676,989 | 734,974 / 728,413 | 46,308,650 / 45,781,994 |
| YeahRight ×8 | 76,258,109 / 76,258,109 | 3,342,288 / 3,315,751 | 190,485,420 / 188,359,212 |
| full rotated YeahRight | 63,243,888 / 59,440,454 | 9,972,500 / 9,926,707 | 593,227,133 / 588,741,037 |

On the 23,788-triangle full row, total peak falls 3,803,434 bytes (6.01%),
the incremental kernel peak falls by the same amount to 52,317,092 bytes
(6.78%), allocation calls fall 45,793, reallocations fall 1,115, and cumulative
payload falls 4,486,096 bytes. Both policies return the exact same certified
empty result.

## Heaptrack ownership

The current strict capture is
`target/phase17-compact-face-work-full-strict.zst`. Heaptrack reports a
59,514,182-byte peak versus 63,317,616 bytes at the parent and 11,645,452 calls
to allocation functions versus 11,692,360.

| Live owner / nested allocation stack | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| exact source owner, reattributed | 31,732,864 | 31,732,864 | unchanged |
| arrangement excluding source caches | 24,387,662 | 20,584,228 | -3,803,434 (-15.60%) |
| raw `corefine_surface` stack | 26,175,990 | 16,270,764 | -9,905,226 (-37.84%) |
| raw pairwise-intersection stack | 24,982,804 | 24,095,008 | -887,796 (-3.55%) |
| total peak | 63,317,616 | 59,514,182 | -3,803,434 (-6.01%) |

The source owner is independently reproduced as 4,000,792 bytes from soup
construction, 20,552,832 bytes of demand-created support planes, and 7,179,240
bytes of demand-created edge cycles. The nested stack rows are not additive;
the pairwise movement reflects the later, lower peak moment. The exclusive
arrangement reduction exactly matches the total-peak reduction.

## Historical and CGAL boundaries

Fresh full processes preserve the exact certified-empty result:

| Policy | Wall time | Maximum RSS |
| --- | ---: | ---: |
| `STRICT` | 1.07 s | 69,172 KiB |
| `APPROXIMATE_512` | 1.07 s | 69,292 KiB |

These single-process clock rows are advisory; retired instructions are the
stable performance control above. Strict RSS falls 5.00% from the parent
checkpoint. Against historical EMBER, the current strict row is about 3,096×
faster and uses 79.0% less RSS. Historical CGAL remains ahead at 0.09 seconds /
15,516 KiB: the current full boundary is about 11.89× slower and 4.46× larger.

Pinned CGAL 6.0.3 EPECK exact-OFF medians are unchanged. Hypermesh was
refreshed as 63 fresh CPU-11-pinned processes per policy. Every output remains
exact, valid, closed, structurally valid, and topology-identical:

| Fixture | CGAL outside / inside | Hypermesh `STRICT` | Ratio | Hypermesh `APPROXIMATE_512` | Ratio |
| --- | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 112,972 / 122,712 ns | 433,890 ns | 3.841× | 430,771 ns | 3.813× |
| affine boxes | 376,124 / 383,553 ns | 819,744 ns | 2.180× | 815,734 ns | 2.169× |

Relative to the parent Hypermesh executable, the four medians improve
0.64–2.66%. Every remaining CGAL loss stays open; no aggregate or historical
gain is called parity.

## Code, binary size, and call graph

The implementation changes one production/test file by 154 insertions and 125
deletions. Default release native text shrinks 2,056 bytes and all-feature
release native text shrinks 1,768 bytes. Optimized release WASM shrinks
1,688/1,678 bytes for default general/immediate and 1,631/1,615 bytes for all
features. Size-profile WASM shrinks 500–650 bytes. Size-profile native text
grows 2,952 bytes in every configuration; that bounded loss remains explicit.

The current five-crate graphs contain 15,097 nodes / 25,175 edges for
production, 17,417 / 28,491 with tests and examples, and 21,463 / 34,592 with
all tests, examples, benches, and fuzz targets. They contain zero exact
EMBER/local-BSP/segment-trace node. The arrangement point arena retains one
direct edge to `PointInterner::try_with_capacity`; every general equality stays
on the canonical policy cascade. Hypercurve and HyperSolve are excluded and
untouched.

## Validation

The following pass:

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

The all-feature suite passes 198 tests with the six documented manual/opt-in
ignores: 151 unit, 8 Boolean, 11 competitive, 14 corpus-manifest, 3
intersection-corpus, 9 policy, and 2 README tests. The minimal library passes
150 tests.

Phases 17 and 18 remain open for the external real-world corpus, remaining
per-case CGAL gaps, further arrangement/corefinement lifetime reduction, and
the final removal/policy/caller audit.
