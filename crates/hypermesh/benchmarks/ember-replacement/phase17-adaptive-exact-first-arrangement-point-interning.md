# Phase 17 checkpoint: adaptive exact-first arrangement point interning

Date: 2026-08-05

Status: accepted checkpoint; Phases 17 and 18 remain open

Hypermesh implementation: `2e91a5177d2d2a66c35489e123e4a6f5b33c30e6`

Measured parent/evidence: `2b88d9c195a43199ab079bbda8048dd4f1ad5e4a`

The arrangement point arena now begins with the complete exact-rational
interner schedule. It retains Hyperreal rational storage identities, canonical
value fingerprints, and collision chains, but does not construct a certified
interval, spatial-cell index, candidate mark, or candidate scratch buffer for
an exact-rational point. If any coordinate is not exact rational, the existing
promotion path indexes every retained point and continues through the same
policy-aware general equality schedule.

This is a scalar-fact scheduling rule. It has no fixture, coordinate value,
triangle-count, operation, topology, expected-result, policy-name, benchmark,
or competitor branch. It adds no compatibility surface and no alternate
Boolean engine.

## Exactness and complete decline path

For exact-rational coordinates, equality is decided by exact rational value.
The first lookup uses the three retained `Rational::storage_identity` values;
a value fingerprint and exact collision chain handle equal values with
distinct storage and fingerprint collisions. No approximate bound is a
decision.

The first non-rational point promotes the interner exactly once:

1. reserve the candidate, interval, mark, cell, and unbucketed storage;
2. derive conservative certified intervals for every retained point;
3. register bounded intervals in the spatial cells and keep unindexable points
   in the complete unbucketed set; and
4. perform candidate equality through `DecisionContext` and Hyperlimit.

Uncertain intervals and unbucketed values therefore retain the full comparison
path. `STRICT` returns typed `PredicateUndecided` when exact equality remains
unresolved. `APPROXIMATE_512` can decide only through Hyperlimit's terminal and
updates the aggregate mesh certainty. Capacity failures remain typed, and the
structural construction-identity map still precedes numeric aliasing and
rejects contradictory materializations.

Tests prove that exact-first capacity allocates no general index, promotion
retains every exact prefix point, equal rationals with different storage still
alias, distinct rationals with the same binary64 key remain distinct, and an
exact-zero prefix followed by symbolic terminal equality preserves the policy
difference and certainty marker.

## Broad performance controls

Saved parent and current default-feature release executables ran on CPU 11
with three `perf stat` repetitions. Instructions and branches improve on every
exact workload:

| Fixture / workload | Parent instructions | Current instructions | Change | Branch change |
| --- | ---: | ---: | ---: | ---: |
| overlapping boxes, all four ×1000 | 4,355,684,489 | 4,016,548,370 | -7.786% | -10.011% |
| sparse multishell 512, all four ×5 | 3,185,472,797 | 2,770,857,207 | -13.016% | -15.885% |
| dense coplanar 32, all four ×1 | 10,485,189,178 | 10,303,958,112 | -1.728% | -2.289% |
| clipped voxel torus 33, all four ×3 | 2,806,713,518 | 2,666,940,145 | -4.980% | -6.667% |
| 2,049-bit rational boxes, union ×5 | 12,731,207,958 | 11,869,941,359 | -6.765% | -6.762% |
| full rotated YeahRight, intersection ×1 | 11,415,839,882 | 10,430,883,271 | -8.628% | -8.942% |

The improvement comes from not building and querying a general interval grid
when exact retained identities already provide a complete equality decision.
There is no change to candidate geometry, triangulation, topology, or winding.

Two general symbolic controls compare the saved parent and current executables
adjacently. Depth 1 `STRICT` remains `Certified`; depth 128
`APPROXIMATE_512` retains the reference output and
`Approximate512Consumed`:

| Symbolic workload | Parent/current instructions | Movement | Branch movement |
| --- | ---: | ---: | ---: |
| depth 1, all four ×20, `STRICT` | 4,286,531,940 / 4,288,945,198 | +0.0563% | -0.1437% |
| depth 128, all four ×5, `APPROXIMATE_512` | 1,505,486,453 / 1,505,496,548 | +0.0007% | -0.0182% |

The small fixed promotion cost is reported rather than hidden. It is retained
because the general path and terminal policy remain complete, branches improve,
and every broad exact workload wins materially.

## Large-fixture heap matrix

All fifteen selectors ran under both `STRICT` and `APPROXIMATE_512`. Every
policy pair is byte-identical and `Certified`. Every total peak is lower or
exactly unchanged; allocation calls and cumulative payload fall in all rows.

| Selector | Parent peak | Current peak | Parent/current allocations | Parent/current payload |
| --- | ---: | ---: | ---: | ---: |
| boxes-3072 | 12,383,968 | 12,383,968 | 132,839 / 31,262 | 22,539,318 / 17,557,542 |
| boxes-3072-general | 12,630,096 | 12,630,096 | 152,027 / 50,448 | 24,893,774 / 19,887,390 |
| dense-coplanar-16 | 15,227,749 | 14,893,071 | 381,397 / 283,087 | 71,883,810 / 66,660,634 |
| dense-coplanar-32 | 60,663,173 | 59,391,879 | 1,532,055 / 1,131,251 | 287,892,866 / 266,498,458 |
| sparse-shells-512 | 12,527,651 | 11,888,929 | 424,215 / 219,630 | 45,748,620 / 35,740,148 |
| self-PWN-clusters-512 | 12,657,681 | 12,284,773 | 424,712 / 221,632 | 46,260,318 / 36,336,670 |
| wide-rational-64 | 17,861,062 | 17,861,062 | 634,992 / 396,646 | 44,169,454 / 33,594,798 |
| wide-rational-512 | 18,750,079 | 18,750,079 | 1,626,080 / 1,307,460 | 122,524,942 / 105,881,886 |
| wide-rational-2048 | 27,423,760 | 27,423,760 | 1,898,144 / 1,579,524 | 424,635,358 / 386,287,470 |
| voxel-torus-33 | 12,784,184 | 12,784,184 | 151,941 / 41,003 | 29,703,148 / 24,189,972 |
| voxel-torus-65 | 51,784,828 | 51,784,828 | 588,128 / 152,672 | 131,364,351 / 109,622,215 |
| YeahRight | 5,371,657 | 5,161,051 | 211,854 / 168,979 | 13,364,677 / 11,588,357 |
| YeahRight ×4 | 20,120,061 | 19,676,989 | 892,738 / 734,974 | 52,886,138 / 46,308,650 |
| YeahRight ×8 | 76,258,109 | 76,258,109 | 3,946,341 / 3,342,288 | 215,353,468 / 190,485,420 |
| full rotated YeahRight | 84,482,432 | 63,243,888 | 11,738,039 / 9,972,500 | 669,521,621 / 593,227,133 |

On the full 23,788-triangle row, total peak falls 21,238,544 bytes (25.14%),
the incremental kernel peak falls by the same amount to 56,120,526 bytes,
allocation calls fall 1,765,539, reallocations fall 42,381, and cumulative
payload falls 76,294,488 bytes. Both policies produce the same exact empty
result with `Certified` certainty.

## Heaptrack ownership

The current strict capture is
`target/phase17-exact-first-arrangement-full-strict.zst`. Heaptrack reports a
63,317,616-byte peak versus 84,556,160 bytes at the parent, the same
21,238,544-byte reduction as the allocator probe. Calls to allocation
functions fall from 13,500,280 to 11,692,360.

At the new peak:

| Live owner / nested allocation stack | Parent | Current | Change |
| --- | ---: | ---: | ---: |
| exact source owner, reattributed | 31,298,480 | 31,732,864 | +434,384 |
| arrangement excluding source caches | 46,060,590 | 24,387,662 | -21,672,928 (-47.05%) |
| raw `corefine_surface` stack | 47,414,534 | 26,175,990 | -21,238,544 (-44.79%) |
| raw pairwise-intersection stack | 24,982,804 | 24,982,804 | unchanged |
| total peak | 84,556,160 | 63,317,616 | -21,238,544 (-25.12%) |

The source movement reflects a later peak moment with more demand-created edge
cycles live; it is not assigned to the arrangement win. The eliminated live
owner is the exact point arena's unnecessary general interval/cell index.
Current arrangement storage excluding source caches remains 24.39 MB and is
the next heap target, led by the retained point arena and face-corefinement
storage rather than a fixture-shaped shortcut.

## Historical and CGAL boundaries

Fresh full processes preserve the exact certified-empty result:

| Policy | Wall time | Maximum RSS |
| --- | ---: | ---: |
| `STRICT` | 1.06 s | 72,816 KiB |
| `APPROXIMATE_512` | 1.02 s | 72,844 KiB |

Strict RSS falls 26.07% from the parent checkpoint. Against historical EMBER,
the current strict row is about 3,125× faster and uses 77.9% less RSS.
Historical CGAL remains ahead at 0.09 seconds / 15,516 KiB: the current full
boundary remains about 11.78× slower and 4.69× larger.

The pinned CGAL 6.0.3 EPECK exact-OFF medians are unchanged. Hypermesh was
refreshed as 63 fresh CPU-11-pinned processes. Every output remains exact,
valid, closed, structurally valid, and topology-identical:

| Fixture | CGAL outside / inside | Hypermesh `STRICT` | Ratio | Hypermesh `APPROXIMATE_512` | Ratio |
| --- | ---: | ---: | ---: | ---: | ---: |
| crossing octahedra | 112,972 / 122,712 ns | 436,690 ns | 3.866× | 442,540 ns | 3.917× |
| affine boxes | 376,124 / 383,553 ns | 836,283 ns | 2.223× | 831,743 ns | 2.211× |

Relative to the parent Hypermesh executable, the four medians improve
4.27–7.45%. All CGAL losses remain open; no aggregate or historical gain is
called parity.

## Code and binary size

Production changes are the exact-first selection plus deferred candidate
reservation; most of the 55-added/2-removed diff is policy and promotion test
coverage. Default release native text shrinks 192 bytes, and all-feature
release native text shrinks 176 bytes. Optimized release WASM grows 115–259
bytes. Size-profile native/WASM movement is only +24–32/+28 bytes. The
performance and heap wins take priority over those bounded WASM increases.

The current five-crate graphs contain 15,092 nodes / 25,163 edges for
production, 17,412 / 28,479 with tests and examples, and 21,458 / 34,580 with
all tests, examples, benches, and fuzz targets. They contain zero exact
EMBER/local-BSP/segment-trace node. The arrangement constructor has one direct
edge to the existing `PointInterner::try_with_capacity`; promotion remains the
single complete general route and every consuming equality stays in the
canonical policy cascade. Hypercurve and HyperSolve are excluded and
untouched.

## Validation

The following pass:

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

The all-feature suite passes 196 tests with the six documented manual/opt-in
ignores: 149 unit, 8 Boolean, 11 competitive, 14 corpus-manifest, 3
intersection-corpus, 9 policy, and 2 README tests. The minimal library passes
148 tests.

Phases 17 and 18 remain open for the external real-world corpus, remaining
per-case CGAL gaps, further arrangement/corefinement lifetime reduction, and
the final removal/policy/caller audit.
