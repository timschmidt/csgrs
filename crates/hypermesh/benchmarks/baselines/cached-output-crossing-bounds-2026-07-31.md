# Hypermesh cached output crossing-bounds checkpoint

Date: 2026-07-31

Direct parent: `e6ded62dd74c04e26a215f32b3dac8c29e2797c6`

Implementation: `25332f79558b9d668577c87744a52f0508fcf460`

Dependencies:

- Hyperreal `d10a01f1b4a6ec202dff6c0fc2a726baa13cc841`
- Hyperlattice `53fbdf6fd35e08b9d696f1e2c3c5e742ea261a96`
- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri `c47601266e0b9b17d0c5a0764fa22b18168ada73`

This checkpoint reduces repeated conservative-bound reconstruction in the
complete output edge-crossing scan. It does not add a topology shortcut or a
second intersection algorithm.

## Retained design and invariant

For exact-rational output vertices, every coordinate already has a certified
outward binary64 enclosure. An edge's three axis bounds are only the endpoint
interval minima and maxima, so reconstructing them cannot strengthen or weaken
the proof. Disjoint outward intervals prove exact separation; overlap remains
inconclusive and reaches the existing exact bounds-overlap and exact
intersection predicates.

The retained implementation has two tiers:

- below 1,024 unique edges, compute the left edge's three bounds once per
  outer sweep iteration and construct right bounds lazily as axes are tested;
- at 1,024 or more edges, attempt one exact-capacity side vector containing
  all edge bounds and load both sides from it for every candidate pair; and
- if the optional allocation cannot be reserved, execute the allocation-free
  direct tier. Failure of a performance optimization is never an operation
  failure.

Symbolic, non-rational, non-finite, and unenclosable values retain the original
exact policy-aware ordering path. Approximate bounds only reject candidates
that are provably disjoint. Every survivor follows the same canonical exact
path under the operation's immutable policy.

Consequently:

- `STRICT` never consumes an approximate terminal decision;
- `APPROXIMATE_512` retains Hyperlimit's terminal 512-bit equality/sign
  interpretation and aggregate certainty reporting;
- the cache has no policy-dependent entries and is operation-local;
- allocation failure cannot change semantics;
- there is no pass, depth, or candidate cap; and
- no public API, retained mesh field, compatibility shim, or policy boundary
  was added.

The threshold regression builds 342 separated exact triangles (1,026 unique
edges), forces the cached tier, and checks both policies for no event, no
mutation, and `MeshCertainty::Certified`.

## Direct-parent CPU results

Release probes were serialized and pinned to CPU 9. Values are per operation
after division by the fixed repetition count. Parent and candidate produced
identical topology and certification on every row.

| Fixture / policy | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` parent | 56.15 ms | 208,009,455 | 544,214,659 | 92,507,648 | 851,780 | 1,393,606 |
| Retained / `STRICT` candidate | 55.60 ms (-0.980%) | 205,439,820 (-1.235%) | 527,310,143 (-3.106%) | 92,041,992 (-0.503%) | 856,725 (+0.581%) | 1,401,292 (+0.552%) |
| Retained / `APPROXIMATE_512` parent | 58.28 ms | 214,789,280 | 544,217,279 | 92,508,277 | 856,651 | 1,422,119 |
| Retained / `APPROXIMATE_512` candidate | 58.02 ms (-0.446%) | 211,734,207 (-1.422%) | 527,305,529 (-3.108%) | 92,040,931 (-0.505%) | 864,193 (+0.880%) | 1,445,319 (+1.631%) |
| Generated 13,452-t / `STRICT` parent | 77.51 ms | 268,775,435 | 598,924,583 | 90,167,105 | 753,730 | 1,763,879 |
| Generated 13,452-t / `STRICT` candidate | 77.02 ms (-0.632%) | 267,571,338 (-0.448%) | 597,754,930 (-0.195%) | 90,091,570 (-0.084%) | 755,463 (+0.230%) | 1,774,464 (+0.600%) |
| Generated 13,452-t / `APPROXIMATE_512` parent | 77.00 ms | 267,799,856 | 598,955,374 | 90,174,814 | 752,193 | 1,757,603 |
| Generated 13,452-t / `APPROXIMATE_512` candidate | 76.79 ms (-0.273%) | 267,126,732 (-0.251%) | 597,754,362 (-0.201%) | 90,091,438 (-0.092%) | 755,441 (+0.432%) | 1,789,887 (+1.837%) |
| 6,144-t boxes / `STRICT` parent | 5.89 ms | 14,402,507 | 35,384,235 | 6,485,997 | 65,708 | 115,119 |
| 6,144-t boxes / `STRICT` candidate | 5.87 ms (-0.340%) | 14,318,529 (-0.583%) | 35,162,102 (-0.628%) | 6,452,461 (-0.517%) | 65,752 (+0.067%) | 109,945 (-4.494%) |
| 6,144-t boxes / `APPROXIMATE_512` parent | 5.87 ms | 14,364,218 | 35,384,216 | 6,486,029 | 65,700 | 114,963 |
| 6,144-t boxes / `APPROXIMATE_512` candidate | 5.81 ms (-1.022%) | 14,182,526 (-1.265%) | 35,162,639 (-0.626%) | 6,452,543 (-0.516%) | 65,699 (-0.002%) | 108,922 (-5.255%) |

The generated strict pair was deliberately repeated in reverse order. Its
static work reduction repeated, while a forward-order bracket moved task clock
about +0.52% and cycles about +0.43%. Generated runtime is therefore reported
as order-noise/neutral; the repeatable result is approximately 0.20% fewer
instructions and 0.08--0.09% fewer branches. The retained and box rows show
larger work reductions and favorable task-clock/cycle movement.

Output topology is identical for parent, candidate, `STRICT`, and
`APPROXIMATE_512`:

- retained: 4,524 input triangles, 625 vertices / 1,246 triangles;
- generated: 13,452 input triangles, 154 / 304; and
- boxes: 6,144 input triangles, 27 / 50.

All outcomes report `MeshCertainty::Certified`; neither measured approximate
row needed Hyperlimit's terminal approximation.

## Profile movement

The retained fixture has no actual crossing event, making it a direct measure
of the complete negative broad-phase proof. A 30-operation, 1,999 Hz profile
lost zero samples.

| Symbol | Parent self | Candidate self |
| --- | ---: | ---: |
| `split_edge_crossing_events` | 6.21% | 5.31% |
| `memcmp` | 3.66% | 3.78% |
| signed product sum | 3.48% | 3.60% |
| mixed-width GCD | 3.11% | 3.93% |
| certified rational line f64 filter | 2.98% | 2.66% |

The owning crossing scan falls 14.49% in relative sample share and is no
longer the dominant retained-profile head. Mixed-width GCD, exact rational
product/comparison, and allocation now lead the next profile-driven audit.

## Large-fixture heap

Heaptrack includes fixture construction and the complete immediate union. The
recording summary's temporary-allocation count is authoritative; reconstructed
`heaptrack_print` classifications differ slightly.

| Fixture / revision | Allocations | Temporary allocations | Peak heap | Candidate Heaptrack RSS |
| --- | ---: | ---: | ---: | ---: |
| Retained parent | 520,148 | 27,637 | 12.69 MiB | - |
| Retained candidate, both policies | 520,149 (+1 / +0.000192%) | 27,637 (unchanged) | 12.71 MiB (+0.02 / +0.158%) | 22.44--22.50 MiB |
| Generated parent | 215,071 | 10,317 | 11.66 MiB | - |
| Generated candidate, both policies | 215,071 (unchanged) | 10,317 (unchanged) | 11.66 MiB | 23.49--23.55 MiB |
| Boxes parent | 27,211 | 79 | 4.70 MiB | - |
| Boxes candidate, both policies | 27,211 (unchanged) | 79 (unchanged) | 4.70 MiB | 11.07--11.72 MiB |

The retained row crosses the 1,024-edge threshold and pays exactly one side
vector allocation. Its roughly 20 KiB peak cost buys a 3.11% instruction and
1.24--1.42% cycle reduction. Generated and box outputs stay below the cache
threshold after earlier output stages, use the direct tier, and are heap and
allocation neutral. A discarded parallel Heaptrack attempt collided on
Heaptrack's shared FIFO; those traces are invalid and are not used here.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. The clean dependency-only
size harness compares the direct parent and committed candidate.

| Consumer | Profile | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General native | Release | 4,032,236 | 4,033,692 | +1,456 / +0.0361% |
| General WASM | Release | 2,705,627 | 2,706,805 | +1,178 / +0.0435% |
| Immediate native | Release | 4,065,836 | 4,067,292 | +1,456 / +0.0358% |
| Immediate WASM | Release | 2,720,665 | 2,721,844 | +1,179 / +0.0433% |
| General native | Size | 1,850,610 | 1,852,778 | +2,168 / +0.1172% |
| General WASM | Size | 1,147,727 | 1,148,885 | +1,158 / +0.1009% |
| Immediate native | Size | 1,863,102 | 1,865,270 | +2,168 / +0.1164% |
| Immediate WASM | Size | 1,158,689 | 1,159,847 | +1,158 / +0.0999% |

The runtime-priority choice grows linked code by at most 0.1172%. Production
source changes by +72/-11 lines (+61 net); the focused regression adds 25
lines. There is no public function, carrier field, or alternate semantic
spine.

The call-graph utility reports:

| Scope | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| Isolated Hypermesh | 7,982 nodes / 19,609 edges | 7,994 / 19,633 | +12 / +24 |
| Five Hyper crates | 19,591 nodes / 39,118 edges | 19,603 / 39,142 | +12 / +24 |

The utility is syntactic and includes the local selector, helpers, regression,
and closures. The production crossing family still has one canonical exact
resolution path.

## Historical and competitive controls

The frozen retained historical row was 944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and about 82.5 MiB maximum RSS. The current strict row
is directionally 94.12% faster, uses 81.24% less peak heap, and performs 89.64%
fewer allocations. Historical polygon topology differed, so these are trend
anchors rather than a correctness A/B.

The immediately preceding same-day competitive measurements were not rerun at
this source checkpoint. Competitors do not implement Hyperreal exactness,
Hyperlimit policy selection, or Hypermesh's certified-output contract.

| Union workload | Hypermesh | boolmesh | manifold-rust | Relative result |
| --- | ---: | ---: | ---: | --- |
| Overlapping 12-triangle boxes | 5.0998 us | 67.195 us | 60.512 us | Hypermesh 13.18x / 11.87x faster |
| 3,072-triangle boxes per operand | 1.8719 ms | 7.6133 ms | 4.4687 ms | Hypermesh 4.07x / 2.39x faster |
| Dyadic YeahRight hull + box | 7.8718 ms | 0.76936 ms | 0.84057 ms | boolmesh 10.23x / manifold-rust 9.36x faster |

The projective row remains the principal competitive gap.

## Full-resolution gate

Exact CGAL EPECK reports a valid empty zero-vertex/zero-face intersection for
the 11,894-by-11,894 rotated fixture. A conservative intermediate 512-bit
candidate completed Hypermesh's operation as the same certified empty result
in 3,357.09 seconds with 319.07 MiB maximum RSS. The user stopped the redundant
approximately 56-minute final-source rerun. It is not restarted here, and the
intermediate timing is not relabeled as a `25332f79` measurement.

## Rejected experiments

- Eagerly constructing all right-edge bounds before short-circuiting raised
  retained instructions from about 544.22 million to 549.06 million.
- Hoisting only the left bounds used no allocation and reached about 536.01
  million retained instructions, but the full side cache reached about
  527.43--527.85 million and approximately 0.98% fewer cycles in the long A/B.
- Always caching small generated outputs was runtime-neutral and added an
  allocation, motivating the measured 1,024-edge threshold.
- An in-place `match` shortening added about 619,000 retained instructions.
- A reference enum without `Clone, Copy` added about 754,000 retained
  instructions.
- An alternate `if`/`map` small-path selector added about 91,000 retained and
  81,000 generated instructions.

Every rejected source form was fully removed. Performance takes priority over
source brevity, while the retained form keeps the linked-size increase near
one tenth of one percent.

## Validation

The committed source passes:

- focused cached-tier and output regressions under both policies;
- default and no-default Hypermesh matrices (1,053 unit tests plus integration,
  policy, regression, and doctest surfaces);
- all features (1,054 unit tests plus integrations);
- warning-denied all-target Clippy for all features and no default features;
- warning-denied documentation, formatting, every fuzz binary check, and
  benchmark compilation; and
- identical certified retained, generated, and box output probes under
  `STRICT` and `APPROXIMATE_512`.

```text
cargo test --locked --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo test --locked --all-features --no-fail-fast
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo clippy --locked --all-targets --no-default-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo fmt --all -- --check
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run
```
