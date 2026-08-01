# Hyperreal exact shifted-magnitude prefix checkpoint

Date: 2026-07-31

Hyperreal direct parent: `fdb981f0bd44bf597160d01386990752e7783c17`

Hyperreal implementation: `dc36d4d5dd1a299448536dced41fd044239b5e75`

Hypermesh measurement source: `25332f79558b9d668577c87744a52f0508fcf460`

Hypermesh evidence parent: `27e3d7b7601790646a2f32a84c74250b24602170`

Other dependencies:

- Hyperlattice `53fbdf6fd35e08b9d696f1e2c3c5e742ea261a96`
- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri `c47601266e0b9b17d0c5a0764fa22b18168ada73`

This checkpoint adds an exact leading-word decision before Hyperreal's complete
allocation-free comparison of shifted BigUint magnitudes. Hypermesh source is
unchanged; only its canonical Hyperreal scalar dependency differs in the
direct A/B.

## Retained design and exact invariant

`compare_shifted_biguints` compares `left << left_shift` with
`right << right_shift` without materializing either shifted integer. The
retained implementation first compares total bit widths, exactly as before.
When they match, it cancels the common low shift and then:

- directly compares the unshifted BigUints when both remaining shifts are
  zero;
- extracts the first most-significant `u64` word of each shifted magnitude
  from borrowed limbs;
- returns only when those aligned words differ; and
- recreates the existing complete most-significant-first iterator scan when
  the leading words are equal.

For a non-word-aligned shift, the leading word is either the high carry or the
exact composition of the highest two source limbs. Whole-word shifts append
only low zero words and therefore do not alter this leading word. Because the
total shifted bit widths are equal before the prefix is examined, the two
words occupy the same absolute bit positions. A mismatch is consequently the
same exact decision the old iterator made on its first iteration. Equality is
not inferred from the prefix: the complete borrowed scan still decides it.

All callers supply nonzero rational magnitudes. The existing generic iterator
continues to cover every sub-word, word-aligned, multiword, equality, and deep
prefix case. The implementation adds no allocation, BigUint clone, retained
cache, public API, scalar field, recursion, compatibility shim, or arbitrary
count/depth limit.

This exact scalar path remains below Hyperlimit's policy boundary:

- `STRICT` still accepts certified or exact decisions only;
- `APPROXIMATE_512` still uses Hyperlimit's terminal 512-bit equality/sign
  interpretation only after the canonical certified path is exhausted;
- no approximation or policy state enters Hyperreal; and
- all inconclusive prefix cases execute the same complete exact comparison as
  the parent.

The focused regression compares against materialized BigUint shifts for 5,000
deterministically generated operand/shift combinations through 512-bit source
magnitudes and 384-bit shifts. Exact equality is checked in both directions
for every sub-word shift 1 through 63 and across the 64/65, 127/128, and 129
boundaries.

## Workload characterization

Temporary trace labels were fully removed after characterization. On the
retained 4,524-triangle mesh, `compare_shifted_biguints` is called 150,616
times:

| Stage | Calls |
| --- | ---: |
| Unequal total bit width, immediate exact decision | 13,741 |
| Equal total bit width | 136,875 |
| First word differs | 131,089 |
| Second word differs | 5,716 |
| Complete value equal | 70 |
| Nonzero sub-word relative shift | 134,543 |
| Zero relative shift | 2,332 |

Thus 95.77% of equal-width calls decide on the first word and 99.95% decide by
the second. The generated 13,452-triangle fixture has 29,837 calls: 1,604
bit-width decisions, 26,070 first-word decisions, 477 equal prefixes, and
1,686 zero-relative-shift comparisons. The box control has only 456 calls.

The ordinary rational-comparison dispatch labels, exact BigUint fallbacks, and
both-policy outputs are identical between parent and candidate. The new
prefix changes work within one already-selected exact path; it does not
reclassify a predicate or consume a policy terminal.

## Direct scalar control

Criterion compares two retained, non-dyadic 1,024-bit-scale rationals whose
dyadic denominators differ by five bits:

| Revision / run | Estimate | 95% interval |
| --- | ---: | ---: |
| Parent | 16.611 ns | 16.493--16.748 ns |
| Candidate run 1 | 15.146 ns | 15.101--15.196 ns |
| Candidate run 2 | 15.074 ns | 15.044--15.108 ns |

The candidate improves the central estimate by 8.82--9.25%. The permanent
benchmark ledger records 15.13 ns with a 15.07--15.20 ns interval. Parent and
candidate use the same benchmark source, clean production revisions, pinned
CPU 9, and the same toolchain.

## Retained-process mesh CPU results

The fixture is constructed once and the exact Boolean is repeated in one
process, matching the retained-workload model and eliminating per-process
startup/hash-seed noise. Release probes are serialized and pinned to CPU 9.
The retained strict and box strict rows aggregate two reverse-order brackets;
all values below are per operation.

| Fixture / policy | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` parent | 41.597 ms | 175,988,507 | 489,335,266 | 85,283,530 | 752,620 | 1,254,922 |
| Retained / `STRICT` candidate | 41.872 ms (+0.661%) | 176,883,561 (+0.509%) | 484,853,339 (-0.916%) | 84,793,520 (-0.575%) | 744,727 (-1.049%) | 1,259,638 (+0.376%) |
| Retained / `APPROXIMATE_512` parent | 42.683 ms | 180,598,579 | 489,334,141 | 85,283,081 | 758,271 | 1,286,457 |
| Retained / `APPROXIMATE_512` candidate | 41.752 ms (-2.182%) | 176,816,308 (-2.094%) | 484,846,349 (-0.917%) | 84,792,179 (-0.576%) | 748,701 (-1.262%) | 1,262,419 (-1.869%) |
| Generated 13,452-t / `STRICT` parent | 13.631 ms | 57,099,111 | 153,816,127 | 25,794,612 | 247,928 | 473,776 |
| Generated 13,452-t / `STRICT` candidate | 13.533 ms (-0.719%) | 56,694,705 (-0.708%) | 152,806,360 (-0.656%) | 25,674,940 (-0.464%) | 249,763 (+0.740%) | 466,628 (-1.509%) |
| Generated 13,452-t / `APPROXIMATE_512` parent | 13.575 ms | 56,905,963 | 153,818,876 | 25,795,192 | 251,932 | 471,274 |
| Generated 13,452-t / `APPROXIMATE_512` candidate | 13.619 ms (+0.323%) | 57,127,750 (+0.390%) | 152,804,298 (-0.660%) | 25,674,687 (-0.467%) | 250,533 (-0.555%) | 464,859 (-1.361%) |
| 6,144-t boxes / `STRICT` parent | 1.830 ms | 7,713,381 | 19,093,569 | 3,255,580 | 23,079 | 81,455 |
| 6,144-t boxes / `STRICT` candidate | 1.848 ms (+0.998%) | 7,784,567 (+0.923%) | 19,082,397 (-0.059%) | 3,254,200 (-0.042%) | 22,788 (-1.262%) | 80,112 (-1.649%) |
| 6,144-t boxes / `APPROXIMATE_512` parent | 1.836 ms | 7,734,030 | 19,093,001 | 3,255,428 | 23,348 | 84,617 |
| 6,144-t boxes / `APPROXIMATE_512` candidate | 1.803 ms (-1.820%) | 7,588,576 (-1.881%) | 19,090,857 (-0.011%) | 3,255,053 (-0.011%) | 23,427 (+0.337%) | 83,530 (-1.285%) |

The policy-independent static signal repeats on both policy rows: retained
instructions fall about 0.916%, generated instructions 0.656--0.660%, and box
instructions 0.011--0.059%. Reverse-order wall-clock brackets straddle layout
noise, while the approximate retained row and generated strict row are also
favorable in task clock and cycles. No runtime gain is claimed from a mixed
timing row; the isolated scalar and repeated static counters decide retention.

Output topology is identical for parent, candidate, `STRICT`, and
`APPROXIMATE_512`:

- retained: 4,524 input triangles, 625 vertices / 1,246 triangles;
- generated: 13,452 input triangles, 154 / 304; and
- boxes: 6,144 input triangles, 27 / 50.

Every result reports `MeshCertainty::Certified`; no approximate row consumes
Hyperlimit's terminal interpretation.

## Profile attribution

The parent retained profile contains 3,243 samples and the candidate 2,583;
both cover 30 operations at 1,999 Hz and lose zero samples.

| Symbol family | Parent self | Candidate self |
| --- | ---: | ---: |
| `compare_shifted_biguints` | 2.35% | 2.01% |
| `split_edge_crossing_events` | 5.61% | 5.35% |
| Mixed-width GCD owner | 4.52% | 4.67% |
| Signed product sum | 3.31% | 3.79% |
| `Rational::partial_cmp` | 3.32% | 3.15% |
| `gcd_word` | 3.02% | 3.15% |
| Certified rational line filter | 3.02% | 2.52% |
| BigUint shift | 2.13% | 2.60% |

The targeted head falls 14.47% in relative share. Sample redistribution makes
cross-symbol percentages descriptive rather than an A/B counter; the direct
benchmark and exact mesh instruction totals are the deciding evidence. The
refreshed profile puts crossing resolution, mixed-width GCD, signed-product
scheduling, general rational comparison, word GCD, and allocation next in the
audit queue.

## Large-fixture heap

Heaptrack covers fixture construction and one complete immediate union. Its
recorder temporary-allocation count is authoritative.

| Fixture / candidate policies | Allocations | Temporary allocations | Peak heap | Heaptrack RSS |
| --- | ---: | ---: | ---: | ---: |
| Retained, both policies | 520,149 | 27,637 | 12.71 MiB | 22.62--22.63 MiB |
| Generated, both policies | 215,071 | 10,317 | 11.66 MiB | 23.55--23.58 MiB |
| Boxes, both policies | 27,211 | 79 | 4.70 MiB | 12.02--13.09 MiB |

Every allocation, temporary-allocation, and peak-heap count exactly matches
the direct parent checkpoint and is identical between policies. Heaptrack's
reconstructed temporary classification reports 27,763 / 10,359 / 81; those
values are not substituted for the recorder summaries.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. The same Hypermesh source
is linked once to the parent Hyperreal and once to the candidate.

| Consumer | Profile | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General native | Release | 4,034,628 | 4,035,028 | +400 / +0.0099% |
| General WASM | Release | 2,707,221 | 2,707,924 | +703 / +0.0260% |
| Immediate native | Release | 4,068,228 | 4,068,628 | +400 / +0.0098% |
| Immediate WASM | Release | 2,722,260 | 2,722,963 | +703 / +0.0258% |
| General native | Size | 1,853,122 | 1,853,362 | +240 / +0.0130% |
| General WASM | Size | 1,149,402 | 1,149,614 | +212 / +0.0184% |
| Immediate native | Size | 1,865,614 | 1,865,862 | +248 / +0.0133% |
| Immediate WASM | Size | 1,160,372 | 1,160,583 | +211 / +0.0182% |

Runtime is the higher priority and every artifact grows by at most 0.0260%.
Production source changes by +30/-2 lines with no new function; the permanent
benchmark/docs add 25 lines and the expanded regression changes +12/-1.

The call-graph utility reports:

| Scope | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| Isolated Hypermesh | 7,994 nodes / 19,633 edges | 7,994 / 19,633 | unchanged |
| Five Hyper crates | 19,612 nodes / 39,161 edges | 19,614 / 39,166 | +2 / +5 |

The two syntactic nodes are the local leading-word closure and permanent
benchmark closure. There is no new public function, comparison family, or
policy spine.

## Historical and competitive controls

The frozen retained historical row was 944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and about 82.5 MiB maximum RSS. The preceding one-shot
strict row was 51.61 ms; this checkpoint further reduces repeated exact work
without changing heap. The current stack therefore remains directionally
about 94.54% faster, uses 81.24% less peak heap, and performs 89.64% fewer
allocations than the historical anchor. Historical topology and timing setup
differ, so this remains a trend rather than a correctness A/B.

The immediately preceding same-day competitive measurements were not rerun
for a scalar-only change. Competitors do not implement Hyperreal exactness,
Hyperlimit policy selection, or Hypermesh's certified-output contract.

| Union workload | Hypermesh | boolmesh | manifold-rust | Relative result |
| --- | ---: | ---: | ---: | --- |
| Overlapping 12-triangle boxes | 5.0998 us | 67.195 us | 60.512 us | Hypermesh 13.18x / 11.87x faster |
| 3,072-triangle boxes per operand | 1.8719 ms | 7.6133 ms | 4.4687 ms | Hypermesh 4.07x / 2.39x faster |
| Dyadic YeahRight hull + box | 7.8718 ms | 0.76936 ms | 0.84057 ms | boolmesh 10.23x / manifold-rust 9.36x faster |

The projective row remains the principal competitive gap.

## Full-resolution gate

The established full-resolution oracle remains exact CGAL EPECK's valid empty
zero-vertex/zero-face intersection for the 11,894-by-11,894 rotated fixture. A
prior conservative intermediate 512-bit candidate completed Hypermesh's
operation as the same certified empty result in 3,357.09 seconds with 319.07
MiB maximum RSS. This scalar comparison checkpoint does not relabel or replace
that source-specific result.

## Rejected experiments

- A dedicated complete sub-word-shift scanner regressed the direct scalar row
  from 16.611 ns to 17.005 ns (+2.37%).
- A normalized-MSB prefix regressed it to 19.134 ns (+15.19%).
- Explicitly unrolling the generic iterator's first call reached only 16.506
  ns, materially worse than the retained 15.07--15.15 ns prefix.
- A greater-than-256-bit gate added dispatch/layout work and regressed all
  exploratory mesh controls.
- A fixed-`u128` tier and direction/width-gated prefix variants lost the
  retained/generated/box Pareto comparison and were removed.

Temporary dispatch labels, the retained-process repetition hook, and every
rejected source variant were removed. An initial Heaptrack run that attached
to an `env` launcher instead of the mesh binary was discarded and repeated
correctly; only the direct-recorder counts above are evidence.

## Validation

The committed source passes:

- Hyperreal default and no-default matrices (557 unit tests plus integrations
  and doctests) and all features (634 unit tests plus integrations);
- the 5,000-case materialized-shift oracle, all 63 sub-word equality shifts,
  word/multiword boundary cases, and the full rational arithmetic surface;
- warning-denied Hyperreal all-target Clippy under all and no-default features,
  warning-denied docs, formatting, every fuzz binary check, and benchmark
  compilation;
- Hyperlattice, Hyperlimit, and Hypertri default, no-default, and all-feature
  matrices;
- Hypermesh default and no-default matrices (1,053 unit tests plus integration,
  policy, regression, and doctest surfaces) and all features (1,054 unit tests
  plus integrations); and
- warning-denied Hypermesh all-target Clippy under all and no-default features,
  warning-denied docs, formatting, every fuzz target check, and benchmark
  compilation.

Representative commands:

```text
cargo test --locked --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo test --locked --all-features --no-fail-fast
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo clippy --locked --all-targets --no-default-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo fmt --all -- --check
cargo check --locked --manifest-path fuzz/Cargo.toml --all-targets
cargo bench --locked --no-run
```
