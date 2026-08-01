# Hyperreal exact leading-significand comparison checkpoint

Date: 2026-07-31

Hyperreal direct parent: `35c57c14d737e6efc1b4fe325dc4818e9a483dd3`

Hyperreal implementation: `fdb981f0bd44bf597160d01386990752e7783c17`

Hypermesh measurement source: `25332f79558b9d668577c87744a52f0508fcf460`

Hypermesh evidence parent: `be2690f148894e260f8c001ef054a3d5dec85054`

Other dependencies:

- Hyperlattice `53fbdf6fd35e08b9d696f1e2c3c5e742ea261a96`
- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri `c47601266e0b9b17d0c5a0764fa22b18168ada73`

This checkpoint replaces floating normalized-rational comparison with exact
cross-products of conservative 53-bit leading-significand intervals. Hypermesh
source is unchanged from the cached crossing-bounds checkpoint; only its
canonical Hyperreal scalar dependency differs in the direct A/B.

## Retained design and exact invariant

For each positive BigUint magnitude, the helper returns lower and upper
leading-significand integers over the common denominator `2^52`, plus the exact
bit count:

- magnitudes of at most 53 bits are shifted into an exact zero-width interval;
- wider magnitudes retain the highest 53 bits as the lower endpoint and use the
  next integer as the outward upper endpoint; and
- zero declines the filter and retains the existing exact fallback.

When two positive rationals have the same exact most-significant-bit exponent,
their normalization shift is necessarily zero or one. The comparison forms
outward rational bounds with exact `u128` cross-products. The largest possible
product, including the one-bit scale shift, is below 108 bits. Therefore no
product can overflow and no rounding occurs.

The filter returns `Less` only when the left upper bound is strictly below the
right lower bound, and returns `Greater` only for the symmetric disjoint case.
Overlapping intervals still reach the unchanged arbitrary-width BigUint cross
product. The optimization consequently cannot manufacture equality, reverse an
order, or remove an exact fallback. The same integer-significand helper also
feeds the public finite binary64 enclosure path; its final divisions retain the
existing outward `next_down` / `next_up` rounding.

The implementation adds no allocation, retained cache, scalar field, public
API, compatibility shim, recursive path, or arbitrary count/depth limit.

This work is below Hyperlimit's policy boundary:

- `STRICT` still accepts certified or exact decisions only;
- `APPROXIMATE_512` still terminates at Hyperlimit's explicit 512-bit
  equality/sign interpretation only after the canonical certified path is
  exhausted;
- no approximate decision is stored in Hyperreal or reused as exact; and
- every inconclusive significand interval reaches the same exact or selected
  policy-terminal path as its parent.

Focused tests check both comparison directions at 129, 257, 521, and 1,025
bits. A deterministic generated oracle checks 10,000 BigUint enclosures from 1
through 1,024 bits and 5,000 rational comparisons against complete BigUint
cross-products, requiring at least 1,000 certified interval decisions. Existing
signed, leading-bit, and binary64-enclosure corpora also pass.

## Exact comparison dispatch

Temporary trace instrumentation characterized the retained 4,524-triangle
exact mesh and was fully removed. Parent and candidate produced identical
counts over 133,962 exact rational-comparison calls:

| Path | Calls |
| --- | ---: |
| Leading-significand interval | 36,801 |
| Dyadic borrowed-digit comparison | 28,967 |
| Magnitude-bit decision | 18,459 |
| Word-sized cross-product | 1,923 |
| Arbitrary-width BigUint cross-product | 324 |
| Pointer, sign, common-denominator, or other early exit | 47,488 |

The candidate neither gains interval certifications by weakening their proof
nor displaces any exact fallback. The 36,801 retained calls are the changed
arithmetic; the 28,967 dyadic calls identify the next comparison audit target.

## Direct scalar control

Criterion compares two already-retained, non-dyadic 1,024-bit rationals whose
order is certified by the leading interval:

| Revision / run | Estimate | 95% interval |
| --- | ---: | ---: |
| Parent | 61.389 ns | 60.217--62.732 ns |
| Candidate run 1 | 48.758 ns | 48.338--49.220 ns |
| Candidate run 2 | 50.577 ns | 49.841--51.430 ns |

The repeated candidate improves the central estimate by 17.61--20.58%. The
permanent benchmark ledger records 48.90 ns with a 48.50--49.36 ns interval.
Parent and candidate were built from clean worktrees using the same toolchain
and benchmark definition.

## Direct-parent mesh CPU results

Release probes were serialized and pinned to CPU 9. The retained strict row
contains 201 operations. The retained approximate row aggregates two
reverse-order 201-operation brackets to reduce layout/order bias. Generated
rows contain 101 operations and box rows 401. Values are per operation.

| Fixture / policy | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` parent | 52.107 ms | 194,641,555 | 522,797,679 | 91,602,549 | 849,835 | 1,340,938 |
| Retained / `STRICT` candidate | 51.608 ms (-0.958%) | 192,656,208 (-1.020%) | 518,598,188 (-0.803%) | 91,089,337 (-0.560%) | 850,556 (+0.085%) | 1,336,650 (-0.320%) |
| Retained / `APPROXIMATE_512` parent | 52.624 ms | 196,029,770 | 522,799,083 | 91,602,949 | 848,511 | 1,348,438 |
| Retained / `APPROXIMATE_512` candidate | 52.250 ms (-0.710%) | 194,633,985 (-0.712%) | 518,593,302 (-0.804%) | 91,088,236 (-0.562%) | 852,413 (+0.460%) | 1,352,761 (+0.321%) |
| Generated 13,452-t / `STRICT` parent | 73.809 ms | 261,392,999 | 597,162,778 | 90,050,940 | 755,002 | 1,792,023 |
| Generated 13,452-t / `STRICT` candidate | 74.036 ms (+0.307%) | 262,129,455 (+0.282%) | 594,944,571 (-0.371%) | 89,996,887 (-0.060%) | 760,190 (+0.687%) | 1,856,307 (+3.587%) |
| Generated 13,452-t / `APPROXIMATE_512` parent | 74.528 ms | 262,838,296 | 597,176,373 | 90,054,200 | 754,774 | 1,803,786 |
| Generated 13,452-t / `APPROXIMATE_512` candidate | 74.115 ms (-0.553%) | 262,222,654 (-0.234%) | 594,992,520 (-0.366%) | 90,008,680 (-0.051%) | 755,478 (+0.093%) | 1,853,595 (+2.761%) |
| 6,144-t boxes / `STRICT` parent | 5.996 ms | 14,138,369 | 35,225,142 | 6,467,632 | 66,828 | 116,154 |
| 6,144-t boxes / `STRICT` candidate | 5.963 ms (-0.561%) | 14,087,515 (-0.360%) | 35,193,769 (-0.089%) | 6,467,143 (-0.008%) | 66,059 (-1.151%) | 109,560 (-5.677%) |
| 6,144-t boxes / `APPROXIMATE_512` parent | 6.037 ms | 14,243,884 | 35,226,986 | 6,468,181 | 66,826 | 116,198 |
| 6,144-t boxes / `APPROXIMATE_512` candidate | 5.994 ms (-0.714%) | 14,129,607 (-0.802%) | 35,195,812 (-0.088%) | 6,467,734 (-0.007%) | 66,120 (-1.057%) | 109,519 (-5.748%) |

The retained fixture repeats a policy-independent 0.803--0.804% instruction
reduction and 0.560--0.562% branch reduction. Its reverse-order approximate
brackets individually straddle wall-clock noise, while their aggregate and the
strict row are favorable. The generated strict cache/layout counters are
mixed, but both generated policies repeat a 0.366--0.371% instruction
reduction. The boxes are a low-comparison-share control and still execute about
0.089% fewer instructions. Static work and the isolated scalar result are the
retention evidence; incidental branch-miss/cache placement is not claimed as
an algorithmic change.

Output topology is identical for parent, candidate, `STRICT`, and
`APPROXIMATE_512`:

- retained: 4,524 input triangles, 625 vertices / 1,246 triangles;
- generated: 13,452 input triangles, 154 / 304; and
- boxes: 6,144 input triangles, 27 / 50.

Every result reports `MeshCertainty::Certified`; no measured approximate row
consumes Hyperlimit's terminal interpretation.

## Profile attribution

The parent retained profile contains 3,979 samples and the candidate 3,243;
both use 30 operations at 1,999 Hz and lose zero samples.

| Symbol family | Parent self | Candidate self |
| --- | ---: | ---: |
| Normalized rational interval / exact significand comparison | 3.29% | 0.67% |
| `split_edge_crossing_events` | 5.37% | 5.61% |
| Mixed-width GCD owner | 4.50% | 4.52% |
| `Rational::partial_cmp` | 2.21% | 3.32% |
| Signed product sum | 4.08% | 3.31% |
| Certified rational line filter | 2.83% | 3.02% |
| `gcd_word` | 2.88% | 3.02% |
| `compare_shifted_biguints` | below reported head | 2.35% |
| Allocation (`malloc` / `_int_malloc`) | 2.97% | 2.69% / 2.54% |

The changed normalized helper's relative profile share falls 79.64% and its
floating interval implementation disappears as a head. Attribution moves into
the owning `partial_cmp`, so the direct scalar row and static mesh counters are
the deciding evidence. The refreshed profile elevates exact dyadic shifted
comparison, signed product scheduling, mixed-width GCD, and crossing resolution
for the next audits.

## Large-fixture heap

Heaptrack covers fixture construction and complete immediate union. Its
recorder temporary-allocation count is authoritative. A direct parent rerun of
the generated fixture confirms that one-count startup variation in an earlier
recording was not candidate work.

| Fixture / revision and policy | Allocations | Temporary allocations | Peak heap | Heaptrack RSS |
| --- | ---: | ---: | ---: | ---: |
| Retained parent and candidate, both policies | 520,149 | 27,637 | 12.71 MiB | candidate 22.56--22.57 MiB |
| Generated parent, both policies | 215,071 | 10,317 | 11.66 MiB | 23.54--23.68 MiB |
| Generated candidate, both policies | 215,071 | 10,317 | 11.66 MiB | 23.47--23.70 MiB |
| Boxes parent and candidate, both policies | 27,211 | 79 | 4.70 MiB | candidate 12.87--13.29 MiB |

The candidate is allocation-, temporary-allocation-, and peak-heap-neutral on
every large fixture and under both policies. This matches the stack-only
integer interval implementation.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. The same Hypermesh source
is linked once to the parent Hyperreal and once to the candidate.

| Consumer | Profile | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General native | Release | 4,033,852 | 4,034,628 | +776 / +0.0192% |
| General WASM | Release | 2,707,085 | 2,707,221 | +136 / +0.0050% |
| Immediate native | Release | 4,067,452 | 4,068,228 | +776 / +0.0191% |
| Immediate WASM | Release | 2,722,124 | 2,722,260 | +136 / +0.0050% |
| General native | Size | 1,852,682 | 1,853,122 | +440 / +0.0237% |
| General WASM | Size | 1,148,925 | 1,149,402 | +477 / +0.0415% |
| Immediate native | Size | 1,865,174 | 1,865,614 | +440 / +0.0236% |
| Immediate WASM | Size | 1,159,893 | 1,160,372 | +479 / +0.0413% |

Runtime is the user's higher priority, and every artifact grows by less than
0.042%. Production source changes by +33/-14 lines with no new production
function; the benchmark/docs add 24 lines and focused regressions add 95.

The call-graph utility reports:

| Scope | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| Isolated Hypermesh | 7,994 nodes / 19,633 edges | 7,994 / 19,633 | unchanged |
| Five Hyper crates | 19,604 nodes / 39,141 edges | 19,612 / 39,161 | +8 / +20 |

The syntactic five-crate movement comes from focused tests, their nested
generator, and benchmark closures. The production helper is renamed rather
than duplicated, and there is no second comparison or policy spine.

## Historical and competitive controls

The frozen retained historical row was 944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and about 82.5 MiB maximum RSS. The current strict row
is directionally 94.54% faster, uses 81.24% less peak heap, and performs 89.64%
fewer allocations. Historical polygon topology differed, so these are trend
anchors rather than a correctness A/B.

The immediately preceding same-day competitive measurements were not rerun for
a scalar-only change. Competitors do not implement Hyperreal exactness,
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

## Validation

The committed source passes:

- Hyperreal default and no-default matrices (557 unit tests plus integrations
  and doctests) and all features (634 unit tests plus integrations);
- focused exact-leading-significand, 10,000-case BigUint interval, 5,000-case
  rational cross-product, leading-bit, and binary64-enclosure regressions;
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
