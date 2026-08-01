# Hyperreal fixed-width binary GCD checkpoint

Date: 2026-07-31

Hyperreal direct parent: `d10a01f1b4a6ec202dff6c0fc2a726baa13cc841`

Hyperreal implementation: `35c57c14d737e6efc1b4fe325dc4818e9a483dd3`

Hypermesh measurement source: `25332f79558b9d668577c87744a52f0508fcf460`

Hypermesh evidence parent: `7aec2888fff60e7a1274f2cfb1e18d2e26e29cba`

Other dependencies:

- Hyperlattice `53fbdf6fd35e08b9d696f1e2c3c5e742ea261a96`
- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypertri `c47601266e0b9b17d0c5a0764fa22b18168ada73`

This checkpoint reduces work in Hyperreal's allocation-free 256- and 512-bit
binary-GCD tiers. Hypermesh source is unchanged from the preceding cached
crossing-bounds checkpoint; only its canonical scalar dependency differs in
the direct A/B.

## Retained design and invariant

The fixed-width reducer copies eligible positive magnitudes into four or eight
stack limbs. The retained implementation:

- computes both trailing-zero counts once and normalizes both operands before
  entering the subtraction loop;
- compares high limbs with one bounded three-way scan instead of a reversed
  iterator comparison that lowered to an external `memcmp` head;
- exits immediately when the normalized operands are equal;
- uses the proven strict ordering to make every following subtraction
  positive, eliminating a redundant full-array zero scan; and
- normalizes that positive difference at the bottom of the loop.

The common power of two is still restored exactly. When both operands become
at most 128 bits, the existing word reducer completes the same integer GCD.
Operands outside the fixed tiers still reach the existing identity,
power-of-two, word, remainder, or dynamic-Lehmer paths. The implementation
adds no allocation, public function, retained field, recursion, semantic
fallback, or compatibility shim.

This is scalar arithmetic below the policy boundary. Therefore:

- `STRICT` continues to accept exact decisions only;
- `APPROXIMATE_512` continues to use Hyperlimit's terminal 512-bit
  equality/sign interpretation only when the canonical exact path cannot
  decide within policy;
- no approximate result is cached as exact, and no policy state is inferred
  from a scalar; and
- every mesh candidate, pass, and exact fallback remains unbounded by an
  arbitrary count or depth limit.

The focused regression checks equal shifted values in both fixed tiers, which
exercises the new equality exit and common-shift restoration. The existing
20,000-case randomized BigUint-reference corpus continues to cover differing
widths, zero patterns, shared shifts, and fixed-tier boundaries.

## Exact workload dispatch

Temporary trace instrumentation was used only to characterize the retained
4,524-triangle exact mesh and was then removed. Its 11,522 GCD operations
dispatch as follows:

| Path | Calls | Share |
| --- | ---: | ---: |
| Fixed 256-bit binary GCD | 1,204 | 10.45% |
| Fixed 512-bit binary GCD | 3,017 | 26.18% |
| Fixed word GCD | 290 | 2.52% |
| Equal wide operands | 159 | 1.38% |
| Wide Euclidean remainder | 1,792 | 15.55% |
| Wide Euclidean to `u128` | 1,512 | 13.12% |
| Wide Euclidean to `u64` | 222 | 1.93% |
| Wide identity | 2,140 | 18.57% |
| Dynamic Lehmer | 223 | 1.94% |
| Power of two | 963 | 8.36% |

The maximum observed operand was 954 bits. The retained change targets 4,221
calls (36.63%) without displacing the necessary wider tiers.

## Direct scalar control

Criterion's cold, coprime 512-bit reduction benchmark isolates the changed
tier:

| Revision | Median | 95% interval |
| --- | ---: | ---: |
| Parent | 5.1563 us | 5.1034--5.2132 us |
| Candidate | 3.5605 us | 3.5467--3.5751 us |

The candidate is 30.9485% faster. Parent and candidate were built from clean
worktrees with the same toolchain and benchmark definition.

## Direct-parent mesh CPU results

Release probes were serialized and pinned to CPU 9. Retained rows contain 201
operations, generated rows 101, and box rows 401. Values below are per
operation. Parent and candidate produce identical topology and certification
on every row.

| Fixture / policy | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` parent | 54.720 ms | 202,451,419 | 527,410,806 | 92,067,650 | 858,966 | 1,381,991 |
| Retained / `STRICT` candidate | 52.725 ms (-3.647%) | 195,117,903 (-3.622%) | 522,916,546 (-0.852%) | 91,619,791 (-0.486%) | 852,491 (-0.754%) | 1,363,941 (-1.306%) |
| Retained / `APPROXIMATE_512` parent | 56.539 ms | 207,900,534 | 527,416,258 | 92,069,138 | 861,526 | 1,409,468 |
| Retained / `APPROXIMATE_512` candidate | 55.285 ms (-2.218%) | 202,121,568 (-2.780%) | 522,918,761 (-0.853%) | 91,620,430 (-0.487%) | 859,972 (-0.180%) | 1,406,042 (-0.243%) |
| Generated 13,452-t / `STRICT` parent | 75.586 ms | 263,858,398 | 597,893,148 | 90,127,655 | 756,553 | 1,795,540 |
| Generated 13,452-t / `STRICT` candidate | 75.202 ms (-0.508%) | 263,002,421 (-0.324%) | 597,322,438 (-0.095%) | 90,080,953 (-0.052%) | 761,504 (+0.654%) | 1,798,606 (+0.171%) |
| Generated 13,452-t / `APPROXIMATE_512` parent | 78.884 ms | 274,485,073 | 597,893,350 | 90,127,614 | 766,849 | 1,882,593 |
| Generated 13,452-t / `APPROXIMATE_512` candidate | 77.026 ms (-2.355%) | 268,413,025 (-2.212%) | 597,304,695 (-0.098%) | 90,076,633 (-0.057%) | 762,184 (-0.608%) | 1,847,082 (-1.886%) |
| 6,144-t boxes / `STRICT` parent | 6.104 ms | 14,210,999 | 35,228,405 | 6,468,477 | 66,135 | 115,628 |
| 6,144-t boxes / `STRICT` candidate | 6.057 ms (-0.777%) | 14,239,231 (+0.199%) | 35,228,350 (-0.0002%) | 6,468,461 (-0.0003%) | 66,171 (+0.054%) | 118,402 (+2.399%) |
| 6,144-t boxes / `APPROXIMATE_512` parent | 6.100 ms | 14,214,310 | 35,230,484 | 6,469,079 | 66,084 | 117,203 |
| 6,144-t boxes / `APPROXIMATE_512` candidate | 6.022 ms (-1.266%) | 14,152,732 (-0.433%) | 35,230,661 (+0.0005%) | 6,469,081 (+0.00003%) | 66,073 (-0.015%) | 117,202 (-0.0005%) |

The retained workload shows the clearest integrated effect: approximately
0.85% less exact work and 2.8--3.6% fewer cycles. The generated workload has a
smaller fixed-tier share but repeats a 0.095--0.098% instruction reduction.
The box workload's instructions and branches are static to five significant
digits, so its mixed wall-clock/cycle/cache movement is an untouched-layout
noise control rather than a claimed algorithmic gain.

Output topology is identical for parent, candidate, `STRICT`, and
`APPROXIMATE_512`:

- retained: 4,524 input triangles, 625 vertices / 1,246 triangles;
- generated: 13,452 input triangles, 154 / 304; and
- boxes: 6,144 input triangles, 27 / 50.

All outcomes report `MeshCertainty::Certified`; neither measured approximate
row consumes Hyperlimit's terminal approximation.

## Profile attribution

The parent retained profile contains 4,087 samples and the candidate 3,979;
both use 30 operations at 1,999 Hz and lose zero samples.

| Symbol family | Parent self | Candidate self |
| --- | ---: | ---: |
| External `memcmp` | 4.30% | absent as a profile head |
| `split_edge_crossing_events` | 4.09% | 5.37% |
| Mixed-width GCD owner | 2.96% | 4.50% |
| Signed product sum | 3.39% | 4.08% |
| Normalized rational interval | 2.52% | 3.29% |
| `gcd_word` | 2.43% | 2.88% |
| Certified rational line filter | 2.78% | 2.83% |
| `partial_cmp` | 3.19% | 2.21% |

The old iterator comparison lowered to an externally attributed `memcmp`.
The replacement scan is charged to its owning GCD, so relative self shares do
not measure its improvement directly. The isolated scalar benchmark and the
static mesh instruction counters are the deciding evidence. The new profile
places crossing resolution, signed products, rational interval construction,
word GCD, and exact comparison next in the audit queue.

## Large-fixture heap

Heaptrack includes fixture construction and the complete immediate union. The
recording summary's temporary-allocation count is authoritative.

| Fixture / revision | Allocations | Temporary allocations | Peak heap | Candidate Heaptrack RSS |
| --- | ---: | ---: | ---: | ---: |
| Retained parent | 520,149 | 27,637 | 12.71 MiB | - |
| Retained candidate, both policies | 520,149 | 27,637 | 12.71 MiB | 22.56--22.59 MiB |
| Generated parent | 215,070 | 10,316 | 11.66 MiB | - |
| Generated candidate, both policies | 215,070 | 10,316 | 11.66 MiB | 23.58--23.64 MiB |
| Boxes parent | 27,211 | 79 | 4.70 MiB | - |
| Boxes candidate, both policies | 27,211 | 79 | 4.70 MiB | 13.00--13.16 MiB |

The candidate is allocation- and peak-heap-neutral on all three large
fixtures. This matches the implementation: fixed operands remain stack arrays
and no new BigUint or mesh carrier is retained. Heaptrack's reconstructed
classification reports 27,763 / 10,358 / 81 temporary allocations; those are
not substituted for the recorder summaries above.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. The harness compares the
same Hypermesh source linked once to the parent Hyperreal and once to the
candidate.

| Consumer | Profile | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General native | Release | 4,033,692 | 4,033,852 | +160 / +0.0040% |
| General WASM | Release | 2,706,805 | 2,707,085 | +280 / +0.0103% |
| Immediate native | Release | 4,067,292 | 4,067,452 | +160 / +0.0039% |
| Immediate WASM | Release | 2,721,844 | 2,722,124 | +280 / +0.0103% |
| General native | Size | 1,852,778 | 1,852,682 | -96 / -0.0052% |
| General WASM | Size | 1,148,885 | 1,148,925 | +40 / +0.0035% |
| Immediate native | Size | 1,865,270 | 1,865,174 | -96 / -0.0051% |
| Immediate WASM | Size | 1,159,847 | 1,159,893 | +46 / +0.0040% |

The runtime-priority choice grows the largest linked artifact by 0.0104%; two
size-profile native consumers shrink. Production source changes by +20/-7
lines and the focused regression adds 11 lines.

The call-graph utility reports:

| Scope | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| Isolated Hypermesh | 7,994 nodes / 19,633 edges | 7,994 / 19,633 | unchanged |
| Five Hyper crates | 19,603 nodes / 39,142 edges | 19,604 / 39,141 | +1 / -1 |

The utility is syntactic; the one-node movement is an implementation-shape
artifact. There is no new production or public function and no second
arithmetic or mesh-resolution spine.

## Historical and competitive controls

The frozen retained historical row was 944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and about 82.5 MiB maximum RSS. The current strict row
is directionally 94.42% faster, uses 81.24% less peak heap, and performs 89.64%
fewer allocations. Historical polygon topology differed, so these remain
trend anchors rather than a correctness A/B.

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
zero-vertex/zero-face intersection for the 11,894-by-11,894 rotated fixture.
A prior conservative intermediate 512-bit candidate completed Hypermesh's
operation as the same certified empty result in 3,357.09 seconds with 319.07
MiB maximum RSS. This scalar-loop checkpoint does not relabel or replace that
source-specific measurement.

## Rejected experiments

- Bypassing the fixed-512 tier for wide Lehmer raised retained instructions
  from about 527.29 million to 585.41 million (+11.0%).
- Special-casing the initial denominator in a shared LCM helper removed
  branches locally but added about 0.018% instructions and slowed the long
  retained pair by about 0.9%.
- A shorter fixed-array shift loop added about 3.6 million instructions and 2
  million branches.
- Fusing the `u128` fit scan into the limb comparison added about 6 million
  instructions and 2 million branches.
- A Boolean-state manual comparison added about 596,000 instructions relative
  to the retained three-way `Ordering` form.

All rejected source and trace instrumentation was removed. These controls are
why the retained form favors the small linked-size cost: it is the only tested
variant that improves the direct scalar and integrated exact-mesh work.

## Validation

The committed source passes:

- Hyperreal default and no-default matrices (555 unit tests plus integrations
  and doctests) and all features (632 unit tests plus integrations);
- the focused 136-test rational arithmetic surface, including 20,000
  randomized fixed-GCD reference cases;
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
