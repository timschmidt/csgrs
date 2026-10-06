# Hypermesh dynamic Lehmer-entry checkpoint

Date: 2026-07-31

Direct Hypermesh parent:
`0819dc8e2e941719845822056d548209fa22263e`

Implementations:

- Hyperreal parent `e2316e038939cd71a949e84e033d6b7ff60f9db2`
- Hyperreal candidate `e365a0153836393954d85a9e8988d9539b2068b0`
- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypermesh `0819dc8e2e941719845822056d548209fa22263e`

The Hypermesh production implementation is unchanged in this checkpoint. Its
ignored full-resolution oracle is corrected from an assumed nonempty result to
the exact certified-empty result established below.

## Outcome

Hyperreal's wide-magnitude GCD selected Lehmer reduction only from the initial
operand widths. An initially unbalanced pair therefore stayed in full-width
Euclidean division even when its first ordinary remainder produced a balanced
wide pair. That path dominates exact rational normalization in the retained
and full-resolution Hypermesh profiles.

The selected reducer now performs the same first exact remainder that the
Euclidean loop would perform, then reassesses the resulting pair. It enters
Lehmer reduction only when the smaller remainder is still at least the retained
192-bit crossover and the two widths differ by no more than one bit. Otherwise
it continues through the existing Euclidean loop.

This eight-line production change improves every measured Hypermesh CPU row,
reduces allocations on both crossing-heavy large meshes, and leaves their peak
heap unchanged. It adds no carrier, allocation, compatibility path, or public
API. The full-resolution 11,894-by-11,894 rotated intersection also completed
with a 319.07 MiB maximum RSS on a conservative 512-bit-entry version of the
same change, versus the historical approximately 116 GiB failure.

## Exactness and policy contract

Let the initially selected wide magnitudes be `larger >= smaller`. When both
are above the Lehmer crossover but their bit widths are too far apart, the
first loop transition is exactly

```text
(larger, smaller) -> (smaller, larger mod smaller)
```

The candidate performs that transition once before selecting the subsequent
reducer. No quotient, remainder, or sign is approximated. If the remainder is
below the crossover or the new pair is still unbalanced, execution resumes in
the unchanged Euclidean loop. If it is wide and balanced, the existing Lehmer
matrix loop performs exact quotient-preserving reductions with its unchanged
fallbacks.

Consequently this is a representation-level exact arithmetic optimization:

- `STRICT` topology consumes no approximate decision;
- `APPROXIMATE_512` retains the same terminal 512-bit predicate behavior;
- the GCD result is independent of predicate policy;
- all rational values remain canonical; and
- all measured mesh outputs are certified and identical between policies.

Randomized equivalence covers initially unbalanced transitions at 191, 192,
256, 512, 1,024, and 4,096 bits against `num::Integer::gcd`. Dispatch tracing
proves that a below-threshold unbalanced pair remains Euclidean and that a
320-bit transition enters Lehmer after the first remainder.

## Scalar crossover evidence

Criterion rows compare the selected reducer with forced full-width Euclidean
reduction on the same initially unbalanced operands. The retained crossover is
192 bits.

| Balanced remainder width | Selected | Forced Euclidean | Selected result |
| ---: | ---: | ---: | ---: |
| 192 bits | 8.086 us | 8.538 us | 5.29% faster / 1.06x |
| 256 bits | 8.338 us | 13.967 us | 40.30% faster / 1.68x |
| 512 bits | 12.705 us | 31.514 us | 59.68% faster / 2.48x |
| 1,024 bits | 25.140 us | 72.899 us | 65.51% faster / 2.90x |
| 4,096 bits | 132.787 us | 514.854 us | 74.21% faster / 3.88x |

The pre-change selected 4,096-bit row was 515.815 us, so the committed dynamic
entry improves that selected path by approximately 74.26%.

## Direct-parent Hypermesh CPU results

The certified-rational orientation checkpoint and this candidate were pinned
to CPU 9 and run serially. Retained and generated rows use 61 repetitions; the
box control uses 201. Each candidate cell includes movement from the direct
parent.

| Fixture / policy | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` | 53.49 ms (-4.04%) | 201,323,486 (-1.24%) | 544,040,004 (-1.671%) | 92,472,137 (-1.785%) | 851,848 (-1.95%) | 1,374,654 (-1.27%) |
| Retained / `APPROXIMATE_512` | 52.69 ms (-5.56%) | 198,540,146 (-2.82%) | 544,050,573 (-1.668%) | 92,474,736 (-1.781%) | 845,724 (-2.41%) | 1,363,900 (-3.23%) |
| Generated 13,452-t / `STRICT` | 73.77 ms (-2.73%) | 263,712,974 (-1.35%) | 598,560,760 (-0.114%) | 90,089,724 (-0.151%) | 743,261 (-3.79%) | 1,906,609 (-0.56%) |
| Generated 13,452-t / `APPROXIMATE_512` | 73.15 ms (-2.09%) | 262,046,810 (-0.64%) | 598,533,620 (-0.116%) | 90,082,931 (-0.155%) | 742,469 (-3.41%) | 1,892,555 (-0.06%) |
| 6,144-t boxes / `STRICT` | 5.81 ms (-1.36%) | 14,222,913 (-1.53%) | 35,383,123 (-0.002%) | 6,485,323 (-0.005%) | 64,929 (-0.91%) | 106,278 (-10.63%) |
| 6,144-t boxes / `APPROXIMATE_512` | 5.78 ms (-2.03%) | 14,186,274 (-0.91%) | 35,382,952 (-0.003%) | 6,485,309 (-0.007%) | 64,930 (-0.97%) | 107,508 (-9.21%) |

Output topology is identical for parent, candidate, `STRICT`, and
`APPROXIMATE_512`:

- retained: 4,524 input triangles, 625 vertices / 1,246 triangles;
- generated: 13,452 input triangles, 154 / 304; and
- boxes: 6,144 input triangles, 27 / 50.

All six outcomes report `MeshCertainty::Certified`; no approximate-512
terminal was consumed.

## Large-fixture heap

Heaptrack includes fixture construction and the complete immediate union.
Allocation and temporary counts come from the recording summary; peak heap
comes from `heaptrack_print`. Candidate counts match between policies.

| Fixture / revision | Allocations | Temporary | Peak heap | Candidate Heaptrack RSS |
| --- | ---: | ---: | ---: | ---: |
| Retained direct parent | 540,315 | 30,081 | 12.69 MiB | - |
| Retained candidate | 521,086 (-3.559%) | 27,181 (-9.641%) | 12.69 MiB | 22.00-22.25 MiB |
| Generated direct parent | 215,813 | 10,382 | 11.66 MiB | - |
| Generated candidate | 215,113 (-0.324%) | 10,300 (-0.790%) | 11.66 MiB | 23.26-23.33 MiB |
| Boxes direct parent | 27,211 | 79 | 4.70 MiB | - |
| Boxes candidate | 27,211 (unchanged) | 79 (unchanged) | 4.70 MiB | 12.82-13.18 MiB |

The optimization reduces transient exact-normalization work. It does not add
retained storage, and the large-fixture live-heap ceilings are unchanged.

## Full-resolution memory gate and corrected oracle

The ignored hard test intersects the 11,894-triangle YeahRight mesh with a
rotated copy. The old test assumed a nonempty result. Three independent
oracles returned an empty intersection; the authoritative exact check used
CGAL's EPECK kernel and `corefine_and_compute_intersection`:

```text
valid=1 vertices=0 faces=0
wall=0.09 s maximum_rss=15516 KiB
```

The Hypermesh test now requires a valid `MeshCertainty::Certified` result with
zero vertices and zero triangles. It remains ignored because it is a roughly
56-minute manual memory-ceiling test, not because the result or memory behavior
is unknown.

A conservative intermediate candidate that entered dynamic Lehmer only at a
512-bit balanced remainder completed that corrected operation under
`APPROXIMATE_512` without consuming its terminal approximation:

| Measure | Result |
| --- | ---: |
| Test harness | 3,356.02 s |
| Wall time | 55:57.09 / 3,357.09 s |
| User / system time | 3,342.89 s / 2.85 s |
| Maximum RSS | 326,724 KiB / 319.07 MiB |
| Major / minor page faults | 30 / 1,509,813 |
| Swaps | 0 |
| Outcome | Certified, 0 vertices / 0 triangles |

The pre-change operation had completed in 5,148.08 s before failing only the
incorrect nonempty assertion. The conservative candidate is 34.81% faster by
harness time. Its maximum RSS is approximately 99.73% below the historical
116 GiB failure.

The committed selector keeps identical behavior for balanced remainders at
512 bits and above and extends the measured-safe transition down to the
existing 192-bit crossover. Scalar crossover, dispatch, randomized backend
equivalence, both-policy large-mesh CPU/heap, and the full validation matrix
all exercise that final source. A redundant final-source hard rerun was
stopped by the user after a terminal/session failure lost the output of an
earlier redundant attempt. This document therefore attributes the 55:57 and
319.07 MiB numbers only to the conservative 512-bit-entry candidate; it does
not relabel them as a direct timing of the final 192-bit selector.

## Historical and competitive controls

The frozen retained historical row was 944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and 82.5 MiB maximum RSS. The current strict candidate
is directionally 94.34% faster, retains 81.27% less peak heap, and performs
89.62% fewer allocations. Historical polygon output differed, so this remains
a directional regression anchor rather than a direct correctness A/B.

Fresh Criterion slope estimates were pinned to CPU 9. Competitors are
throughput references and do not provide Hypermesh's exact `Real`, explicit
terminal policy, or certified-output contract.

| Workload | Hypermesh | boolmesh | manifold-rust | Relative result |
| --- | ---: | ---: | ---: | --- |
| Overlapping 12-triangle boxes, union | 5.0998 us | 67.195 us | 60.512 us | Hypermesh 13.18x / 11.87x faster |
| 3,072-triangle boxes per operand, union | 1.8719 ms | 7.6133 ms | 4.4687 ms | Hypermesh 4.07x / 2.39x faster |
| 3,072-triangle mesh import | 406.91 us | 909.54 us | 1.1466 ms | Hypermesh 2.24x / 2.82x faster |

The same-day projective control was not selected by this fresh default
competitive invocation, so its immediately preceding stored slope is retained
without claiming a rerun: Hypermesh 7.8718 ms, boolmesh 0.76936 ms, and
manifold-rust 0.84057 ms. The non-exact competitors remain 10.23x and 9.36x
faster on that exact-rational workload.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. Clean default-feature
consumers compare the certified-rational orientation checkpoint with this
candidate.

| Consumer | Profile | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General native | Release | 4,031,604 | 4,032,340 | +736 / +0.0183% |
| General WASM | Release | 2,705,155 | 2,705,575 | +420 / +0.0155% |
| Immediate native | Release | 4,065,204 | 4,065,940 | +736 / +0.0181% |
| Immediate WASM | Release | 2,720,193 | 2,720,613 | +420 / +0.0154% |
| General native | Size | 1,850,554 | 1,850,730 | +176 / +0.0095% |
| General WASM | Size | 1,147,695 | 1,147,862 | +167 / +0.0146% |
| Immediate native | Size | 1,863,046 | 1,863,238 | +192 / +0.0103% |
| Immediate WASM | Size | 1,158,660 | 1,158,824 | +164 / +0.0142% |

The largest linked-code increase is 0.0183%. Performance has priority, and
that small increase is retained for consistent CPU and allocation reductions.

## Source and call graph

Hyperreal production changes by +8/-2 lines. The commit also adds the scalar
benchmark matrix, randomized crossover coverage, and dispatch assertions. No
new production function, carrier, or compatibility shim is introduced.

| Scope | Direct parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| Isolated Hypermesh | 7,982 nodes / 19,609 edges | 7,982 / 19,609 | unchanged |
| Five Hyper crates | 19,580 nodes / 39,101 edges | 19,582 / 39,110 | +2 / +9 |

The graph utility is syntactic and includes the added tests and closures. The
production path remains the single canonical rational normalization reducer.

## Rejected alternative

The recursive half-GCD candidate remains benchmark-only. Fresh pinned rows
show that enabling it would make the large exact-normalization head worse:

| Width | Half-GCD candidate | Retained Lehmer | Candidate slowdown |
| ---: | ---: | ---: | ---: |
| 16,384 bits | 3.206 ms | 935.547 us | 3.43x |
| 65,536 bits | 24.918 ms | 9.509 ms | 2.62x |
| 262,144 bits | 358.124 ms | 140.251 ms | 2.55x |

No half-GCD production code was enabled.

## Validation

Hyperreal passes its default, no-default, and all-feature matrices (554, 554,
and 631 unit tests plus integrations and doctests), warning-denied full/minimal
Clippy, warning-denied documentation, formatting, every fuzz binary check, and
benchmark compilation. Focused nightly AddressSanitizer passes the randomized
dynamic-entry equivalence test with leak detection disabled for the sandbox.

Hypermesh passes its default, no-default, and all-feature matrices (1,052,
1,052, and 1,053 unit tests plus integrations and policy suites),
warning-denied full/minimal Clippy, warning-denied documentation, formatting,
every fuzz binary check, and benchmark compilation. Hyperlattice, Hyperlimit,
and Hypertri pass their default, no-default, and all-feature test matrices.
Every direct mesh probe returns identical certified topology under `STRICT`
and `APPROXIMATE_512`.

```text
# each of hyperreal, hyperlattice, hyperlimit, hypertri, and hypermesh
cargo test --locked --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo test --locked --all-features --no-fail-fast

# hyperreal and hypermesh lint/build surfaces
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo clippy --locked --all-targets --no-default-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo fmt --all -- --check
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run

# focused Hyperreal sanitizer
CARGO_TARGET_DIR=/tmp/hyperreal-dynamic-lehmer-asan \
  RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
  cargo +nightly test --locked --target x86_64-unknown-linux-gnu \
  gcd_wide_magnitudes_dynamic_lehmer_matches_backend --lib

# competitive and manual full-resolution surfaces
taskset -c 9 cargo bench --locked --bench competitive
YEAHRIGHT_BENCH=1 cargo test --locked --release --test competitive \
  full_resolution_yeahright_rotated_intersection_certifies_empty \
  -- --ignored --exact --test-threads=1
```
