# Hypermesh certified-rational orientation reuse checkpoint

Date: 2026-07-31

Direct Hypermesh parent:
`57ed1f283e1d6d168cb94d6e53199d67e689e7e9`

Implementations:

- Hyperreal `e2316e038939cd71a949e84e033d6b7ff60f9db2`
- Hyperlimit `3e5d8816cd32bba46f48e0c6c13ab7a9da227784`
- Hypermesh `e2a682e2867b241d82eac724a416ef166e550f66`

## Outcome

The adaptive checkpoint's retained profile was still dominated by arbitrary
precision division, shifts, and repeated lossy conversion below projected
orientation and output-crossing cleanup. This checkpoint adds one certified
filter layer at each owning abstraction instead of weakening the exact path:

1. Hyperlimit tries Hyperreal's certified exact-rational line-sign filter after
   its exact dyadic and bounded-word kernels but before materializing the
   arbitrary-width rational determinant. Triangle-degeneracy classification
   shares the same projection filter.
2. Hypermesh constructs one `RationalLine2Filter` for each projected edge and
   reuses it for both query endpoints. An inconclusive result falls through to
   the existing exact rational signed-product sum.
3. Hyperreal can construct `RationalPoint3Query` from already-certified
   outward binary64 enclosures. Hypermesh therefore reuses the enclosure array
   that scheduled the output sweep instead of converting the same four exact
   vertices again for every projection.

The final direct-parent retained result executes 45.60% fewer instructions,
41.06% fewer cycles, and 56.70% fewer allocations. The generated 13,452-input-
triangle control executes 10.22% fewer instructions and 28.77% fewer
allocations. The 6,144-triangle box control executes 1.57% fewer instructions
with allocation counts unchanged. No retained carrier or heap allocation was
added.

## Exactness and policy contract

`RationalPoint3Query::from_certified_enclosures` accepts three outward
`[lower, upper]` pairs. It stores `lower` plus `next_up(upper - lower)` as a
conservative absolute error. For an exact coordinate `x`, the caller's
enclosure proves `lower <= x <= upper`, hence `|x - lower|` is no greater than
the stored error. Invalid, inverted, non-finite, overflowing, or otherwise
unrepresentable intervals return `None`.

The existing line filter propagates those errors through outward-rounded
determinant bounds and returns a sign only when zero is excluded. It returns
`None` on a boundary or unsafe arithmetic. Hypermesh treats `Some(None)` as
evidence that conversion was already attempted, but still executes the exact
arbitrary-width rational determinant. Symbolic and out-of-range coordinates
continue through the existing policy-aware `Real` fallback.

Consequently:

- `STRICT` never consumes a terminal approximation;
- `APPROXIMATE_512` retains the same terminal 512-bit boundary and can consume
  it only after all certified and exact stages fail;
- every filter success is mathematically exact and reports
  `Certainty::Exact`/`Escalation::Exact` in Hyperlimit;
- exact-collinear rational inputs deliberately fall through and return exact
  zero;
- projection order and topology are unchanged; and
- every measured output is certified and identical between policies.

Focused coverage includes wide rationals beyond the i128 word kernel, their
exact-collinear boundary, three-dimensional degeneracy projections, proper
crossing/same-side/endpoint-touch topologies, invalid enclosure inputs, and
20,000 deterministic randomized rational triples. Every randomized sign
returned by the enclosure filter matched the exact determinant; more than half
of the corpus certified in the filter. Existing symbolic, out-of-range,
binary64-collapse, closure, and both-policy regressions remain green.

## Layer-by-layer evidence

Each layer was measured before the next was added:

| Layer | Retained instructions | Generated instructions | Box instructions |
| --- | ---: | ---: | ---: |
| Hyperlimit rational filter | about -36.3% | about -7.3% | about +0.12% |
| Reuse two line filters per edge pair | a further -4.90% | -1.05% | -0.57% |
| Reuse four certified point enclosures | a further -10.16% | -2.10% | -1.12% |

The final symbolized retained profile reports
`split_edge_crossing_events` at 7.14% self,
`certified_rational_line2_sign_f64` at 2.51%, and `to_f64_lossy` at 1.27%.
No scalar conversion remains a dominant isolated head; the remaining work is
distributed across exact rational comparison/reduction, output processing,
and allocation.

## Direct-parent CPU results

The committed adaptive binary and final candidate were pinned to CPU 8 and
run serially. Retained and generated rows use 61 repetitions; the smaller box
control uses 201. Each cell is `parent -> candidate (movement)`.

| Fixture / policy | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Retained / `STRICT` | 88.52 -> 55.74 ms (-37.03%) | 345,851,665 -> 203,850,621 (-41.06%) | 1,017,125,451 -> 553,285,929 (-45.60%) | 173,582,114 -> 94,152,789 (-45.76%) | 1,406,260 -> 868,831 (-38.22%) | 1,448,164 -> 1,392,362 (-3.85%) |
| Retained / `APPROXIMATE_512` | 90.01 -> 55.79 ms (-38.02%) | 346,542,778 -> 204,310,629 (-41.04%) | 1,017,123,531 -> 553,279,647 (-45.60%) | 173,581,617 -> 94,151,291 (-45.76%) | 1,406,629 -> 866,642 (-38.39%) | 1,467,294 -> 1,409,445 (-3.94%) |
| Generated 13,452-t / `STRICT` | 82.09 -> 75.84 ms (-7.61%) | 290,626,107 -> 267,333,367 (-8.01%) | 667,448,760 -> 599,241,541 (-10.22%) | 101,833,731 -> 90,226,341 (-11.40%) | 861,879 -> 772,510 (-10.37%) | 1,871,120 -> 1,917,423 (+2.47%) |
| Generated 13,452-t / `APPROXIMATE_512` | 80.77 -> 74.71 ms (-7.50%) | 288,299,844 -> 263,741,837 (-8.52%) | 667,420,479 -> 599,226,965 (-10.22%) | 101,826,680 -> 90,223,024 (-11.40%) | 859,748 -> 768,685 (-10.59%) | 1,848,973 -> 1,893,622 (+2.41%) |
| 6,144-t boxes / `STRICT` | 5.86 -> 5.89 ms (+0.51%) | 14,368,658 -> 14,443,459 (+0.52%) | 35,947,241 -> 35,383,685 (-1.57%) | 6,578,675 -> 6,485,675 (-1.41%) | 65,472 -> 65,524 (+0.08%) | 106,715 -> 118,919 (+11.44%) |
| 6,144-t boxes / `APPROXIMATE_512` | 5.95 -> 5.90 ms (-0.84%) | 14,441,000 -> 14,315,983 (-0.87%) | 35,947,224 -> 35,383,880 (-1.57%) | 6,578,707 -> 6,485,746 (-1.41%) | 65,612 -> 65,569 (-0.07%) | 106,761 -> 118,416 (+10.92%) |

The box cache-miss increase is roughly twelve thousand events and does not
produce a repeatable task-clock regression: strict moves +0.03 ms while
approximate moves -0.05 ms. Instructions and branches improve under both
policies. Performance is the primary optimization objective, so the large and
projective gains outweigh this small linked-layout control movement.

Output topology is identical for parent, candidate, `STRICT`, and
`APPROXIMATE_512`:

- retained: 4,524 input triangles, 625 vertices / 1,246 triangles;
- generated: 13,452 input triangles, 154 / 304; and
- boxes: 6,144 input triangles, 27 / 50.

## Large-fixture heap

Heaptrack includes fixture construction and the complete immediate union. The
table uses recording summaries for allocation/temporary counts and
`heaptrack_print` for peak heap. Candidate counts are identical between
policies.

| Fixture / revision | Allocations | Temporary | Peak heap | Candidate Heaptrack RSS |
| --- | ---: | ---: | ---: | ---: |
| Retained parent | 1,247,977 | 172,658 | 12.70 MiB | - |
| Retained candidate | 540,315 (-56.70%) | 30,081 (-82.58%) | 12.69 MiB | 22.51-22.52 MiB |
| Generated parent | 303,000 | 27,004 | 11.66 MiB | - |
| Generated candidate | 215,813 (-28.77%) | 10,382 (-61.55%) | 11.66 MiB | 23.49-23.61 MiB |
| Boxes parent | 27,211 | 79 | 4.70 MiB | - |
| Boxes candidate | 27,211 | 79 | 4.70 MiB | 11.86-12.67 MiB |

The four point-query carriers and two line filters are stack values. The
allocation reductions come from avoiding repeated arbitrary-width rational
fallback temporaries; peak live storage is unchanged.

## Historical and competitive controls

The frozen historical retained row was 944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and 82.5 MiB maximum RSS. The current strict candidate
is 94.10% faster, retains 81.27% less peak heap, and performs 89.24% fewer
allocations. Historical polygon output differed, so this comparison remains
directional rather than a direct correctness A/B.

Fresh Criterion slope estimates were pinned to CPU 8. Competitors are
throughput references and do not provide Hypermesh's exact `Real`, explicit
terminal policy, or certified-output contract.

| Union workload | Hypermesh | boolmesh | manifold-rust | Relative result |
| --- | ---: | ---: | ---: | --- |
| Overlapping 12-triangle boxes | 5.1454 us | 66.916 us | 60.048 us | Hypermesh 13.00x / 11.67x faster |
| 3,072-triangle boxes per operand | 1.8934 ms | 7.5805 ms | 4.5331 ms | Hypermesh 4.00x / 2.39x faster |
| Dyadic YeahRight 840-triangle hull + box | 7.8718 ms | 0.76936 ms | 0.84057 ms | boolmesh 10.23x and manifold-rust 9.36x faster |

Against the adaptive checkpoint's stored Hypermesh slope, the small row is
2.53% slower across runs, the large row is 4.62% faster, and the projective row
is 40.65% faster. The paired counter probes above are the stronger incremental
signal. The exact projective gap remains, but it is substantially narrower.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. Clean default-feature
consumers compare the adaptive checkpoint with this candidate.

| Consumer | Profile | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General native | Release | 4,021,580 | 4,031,604 | +0.2493% |
| General WASM | Release | 2,697,828 | 2,705,155 | +0.2716% |
| Immediate native | Release | 4,055,196 | 4,065,204 | +0.2468% |
| Immediate WASM | Release | 2,712,803 | 2,720,193 | +0.2724% |
| General native | Size | 1,846,370 | 1,850,554 | +0.2266% |
| General WASM | Size | 1,144,901 | 1,147,695 | +0.2440% |
| Immediate native | Size | 1,858,862 | 1,863,046 | +0.2251% |
| Immediate WASM | Size | 1,155,863 | 1,158,660 | +0.2420% |

The largest linked-code increase is 0.2724%. It is retained for the 45.60%
retained and 10.22% generated instruction reductions. The production change
adds no heap allocation or persistent data field.

## Source and call graph

| Crate | Production | Tests/coverage | Total |
| --- | ---: | ---: | ---: |
| Hyperreal | +29 | +82 | +111 |
| Hyperlimit | +33 | +81 | +114 |
| Hypermesh | +98/-10 | +83/-3 | +181/-13 |

The production increase is +160/-10 across the three owning layers. Most
remaining source is focused proof and path coverage.

| Scope | Parent | Candidate | Movement |
| --- | ---: | ---: | ---: |
| Isolated Hypermesh | 7,963 nodes / 19,581 edges | 7,982 / 19,609 | +19 / +28 |
| Five Hyper crates | 19,535 nodes / 39,024 edges | 19,580 / 39,101 | +45 / +77 |

The graph utility is syntactic and includes tests and closures. Production
ownership is one hidden Hyperreal query constructor, one private Hyperlimit
filter stage, and private Hypermesh reuse plumbing; there is no compatibility
shim or second topology implementation.

## Rejected alternative

Extracting the four projected-side calls into a shared helper reduced source
repetition but increased the retained quick-profile count from about 553.28
million to 554.67 million instructions (+0.25%). Because performance has
priority, the helper was fully removed and the explicit optimized calls were
restored. No code from that source-extraction experiment remains.

Hyperreal's recursive half-GCD candidate was also remeasured because the
full-resolution live profile is dominated by arbitrary-width normalization.
At 16,384, 65,536, and 262,144 bits it took about 3.2062 ms, 24.918 ms, and
358.12 ms, versus 0.94402 ms, 9.5085 ms, and 140.25 ms for the retained
Lehmer reducer. The candidate is therefore 2.55-3.40x slower and remains a
benchmark-only rejected alternative.

## Validation

The five-crate default, no-default, and all-feature test suites pass, as do
warning-denied all-feature and no-default Clippy, warning-denied documentation,
formatting, every fuzz-manifest binary check, and benchmark compilation. The
new Hyperreal public constructor is explicitly classified as a proof-carrier
API with no like-for-like GMP operation. Hyperlattice's benchmark-only
allocator dependency was compiled with `CCACHE_DISABLE=1` because the sandbox
does not permit the host cache directory.

Nightly AddressSanitizer also passes the randomized Hyperreal enclosure test,
both wide/boundary Hyperlimit filter tests, and the Hypermesh crossing-topology
test. Leak detection is disabled because LeakSanitizer cannot terminate under
the workspace's ptrace-based sandbox; both-policy Heaptrack recordings provide
the allocation evidence instead.

The direct release probes, both-policy Heaptrack rows, competitive Criterion
controls, native/WASM release and size consumers, symbolized profile, isolated
and five-crate call graphs, and opt-in YeahRight invariants were all refreshed.

```text
# hyperreal, hyperlattice, hyperlimit, hypertri, and hypermesh
cargo test --locked --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo test --locked --all-features --no-fail-fast
cargo clippy --locked --all-targets --all-features -- -D warnings
cargo clippy --locked --all-targets --no-default-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo fmt --all -- --check
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run

# focused ASan; repeat in each owning crate with its focused test filter
CARGO_TARGET_DIR=/tmp/<crate>-rational-asan \
  RUSTFLAGS='-Zsanitizer=address' ASAN_OPTIONS=detect_leaks=0 \
  cargo +nightly test --locked --target x86_64-unknown-linux-gnu <filter> --lib

# Hypermesh evidence surfaces
YEAHRIGHT_BENCH=1 cargo test --locked --release --test competitive \
  -- --ignored --test-threads=1
./benchmarks/size-harness/measure.sh default
cargo bench --locked --bench competitive -- <workload-filter>

../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . --crate-name hypermesh --out-dir /tmp/hypermesh-phase7-rational-callgraph
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --out-dir /tmp/hyperstack-phase7-rational-callgraph
```

The retained fixture SHA-256 is
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`.
