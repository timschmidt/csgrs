# Phase 17: retained exact-rational line-filter views

Captured 2026-08-04. The paired parent is Hyperreal `0a952182`, Hyperlimit
`6e4d68c8`, and Hypermesh `d30653cf`. The retained implementation is Hyperreal
`8e2ed531`, Hyperlimit `76281a34`, and the unchanged Hypermesh
`d30653cf`.

## Result

Repeated exact-rational 2D line predicates now retain their certified
binary64 conversion eligibility and value in the canonical scalar owner.
Hyperreal observes a successful directly bounded `Rational` conversion once,
publishes the scalar-local view only after a second successful use, and then
lets every `Real` wrapper over that immutable rational reuse its existing
primitive approximation cache. The determinant filter still returns a sign
only when its conservative propagated error is strictly separated from zero.
Every declined conversion, uncertain determinant, exact boundary, symbolic
value, and out-of-range value reaches the same exact or policy-aware fallback
as before.

Hyperlimit schedules this view as the third reusable line-predicate layer:

1. certified exact-dyadic binary64 determinant;
2. checked exact homogeneous `i128` determinant;
3. certified relative-error rational binary64 determinant;
4. arbitrary-precision exact-rational determinant;
5. general `Real` refinement and the caller's Hyperlimit terminal policy.

The rational filter is deliberately constructed on demand instead of adding
its 64-byte carrier to every retained `Line2Orientation`. This keeps the fixed
evidence compact while the scalar cache amortizes the conversions actually
reused. Controlled callers moved directly from raw-`Rational` line-filter
entry points to `Real` entry points; the old line APIs were removed, not
forwarded.

This is a general predicate schedule. It does not inspect a mesh, fixture,
coordinate width, triangle count, topology, Boolean operation, policy name,
expected output, benchmark, or competitor. No compatibility path, alternate
Boolean engine, EMBER route, or benchmark-only production code was added.
Phase 17 and Phase 18 remain open.

## Exactness and publication invariant

The retained view is proof input, so publication is intentionally stricter
than generic lossy conversion:

- Every candidate must first expose an exact `Rational` from a non-computable
  `Real`.
- Direct `Rational::to_f64_lossy` conversion must succeed with a finite normal
  value, or with zero only when the rational is exactly zero.
- The approximation receives a conservative relative radius of
  `32 * f64::EPSILON`. Numerator conversion, denominator conversion, and final
  division fit inside that radius; non-normal error radii decline.
- A first successful conversion sets only a reuse-observed fact. A second
  successful direct conversion sets the reusable-view fact and attempts to
  publish that same approximation in the `Real` primitive cache. Only then may
  the retained route read a scalar-local cache.
- Direct rational conversion canonicalizes lazy ratios and learns dyadic shape
  on every attempt. For an immutable rational, a direct success therefore
  cannot first appear after an earlier generic-cache fallback. This makes a
  view retained through one wrapper safe for an independently prewarmed
  wrapper over the same rational.
- Atomic retained facts and the existing thread-safe primitive cache make
  redundant concurrent observations harmless. Eight distinct wrappers and
  512 concurrent calls are covered permanently.

The two new facts occupy bits 30 and 31 of the existing retained rational fact
word, and the approximation uses the existing `Real` primitive cache. No
field or allocation was added. Exact-dyadic shift caching now uses bits 8–29;
shifts above 4,194,302 simply decline that cache and retain the same exact
behavior.

Permanent regressions also prove that a generic lossy cache for a balanced
2,049-bit rational cannot become predicate evidence: direct bounded
conversion declines, the rational view fact remains clear, and exact fallback
is preserved. Exact-boundary line determinants return `None`. The complete
line-filter sign matrix, independent-wrapper reuse, and concurrent
publication all pass.

Hyperlimit assigns `Certainty::Exact` only after one of the three certified
layers or the arbitrary-precision kernel proves a sign. `STRICT` still cannot
consume an approximation. `APPROXIMATE_512` still terminates only in
Hyperlimit's existing 512-bit terminal after every exact route declines, and
aggregate mesh certainty is unchanged. Both large rational fixtures remain
`Certified` and policy-identical.

## Paired deterministic work

Parent and candidate were built from independent clean source archives with
the same compiler, lockfiles, release settings, and fixture archive. Runs were
pinned to CPU 11. Instructions and branches are the acceptance metrics; no
result is inferred from wall-clock frequency variation. The full and wide
rows execute the hard operations directly. The two broad controls use
repetition to expose sub-percent movement.

| Workload | Parent instructions | Current instructions | Movement | Branch movement |
| --- | ---: | ---: | ---: | ---: |
| Full rotated YeahRight, 23,788 triangles | 16,506,119,778 | 14,589,807,931 | -11.6097% | -13.5151% |
| 2,049-bit rational boxes, union, five arrangements | 14,507,737,871 | 14,489,117,465 | -0.1283% | -0.1135% |
| Clipped voxel torus 33, all results, three arrangements | 3,129,123,700 | 3,132,160,147 | +0.0970% | -0.0591% |
| Ordinary overlapping boxes, all results, 1,000 arrangements | 5,667,185,095 | 5,677,521,730 | +0.1824% | +0.1470% |

The full result is identically empty and `Certified`. The wide result remains
2,410 vertices and 4,816 triangles. The small torus and ordinary-box
instruction regressions are retained and reported rather than rounded away:
performance has priority over size, and the 11.61% difficult-case reduction
materially outweighs these 0.10–0.18% broad costs. The schedule remains one
clean scalar/predicate algorithm; there is no workload switch to remove those
costs.

## Dispatch evidence

The permanent YeahRight dispatch trace records:

| Route | Count |
| --- | ---: |
| Hyperlimit certified rational determinant | 16,499 |
| Hyperlimit certified dyadic determinant | 1,043 |
| Hyperlimit exact-word rational determinant | 186 |
| Hyperreal first reuse observation | 396 |
| Hyperreal view retained after reuse | 396 |
| Hyperreal retained-view consumption | 112,950 |

Thus 792 direct observations establish 396 scalar views and those views serve
112,950 later coordinate uses. This is the retained-fact behavior responsible
for the hard-case reduction, not a mesh-level memo table.

## Large-fixture heap

The direct global-allocator probe excludes fixture construction. Candidate,
parent, `STRICT`, and `APPROXIMATE_512` counters are exactly equal.

| Selector | Result | Incremental peak | Allocations | Reallocations | Added bytes |
| --- | --- | ---: | ---: | ---: | ---: |
| `yeahright-full-rotated` | certified empty | 158,258,204 B | 16,928,390 | 2,385,300 | 913,360,240 B |
| `wide-rational-2048` | certified 2,410 / 4,816 | 31,234,658 B | 2,092,630 | 227,677 | 459,671,086 B |

The full row records 16,575,749 deallocations, 888,970,336 removed bytes, and
24,389,848 bytes of input-attached fact growth. The wide row records
2,092,488 deallocations, 459,124,046 removed bytes, and 26,568 bytes of
input-attached fact growth. Retaining the schedule adds zero measured heap,
which matches its reuse of existing fact and primitive-cache storage.

## Source and linked size

The two implementation commits change 313 inserted and 47 deleted lines,
including the complete exactness, race, cascade, and API tests. They add no
dependency or retained field. Native values are `.text`; WASM values are
`wasm-opt -Oz` bytes.

| Profile / consumer | Parent native | Current native | Parent WASM | Current WASM |
| --- | ---: | ---: | ---: | ---: |
| release / general | 1,968,630 | 1,966,294 (-2,336) | 1,391,425 | 1,391,624 (+199) |
| release / immediate | 1,971,774 | 1,969,438 (-2,336) | 1,393,283 | 1,393,482 (+199) |
| size / general | 1,071,807 | 1,071,231 (-576) | 668,459 | 667,517 (-942) |
| size / immediate | 1,072,771 | 1,072,195 (-576) | 668,870 | 667,928 (-942) |

Release-native and both size-profile formats shrink. The only linked growth is
199 bytes (0.0143%) in optimized release WASM. Current general-release file,
gzip, and Brotli sizes are 2,470,976 / 958,858 / 727,244 bytes; current
immediate-release sizes are 2,474,136 / 959,964 / 727,978 bytes. Current
size-profile general values are 1,300,608 / 522,071 / 432,443 bytes, and
immediate values are 1,301,584 / 522,481 / 432,865 bytes.

## Call graph and removal audit

The regenerated five-crate graph at
`/tmp/hypermesh-retained-rational-filter-final-callgraph-2026-08-04` contains
14,833 function nodes and 24,701 edges:

| Crate | Nodes | Edges |
| --- | ---: | ---: |
| Hyperreal | 7,243 | 12,452 |
| Hyperlattice | 1,370 | 2,560 |
| Hyperlimit | 1,938 | 2,982 |
| Hypertri | 1,375 | 2,009 |
| Hypermesh | 2,913 | 4,668 |

Direct edges show Hyperlimit's oriented-line and ordinary `orient2d` routes
entering the outlined certified-rational filter, then
`Real::certified_rational_line2_sign`, the exact-rational conversion schedule,
retained fact access, and the existing primitive cache. There is no stored
mesh cache or alternate predicate graph. Production-module, Cargo, and public
API searches for the historical `EMBER`, `ember::`, and `ember_` routes return
no result. Hypercurve and HyperSolve are excluded and untouched.

## Validation

- Hyperreal passes 649 all-feature unit tests, every integration/oracle test,
  and 24 doctests; its default 572-unit-test suite, integrations, and 19
  doctests also pass.
- Hyperlimit passes 154 all-feature and 144 default unit tests plus every
  integration test. Its retained cascade and wide-rational tests exercise both
  policies and all three certified layers.
- Hyperlattice's complete suite passes. Hypertri passes 74 unit tests plus all
  integrations and doctests.
- Hypermesh passes 179 all-feature and 178 default executions; six documented
  manual/external tests remain ignored.
- No-default checks, warning-denied Clippy, warning-denied rustdoc, every fuzz
  binary, every benchmark target, native/WASM size builds, formatting, and
  diff checks pass.
- The full and wide direct heap sentinels pass under both policies with exact
  parent and policy equality.

## Rejected alternatives

- A Hypertri-global `Vec` cache keyed by rational storage identity added 4.35%
  full-row and 21.3% torus instructions. It was fully removed; scalar facts
  are the correct owner.
- Explicit retained line evidence in Hyperlimit's segment classifier added
  3.24% full-row and 2.15% torus instructions. It was fully removed.
- Storing the rational filter in every `Line2Orientation` enlarged each value
  by 64 bytes and was slightly slower. On-demand construction was retained.
- Restricting the filter by word width surrendered about 1.5 percentage points
  of the full improvement without fixing the broad controls. It was removed.
- Applying a cold layout attribute to the observation path increased static
  work. The observation helper remains simply outlined.
- A stable-ratio-only eligibility rule added about 5% instructions on the
  2,049-bit dyadic control. It was removed.

No rejected route, diagnostic counter, workload condition, or temporary probe
remains in production.

## Competitive status and open work

This checkpoint is gauged against the permanent historical and competitive
ledger without relabeling old measurements as current. The last pinned CGAL
6.0.3 EPECK comparison reports a 19.00x full-row runtime loss and 12.25x
fresh-process RSS loss, with ordinary-box losses of 4.81x under `STRICT` and
4.53x under `APPROXIMATE_512`. Those CGAL rows were not rerun for this scalar
checkpoint. The 11.61% deterministic full-work reduction is real, but no new
runtime or RSS ratio is inferred from it.

Current CGAL confidence runs, external real-world and deeper-symbolic fixture
families, sparse/multi-shell/pathological expansion, stage-specific arena and
retained-fact lifetime attribution, every remaining per-case runtime/RSS gate,
and the Phase 18 audit remain open. Algorithmic cleanup, fact reuse, and
specialized scheduling should continue to play to Hyperreal's strengths; a
benchmark-specific branch or an obscured algorithm is not an acceptable way
to close a row.

## Reproduction

```sh
(cd ../hyperreal && cargo test --locked --all-features && cargo test --locked)
(cd ../hyperlimit && cargo test --locked --all-features && cargo test --locked)
(cd ../hyperlattice && cargo test --locked --all-features)
(cd ../hypertri && cargo test --locked --all-features)
cargo test --locked --all-features
cargo test --locked
cargo check --locked --no-default-features
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --locked --all-features --no-deps
cargo check --locked --manifest-path fuzz/Cargo.toml --bins
cargo bench --locked --no-run

YEAHRIGHT_BENCH=1 CARGO_TARGET_DIR="$PWD/target" \
  taskset -c 11 perf stat -x, -e instructions:u,branches:u \
  target/release/examples/large_mesh_heap_probe \
  yeahright-full-rotated strict
taskset -c 11 perf stat -r 6 -x, -e instructions:u,branches:u \
  target/release/examples/competitive_arrangement_probe \
  overlapping_boxes all strict 1000
target/release/examples/large_mesh_kernel_heap_probe \
  <fixture-selector> <strict|approximate-512>
benchmarks/size-harness/measure.sh default
YEAHRIGHT_BENCH=1 cargo bench --locked --features dispatch-trace \
  --bench dispatch_trace

(cd .. && tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-retained-rational-filter-final-callgraph-2026-08-04 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format dot,json,mermaid,graphml \
  --per-library)
```

For a paired rebuild, use Hyperreal `0a952182`, Hyperlimit `6e4d68c8`, and
Hypermesh `d30653cf` as the parent archive, and change only Hyperreal to
`8e2ed531` and Hyperlimit to `76281a34` for the candidate archive.
