# Hypermesh exact binary64 enclosure scheduling checkpoint

Date: 2026-07-31

Direct Hypermesh parent: `f7efd6f8b86ff26ea57869d21e60f57e9009f201`

Hyperreal enclosure implementation:
`5bc2206e670998ad8adf1bd178f683242f466242`

Hyperreal exact-normal fast path:
`af698a8ed422fd724df62b88e4d4b1eb2feed715`

Hypermesh implementation:
`5d935f9ceabf9e17984f217e803d997e69e9e8c4`

## Outcome

Hyperreal now exposes finite, outward-rounded binary64 bounds for an exact
`Rational`. Hypermesh uses those bounds as one operation-local scheduling
cache for output T-junction and edge-crossing discovery. Floating-point work
can prove only disjointness or choose work order; every survivor still reaches
the existing exact, policy-aware predicate path.

The result closes four previously weak scheduling cases:

- distinct exact coordinates that collapse to the same nearest binary64 value;
- non-dyadic rational coordinates whose rounded scalar was not an enclosure;
- symbolic coordinates, which retain the exact axis-order fallback;
- exact rational coordinates outside the supported finite binary64 enclosure
  range, which also retain the exact fallback.

Output repair now reaches a fixed point in an explicit priority order:
T-junction passes are exhausted before one crossing batch is committed, new
crossing vertices invalidate and rebuild the enclosure cache, and the process
repeats until neither finite event family changes the mesh. There is no pass
limit, lossy equality, compatibility shim, or alternate policy-free entry
point.

Both `STRICT` and `APPROXIMATE_512` run the same scheduler. When an enclosure
overlaps, the selected Hyperlimit policy remains authoritative for the exact
comparison. All measured rational fixtures complete as
`MeshCertainty::Certified`; the approximate terminal is available but is not
consumed.

## Hyperreal enclosure contract

`Rational::to_f64_enclosure` returns `[lower, upper]` only when both endpoints
are finite and:

```text
lower <= exact rational <= upper
```

Exactly representable binary64 values return a zero-width enclosure. This
includes positive and negative least subnormals (`2^-1074`), repairing the
previous exact-conversion omission; `2^-1075` correctly remains
unrepresentable. Dyadic values that require rounding use adjacent outward
endpoints. General finite-range rationals use a normalized magnitude interval,
then expand both endpoints outward. Unsupported extreme magnitudes return
`None` rather than weakening the guarantee.

The validation corpus includes signs, zero, exact normal and subnormal
boundaries, non-dyadics, varied numerator/denominator widths through 4,096
bits, and a GMP/MPFR cross-check. The final fast path proves that a normal
dyadic with at most 53 numerator bits is exactly representable without scanning
its trailing zeroes. Its normal exponent range implies that its least bit
cannot fall below `2^-1074`.

The public benchmark governance test has a real counterpart for the new API:
Rug converts the same GMP rational to 53-bit MPFR values with `Round::Down`
and `Round::Up`. On the benchmark input, Hyperreal takes 33.274 ns and the
GMP/MPFR route takes 130.86 ns, making Hyperreal 3.93x faster.

## Hypermesh exactness and path completeness

The output layer stores each exact-rational vertex as
`[[lower, upper]; 3]`. The cache has the following permitted uses:

- interval overlap admits a possible T-junction candidate;
- disjoint intervals prove endpoint order when building exact edge bounds;
- lower and upper endpoints order the crossing sweep and prove a safe break;
- disjoint three-axis edge intervals reject a crossing pair;
- lower endpoints select a promising projection axis.

None of these uses proves coincidence, collinearity, containment, a crossing,
or topology. Exact Hyperreal/Hyperlimit predicates prove all survivors.
Overlapping endpoint intervals fall through to `compare_real_decision`, so
`STRICT` can return an undecided error and `APPROXIMATE_512` can consume its
terminal only at the same canonical policy boundary as the rest of Hypermesh.

If any vertex is symbolic or outside the enclosure range, the entire cache is
declined and the existing exact policy-aware axis ordering is used. Regressions
cover symbolic `sqrt(2)`, the exact rational `(2^1025)/3`, binary64-collapsed
T-junctions and crossings, and a mixed event set that proves T-junction
exhaustion precedes crossing batches. Every regression runs under both
policies and remains certified.

## Direct-parent CPU results

Both sides were rebuilt after the final Hyperreal commit and therefore use the
same scalar, lattice, policy, and triangulation sources. The direct A/B isolates
the Hypermesh output scheduler. Release probes were serialized on CPU 8.
The 6,144-triangle box control uses 201 repetitions. Hard-mesh rows use 61
repetitions; the retained `STRICT` row was interleaved parent/candidate by pair.

Each cell shows `parent -> candidate (movement)`.

| Fixture / policy | Task clock | Cycles | Instructions | Branches | Branch misses | Cache misses |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 6,144-t box / `STRICT` | 5.94 -> 5.92 ms (-0.3367%) | 14,632,116 -> 14,503,150 (-0.8814%) | 35,925,604 -> 35,897,755 (-0.0775%) | 6,579,193 -> 6,574,944 (-0.0646%) | 66,397 -> 65,584 (-1.2245%) | 115,805 -> 107,613 (-7.0740%) |
| Retained / `STRICT` | 93.009 -> 93.404 ms (+0.4247%) | 363,207,049 -> 364,503,616 (+0.3570%) | 1,071,842,549 -> 1,070,602,898 (-0.1157%) | 181,157,969 -> 180,761,965 (-0.2186%) | 1,474,662 -> 1,458,498 (-1.0961%) | 1,510,598 -> 1,502,717 (-0.5217%) |
| Retained / `APPROXIMATE_512` | 93.65 -> 92.80 ms (-0.9076%) | 365,626,338 -> 363,068,583 (-0.6996%) | 1,071,851,266 -> 1,070,599,496 (-0.1168%) | 181,160,133 -> 180,761,058 (-0.2203%) | 1,481,300 -> 1,461,079 (-1.3651%) | 1,548,906 -> 1,512,861 (-2.3271%) |
| Generated 13,452-t / `STRICT` | 80.08 -> 79.80 ms (-0.3497%) | 286,531,056 -> 286,018,738 (-0.1788%) | 672,318,092 -> 671,876,556 (-0.0657%) | 102,462,232 -> 102,373,458 (-0.0866%) | 866,020 -> 867,010 (+0.1143%) | 1,877,696 -> 1,885,791 (+0.4311%) |
| Generated 13,452-t / `APPROXIMATE_512` | 80.14 -> 79.55 ms (-0.7362%) | 286,241,011 -> 285,858,686 (-0.1336%) | 672,299,346 -> 671,867,285 (-0.0643%) | 102,457,443 -> 102,371,105 (-0.0843%) | 866,085 -> 866,848 (+0.0881%) | 1,869,339 -> 1,889,965 (+1.1034%) |

The small-box row improves every counter. The retained approximate row also
improves every counter. The retained strict row has a mixed sub-percent
wall/cycle result while instructions, branches, and both miss counters improve;
that wall movement is reported as measured rather than classified as a win.
Both generated rows improve time, cycles, instructions, and branches, with
small miss-count regressions. No measured workload executes more instructions.

## Large-fixture heap

Heaptrack includes fixture construction and the complete immediate operation.
The parent and candidate use the same final dependency sources. Candidate
`STRICT` and `APPROXIMATE_512` counts are identical on every fixture, and all
outputs are certified.

| Fixture / revision / policy | Allocations | Temporary | Peak heap | Heaptrack RSS | Output |
| --- | ---: | ---: | ---: | ---: | --- |
| Retained parent / either policy | 1,254,723 | 173,192 | 12.70 MiB | about 22.63 MiB | 625 v / 1,246 t |
| Retained candidate / `STRICT` | 1,254,715 | 173,184 | 12.70 MiB | 22.35 MiB | 625 v / 1,246 t |
| Retained candidate / `APPROXIMATE_512` | 1,254,715 | 173,184 | 12.70 MiB | 22.32 MiB | 625 v / 1,246 t |
| 6,144-t boxes parent / either policy | 27,214 | 85 | 4.70 MiB | — | 27 v / 50 t |
| 6,144-t boxes candidate / either policy | 27,211 | 81 | 4.70 MiB | — | 27 v / 50 t |
| Generated 13,452-t parent / either policy | 304,574 | 27,064 | 11.66 MiB | — | 154 v / 304 t |
| Generated 13,452-t candidate / either policy | 304,568 | 27,058 | 11.66 MiB | — | 154 v / 304 t |

The enclosure cache performs one capacity-matched vector allocation. Reusing it
across all T-junction passes and crossing discovery removes more other
temporary work than it adds: allocation calls fall by 3–8 and temporary
allocations fall by 4–8 without moving any peak-heap row. RSS is profiler- and
layout-sensitive, so it is informative rather than a retained-memory gate.

## Historical and competitive controls

The frozen historical retained row was 944.8 ms, 67.74 MiB peak heap,
5,020,891 allocations, and 82.5 MiB maximum RSS. The current strict row is
90.11% faster, retains 81.25% less peak heap, and performs 75.01% fewer
allocations. That historical implementation materialized a different polygon
output, so this comparison is directional rather than a direct A/B oracle.

The previous projective checkpoint measured about 90.27 ms with the same
12.70 MiB heap. The current 92.80–93.40 ms retained measurements include the
new scalar/output work and host-wide run drift. The direct-parent counters
above are the authoritative incremental comparison.

Fresh Criterion throughput controls report:

| Union workload | Hypermesh | boolmesh | manifold-rust |
| --- | ---: | ---: | ---: |
| Overlapping 12-triangle boxes | 5.5012 us | 68.242 us | 64.656 us |
| 3,072-triangle boxes per operand | 1.9398 ms | 7.6709 ms | 4.4361 ms |
| Dyadic YeahRight 840-triangle hull + box | 13.813 ms | 0.80748 ms | 0.69625 ms |

Hypermesh is 12.40x and 11.75x faster on the small exact-cell row, and 3.95x
and 2.287x faster on the large exact-cell row. On the projective row, boolmesh
and manifold-rust are 17.11x and 19.84x faster throughput references. They do
not provide Hypermesh's exact `Real`, policy, or certification contract and are
not correctness oracles.

The prior same-host rows were 5.0826/65.829/57.834 us,
1.8464/7.4557/4.3513 ms, and 13.160/0.75679/0.66567 ms respectively. Every
engine moved 1.95–11.80% slower in the fresh run, so ratios and the controlled
direct-parent probe are stronger incremental signals than comparison to that
stored Criterion sample.

## Native and WASM linked code

Native code is `.text`; WASM code is `wasm-opt -Oz`. These are the canonical
default-feature general and immediate consumers. The complete linked-code cost
is 0.0278–0.0760%, retained because performance is primary and the scheduler
improves instructions on every measured workload.

| Consumer | Profile | Parent | Candidate | Movement |
| --- | --- | ---: | ---: | ---: |
| General native | Release | 4,018,964 | 4,020,108 | +0.0285% |
| General WASM | Release | 2,695,477 | 2,696,430 | +0.0354% |
| Immediate native | Release | 4,052,596 | 4,053,724 | +0.0278% |
| Immediate WASM | Release | 2,710,454 | 2,711,466 | +0.0373% |
| General native | Size | 1,844,306 | 1,845,258 | +0.0516% |
| General WASM | Size | 1,143,465 | 1,144,334 | +0.0760% |
| Immediate native | Size | 1,856,806 | 1,857,758 | +0.0513% |
| Immediate WASM | Size | 1,154,872 | 1,155,297 | +0.0368% |

The direct release heap-probe executable moves from 4,031,287 to 4,031,335
bytes of `.text` (+48 bytes, +0.0012%), while its file shrinks from 6,326,272
to 6,322,120 bytes (-4,152 bytes, -0.0656%).

## Source and call graph

The Hyperreal enclosure/test/benchmark commit adds 203 lines and removes 9;
the exact-normal optimization adds 11 and removes 2. Hypermesh adds 162 lines
and removes 67, including both-policy path regressions. No wrapper or
compatibility layer was added.

The workspace call-graph utility reports:

| Scope | Parent | Current | Movement |
| --- | ---: | ---: | ---: |
| Isolated Hypermesh | 7,950 nodes / 19,555 edges | 7,953 / 19,563 | +3 / +8 |
| Five Hyper crates | 19,503 nodes / 38,951 edges | 19,531 / 39,025 | +28 / +74 |

The utility's syntactic alias heuristic represents `to_f64_enclosure` as a
local unresolved stub in one view, so the counts are structural change
indicators rather than dynamic-call counts.

## Rejected alternatives

- Packing `ExactEdgeBounds` endpoint selection into a bit mask saved only tens
  of linked-code KiB and added roughly 0.37 million instructions on the
  retained probe. The clearer indexed form was restored.
- A retained cached-f64 approximation feature reduced retained-fixture
  instructions by about 0.25%, but increased box instructions by about 1.7%
  and cycles by about 4.5%. It was removed.
- The first enclosure implementation routed exact dyadics through the generic
  conversion path and regressed the dominant small-mesh row. Direct dyadic
  construction plus the exact-normal shortcut recovered it.
- A direct exact-rational affine key from the preceding projective checkpoint
  saved a small number of allocations but was slower than the retained modular
  inequality filter. It remains rejected because runtime has priority.

## Validation

All completed successfully after the final Hyperreal retry:

- Hyperreal: 553 default and 630 all-feature unit tests, every integration
  suite and doctest, warning-denied all-target/all-feature Clippy, warning-free
  docs, formatting, and GMP benchmark compilation;
- Hyperlattice: default, no-default, and all-feature suites, Clippy, docs, and
  formatting;
- Hyperlimit: default, no-default, and all-feature suites (147 all-feature
  unit tests), Clippy, docs, and formatting;
- Hypertri: default, no-default, and all-feature suites (66 all-feature unit
  tests plus policy/CDT integrations), Clippy, docs, and formatting;
- Hypermesh: 1,050 default, 1,050 no-default, and 1,051 all-feature unit tests
  plus every regular integration suite, warning-denied all-target/all-feature
  Clippy, warning-denied docs, formatting, and diff hygiene;
- both-policy large-fixture CPU and heap probes, competitive Criterion controls,
  native/WASM release and size consumers, and isolated/five-crate call graphs.

## Reproduction

```sh
cd ../hyperreal
cargo test --locked --no-fail-fast
cargo test --locked --all-features --no-fail-fast
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features
cargo bench --locked --bench gmp_api -- to_f64_enclosure

cd ../hypermesh
cargo test --locked --no-fail-fast
cargo test --locked --no-default-features --no-fail-fast
cargo test --locked --all-features --no-fail-fast
cargo clippy --locked --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --locked --no-deps --all-features

cargo build --locked --release --example large_mesh_heap_probe
target/release/examples/large_mesh_heap_probe boxes-3072 strict
target/release/examples/large_mesh_heap_probe boxes-3072 approximate-512
YEAHRIGHT_HULL_OBJ=/path/to/yeahright_boolean_hull.obj \
  target/release/examples/large_mesh_heap_probe yeahright strict
YEAHRIGHT_HULL_OBJ=/path/to/yeahright_boolean_hull.obj \
  target/release/examples/large_mesh_heap_probe yeahright approximate-512
YEAHRIGHT_BENCH=1 \
  target/release/examples/large_mesh_heap_probe yeahright-8 strict
YEAHRIGHT_BENCH=1 \
  target/release/examples/large_mesh_heap_probe yeahright-8 approximate-512

heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe boxes-3072 strict
YEAHRIGHT_HULL_OBJ=/path/to/yeahright_boolean_hull.obj \
  heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe yeahright strict
YEAHRIGHT_BENCH=1 heaptrack --record-only \
  target/release/examples/large_mesh_heap_probe yeahright-8 strict

cargo bench --locked --bench competitive
./benchmarks/size-harness/measure.sh default

../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . --crate-name hypermesh \
  --out-dir /tmp/hypermesh-enclosure-callgraph-current
../tools/hyper-callgraph/target/release/hyper-callgraph \
  --root .. \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --out-dir /tmp/hypermesh-enclosure-callgraph-five
```

The retained fixture has SHA-256
`5f1ac8a2f8bf2b0c67ce95dc8146cc783155dc9c34eeee164f03446a0830886c`
and is presented as 4,524 input triangles after exact subdivision.
