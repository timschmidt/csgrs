# Phase 17 retained wide-rational linear-form checkpoint

Captured 2026-08-03 on the established Ryzen 7 5800X3D / CPU 11 protocol at
Hyperreal `e0f9e1ec3f1c4c56adbfcae5c825f01d7b526d9f` and Hypermesh
`d58703d65c5236895c7c0bcacb2f1e826ec87bf7`.

This checkpoint attacks the exact-arithmetic cliff exposed by the permanent
fixed-topology wide-rational corpus. Hyperreal now retains a compact exact
four-term rational form only when backend width and demonstrated reuse make
its one-time reduction profitable. Hypermesh stores those uncommon forms in a
lazy operation-local sidecar and reuses them for subsequent point/plane signs.
Every filter decline, unsupported scalar representation, failed compaction,
and unscheduled query continues through the former complete exact product-sum
path. The change is a general retained-fact schedule; it contains no fixture,
operation, topology, expected-result, or benchmark dispatch.

This is a retained Phase 17 optimization checkpoint. It does not close the
remaining corpus, size-recovery, absolute CGAL EPECK, stage-lifetime, Phase 17,
or Phase 18 gates.

## Exact representation and schedule

`ExactRationalLinearForm4` represents the same homogeneous form up to one
strictly positive scale. Construction:

1. canonicalizes four borrowed exact-rational coefficients;
2. requires dyadic denominators and stores them as four shift counts;
3. computes the common positive numerator content once;
4. divides four numerator magnitudes by that content; and
5. retains signs, compact numerators, and shifts without rebuilding four
   `Rational` values.

Evaluation borrows the query rationals, validates their dyadic representation,
multiplies the compact coefficients directly, aligns powers of two, and
compares exact positive and negative `BigUint` totals. A non-dyadic query or
shift overflow returns `None`; Hypermesh then calls the unchanged
`Rational::signed_product_sum_ordering`. No approximate value makes a
topological decision.

The clean schedule is tied to the locked `num-bigint` basecase factor envelope,
`32 * usize::BITS`, rather than a corpus width. Retention requires all of the
following:

- at least two nonzero coefficients and two live products;
- a query numerator wider than one backend envelope;
- a coefficient numerator wider than three backend envelopes;
- a live coefficient/query product with both factors wider than one envelope;
- dyadic coefficients and live query values; and
- nontrivial common positive numerator content.

The envelope is pointer-width aware, including 32-bit WASM. These tests are
only scheduling gates. They neither alter the mathematical result nor remove
the complete arbitrary-rational and symbolic routes.

Hypermesh's existing 8,192-entry certified filter cache keeps its former dense
entry layout. Two lazy vectors form the exact sidecar: one `u16` index/decline
map aligned only through the demanded cache entry and one contiguous vector of
compact forms. A cache hit reuses its already known dense-entry index, so there
is no second hash lookup. The sidecar clears atomically with the bounded cache.
Ordinary rows therefore pay neither a per-entry exact carrier nor a per-entry
heap allocation.

Both `STRICT` and `APPROXIMATE_512` use this exact certified route identically.
`STRICT` still cannot consume a terminal approximation;
`APPROXIMATE_512` retains its Hyperlimit-owned 512-bit terminal for only the
paths that genuinely reach it. Every wide-corpus result here remains
`Certified`, and both policies return exactly equal meshes.

## Fixed-topology performance

The operation is union of the permanent 6,144-triangle overlapping-box family.
Current values are medians of three independent warm aggregate means after
exact import and policy-qualified PWN priming. Historical Hypermesh and pinned
CGAL EPECK values are the unchanged exact-input rows from
`phase11-17-fixed-topology-wide-rational-corpus`; CGAL was not rerun because
the checksum-pinned OFF assets and competitive executable did not change.

| Exact scale component | Historical STRICT | Current STRICT | Historical APPROXIMATE_512 | Current APPROXIMATE_512 | Current STRICT / CGAL |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 1 bit | 94.123 ms | 96.830 ms (+2.88%) | 93.283 ms | 95.894 ms (+2.80%) | 20.72x |
| 65 bits | 141.633 ms | 144.259 ms (+1.85%) | 140.274 ms | 140.071 ms (-0.14%) | 6.45x |
| 513 bits | 187.713 ms | 186.939 ms (-0.41%) | 192.244 ms | 193.614 ms (+0.71%) | 6.39x |
| 2,049 bits | 542.174 ms | 488.674 ms (-9.87%) | 540.822 ms | 489.002 ms (-9.58%) | 8.85x |

The retention schedule cannot activate on the first three rows; their small
movements are controls and are not claimed as improvements. At 2,049 bits the
form crosses the backend gate and repeated exact plane signs repay one common
reduction. The absolute loss to pinned CGAL remains open: the current ratio is
8.846x under `STRICT` and 8.852x under `APPROXIMATE_512`, improved from
9.814x/9.790x but still far from parity.

Single-sample interleaved parent/candidate probes at adjacent generated shifts
locate the algorithmic crossover. They are directional schedule evidence, not
canonical timing claims:

| Shift | Parent | Retained form | Movement |
| ---: | ---: | ---: | ---: |
| 512 | 189.527 ms | 189.828 ms | +0.16% |
| 1,536 | 260.888 ms | 262.916 ms | +0.78% |
| 1,792 | 271.171 ms | 275.512 ms | +1.60% |
| 1,984 | 294.796 ms | 290.571 ms | -1.43% |
| 2,048 | 528.517 ms | 498.435 ms | -5.69% |
| 2,112 | 543.329 ms | 504.429 ms | -7.16% |
| 2,304 | 599.920 ms | 557.068 ms | -7.14% |
| 3,072 | 797.536 ms | 766.574 ms | -3.88% |

`competitive::support::wide_rational_shift` now parses any positive `u32` from
the corpus-only `wide_rational_boxes_<shift>` spelling so adjacent schedule
probes do not require named fixtures. This parser is not reachable from the
production Boolean engine.

## Whole-process retired work

Eleven fresh-process `perf stat` repetitions of the 2,049-bit `STRICT` heap
probe include fixture preparation, exact import, PWN priming, and one Boolean.
That setup dilutes the kernel-only runtime improvement.

| Counter | Historical | Current | Movement |
| --- | ---: | ---: | ---: |
| Task clock | 744.52 ms | 724.81 ms | -2.65% |
| Cycles | 2,970,098,453 | 2,882,230,373 | -2.96% |
| Instructions | 8,612,012,286 | 7,896,973,804 | -8.30% |
| Branches | 1,397,652,445 | 1,203,350,319 | -13.90% |
| Branch misses | 5,923,494 | 5,902,340 | -0.36% |
| Cache misses | 5,871,094 | 5,976,366 | +1.79% |

The meaningful reduction is deterministic arithmetic and allocation work, not
a noisy branch-predictor effect. Cache misses are reported rather than hidden
and remain an optimization target.

## Direct large-mesh heap

The allocator-instrumented kernel boundary excludes fixture preparation and
drops output and input separately. `STRICT` and `APPROXIMATE_512` are
byte-identical on every row and return the certified 2,410-vertex,
4,816-triangle union.

| Scale bits | Incremental peak | Alloc calls | Realloc calls | Allocated-byte churn | Output payload | Input fact growth |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 65 | 26,906,378 B | 1,076,635 | 2,035 | 68,227,422 B | 520,472 B | 9,976 B |
| 513 | 46,033,482 B | 1,806,898 | 38,201 | 293,106,862 B | 520,472 B | 12,744 B |
| 2,049 historical | 111,848,010 B | 3,183,205 | 955,733 | 1,488,168,086 B | 520,472 B | 26,568 B |
| 2,049 current | 114,577,326 B | 2,425,407 | 443,138 | 851,964,750 B | 520,472 B | 26,568 B |

The 65- and 513-bit rows are exactly unchanged, proving that the lazy sidecar
does not tax the ordinary cache population. At 2,049 bits, peak rises
2,729,316 bytes (2.44%), while allocation calls fall 757,798 (23.81%),
reallocations fall 512,595 (53.63%), and byte churn falls 636,203,336 bytes
(42.75%). Performance has priority after exactness; the bounded operation-local
peak increase is retained for the material time and churn reductions. Output
ownership and input-attached fact growth remain unchanged.

## Unaffected controls

Fresh current control timings, which do not activate this wide-form schedule,
remain healthy: overlapping-box union is 0.848 ms `STRICT` and 0.923 ms
`APPROXIMATE_512`; the ordinary 6,144-triangle subdivided-box union is
95.599 ms; dense-coplanar intersection is 289.623 ms at 6,144 triangles and
1,201.357 ms at 24,576 triangles; clipped-torus intersection is 95.357 ms at
6,412 triangles and 408.119 ms at 25,100 triangles. These are guard rows, not
new comparative claims.

## Code, binary size, and graph

The implementation commits add a net 293 lines in Hyperreal and 160 lines in
Hypermesh, including focused unit and policy/cache regressions. The complete
default size harness changes as follows:

| Profile / consumer | Historical native text | Current native text | Historical optimized WASM | Current optimized WASM |
| --- | ---: | ---: | ---: | ---: |
| release / general | 2,033,826 B | 2,040,610 B (+0.334%) | 1,441,629 B | 1,448,546 B (+0.480%) |
| release / immediate | 2,036,970 B | 2,043,754 B (+0.333%) | 1,443,484 B | 1,450,406 B (+0.480%) |
| size / general | 1,080,599 B | 1,083,943 B (+0.309%) | 675,111 B | 678,107 B (+0.444%) |
| size / immediate | 1,081,539 B | 1,084,883 B (+0.309%) | 675,517 B | 678,514 B (+0.444%) |

This bounded growth remains a Phase 17 recovery item. It is retained because
the exact high-width runtime and allocation improvements are material and the
smaller measured alternatives were slower.

The committed five-crate call graph was regenerated over exactly Hyperreal,
Hyperlattice, Hyperlimit, Hypertri, and Hypermesh. It contains 14,669 function
nodes and 24,364 edges at
`/tmp/hypermesh-retained-wide-form-callgraph-2026-08-03`. Hypercurve and
HyperSolve are excluded. The graph shows one Hyperreal carrier consumed by the
existing Hypermesh predicate/cache path, not another Boolean engine or scalar
compatibility layer.

## Rejected implementations

Every losing experiment was removed completely:

- An eager primitive-integer `Rational` carrier made the 2,049-bit row about
  41% slower. A thresholded form still added 1.01% instructions, 0.83% cycles,
  and 9.21% cache misses.
- Replacing coefficients with full retained `Rational`s reached about
  515.55 ms but raised peak heap to 126.31 MB.
- Delayed/rebuilt carriers measured about 543.64–669.83 ms and up to
  125.70 MB peak.
- Coefficient-only thresholds either regressed the adjacent 1,536/1,792 shifts
  by about 16% or discarded nearly all of the 2,049-bit gain.
- Eager compact construction at a 4,096-bit threshold reached about 497 ms at
  2,049 bits but regressed the adjacent mid-width rows by 19–20%.
- Adding a dyadic query fact to every point query regressed broad workloads by
  7–8%.
- Storing an exact carrier inline in every filter entry added exactly 131,072
  peak bytes to both the 65- and 513-bit heap rows. The retained lazy sidecar
  removes that ordinary-row tax.

These results are why the retained implementation uses one compact exact form,
one backend-derived schedule, and one lazy sidecar. No benchmark-specific
threshold, output shortcut, or alternate algorithm remains.

## Validation

Hyperreal passes `cargo test --all-features` (648 unit tests, all integration
tests, and 24 documentation tests), no-default checking, warning-denied
all-target/all-feature Clippy, warning-denied rustdoc, all fuzz-bin checks,
bench compilation, and the GMP public-API classification audit.

Hypermesh passes 119 unit, 8 Boolean, 8 executed competitive, 11 manifest,
2 intersection, 9 policy, and 2 README tests: 159 executed tests with six
documented opt-in/manual ignores. No-default checking, warning-denied
all-target/all-feature Clippy, warning-denied rustdoc, all fuzz-bin checks,
bench compilation, both-policy direct heap rows, wide and unaffected control
rows, formatting, diff checks, the size harness, and exact/certainty checks are
green. Tests cover differential exact signs, the backend schedule boundary,
non-dyadic and no-common-content declines, cache-sidecar clearing, both
policies, and aggregate certainty.

## Open work

The 2,049-bit row is still roughly 8.85x pinned CGAL EPECK, ordinary exact-box
rows remain slower, and the retained form adds native/WASM code. Phase 17 must
continue through the actual profiles: reduce event and arithmetic work, improve
locality, isolate stage lifetimes, and recover linked size without flattening
Hyperreal expressions or distorting clean algorithms. Phase 11 still needs
external real-world, deeper-symbolic, and further sparse/multi-shell fixtures.
Phase 18 remains open until every shared-contract competitive row and final
requirement audit closes.

## Reproduction

```sh
cargo test --all-features
cargo clippy --all-targets --all-features -- -D warnings

taskset -c 11 target/release/examples/competitive_arrangement_probe \
  wide_rational_boxes_2048 union strict 5
taskset -c 11 target/release/examples/competitive_arrangement_probe \
  wide_rational_boxes_2048 union approximate-512 5

taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 strict
taskset -c 11 target/release/examples/large_mesh_kernel_heap_probe \
  wide-rational-2048 approximate-512

taskset -c 11 perf stat -r 11 -x, \
  -e task-clock,cycles,instructions,branches,branch-misses,cache-misses \
  target/release/examples/large_mesh_heap_probe wide-rational-2048 strict

benchmarks/size-harness/measure.sh

tools/hyper-callgraph/target/release/hyper-callgraph \
  --root . \
  --out-dir /tmp/hypermesh-retained-wide-form-callgraph-2026-08-03 \
  --crate-name hyperreal,hyperlattice,hyperlimit,hypertri,hypermesh \
  --format dot,json,mermaid,graphml \
  --per-library
```
