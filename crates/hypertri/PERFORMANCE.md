# Performance and reference audit

This document records the July 2026 audit of every source in the README reference
section. It distinguishes exact correctness machinery from scheduling ideas and keeps
only changes that survived tests and Criterion A/B measurements. Timings are local
Criterion estimates from an optimized build; they are evidence for relative changes,
not portable absolute promises.

The automatically maintained [`benchmarks.md`](benchmarks.md) catalogues every
registered benchmark target and every stored Criterion row, and presents
same-input implementation rows as explicit timing comparisons. The dispatch
harness writes its complete non-timing evidence to `dispatch_trace.md`.

The explicit-policy runtime, native/WASM size, and call-graph baseline is
recorded separately in
[`benchmarks/baselines/policy-context-2026-07-30.md`](benchmarks/baselines/policy-context-2026-07-30.md).

## Exhaustive representation and competitor baseline

The August 30, 2026 verification extension inventories all 22 optimized finite
Hyperreal certificate classes and all eight public structural kinds. The
integration matrix exercises every ordered class pair and every class across
earcut, ordinary and spatial Delaunay, crossing and convex-hull CDT,
topology-only CDT, D-dimensional Delaunay, and runtime selection. The same
matrix passes with no approximation cache, each cache independently, and both
`cached-f32-approx` and `cached-f64-approx` together. Variable-depth opaque
expression DAGs extend the same corpus under ASan fuzzing.

Direct LLVM instrumentation across minimal, executor-free runtime,
single-algorithm runtime, complete, example, and benchmark configurations
reports 6,270 of 6,599 production executable lines (95.01%), 96.15% of all
instrumented lines, and 100% of functions. `scripts/coverage.sh` enforces the
95% production threshold.

### Competitive runtime baseline

Criterion results below are local estimates from `rustc 1.97.0` on an x86-64
Ryzen 7 5800X3D. Hypertri's rows provide exact topology over `Real`; `earcutr`
and Delaunator are ordinary finite-`f64` speed references and do not provide
equivalent general exact-real semantics. Pre-lifted rows exclude boundary
conversion, while `hypertri_f64_boundary` includes exact lifting.

| workload | Hypertri exact pre-lifted | Hypertri alternate/boundary | float reference |
| --- | ---: | ---: | ---: |
| 32-vertex polygon | 22.96 µs | 32.27 µs (`f64`) | 1.524 µs (`earcutr`) |
| 128-vertex polygon | 101.03 µs | 131.58 µs (`f64`) | 6.020 µs (`earcutr`) |
| 64-point Delaunay | 212.76 µs | 158.01 µs (spatial), 193.26 µs (`f64`) | 6.841 µs (Delaunator) |
| 400-point Delaunay | 3.520 ms | 3.358 ms (spatial), 3.568 ms (`f64`) | 52.74 µs (Delaunator) |

The 64-point ordinary Delaunay row had a wide 180.86–244.34 µs confidence
interval; the spatial row was stable at 156.72–159.05 µs. These measurements
are a transparent cost boundary, not a claim that unlike numeric contracts
should have equal throughput.

### Representation-shaped runtime and memory

`benches/representations.rs` measures one composite invocation of all five
topology families for each finite certificate. Criterion central estimates
ranged from 29.05 µs for the exact-rational `One` class to 5.63 ms for
`TanPi`; the slowest classes identify symbolic reduction work for investigation
without weakening predicate policy.

`examples/allocation_profile.rs` warms fixtures before measurement and records
allocations, allocated bytes, reallocations, peak live bytes, and end-of-epoch
live deltas for 22 classes by five operations. In an eight-iteration run,
crossing CDT ranged from 12,349 bytes/op for `One` to 1,537,097 bytes/op for
`TanPi`; the largest per-row counting-allocator peak was 43,152 bytes. An
independent one-pass Valgrind Massif profile peaked at 169,744 total heap bytes.
Memcheck observed 1,315,139 allocations and reported zero definitely,
indirectly, or possibly lost bytes and zero memory errors; the 90,728 bytes
still reachable at process exit are shared Hyperreal caches/constants.

Reproduce these profiles with:

```text
cargo bench --bench competitive --features all-algorithms,f64-interop
cargo bench --bench representations --features all-algorithms,runtime-select
scripts/allocation_profile.sh
```

## Retained changes

### Immediate runtime triangulation reports

Runtime algorithm selection formerly exposed `plan_polygon_triangulation`,
which cloned polygon facts into a public plan before callers separately invoked
`triangulate_polygon`. Runtime selection now stays inside the immediate
operation: `triangulate_polygon` returns only triangles, while
`triangulate_polygon_with_report` returns the triangles and the selected
algorithm, quality policy, and input facts together.

The stable runtime rows use the same five-vertex rational spike and were
collected serially with all algorithms and runtime selection enabled:

| benchmark | plan API | immediate API | result |
| --- | ---: | ---: | ---: |
| `runtime_polygon_triangulation` | 2.683 µs | 2.650 µs | 1.2% faster |
| `runtime_polygon_triangulation_report` | 2.805 µs | 2.685 µs | 4.3% faster |
| `exact_rational_spike_earcut` control | 2.694 µs | 2.662 µs | 1.2% faster |

### Opt-in BRIO-style batch insertion

Amenta, Choi, and Rote show that biased randomized insertion orders preserve
theoretical randomized-incremental guarantees while improving locality inside
successively larger spatially ordered rounds.  `delaunay_spatial` applies that
scheduling idea without changing exact topology decisions: SplitMix-derived
levels form deterministic growing rounds, exact coordinate comparisons produce
a median spatial traversal inside each round, and triangles continue to refer
to the caller's original point indices.  The historical `delaunay` schedule is
unchanged because insertion order can choose among valid cocircular diagonals.

| benchmark | ordinary order | spatial order | result |
| --- | ---: | ---: | ---: |
| 64-point located input | 4.001 ms | 2.991 ms | 25.2% faster |
| 400-point located input | 38.867 ms | 33.311 ms | 14.3% faster |
| 400-point scattered input | 35.915 ms | 34.775 ms | 3.2% faster |

The original 400-point API row remained at 38.867 ms after the shared insertion
helper refactor, a 0.4% movement within noise. Unit tests prove deterministic
permutation and index retention; exact validation, an `f64` lifting test, and a
256-case property sweep cover reordered output.

### Reuse positive orientation during Bowyer-Watson cavity tests

`incremental_delaunay` creates both the super-triangle and every cavity replacement
with `make_oriented`. Active triangles therefore carry a certified positive-orientation
invariant. The in-circle cavity test now consumes that invariant directly instead of
re-evaluating `orient2` for every triangle/query pair. Exact `incircle2` still makes
the topology decision and cospherical points remain in the cavity.

On `exact_delaunay_400_located_insertions`:

| state | estimate | change |
| --- | ---: | ---: |
| original | 54.530 ms | baseline |
| positive-orientation reuse | 47.938 ms | 12.1% faster |

### Do not immediately repeat completed builder proofs

The 2D builder admits only nondegenerate, positively oriented triangles and makes its
Delaunay choices with exact in-circle predicates. The D-dimensional exhaustive builder
similarly validates its common input once and admits a simplex only after exact
orientation and empty-sphere checks. Both builders formerly called their public
validators immediately afterward, repeating those same determinants. They now return
the records proved by construction. Public `validate` methods remain available for
deserialized or manually assembled records, and the test suite explicitly validates
builder outputs.

| benchmark | before | after | change |
| --- | ---: | ---: | ---: |
| `exact_delaunay_400_located_insertions`, after orientation reuse | 47.938 ms | 39.026 ms | 18.6% faster |
| `exact_delaunay_400_located_insertions`, original to final | 54.530 ms | 39.026 ms | 28.4% faster |
| `exact_nd_4d_delaunay_complex` | 5.2777 ms | 2.7994 ms | 47.0% faster |

## Retained structural boundary-conformity certificate

Holed earcut output previously entered `split_edges_at_input_vertices`
unconditionally. That exact repair is deliberately conservative: it compares every
authored vertex with every emitted triangle edge so a source vertex skipped during
normalization cannot remain in the interior of a long boundary edge. Most ordinary
earcut results already contain every authored boundary edge, making the scan duplicate
work.

The retained path first counts undirected triangle edges and compares them with the
authored exterior and hole edges. It skips the geometric scan only when every authored
boundary edge occurs once and every other emitted edge occurs twice. Any missing,
extra-boundary, or malformed edge rejects the structural certificate and runs the
unchanged exact repair. A collinear-boundary regression constructs an incomplete mesh,
proves rejection, repairs the long edge into three triangles, and then proves the
result satisfies the certificate.

On Hypercurve's eight-triangle finite-ring workload, five same-machine release runs
reduced the median from 62.656 us/iter with the unconditional scan to 32.671 us/iter
(47.9%), preserving the 80,000 checksum over 10,000 calls. Dispatch events fell from
8,460,001 to 7,260,001 (14.2%) and exact predicate calls from 5,460,000 to 4,260,000
(22.0%), with zero refinement events on both paths. The complete all-target/all-feature
HyperTri gate, strict Clippy, and warning-denied rustdoc remained green.

The next trace showed that the certified result still spent 141 exact scalar
comparisons and 87 orientations bridging and clipping the common one-hole rectangular
annulus. A second structural dispatch now recognizes exactly four structural x/y
corner combinations in both rings, proves strict hole containment with exact scalar
comparisons, emits the four surrounding quadrilateral bands as eight triangles, and
accepts them only if the same authored boundary-edge certificate passes. Rotated start
vertices and reversed winding are canonicalized. Nonrectangular, touching, symbolic-
equivalent-but-structurally-distinct, multi-hole, and authored-collinear cases retain
the general exact path.

Five release runs reduced the already-optimized median from 32.671 to 6.254 us/iter
(80.9%), or 90.0% from the original 62.656 us/iter control, with the same checksum.
One-call trace events fell from 726 to 136 (81.3%), predicates from 426 to 52 (87.8%),
exact scalar comparisons from 141 to 8, and orientations from 87 to 8; refinements
remained zero. The full gate included 52 unit tests, 26 adversarial tests, six
`earcutr` differential tests, eight property tests, and every benchmark and example
target.

### Preserve the triangulation policy through crossing construction

Constraint planarization now passes its retained `PredicateEvaluator` into the
policy-aware Hyperlimit line-intersection constructor. This closes a
construction gap where the exact segment predicates could certify a proper
crossing but a fresh ordinary scalar division could still reject the same
nonzero determinant. A regression uses an exact-normal determinant equal to
`2^-3000`: ordinary inversion is undecided, the evaluator's strict policy
constructs the Steiner point, and an independently unsupported zero still
fails closed. The change adds no second predicate policy or topology branch;
the existing evaluator remains the single owner of the decision.

## Rejected experiment

The Lawson/Lee-Lin legalization loop first certifies that an illegal edge is flippable,
then `flip_edge` checks adjacency and flippability again. A trial passed the already
certified adjacent triangles directly to the rewrite. It was restored because the
targeted rows did not establish a useful improvement:

| benchmark | before | trial | result |
| --- | ---: | ---: | --- |
| `exact_cdt_separated_cycles_general_pslg` | 428.79 us | 426.20 us | 1.34% change, within noise |
| `f64_exact_lifted_cdt_edge_flip_recovery` | 132.60 us | 132.99 us | no change detected |

## Scholarly reference mapping

| source | HyperTri disposition |
| --- | --- |
| Amenta--Choi--Rote, *Incremental Constructions con BRIO* | Retained as the opt-in `delaunay_spatial` schedule. Deterministic biased randomized rounds and exact median spatial ordering improved the ordered 64- and 400-point sentinels without replacing a single exact orientation or in-circle decision. The default API remains insertion-order-stable for cocircular tie topology. |
| Bareiss, *Sylvester's Identity and Multistep Integer-Preserving Gaussian Elimination* | Fraction-free determinant work belongs to `hyperlimit`; HyperTri calls its exact orientation, in-circle, and in-sphere predicate surface rather than owning a second elimination implementation. No triangulation-local Bareiss change was justified. |
| Boehm et al., *Exact Real Arithmetic* | `Point2` and `PointD` retain `hyperreal::Real` values and never approximate a topology branch through `f64`. The audit preserved that abstraction boundary; representation-specific shortcuts belong in `hyperreal`. |
| Boissonnat et al., *Triangulations in CGAL* | The separation of geometric traits/predicates from combinatorial storage maps to `Kernel`/`hyperlimit` versus `TriangulationDataStructureD`. Stable handles, opposite-facet neighbor slots, canonical facets, and explicit validators already follow the paper's design. Reusing certified builder facts is the retained implementation consequence. |
| Bowyer, *Computing Dirichlet Tessellations* | `incremental_delaunay` uses the empty-circumcircle conflict cavity and boundary restitch. Its positive-orientation fact reuse produced the retained 12.1% cavity-loop improvement. |
| de Berg et al., *Computational Geometry: Algorithms and Applications* | Ring normalization, exact segment intersection, point location, polygon visibility, triangulation validation, and incremental Delaunay follow the textbook decomposition. Sweep-line or monotone-polygon additions are separate algorithms rather than safe local substitutions, so they are architecture-inapplicable to the audited paths. |
| Delaunay, *Sur la sphere vide* | Empty-circle/empty-sphere legality remains the defining invariant in 2D, constrained local validation, and D-dimensional construction. The audit removed duplicate evaluations, never the invariant itself. |
| Ericson, *Real-Time Collision Detection* | Exact triangle AABBs reject impossible ear-containment candidates before full point-in-triangle predicates. The existing diagnostic counters expose those rejections. No float bounding-volume shortcut is allowed to decide topology. |
| Lawson, *Software for C1 Surface Interpolation* | Local diagonal flips drive Delaunay selection and unconstrained CDT legalization. Reusing flip certification was measured and rejected because it did not beat noise on representative paths. |
| Lee and Lin, *Generalized Delaunay Triangulation for Planar Graphs* | Protected PSLG subsegments and exact local legality on every unprotected interior edge implement the constrained-Delaunay criterion. The general PSLG benchmark guards this path; the rejected flip experiment left it unchanged. |
| Meisters, *Polygons Have Ears* | The ear loop certifies local convexity and rejects ears containing active reflex vertices. Prepared convex/reflex facts and exact triangle AABBs are updated locally after each clip. Split and cure fallbacks cover the implementation's broader practical input contract. |
| Pachner, *P.L. Homeomorphic Manifolds Are Equivalent by Elementary Shellings* | `BistellarFlipD` models the two sides of a `d + 2` circuit; validation proves removed-cell presence, replacement independence, and Delaunay legality before the functional oracle rewrite. A mutable TDS scheduler would be a new storage architecture, not an optimization of the functional oracle. |
| Shewchuk, *Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates* | Predicate ownership stays in `hyperlimit`; HyperTri consumes decided exact signs and rejects unknown outcomes. Since native inputs are general exact reals rather than primitive floats, duplicating Shewchuk expansion arithmetic here would violate layering. |
| Shewchuk and Brown, *Fast Segment Insertion and Incremental Construction of CDTs* | HyperTri planarizes constraints, walks crossed unconstrained edges, flips them, and re-legalizes locally. The paper's randomized linear-in-crossings segment insertion requires persistent mutable adjacency; grafting it onto repeated flat-triangle scans would not realize the bound, so it is architecture-inapplicable to the current representation. |
| Watson, *Computing the n-Dimensional Delaunay Tessellation* | The small D-dimensional oracle uses exact orientation and in-sphere tests, reports conflict cells and canonical boundary facets, and deliberately rebuilds exhaustively. Removing its immediate duplicate validation yielded the retained 47.0% improvement. A mutable TDS cavity stitcher would replace this deliberately small-input oracle rather than optimize it locally. |
| Yap, *Towards Exact Geometric Computation* | Exact predicates decide every combinatorial branch; cached structural facts may schedule or reject work but cannot replace certification. This policy is preserved across earcut, CDT, and D-dimensional features. |

## Implementation lineage and comparison projects

| project | Finding and disposition |
| --- | --- |
| Mapbox Earcut | Its modified ear slicing, hole bridging, curing/splitting, and spatial filtering inform HyperTri's owned exact port. Mapbox's z-order hash favors primitive coordinate normalization and practical robustness; without a cheap exact-rational/dyadic key it is incompatible with the current exact-real cost model. |
| `earcutr` | Remains a dev-only differential oracle for ordinary finite cases. Production code does not link it. The audited crate metadata identifies `frewsxcv/earcutr`, correcting the previous README link. |
| Spade | Confirms the value of persistent DCEL adjacency, point-location hints, and exact predicates. HyperTri's current flat 2D triangle arrays cannot cheaply adopt those pieces independently; the D-dimensional TDS is the architectural staging point. |
| CGAL triangulation packages | Confirms explicit infinite elements, face/neighbor storage, geometry/combinatorics separation, constrained subsegment provenance, and hierarchy-assisted point location. Existing HyperTri APIs cover the validation-oriented subset; hierarchy and mutable insertion require a different persistent storage contract. |

## Architecture-change triggers

The reference audit is complete for the current APIs. These are explicit
conditions that would justify a new architecture audit, not unattempted local
optimizations:

- a demonstrably cheap exact z-order or grid/dyadic key for ear candidates;
- replacement of flat 2D triangle arrays with persistent adjacency, enabling
  randomized segment insertion and point-location hierarchies;
- a large-input mutable D-dimensional TDS contract that can consume
  `DelaunayInsertionReportD` conflict boundaries;
- new public algorithms for monotone partitioning, sweep-line planarization, or
  convex-only fan triangulation, each with separate proofs and benchmarks.

## Validation protocol

The retained implementation was checked with the all-feature unit, adversarial,
differential, property, and doctest suites. Release-readiness additionally requires:

```text
cargo fmt --all -- --check
cargo test --all-features --locked
cargo clippy --all-targets --all-features --locked -- -D warnings
cargo check --all-targets --all-features --locked
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps --locked
```
