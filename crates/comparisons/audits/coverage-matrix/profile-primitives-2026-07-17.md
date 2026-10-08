# CSGRS profile primitive optimization — 2026-07-17

This cycle split the existing 33-constructor profile catalog into a durable
per-constructor benchmark, profiled the dominant exact paths, and replaced
three general region Booleans with directly proven topology.

## Baseline

```sh
CSGRS_BENCH_FILTER='profile_primitives/catalog' \
CSGRS_BENCH_SAMPLES=5 \
CSGRS_BENCH_WARMUP=1 \
cargo bench --bench feature_pipeline --features offset,bevymesh
```

The five catalog samples took 689,942,327–705,127,055 ns for eight iterations,
or approximately 86.9 ms per catalog construction.

`perf` recorded 3,133 cycle samples over eight catalog iterations. The hot
self-time was dominated by bigint multiplication, computable bound/sign
refinement, and allocation. The new `profile_primitives` benchmark isolated the
public constructors responsible for most of that work:

| Constructor | Baseline median ns/op |
| --- | ---: |
| `ring` | 16,797,430 |
| `circle_with_two_flats` | 16,878,442 |
| `reuleaux` | 14,442,735 |
| `crescent` | 12,306,461 |
| `circle_with_flat` | 9,295,232 |

The gear constructors were below 650 microseconds, disproving the initial
aggregate-trace hypothesis that gear/trigonometric generation dominated.

## Exact structural changes

- `ring` now constructs a `CurveRegion2` with one explicit material contour and one
  explicit hole contour. Its positive inner diameter and thickness prove the
  concentric nesting, so a general region difference is unnecessary.
- `circle_with_flat` and `circle_with_two_flats` now clip the same symbolic
  sampled-circle vertices against `y = ±flat_dist`. Side decisions use exact
  `Real` comparison and edge intersections use exact `Real` arithmetic.
- No analytic circle replaces the sampled polygon boundary, and no finite
  coordinate decides topology.

The exact oracle test reconstructs the former rectangle-difference results and
compares both optimized constructors over 36 off-boundary rational points each.
Dedicated tests also verify material/hole roles and representative inside/outside
classifications.

## Result

| Constructor | Final median ns/op | Speedup |
| --- | ---: | ---: |
| `ring` | 248,654 | 67.6x |
| `circle_with_flat` | 243,603 | 38.2x |
| `circle_with_two_flats` | 321,381 | 52.5x |

The final aggregate samples took 345,548,474–348,526,339 ns for eight
iterations, or approximately 43.4 ms per catalog construction: a 2.00x
catalog-level speedup while leaving the other 30 constructors unchanged.

## Retained shared-kernel simplification

The Reuleaux trace exposed an unreachable second Real-facts pass in
`hyperlimit::resolve_real_sign`. The first `decide_real_sign` stage always
decides exact rationals because their structural facts carry an exact sign. The
following `exact_real_sign` stage therefore returned `None` for every unresolved
symbolic value after re-reading the same facts. Removing it preserves the
filter/exact/refinement ladder and all outcomes.

Real comparison and scalar sign classification have no predicate-specific
filter or exact callback, so they now use a direct resolver that calls
hyperreal's certificate-bearing `certified_sign_until` once. Its certificate is
mapped back to the same structural or refined predicate stage; the
exact-rational comparison branch is unchanged. A close `pi < 355/113` regression
proves preservation of the bounded-refinement certificate.

A 30-sample, four-iteration A/B measured 14.610 ms/op before and 14.402 ms/op
after (1.43% faster). The patched IQR was 14.350--14.455 ms/op versus
14.502--14.722 ms/op before. Dispatch events fell from 206,175 to 184,416 while
the 7,251 successful refinements, two unknown outcomes, and final topology were
unchanged.

The direct route was then measured separately against the already-simplified
resolver: 14.792 ms/op before and 14.125 ms/op after (4.51% faster in the paired
run), with non-overlapping 14.733--14.927 and 14.027--14.250 ms/op IQRs. The
final trace contains 141,208 events, 64,967 fewer than the original trace, while
retaining the same 7,251 refined decisions and two explicit unknown outcomes.

## Retained AABB short-circuit

The next inclusive profile showed that `hypercurve::Aabb2::overlaps` certified
all four directed axis separations before combining them. One strict separation
already proves two closed boxes disjoint exactly. The classifier now returns on
the first certified separation, preserving edge/corner contact while avoiding
irrelevant later order predicates. This also prevents a later undecidable axis
from replacing an already-proved disjoint result with uncertainty.

Against the preceding optimized stack, the paired 30-sample Reuleaux median
fell from 14.266 to 12.523 ms/op (12.22%), with non-overlapping
14.199--14.441 and 12.405--12.598 ms/op IQRs. Comparisons fell from 16,081 to
12,307, comparison refinements from 7,163 to 5,830, and total dispatch events
from 141,208 to 116,040. Hypercurve's complete all-target/all-feature test and
benchmark gate passed.

## Retained attached-keyway splice

`Profile::circle_with_keyway` previously sent a convex sampled circle and an
axis-aligned rectangle through the full exact region Boolean even when the
rectangle formed one ordinary notch attached at the circle's +X vertex. The
constructor now proves that topology by testing the upper-left cutter corner
against the strictly convex ordered ring with a logarithmic fan search. Exact
X-axis symmetry certifies the reflected lower corner. The direct path traverses
the circle once, inserts the certified boundary intersections and at most two
rectangle corners, and retains the general Boolean fallback for deep or wide
cutters outside the certificate.

On the 24-segment `(radius=6, width=2, depth=2)` benchmark, the 30-sample median
fell from about 4.72 ms/op to 0.209 ms/op, or roughly 22.6x. The output checksum
and single-material/no-hole topology are unchanged. The two-operation trace
fell from 82,425 to 4,910 dependency events (94.0%), real comparisons from
4,888 to 54, and refinement events from 6,620 to 102. Four certified geometries
match the former Boolean's contour roles, signed area, and 484 exact-rational
containment classifications; a deep/wide case proves the fallback remains in
use when the direct topology certificate does not apply.

## Retained attached-keyhole splice

The keyhole constructor had the dual cost: it unioned a centered rectangle with
the sampled circle through the general region Boolean. When the handle height
is at least the radius and its bottom corner is exactly inside the convex ring,
the rectangle covers one complete upper boundary chain. A logarithmic convex
certificate plus exact intersections with the two vertical handle sides now
constructs that union ring in one traversal. Short or overly wide handles keep
the general Boolean path because they can replace different boundary chains.

For `(radius=4, width=2, height=6, segments=24)`, the stabilized 30-sample
median fell from about 5.58 ms/op to 0.161 ms/op, roughly 34.7x, with the same
checksum. The four-operation trace fell from 131,976 to 8,488 dependency events
(93.6%), real comparisons from 7,652 to 88, and refinements from 10,072 to 84.
Four certified geometries match the former Boolean's contour roles, signed
area, and 572 exact-rational containment classifications. Separate short-handle
and wide-handle regressions prove both fallback classes remain delegated.

## Retained two-crossing crescent splice

The crescent constructor previously routed every displaced-circle difference
through the general region Boolean. Both operands are homothetic regular
polygons translated only along X. The constructor now classifies their exact
boundary transitions, requires one enter and one exit on each ring, certifies
the two segment intersections, and joins the outer outside arc to the reversed
inner inside arc. Exact X-axis reflection eliminates duplicate lower-half
membership predicates. Noncrossing topology is decided from exact scalar set
identities: `|offset| <= outer-inner` creates explicit material/hole contours,
while `|offset| >= outer+inner` returns the outer ring. The equality cases cover
internal and external tangency and remove the former internal-tangent
`RealSign` panic.

The `(outer=6, inner=4, offset=3, segments=24)` 30-sample median fell from
about 10.9 ms/op to 1.42 ms/op, roughly 7.7x, with the same checksum. Its
two-operation trace fell from 93,781 to 15,134 dependency events (83.9%), real
comparisons from 7,432 to 366, and refined direct comparisons from 3,604 to
268. Five crossing geometries—including odd segments and negative offset—match
the Boolean oracle's contour roles, signed area, and 605 exact-rational
containment classifications. Contained, disjoint, and both tangent identities
have separate regressions.

## Retained shared-coordinate exact NURBS solve

Hypercurve's global interpolation solved one exact basis matrix for x and y by
constructing the matrix determinant and every Cramer replaced-column
determinant separately. Hypersolve now exposes a multi-right-hand-side Bareiss
surface that eliminates the coefficient matrix once, carries both augmented
columns through the same certified row operations, reconstructs the retained
Cramer numerators, and independently replays each `A*x-b`. Hypercurve uses it
only when all matrix and right-hand-side entries are exact rationals; the
symbolic centripetal identity path is unchanged.

The uniform quadratic API-surface workload's five-run median fell from 19.276
us/iter to 13.098 us/iter (32.1%) over 10,000 constructions with the same
checksum. Its trace fell from 11,520,001 to 6,400,001 dispatch events (44.4%)
with no refinements. Hypersolve's direct paired Criterion sentinel measured two
independent 2-by-2 solves at 2.562 us versus 2.228 us for the shared solve
(13.0%). Nine interpolation tests cover determinant/numerator evidence,
weighted and nonuniform rational systems, the symbolic fallback, invalid and
singular cases, and generated cubic recovery. A preliminary use of two
independent augmented solves regressed the end-to-end median to 20.356 us and
was superseded by the shared elimination.

## Retained certified triangulation-conformity fast path

Hypercurve's finite-region triangulation enters HyperTri's holed-polygon earcut path.
That path formerly followed every successful triangulation with a global exact scan of
all authored vertices against all triangle edges, repairing the rare case where a
normalized-away boundary vertex lies inside a long emitted edge. The retained path
first checks a purely combinatorial certificate: every authored exterior/hole edge
must occur once and every other emitted triangle edge twice. Failure keeps the
unchanged exact scan. A collinear authored-boundary regression proves an incomplete
mesh is rejected, split into a conforming three-triangle mesh, and then accepted.

The same-source five-run release control for `finite_ring_triangulation` had a
62.656 us/iter median; the certified path measured 32.671 us/iter (47.9% faster) with
the same eight-triangle checksum. Over 10,000 calls, dispatch events fell from
8,460,001 to 7,260,001 (14.2%) and predicates from 5,460,000 to 4,260,000 (22.0%),
with zero refinements. HyperTri's full all-target/all-feature gate and Hypercurve's
triangulation integration tests, strict Clippy, and warning-denied docs all passed.

The retained follow-up recognizes the common exact rectangular-annulus topology before
hole bridging. Each ring must expose exactly four structural x/y corner combinations;
exact comparisons must prove the hole is strictly contained; the eight emitted
triangles must then pass the authored boundary-edge certificate. Rotated start indices
and reversed winding are covered, while nonrectangular, touching, multi-hole, and
authored-collinear cases retain the general exact fallback.

The five-run median fell again from 32.671 to 6.254 us/iter (80.9%), or 90.0% from the
original 62.656 us/iter control, with the same checksum. One-call trace events fell
from 726 to 136 (81.3%), predicates from 426 to 52 (87.8%), scalar comparisons from
141 to 8, and orientations from 87 to 8; refinements stayed zero. HyperTri's 52 unit,
26 adversarial, six differential, and eight property tests plus every target passed,
as did both crates' strict Clippy and warning-denied documentation gates.

## Rejected experiment

A sequential exact convex-intersection splice for Reuleaux disks was also
implemented, tested on triangle and pentagon topology, benchmarked, traced, and
fully removed. Its repeated logarithmic point-location pass could not certify a
later intermediate ring, so it paid that predicate cost and then fell back to
the region Boolean. The representative median remained about 12.9 ms/op while
the two-operation trace grew from 117,970 to 241,718 events. The shared-kernel
follow-up is the single-pointer linear convex intersection of O'Rourke, Chien,
Olson, and Naddor, *A New Linear Algorithm for Intersecting Convex Polygons*
(1982), which advances around both boundaries in O(m+n) rather than repeatedly
classifying vertices. Hypercurve's prepared contour facts already identify
convexity as a future dispatch fact; the optimization belongs there rather than
in another constructor-local partial Boolean.

An exact Sutherland–Hodgman intersection of the convex sampled disks used by
`reuleaux` was implemented and fully removed. Symbolic orientation refinement
pushed the pentagon regression beyond 60 seconds in debug mode, and the
representative constructor returned empty topology. Restoring the prepared
hypercurve region Boolean returned both tests to roughly one second. This path
needs a predicate/filter improvement in the shared kernel, not a csgrs-local
clipping replacement.

A shared-kernel structural-equality shortcut in
`hyperlimit::compare_reals_report_with_policy` was also implemented, proved,
traced, benchmarked, and fully removed. `Real::PartialEq` is an exact structural
relation, so the shortcut was semantically sound and avoided constructing
`left - right` for matching symbolic representations. On the Reuleaux trace it
matched only 199 of 16,081 comparisons (1.24%), reducing dispatch events from
206,175 to 204,865 but leaving 14,357 difference-sign comparisons. An immediate
10-sample A/B measured patched and unpatched medians of 14.638 and 14.606 ms/op,
respectively. The low hit rate did not justify adding a branch to every symbolic
comparison, so no hyperlimit source change was retained.

A hyperreal follow-up factored `Computable::sign_until` so
`Real::certified_sign_until` could enter the precision-refinement tail directly
after an inconclusive structural-facts pass. This safely skipped repeated cached
exact-sign, approximation, and bound probes, but a paired 30-sample run measured
14.261 ms/op for the experiment and 14.266 ms/op for the control (0.04%), with
heavily overlapping interquartile ranges. The cached probes are already cheap;
the extraction was fully removed rather than retaining complexity for a
noise-level change.

A csgrs Reuleaux experiment reused one exact unit-direction sample table across
its three equal-radius translated disks. Every vertex remained algebraically
`center + radius * direction`, but the extra clones and scales outweighed
hyperreal's already-cheap special-form trigonometric construction: the
30-sample median moved from 12.523 to 12.616 ms/op (0.74% slower). The helper
and all call-site changes were fully removed.

Benchmark commands:

```sh
CSGRS_BENCH_FILTER='constructor/ring' \
CSGRS_BENCH_SAMPLES=10 CSGRS_BENCH_WARMUP=2 \
cargo bench --bench profile_primitives --features sketch

CSGRS_BENCH_FILTER='constructor/circle_with_flat,constructor/circle_with_two_flats' \
CSGRS_BENCH_SAMPLES=10 CSGRS_BENCH_WARMUP=2 \
cargo bench --bench profile_primitives --features sketch
```
