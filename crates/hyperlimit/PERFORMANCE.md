# Performance and Reference Audit

This document records the reference-driven optimization audit for `hyperlimit`.
Changes are retained only when the exact report contract remains intact and a
focused Criterion comparison shows a meaningful improvement.

## 2026-08-29 verification, parallelism, and comparison audit

An exhaustive source audit expanded unit, integration, property, retained-
report forgery, metamorphic, batch-parity, and boundary tests. A fresh
LLVM source-coverage run over every feature-enabled unit and integration test
now executes all 10,532 production physical lines in every source module
(100.00%). LLVM's raw source view also includes trailing inline `#[cfg(test)]`
modules because they share the production file paths; that view covers all
1,216 functions, 17,853 of 17,926 lines (99.59%), and 27,798 of 28,356 regions
(98.03%). Every one of its 73 missed lines is inside an inline test module, not
production code. The pre-audit baseline covered 85.43% of source lines and
91.40% of source functions. `scripts/coverage.sh` uses the toolchain-matched
`llvm-tools-preview`, isolates its build/profile artifacts, excludes dependency
and external-test paths, reports the raw and production-only views separately,
and writes the raw annotated HTML report.

Source-line coverage is paired with an explicit `Real` representation matrix;
running a common arithmetic line with one convenient rational is not treated
as representation coverage. `tests/real_representations.rs` constructs every
public `StructuralKind` and all twenty-two optimized Hyperreal class certificates:
`One`, `Pi`, `PiPow`, `PiInv`, `PiExp`, `PiInvExp`, `PiSqrt`, `ConstProduct`,
`ConstOffset`, `ConstProductSqrt`, `Sqrt`, `Exp`, `Ln`, `LnAffine`, `LnProduct`,
`Log10`, `Log2`, `Pow10`, `Pow2`, `SinPi`, `TanPi`, and `Irrational`. Each is translated through
scalar ordering/sign, 2D orientation, segment and triangle containment, 3D
orientation, and plane incidence under strict and approximation-capable
policies. The matrix separately covers zero/word/multi-limb/very-large rational
storage, dyadic/non-dyadic values, binary32/binary64/subnormal imports, positive
and negative/fractional scales, cold and warmed `F32(Some/None)` and
`F64(Some/None)` primitive caches, untriggered and triggered abort signals,
JSON and CBOR round trips, and a terminally unresolved opaque identity. A Serde
unknown-variant probe compares the matrix with Hyperreal's private class enum,
so adding a certificate makes this suite fail until that representation gains a
predicate case. `scripts/representation_coverage.sh` runs all four optional
primitive-cache feature combinations and checks whether private F32 cache
states are actually present or absent as configured.

The finite matrix also exhausts all four `RationalStorageClass` variants, all
five `PrimitiveFloatStatus` variants for both binary32 and binary64, all 57
private `Computable::Approximation` node variants, and all 18 shared-constant
payloads. Exhaustive Rust matches make additions to the public enums fail to
compile; Serde unknown-variant probes do the same for Hyperreal's private class,
node, and constant enums. Every computable node and constant representative is
deserialized, evaluated at a fixed precision, round-tripped, embedded in an
opaque `Real`, and crossed through exact predicate translations. Recursive
computable graph topology remains mathematically unbounded, so variable-depth
shared DAGs and metamorphic fuzzing cover composition without making the false
claim that every possible expression tree can be enumerated.

### Scalar and geometry ownership

| Layer | Owns | Performance boundary |
| --- | --- | --- |
| Hyperreal | Exact `Rational` storage, the scaled 22-class `Real` certificate, lazy `Computable` graphs, scalar facts, refinement, and approximation caches. | Simplifies and certifies scalar structure before allocating or refining a generic graph. |
| Hyperlattice | Fixed-size point/vector/matrix/projective carriers, shared-scale views, sparse support, zero masks, determinant schedules, and exact reducers. | Reuses object-level algebra and facts; it constructs expressions but does not classify geometry. |
| Hyperlimit | Policy-aware predicate cascades, typed classifications, bounded escalation, exact predicate constructions, retained evidence, reports, replay, and validation. | Consumes scalar/carrier facts and returns `Unknown` rather than inventing an epsilon or primitive-float topology decision. |
| Hypercurve/Hypertri/Hypermesh and higher crates | Curves, triangulations, mesh topology, Boolean structure, and application policy. | Consume Hyperlimit outcomes; they do not redefine scalar or predicate semantics. |

This split is why representation coverage begins below Hyperlimit: a predicate
must remain correct for every scalar/storage/graph state Hyperreal can carry and
every retained carrier fact Hyperlattice can expose, while topology stays out
of the predicate crate.

The audit found that all twelve `_parallel` batch functions were feature-
gated but still iterated sequentially. They now use Rayon's parallel iterator,
and tests compare every sequential and parallel result. On a 16-thread Ryzen
7 5800X3D, 8,192-case near-degenerate batches measured these speedups:

| Predicate | Sequential | Rayon | Speedup |
| --- | ---: | ---: | ---: |
| `orient2d` | 368.01 us | 118.23 us | 3.11x |
| `orient3d` | 658.70 us | 173.44 us | 3.80x |
| `incircle2d` | 385.39 us | 135.66 us | 2.84x |
| `insphere3d` | about 1.17 ms | 275.90 us | 4.24x |

The same audit made a retained triangle/triangle validation error reachable:
a non-coplanar relation that retained a coplanar report now returns
`UnexpectedCoplanarClassification`, matching the public validation contract.

### Competitive benchmark scope

Criterion now runs the four core predicates against `robust` and
`geometry-predicates`, with `apfp` covering its supported 2D predicates. Each
row processes the same 512 prebuilt binary64 coordinate sets. These are API-
level comparisons, not semantic-equivalence claims: Hyperlimit converts to
exact `Real` coordinates and returns certainty/escalation metadata, and its
evidence rows reuse retained polynomial data; competitors consume binary64
coordinates and return only a determinant value or sign. Representative easy-
case batch means were:

| Predicate | Hyperlimit | Retained evidence | `robust` | `geometry-predicates` | `apfp` |
| --- | ---: | ---: | ---: | ---: | ---: |
| `orient2d` | 13.25 us | - | 1.30 us | 1.23 us | 986.89 ns |
| `orient3d` | 47.41 us | - | 8.40 us | 3.49 us | - |
| `incircle2d` | 24.98 us | 18.06 us | 2.91 us | 2.92 us | 2.50 us |
| `insphere3d` | 71.31 us | 41.97 us | 14.21 us | 10.99 us | - |

The unfiltered result set, including internal regression rows, competitive
implementations, near-degenerate inputs, confidence intervals, and relative
timing ratios, is generated in [`benchmarks.md`](benchmarks.md). The predicate
benchmark refreshes it automatically; `cargo run --example
write_benchmarks_md` rebuilds it from stored Criterion output.

### Steady-state allocation profile

`examples/allocation_profile.rs` installs a counting system allocator, builds
and warms fixtures before measurement, and reports calls, bytes, reallocations,
peak live bytes, and retained bytes. In a 256-iteration release run, exact-
rational `orient2`, `orient3`, `incircle2`, `insphere3`, point/AABB, and three-
plane intersection queries made zero steady-state allocations. The exact 3D
segment classifier used 3 allocations/120 bytes, its segment/triangle report
used 9/360 bytes, and a 512-case orientation batch used one 1,536-byte output
allocation. The Rayon batch averaged 1.02 allocations and 1,559.8 bytes per
call, including amortized worker-pool activity. A symbolic pi-offset `orient2`
query used 27 allocations/1,392 bytes, exposing the intentional generic exact-
expression cost rather than fixture allocation.

### Fuzzing scope

The two libFuzzer targets now consume typed inputs instead of treating fuzzer
bytes as passive payload. They vary rational offsets, scale, step, shear,
structural `Real` representation, and strict versus approximate policy while
checking scalar/batch/Rayon agreement, sign reversal, translation and incidence
laws, retained report validation/replay, and ordering consistency. The
representation target constructs all twenty-two optimized certificates on every
execution, rotates their pairings from fuzz input, and grows variable-depth
opaque computable DAGs. Both targets compile independently and completed
sanitizer-backed smoke campaigns without a crash. The representation carrier
is deliberately eight bytes (its policy uses the high stride bit), matching
the existing corpus rather than silently rejecting every saved seed: its final
campaign executed 512 inputs and reached 7,024 coverage counters. The predicate
campaign replayed 1,215 inputs and reached 8,297 counters.

## Immediate explicit-sphere API gate

`PreparedExplicitSphere3` stored only borrowed center and squared-radius
references and forwarded its sole point query. The immediate
`classify_point_sphere3` API now carries that operation directly.

A paired Criterion gate kept a transition row beside an unchanged immediate
control. The transition measured 113.73 ns through the former wrapper and
110.75 ns through the immediate API. The control moved from 114.95 ns to
111.34 ns in the same runs, showing that the transition adds no overhead after
normalizing for run-wide variation.

## Immediate 3D segment API gate

`PreparedSegment3` likewise stored only borrowed endpoints and forwarded point,
containment, and intersection queries. Those operations now use
`classify_point_segment3`, `point_on_segment3`, and
`classify_segment3_intersection` directly.

The paired point benchmark measured 394.33 ns through the former carrier
against a 380.51 ns immediate control. After migration, the two identical
immediate rows measured 401.73 ns and 403.24 ns: both shared run-wide drift,
while the migrated row removed the wrapper overhead. The intersection pair
measured 936.57/945.32 ns before and 930.88/922.66 ns after; the sub-2% reversal
between identical post-change bodies is harness-layout noise rather than a
predicate regression.

## Exact 3D point/triangle distance completion

The exactCore `students/xinwei` sphere-planning snapshot needs point-to-triangle
distance, but its copied binary32 closest-point routine divides by a zero Gram
determinant on collinear triangles and feeds `NaN` into a classifier that then
returns `FREE`. Hyperlimit now completes its squared-distance predicate family
with `compare_point_triangle3_distance_squared`. Non-degenerate triangles use
unnormalized barycentric projection numerators and a division-free plane
comparison; exterior projections and all degenerate triangles reduce to the
minimum of the three existing closed-segment comparisons.

A dedicated rational kernel avoids materializing the larger generic `Real`
graph. Each Criterion iteration processes 512 prebuilt cases, one quarter of
them degenerate:

| Exact-rational workload | Generic composition | Dedicated kernel | Change |
| --- | ---: | ---: | ---: |
| Non-dyadic coordinates and threshold | 1.4861 ms | 1.2294 ms | -17.27% |
| Non-dyadic coordinates, dyadic threshold | 1.1729 ms | 0.78962 ms | -32.68% |
| Integer coordinates and threshold | 1.5646 ms | 0.96167 ms | -38.54% |

The retained kernel uses 36 allocations/2,016 requested bytes for the measured
interior and degenerate calls and 72/4,032 for the exterior call. Against the
generic composition this saves 3 allocations/168 bytes on the interior path,
is equal on the degenerate path, and costs 3 allocations/168 bytes on the
exterior path. The all-feature release rlib grows by 22,140 file bytes and its
metadata by 5,585 bytes; summed object sections grow by 11,407 text bytes while
data falls by 8 bytes, for 11,399 additional loadable bytes.

Exact analytic tests cover face, edge, vertex, repeated-vertex, collinear, and
strict-uncertainty cases. A 512-case property matrix checks permutation
invariance and zero vertex distance, every public Hyperreal representation
crosses the API under both policies, and an independent Ericson-region release
oracle agreed on all 49,997 non-boundary integer cases (three exact/near
boundaries were left to the analytic tests). The exactCore regression preserves
the diagonal-shell case whose squared distance is `25/12`: outside the legacy
`sqrt(2)` shell but inside the sound `sqrt(3)` shell.

## Immediate facts-aware 2D segment API gate

`PreparedSegment2` combined borrowed endpoints with public `Segment2Facts`.
Callers can now retain that compact evidence directly and pass it to
`classify_point_segment_with_facts`, `point_on_segment_with_facts`, or
`classify_segment_intersection_with_facts`, without a second object interface.

The paired point benchmark measured 256.57 ns through the former carrier and
254.45 ns through the immediate facts-aware control. The migrated transition
then measured 243.69 ns. A degenerate point/segment intersection measured
257.65 ns before and 258.49 ns after, which Criterion classified as no
performance change (`p = 0.20`).

## Immediate orientation-aware 2D triangle API gate

`PreparedTriangle2` retained three borrowed vertices, structural facts, and a
fixed orientation. Repeated queries only consume the orientation, so
`classify_point_triangle_with_orientation` now accepts that exact outcome
directly. Callers can compute it once with `orient2` and keep point
classification immediate.

The paired baseline measured 73.093 ns through the former carrier and
73.504 ns through the immediate control. In the repeated post-change run, the
two source-identical immediate rows measured 76.002 ns and 76.282 ns, with both
moving by the same +3.35% against baseline. The equal drift and 0.4 ns
within-run spread show no normalized regression.

## Immediate orientation-aware 3D triangle API gate

`PreparedTriangle3` retained borrowed vertices alongside an exact derived normal
and its certified component signs. `Triangle3Orientation` now owns only that
derived evidence, and `classify_point_triangle3_with_orientation` accepts it
without turning the triangle into a query object.

The paired baseline measured 1.2454 us through the former carrier and
1.2204 us through the immediate orientation control. After migration, the
transition measured 1.1957 us, a statistically significant 4.32% improvement,
while the control measured 1.2317 us with no detected performance change
(`p = 0.44`).

## Retained optimization

### Collapse redundant Real sign-resolution passes

`resolve_real_sign` already asks `decide_real_sign` for the value's structural
facts before predicate-specific filters. Exact-rational structural facts always
carry `sign: Some(_)`, so they decide on that first pass. The former
`exact_real_sign` stage then re-read the same facts: it was unreachable for
exact rationals and returned `None` for every unresolved symbolic value. The
stage and its duplicate structural traversal are now removed; predicate
filters, exact callbacks, bounded refinement, certainty, and escalation remain
in the same order.

Scalar sign classification and Real ordering have no predicate-specific filter
or exact callback. They now use `resolve_real_sign_direct`, which calls
hyperreal's certificate-bearing `certified_sign_until` once and maps
`StructuralFacts`/`ExactZeroScale` to the structural stage and
`BoundedRefinement` to the refined stage. The exact-rational ordering branch is
unchanged. A close `pi < 355/113` regression proves that symbolic ordering still
reports `PredicateCertificate::BoundedRefinement`.

On the csgrs Reuleaux region-Boolean workload, a 30-sample, four-iteration A/B
measured 14.610 ms/op before and 14.402 ms/op after, a 1.43% reduction. The
patched interquartile range was 14.350--14.455 ms/op versus
14.502--14.722 ms/op before. Dispatch tracing removed 21,759 events
(206,175 to 184,416): 7,253 redundant `exact_real_sign` calls and the two
associated fact-query events for each call. All 7,251 successful refinements,
two unknown outcomes, and predicate decisions were unchanged.

The direct certificate route was measured separately against that already
simplified resolver: 14.792 ms/op before and 14.125 ms/op after, a 4.51%
reduction in the paired run. Its 14.027--14.250 ms/op interquartile range did not
overlap the 14.733--14.927 ms/op control range. The final trace contains 141,208
events, 64,967 fewer than the original 206,175, while retaining the same 7,251
refined decisions and two explicit unknown outcomes.

### Use canonical scalar fact queries

The predicate boundary no longer defines `RealFacts` and `RealZeroKnowledge`
compatibility aliases for Hyperreal's `RealStructuralFacts` and
`ZeroKnowledge`. Evidence and geometry fact packets name the scalar-owned types
directly. `RealPredicateExt` now contains only predicate-specific sign mapping
and bounded refinement; its duplicate `real_facts` and `zero_knowledge` methods
were removed in favor of `Real::structural_facts` and `Real::zero_status`.

The zero-only displacement cascade benefits from the narrower inherent query:
two coordinate differences are classified without constructing full sign and
magnitude packets. The 40-sample `evidence_derivation/point2/displacement_facts`
sentinel improved from 134.77 ns to 110.92 ns, a 17.93% mean reduction with
Criterion's 95% change interval of -18.99% to -17.03%. The retained
`Point2DisplacementFacts` packet and every downstream segment, triangle, and
AABB filter remain unchanged.

```sh
cargo bench --bench predicates -- \
  'evidence_derivation/point2/displacement_facts' \
  --warm-up-time 1 --measurement-time 3 --sample-size 40
```

### Cascade projected triangle degeneracy by stage

`classify_triangle3_degeneracy` is now the canonical 3D collinearity predicate
used by Hypermesh plane validation. It previously cloned three pairs of
projected `Point2` values and eagerly ran all three orientation cascades before
inspecting any result. The predicate now borrows the original coordinates and
tries every projection at one escalation stage before advancing: certified
primitive determinant filters, word-sized exact rationals, arbitrary exact
rationals, and finally generic `Real` evaluation. A certified nonzero
projection exits immediately; all three projections must still be certified
zero before degeneracy is reported.

The 40-sample common XY-nondegenerate sentinel improved from 436.90 ns to
33.72 ns, a 92.26% mean reduction with Criterion's 95% change interval of
-92.33% to -92.20%. Hypermesh removed its separate 80-line nondegeneracy
cascade and now consumes this result directly.

```sh
cargo bench --bench predicates -- \
  'hypermesh_port_helpers/triangle3_degeneracy/projected_orientations' \
  --warm-up-time 1 --measurement-time 4 --sample-size 40
```

### Batch exact ordered-AABB decisions

Retained spatial structures already maintain ordered 3D box endpoints, but
higher crates were rebuilding overlap, containment, and relative-interior
tests from six to nine independent scalar predicates. Hyperlimit now owns
`ordered_aabb3s_intersect`, `ordered_aabb3_contains`, and
`point_in_ordered_aabb3_relative_interior`. Their first cascade stage borrows
all coordinates and decides exact-rational boxes as one batch; unresolved
symbolic coordinates retain the existing per-comparison predicate pipeline.
The general, endpoint-normalizing 3D point and intersection classifiers use the
same exact-rational stage before falling through to their prior interval
cascade.

The general intersection row improved from 113.18 ns to a fresh 85.17 ns
(24.75%), and point classification improved from 58.95 ns to 54.98 ns (6.48%).
Full-axis ordered intersection, containment, and relative-interior decisions
measured 24.50 ns, 24.12 ns, and 29.94 ns respectively.

Downstream serialized gates found no Hypermesh regression: the 6,144-triangle
polygon-soup row moved by +0.68% (`p = 0.22`) and the 192-cell Boolean row by
-0.34% (`p = 0.69`). Hypervoxel's exact box and triangle-solid voxelization
rows improved by 4.89% and 3.89%. Focused Hypersdf AABB point/cell rows measured
238.72/252.83 ns, Hyperbrep face-AABB preflight measured 417.26 us, and
Hyperphysics exact AABB contact replay measured 535 ns. A 20-sample CSGRS warm
check remained at 1.019 us for a box and 75.633 us for a medium sphere.

```sh
cargo bench --bench predicates -- 'aabb_immediate/3d/' \
  --warm-up-time 1 --measurement-time 4 --sample-size 40
```

### Reuse point/ring edge orientations

`classify_point_ring_even_odd_report` formerly evaluated `orient2(a, b,
point)` once while classifying the point against every edge, then evaluated the
same determinant again for each y-straddling edge. The report requires the full
`OffLine` versus `CollinearOutside` distinction, so the Hormann--Agathos idea of
skipping most boundary predicates cannot be applied without weakening retained
evidence. The implementation now certifies orientation once, reuses its sign for
crossing parity, and invokes only the collinear interval classifier when the
sign is zero.

Focused benchmark:

```sh
cargo bench --bench predicates -- \
  'exact_rational_kernels/ring/even_odd_reports' \
  --warm-up-time 1 --measurement-time 3 --sample-size 50
```

| Variant | Mean per 512 queries | Change |
| --- | ---: | ---: |
| Recompute orientation on straddling edges | 634.59 us | baseline |
| Reuse certified orientation | 504.88 us | -20.41% |

Criterion reported the improvement as statistically significant (`p = 0.00`).
The focused ring tests include inside, outside, edge boundary, vertex straddle,
indexed topology, repeated closing vertices, source replay, and the retained
`CollinearOutside` edge classification.

### Reuse triangle/plane vertex sides across edge reports

The 3D triangle/triangle classifier first certifies all three vertices of each
triangle against the opposite supporting plane. Its non-coplanar path then
classifies all six edges against the opposite triangle. Previously every edge
classifier recomputed the two endpoint `orient3` signs, so each retained vertex
side was evaluated twice more. The Guigue--Devillers orientation decomposition
supports carrying those signs forward: the implementation now keeps both
three-element side arrays, passes each edge's certified endpoint pair into the
unchanged segment/triangle tail, and reuses the retained supporting
plane for the exact crossing construction.

The same retained oriented-plane filters also replace a redundant second linear-
form preparation during the initial plane tests. Both coplanar and non-coplanar
sentinels improved:

| Workload | Before | After | Change |
| --- | ---: | ---: | ---: |
| Non-coplanar triangle report replay | 26.547 us | 20.609 us | -21.62% |
| Coplanar triangle report replay | about 20.07 us | 14.898 us | -25.76% |

Criterion reported both changes as statistically significant (`p = 0.00`). The
public report remains unchanged: all six non-coplanar edge relations are still
retained, and degeneracy, separation, boundary, proper crossing, and coplanar
tests replay successfully.

### Replace prepared 2D lines with immediate classification

`PreparedLine2` duplicated its endpoints inside a classifier handle even though
the reusable state is only orientation evidence. The public surface now exposes
owned `Line2Orientation` evidence and immediate
`classify_point_line_with_orientation` functions. Endpoints remain explicit at
each call, while the certified dyadic filter, exact-word filter, and scheduling
facts remain reusable.

The paired Criterion gate compared the old prepared method with the new
immediate function against an unchanged immediate control:

| Row | Before median | After median | Change |
| --- | ---: | ---: | ---: |
| Prepared-to-immediate transition | 16.064 ns | 16.108 ns | +0.27% |
| Unchanged immediate control | 15.802 ns | 16.231 ns | +2.72% |

The transition moved less than the control in the same serialized run, so
there is no normalized regression. Hypercurve also retains this owned
orientation evidence in its repeated containment paths rather than rebuilding
both filters for every query.

### Replace prepared lifted predicates with immediate evidence

`PreparedIncircle2` and `PreparedInsphere3` mixed borrowed source points with
owned filters and lifted-polynomial coefficients. The reusable state is now
owned by `Incircle2Evidence` and `Insphere3Evidence`; immediate query functions
take the ordered source points explicitly. The supporting public data types are
also named for what they contain: `PredicateFacts`, `LiftedPolynomialFacts`,
`Circle2Polynomial`, and `Sphere3Polynomial`.

The serialized paired Criterion gate compared each compatibility method with
the corresponding immediate evidence function, with an identical immediate
control in the same run:

| Predicate row | Before median | After median | Change |
| --- | ---: | ---: | ---: |
| In-circle transition | 34.641 ns | 34.243 ns | -1.15% |
| In-circle unchanged control | 34.667 ns | 34.176 ns | -1.42% |
| In-sphere transition | 75.544 ns | 76.095 ns | +0.73% |
| In-sphere unchanged control | 75.378 ns | 76.433 ns | +1.40% |

The in-circle transition tracked its control within 0.27%, while the in-sphere
transition moved 0.67% less than its control. Neither predicate regressed after
normalizing for same-run drift. Evidence construction is the former
compatibility constructor body without storing the four borrowed source
references, so the migration also removes handle state without adding work.

The 2026-07-27 substrate cleanup also replaced Hyperreal's remaining
preparation methods with direct `Incircle2Filter` and `Insphere3Filter`
constructors. Hyperlimit's evidence layout and immediate query surface are
unchanged.

| Row | Before median | After median | Result |
| --- | ---: | ---: | --- |
| In-circle retained query | 35.430 ns | 34.545 ns | improved |
| In-sphere retained query | 77.820 ns | 76.003 ns | improved |
| In-circle filter construction | 15.671 ns | 15.508 ns | no regression |
| In-sphere filter construction | 32.076 ns | 32.722 ns | within noise |
| In-circle evidence derivation | 1.736 us | 1.716 us | no regression |
| In-sphere evidence derivation | 3.399 us | 3.433 us | within noise |

### Remove the unused prepared halfspace cache

`PreparedHalfspaceSystem3` stored a borrowed plane slice and eagerly allocated
one `Plane3Facts` entry per plane, but feasibility never read those facts. Its
query method only forwarded the original slice to
`classify_halfspace_feasibility3`. The handle and its duplicate benchmark/fuzz
paths are removed; callers use the immediate report-bearing classifier.

Serialized paired Criterion runs covered both a feasible shifted box and an
infeasible opposed pair:

| Halfspace row | Before median | After median | Change |
| --- | ---: | ---: | ---: |
| Feasible transition | 13.878 us | 13.569 us | -2.22% |
| Feasible unchanged control | 13.803 us | 13.550 us | -1.83% |
| Infeasible transition | 1.892 us | 1.695 us | -10.42% |
| Infeasible unchanged control | 1.809 us | 1.696 us | -6.21% |

The transition improved slightly more than its same-run control in both cases.
Construction also loses the unused per-plane allocation and structural-fact
scan.

### Replace prepared planes with immediate evidence

`PreparedPlane3` mixed a borrowed source with coefficient facts and one
certified filter, while its segment, triangle, and AABB methods merely
forwarded to existing immediate functions. `PreparedOrientedPlane3` owned
useful derived state but exposed it through the same lifecycle abstraction.
They are replaced by `Plane3Evidence`, `OrientedPlane3Evidence`, evidence
derivation functions, and immediate point classifiers. Non-point queries use
their existing immediate APIs directly.

Serialized 100-sample Criterion comparisons retained the old benchmark ids
only for the paired gate:

| Plane row | Before median | After median | Change |
| --- | ---: | ---: | ---: |
| Explicit plane, easy batch | 13.767 us | 8.836 us | -35.82% |
| Explicit plane, near-degenerate batch | 12.797 us | 7.420 us | -42.02% |
| Oriented plane, easy batch | 19.980 us | 19.021 us | -4.80% |
| Oriented plane, near-degenerate batch | 20.345 us | 19.359 us | -4.85% |
| Explicit-plane evidence derivation | 798.89 ns | 816.58 ns | +2.21% |
| Dyadic oriented-plane evidence derivation | 1.362 us | 1.369 us | +0.54% |
| Rational oriented-plane evidence derivation | 1.660 us | 1.618 us | -2.53% |

The immediate evidence classifiers are inline at cross-crate boundaries, which
removes the former handle-call overhead. Criterion classified both positive
derivation movements as within its noise threshold and found no regression.
HyperBrep and HyperGraphics consumer gates are recorded in those crates.

### Normalize triangle/plane reports

The replayable triangle/plane result now follows the same public vocabulary as
`PlaneAabbReport`: `TrianglePlaneReport` retains the coarse relation and three
vertex sides, while `TrianglePlaneReportValidationError` describes structural
or source-replay mismatches. Obsolete README claims for removed versioned
prepared/session types were deleted.

The dedicated `hypermesh_port_helpers/triangle_plane/report_replay` sentinel
measured 899.03 ns before and 898.88 ns after the type-only normalization.
Criterion detected no performance change (`p = 0.13`, 95% interval -0.93% to
+0.07%).

### Use immediate checked-word `orient3`

Exact-rational one-shot orientation now tries Hyperreal's checked homogeneous
`i128` determinant before constructing arbitrary-precision rational
differences and products. Retained oriented-plane evidence uses the same
descriptive filter types directly; overflow and symbolic inputs still enter
the existing exact fallback.

Serialized 100-sample Criterion gates found no regression:

| Workload | Before | After | Change |
| --- | ---: | ---: | ---: |
| Exact-rational `orient3`, common denominator | 237.34 us | 139.99 us | -41.01% |
| Dyadic affine-det3 filter construction | 24.234 ns | 22.021 ns | -9.13% |
| Exact-word affine-det3 filter construction | 190.40 ns | 188.98 ns | -0.75% |
| Dyadic oriented-plane evidence | 1.3791 us | 1.3608 us | -1.33% |
| Exact-rational oriented-plane evidence | 1.6781 us | 1.6453 us | -1.95% |

The exact-rational orientation and dyadic construction improvements were
statistically significant; Criterion classified the other movements within
the configured noise threshold.

### Use direct linear-form filters

Explicit-plane evidence now constructs `LinearForm3Filter` directly, while
one-shot classification continues through `certified_linear_form3_sign`.
The unused rational-query method was removed from the retained dyadic carrier;
non-dyadic queries keep the existing exact fallback.

Serialized 100-sample Criterion gates found no regression:

| Workload | Before | After | Change |
| --- | ---: | ---: | ---: |
| Linear-form filter construction | 12.892 ns | 12.633 ns | -2.01% |
| Explicit-plane evidence derivation | 831.95 ns | 829.42 ns | -0.30% |
| Point/plane, immediate easy batch | 20.751 us | 20.878 us | +0.61% |
| Point/plane, immediate near-degenerate batch | 19.538 us | 19.246 us | -1.49% |
| Point/plane, retained easy batch | 8.9877 us | 8.8333 us | -1.72% |
| Point/plane, retained near-degenerate batch | 7.5186 us | 7.5470 us | +0.38% |
| Plane/segment dyadic composition | 142.88 us | 141.14 us | -1.22% |
| Plane/triangle dyadic composition | 45.859 us | 44.887 us | -2.12% |

Criterion classified the two small positive movements as noise.

### Reuse policy-certified nonzero construction evidence

Geometry construction now retains the same policy decision that established a
denominator as nonzero. Segment/plane and coplanar parameters, halfspace
active-set projections, affine conversion of three-plane intersections, and
ray/triangle crossings construct one reciprocal with
`inverse_ref_assuming_nonzero` instead of asking ordinary `Real` division to
repeat its bounded zero proof. The 2D line constructor exposes the same route
through `construct_line_intersection_point_with_policy`; the original API
remains a strict-policy wrapper. HyperTri passes its evaluator policy through
that boundary.

Regressions use a positive expression equal to `2^-3000` whose structural zero
status and ordinary inverse remain undecided, while the strict exact-normal
stage proves it nonzero. The affected constructions now succeed exactly;
independent unsupported transcendental zero controls still return `Unknown` or
construction failure. The shared adversarial fixture also replaced four
duplicate module-local copies and the inline segment/plane copy.

Pinned same-machine Criterion A/B runs retained or improved ordinary exact-
rational performance:

| Workload | Legacy division | Retained policy reuse | Change |
| --- | ---: | ---: | ---: |
| Segment/plane determinant ratio | 1.8184 us | 1.7430 us | -4.15% |
| Halfspace feasibility active sets | 16.416 us | 15.945 us | -2.25% |
| Ray/triangle report replay | control distribution | 1.3698 ms | no detectable change (`p = 0.74`) |

## Rejected experiments

### Merge rational queries into `LinearForm3Filter::sign`

Folding exact-rational conversion into the retained filter's public `sign`
method made the representation surface smaller, but inlining the fallback
inflated the dyadic evidence loop by 52--76%. Moving the rational work behind a
cold non-inlined helper still left retained easy and near-degenerate batches
9.5% and 12.9% slower, with one-shot batches also 2--3% slower. The experiment
was rejected. The final API removes the unused rational-specific method and
keeps the hot retained filter explicitly dyadic.

### Search short Farkas certificates before active sets

After the origin candidate failed, an experimental schedule searched all one-
and two-plane Farkas certificates before constructing the remaining geometric
active-set candidates. It quickly handled opposed slabs but imposed quadratic
proof-search work on the shifted feasible case.

| Variant | Mean per mixed feasible/infeasible query | Change |
| --- | ---: | ---: |
| Existing active-set-first schedule | 16.51 us | baseline |
| Early one/two-plane certificate search | 18.37 us | +10.69% |

Criterion reported a significant regression (`p = 0.00`), so the experiment
was removed completely.

### Reorder k-DOP axes

Klosowski et al. observe that testing widely separated directions in sequence
may improve early exits, but explicitly leave a specially designed ordering as
future work and provide no evaluated order. `SupportDop3` also preserves caller
slab order and reports terminal source indices. No speculative reorder was
introduced.

## Reference-to-implementation audit

| Reference | Relevant idea | Result in `hyperlimit` |
| --- | --- | --- |
| Arvo, *Transforming Axis-Aligned Bounding Boxes* | Select min/max affine contributions rather than transform every corner. | `classify_plane_aabb3_report` already performs the corresponding exact term-interval reduction and retains selected-corner evidence. A general affine box-transform carrier is outside this crate's present API. |
| Bareiss, integer-preserving elimination | Fraction-free cubic determinant evaluation with controlled intermediate growth. | Already used by the exact-rational `orient_d` and `insphere_d` paths. |
| Bentley--Ottmann, geometric intersections | Event queue plus ordered sweep status gives output-sensitive batch segment intersection. | This crate supplies the exact segment predicates needed by a sweep. Arrangement/event ownership belongs in `hypercurve` or `hypertri`, not a per-pair predicate API. |
| de Berg et al., *Computational Geometry* | Robust plane sweep, randomized low-dimensional LP, convex hull, point location, and ownership-aware planar subdivisions. | Confirms the separation between exact primitive decisions here and topology/data structures in higher crates. The randomized LP alternative is covered below. |
| Ericson, *Real-Time Collision Detection* | Prepared bounding volumes, separating axes, early rejection, degeneracy handling, and robustness. | Explicit evidence for lines and planes, immediate segment/circle/sphere predicates, DOPs, and exact interval/SAT-style predicates implement these principles without epsilon decisions. |
| Guigue--Devillers, triangle overlap | Boolean triangle overlap using orientation signs only, minimizing intermediate constructions and `orient3` calls. | The public triangle/triangle report cannot adopt the paper's boolean-only output, but it now reuses the six initial vertex/plane signs across all edge reports, producing the 21.62% non-coplanar improvement above without discarding evidence. |
| Gustavson, sparse matrix algorithms | Row-wise sparse multiplication through an unordered accumulator/merge. | Hyperlimit determinants are tiny dense matrices with structural sparse-coordinate schedules, not general SpGEMM. Introducing sparse matrix storage would add overhead at present dimensions. |
| Hormann--Agathos, point in polygon | Half-open y-straddles, determinant-based crossings, integrated boundary handling, and cheap rejection before division. | Half-open straddles and exact orientation crossings were already present. Reusing the retained edge orientation produced the measured 20.41% improvement above. |
| Moore, *Interval Analysis* | Inclusion-preserving interval enclosures can certify results that exclude zero. | `certified_interval_sign`, certified balls, and determinant filters already use enclosures only as proofs; intervals crossing zero escalate rather than guess. |
| Möller, triangle intersection | Plane-side rejection, line-of-intersection projection on the largest component, then one-dimensional interval overlap. | Hyperlimit uses exact plane classifications and projection-aware segment/triangle composition, but also handles degeneracy and coplanarity and preserves replayable reports that Möller's fast boolean path omits. |
| Klosowski et al., k-DOP BVHs | Fixed-direction support intervals, early separation, tightness/cost tradeoffs, and hierarchy construction. | Exact witnessed support slabs and conservative overlap reports are implemented. BVH construction and temporal updates belong to higher crates. Axis-order speculation was rejected above. |
| Seidel, low-dimensional LP | Randomized incremental LP with recursive boundary subproblems and expected linear time for fixed dimension. | Current feasibility reports are deterministic and preserve exact witnesses or Farkas certificates. Seidel requires canonical lexicographic degeneracy handling and randomized scheduling; its paper also notes the implementation is not necessarily practical. No workload currently justifies replacing the exact active-set solver. |
| Schrijver, linear/integer programming | Polyhedral feasibility, duality, and Farkas certificates. | Infeasible 3D halfspace reports search support sets of at most four planes and replay exact nonnegative multiplier certificates. The attempted certificate reschedule regressed and was removed. |
| Shewchuk, adaptive robust predicates | Fast filters followed by exact/adaptive stages, with degeneracy decided exactly. | Dispatch tracing sends the standard 512-case easy and near-degenerate `orient2d`, `orient3d`, `incircle2d`, and `insphere3d` workloads through the certified Real filter with no fallback traffic. An additional expansion stage is not supported by current traces. |
| Yap, exact geometric computation | Separate combinatorial decisions from numeric approximation and refine only when certification requires it. | `PredicateOutcome`, certainty/escalation metadata, retained exact evidence, and replayable reports follow this architecture throughout the crate. |

## Trace evidence

The generated `dispatch_trace.md` standard workload contains 512 easy and 512
near-degenerate cases for each core predicate. All terminate at the certified
Real filter; no exact-rational, adaptive-refinement, or unknown fallback route
is exercised by those workloads. Transformed exact-rational cases separately
exercise the Bareiss and lifted determinant routes.

The triangle/triangle trace separately records
`reuse-plane-sides-for-edges`, making the retained Guigue--Devillers scheduling
choice distinguishable from the coplanar projection and early plane-separation
paths.

## Verification

The retained implementation is checked with:

```sh
cargo test --all-targets --all-features
cargo test --all-targets --no-default-features
cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --no-deps --all-features
cargo check --examples --benches --all-features
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo bench --all-features --bench predicates --no-run
cargo run --release --all-features --example allocation_profile
scripts/representation_coverage.sh
scripts/coverage.sh
cargo fmt --all -- --check
git diff --check
```
