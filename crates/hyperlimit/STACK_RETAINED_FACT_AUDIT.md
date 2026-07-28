# Stack Retained-Fact Audit

Audit date: 2026-07-28

## Scope and method

This audit follows retained numerical structure from `hyperreal::Real` through
Hyperlattice, Hyperlimit, and Hypercurve. It also checks the Hypermesh/CSGRS
adapter boundary where exact support-plane evidence crosses into Boolean
topology.

The inventory combined:

- a declaration scan for `Fact`, `Evidence`, `Report`, `Retained`, `Cache`,
  `Witness`, `Certificate`, `Prepared`, `Classification`, `Summary`, `Query`,
  `Filter`, `Plan`, `Schedule`, `Status`, `Relation`, `Decision`, and `Proof`;
- the workspace call graph (33,943 nodes and 57,714 edges before the retained
  changes);
- construction and query-path tracing for every owner named by the scan;
- size-budget tests and owner-layout inspection;
- serial tests, benchmarks, and focused profiling.

The declaration scan found 49 Hyperreal, 20 Hyperlattice, 64 Hyperlimit, and
169 Hypercurve declarations. The registry at the end records every hit.
Semantic owners whose names do not contain one of those words, such as
`Real`, `RationalData`, `CurveData2`, `CurvePathData2`,
`ExactCurveWorkspace2`, and `CurveRegion2`, were inspected separately.
The post-change graph spans all six layers through CSGRS: 51,233 function
nodes and 90,824 edges. It confirms the retained support query at the
CSGRS/Hypermesh boundary and contains no removed prepared line/arc fact-owner
path.

## Retention rule

The stack now follows one consistent ownership rule:

1. Scalar facts that are intrinsic, monotonic, and frequently queried belong
   with the scalar representation.
2. Fixed-input predicate preparation belongs in a reusable evidence/query
   object when it owns filters or expensive derived coefficients.
3. Query-dependent decisions, proof trails, and validation results remain
   returned reports. They are not attached to immutable geometry.
4. Aggregate facts are cached only by an owner that already shares lazy
   storage and has demonstrated repeated demand.
5. Inline geometry does not grow for a speculative cache. A missing fact may
   select a slower exact route, but it must never change a topology decision.

This distinction is important: facts schedule a cascade; evidence owns a
reusable stage of that cascade; reports describe one completed query.

## Hyperreal

### Retained with the scalar

`Real` keeps its exact `Rational`, compact symbolic `Class`, optional
`Computable`, and atomic primitive-approximation cache. The 48-byte hard size
limit remains intact.

The shared `Computable::Node` packs bound and exact-sign knowledge into one
`AtomicU64`. Its approximation cache is one null pointer until first use, then
publishes one synchronized best approximation. This retains:

- structural zero/nonzero, sign, magnitude, and exact-sign knowledge;
- the finest computed approximation;
- abort-safe cache invalidation;
- clone sharing for opaque expression graphs.

`RationalData` retains monotonic reuse, exact-binary64, dyadic, conflict, and
unreduced-state bits in one `AtomicU32`. Its product cache, compact lazy linear
cache, unary/square reductions, and small-rational interning avoid repeating
normalization and arithmetic. Its 88-byte hard size limit remains intact.

The public fact surfaces deliberately split cost:

- `RealStructuralFacts` is the cheap hot query;
- `RealDetailedFacts` adds identity, rational-storage, primitive-range,
  ordering, domain, and symbolic facts only on request;
- `RealExactSetFacts` scans a borrowed set once and carries common-scale,
  dyadic, integer-grid, signed-unit, sparsity, sign-pattern, and storage-class
  scheduling facts upward.

### Cascades that consume the facts

The determinant, linear-form, line, incircle, and insphere filters are
storage-free or compact fixed-input packages. Their normal order is:

1. retained scalar class/zero/sign facts;
2. primitive or dyadic certified filter;
3. exact-word or shared-denominator filter;
4. arbitrary-rational kernel;
5. general `Real` expression and bounded exact refinement.

The dyadic product-sum and line-intersection plans preserve compact numerators,
scales, and parameter comparisons instead of expanding into generic expression
graphs. Certificates are returned decisions, not cached scalar state.

### Disposition

Keep the current split. Collapsing detailed public facts into `Real` would
duplicate values derivable from the existing compact class and exceed the
48-byte scalar budget. Attaching query certificates would be incorrect because
they depend on another value and a refinement policy. The packed monotonic
bits and lazy node caches are the appropriate scalar-level carriers.

## Hyperlattice

Points, vectors, and matrices are inline `Real` aggregates. Their
`Point*Facts`, `Vector*Facts`, and `Matrix*StructuralFacts` are immediate
by-value packets. They combine `RealExactSetFacts` with sparse masks, signed
axis/permutation facts, affine/diagonal/triangular shape, and symbolic
dependencies.

The positive owning model is `SharedScaleVec<N>`: construction proves a common
exact scale once, stores exactly one `VectorSharedScaleFacts<N>`, and every
borrowed view reuses it. No second exact-set summary is retained.

Matrix operations build private `Matrix3Facts`, `Matrix4Facts`,
`Matrix4TransformDispatchFacts`, and identity/diagonal facts once per operation
and pass them through determinant, inverse, division, and transform cascades.
The hard test permits only 32 bytes beyond the public structural packet, which
prevents a duplicate `RealExactSetFacts` (96 bytes on the validation target).

### Evaluated collapses

- **Generic point/vector cache: rejected.** It would enlarge every inline
  carrier and there is no stable shared owner. Repeated callers should use a
  shared-scale owner or an operation-local prepared object.
- **Matrix cache: rejected.** Matrix facts are operation-specific scheduling
  packets; adding shared allocation or interior mutability would penalize
  dense transforms and clones. Current kernels already reuse one packet within
  an operation.
- **Shared-scale fact collapse: retained.** One owning fact packet is reused by
  views and exact fused algebra with no rescan.
- **`ProjectivePlane3`: not a fact carrier.** It is a scanner-name collision
  (`Plan` in `Plane`) and remains ordinary geometry.

## Hyperlimit

### Immediate geometry facts

`Point2DisplacementFacts`, `Segment2Facts`, `Triangle2Facts`, `Aabb2Facts`,
`Plane3Facts`, `Ring2Facts`, `PredicateFacts`, and `LiftedPolynomialFacts` are
small, copyable scheduling packets. They are accepted explicitly by the
`*_with_facts` APIs so a caller that already scanned fixed geometry does not
repeat that work.

They drive early cascade branches for:

- degenerate point/segment/triangle and AABB dimension cases;
- exact-rational, dyadic, shared-denominator, and exact-kernel schedules;
- sparse plane coefficients and zero/nonzero support;
- fixed-coordinate predicate filters;
- lifted-polynomial zero patterns.

### Reusable evidence

`Plane3Evidence`, `OrientedPlane3Evidence`, `Incircle2Evidence`, and
`Insphere3Evidence` are the correct higher owner for repeated queries.

- Plane evidence owns `Plane3Facts` plus a fixed linear-form filter.
- Oriented-plane evidence also owns the reduced explicit plane and exact-word
  determinant filter.
- Circle/sphere evidence owns fixed-point facts, certified filters, lifted
  coefficients, and coefficient facts.

The evidence benchmarks show why these remain explicit:

| Repeated query | Immediate | Retained evidence | Result |
| --- | ---: | ---: | ---: |
| explicit point/plane, easy | 20.86 us | 8.83 us | 2.36x faster |
| explicit point/plane, near-degenerate | 19.21 us | 7.53 us | 2.55x faster |
| oriented point/plane, easy | 37.18 us | 19.12 us | 1.94x faster |
| incircle, easy | 23.19 us | 16.86 us | 1.38x faster |
| insphere, easy | 64.48 us | 40.56 us | 1.59x faster |

`Line2Orientation` follows the same pattern and is consumed by Hypercurve's
prepared line/arc classifiers.

### Reports, witnesses, and certificates

The plane/AABB, triangle/plane, segment/triangle, ray/triangle, ring,
halfspace, and support-DOP report families retain one query's exact extrema,
signs, sides, witnesses, blockers, and replay validation. They remain returned
values because their data depends on both operands and often on orientation or
policy.

`WitnessedSupportDop3` is an intentional owning exception: its support
witnesses are the DOP representation used by refresh and plane/AABB queries,
not an incidental report attached to unrelated geometry.

### Evaluated collapses

- **Facts inside `Plane3`: rejected.** A plane is four inline `Real` values;
  attaching a full packet or lazy pointer taxes every one-shot predicate.
  Repeated users already have `Plane3Evidence`, with measured benefit.
- **Evidence inside source points/planes: rejected.** Oriented-plane,
  incircle, and insphere evidence is ordered and multi-object. No single source
  object owns it semantically.
- **Reports inside geometry: rejected.** Reports are operand- and
  query-dependent and already expose `validate`/source-replay checks.
- **Evidence/fact carrier combination: retained where valid.** Plane and
  circle/sphere evidence already contain the corresponding fact packets and
  filters, so callers do not need parallel carriers.

## Hypercurve

### Compact native carriers

`Point2` is one pointer and `Point2Data` is exactly two `Real` values.
`LineSeg2` and `Segment2` have a 48-byte ceiling. A line already retains
constructor-discovered endpoint distinctness, optional shared support,
fragment-support cache, direction provenance, and offset provenance.

These size constraints reject inline structural-fact caches for points and
lines. Their facts remain immediate by-value queries.

`CircularArc2` is different: it is already one pointer to
`CircularArcRetainedFacts2`, which owns geometry plus lazy sweep,
decomposition, representative point, angle, parameter-lineage, inverse
witness, and fragment caches. `CircularArc2Facts` now uses one additional lazy
pointer in that existing allocation and allocates the fact packet only on
first request. The one-word arc handle and 48-byte `Segment2` ceiling are
unchanged.

### Prepared queries

`PreparedLineSeg2`, `PreparedCircularArc2`, `PreparedSegment2`,
`ContourQuery2`, and `RegionQuery2` own fixed-input Hyperlimit evidence and
conservative AABB/winding indexes for repeated topology queries.

The prepared line and arc objects previously also stored
`LineSeg2Facts`/`CircularArc2Facts`, but used those fields only in
`PartialEq`. Geometry equality already implies equal structural facts, while
`Line2Orientation` retains the facts and filters actually consumed by the
classifier. The duplicate fields and their construction scans were removed.

Kind-only topology/report paths now call `Segment2::kind()` directly instead
of constructing a full `Segment2Facts`. This applies to curve-string
intersection evidence and every exact region-arrangement source, blocker, and
output-provenance record.

### Higher owners

The expensive carriers already follow the clone-shared lazy model:

- `RationalBezierData` retains homogeneous controls/power basis, derivative
  numerators, monotonicity, injectivity, exact line/conic/circle promotions,
  degree elevations, contacts, and arrangement topology.
- `PolynomialSplineData2` retains authored source identity, endpoint
  semantics, Bezier decomposition, and rational spans.
- `NurbsData2` additionally retains native subcurves plus bounded knot
  refinements/removals and degree elevations.
- `CurveData2` retains parameter domain, native fragments, rational
  evaluators, bounds, and root-lineage injectivity.
- `CurvePathData2` retains flattened native fragments, boundary-loop
  materialization, and bounds.
- `CurveRegion2` retains certified loop roles/fill rules plus shared lazy
  filled-side, native-loop, bounds, line-image, rational-evaluator, and signed
  area results.

Bezier control-point fact packets are currently public/tests-only scheduling
surfaces. Converting the inline quadratic, cubic, or rational-quadratic
carriers to shared allocations merely to cache them was rejected: there is no
production repeated-fact demand, and the general rational/spline/top-level
owners already cache the expensive derived structures that their algorithms
consume.

### Arrangement evidence

`ExactCurveWorkspace2` is an evaluated-operation owner, not ordinary geometry.
Its source, AABB, kind, endpoint, split-schedule, intersection, endpoint-graph,
ring-assembly, output, status, provenance, and role caches are built
opportunistically as each stage runs. `RegionArrangement2` shares that
workspace and exposes a compact `RegionArrangementSummary2`; the detailed
facts remain queryable without a second report lifecycle.

Keeping these caches together is preferable to attaching them to individual
segments: they encode pair identities, output indices, nesting depth, and
operation status. Splitting them among geometry objects would duplicate source
and index data and make invalidation ambiguous.

## Hypermesh and CSGRS boundary

CSGRS remains a thin grammar/conversion layer over Hypercurve and Hypermesh.
It must preserve exact evidence when available without synthesizing
Hypermesh-specific packets for every ordinary mesh.

`Mesh::boolean_operation` previously forced a complete
`InputTrianglePlanes` array for both operands before every first Boolean. The
adapter now:

- forwards already-retained transform-derived support planes;
- derives only the missing operand when the other operand already requires the
  aligned plane-aware API;
- otherwise calls Hypermesh's ordinary certified-convex triangle-soup API and
  lets Hypermesh own native preparation.

The same audit corrected the competitive fixture: CSGRS now receives the exact
4,512-triangle subdivision of the 1,128-triangle certified hull, rather than
reconstructing a noisy convex hull from serialized subdivided coordinates.
Competitive output conversion also exports only positions/indices, matching
the peer contract instead of computing exact unit normals.

The original full-resolution YeahRight control mesh remains a hard contract at
both levels. It has 5,687 vertices, 5,845 source polygons, 11,894 fan
triangles, and genus 131. CSGRS and Hypermesh always import and validate it;
both retain an ignored rotated-copy Boolean as the explicit memory-ceiling
test.

## Experiments and performance gate

| Experiment | Memory result | Serial performance | Disposition |
| --- | --- | --- | --- |
| Cache `CircularArc2Facts` in existing arc owner | +1 lazy pointer per shared arc allocation; fact box only on demand; handle unchanged | replay 470 ns -> 3-5 ns, about 117x | Keep |
| Cache `Point2Facts` | would violate one-pointer handle / exact two-`Real` data layout | no production repeated demand | Reject |
| Cache `LineSeg2Facts` | would exceed or consume the 48-byte line/segment ceiling | no production repeated demand | Reject |
| Cache inline Bezier facts | requires larger inline values or new shared allocation | facts are public/tests-only in call graph | Reject |
| Retain duplicate facts in prepared line/arc | one large aggregate packet per prepared segment | construction scan removed; neighboring arc/region rows unchanged | Remove |
| Replace kind-only full fact scans | no memory change | removes scalar-set scans from evidence assembly | Keep |
| Attach Hyperlimit facts to `Plane3` | permanent per-plane growth | evidence path is already 1.9-2.6x faster for repeated queries | Reject |
| Attach query reports to geometry | potentially unbounded/query-specific | would not improve the predicate cascade | Reject |
| Hyperlattice generic inline caches | permanent point/vector/matrix growth | operation packets already amortize scans | Reject |
| `SharedScaleVec` single retained packet | one intentional packet, no duplicate summary | views and fused exact algebra avoid rescans | Keep |
| CSGRS eager support-plane synthesis | one complete plane array and exact normal work per first Boolean | 4,512-face first union 77.66 ms -> 21.20 ms across staged fixes | Remove |
| Correct exact CSGRS YeahRight handoff/output contract | no duplicate noisy hull or output normals | false 18.3 s row -> 20.45 ms final serialized union / 13.79 ms repeated; Hypermesh native 10.3-10.7 ms | Keep |

After the Hypercurve changes, neighboring arc rows remained stable:
major containment 9.35 us, top-level evaluation 7.19 us, inverse-witness replay
1.56 us, sparse 256-arc path intersection 1.11 ms, and 256-arc region
containment 0.992 ms.

Full-resolution import remains measured at both boundaries:

| Engine | CSGRS harness | Hypermesh Criterion |
| --- | ---: | ---: |
| CSGRS wrapper / Hypermesh native | 16.72 ms | 2.90-3.00 ms |
| boolmesh | 5.75 ms | 7.98-8.23 ms |
| manifold-rust | 5.17 ms | 5.70-5.75 ms |
| tri-mesh | 1.54 ms | 1.89-1.90 ms |

The wrapper row includes CSGRS object construction and exact conversion; the
native row measures Hypermesh import directly. The dangerous 11,894 by 11,894
rotated Boolean is not part of routine gates because it previously reached
about 116 GiB RSS.

## Complete declaration registry

The list below is the exact declaration-name scan. `Plane3`,
`ProjectivePlane3`, and some `Classification`/`Relation` names are included
because the audit intentionally reviewed semantic decisions as well as names
that literally store facts.

### Hyperreal (49)

- `computable/node/bounds.rs`: `BoundCache`, `ExactSignCache`
- `computable/node/representation.rs`: `ApproximationCache`, `AtomicFacts`, `CachedApproximation`
- `dispatch_trace.rs`: `CommonFactorBuckets`, `LayerSummary`, `OperationSummary`, `TraceCorrelationSummary`
- `rational/arithmetic/aggregate_products.rs`: `DyadicLineIntersectionPlan`, `DyadicProductSumPlan`, `DyadicWideLineIntersectionPlan`
- `rational/arithmetic/barrett_division.rs`: `PreparedBarrettDivisor`
- `rational/arithmetic/representation.rs`: `CachedRationalArithmetic`, `CachedRationalLinearEntry`, `CachedRationalLinearKind`, `CachedRationalProduct`, `CachedRationalSquareReduction`, `CachedRationalUnary`
- `real/arithmetic/classification.rs`: `AtomicPrimitiveApproxCache`, `PrimitiveApproxCache`
- `real/arithmetic/linear_algebra.rs`: `AffineDet2ExactWordFilter`, `AffineDet2Filter`, `AffineDet2PairFilter`, `AffineDet3ExactWordFilter`, `AffineDet3Filter`, `Incircle2Filter`, `Insphere3Filter`, `LinearForm3Filter`, `RationalLine2Filter`, `RationalLinearForm4Filter`, `RationalLinearForm4Query`, `RationalPoint3Query`
- `real/exact_set.rs`: `RealExactSetFacts`
- `structural.rs`: `DomainFacts`, `DomainStatus`, `IdentityFacts`, `OrderingFacts`, `PrimitiveFacts`, `PrimitiveFloatStatus`, `RationalFacts`, `RealDetailedFacts`, `RealEqualityCertificate`, `RealOrderingCertificate`, `RealSignCertificate`, `RealStructuralFacts`, `SymbolicFacts`, `ZeroOneMinusOneStatus`, `ZeroOneStatus`

### Hyperlattice (20)

- `algebra2.rs`: `Displacement2Facts`, `Orient2Facts`, `ProductSum2Facts`, `ProductTerm2Facts`
- `matrix/core.rs`: `Matrix3Facts`, `Matrix3StructuralFacts`, `Matrix4Facts`, `Matrix4StructuralFacts`, `Matrix4TransformDispatchFacts`, `MatrixDeterminantScheduleHint`, `MatrixIdentityDiagonalFacts`
- `point.rs`: `Point2Facts`, `Point3Facts`, `PointSharedScaleFacts`
- `projective.rs`: `ProjectivePlane3`
- `vector.rs`: `Vector2Facts`, `Vector3Facts`, `Vector4Facts`, `Vector4GeometricFacts`, `VectorSharedScaleFacts`

### Hyperlimit (64)

- `batch.rs`: `PointPlaneCase`
- `classify.rs`: `CircleLineRelation`, `CircleSegmentRelation`, `PlaneAabbRelation`, `PlaneSegmentRelation`, `PlaneSide`, `PlaneTriangleRelation`, `SupportDopPlaneRelation`, `SupportDopRelation`
- `geometry/facts.rs`: `Aabb2Facts`, `Point2DisplacementFacts`, `Segment2Facts`, `Triangle2Facts`
- `geometry/plane.rs`: `OrientedPlane3Evidence`, `Plane3`, `Plane3Evidence`, `Plane3Facts`, `PlaneAabbReport`, `PlaneAabbReportValidationError`, `TrianglePlaneRelation`, `TrianglePlaneReport`, `TrianglePlaneReportValidationError`
- `predicate.rs`: `DeterminantScheduleHint`
- `predicates/aabb.rs`: `DecisionTrace`, `OrderedAabb3Relation`, `UnknownDecision`
- `predicates/coplanar.rs`: `CoplanarTriangleClassification`, `CoplanarTriangleRelation`
- `predicates/dop.rs`: `PlaneQuerySide`, `SupportDopAabb3Report`, `SupportDopAabb3SlabReport`, `SupportDopExpansionReport`, `SupportDopPlane3Report`, `SupportDopPlane3ValidationError`, `SupportDopRefreshReport`, `SupportWitness3`, `WitnessedSupportDop3`, `WitnessedSupportSlab3`
- `predicates/filters.rs`: `BallFilterResult`
- `predicates/halfspace.rs`: `CertificateSearch`, `HalfspaceFeasibilityReport`, `HalfspaceInfeasibilityCertificate`
- `predicates/interval.rs`: `DecisionTrace`, `UnknownDecision`
- `predicates/orient.rs`: `Incircle2Evidence`, `Insphere3Evidence`, `LiftedPolynomialFacts`, `PredicateFacts`
- `predicates/ring.rs`: `DecisionTrace`, `Ring2Facts`, `RingEvenOddEdgeReport`, `RingEvenOddReport`, `UnknownDecision`
- `predicates/segment.rs`: `DecisionTrace`, `UnknownDecision`
- `predicates/segment_plane.rs`: `SegmentPlaneConstructionFailure`, `SegmentPlaneEventConstruction`, `SegmentPlaneIntersection`, `SegmentPlaneParameterRatio`, `SegmentPlaneRelation`, `SegmentPlaneValidationError`
- `predicates/triangle.rs`: `RayTriangleIntersectionReport`, `SegmentTriangleIntersectionReport`
- `predicates/triangle_triangle.rs`: `TriangleTriangleClassification`

### Hypercurve (169)

- `bezier_algebraic_image.rs`: `BezierAlgebraicImageStatus`, `RetainedRationalPointExpression`, `RetainedRationalPointParametricSource`
- `bezier_arrangement.rs`: `RetainedAlgebraicDerivativeSource`, `RetainedEndpointData`, `RetainedEndpointKey`, `RetainedEndpointScope`, `RetainedEndpointSideData`, `RetainedEndpointStartIndex`, `RetainedTangentVector`
- `bezier_fit.rs`: `BezierFitCertificate`, `BezierLineFitRelation`, `BezierLineImageFitRelation`, `BezierPointFitRelation`, `BezierPointImageFitRelation`
- `bezier_flatten.rs`: `BezierFlatteningCertificate`
- `bezier_moment.rs`: `RationalQuadraticAreaIntegralCache`
- `bezier_parameter.rs`: `RepeatedRootEvidence`, `RetainedRationalBezierAlgebraicImageCurve`, `RetainedRationalBezierAlgebraicImages`, `SturmPointEvidence`
- `bezier_region.rs`: `CurveRegionCertifiedParallelLoopEvidence2`, `CurveRegionCertifiedParallelOffsetEvidence2`, `CurveRegionCertifiedSegmentationEvidence2`, `CurveRegionLineRoleEvidence2`, `CurveRegionNestingRoleEvidence2`, `CurveRegionSegmentationLoopEvidence2`, `CurveRegionSegmentedOffsetEvidence2`, `CurveRegionSignedAreaRoleEvidence2`, `RetainedEndpointEquality`, `RetainedEndpointEvidence`, `RetainedLineFragmentEndpoints`, `RetainedLineFragmentSource`, `RetainedLineLoopContour`, `RetainedLoopRoleDecision`
- `bezier_retained_measure.rs`: `BezierRetainedCurveEnvelope2`, `BezierRetainedEndpointEnvelope2`, `BezierRetainedEnvelopeSourceKind`
- `bezier_retained_overlap.rs`: `BezierRetainedLineOverlapSplit2`, `BezierRetainedLinearOverlapSplit2`, `BezierRetainedLinearOverlapSplitGraph2`, `BezierRetainedLinearOverlapTraversal2`, `BezierRetainedOverlap2`, `BezierRetainedOverlapEvidence2`, `BezierRetainedOverlapExtent2`, `BezierRetainedOverlapOrientation2`, `BezierRetainedOverlapRefinedFragment2`, `BezierRetainedOverlapRelation2`, `BezierRetainedOverlapTraversal2`, `BezierRetainedRationalOverlapSplit2`, `BezierRetainedRationalOverlapSplitGraph2`, `BezierRetainedRationalOverlapTraversal2`, `BezierRetainedResolvedLinearOverlap2`, `BezierRetainedResolvedRationalOverlap2`
- `bezier_tangent_order.rs`: `AlgebraicPowerEvidence`, `BezierAlgebraicSameTangentOrderEvidence`, `BezierAlgebraicSameTangentOrderStatus`, `BezierAlgebraicScalarSignEvidence`, `BezierAlgebraicTangentOrderEvidence`, `BezierAlgebraicTangentOrderStatus`, `BezierAlgebraicTangentVectorEvidence`, `BezierAlgebraicTangentVectorStatus`, `ScalarSignStatus`
- `bezier_topology.rs`: `BezierCurveRelation`, `BezierCuspClassification`, `BezierInflectionClassification`, `BezierLineContactRelation`, `BezierLineRelation`, `DyadicAxisPlan`
- `boolean.rs`: `BooleanFragmentClassification`, `FragmentInteriorClassification`
- `bspline.rs`: `RationalBSplineNativeTopologyEvidence2`, `RationalBezierSpanTopologyEvidence2`, `RetainedBSplineSpanFactEvidence2`, `RetainedBSplineSpanFacts2`, `RetainedSpanAxisMonotonicity`, `RetainedSpanWeightDomainEvidence2`
- `classify.rs`: `Classification`
- `contour.rs`: `RetainedContourOffsetRelation2`
- `curve_string.rs`: `CurveStringXOverlapSchedule`
- `events.rs`: `DenseAabbRankSchedule`
- `facts.rs`: `Bezier2Facts`, `CircularArc2Facts`, `CurveStringFacts`, `LineSeg2Facts`, `Point2Facts`, `RationalQuadraticBezier2Facts`, `RegionFacts`, `Segment2Facts`
- `intersect.rs`: `CertifiedLineSegmentSupportRelation`, `CircleCircleRelation`, `LineCircleRelation`
- `nurbs.rs`: `Cached`
- `polynomial_spline.rs`: `Cached`
- `prepared.rs`: `ContourQuery2`, `PreparedCircularArc2`, `PreparedLineSeg2`, `PreparedLineWindingIndex`, `PreparedSegment2`, `RegionQuery2`
- `rational_bezier_general.rs`: `RationalBezierIntersectionPointEvidence2`
- `region_boolean.rs`: `BoundaryContainmentRelation`
- `region_nesting.rs`: `ExactCurveArrangementArrangedEndpointBucketCache2`, `ExactCurveArrangementArrangedEndpointDegreeBucketCache2`, `ExactCurveArrangementArrangedEndpointPointCache2`, `ExactCurveArrangementArrangedEndpointSideBucketCache2`, `ExactCurveArrangementArrangedFragmentCache2`, `ExactCurveArrangementArrangedFragmentKindBucketCache2`, `ExactCurveArrangementArrangedFragmentSourceRangeCache2`, `ExactCurveArrangementArrangedFragmentStatusBucket2`, `ExactCurveArrangementArrangedFragmentStatusBucketCache2`, `ExactCurveArrangementArrangedFragmentStatusRef2`, `ExactCurveArrangementEndpointGraphCache2`, `ExactCurveArrangementOutputBoundaryCache2`, `ExactCurveArrangementOutputBoundaryRoleBucketCache2`, `ExactCurveArrangementOutputCache2`, `ExactCurveArrangementOutputRingBucketCache2`, `ExactCurveArrangementOutputRingContinuityCache2`, `ExactCurveArrangementOutputRoleCache2`, `ExactCurveArrangementOutputRoleContainmentBucketCache2`, `ExactCurveArrangementOutputRoleNestingDepthBucketCache2`, `ExactCurveArrangementOutputRoleSourceContourBucketCache2`, `ExactCurveArrangementOutputRoleStatusBucket2`, `ExactCurveArrangementOutputRoleStatusBucketCache2`, `ExactCurveArrangementOutputRoleStatusRef2`, `ExactCurveArrangementOutputSegmentDirectionBucketCache2`, `ExactCurveArrangementOutputSegmentEndpointCache2`, `ExactCurveArrangementOutputSegmentKindBucketCache2`, `ExactCurveArrangementOutputSegmentSourceBucketCache2`, `ExactCurveArrangementOutputSegmentSourceRangeCache2`, `ExactCurveArrangementOutputSegmentStatusBucket2`, `ExactCurveArrangementOutputSegmentStatusBucketCache2`, `ExactCurveArrangementOutputSegmentStatusRef2`, `ExactCurveArrangementRingAssemblyCache2`, `ExactCurveArrangementSourceAabbBucketCache2`, `ExactCurveArrangementSourceAabbStatus2`, `ExactCurveArrangementSourceEndpointBucketCache2`, `ExactCurveArrangementSourceSegmentCache2`, `ExactCurveArrangementSourceSegmentFact2`, `ExactCurveArrangementSourceSegmentKindBucketCache2`, `ExactCurveArrangementSplitBlockerCache2`, `ExactCurveArrangementSplitCache2`, `ExactCurveArrangementSplitCandidateAabbStatus2`, `ExactCurveArrangementSplitIntersectionBucketCache2`, `ExactCurveArrangementSplitIntersectionParameterCache2`, `ExactCurveArrangementSplitRelationBucket2`, `ExactCurveArrangementSplitRelationBucketCache2`, `ExactCurveArrangementSplitRelationClass2`, `ExactCurveArrangementSplitScheduleBucket2`, `ExactCurveArrangementSplitScheduleBucketCache2`, `ExactCurveArrangementSplitScheduleCache2`, `ExactCurveArrangementSplitScheduleRef2`, `LineSegmentEndpointGraphEvidenceParts`, `LineSegmentRingAssemblyEvidenceParts`, `LineSegmentSplitEvidenceParts`, `RegionArrangementSummary2`, `RegionBoundaryContourBuildEvidence2`, `RegionBoundaryContourRoleEvidence2`, `RegionLineSegmentArrangedSourceEvidence2`, `RegionLineSegmentRegionBuildEvidence2`, `RegionLineSegmentRingSourceEvidence2`, `RegionLineSegmentSplitIntersectionEvidence2`
- `retained_status.rs`: `RetainedTopologyStatus`
- `segment.rs`: `CircularArcFragmentWitness2`, `CircularArcParameterWitness2`, `CircularArcRetainedFacts2`, `RetainedLineRelation2`
- `self_intersect.rs`: `SelfContactXSchedule`
- `translation_obstacle.rs`: `TranslationObstacleEvidence2`

## Conclusion

Every declaration in the retained-fact/evidence/report scan has an explicit
owner and disposition. The only new object-local fact cache is the
benchmark-proven lazy arc packet in an existing shared allocation. Duplicate
prepared facts and kind-only aggregate scans were removed. Hyperlimit's
reusable evidence objects remain the preferred collapse point for multi-object
predicate preparation, while reports stay query results. No compact inline
carrier or hard size budget was enlarged.
