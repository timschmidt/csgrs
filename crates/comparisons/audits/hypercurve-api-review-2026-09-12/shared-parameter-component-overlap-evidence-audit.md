# Shared nonlinear parameter-component correspondence

The full implementation goal remains active. This migration consolidates existing overlap transport in preparation for common analytic-curve intersection dispatch.

`BezierParameterComponentOverlap2` already retains the bivariate component support, selected fiber ranks, branch witness, original overlap domain and endpoint inclusion. Its mapper handles ordinary algebraic, selected-fiber and recursive-projective parameters. Region Boolean code separately wrapped this authority in `RegionPairOverlapSource::ParameterComponent` and implemented its own operand swapping and range-clipping branch.

The authority now enters the shared `CurveOverlapCorrespondence2::ParameterComponent`. The region-only variant and clipping branch are removed, and both production publication and the existing selected-component fixture construct the shared form directly. Circle and parameter-component transport normalize operand order once in the shared dispatcher. Source charts, field evidence, component mapping equations, branch selection, endpoint ownership and policy handling continue through the existing exact kernel. No compatibility interface or parameter promotion was introduced.

Validation reuses the meaningful existing obligations: nonlinear selected clipping in both operand orders; projective overlap clipping without global projection; degree-135 selected-scalar transport; recursive-projective component mapping; and circle correspondences under operand and traversal reversal. The frozen 49-target qualification and public matrices also exercise ordinary chord/rational transport through the same dispatcher. No test merely mirrors the new enum wrapping.

This consolidation does not by itself implement the missing Parallel/Bezier, Parallel/Line or Parallel/Parallel cases in common curve intersections. The separate public region overlap evidence gap also remains: `CurveRegionIntersectionOverlap2::source()` exposes an optional native curve-overlap wrapper and does not publish every analytic correspondence. Those remaining obligations are recorded in `analytic-common-dispatch-followup.json`.

The final result, source and normal-library hashes, dependent checks, public matrices and commit are recorded in `shared-parameter-component-overlap-qualification.json`. No new performance or memory claim is made for this migration.

Final qualification committed as Hypercurve `8b3573224c5b13fcf2a953e0f8cdae5c70d322a0`: 49 targets, 2,210 passes (1,978 Hypercurve + 232 HyperBREP), five unchanged known failures, nine ignored tests, eight unchanged expensive exclusions, no new failures and no timeouts. Both all-target checks and separate fuzz/UI checks pass. Formatting, whitespace and all 396 frozen source hashes pass. All 30 repositories are clean. Normal-library SHA: `d30bc1c78a473adc5480b8cb0ff1c0cc20d813d9061281132a4c8019872fa30a`.

The circle, carrier-replay, general-trim and constant-image public matrices all retain their exact counts and certification. The carrier matrix completes in about 1.4155 seconds in this qualification; the controlled performance comparison belongs to the preceding `ef21147` change. No separate performance or memory improvement is claimed for this consolidation.
