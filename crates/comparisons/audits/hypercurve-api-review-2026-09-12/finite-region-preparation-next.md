# Next finite-domain preparation audit

This is a static follow-up to finite-region-bounds, not a passing qualification or a reproduced failure. Do not edit production, test or probe sources until the active frozen qualification finishes.

`try_new_unary` and `build_cross_operand_carrier_pairs` both prepare Bezier operands with `Curve2::from(source.clone())`. The carriers retain exact start/end and selected endpoint images, but this preparation drops them. The common CurveIntersectionContext already dispatches retained curves through the finite support/range authority; native unit curves keep their batch cache path. Consolidate both preparation loops around the existing `CurveSupport2::restrict_certified` and `Curve2::from_retained_fragment`, keeping the original parameter chart and selected endpoints.

Proposed public baseline: reuse the cap P(t)=(-(t-s)^2,t-s) on [s,s+1] for s=0,1,-2. Trim the unit-chart polynomial vertical segment x=-1/4, y=u against each cap. The retained source interval should be [1/4,1/2] in every chart, with two boundary contacts. First compare direct curve intersection with the cap boundary to ensure the common finite kernel has the required authority. Check both policies and traversal directions; test parameter and point replay. A line represented as a Bezier may expose the dropped range while keeping the expected answer rational. If a line shortcut masks it, use a noncollinear quadratic with explicitly known contacts.

This migration alone does not repair native self-intersection caches or unit-only injectivity proofs. Those proofs must receive the active range; source-global cache keys cannot own an arbitrary partial-range proof. Other pending shortcuts include rational_control_hull_is_strictly_one_sided, parallel coordinate-injectivity pair pruning, and cold analytic point-incidence. Selected-fiber roots must remain selected rather than require global scalar projection.

Keep finite-domain admission, support evaluation and original root provenance together; avoid introducing a compatibility adapter or another family of public carriers.
