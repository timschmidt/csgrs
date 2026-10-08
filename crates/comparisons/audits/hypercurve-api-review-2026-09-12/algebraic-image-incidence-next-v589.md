# Algebraic-image whole-domain incidence probe (UNRUN)

After V587 is fully successful, exact outer 99449 reaped, and its commit
sealed, run probe-algebraic-image-collapse-20260928-v590.py. Do not run any
other build/qualifier concurrently. This copies production and injects only
the V589 test sidecar; workspace production remains unchanged.

The independent query Q(alpha)=(2*alpha^2,0) with alpha^2=1/2 is proved equal
to (1,0). A quarter of the unit circle about (1,0), offset left by +1, is
exactly that point for all parameters. Offset -1 has no such incidence.
The public algebraic-image point path currently enumerates two squared
coordinate equations and returns Boundary when both projections vanish.
The new retained-field GCD query already represents the common zero
polynomial as the entire domain. Two isolated tests exercise the public
path and this existing generic helper using the same independent oracle.

No result has been observed yet. If the public path fails and the retained
field path succeeds, prioritize consolidating the point query before the
V588 constructor-only cleanup. Keep native fast paths only where measured;
preserve winding endpoint ownership and parameter ordering. The algebraic
point predicate already holds its original image by reference; inspect its
existing accessors before inventing a new reconstruction or authority.

Read-only consolidation findings while V587 runs:
- RationalBezierAlgebraicPointPredicate2 already exposes point_image(),
  retained_root(), retained_parameter(), denominator_sign() and
  coordinate_polynomials(). No new accessor or reconstructed root is needed.
- predicate_evaluator resolves deferred images, so retain that demand-driven
  preparation before using point_image() in a common query.
- visit_point_parameters_from_systems and visit_point_parameters are used only
  by the general Algebraic case and BezierParallelAlgebraicRay2::contains_point.
  Replacing those consumers directly may remove about 140 lines.
- system/projected_parameters/expression_sign_at_candidate are still consumed
  by winding and must remain unless winding is separately proven equivalent.
- Generic membership needs no traversal order, while winding does. Do not let
  a membership consolidation remove the winding ray's endpoint ownership.

Admission detail: predicate_evaluator also certifies the selected point
denominator as nonzero, and exposes the resolved image through point_image().
The recursive point importer alone does not replace that check for arbitrary
algebraic images. Preserve it when consolidating; current three generated
point forms already carry a nonzero projective-denominator construction proof.

V590 outcome (now observed): exact outer 14311 reaped 1. Public algebraic
image test failed Boundary (0.016 s) after exact point coincidence passed;
existing common retained-field query passed (0.008 s). V591 applies the
consolidation directly, removes the duplicated native point visitors, and
retains predicate admission and winding-specific machinery. V592 normal
production focus is active (outer 26546, 33 cases). V593 broad driver is
prepared but unrun; it requires an explicit successful V592 reaping receipt
and identical source/library hashes before reusing focused test receipts.
