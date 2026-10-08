# Remove the redundant NURBS decomposition wrapper (candidate, unrun)

NurbsBezierDecomposition2 owns exactly one RationalBSplineBezierExtraction2 and forwards five queries. All callers naming the wrapper are inside nurbs.rs and its public export. The isolated two-file candidate removes that struct and implementation; the existing cache directly owns and returns RationalBSplineBezierExtraction2. It removes wrapper construction/mapping and updates native-span promotion to consume the same extraction directly. No alias or compatibility interface is added.

The actual homogeneous controls, knot intervals, policy-isolated cache, span evaluators, native-subcurve promotion and evidence remain on the same existing extraction value. Callers additionally have direct access to its existing span_fact_evidence and native_subcurves queries. Existing NURBS/BSpline and downstream tests, caches, approximate-first policy guards, periodicity and infinite-control cases should qualify this migration; no implementation-mirroring test is needed. Low-level authoring carriers remain a separate later consolidation.

This candidate is not formatted, built or promoted. V657 is the active qualification; V659 independent nonlinear cusp probe must follow its commit. Next available artifact V661.
