# Single-line construction review

The dispatch is in `CurveRegion2::stroke_path_raw`, after the existing path
connectivity, strictly positive half-width, and corner-style checks. It requires
exactly one represented native line, retained evidence of distinct endpoints,
and a butt or round cap. Other cases keep the existing route. The helper uses
the existing offset, line, arc, contour and region-regularization code; it does
not implement another arrangement or a new scalar simplifier.

For endpoints A and B, let d = B - A, L = |d| > 0, and
n = (-d_y, d_x) / L. With half-width r > 0, the existing `offset_left`
primitive constructs A + rn to B + rn and A - rn to B - rn. Its exact
normal has length one. The candidate follows the right side A - rn to B - rn,
then the end cap, then the reversed left side B + rn to A + rn, then the
start cap. Butt caps close the rectangle. Each round cap has antipodal
endpoints about its authored endpoint, squared radius r², and a
counterclockwise semicircular sweep. The end cap occupies the outward half
disk at B; the start cap occupies the outward half disk at A. These four
edges bound the ordinary rectangle or capsule without an interior seam.

These hypotheses justify the private certified radius/sweep constructor.
The resulting contour still enters `try_from_native_contours_raw` and
`finish_construction`; it does not bypass the owning region implementation.
An unresolved endpoint-distinctness fact cannot enter the specialization.
Validation occurs before the branch, so it cannot conceal an invalid width
or corner option. Existing outer evaluation retains typed shared-budget
denial even when downstream facts are already cached.

For the non-preview STRICT and APPROXIMATE_512 contexts, a positive width
decision is backed by nonzero evidence. Hyperlimit's terminal sign path uses a
certified rational interval: Positive requires its lower endpoint to exceed
zero; an interval containing zero yields the qualified Zero interpretation.
That Zero interpretation fails the positive-width check before this helper.
The existing `path_stroke_requires_a_policy_positive_half_width` regression
exercises zero, negative and terminally unresolved zero widths under both
policies. It passes in the complete default run. The public
`CurvePreviewOptions::evaluate` contract separately identifies its output as
lossy preview evidence that cannot be retained as certified topology; the
exact-certification claim here does not override that format/display boundary.

For the exact set, independently expected area is 2rL for butt caps and
2rL + πr² for round caps. Round bounds extend endpoint extrema by r.
Butt bounds extend endpoint extrema in x by |d_y|r/L and in y by |d_x|r/L.
The diagnostic uses exact integer squared-distance and projection comparisons
on half-unit grids, independently of the kernel's winding implementation.
Area, bounds, membership, contour reconstruction, reversed source orientation,
symbolic translation/radius, aggregation, tangent contacts and resource denial
are separate acceptance observations. Finite observations do not prove a
universal implementation theorem.

The retained offset lines carry the existing offset provenance. The caps retain
their exact source centers and radius/sweep facts. This review does not claim
complete public source-element provenance across every later operation or
serialization, or that operation-local certification repairs an earlier
approximate construction. Those broader obligations remain open.

Native boundary subdivision/order and equivalent scalar expression forms can
change intentionally. Exact snapshots and their differences are retained;
semantic equality is checked separately. The public documentation identifies
this representation migration. The KiCad noncoincident-track adapter and
finite `line_polygon` path are unchanged in this kernel patch.
