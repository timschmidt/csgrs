# Face and loft construction repair

Hypertri will add a simple-ring entry point sharing its existing earcut core
and Hyperlimit predicates. Its domain is a sequence of at least three distinct
vertices defining one nonzero-area simple polygon, without holes. A repeated
closing point remains supported, including qualified equality, as required by
the existing `geometry_context` test. Forward collinear boundary vertices are
supported and retained. Other duplicate coordinates, backtracking, self-touching
and crossing boundaries are typed invalid input;
unresolved predicates remain typed undecided. The result retains the input
winding and triangles oriented with that winding. Existing general earcut APIs
and their normalization/hole contracts remain available unchanged.

CSGRS polyhedron faces and loft sections consume this shared validation and
triangulation. Projection remains an invertible coordinate projection, with
planarity proved under the caller's policy. Whole-ring winding corrects the
support normal used by loft orientation. Polyhedron preserves authored face
orientation, unused positions and intentionally open surfaces; it does not
claim that an arbitrary collection of faces is a solid. Empty points/faces
remain canonical empty. A points-only carrier remains a structural buffer,
not a valid empty solid.

Loft connects corresponding vertices using the existing side diagonals and
cap order. Consecutive sections proved pointwise equal are redundant and are
coalesced; their equality decisions contribute certainty. At least two distinct
sections with nonzero normal travel remain required. Loft sections remain open
sequences (their first point is not repeated); this preserves the existing
correspondence contract. Every retained section is simple and planar. A
completed loft must pass native closed-PWN validation;
stationary-vertex degeneracy and other invalid mesh boundaries fail with their
native cause before publication. This is a PWN boundary claim, not a proof of
embeddedness or fidelity to an unspecified continuous swept surface.

Both strict and context APIs preserve native triangulation/mesh errors and
aggregate consumed certainty. Approximate selection alone never weakens a
certified answer. Internal construction vectors use the existing checked
capacity/reservation helpers. Native triangulation, scalar expression growth,
total work, heap bytes and cancellation still have no complete internal bound;
the O(n²) simple-ring edge-pair validation is externally contained in tests.

Migration: previously reversed concave faces acquire authored winding; invalid
rings stop returning incomplete successful meshes; redundant loft sections stop
creating degenerate faces. Correct original tetrahedron and square-loft ordered
artifacts must remain identical. No performance-only claim is made. Original
full-size fixtures, both policies, symbolic planarity, all cyclic/reversed L
orders, collinear boundaries, exact area/volume/membership, downstream Booleans,
typed failures and affected binding consumers are acceptance evidence.

Minkowski semantics, inherited provenance, release installation and the broader
properties obligations remain open. This is one coherent implementation step.
