# HyperBREP Architecture

HyperBREP is an exact, immutable boundary-representation kernel.
`hyperreal::Real` is the sole authoritative scalar. Approximate numbers may
eventually propose work at explicit adapter boundaries, but they never decide
incidence, ordering, containment, topology, or regularization.

## Dependency ownership

The core dependency direction is:

```text
hyperreal
    ↓
hyperlattice
    ↓
hyperlimit      hypersolve
    ↓              ↓
hypercurve
    ↓
hyperbrep
```

Hypercurve owns exact planar curves, contours, regions, classification, and
regularized planar Booleans. Hyperlimit owns certified predicates and explicit
unresolved outcomes. Hyperlattice owns exact vector, point, matrix, and bounds
algebra. HyperBREP owns spatial carriers, topology, validation, solid
certificates, persistence, and modeling operations.

HyperTRI is an optional derived consumer behind the `tessellation` feature. It
is not a core dependency and cannot mutate or replace a source BREP.

## Canonical model

`Model` is an immutable `Arc`-shared snapshot over private typed arenas:

```text
Vertex → Edge → EdgeUse → Wire → Face → Shell → Solid
           ↓       ↓              ↓
         Curve3  Pcurve         Surface
```

An `Edge` owns one canonical spatial curve and exact parameter domain. Every
incident `EdgeUse` owns its direction, face-local Hypercurve pcurve, and exact
`ParameterCorrespondence`. Adjacency and ownership indexes are built once and
retained for O(1) lookup.

`ModelBuilder` is the publication boundary. Its noun-style methods stage
records, while `finish` proves global ownership, closure, image agreement,
winding/nesting, manifoldness, and the active solid-family certificate.
Ordinary `Model` values therefore need no readiness report.

A `Face` is either trimmed by one outer wire plus inner wires or denotes its
complete closed support surface. The latter is the canonical whole-sphere
topology: one boundaryless face and no fabricated longitude seam, pole edge,
or tolerance-merged vertex. A sphere split by a transverse intersection circle
uses a periodic latitude wire: its lifted pcurve closes by exactly one angular
period, so the cap is certified on the quotient parameter domain without
authoring a longitude seam.

`RawModel` is deliberately untrusted. Exact JSON decoding reconstructs every
carrier through its validating constructor and then replays topology through
`ModelBuilder`. Derived certificates and caches are never trusted serialized
state.

`Model::split_edge` is a topology-changing exact edit. It splits every
incident pcurve, updates forward and reversed wire traversal, and republishes
only after complete validation. Affine correspondences are rederived from the
new domains; angular correspondences invert through Hypercurve's certified
rational-conic parameter solver.

`Model::split_face` is the planar region counterpart. Two nonadjacent
outer-boundary vertices define one exact line chord. The chord is authored once
as a canonical edge with opposite face-local uses; the source face/wire IDs are
retained for one result, holes are reassigned by certified classification, and
the owning shell is updated by identity. Prism and cylinder certificates derive
cap regions from stitched external boundaries, so internal face subdivision
does not alter the represented solid.

The native cylinder and cone-frustum certificates also derive an exact
parameter-space grid from every current side descendant. Each rectilinear
pcurve loop must have the exact area of its bounding rectangle; those
rectangles must cover every quarter-angle/axial grid cell exactly once. This
admits latitude cuts and temporarily subdivided axial boundary edges without
accepting overlaps, gaps, or a sampled approximation.

The world-z prism optimization is separately gated by a certified translation
family. Two vertex-height layers and a planar cap are not sufficient evidence:
in particular, a cone frustum must fall through to retained curved stitching
instead of being interpreted as a constant-profile extrusion.

## Exact image agreement

Endpoint agreement is necessary but not sufficient. Each supported
curve/pcurve/surface tuple has a complete-interval structural proof:

- affine line images on planes;
- circular pcurves on planes through directed angular sweep;
- axial lines and constant-height circles on cylinders;
- generators and constant-height circles on cones;
- both circle directions on ring tori;
- line and circular profile images on extrusion surfaces.
- rational Bézier and NURBS control-net rows/columns on tensor surfaces,
  including exact edge subintervals, interior iso-curves, and projectively
  equivalent weights.
- rational Bézier and arbitrary-span NURBS spatial sections with degree-elevated
  rational graph pcurves on translation tensors linear in either axis, reconstructed
  control-for-control rather than accepted from endpoint incidence.

`builder::tensor_patch_shell` applies the same whole-curve certificate before
sharing topology. Matching endpoint vertices alone never sew patches: degrees,
knots, every control point, and weights up to one nonzero projective scale must
agree exactly after any required reversal. One canonical edge is then reused
by the adjacent face-local pcurves.

Unknown predicate outcomes remain errors. Unsupported tuples are rejected
during construction rather than published with an optimistic flag.

## Solid certificates

Closed manifold topology alone does not prove a non-self-intersecting regular
solid. The current kernel publishes solids only when one of these exact
families is recertified:

- line-bounded affine prisms, including through holes and void shells;
- arbitrary closed straight-planar outer and void shells after exact pairwise
  self-intersection and strict nesting certification, with a convex
  half-space fast path for outer shells;
- native line/arc extrusion prisms, including holes and disconnected models;
- native line/arc profile revolutions, including exact inward profile
  cavities and identity-shared periodic latitude/meridian edges;
- multi-section polygon lofts whose spans independently carry either a
  positive homothetic certificate or an exact convex-correspondence
  certificate and native bilinear tensor sides;
- fixed-frame rational Bézier path sweeps whose connector curves are exact
  translates, whose tensor sides reproduce the complete translated control
  net, and whose normalized profile-plane progress is certified affine and
  strictly positive by exact Bernstein coefficient identities;
- analytic cylinders;
- complete closed-surface spheres, spherical voids, exact plane-capped axial
  sphere segments, and exact two-cap sphere/sphere Boolean shells;
- truncated cones that exclude the singular apex;
- full periodic ring tori and exact axial torus bands closed by zero, one, or
  two planar annular cap groups.

Certificates are compact retained facts used for exact volume and point
classification. They are rebuilt from geometry and topology after persistence.
Supported transforms update the certificate directly and preserve its proof.

Torus certification derives longitude/latitude rectangles from the current
trimmed topology. Longitude cells must cover one complete period. Latitude
cells are split at every exact trim and at the four canonical quarter angles,
then must cover exactly the cells whose `r*sin(v)` image lies in the retained
axial interval. Each planar cap group must expose two complete concentric
circle loops with radii `R ± sqrt(r²-h²)` and the correct outward axial normal.
This proves full tori, two-cap interior bands, and one-cap bands that close at
a natural torus extremum. Volume uses the exact `Real` antiderivative of
`4*pi*R*sqrt(r²-h²)`; classification combines the same axial interval with the
ring-torus implicit predicate.

A whole sphere remains one boundaryless face until an actual retained latitude
requires topology. The first exact axial plane cut authors two identity-shared
circle halves and two complementary periodic spherical-cap faces; no
always-present pole or longitude seam is introduced. A second latitude splits
only its containing cap and represents the intervening material as one
periodic spherical-band face with two latitude loops. Full-sphere
recertification proves the lower-cap/band/upper-cap latitude chain. Selected
one-cap and two-cap results validate every exact circle, full angular coverage,
and outward planar-cap normal. Volume integrates the exact cross-section
`pi*(r²-h²)` over the retained axial interval, and classification combines
that interval with the radial sphere predicate.

Line/arc revolution certificates project every authored meridian carrier into
one exact positive-radius radial/axial contour. Circular meridians are checked
as complete rotated parameterizations, not endpoint samples. Hypercurve owns
the contour’s exact Green-theorem x-first-moment
`1/2 * integral(x^2 dy)`; HyperBREP applies the exact Pappus factor `2*pi`,
subtracts cavity moments, and reuses the retained contour for radial point
classification. Circular-profile face area integrates radius times native arc
length analytically.

For a non-homothetic loft, corresponding edge vectors vary linearly in the
section parameter. Consecutive-edge turns are therefore quadratic. HyperBREP
certifies their degree-two Bernstein endpoint coefficients strictly positive
and their mixed coefficient nonnegative, proving every intermediate polygon
strictly convex without sampling. Each ruled side is a 2×2 unit-weight rational
Bézier patch whose four exact line boundaries are checked against degree-one
tensor iso-curves. Volume integrates the exact Bernstein quadratic section
area; point classification reconstructs the exact convex section at the query
height. Concave homothetic lofts retain the scale/profile fast certificate.
For three or more authored sections, the validator reconstructs every layer
from exact cap-normal height, proves each ring and connector bijection from
topology, and recertifies each side carrier. Intermediate rings are shared
edges between neighboring spans, making the promised continuity exactly C⁰.
Volume sums the exact quadratic area integral over normalized span widths;
classification selects the exact containing span before evaluating its
section certificate.

For a fixed-frame curved sweep, the profile section is
`path(t) + u*x + v*y`. The retained certificate reconstructs one canonical
connector path and every translated connector and tensor side from topology.
It proves that projection of the rational Bézier path onto `u × v` is exactly
affine in `t`, not merely monotone at samples. This makes section inversion
exact, excludes folds through the profile planes, and leaves lateral path
curvature unrestricted. Moving frames and corner-continuity policies are not
inferred.

## Queries and operations

Exact face measurement integrates the actual carrier Jacobian. Planes,
cylinders, extrusion surfaces, spheres and spherical caps, cones, and tori use
family-specific exact routes; no curved boundary is replaced by chords.

Regularized z-prism Booleans delegate planar arrangement and role assignment to
Hypercurve, then reconstruct line/arc material contours and holes as newly
validated BREP solids. Empty, incompatible, unsupported, and unresolved
outcomes remain distinct.

The retained intersection-graph route is the general planar BREP pipeline:
exact transverse and coincident support arrangements partition both operands,
interior witnesses select face cells, and selected cells are copied into a new
arena with identity-shared cross-operand edges. Edge parameter domains are
remapped exactly, reversed difference faces reverse their pcurves, coplanar
cap fragments are certified in one geometric plane frame, and connected
components are republished only after the ordinary shell and solid validators
accept them. This is the correctness fallback behind the standard solid
Boolean API; the world-z profile kernel is only an optimization and its
failure cannot block an otherwise certifiable oriented planar result.

Finite spline/ruled carrier intersections retain one spatial `Curve3` and an
exact pcurve on each surface operand. Hypercurve region trimming preserves the
promoted Bézier span and top-level public parameter range for every fragment;
HyperBREP intersects those ranges and uses exact `Curve3::subcurve`
materialization. Plane/extrusion generator lines use the same principle with
an affine surface-parameter line. A plane cutting a rational Bézier/NURBS
translation tensor linear in either parameter axis uses the same projection
theorem but retains the non-isoparametric tensor image as an exact rational
graph `(u(v), v)` or `(u, v(u))`.
Positive-weight control hulls certify complete bounded sections and
disjointness. Mixed hulls are clipped through Hypercurve in the bounded tensor
parameter rectangle; every `Real`-represented boundary root produces an exact
spatial/pcurve subcurve, while non-representable algebraic boundaries remain
explicit. This avoids sampled projection and endpoint-only
inverse location when transferring an intersection into trimmed face topology.
`boolean::intersect_faces` owns this face-pair operation:
validated open-shell faces and solid-owned faces use the same certified bounds,
carrier relation, two-pcurve clipping, and explicit unsupported evidence.
`boolean::intersection_graph` is the solid-level enumeration and accounting
layer over that primitive, not a separate geometry implementation. Its native
finite trim fragments retain the spatial curve and both pcurves as
`SurfaceIntersectionCurve`; fragment restriction composes exact affine
parameter maps so all three evaluations share the fragment curve's public
domain even when different curve families use different subcurve-domain
conventions. `SurfaceIntersectionPcurve::materialize` exposes one exact
Hypercurve `Curve2` plus the affine map from its public parameter back to the
spatial fragment parameter. Topology transfer therefore needs neither inverse
fitting nor an assumed identity parameterization.
Retained pcurves may run in either source direction; materialization preserves
that affine direction exactly for lines, rational Béziers, and NURBS curves.
For an authored-frame-aligned coaxial sphere/cylinder pair, each intersection
circle instead has two constant-`v` analytic pcurves. Face-pair clipping
restricts the cylinder image to its four native parameter rectangles. Before
the boundaryless sphere is edited, exact common circle support and adjacent
angular domains coalesce the patch fragments back into one periodic trace;
the coalescer reconstructs both constant parameters rather than substituting a
plane projection. Operand reversal and a common rigid transform preserve the
same two authoritative images.
Selected intersection faces form two spherical caps and one subdivided
cylindrical band. Their mixed-shell certificate reuses the cylinder's exact
parameter-cell coverage, proves both cap latitudes have the same sphere and
opposite heights, and matches each circle radius and axial parameter to that
band. Only then does the query layer evaluate the intersection predicate
`sphere ∩ radial cylinder`; its volume is
`4*pi*(sphere_radius³-axial_half_height³)/3`.
Closed face-local curves are split into two canonical edge records so the new
interior face's outer wire and the surrounding face's inner wire share
opposite edge uses by identity. Multiple traces are atomized at represented
pcurve contacts and routed by exact `CurvePath2` classification against the
current descendant regions.
After partitioning, `select_first_faces` and `select_second_faces` generate
exact parameter candidates from the retained pcurve boundary and its certified
`Aabb2`. Candidate coordinates are merely a search schedule: a point becomes a
witness only after the complete `CurvePath2` outer/hole classifier proves it
interior. The corresponding exact surface image is then classified by the
opposite solid. Boundaryless carriers use canonical parameters derived from
their declared unbounded, closed, periodic, or lower-bounded domains.

Selected-result transfer is likewise all-face. Source-local edge identity is
preserved independent of curve family. Cross-operand candidates are first
atomized at every represented exact endpoint and then share one result edge
only when their endpoints and complete restricted `Curve3` data agree, with an
exact reversal proof when traversal differs. Endpoint equality alone is used
only after a carrier has been certified and canonicalized as a line.

Regularization happens before publication, not after approximate sewing. A
rational Bézier becomes a `Line` only when all weights agree and every control
is the exact degree-elevated affine control. A rational-Bézier or NURBS tensor
surface becomes a `Plane` only when a nondegenerate frame can be chosen from
its controls and every remaining control has exactly zero signed plane value.
Every affected face pcurve is then rebuilt by exact projection of its spatial
edge into the new plane frame, and its affine edge correspondence is derived
from the two exact parameter domains. The normal builder validation and
untrusted persistence replay see only the regularized geometry; no Boolean
provenance is trusted.

For tensor carriers, `Surface::iso_curve` derives the complete exact
homogeneous row or column at any represented constant parameter. The topology
validator accepts an edge only when its full carrier or represented subcurve is
projectively identical to that derived iso-curve. This lets
`Model::split_face_by_surface_curve` attach an intersection to opposite
boundary edges and publish two identity-sharing rational Bézier or NURBS faces
without downgrading validation to endpoint sampling. Complete
non-isoparametric rational graph sections on translation tensors linear in either axis use
an analogous retained proof: the validator reconstructs each translation
coefficient from rational-Bézier or NURBS spatial controls,
retains the NURBS degree, knot vector, and native domain, exactly
degree-elevates each exact knot span and joins multi-span graphs as one
native-domain NURBS pcurve, reconstructs partial source-profile knot ranges,
certifies same-boundary two-edge pockets and their complementary loops, rejects an
endpoint-preserving control forgery, and certifies the two curved descendant
loops before publication. General rational Bézier pcurves are serialized
exactly and replayed through the same validation boundary.
`Model::split_face_by_surface_curves` accepts retained intersection objects
directly, uses an explicit operand selector for their first or second pcurve,
canonicalizes each unordered spatial endpoint pair, and locates every
successive fragment on exactly one current descendant. Before editing, every
pair of materialized operand pcurves is intersected through Hypercurve's
certified topology kernel. Represented transverse contacts are mapped back to
the authoritative spatial parameter and atomize later traces; attaching those
atoms splits the already-shared earlier edge at one identity vertex. Positive
length overlaps and unresolved contacts are rejected. Applying disjoint or
transversely crossing fragments is therefore independent of caller order and
does not require a caller to guess descendant IDs. Wholly interior closed
curves use the same arrangement, including nested loops and later
boundary-attached traces. Curved-loop orientation first uses Hypercurve's exact
native-Bezier Green integral when available; the monotone tensor-graph proof
remains the exact fallback for graph families without a materializable area
integral.

Curve-sweep solid certification is topology-derived rather than tied to the
builder's primitive face and edge counts. Coincident planar cap descendants
are grouped and their internal shared edges cancel. Descendants on each
rational-Bézier side support are grouped by exact `SurfaceId`; their external
pcurve intervals must tile all four sides of one exact active tensor rectangle
with no gap or overlap. A connector may be one edge or an identity-connected
chain: its edge domains must tile the complete retained spatial curve while
its tensor pcurves tile the common active path interval. The side support is
restricted to that exact interval by homogeneous subdivision before its
translated controls are compared with the reconstructed connector subpath.
This admits exact transverse clipping at represented path parameters while
preserving the same curve-sweep certificate. The proof runs during untrusted
persistence replay, so cap, connector, and side subdivision does not rely on
edit provenance.

Closed planar line shells outside the translation-prism family have two exact
publication routes. Convex shells use a fast certificate: every face loop is
convex and every shell vertex lies on or behind every outward-oriented face
plane. General shells use an exact pairwise certificate. Coincident face
regions are intersected in one common plane frame; transverse plane
intersections are reduced to exact material intervals and boundary contacts on
their common line; and coincident boundary segments are intersected exactly.
An overlap or contact is accepted only when the faces share the topological
edge or vertex that represents it. This admits concave non-prismatic shells
without sampling and rejects geometric crossings and non-manifold line or
point contacts as `SelfIntersectingSolidShell`. Both certificates are
recomputed during persistence import.

Selected planar Boolean components are oriented by exact signed shell volume.
Outward components become solids; inward components are assigned to exactly
one containing outward shell by exact ray classification. Solid construction
then rechecks every outer/void and void/void face pair, strict containment, and
non-nesting. A boundary contact therefore produces `VoidShellOutside` or
`IntersectingVoidShells` rather than a non-manifold cavity.

The optional tessellation adapter has separate exact and lossy types. Exact
planar triangulation copies validated line-boundary parameters into HyperTRI
and maps the result back to exact model-space points.
`approximate_tensor_face_chordally` accepts only line-trimmed finite rational
Bézier/NURBS faces and an explicit integer parameter-subdivision policy.
HyperTRI's planar ear clipping may remove collinear UV samples, so the adapter
reinserts every requested boundary sample into the triangle topology before
refinement: those UV-collinear samples can have non-collinear images on a
curved surface. Refinement uses shared exact UV midpoints. The derived artifact
retains an exact parameter beside every exact surface evaluation and exposes
`ExactAtVerticesOnly`; model-space edges and interiors are chords with no
invented Hausdorff or normal-error certificate. It owns no mutable source
state and cannot replace the BREP.

## Performance contract

Correctness does not vary by execution route. Retained adjacency, structural
carrier facts, immutable caches, broad-phase bounds, and compact solid
certificates select faster exact work. Numerical refinement or approximate
filters may only avoid work or propose candidates whose results are replayed
exactly.

See [`../PERFORMANCE.md`](../PERFORMANCE.md) for benchmark cohorts and current
baselines, and [`SUPPORT_MATRIX.md`](SUPPORT_MATRIX.md) for the exact operation
matrix.
