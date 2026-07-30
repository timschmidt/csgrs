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
state. The decoder permits the full depth of authoritative `Real` expression
trees rather than inheriting Serde JSON's generic recursion ceiling, and wraps
deserialization in a dynamically growing stack adapter before reconstruction;
geometry and topology validation remain the trust boundary.

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
- exact restricted line, circle, rational Bézier, and NURBS meridians plus
  constant-profile latitude circles on revolution surfaces, including affine
  remapping between normalized edge domains and the authoritative profile
  parameter.
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

All kernel quadrant construction uses one checked `certified_atan2` path. It
decides both coordinate signs through the exact comparison policy, returns
axis angles structurally, and applies the exact single-argument arctangent
only after the quadrant is certified. Symbolic cancellation therefore becomes
an exact axis result or a typed predicate/elementary-function error; it cannot
panic through `Real::atan2`'s convenience refinement floor.

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
  cavities, exact profile-parameter subdivision grids, identity-shared
  periodic latitude/meridian edges, and axis-normal planar annular caps;
- exact closed `CurvePath2` profile revolutions with retained Bézier,
  polynomial B-spline, and finite NURBS meridians, injective native-span and
  pairwise simple-loop proofs, curved inward cavities, and topology-rebuilt
  boundary certificates;
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

Spline-profile revolution certificates instead reconstruct the represented
meridian carriers as one exact `CurvePath2`; unit-weight NURBS spans remain
polynomial B-splines and rational Bézier controls and weights are retained
without flattening. A single periodic polynomial B-spline or NURBS profile is
partitioned at its exact native knot intervals into finite clamped
piecewise-Bézier spline carriers before topology publication. This retains the
source parameter intervals and rational images while avoiding a degenerate
one-vertex closed edge. Hypercurve classifies the radial/axial boundary
directly and supplies exact Green-theorem area moments for polynomial and
polynomial-equivalent Bézier fragments. Finite rational quadratics use exact
homogeneous Green integrals with denominator `W^4`, reduced symbolically to
rational endpoint terms and certified `atan`/`ln` inverse-quadratic branches.
Arbitrary-degree rational carriers exactly inverse-elevated in homogeneous
Bernstein space reuse that conic kernel, including after untrusted replay where
no provenance cache exists. Genuinely higher-degree images whose homogeneous
weight polynomial certifiably has degree at most two use exact polynomial
division before the same square-free, repeated-root, or linear-denominator
Hermite branches.
A nonuniform rational span certified to have an exact finite line image uses
that line's geometric moment, independent of its projective speed. HyperBREP
applies the same exact Pappus factor and cavity subtraction. Rational images
with cubic-or-higher weight polynomials and the general
radius-times-spline-speed face integral are explicit unsupported measurements
until symbolic integrators exist. Native-only contours remain the sole input
to the optimized coaxial-profile Boolean path; curved profiles use the retained
face graph instead of claiming compatibility geometry.

Subdivision does not change that certificate. Revolution descendants must tile
four exact quarter-angle columns and every represented profile-parameter cell
without overlap or omission; the retained v-range, not the untrimmed carrier
domain, determines the reconstructed meridian segment. An axis-normal planar
cap group is accepted only when its effective oriented normal is axial and its
boundary is exactly two complete concentric circle loops. Their radii and
axial coordinate reconstruct one radial profile segment, allowing selected
revolution bands and planar annuli to form one ordinary certified solid.

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
materialization. An axis-normal plane/revolution query first intersects the
finite authored profile with the plane. Every isolated positive-radius
profile point becomes its exact rotational orbit: one spatial circle, one
native planar projection, and one constant-profile revolution pcurve sharing
the circle-angle domain. Axis contacts remain isolated points. A contained
profile interval, a mixed point-and-circle result, or multiple profile
parameters covering the same spatial circle remains explicit unsupported
evidence because the retained relation does not pretend those are one
single-valued pcurve. Finite NURBS meridians are decomposed into exact rational
Bézier knot spans for the plane relation; isolated parameters are mapped back
to the authoritative knot domain and contacts shared by adjacent spans are
deduplicated before circle publication. Plane/extrusion generator lines use
the same principle with an affine surface-parameter line. An axis-containing
plane/cone relation is represented explicitly as two lower-bounded
`SurfaceIntersectionRay` values,
each carrying the authoritative spatial origin, direction, and minimum plus
an affine parameter ray for both operands. Face clipping works directly in
those retained parameter rays, intersects the exact trim intervals, clamps
them at the apex, and admits only finite `SurfaceIntersectionCurve` fragments
to topology; no cone inverse fit is needed. A plane cutting a rational Bézier/NURBS
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
An authored-frame-aligned coaxial sphere/cone pair follows the same retained
circle contract. The carrier solver works in the cone's native nonnegative
slant parameter, certifies the exact quadratic discriminant, and emits sphere
latitude plus cone slant pcurves for each positive root. Frustum face clipping
therefore partitions the full circle across its native quarter-angle cells
without a sampled inverse. A lone zero root is the exact apex point; a zero
root accompanied by a positive circle is rejected as an unsupported
mixed-dimensional relation rather than losing one component.
Selected intersection faces form two spherical caps and one subdivided
cylindrical band. Their mixed-shell certificate reuses the cylinder's exact
parameter-cell coverage, proves both cap latitudes have the same sphere and
opposite heights, and matches each circle radius and axial parameter to that
band. Only then does the query layer evaluate the intersection predicate
`sphere ∩ radial cylinder`; its volume is
`4*pi*(sphere_radius³-axial_half_height³)/3`.
The complementary difference selects the native two-loop spherical band and
the same four cylinder cells with inward orientation. The shared mixed-shell
certificate records which side of the radial cylinder is material rather than
inferring it from floating samples. For `sphere \ radial cylinder`, the exact
volume is `4*pi*axial_half_height³/3`; point classification combines the same
sphere predicate with the complementary radial inequality.
For `radial cylinder \ sphere`, retained selection instead creates one
component beyond each sphere/cylinder latitude. Every component contains a
forward cylinder parameter grid, one outward planar cylinder cap, and one
reversed spherical cap. Its cylinder certificate carries an explicit
spherical exclusion and proves the common axis and center line, exact
intersection height and radius, complete cylinder cells, and a planar cap
strictly beyond the corresponding sphere pole. Volume is the exact cylinder
interval minus the enclosed spherical cap; classification applies the
cylinder interval/radius predicate and then excludes the sphere. These mixed
certificates are deliberately absent from the plain-cylinder profile fast
paths.
The regularized union uses the complementary topology: one forward spherical
band joins complete lower and upper cylinder parameter grids and their two
outward caps. Certification proves the two cylinder ranges terminate at the
band's exact latitudes, both outer caps lie beyond the sphere poles, and all
carriers share one axis and center line. Sphere query state is a closed
`CertifiedSphereRegion` enum—whole, axial interval, radial side, or finite
cylinder union—so union semantics cannot coexist accidentally with a clip.
Volume is the cylinder volume plus the sphere material outside its radial
core. Point classification combines the sphere and finite-cylinder predicates
as a true union, making either carrier's boundary internal whenever the other
predicate is strictly inside. Mixed union results advertise neither a plain
sphere nor a plain cylinder optimization profile.
Strict no-contact containment uses the whole native operand shells rather than
authoring intersection topology. A sphere contains a finite cylinder only
when the exact maximum corner-distance certificate—maximum axial endpoint
distance and radial axis-offset plus cylinder radius—is strictly below the
sphere radius. A finite cylinder contains a sphere only when both axial
clearances and the radial axis-offset clearance are strict. The nontrivial
difference reverses the contained shell and assigns it by this certificate
before any planar representative-point fallback. Sphere void state is a
closed sphere-or-finite-cylinder enum; cylinder spherical subtraction is a
closed whole-void-or-intersecting-component enum. This makes exact volume,
classification, rigid transforms, and replay share the same topology proof,
while mixed-void results remain excluded from primitive optimization profiles.
The same authoritative clearance predicates live on the certified primitive
profiles and run before intersection-graph construction. Consequently,
strictly contained off-axis pairs select the whole outer/inner operands or
copy the two complete shells into an exact outer-plus-reversed-void result
without asking the narrower coaxial carrier-intersection implementation.
Equality is deliberately excluded: tangent or otherwise non-strict off-axis
pairs fall through to explicit unsupported intersection evidence.
Finite coaxial intervals need not span both intersection latitudes. A selected
one-latitude shell certifies one forward spherical cap, one complete
cylindrical parameter grid, and one planar cap. Cylinder orientation plus
whether its represented interval runs toward or away from the sphere center
determines intersection, union, or sphere-minus-cylinder semantics; the
opposite difference reuses the existing reversed-spherical-cap cylinder-end
certificate. `CertifiedSphereRegion::FiniteCylinder` stores that closed
operation enum. Its volume uses one exact piecewise overlap integral split at
`±sqrt(R²-r²)`, while classification combines the finite-cylinder and sphere
locations by the corresponding Boolean truth table. Invisible material beyond
the selected spherical pole is represented by a certified effective interval,
preventing discarded source faces from becoming false query boundaries.
An axis-containing plane cuts a ring torus into two exact native meridian
circles. Each result retains a planar circular pcurve and a constant-longitude
torus pcurve over the same `Real` parameter domain. The carrier predicate also
retains exact point tangency at `major_radius + minor_radius` and certifies
strictly exterior parallel-to-axis planes as disjoint before graph
construction. Oblique-through-axis graph fixtures prove
that the two circles clip into eight patch-local fragments, partition every
periodic torus cell, and author two closed planar loops while preserving both
source volumes. `CertifiedTorusRegion` then closes the query state over whole,
axial-band, and longitudinal-half regions. The longitudinal proof requires two
complete minor-radius meridian disks, antipodal major-radius centers, one
axis-containing cap plane with a consistent outward orientation, exactly
`pi` of selected longitude, complete latitude coverage, and no partially
covered parameter cell. Intersection and complementary torus difference
therefore publish exact half-tori with volume `pi²*R*r²`, boundary area
`2*pi²*R*r + 2*pi*r²`, halfspace-aware classification, transform replay, and
no false whole-torus optimization profile.
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

The same projection rule applies to line edges on planar faces inside mixed
curved shells even when no carrier regularization is required. This makes the
spatial edge and actual plane frame authoritative at result transfer time,
discarding exact-but-pathological expression history introduced by planar
arrangement routing while retaining byte-stable exact replay.

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
For curved components, boundary-chord tetrahedra are not a volume or
orientation certificate; stitching instead retains the exact outer/void
material-side role of every selected source face through reversal. Any
component containing transferred outer material boundary is an outer shell;
components made only from retained or reversed void-side boundary are voids.
Inward components are assigned to exactly one containing outward shell by
exact classification. Solid construction then rechecks every outer/void and
void/void face pair, strict containment, and non-nesting. A boundary contact
therefore produces `VoidShellOutside` or `IntersectingVoidShells` rather than
a non-manifold cavity.

The optional tessellation adapter has separate exact and lossy types. Exact
planar triangulation copies validated line-boundary parameters into HyperTRI
and maps the result back to exact model-space points.
`approximate_face_chordally` accepts any validated face with an explicit
finite boundary plus seam-free complete spherical faces, under an explicit
integer parameter-subdivision policy. All supported pcurve and surface
families are evaluated through their native exact `Real` parameterizations;
no sampled carrier replaces either one. A complete sphere's longitude seam
and pole indexing exist only in the derived artifact and never alter the BREP.
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
