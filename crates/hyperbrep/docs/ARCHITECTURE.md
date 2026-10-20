# HyperBREP Architecture

Procedural parallel pcurves borrow their source definition through Hypercurve's
`Curve2::analytic_parallel()` view, including analytic selected-fiber restrictions.
HyperBRep's [parallel archive records](../src/persistence/parallel.rs) own format
mapping and pre-decode resource counts; Hypercurve rechecks regularity and
selected ranges. No offset formula or proof flag is duplicated in BRep.
`Curve3::planar_image` now supplies the exact spatial lift. Its immutable
source, injective plane frame and affine parameter chart are owned by
[`geometry/planar_image.rs`](../src/geometry/planar_image.rs). For spatial
parameter `s` in `[0,1]`, the source parameter is `a+(b-a)*s` and the point is
`origin + u*x(t) + v*y(t)`. Reversal and subdivision change the chart; an affine
spatial transform changes the frame. It does not re-offset a transformed curve.
Queries delegate planar evaluation, derivative and incidence to Hypercurve.
The frame cross product proves injectivity, and its four lifted source-bound
corners give a conservative spatial bound even after restriction when those
bounds are available. Retaining an arbitrary source is not a finite-loop
certificate: source poles and other query failures remain typed, and face
admission still requires the complete loop proof.

Whole-edge admission compares complete definitions after an exactly proved
plane-chart similarity. It also checks the explicit source-to-edge parameter
map. Endpoint checks supplement this proof; matching endpoints cannot replace
it. Translated extrusion profiles reuse the same identity proof. Retained
procedural wires use Hypercurve's complete simple-loop arrangement proof,
including intrinsic span injectivity and directed endpoint parameters.
Cap pairing and transverse translation then use the existing prism validator.
No represented area or volume is required to certify that topology.

The [planar lift admission](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-planar-curve-lift/README.md)
covers the original full nonlinear parallel cap, its quadratic closing spans,
planar faces, path/region extrusion, holes/components, exact queries, edge
splits, affine replay and repeated same-profile slab cuts/restoration. The
[selected-image admission](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-planar-images/README.md)
extends point queries and topology splits to retained selected roots. A point
owns its original spatial source, parameter and planar point evidence. Frame
linear forms delegate to Hypercurve's `CurvePointAffineForm2`; no independent
coordinates or copied offset formulas are needed. Cold trim admission compares
the complete source/frame/chart identity after exact endpoint correspondence.
Injective adjacent source pieces may share a simpler proof image while their
topology stays intact. Complete horizontal-prism certificates support subsequent
slab operations without scalar vertex views. Newly authored source chart endpoints
without scalar views, full source-history/intent receipts and general surface
intersections remain separate work.

Finite curve/plane contacts live in
[`geometry/curve_plane.rs`](../src/geometry/curve_plane.rs). Positive-weight
spatial Béziers reduce to the homogeneous plane numerator; Hypercurve owns
its complete root census and derivative-sign queries. NURBS decomposition
retains authored knot parameters and original selected spatial images.
Planar images reduce the plane to an affine form on their actual consumed
source interval. Hypercurve owns chart-preserving restriction, finite bounds,
contacts and span-to-source parameter transport. A pole outside that interval
does not invalidate the finite restriction; a pole inside it blocks a definite
answer, including containment.

`CurveSurfacePoint` carries `Parameter`, `GeometricPoint3` and
`CurveSurfaceContactKind`. `Simple` and `Repeated` describe nonzero versus zero
first derivative of the contact equation, including both incident spline spans;
they do not promise global crossing behavior or an integer multiplicity.
`Boundary` identifies a singleton clipped from coincident support.
`CurveSurfaceIntersection::Components` retains mixed isolated points and
maximal closed parameter ranges. Ranges own their endpoints; separate source
visits to one spatial point retain separate parameters. Complete overlap is
`Contained`. Hypercurve's complete overlap proof covers the interior; BRep
also checks the transported closed endpoint images and their plane membership.
An ambiguous full-multiplicity planar spline joint retains the owner's typed
`Boundary` blocker. Unambiguous interior contacts and finite restrictions away
from the jump remain consumable. Coefficient-only consumers explicitly request optional scalar
views and preserve typed failures. A boundary mapper can reuse its supplied
represented parameter only after exact equality proves it is the retained root.
The [contact admission](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-retained-plane-contacts/README.md)
records this public API migration and the supported topology/format consumers.

The shared surface-pcurve materializer also consumes a contained planar image
when the projected source frame is an owner-certified similarity. It restricts
the original source chart and returns an explicit signed affine correspondence
to the spatial interval, including reversed mappings. It does not rebase the
procedural source. General non-similarity projections remain typed owner
construction failures; finite plane incidence alone does not prove that a
particular planar carrier can encode the projected image.

HyperBREP is an exact, immutable boundary-representation kernel.
`hyperreal::Real` is the sole authored coefficient scalar. Approximate numbers may
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

The downstream [HyperBRep IO crate](../../hyperbrep-io/README.md) owns bounded
Part 21 syntax, supported STEP geometry mapping and source-entity accounting.
It consumes the native model interface and Hyperreal's exact numeric conversion;
the core does not depend on the exchange layer.
Parsed records confer no geometry authority. Source-scoped views and flat
parameter storage preserve references and avoid recursive parameter-tree drop.

## Retained rotation orbits

`Curve3::rotation` stores an exact point, authored unit axis and finite angular
domain of at most one turn. Its chart is `theta = s*t+b`, with `s` equal to +1 or
-1. Authored angular evaluation applies the existing affine rotation primitives
to the retained point. Reversal and restriction preserve the source; similarities
transport the axis as an axial vector, including the orientation sign under
reflection. Bounds conservatively enclose the full orbit. Positive-order vector
derivatives require stored origin coordinates; independent selected angular
parameters and general affine orbit transport remain typed frontiers.

The [rotation implementation](../src/geometry/rotation.rs) owns these invariants.
Selected revolution patches use it for their latitude boundaries. Ordinary
face admission checks complete coordinate images and profile-axis clearance,
including during cold reconstruction. Those checks do not certify general
rank-two surface regularity, global embedding or solid interior. The
[finite evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-revolution/README.md) records composition and replay.

## Closed edges and periodic seam uses

`ModelBuilder::edge` may reference the same vertex twice for a circle, ellipse
or retained rotation whose exact parameter interval spans one `tau` period.
Positive radii and independent axes prove the native conic is regular and simple.
A retained rotation instead certifies a unit axis, a nonzero radial component
and a signed angular chart with derivative +1 or -1. Both endpoint images must
still agree with the vertex. A shorter interval returns `ClosedEdgePeriodMismatch`; other closed
curve families return `UnsupportedClosedEdge` until they have an appropriate
regularity certificate. Equality of endpoint coordinates alone is insufficient.

Vertex adjacency lists each incident edge once. Edge uses remain distinct:
a cylinder seam can have two uses on opposite boundaries of one UV rectangle.
A circular edge can bound an entire cap and one full-period side interval.
The seam's spatial endpoint identity never identifies the two parameter
endpoints. Directed angular-sweep and affine correspondences retain those
endpoints and interior parameters through reversal, subdivision and replay.
Existing whole-image, cap, shell-incidence and periodic-coverage checks remain
mandatory. This admission does not add general ellipse/plane image proofs,
closed spline certificates, general chart gluing or curved STEP mapping.

Hypercurve owns the corresponding planar rule. A `CircularArc2` with equal
endpoints denotes a full circle; the arc carrier owns radius and incidence
checks. Its constructor rejects known zero radius or unequal endpoint radii;
unknown structural facts still require certification by exact consumers.
`CurveString2` rechecks zero-length chords only for lines, which can
be constructed unchecked. Full-circle contours use the existing single
`CurveRegion2` carrier, winding, Boolean and offset algorithms. See the
[closed-edge qualification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-closed-edges/README.md).

Disconnected selected solids retain these records through the shared topology
transfer. A finite cylinder trim has a conservative face bound from its boundary
curves: each Cartesian coordinate is affine along axial lines that reach the
trim boundary. This supplies the existing disjoint Boolean path with a per-face
proof, preserving selected closed edges without transferring unselected bodies.

## Retained inverse parameters

[`Parameter`](../src/parameter.rs) holds an authored `Real` or a certified
selected algebraic root. `try_real` borrows an available scalar view without
reconstruction. `compare` uses strict Hyperlimit order for two represented
values and Hypercurve's retained comparison otherwise. It exposes no infallible
mathematical `Eq` or `Ord`, and an unresolved comparison remains an error.
Use `OperationContext::evaluate` to share instrumented solver work across a
batch. The current implementation retains Hypercurve's admitted Bézier root
authority; it adds no independent root-isolation or coefficient-field engine.

[`geometry/location.rs`](../src/geometry/location.rs) owns spatial inverse
queries. For a rational Bézier, a varying planar projection proposes certified
parameters. Represented parameters retain the existing full-3D evaluation
check. Selected roots use a certified constant remaining coordinate, or replay
a second projected point at the **same** root and compare its exact image.
Independent roots of different projections cannot establish spatial incidence.
Positive weights are already an admission invariant. NURBS queries reuse
Bezier-span decomposition and Hypercurve's affine root transport to retain
authored knot parameters; exact comparison deduplicates shared seams.
Partly constant preimages remain an explicit unsupported result.

The [inverse-query qualification record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-retained-inverse-parameters/manifest.json)
records that increment's covered cases and limits. The selected spatial
evaluation added subsequently is described below.

## Retained spatial points

`Curve3::point_at(&Parameter)` returns `GeometricPoint3`. Authored scalar
parameters on authored carriers store `Point3` coordinates. Retained translation
and rotation origins can preserve a selected source even at scalar parameters.
A selected line, rational Bézier, finite NURBS or planar image retains its
immutable source curve and one shared parameter; `try_coordinates` returns `None`
without attempting materialization. Conic angular parameters still require a scalar view and
otherwise return `UnsupportedSelectedEvaluation`.

[`geometry/point.rs`](../src/geometry/point.rs) owns the spatial evidence.
`coincides_with` first checks shared point/source identity, then compares all
three coordinates through Hypercurve's existing selected-image predicates.
Different parameters do not prove different points. An unresolved coordinate
does not prevent a definite separation in another coordinate from proving
noncoincidence. Both projections use the same selector; independently chosen
roots cannot establish a spatial point.

`compare_linear_form(direction, rhs)` compares `direction · point` with `rhs`
under strict predicates. It maps authored source controls to the scalar linear
form and reuses the same selected parameter and rational denominator through
Hypercurve. It certifies oblique plane membership and sides without separate
coordinate reconstruction. This projection is local to the query; there is no
unbounded cache keyed by arbitrary plane coefficients. Typed uncertainty can be
retried after stronger scalar evidence.

For native curve images, the XY and XZ projections and their selected points are
prepared independently on demand. Successful projection preparation is cached, while failed work remains
retryable. Cold readers may duplicate preparation before publishing a shared
success. Source geometry owns no point or model reference, preventing a cycle.
Bounds are conservative exact enclosures, with no promised width. Affine
transforms certify invertibility, transform the source controls and reuse the
selector; they do not add a chain of point-expression wrappers.

The
[qualification record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-retained-spatial-points/manifest.json)
records that increment's covered cases and limits. The topology transfer
implemented subsequently is described below.

## Selected topology trims

`ParameterDomain` describes authored carrier domains with `Real` coefficients.
`ParameterRange` describes canonical increasing edge trims with exact
`Parameter` endpoints. Vertex records store `GeometricPoint3`. Builder incidence
checks compare retained images; they do not demand stored coordinates.
Scalar coefficient algorithms explicitly request an available scalar or
coordinate view and return a typed blocker otherwise.

`EdgeRef::split(Parameter)` restricts the original spatial carrier and splits
every incident pcurve in traversal order. An affine pcurve relation transports
the same selector through Hypercurve. Selected restrictions preserve their
source chart and the original relation; native reparameterized pieces get
exact recomputed affine coefficients. Validation checks the actual map at
both endpoints. It cannot infer agreement merely because a queried parameter
is an endpoint.

The exact subdivision inherits the source edge-image, wire simplicity,
orientation, nesting and solid certificates: a partition of a validated
boundary preserves its image. Publication still checks connectivity, ownership
and parameter correspondence. Arbitrary replacements invalidate those proofs.
No-op edits retain them. These in-memory certificates are not trusted archive
input.

Hypercurve's `Curve2::parameter_endpoints` exposes traversal order separately
from its parameter-domain order. BREP reflection uses this distinction to
reverse retained pcurves without reflecting their source parameter chart.
Spatial transforms reuse the transformed arena curve for retained vertices;
the source lookup is built only when a selected vertex needs it. Bounds union
selected-point enclosures with the existing conservative curve bounds.

The [topology qualification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-selected-topology/README.md)
covers shared-edge splits, repeated edits, rational/NURBS charts, reflections
and retained solid queries. [Archive version 12](SELECTED_ARCHIVE.md) retains
selected roots, flattened charts, spatial curve images and authored pcurve
restrictions. Independent replay covers the selected sheet fixtures and exact
planar area, every cuboid edge alone or together (including shear/reflection),
and cylinder axial subdivisions. Optional prism proofs project selected cap
sources into the cap frame and retain their trim ranges. Generic convex planar
proofs use retained linear-form comparisons for vertex half-spaces. When a
wire lacks scalar vertex coordinates, the planar signed-volume proof integrates
its checked boundary image; native scalar triangle fans remain the fast path.
Cylinder proofs reconstruct parameter rectangles from checked boundary images.
All topology records survive, and archives supply no trusted solid proof flags.
See [solid replay evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-selected-solids/README.md).
General selected solid/surface/contact graphs and Boolean transfer remain open. Native
coefficient subdivision and angular conic evaluation retain their explicit
scalar requirements. WP02/G1 and WP13 are not complete.

Spatial curve domains now use `ParameterRange`. Retained planar lifts expose
an authored source scale/offset rather than two scalar endpoints; selected
source intervals keep their chart. Straight axial sweeps use `Curve3::translation`
to preserve a selected origin. Hypercurve owns scalar-chart, affine-point and
point-definition replay; BRep owns the spatial frame and complete extrusion
image proof. Coordinate pcurve maps preserve selected UV corners through edge
splits and reflection. Archive version 12 records these definitions and rejects
versions through 11 without an adapter. General recursive selected fields remain outside
the supported replay domain.

Independent planar spline image validation compares the complete lifted pcurve
with the spatial curve restricted to the mapped edge interval. The shared
affine lift handles rational Bezier/quadratic controls and transforms NURBS knot
charts, including reversed parameter direction. Selected source restrictions
can therefore replay after prior native subdivision without requiring the
source pcurve to span the entire spatial curve. The archive contract also
defines Hypercurve's complete-image injectivity gate for boundary reassembly;
connected multi-span NURBS with one common strict monotone coordinate can pass.
See [composition evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-spline-replay/README.md).

Archive decoding requires explicit `ArchiveLimits` for input bytes, JSON depth,
tokens, records, degrees, variable geometry elements and topology references.
A constant-storage syntax scan precedes a resource-only metadata pass. Stateful
Serde seeds carry the shared budget through arenas and detached selected sources;
metadata summaries contain no exact scalars or per-entry collections. Successful
inputs read version and metadata together. The full record decoder runs only
after admission, and model validation still rebuilds every geometry proof.
The [archive contract](SELECTED_ARCHIVE.md#input-admission) defines exact units
and retry behavior. Allocation/scalar-work limits and an iterative WASM-safe
record decoder remain separate work; structural admission is not a total bound.

## Source organization

The private [topology module](../src/topology/mod.rs) owns arena storage and
shared certificate records. The crate root remains the public type facade;
there is no parallel legacy model API.

| Module | Responsibility |
| --- | --- |
| `ownership/` | Canonical public model references, scoped construction keys, owned operation results and transactions |
| `construction.rs`, `boolean_kernel.rs`, `meshing.rs` | Private constructor, Boolean and derived-output algorithms |
| `boolean_kernel/sets.rs` | Regularized finite-union operands, intersection distribution, successive subtraction and connected-component normalization through the shared solid kernel |
| `context.rs` | Explicit strict operation configuration and shared Hypersolve budgets; strict authority at fallible public operation boundaries |
| `cache.rs` | Shared publication of successful immutable certificates; failed queries remain retryable |
| `parameter.rs`, `geometry/location.rs`, `geometry/point.rs` | Retained parameters, spatial inverse queries and shared selected point images |
| `topology/ids.rs`, `records.rs`, `correspondence.rs` | Typed identifiers, immutable topology records and exact parameter relations |
| `topology/model.rs`, `builder.rs`, `error.rs` | Arena access, adjacency, staged construction/publication and typed failures |
| `topology/transfer.rs` | Common full-model and selected-solid transfer; source-order remapping retains curve images, parameter correspondence, voids and validated solid certificates |
| `topology/edit/` | Edge/face/surface splitting, multi-trace partitioning, endpoint attachment, transactions and edit results |
| `topology/validation/` | Image agreement, wire/shell checks and existing analytic/prism/sweep/loft family certificates |
| `topology/query.rs`, `measure.rs`, `transform.rs` | Classification, exact measurements and immutable transforms |
| `topology/helpers.rs`, `tests.rs` | Shared exact support routines and topology regressions |

This split preserves the existing algorithms and admitted families. Complete
operation accounting, general curved-shell validation and retained algebraic
parameters remain tracked in the
[implementation ledger](IMPLEMENTATION_STATUS.json).

Curve, surface and model bounds and face contours retain successful results
only. A predicate or resource failure belongs to its query, so later requests
can benefit from stronger scalar evidence. Concurrent cold readers may repeat
the calculation; publication keeps one successful value, and a failed reader
uses a success published before its final cache check. Warm reads do not
recompute. See the [cache regression evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-14-retryable-caches/manifest.json).

## Operation scopes

`OperationContext::strict` takes the existing Hypersolve `WorkBudget`, exposed
at the crate root alongside `BudgetOutcome`, `BudgetExceeded`, `WorkKind` and
`WorkUsage`. `evaluate` establishes one synchronous strict scope for a closure.
Put a sequence of modeling operations inside that closure to share a total
budget. Each evaluation of the reusable configuration creates a fresh scope;
nested scopes also charge every parent and cannot reset a parent's allowance.
A child with a smaller allowance may fail while its parent can still proceed.
Unwinding restores the prior scope without refunding its already admitted work.

Resource exhaustion is the outer `BudgetOutcome::result` error; the callback's
domain errors, unknowns and completeness reports remain in its return type.
An exhausted scope cannot return that value even when the callback ignores a
nested denial. The scope does not roll back arbitrary caller side effects.
The strict policy is queryable and suspends ambient Hypercurve approximation
and preview authority. Existing individual BREP operations continue to enforce
strict policy even outside a bounded batch.

The two counters cover existing instrumented solver refinements and
promotions, with the same units and cache-sensitive usage as
`hypersolve::work_budget`. Scalar precision refinement, remaining geometry and
elimination work, degree/coefficient growth and allocations are not completely
charged. This is not a time or memory bound. Worker allocations, async scopes,
cancellation, detailed proof-stage receipts and remaining counters are open
WP01 work. The plan's broader `WorkLimits` sketch remains a target; unsupported
limit fields are not exposed as if they were enforced.

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

The public `VertexRef`, `EdgeRef`, `FaceRef`, `SolidRef` and corresponding
reference types borrow one immutable revision. Traversal preserves that owner;
arena positions alone cannot resolve entities. Builder keys resolve only in
their construction scope and its published revision. Model clones share
identity; transforms, committed edits and decoded archives create new
revisions. Owned operation results retain the revisions needed by their
borrowed detail views, including intermediate multi-trace split steps. See
[ADR 0002](architecture/0002-model-ownership-and-validity.md).

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

`EdgeRef::split` is a topology-changing exact edit. It splits every
incident pcurve, updates forward and reversed wire traversal, and republishes
only after complete validation. Affine correspondences are rederived from the
new domains; angular correspondences invert through Hypercurve's certified
rational-conic parameter solver.

`FaceRef::split` is the planar region counterpart. Two nonadjacent
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
Every admitted edge connecting distinct height levels must also preserve x/y;
a skew extrusion cannot become a world-z Cartesian product from its cap alone.

## Exact image agreement

Endpoint agreement is necessary but not sufficient. Each supported
curve/pcurve/surface tuple has a complete-interval structural proof:

- affine line images on planes;
- circular pcurves on planes through directed angular sweep;
- axial lines and constant-height circles on cylinders;
- latitude and meridian circles on spheres;
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
- positive-weight `2×2` rational Bézier plane sections, reconstructed by exact
  rational-quadratic parameter-graph substitution into the complete
  weighted rational-quartic spatial control net. Sign-changing graph
  denominators are partitioned at exact poles and tensor-boundary contacts;
  factorized relations retain their native iso-lines.

`ModelBuilder::validate_edge_image` exposes this same check for a staged edge,
direction, pcurve, parameter correspondence and surface. It checks all three
key owners before geometry work, runs strict predicates even in an ambient
approximate scope, and does not stage topology or install an inherited proof
flag. A caller can retain alternate source pcurves in the builder and check
each before constructing face-local uses. Face construction still validates
the selected geometry independently. `Ok(())` proves only the complete trim
image and directed correspondence; it does not establish shell validity or
fidelity to a different source. Unsupported, unresolved and invalid outcomes
remain distinct typed errors.

Circular iso-curves on cylinders, spheres, cones and tori share one complete
angular comparison. After restricting the spatial circle to the edge domain,
the validator checks centers, radii, signed angular span and both vectors of
the radial/tangent frame at the common domain start. Reversal and phase changes
are included in that frame. The older direct-domain cosine/sine form remains
an alternate exact proof when the anchored comparison is unresolved. This
preserves its certified cases while avoiding doubled inverse-angle phases for
reversed imported circles. Matching only the center and one tangent is
insufficient: a wrong phase or tilted circle can pass that test. Six such false proofs were
reproduced and repaired; the
[image validation evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-edge-image-proof/README.md)
retains the failures, parameterization matrix and downstream qualification.

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
an exact axis result or a typed predicate/scalar error; it cannot
panic through `Real::atan2`'s convenience refinement floor.

Fallible scalar division, elementary functions and homogeneous transforms
retain their `hyperreal::Problem` in `GeometryError::Numeric`. Domain errors,
unresolved zero tests and exhausted refinement remain distinguishable through
the public error variants and standard error cause chains. The exact meaning
of these outcomes is specified in
[ADR 0001](architecture/0001-exact-decision-contract.md#outcomes).

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
- fixed-frame rational Bézier path sweeps of polygonal regions, including
  through-holes whose cap loops and tensor walls form one genus shell; their
  connector curves are exact translates, tensor sides reproduce the complete
  translated control net, and normalized profile-plane progress is certified
  affine and strictly positive by exact Bernstein coefficient identities;
- analytic cylinders;
- complete closed-surface spheres, spherical voids, exact plane-capped axial
  sphere segments, and exact two-cap sphere/sphere Boolean shells;
- truncated cones that exclude the singular apex;
- full periodic ring tori and exact axial torus bands closed by zero, one, or
  two planar annular cap groups.

Certificates are compact retained facts used for exact volume and point
classification. They are rebuilt from geometry and topology after persistence.
Supported transforms update the certificate directly and preserve its proof.

Successful builder admission now retains its positive strict family proofs
until `finish` or drop. These partial records differ from a validated transfer's
complete available proof set: a missing admission proof still invokes the
existing recognizer. Whole-solid sphere/cylinder facts include their supported
voids; outer-shell facts are reusable only without voids. Finalization still
derives the secondary prism proof and applies the prism/loft eligibility rules.
Ownership, orientation, nesting and global orphan validation remain mandatory.
Edits invalidate certificates before replay. The private line-pcurve rewrite
cannot reach an admitted boundary, and later appended faces cannot change a
shell-scoped proof. Retention is one owned record per solid, with no global key
or eviction policy; this does not bound retained bytes, total work or cancellation.
See the [proof-scope review and executed evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-builder-certificates/README.md).

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
source parameter intervals and rational images while avoiding an unsupported
one-vertex closed spline edge. Hypercurve classifies the radial/axial boundary
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
that line's geometric moment, independent of its projective speed. The same
complete collinearity and monotone-control proof lets revolution faces use the
straight meridian's exact radius-times-length integral without treating
projective speed as arc length. HyperBREP applies the exact Pappus factor and
cavity subtraction. Rational images
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
curvature unrestricted. For a region sweep it additionally reconstructs one
positive outer contour and every negative cap-hole contour, proves every side
group tiles its complete tensor boundary, and classifies and measures the
outer-minus-hole material section. Moving frames and corner-continuity
policies are not inferred.

## Queries and operations

Exact face measurement integrates the actual carrier Jacobian. Planes,
cylinders, extrusion surfaces, spheres and spherical caps, cones, and tori use
family-specific exact routes; no curved boundary is replaced by chords.
Line-bounded prism shells may retain an affine-image rational Bézier or NURBS
tensor face. The complete Bernstein/Greville lattice certificate derives a
canonical geometric plane for cap grouping, orientation, volume, and point
classification while leaving the authoritative face carrier and pcurves
unchanged. Merely coplanar control points are insufficient: a non-affine
in-plane tensor remains explicit unsupported shell evidence.
Extrusion measurement uses a constant Jacobian only for line profiles and
circles swept normal to their plane. A four-edge rectangular trim of a
certified planar rational Bézier/NURBS profile instead measures the exact
profile chord crossed with the authored direction when every positive-weight
control projection is transverse-monotone. This covers curved in-plane strips
and projectively parameterized line images independently of projective speed.
Profiles outside that plane or with transverse backtracking return
`UnsupportedMeasurement`.

Standalone planar faces consume Hypercurve `CurvePath2` loops directly.
Hypercurve certifies complete fragment injectivity, path simplicity, pairwise
loop non-contact, containment, and non-nesting before HyperBREP authors one
plane, its spatial edges, exact pcurves, wires, face, and open shell. Lines,
rational Béziers, and finite NURBS retain native persistent carriers;
circular, polynomial Bézier, rational-quadratic, and polynomial B-spline
inputs are promoted exactly to persistence-supported rational Bézier or NURBS
boundaries without fitting or sampling.

Path extrusion applies the same certified region boundary to two z-plane caps
and one exact extrusion carrier per path curve. Cap edges and pcurves retain
their native line, circular, rational Bézier, or NURBS domains; polynomial
families are promoted exactly. The prism certificate retains the complete
`CurvePath2` outer-minus-hole region, so exact volume and point
classification never reduce spline boundaries to chords.

Region extrusion asks the authoritative `CurveRegion2` component for its
certified borrowed line/arc view before converting boundaries to spatial
curves. Available circles remain native circles; other persistent curves use
the existing exact conversion. Failure to prove a view remains distinct from
an unavailable view. The original region certificate remains authoritative.
This intentionally changes some boundary tags and archive bytes from rational
Bézier spans to native circles; it is a representation migration, not a
performance-only optimization.

Prism replay derives cap area from retained spatial conic integrals when
available, projected into the certified cap basis. Other paths retain their
exact path-area calculation. The area is a transient validated fact, never an
unchecked serialized value. Signed cap area and the oriented extrusion
determinant also establish shell orientation when a chord mesh would have zero
volume, including an annulus bounded by two semicircles per loop. Inward shells
remain invalid. A cylindrical lateral face requires an exact rectangular trim,
axis parallel to the cap translation and cap normal, and matching axial bounds;
this extends the shared prism validator used by replay and STEP import.

Finite revolution patches share one native latitude extractor between
construction and validation. Latitude edges are compared against the complete
expected circle parameterization, including frame, domain, and directed
surface-parameter correspondence; a matching endpoint and one tangent are not
treated as an interval proof. One patch spans strictly less than `tau` so a
complete revolution remains explicit periodic topology. Construction also
certifies the complete profile interval strictly off-axis through a
control-hull separating plane or exact two-plane axis-contact reduction;
unsupported higher-degree intersection cases remain explicit.

Regularized z-prism Booleans delegate planar arrangement and role assignment to
Hypercurve, then reconstruct line/arc material contours and holes as newly
validated BREP solids. Empty, incompatible, unsupported, and unresolved
outcomes remain distinct.
Intersection uses the overlapping axial interval; union requires equal
intervals. Directed difference accepts a cutter interval containing the entire
source interval, including equal endpoints. Partial coverage, an exact 1e-12
gap, touching, and disjoint intervals do not satisfy that difference rule;
other existing Boolean routes may still complete them. The same operation
handles replayed right prisms in a common orthonormal chart when both certified
axes are exactly parallel. Skew or nonparallel axes are outside this route.
No second planar kernel, tolerance or sampled recognition is introduced.

Rationally rotated split circles expose two necessary scalar identities:
principal rational atan reciprocal normalization and half-atan sine/cosine
reduction when the rational argument has a rational hypotenuse. These proofs
belong to Hyperreal; BREP keeps the original strict comparisons and full-size
rotated inputs. This is the reproduced dependency exception to the branch's
BREP-focused ownership. Other trig branches and fields retain their existing
representations and failure contracts.

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
deduplicated before circle publication. A coaxial revolution/revolution query
instead proves that both supported affine-control meridians occupy one
strictly positive radial half-plane, projects them into a shared exact
`(radius, axial-height)` chart, and delegates their complete contacts to
Hypercurve. Represented parameters lift directly to latitude circles. Equal
or reversed complete meridians are coincident; partial overlaps, incomplete
evidence, and algebraic-only parameters stay explicit. Equal radial frames
retain both constant-profile pcurves, with an exact `tau-angle` map for a
counteroriented second axis. Rotated frames retain only the spatial circle
because their modulo-angle correspondence crosses the authored seam and is
not one affine pcurve. Plane/extrusion generator lines use
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
A positive-weight `2×2` rational Bézier tensor has a bilinear homogeneous
plane numerator even when its affine image is not polynomial. When either
graph denominator has one certified strict sign, HyperBREP degree-elevates the
graph to a positive-weight rational quadratic. Exact Bernstein products of its
homogeneous `U`, `V`, and `W` controls substitute that pcurve into the four
bilinear corner basis functions. The resulting spatial homogeneous numerator
is `Σ wᵢⱼPᵢⱼCᵢⱼ`, its denominator is `Σ wᵢⱼCᵢⱼ`, and both form one
five-control rational quartic. Mixed graphs are clipped in the unit parameter
rectangle before spatial composition, so a pole in the off-domain rational
surface extension cannot invalidate a represented in-domain fragment.
If the denominator changes sign in the retained interval, HyperBREP inserts
the exact roots of the denominator, numerator, and numerator-plus-denominator.
Those roots are respectively graph poles and contacts with the solved
coordinate's zero and one boundaries. Each resulting open interval has
constant membership and denominator sign, so one exact midpoint comparison
selects the bounded cells before local positive-weight rational quadratics are
composed spatially. A common numerator/denominator root factors the bilinear
relation into one or two native iso-lines instead. For a sign-definite
Bernstein hull, the zero-control mask exactly distinguishes one corner, two
opposite corners, one boundary edge, or two boundary edges; the public
`Point`, `Points`, `Curve`, and `Curves` results retain those dimensions
without a compatibility wrapper. A complete zero hull is two-dimensional
containment rather than a curve intersection; it returns
`ContainedSurface(operand)`, naming the complete bounded tensor domain that
forms the intersection. This is distinct from `Coincident`, which remains
reserved for equal complete carriers. Face-pair enumeration retains the exact
containment relation and projects the contained face's oriented exact spatial
boundary into the plane operand's parameter space. Hypercurve intersects that
mixed-family `CurveRegion2` with the bounded plane face region, so
`FacePairTrim::SurfaceRegion` represents full or partial two-dimensional
overlap without sampling; an empty intersection is `NoContact`. A lazy exact
region difference records whether the plane face covers the complete contained
face. On the plane operand, projected boundary fragments must carry a
Hypercurve exact line-image certificate before they are promoted to complete
planar support traces and passed through deterministic multi-trace partition.
The contained operand needs no split when coverage is complete. Partial
tensor-side partition remains explicit unsupported evidence because it
requires inverse tensor pcurves; neither endpoint chords nor fitted parameters
are accepted.

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
`SurfaceIntersectionPcurve::point_at(&Parameter)` returns a retained Hypercurve
`CurvePoint2`, with exact domain checks and no coordinate reconstruction.
`geometry/pcurve.rs` owns evaluation and correspondence. Native rational section
spans and plane projections preserve selected parameters; iso/retained mappings
reuse their source authority. Planar procedural projections currently require a
similarity of the source chart. Face classification accepts the retained UV
point directly; coordinate algorithms explicitly request an optional view.
Selected angular/procedural surface images and full selected section restriction remain
open boundaries, documented in the [evaluation record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-pcurve-evaluation/README.md).
`Surface::point_at(&hypercurve::CurvePoint2)` returns `GeometricPoint3`.
`geometry/surface_point.rs` validates retained UV domains; plane evaluation stores
the immutable surface and the joint UV point directly. Spatial affine-form
queries share the planar owner's proofs with curve images. Point transforms map
the source plane frame without changing the UV selector. The private represented
surface evaluator remains the common coefficient path. Endpoint image validation
consumes retained pcurve and surface points directly. Archive 12 persists surface
point definitions and generic translation/rotation origins through
`persistence/spatial_points.rs`; nested shape admission precedes scalar decoding.
See the [surface evaluation record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-surface-evaluation/README.md).
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
For an authored-frame-aligned coaxial cylinder/cone pair, the cone slant
parameter is obtained directly from the common radius and exact semi-angle.
The retained full circle carries constant-height cylinder and constant-slant
cone pcurves. Bounded cylinder/frustum graphs clip it across matching periodic
quarter cells without fitting or root isolation.
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
`FaceRef::split_by_surface_curve` attach an intersection to opposite
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
Weighted bilinear sections use the stronger direct construction proof:
validation orients the retained rational-quadratic pcurve, recomposes all five
weighted rational-quartic spatial controls, and requires complete
parameterization equality. This applies unchanged after represented rectangle
clipping or exact pole-cell partitioning and rejects an endpoint-preserving
middle-control forgery. Factorized and sign-definite boundary components use
native tensor iso-curves, so their existing complete-control proof remains the
authoritative validator.

`FaceRef::split_by_surface_curves` accepts retained intersection objects
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

## Analytic similarity transport

`Model::transformed` first certifies an invertible affine map. A per-operation
`TransformMetric` lazily proves `AᵀA = s²I`, `s > 0`, once and shares the positive
scale across analytic carriers and shell certificates. Unit directions become
`A*d/s`; points follow the affine map. Radii, axial bounds and void dimensions
scale by `s`, while angles, selected curve parameters and dimensionless profile
parameters retain their meanings. Polynomial carriers keep general affine
transport. Circular/elliptical curves also retain their previously supported
plane-isometry case when the unused normal is transformed differently.

Cylinder/cone charts map `(u,v)` to `(u,s*v)`. For straight source pcurves,
Hypercurve's exact similarity restricted to that line implements this map and
retains selected-source lineage. Nonlinear axial pcurves require future exact
affine carriers and fail typed. A pcurve shared by distinct face chart maps is
separated into records; the first variant keeps its original slot. Reflection
also reverses wires, directed uses and parameter correspondences. An extrusion
reverses its transformed profile and reflects `u` about the profile domain
endpoint sum; its affine extrusion vector and `v` coefficient remain paired.
This repairs the old mismatch between reflected `u` and a negated `v` vector.
Independent replay certifies complete forward or reversed profile equality and
the matching parameter map. Reflected subcurves may have translated parameter
origins; only the comparison chart is shifted, with equal-span and full carrier
proofs retained. Matching endpoints alone are insufficient. Elliptic
revolution meridians use the same existing complete-parameterization proof.
No metric
proof is cached across operations or mutable inputs; transformed query caches
start empty.

All analytic shell certificates transport physical dimensions and directions,
including sphere/cylinder voids, clipped bodies, paired spheres, tori and cones.
Revolution certificates scale their complete 2D contour/path profiles through
Hypercurve. Prism, loft and sweep affine bases retain their own dimensionless
coefficients. A torus axial-band integral normalizes height by minor radius and
uses Hyperreal's existing exact quadratic-tower reduction before `sqrt`/`asin`;
this preserves recognizable special angles after radical similarities.

Persistence projects planar pcurves and writes full-domain rational edge/profile
endpoints from the actual emitted vertex coordinates rather than runtime vertices
replaced by curve-derived canonical coordinates.
This keeps radical scale/inverse replay deterministic. It does not reconstruct
selected coordinate images or change their authority. The
[admission evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-analytic-similarity/README.md) records
original failures, exact expectations, composition, archive effects and cost.
The source/work/heap/cancellation limits of the existing kernel still apply;
this transform extension does not establish global resource admission.

## Retained profile images

Retained planar spline restrictions enter the existing extrusion representation
through Hypercurve's `Curve2::native_image()`. The image borrows its proving
source and uses exact subcurve substitution for represented selected endpoints;
its native chart can differ from the source chart. BREP consumes the complete
image and still validates the whole wire, cap and shell. Shared image/area
proofs belong in Hypercurve, while spatial lifting and product-solid continuation
belong in BREP. Empty exact planar material intersection proves product
subtraction is unchanged; retaining two solids for union additionally requires
two separate planar material components. This continuation follows existing
optimized and graph paths. Native geometry replay and a complete persistent
authoring graph remain separate contracts. The
[selected-profile proof record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-profile-extrusion/proofs.md)
states the admitted domain and remaining limits.

## Seam-aligned analytic cuts

Closed-set curve/region clipping retains positive-length boundary overlaps as
well as interior intervals. Hypercurve owns that incidence proof. BREP must not
discard an overlap by subsequently classifying its midpoint as boundary. Native
iso pcurves are lifted by whole surface periods into the actual authored trim
chart before clipping; spatial parameters and contact records remain paired.
Each periodic axis admits at most three intersecting copies, independent of
how many periods separate the chart origins. Wider chart spans remain typed
unsupported. This bounds copy enumeration, not total scalar work or memory.

`topology/edit/boundary.rs` proves complete coverage of a materialized line
pcurve by existing outer/inner boundary line uses. It checks the retained
mapping's native representation before materializing; nonlinear families
continue through ordinary partitioning without constructing this line-only
probe. Exact paired overlap ranges
must cover its entire normalized interval without gaps. It then invokes the
native complete spatial-curve/pcurve/surface image validator on the actual face,
without accepting the trace's inherited proof flag. Covered traces leave the
face unchanged in a multi-trace partition; a single-face split reports
`DegenerateFaceSplit`. Partial overlaps, endpoint agreement and uncertain
incidence are not coverage proofs. Nonlinear boundary pcurve coverage remains
outside this path.

`FaceTracePartitionRef::boundary()` exposes `BoundarySurfaceTraceRef`, its full
intersection, selected operand, actual `source_model()` and overlap records.
Each `BoundaryTraceOverlapRef` borrows its source edge use and paired normalized
trace/boundary ranges, including reversed parameter orientation. Earlier face
partitions may have subdivided shared edges, so the proof owns that staged
revision. Interior split records precede boundary records; every record retains
its caller `source_index`. A covered trace has one original spatial segment and
no splits. These records do not yet constitute complete operation provenance.

Planar cap assembly coalesces duplicated seam reports only after a complete
parameterization proof. Selected circular edges with different chart origins or
reflected frames reuse topology's existing full center/radius, angular-span,
start-radial-vector and signed-tangent proof. Closed edges require both traversal
trials even when both endpoint tests match. Same endpoints or common support
alone never certify the selected arc. Unsupported infinite carrier intersections
can be excluded when Hyperlimit certifies strict separation between a plane and
the other finite face's bounds; touching and Unknown remain unrejected.

Native line construction and plane-point projection compact exact coordinates
through Hyperreal's bounded `compact_pi_laurent` API before later construction
multiplies the expressions. Persistence now shares that plane projection instead
of maintaining a duplicate formula. Axial chart transport likewise compacts
line-direction differences before computing its restricted similarity. Torus and
cone area magnitudes consume strict signs before introducing an unnecessary
square root of a squared area. Unrecognized forms retain the original exact
values; no recognition budget, predicate policy or archive limit is increased.
Canonical expression bytes can change under the existing archive schema.

The [evidence record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-seam-cuts/README.md) retains the original
full-size seam failures and the subsequent repeated-use failures, including the
70 MB swapped-cone line archive. The repeated-composition extension below addresses those specific cap/recut/reunion
failures. Arbitrary periodic traces, general Boolean closure, general torus/cone STEP
encoding and project-wide resource/provenance/delivery gates remain open.

## Repeated curved composition

`FacePairTrim::CoincidentFaces` replaces `CoincidentPlanar`; its two trace lists
contain retained `SurfaceIntersectionCurve` values, including both actual
surface charts, source intervals and clipping contacts. This is an intentional
breaking API change without an alias. Planar straight boundaries retain their
existing support-line arrangement; native circular boundaries retain finite
curves. Cone/torus coincidence is admitted only when origins, oriented frames
and all carrier parameters agree exactly and their validated boundary uses have
affine iso-parameter maps. Existing bounded whole-period lifting moves only the
target chart. Image coincidence alone does not establish a shared parameter map.

Prepared region fragment indices do not identify authored edge uses. BREP contact
mapping queries all actual boundary uses of the appropriate contour role through
Hypercurve's native `point_locations`, retaining every supported incident
parameter before applying the authored edge correspondence. Unrepresented
selected-root values remain typed blockers at the existing contact boundary. Circular evaluation parameters use native rational
charts; angular sweep fractions are not interchangeable with those parameters.
Missing, unsupported or unresolved preimages remain typed failures. A strict
native line-side certificate can exclude unrelated straight boundary uses before
constructing incidence equations. On-line and blocked classifications retain
the complete native inverse, and invalid input errors propagate. No additional
cache or approximate predicate is introduced.

A circular boundary no-op requires complete, closed native overlap coverage and
full spatial image validation on the actual face carrier. It handles subdivided
boundaries, holes and reversed charts, including circular images retained as
rational curves. `BoundaryTraceOverlapRef::native_overlap()` exposes the original
span overlap witness; its public paired ranges use the actual pcurve charts.
The proof retains its source model revision. Wrong spatial interiors are rejected
even when endpoints and an internal cached image flag agree. These records do
not yet constitute complete operation provenance.

Coplanar cap traces with equal parameterizations are coalesced by exact interval
union including angle offsets. Different frames require either an existing
complete period on the same circle support or full closed native overlap coverage
of a complete circle candidate. Gaps and complementary parameter ranges that
actually map to the same semicircle never establish full coverage. The graph
retains original face incidences. Cone transverse sections share the authored
meridian parameter and radial image, avoiding a separately reconstructed tangent
radius. Cone/torus shell validators accept different arena records only after
proving the same complete parameterization; all embedding, coverage, incidence
and orientation checks remain required.

The original three full-size half-cut fixtures now support recut intersection,
subtraction to canonical empty with recovery, and complementary reconstruction.
Both orders are exercised where applicable, at identity and six exact
similarities: 63 configurations and 126 continuation outputs. Independent exact
volume/area, inside/outside/boundary and near-offset classification, opposite
paired edge uses, deterministic untrusted replay, subdivision and reflected
rescaling accompany every nonempty result. The
[evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-curved-coplanar/README.md) preserves the complete
fifteen-case diagnostic frontier, failed candidates and cost observations.
The sector extension below continues the five quarter/band cases. Differently
parameterized coincident surfaces, general curved shells, complete operation
receipts and resource/release gates remain open.

## Analytic sector and band continuation

Native line partitions incident on an inner wire share the retained-surface
bridge arrangement. A complete bridge cycle must pass full wire simplicity,
orientation and nesting validation before its new wires acquire reusable
simplicity certificates. Endpoint attachment alone cannot justify them; a bridge
crossing a hole is rejected. The public partition retains original trace indices,
source/output model revisions, canonical ordering and exact segments.

Planar circular traces share exact connected-interval union even when other
traces are straight. Only a proved complete period joins the first and last
components across a seam; all other gaps remain. Full-circle tests for different
frames remain in force. The sphere path still requires complete circular coverage.

Clipping uses `MaterializedSurfacePcurve` and its actual correspondence. Native
circular charts use their nonlinear rational-to-angular map; a fraction of the
rational domain is not an angular fraction. Restriction and reversal transport
both source intervals and span order. A periodic seam retains distinct start/end
parameter identities even when coordinates agree. No additional approximate
carrier or tolerance is introduced.

Materializing an iso curve reduces scalar-proven Laurent coordinates before
those values become line coefficients. This preserves exact angular endpoints
through repeated interpolation and untrusted replay after irrational reflection.
The source parameter correspondence and every unrecognized exact expression are
retained. The failed full cone-band fixture remains in the evidence record.

The cone/torus shell validators admit complete convex angular sectors and axial
bands only on the supported native rectangular UV grids. Curved cells have
multiplicity one. Every selected angular column has exactly the same admitted
axial mask, lies wholly in each longitudinal halfspace, and the sum of its
interval widths equals the independently derived halfspace-intersection angle.
Intervals are ordered inside one period; intervals wider than pi, gaps, wrong
sides and partial columns fail this proof. Strict endpoint and midpoint signs on
an interval of width at most pi prove its sinusoid remains in the halfspace.

There are at most two planes containing the axis and two perpendicular to it.
Cone caps coincide with the carrier limits; torus axial caps are strictly between
its natural extrema. Every planar face has the required support and outward
orientation, with validated images, wires and oppositely paired shell edge uses.
These planes have no bounded three-dimensional arrangement cell. A compact closed
oriented boundary with the certified curved part therefore has a unique planar
completion: the difference of two completions would be a closed planar chain
with nonzero winding in an unbounded cell. This argument depends on the entire
restricted arrangement and carrier coverage. It is a mathematical justification,
not a machine-checked proof of the Rust implementation, and does not admit
arbitrary clipping planes or general curved shells.

Planar line/conic area integrates `(r-origin) cross dr` using authoritative spatial
angular intervals. Choosing a retained conic center as the common wire origin
preserves the closed integral while avoiding translated endpoint terms that would
otherwise cancel after subdivision. The existing scalar Laurent reducer and
strict sign proof retain a usable signed magnitude. Unrecognized forms retain
the original exact absolute value. Pure polygon paths and general spline
measurement contracts remain distinct. The ordinary wire validator handles
multiple circle supports and exterior chords outside the one-disk specialization;
a valid two-edge arc/chord wire may share both cyclic endpoints.

One reproduced BREP blocker required a Hyperreal correction after checking
upstream: `Computable::acos` now preserves `acos(1/2)=pi/3` and
`acos(-1/2)=2pi/3`, including values discovered by its existing rational normal
form. This lets a retained `pi/2-acos(1/2)` contact compare exactly with `pi/6`.
No BREP-specific scalar identity or larger recognition budget was added.

The [acceptance corpus](../tests/support/sectors.rs) keeps the original full-size
cutters and strict policies. Five original and 30 exact similarity configurations
exercise 175 nonempty continuation outputs and 35 canonical empty results.
Each nonempty output has independent exact volume/area, analytic point tables
including 1e-12 offsets, paired opposite edge uses, deterministic raw replay,
subdivision, reflected rescaling and further replay. This finite corpus does not
establish arbitrary-angle or general Boolean closure. That historical increment
kept ten typed unsupported torus/cone STEP controls. Ring-torus exchange now
admits the exactly representable torus cases; cones and nonrepresentable scalar
encodings retain typed diagnostics and same-context supported recovery. See the
[evidence record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-sector-cuts/README.md) for executed commands,
failed candidates, costs and remaining gates.

## Exact ring-torus exchange and repeated angular replay

`TorusGeometry` borrows the native right-handed frame and radii; authoritative
geometry remains in `Surface`. The STEP adapter owns `TOROIDAL_SURFACE`, its two
angular coordinates, length units and exact finite-decimal encoding. It builds
latitude/meridian UV candidates through native circle incidence, preserves a
continuous loop lift and translates by an exact period into the native chart.
Whole-image validation admits those candidates. Distinct opposite seams remain
explicit, including the axis identity of supplied pcurves. Other trim families
and loops requiring additional periodic subdivision fail typed.

Repeated rational-band subdivision exposed lost affine contact expressions and
an inverse-angle equality unavailable at scalar replay. Planar cuts now retain
the validated preimage alongside the canonical edge parameter. The topology
owner normalizes only exactly rational affine split parameters. Equal authored
circle frames/phases give a positive whole-map certificate; endpoint sine/cosine
congruence can recover lifts in an already validated at-most-one-turn chart.
The existing alternate certificates remain available when these proofs fail.

The shared scalar normal form recognizes rational/pure-quadratic tangents with
an exactly rational sine, first-quadrant complementary acos arguments and
reconstructed rational scales through its existing bounded Laurent authority.
This necessary dependency repair replaces a rejected BREP multiplier cascade.
It introduces no numeric carrier, tolerance, terminal or larger recognition
budget. See the [ownership/proof review](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-torus-exchange/review.md)
and [executed evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-torus-exchange/README.md), including
four split/reload cycles, wrong-quadrant controls and preserved adverse results.

## Performance contract

Correctness does not vary by execution route. Retained adjacency, structural
carrier facts, immutable caches, broad-phase bounds, and compact solid
certificates select faster exact work. Numerical refinement or approximate
filters may only avoid work or propose candidates whose results are replayed
exactly.

See [`../PERFORMANCE.md`](../PERFORMANCE.md) for benchmark cohorts and current
baselines, and [`SUPPORT_MATRIX.md`](SUPPORT_MATRIX.md) for the exact operation
matrix.

## Periodic conic parameters in Boolean continuation

The native conic parameter is an identity in an authored chart, not just an
angle modulo one revolution. Inverse queries, plane contacts and Boolean edge
atomization now share `geometry::angular_parameter_lifts`. For a validated
unit-speed conic interval of length at most tau, a certified integer ceiling
selects the first equivalent parameter in that interval. A full-period seam
retains both endpoint identities; a shorter interval can return no preimage.
Stored endpoints are reused on certified equality. Enumeration does not grow
with the chart's turn count; scalar quotient and coefficient costs retain their
existing resource limitations.

The helper computes strict orderings against the interval start and end once
for each candidate parameter. That same pair decides membership and endpoint
identity. A periodic lift changes the value, so it receives a fresh pair of
comparisons. An undecided comparison still fails explicitly; the two endpoint
identities of a complete period remain distinct. This reuse lasts only for one
helper call and adds no persistent cache. The ordering cascade already proves
the structural endpoint equality previously checked by `ParameterDomain::contains`.
The [comparison evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-conic-comparison-reuse/README.md)
retains the complete output contracts and observed costs; no general resource
bound follows from fewer comparisons or allocations.

The periodic lift repairs a lost subdivision when a retained parameter pi/2 belongs to a
coalesced target arc spanning [7pi/4, 11pi/4]: the target preimage is 5pi/2.
Previously the parameter was discarded as outside the interval, leaving the
torus band cap unpaired. Direct inverse queries also previously lost valid
preimages in distant authored charts because normalization added only one turn.

`retained_curve_face_intervals` receives the actual spatial curve and obtains
native boundary contacts before recovering its clipped endpoints. For an
interior angular pcurve parameter, `retained_spatial_parameter` reuses a native
contact only after certifying equal full spatial parameterization and finding
one lift in the target domain. The native trim contact supplies the proof that
the point is on this fragment; matching endpoints or circle supports alone
does not justify reuse. Original pcurve endpoints use the retained correspondence
so the two ends of a periodic seam remain distinct. Other correspondences and
ambiguous lifts continue through the existing exact mapping.

The [acceptance corpus](../tests/support/oblique.rs) retains the original
40-unit cutter rotated 45 degrees relative to a radius-5/minor-radius-2 torus,
then clips the half by z >= 1. Independent volume, area and point expectations
exercise intersection in both orders, subtraction, reconstruction in both
orders, subdivision, exact similarities and archive replay. Chart controls use
both conic families, reversed/restricted curves, turns through +/-10^12 and
1e-12 seam gaps. Wrong phase and different-support contacts are negative controls.
For minor radius 2 and major radius 5, the disk segment above z = 1 has area
S = 4pi/3 - sqrt(3). Rotation through pi gives volume 5pi*S. The curved area is
20pi^2/3, the axial annular cap has area 10pi*sqrt(3), and the two longitudinal
caps contribute 2S. The complement uses the remaining disk area and adds the
same shared axial cap. These expectations are derived independently of BREP
measurement; the exact similarity cases apply the corresponding square/cube
scaling laws and transformed point tables.
The [evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-oblique-cuts/README.md) also retains all nine
wider oblique fixtures and their failures. This extends parameter evidence
reuse; it does not supply a general curved-shell embedding proof or general
oblique Boolean closure.

## Axial-shell certificates and native carrier transport

Topology's [axial-shell admission](../src/topology/validation/axial_shell.rs)
proves complete plane/cylinder boundary structure before publishing a partial-depth
solid. All native lateral edge/chart images are constant through each open
height band. Shared axial edge identities assemble simple section loops whose
individual winding transitions prove multiplicity one; Hypercurve owns the
resulting material regions. Exact cap differences, event contacts and connected
vertex links establish the transitions and embedding. Separate void shells and
general curved embedding remain outside this certificate.

`CertifiedAxialShell` is private immutable model data: a frame, exact event
heights and region/area bands. It is indexed by the owning solid, scoped to that
model's geometry and strict proof. Builder admission and validated transfer
retain it; edits clear it and cold archives reconstruct it. There is no public
proof flag, separate mutable cache or global identity key. Stored regions and
vectors have no independent retained-byte budget; this is not a heap or total-work
bound. Native volume and point classification consume the same certificate,
including exact event-side ownership and transformed frames.

[Cylinder chart transitions](../src/geometry/periodic_charts.rs) prove complete
angular sign/phase and axial sign/displacement maps. Boolean selection retains
the source UV witness and checks the opposite authored trim directly. Native
iso traces keep complete spatial/pcurve images; meridian support subdivisions
only partition the existing face and preserve original intersection incidences.

Native line/circle extrusion walls now publish planes/cylinders with explicit
chart and orientation maps. This is a greenfield representation/API migration,
with the measured artifact differences recorded in the [feature evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-17-axial-shells/README.md).
Before a supported non-similarity transform of a certified prism, exact
[carrier promotion](../src/topology/transform/affine_prisms.rs) restores a finite
extrusion representation over the actual angular trims. Its equation and pcurves
are unchanged, preserving the original axial-shear domain. Other unsupported
analytic affine images still fail explicitly; no fitting or tolerance is used.

### Retained full revolutions

`construction/revolution.rs` connects retained planar meridians and exact
rotation latitudes across four angular cells. Point-ordered source chords use
Hypercurve's complete scalar-chart proof before becoming surface coordinates.
`validation/retained_revolution.rs` replays the face rectangles, single-cover
angular/profile grid, common meridian plane and exact projected profile. Curve
simplicity and winding plus strictly positive radial bounds prove the product
shell's embedding. Area/volume formulas are separate optional queries. Native
primitive/path fast paths keep their existing carriers. `revolve_curve_region`
uses the unified region's material components and hole ownership, including
empty input. The [admission record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-procedural-revolution/README.md)
limits this proof to its supported domain; general shell embedding remains open.

## Rational surface points and affine curve images

`Surface::point_at` admits selected joint UV points on positive-weight rational
Bézier and finite NURBS tensor patches, and extrusions of native line/Bézier/NURBS
profiles. The source surface and entire UV point remain replay authority.
`geometry/surface_point/rational.rs` derives homogeneous coordinate polynomials;
Hypercurve's `CurvePointRationalForm2` evaluates their signs, comparisons and
bounds in the retained joint field. Original denominators are checked before
any cancellation. Hypersolve owns Bernstein and original-knot B-spline basis
conversion and homogeneous recursive-field evaluation. NURBS span ownership is
right-sided at internal knots and includes the final endpoint. Basis conversion
uses an explicit -512 precision floor and preserves undecided/invalid statuses.

Derived coefficient maps are immutable and point-owned. The point retains its
source surface, with no back reference or global cache. Transforms rebuild the
maps from the transformed source and unchanged UV selector. Bounds are exact
conservative enclosures, not promised-width approximations or topology proofs.
Selected angular surfaces and general retained procedural extrusion profiles
remain unsupported. Full section restrictions and branch/event graphs remain
open independently of point evaluation.

`Curve3::affine_image` retains any spatial source under a certified nonsingular
affine matrix. Its `AffineImage` family has a borrowed `affine_geometry` view;
repeated images compose matrices instead of wrapper chains. Evaluation,
derivatives, bounds, inverse location, reversal and restriction delegate to the
source with its typed limitations. Regular angular closed edges remain regular
under the invertible map. General transforms of retained rotations use this
carrier when their native metric path is unavailable. Native conic model transforms keep their existing
metric restriction until complete face-image and replay proofs are available.
Coefficient-only and unsupported surface-intersection algorithms remain typed
blockers; an affine image is not mislabeled as a circle or rotation.

Archive 12 stores the complete source curve and matrix and admits nested source
metadata before decoding scalars. Rational surface maps are recomputed from
source definitions during replay. Versions through 11 are deliberately rejected,
without an adapter. Source-path native/WASM consumers and preserved complete
workflows are recorded in the [admission evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-rational-surface-evaluation/README.md). Installed
distributions, full release/push gates and complete BRep modeling remain open.


## Joint surface differentials and unit-vector queries

The public differential boundary takes `CurvePoint2` directly and returns
`GeometricVector3` partials and normals. One private represented evaluator keeps
the original analytic formulas. Selected rational tensors differentiate their
original homogeneous coordinate maps in `geometry/surface_point/rational.rs`;
profile extrusions retain the profile derivative. Native NURBS span ownership
and original denominator checks precede publication. A unit normal is distinct
from a merely oriented tangent direction.

`geometry/vector/algebra.rs` composes coordinate forms in Hypercurve's common
source field. That owner pulls back polynomial-fiber maps, native parameter
images, similarities and resolved endpoints; different selected branches cannot
be joined just because their base coordinate agrees. Binary forms retain an
immutable pair of original operands, including their checked pole guards.
Structural common denominators stay shared during sums. Debug output does not
expand the operand DAG recursively. No new cache or coordinate-root authority
is introduced.

A private positive-form proof retains the squared divisor of a root-scaled
vector. Sign and squared-polynomial identities compare its coordinates, dot
products and length without independently constructing square-root coordinates.
Affine transport changes the numerator vector and retains that divisor;
renormalization proves a new positive length. A surface's normal is computed
from its authored partial order, so a reflection carries the determinant sign.
Hypercurve's direction image discards only positive scale and permits parallel
unit normals and zero-dot products without claiming unavailable radical speeds.

`SurfacePartials::apply_direction` evaluates the Jacobian on a retained UV
velocity. Materialized pcurves use `Parameter::curve_parameter` and their exact
spatial-parameter correspondence; reversing that chart can require a negative
scale. Pointwise chain identities and cold replay are tested independently from
archive byte identity. They do not prove whole-section differential agreement
at singular events or complete trim topology. Vector proofs are transient;
archive 14 stays unchanged. Original shared guards can still grow with the input
composition: degree shortcuts and solver counters are not total-work, heap,
stack or cancellation bounds. The [admission record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-surface-differentials/README.md)
retains these limits and the full previous corpus.
