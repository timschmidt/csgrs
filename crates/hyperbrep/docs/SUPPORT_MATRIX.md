# HyperBREP Exact Support Matrix

This matrix describes certified behavior in the current clean-break API.
“Unsupported” means the operation returns a structured error; it never means
that sampled or tolerance-based geometry is substituted.

The updated [acceptance contract](../../../docs/admission.md) applies to
completed workflows, beyond individual rows in this matrix. Numerical
certainty, topology validity and fidelity to a source are distinct claims.
The [integration record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-upstream-integration/README.md)
retains current checks and open composition/delivery gates; no whole-stack or
released-package qualification is inferred from the carrier inventory.

## Exchange

[HyperBRep IO](../../hyperbrep-io/README.md) parses/writes the bounded Part 21
HEADER/DATA subset with exact numeric lexemes, simple/complex instances, typed
parameters, aggregates, original encoded string tokens and multiple DATA sections.
Unknown records and unresolved references remain addressable by source IDs.
Native tests and an executed WASM probe cover deep input, round trips and limits.
Syntax success alone confers no geometry authority. The typed STEP layer now
adds checked standard headers, decoded text, exact unit graphs, selected
faceted import, planar straight-edge advanced import with 3D/pcurve source
parameter maps, direct circular edges and supplied analytic pcurves on
planes/cylinders/ring tori, and exact faceted or selected analytic export. Circular source maps retain
independent angle units, sense and phase; full edges and seam uses remain distinct.
Holes and selected body occurrences are accounted for; unsupported records
remain in the source exchange. Import uses native strict validation without
implicit healing or tolerance welding. See the
[mapping contract](../../hyperbrep-io/docs/STEP_MAPPING.md),
[dependency decision](../../hyperbrep-io/docs/PARSER_REVIEW.md) and
[advanced-import evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-step-advanced/README.md).
Complete EXPRESS/AP conformance, whole-document/product traversal, assemblies,
general periodic seams, other analytic/spline exchange, void and remaining analytic export
and approximate export remain open. The
[circular import evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-step-circles/README.md) exercises
native edits, replay and Boolean/empty continuation separately from an
independent floating-point reader.

`write_advanced` preserves explicit native vertex/edge identity, every directed
use and face bound, orientation, and selected occurrence for plane/cylinder/ring-torus
solids with line/circle edges. Reports retain native sources, exact emitted
curve parameter intervals and UV chart maps. Native pcurves/history are not
serialized by this direct-curve exchange encoding. Decimal and homogeneous
direction representability, unsupported families and limits fail typed.
The [analytic-export evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-step-analytic-export/README.md)
includes independent graph checks, oblique/radical frames, split/reflection,
holes, units, near-coincidence, native replay, Boolean/empty continuation,
OCCT reading and real WASM execution. The subsequent
[native similarity evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-analytic-similarity/README.md)
retains the original frame failure and completes physical cylinder scaling,
reflection, exact unit export/reimport, native replay and Boolean continuation.
Unit selection itself does not transform the source geometry.

## Operation resources

`OperationContext` shares explicit refinement and promotion allowances across
a synchronous batch and its nested BREP/Hypercurve/Hypersolve calls. It returns
typed resource exhaustion and admitted usage while retaining strict predicate
authority. This is partial solver-work accounting: scalar precision, complete
geometry work, memory, deadlines, cancellation and worker allocations are not
covered. See the [operation-scope contract](ARCHITECTURE.md#operation-scopes).

## Planar polynomial branch prerequisite

Hypercurve consumes complete admitted rational bivariate zero-set graphs as
native branches and separate isolated points. Branches retain selected source
endpoints and ordered fiber identity through evaluation, subdivision, reversal
and similarity; bounds conservatively enclose the full source rectangle.
Correlated point definitions replay through the existing selected-fiber owner.
The BRep archive accepts these definitions in retained-chord endpoints, charges
both equation degrees and coefficient/row entries, and rejects forged singletons.
See the [finite branch evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-polynomial-branches/README.md).
Mixed affine coordinates now remain selected in the same algebraic field and
carry replayable point/form recipes. Existing BRep scalar charts and planar
spatial images consume them through complete edge validation, transforms and
deterministic native archive replay; see the
[image evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-selected-fiber-images/README.md).
This does not supply general rational planar/spatial maps, branch derivatives,
filled arrangements, tensor-surface images, both section charts or whole
implicit-branch archives. The original
public biquadratic interior-loop section still reports `UnsupportedIntersection`.

Materialized pcurve correspondence now accepts retained `CurveParameter2`
values and returns spatial `Parameter` values, with strict domain membership.
Selected retained/iso mappings preserve restrictions, reversal and checked
archive replay; selected iso carriers use degree-one splines in the original
chart. Represented iso carriers retain native lines. Angular selected interior
queries still fail typed when their coordinates are unavailable. The complete
section evaluator and selected angular/procedural surface images remain open boundaries.
See the [parameter-map evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-selected-pcurve-maps/README.md).

Retained pcurve `point_at` accepts `Parameter` and returns
`hypercurve::CurvePoint2`, preserving selected UV images. Native rational
sections/projections, retained/iso charts and admitted planar-lift similarity
charts support exact queries and point-graph replay. The face classifier accepts
these images directly; periodic coordinate arithmetic explicitly requests a
stored view. Selected angular and general procedural non-similarity projections
remain typed blockers. `Surface::point_at` consumes joint UV points and retains plane, rational tensor and native rational-profile extrusion images; selected angular and general procedural images remain typed blockers.
See the [evaluation evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-pcurve-evaluation/README.md).

`Surface::point_at(&hypercurve::CurvePoint2)` returns `GeometricPoint3`.
Represented inputs preserve all nine surface evaluators. Selected plane images
retain the source surface and joint UV authority, with exact affine queries,
conservative source bounds, transforms, translation/rotation origins and archive
12 replay. Finite and lower-bounded domains are checked before representation
dispatch; periodic axes retain their unbounded real semantics. Selected angular and general procedural
images remain unsupported. Bounds retain the planar owner's representation
limits. See the [surface evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-surface-evaluation/README.md).

## Spatial curve carriers

Procedural analytic-parallel pcurves replay quadratic/cubic/homogeneous rational
sources, signed distance, selected parameters and traversal. The spatial
`PlanarImage` carrier retains that source in an injective affine plane frame.
Represented endpoint intervals use a unit spatial chart; selected intervals
retain a signed source chart with exact `ParameterRange` endpoints. Native scalar
queries and inverse location reuse the planar owner. Selected point queries
retain that owner's evidence for exact affine comparisons, conservative bounds,
transforms and native replay; absence of scalar coordinates is an unavailable
view. See the [selected-image admission](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-planar-images/README.md).

| Family | Evaluate / derivatives | Bounds | Parameter location | Reverse / split | Persistence |
| --- | --- | --- | --- | --- | --- |
| Affine image | Source-supported exact point/derivative mapped by a certified invertible matrix | Conservative image of source bounds | Source query in inverse coordinates; original typed limitations | Source domain/restriction semantics | Exact source and matrix, archive 12 |
| Planar image | Represented or retained selected point, including selected finite-domain endpoints; owner-supported represented-parameter derivatives (first order for analytic parallels) | Conservative lifted finite source or point bound | Plane membership plus exact source inverse query and chart filtering | Exact represented restriction / reversal; selected topology edge split | Exact for admitted pcurve sources, source charts and selected points/trims |
| Translation of retained point | Authored parameter retains the original point; constant first derivative | Conservative union of endpoint bounds | Unsupported | Exact reversal and authored-parameter split | Exact retained origin and authored displacement |
| Rotation of retained point | Authored angle retains source; positive-order derivatives require stored origin coordinates | Conservative full-orbit enclosure | Unsupported | Exact reversal and authored-angle split | Exact origin, axis and angular chart |
| Finite line | Exact, arbitrary positive derivative order | Exact | Certified | Exact | Exact |
| Circular arc | Exact angle semantics | Conservative exact | Certified, including seams | Exact | Exact |
| Elliptic arc | Exact angle semantics | Conservative exact | Certified | Exact | Exact |
| Rational Bézier | Homogeneous exact | Positive-weight control hull | Certified algebraic decomposition | Exact | Exact |
| Finite nonperiodic NURBS | Homogeneous de Boor / arbitrary derivatives | Positive-weight control hull | Certified Bézier-span decomposition | Exact knot insertion | Exact |

Planar-image archive sources include lines, circular arcs, quadratic/cubic
polynomials, rational quadratic/general Beziers, NURBS, analytic parallels and
admitted source restrictions. Polynomial B-spline source archives remain typed
unsupported; builders retain their existing exact NURBS promotion. Version 10
stores explicit source-chart coefficients and retained point definitions. Finite plane
contacts delegate to the planar owner after proving finiteness on the consumed
source interval, including represented restrictions of signed-weight sources.
General carrier intersections, independently selected axial translation parameters, closed planar-image
edges, general revolution certificates, and canonical ordering of unlike
procedural sources remain typed frontiers. Extrusion inverse queries are exact
when the retained source plane is transverse to the extrusion direction.
No STEP family or general metric integration is added.

`Curve3::parameters_of` returns `Parameter` values, including selected algebraic
roots without a stored `Real` view. Rational Bézier queries certify all three
coordinates at the same selected root. NURBS queries preserve authored knot
charts and deduplicate seam parameters. Partly constant NURBS preimages remain
`UnsupportedParameterLocation`; the API does not return a finite subset of an
interval preimage.

`Curve3::point_at(&Parameter)` returns `GeometricPoint3`, retaining selected
images for lines, rational Béziers, finite NURBS and planar images. It supports strict
coincidence/coordinate/linear-form comparisons, conservative enclosures and certified
invertible affine transforms. NURBS selectors stay in their authored knot
chart. Planar images retain the source point and evaluate their frame's affine
forms through Hypercurve. Scalar parameters expose stored coordinates for
authored carriers; retained translation/rotation origins can keep selected
source images even at scalar parameters. Independently selected conic/rotation
angular evaluation returns `UnsupportedSelectedEvaluation`.

Native coefficient derivatives/subdivision still require stored
parameters/coordinates. `Curve3::domain`, closed surface domains and contact-pcurve
domains use `ParameterRange`; coefficient consumers request fallible represented
views. Archive version 12 retains selected geometry graphs, correlated point
definitions and straight translations of retained origins. Topology vertices and edge trims retain selected
images and parameters; affine pcurve transfers and exact edge splits preserve
them. A remaining scalar-only path returns `UnrepresentableParameter` or
`UnrepresentablePoint`. Selected sheet fixtures, cuboid edge subdivisions and
cylinder axial subdivisions independently replay. General selected solid
recertification and complete resource bounds for decoding remain open. See the
[archive contract](SELECTED_ARCHIVE.md). See the
[parameter contract](ARCHITECTURE.md#retained-inverse-parameters) and
[spatial point contract](ARCHITECTURE.md#retained-spatial-points).

Circle, ellipse and nondegenerate retained rotation edges may retain one
vertex at both endpoints when their exact interval spans one full period. The
vertex lists that edge once; incident edge uses and parameter endpoints remain
distinct. Single-edge circular caps
and a cylinder with one periodic side face validate, subdivide and replay.
Standalone closed ellipse and retained rotation edges subdivide and replay;
this does not add ellipse/plane pcurve-image admission. Other closed curve families return
`UnsupportedClosedEdge`. See the [closed-edge contract](ARCHITECTURE.md#closed-edges-and-periodic-seam-uses)
and [evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-closed-edges/README.md).

Retained rotations preserve their point source, unit axis and signed angular chart.
Similarities and reflections transport that chart; general affine transforms,
independent selected angular parameters, retained-vector derivatives and general
inverse/intersection queries remain typed frontiers. Selected revolution patches
prove complete boundary images and axis clearance, not global embedding or solid
interior. See the [finite qualification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-revolution/README.md).

## Surface carriers

| Family | Domain / evaluation / partials | Bounds | Supported topology image proof | Transform | Persistence |
| --- | --- | --- | --- | --- | --- |
| Plane | Exact | Unbounded | Line, circular arc, rational Bezier/NURBS and retained planar-image agreement | Affine | Exact |
| Cylinder | Periodic `u`, unbounded `v` | Unbounded | Axial line and constant-`v` circle | Certified similarity / reflection | Exact |
| Sphere | Periodic longitude, closed latitude | Exact cube | Complete closed-surface face without seam/pole edges | Certified similarity / reflection | Exact |
| Cone | Periodic `u`, lower-bounded `v` | Unbounded | Generator and constant-`v` circle away from apex | Certified similarity / reflection | Exact |
| Ring torus | Periodic `u` and `v` | Exact cube | Both constant-parameter circle families | Certified similarity / reflection | Exact |
| Linear extrusion | Profile domain × unbounded | Unbounded | Line/circle/ellipse/rational Bézier/NURBS/planar-image profile and extrusion direction | Family-preserving | Exact |
| Revolution | Periodic × profile domain | Conservative exact | Native meridians/latitudes; coordinate charts also retain selected planar-image meridians and rotation origins with complete axis clearance | Certified similarity / reflection | Exact |
| Tensor rational Bézier | Closed × closed | Control hull | Exact iso-boundary rational Bézier curves and subintervals | Affine | Exact |
| Tensor NURBS | Active closed domains | Control hull | Exact iso-boundary NURBS curves and subintervals | Affine | Exact |


`partials_at` and `normal_at` take a joint Hypercurve UV point. First partials
and unit normals are exact spatial vectors with optional stored coordinates.
Selected planes, positive-weight rational tensors and native/admitted retained
profile extrusions are supported; selected angular surfaces and unavailable
profile fields remain typed frontiers. NURBS preserve native knot ownership,
including nonsmooth internal knots and the closed final endpoint. The derivative
at a nonsmooth knot is the owned span's derivative, not a two-sided limit.

Shared-field spatial cross/dot/length/equality and normalization queries retain
positive root branches. Parallel-offset unit normals and zero-dot comparisons
can discard a proved positive speed factor; full radical derivative magnitudes
and their chain-rule consumers remain unsupported. `apply_direction` consumes
admitted UV derivatives, with explicit pcurve/spatial parameter correspondence.
The complete circle, cubic and projective fixtures supply both consumers.
Zero vectors, rank-deficient surfaces, invalid UV domains and unavailable field
joins remain distinct failures. See the [finite acceptance record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-surface-differentials/README.md).
These queries do not complete singular germs, trim arrangements or curved-solid
Booleans. The native model archive remains 14.

Extrusion-face measurement is exact for line profiles, circles swept normal to
their plane, and complete rectangular trims of planar rational Bézier/NURBS
profiles with certified transverse-monotone positive-weight controls. Other
variable-speed profile areas remain explicit unsupported measurements.
Affine-image rational Bézier/NURBS tensor faces with exact line boundaries may
also remain native inside a certified prism shell. A derived canonical plane
is used only as shell evidence; coplanarity without the complete affine
control-lattice proof is not accepted.

Analytic similarities certify `AᵀA = s²I`, `s > 0`, and transport lengths
separately from angular/profile parameters. Cylinder/cone axial pcurves require
straight native source support (including retained selected restrictions);
nonlinear axial images return `UnsupportedTransform`. Shared face charts may
require separate pcurve records. Plane and spline carriers retain their affine
domain. General nonuniform analytic promotion remains open. See the
[metric/chart contract](ARCHITECTURE.md#analytic-similarity-transport).

## Standard builders and solid queries

| Builder | Native topology | Exact face area | Exact volume | Exact point classification | Supported transform / persistence |
| --- | --- | --- | --- | --- | --- |
| `cuboid` | Shared line edges / planes | Yes | Yes | Yes | Affine / exact |
| `extrude` / `extrude_region(s)` | Line prisms, holes, components | Yes | Yes | Yes | Affine / exact |
| `extrude_with_voids` | Nested inward line-prism shells | Yes | Yes | Yes | Affine / exact |
| `extrude_contour(_regions)` | Native line/arc caps and plane/cylinder sides | Yes | Yes | Yes | Family-preserving / exact |
| `extrude_curve_region` | Normalized `CurveRegion2` material components and holes, retaining native line/arc images and the original component proof; empty material gives empty bodies | Exact for native line/normal-circle sides; persistent fallback families retain their measurement restrictions | Exact certified component area × height when represented; otherwise explicit unsupported query | Exact height interval and retained material-region test | Family-preserving / exact |
| `extrude_path(_region(s))` | Native `CurvePath2` caps and one exact side per persistent path curve (native lines/circles specialize to planes/cylinders); through-holes share one genus shell | Exact for line, normal-circle, and certified transverse-monotone planar rational Bézier/NURBS sides; other variable-speed sides unsupported | Exact Hypercurve outer-minus-hole area × height when represented; otherwise explicit unsupported query | Exact height interval and `CurvePath2` material-region test | Family-preserving / exact |
| `cylinder` | Circular caps and one cylinder carrier | Yes | `pi*r²*h` | Analytic | Certified similarity / exact |
| `sphere` | One boundaryless closed-surface face | `4*pi*r²` | `4*pi*r³/3` | Analytic radial test | Certified similarity / exact |
| `sphere_with_voids` | Complete outer and inward spherical faces | Yes | Exact sphere subtraction | Analytic radial test | Certified similarity / exact |
| `cone_frustum` | Circular caps and one cone carrier | Yes | Truncated-cone formula | Analytic | Certified similarity / exact |
| `torus` | 4×4 periodic native-circle grid | Yes | `2*pi²*R*r²` | Analytic implicit test | Certified similarity / exact |
| `revolve` / `revolve_region` | Four periodic cells per off-axis polygon edge; inward cavity shells | Exact line-profile integral | Exact first-moment theorem with hole subtraction | Exact radial/profile test | Certified similarity / exact |
| `revolve_contour` / `revolve_contour_region` | Four periodic cells per native line/arc profile segment; inward line/arc cavity shells | Exact line/circular-arc profile integral | Exact contour x-first-moment theorem with hole subtraction | Exact radial/line-arc profile test | Certified similarity / exact |
| `revolve_path` / `revolve_path_region` | Four periodic cells per exact `CurvePath2` carrier; retained Bézier/B-spline/NURBS and supported procedural meridians with selected latitudes, exact clamped knot-span partition of single periodic spline carriers, and curved cavity shells | Line/arc and certified monotone rational line-image faces exact; general curved spline faces explicitly unsupported | Exact for polynomial-equivalent, rational-quadratic, exact degree-elevated conic, arbitrary-degree at-most-quadratic-weight, and nonuniform rational line-image moments; cubic-or-higher weight-degree rational moments explicitly unsupported | Exact radial/curved-path test | Certified similarity / exact |
| `revolve_curve_region` | Unified region winding/holes; one solid per material component; canonical empty stays empty; native line/arc fast path and retained procedural/selected profiles | Existing per-family measurements only | Existing moments when available; unsupported integrals do not prevent solid admission | Exact radial/profile classification including cavities | Certified similarity / exact |
| `sweep` / `sweep_region` | Exact affine image of a polygonal linear-path sweep | Yes | Exact determinant-scaled prism volume | Exact inverse-frame profile test | Affine / exact |
| `sweep_curve` / `sweep_curve_region` | Fixed-frame polygon or through-hole region swept along an affine-progress rational Bézier path / native tensor translation sides and inner cap wires | Caps only | Exact material-region area × signed plane progress | Exact progress inversion and outer-minus-hole profile test | Affine / exact |
| `sweep_moving_frame` / `sweep_moving_frame_region` | Explicit shared-weight rational Bézier origin/axis frame / native tensor sides and inner cap wires | Caps only | Exact material-region area × signed plane progress × integrated frame-area law | Exact moving inverse-frame profile test | Affine / exact |
| `loft` | Two or more corresponding sections; each span positive homothetic or exact convex interpolation / planar or bilinear tensor sides with explicit C⁰ rings | Homothetic sides only | Exact piecewise-integrated quadratic section area | Exact span/section test | Affine / exact |
| `planar_face` | One exact `CurvePath2` outer loop plus disjoint holes in an authored plane, including supported selected procedural/source-restriction endpoints / open shell | Exact Hypercurve signed region area × plane-frame Jacobian when represented; otherwise explicit unsupported query | N/A | N/A | Affine / exact |
| `extrusion_patch` | One finite rectangular exact extrusion face, including selected profile domains with retained connector origins and authored sweep bounds / open shell | Line, normal-circle, and transverse-monotone planar rational Bézier/NURBS profiles exact; other variable-speed profiles unsupported | N/A | N/A | Family-preserving / exact |
| `revolution_patch` | One sub-period, completely axis-clear rectangular revolution face / open shell, including supported selected meridian endpoints | Line, circular-arc, and certified monotone rational Bézier/NURBS line-image profiles exact; other curved spline profiles unsupported | N/A | N/A | Certified similarity / exact |
| `rational_bezier_patch` | One trimmed exact tensor patch / open shell | Exact for a constant-weight affine Bernstein lattice under any validated trim, or the complete affine image under separable positive weights; general curved patches unsupported | N/A | N/A | Affine / exact |
| `nurbs_patch` | One trimmed exact tensor patch / open shell | Exact for a constant-weight affine Greville lattice under any validated trim, or the complete native-domain affine image under separable positive weights; general curved patches unsupported | N/A | N/A | Affine / exact |
| `tensor_patch_shell` | Multiple exact rational Bézier/NURBS patches; projectively identical boundaries identity-stitched | Per-face exact for the same affine tensor families | N/A | N/A | Affine / exact |

Retained path boundaries use Hypercurve's source-bound `Curve2::native_image`
view for complete oriented native geometry. Its line/arc path preserves the
`native_segment` traversal proof; selected Bezier spans use exact subcurve
substitution when their endpoints have proved scalar views. Source ranges require
range and traversal evidence; source fitting or endpoint sampling is insufficient.
BREP then validates the complete path and uses the native image's parameter chart.
The [retained-profile evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-native-boundary-images/README.md)
covers the mixed parabola/chord profile, reversal, affine maps, holes/components,
planar faces, cold replay and further native cuts/restoration. The live extrusion
retains its original region. Native geometry replay does not persist the full
authoring boundary graph. The selected parabola difference/XOR and affine
fixtures now have exact areas, native extrusion, edits and repeated slab
composition; see the [selected-profile record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-profile-extrusion/README.md).
The original nonlinear parallel cap now extrudes and replays with its full
procedural arch and quadratic closing segments. Authored paths and planar faces
share admission; holes/components, represented edge splits, nonuniform reflected
affine maps, exact near-boundary queries, empty identities and repeated supported
same-profile slab cuts consume the result. General selected-field integrals
remain unsupported. The
[spline exchange increment](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-spline-exchange/README.md)
adds exact clamped rational B-spline edge and linear-extrusion STEP exchange for
the admitted profiles. Independent authored records retain weights, knots,
interior trims, nonunit source charts and units. Imported solids validate, replay
and continue through the same supported global-z cuts/restoration. Native point
location supports oblique transverse charts; it does not alone admit general
oblique solid modeling. Whole-edge proofs remain mandatory. General spline
surfaces, arbitrary wall pcurves and ambiguous or unrepresented trims are open.

The [measurement admission increment](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-measurement-admission/README.md)
admits native rational prisms with no represented area integral. Complete planar
topology and winding prove orientation; paired caps and transverse translation
prove shell orientation. Positive material uses a simple outer loop and strictly
contained, disjoint holes. Degree-five Bezier/NURBS profiles cover reversal,
holes/components, affine images, split/replay, empty identities and repeated
same-profile slab cuts. An independent STEP fixture covers both source directions
and metre/mm units. The path extrusion front doors and `planar_face` consume the
same orientation proof; explicit outside-hole inputs still fail. Volume and unavailable cap-area queries stay explicitly
unsupported. Metric-evaluation errors still propagate; general certified
integration, deferred metric computation and selected non-native spatial
boundaries remain open. No stored trusted orientation or fabricated volume
replaces cold validation.

Full cones remain absent until the apex singularity has a topological
representation rather than degenerate tolerance sewing.

Moving-frame curved sweeps are supported through explicitly authored
`RationalBezierSweepFrame` data. Origin and both axes share one positive
rational Bézier weight vector; complete Bernstein identities must prove
parallel section planes and affine strictly positive plane progress. A
nonuniform rational frame must preserve oriented section area exactly; a
polynomial-equivalent frame may instead have a strictly positive Bernstein
area law, which is integrated exactly for volume. This admits exact authored
shear and taper while rejecting folds, uncertified rational area change,
inferred Frenet transport, and sampled frame decisions.
`sweep_curve` and `sweep_curve_region` remain the simpler fixed-frame special
case. Region holes are exact inner cap wires joined by reversed material-side
tensor walls in one genus shell, not detached voids.
Multi-section lofts expose
identity-shared C⁰ section rings and make no stronger continuity claim.
Non-homothetic spans are accepted only when the corresponding endpoint
polygons and their complete linear interpolation pass the exact
strict-convexity certificate.

## Intersections

| Pair | Exact outcomes |
| --- | --- |
| Finite line / plane | None, point, contained |
| Positive-weight rational Bézier / plane | Complete finite isolated contacts with retained selected parameters and original-source point images; none or contained |
| Finite positive-weight NURBS / plane | Authored-knot contacts, maximal coincident ranges, mixed components, none or contained; shared joints deduplicated without merging distinct visits |
| Planar image / plane | Owner-supported complete finite point/range contacts after bounds prove the consumed source interval; selected points retain the original planar authority and spatial frame |
| Circular or elliptic arc / plane | None, simple/tangent points with represented seam parameters, contained |
| Circular arc / sphere | None, simple/tangent points with represented seam parameters, contained |
| Transverse circular arc / cylinder | None, simple/tangent points with represented seam parameters, contained |
| Transverse circular arc / cone | Upper-nappe none, simple/tangent points with represented seam parameters, contained |
| Transverse circular arc / torus | Both radial sections, simple/tangent points with represented seam parameters, contained |
| Plane / plane | None, coincident, unbounded line |
| Finite line / sphere | None, tangent/simple points |
| Finite line / cylinder | None, tangent/simple points |
| Finite line / cone | None, tangent/simple points, generator overlap; lower nappe rejected |
| Plane / sphere | None, tangent point, full circle; authored-axis transverse circles retain exact pcurves on both carriers |
| Sphere / sphere | None, coincident, tangent point, full circle |
| Plane / cylinder | Perpendicular circle; oblique ellipse; axial-parallel none, tangent line, or two lines, with exact native affine axial correspondence and authored-periodic trim transfer |
| Plane / cone | Transverse lower none, apex point, or upper circle; axis-containing two lower-bounded upper-nappe generator rays with exact affine pcurves on both operands |
| Plane / torus | Axis-transverse none, tangent circle, or two latitude circles; axis-containing two meridian circles; parallel-to-axis outer point tangency or strict separation |
| Plane / extrusion surface | Transverse native line/rational Bézier/NURBS curve; parallel none or lifted profile-contact lines |
| Plane / revolution surface | Authored-axis transverse none, isolated axis point, or one/more exact profile-contact circles with native pcurves on both carriers; finite multi-span NURBS contacts map to authored knot parameters and deduplicate knot seams |
| Coaxial revolution / revolution surface | Line/rational Bézier/NURBS meridians in certified positive-radius axial half-planes reduce to exact Hypercurve contacts; represented contacts lift to latitude circles, equal or reversed complete meridians are coincident, and equal/mirrored angular frames retain both native pcurves |
| Parallel cylinder / cylinder | None, coincident, tangent line, or two axial lines |
| Coaxial cylinder / cone | One exact circle; authored-frame-aligned carriers retain cylinder height and cone slant pcurves |
| Coaxial sphere / cylinder | None, tangent circle, or two circles; authored-frame-aligned circles retain both exact pcurves |
| Coaxial sphere / cone | None, isolated apex point, one tangent/secant circle, or two circles; authored-frame-aligned circles retain sphere latitude and cone slant pcurves |
| Plane / linear-extrusion rational Bézier tensor patch | None or one native rational Bézier iso-curve |
| Plane / degree-1-v linear-extrusion NURBS tensor patch | None or one native NURBS iso-curve |
| Plane / one-axis-linear rational Bézier translation tensor | None or one complete native non-isoparametric rational Bézier curve |
| Plane / degree-1 translation-axis NURBS tensor | None or one complete native non-isoparametric NURBS curve |
| Plane / positive-weight `2×2` rational Bézier tensor | None; one/multiple isolated corner points; one/two native boundary or factorized iso-lines; one/multiple exact bounded rational-quartic branches, including retained denominator poles; or exact containment of the complete bounded tensor surface |
| Plane / positive-weight degree-one `2×2` NURBS tensor | The same complete rational-bilinear outcome matrix, with every retained line or rational pcurve mapped exactly into the authored active knot domains |

`CurveSurfacePoint` exposes `Parameter`, `GeometricPoint3` and
`CurveSurfaceContactKind::{Simple, Repeated, Boundary}`. The first two kinds
classify the first derivative of the contact equation, including both sides
of native spline joints; a repeated odd root can still cross. They are not
integer multiplicities or a global smoothness guarantee. `Boundary` denotes
a singleton clipped from coincident support, such as an upper-cone apex.
`Overlap` uses `ParameterRange`; `Components` exposes normalized isolated
contacts and closed ranges, whose endpoints belong to the ranges. Missing
root, derivative, finite-range or transport evidence returns a typed error;
partial contact sets are not successful complete intersections. The
[contact admission](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-retained-plane-contacts/README.md)
records exact replay, downstream split/solid use and the deliberate API break.
Closed overlap endpoints must have usable original-source point images on the
plane. A planar spline's full-multiplicity joint with ambiguous one-sided images
retains the owner's `Boundary` blocker; interior contacts and finite subcurves
away from that joint still succeed. Explicit one-sided parameter and open-range
carriers remain separate work.

After containment, `Surface::planar_pcurve` and the shared retained mapping
materializer accept planar images whose source frame projects through a
certified Hypercurve similarity. They preserve the consumed source interval
and signed map to spatial parameters, including reversed and nonunit charts.
General non-similarity procedural projections retain the owner's typed
`InvalidSimilarityTransform` construction failure.

Transverse plane/extrusion intersections apply the exact affine projection
along the authored extrusion direction; named conics whose projection needs a
new normalized carrier remain unsupported. Parallel extrusion directions lift
supported exact profile/plane point contacts into unbounded lines and reject
two-dimensional overlaps explicitly. Their line origins and revolution latitude
coefficients still need represented scalar views; selected plane contacts alone
do not extend those surface-result carriers. Finite curve results retain their exact
model-space `Curve3` plus an evaluable pcurve on each operand. The intersection
graph clips line and rational-Bézier extrusion sections and generator lines in
both trimmed parameter regions, retains Hypercurve's top-level source ranges,
and materializes the matching exact spatial subcurves. NURBS extrusion
sections are decomposed at their exact knot spans into rational-Bézier pcurve
graphs; each local trim range maps affinely back to the authoritative NURBS
parameter interval. `boolean::intersect_faces` exposes this carrier-and-trim
operation directly for any two validated faces, including rational Bézier and
NURBS patches in open shells. It returns `None` only for certified broad-phase
or carrier disjointness; unsupported carrier pairs and supported relations with
their trim evidence remain explicit. Native finite results use
`FacePairTrim::SurfaceCurveFragments`: each fragment retains its spatial
`Curve3`, both exact pcurves, and one shared public parameter domain. Exact
affine composition reconciles unit-domain rational Bézier fragments with
native-domain NURBS fragments.
Authored-axis transverse plane/revolution intersections instead solve the
exact finite profile/plane relation and orbit each isolated positive-radius
profile point into one spatial circle. The planar pcurve is its native exact
projection and the revolution pcurve is `(angle, constant-profile-parameter)`.
Axis contact remains an isolated point. A contained profile interval,
mixed singular-point/circle coverage, and coincident circles with distinct
profile parameters remain explicit unsupported relations rather than losing
their multiplicity.
Coaxial revolution/revolution intersections project each supported meridian
into a common exact `(radius, axial-height)` frame after proving every affine
control lies in one strictly positive radial half-plane. Hypercurve then owns
the complete line/rational-Bézier/NURBS contact calculation. Each contact with
two represented authored parameters lifts without fitting to a full spatial
latitude circle. Equal or reversed complete projected meridians publish
coincidence; partial overlaps, noncoaxial axes, profiles crossing the axis,
and incomplete contact evidence remain explicit unsupported relations.
Algebraic-only contact parameters return `UnrepresentableParameter` instead
of sampling. Equal radial frames retain `(angle, first-profile-parameter)` and
`(angle, second-profile-parameter)` pcurves; counteroriented axes use the exact
`tau-angle` map. A rotated radial frame still retains the exact spatial circle
but does not publish a discontinuous seam-crossing modulo-angle pcurve.
Authored-frame-aligned coaxial sphere/cylinder circles use constant-latitude
and constant-height pcurves directly. The graph clips them across the four
native cylinder patches and exactly coalesces adjacent common-support
fragments before partitioning the boundaryless sphere; both partitioned
operands retain certified volume and byte-identical persistence.
Coaxial sphere/cone intersections solve one exact quadratic in the cone's
native nonnegative slant parameter. Matching authored frames retain
constant-latitude sphere pcurves and constant-slant cone pcurves; a bounded
frustum therefore clips one complete carrier into four exact quarter-circle
fragments without inverse fitting. A sphere passing through the apex can
produce both an isolated apex point and a positive-radius circle; that mixed
dimensional result remains explicitly unsupported until the result enum can
represent both components without discarding either.
Coaxial cylinder/cone intersections use the same native cone coordinate
directly: the unique positive slant is
`cylinder_radius / sin(semi_angle)`. Matching authored frames retain the
cylinder height and cone slant pcurves on the common full circle. Bounded
cylinder and frustum faces clip that carrier into four exact quarter-circle
fragments; skew axes or offset center lines remain explicit unsupported
relations.
An axis-containing plane/ring-torus pair returns two native minor-radius
meridian circles. Their exact planar circular pcurves and constant-longitude
torus pcurves share the spatial curve domain. The graph clips an oblique
axial cutter across all four periodic torus latitude cells per circle and
partitions both torus and planar operands exactly. Parallel-to-axis planes
retain the exact outer point tangency at `major_radius + minor_radius` and are
certified disjoint when strictly farther away; interior offset and general
oblique torus sections remain explicit unsupported quartic cases. An
axis-containing cuboid cutter publishes either selected half-torus through
standard intersection or torus-minus-cutter difference. The closed
longitude-region certificate proves both meridian disks, exactly half of the
periodic longitude cells, complete latitude coverage, exact `pi²*R*r²`
volume, `2*pi²*R*r + 2*pi*r²` boundary area, halfspace-aware point
classification, operand reversal, rigid/reflection transforms, untrusted
replay, and exclusion from the whole-torus optimization profile.

For a positive-weight `2×2` rational Bézier tensor, the homogeneous plane
numerator is a scalar bilinear Bernstein polynomial. HyperBREP solves it as
`(u(v),v)` or `(u,v(u))` when one linear denominator has a certified strict
sign over the unit interval. Degree elevation yields an exact positive-weight
rational-quadratic pcurve. Homogeneous substitution carries every authored
surface weight into an exact rational-quartic spatial curve whose five controls
and weights are derived without fitting. A strict one-sided weighted
plane-value control hull proves disjointness; a complete zero hull is a
two-dimensional overlap and returns `ContainedSurface` naming the complete
bounded tensor operand. It is not conflated with equal complete carriers.
Face-pair enumeration retains this exact carrier relation, projects the
contained face's oriented exact boundary into plane parameters, and intersects
the resulting `CurveRegion2` with the bounded plane face. Full, partial,
holed, and empty results remain exact `SurfaceRegion` or `NoContact` trim
evidence. Exact difference proves complete contained-face coverage. Certified
straight images of represented contained boundaries transfer to deterministic
support-line partitions on the plane operand; a completely covered contained
operand needs no partition. A
sign-definite hull with zero controls exactly returns its one/multiple corner
points or boundary iso-lines. Mixed sections are clipped in UV through the
same exact tensor rectangle path used by translation graphs before the
retained graph is composed spatially. Therefore an off-domain denominator
pole cannot suppress a valid represented fragment. A retained denominator
pole partitions its graph axis together with exact contacts against solved
coordinates zero and one;
constant-sign cells inside the tensor rectangle become separate exact
rational-quadratic/rational-quartic branches. A common numerator/denominator
root factors exactly into one or two native iso-lines, including crossing
relations consumable by multi-curve face partitioning.
Degree-one `2×2` NURBS tensors use this same homogeneous carrier proof. Their
unit-domain solution pcurves are transformed exactly into the authored active
knot rectangle before publication; topology agreement normalizes those native
coordinates back to the homogeneous carrier and rejects any control forgery.

For a tensor iso-section, the two controls on the linear axis must have exactly
equal weights and one exact common control translation; every profile control
must have the same plane value. For a translation tensor linear in either
parameter axis, a transverse plane instead projects the native opposite-axis
profile along that translation and retains the surface pcurve as the exact
rational graph `(u(v), v)` or `(u, v(u))`. Positive weights and the
Bernstein/B-spline convex-hull property certify either that every graph control
lies in the bounded translation-axis domain or that the complete carrier is
outside it. Mixed graph hulls are clipped exactly against the tensor rectangle:
represented roots retain one or more exact fragments, while algebraic roots
that cannot enter `Real` remain explicit. Remaining unsupported work includes
oblique cone sections, non-axial torus quartics, inverse-pcurve partition and
stitching on partially clipped tensor operands, and the rest of the
analytic/spline pair matrix.

## Booleans

The [axial-shell evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-17-axial-shells/README.md) completes the three original
partial-depth cylinder cuts and covering reversed difference. Eight rationally
rotated/reflected workflows verify independent measures, incidence, thin-layer
and event classification, cold replay, repeated cuts, reconstruction, transverse
cuts, empty recovery and STEP reimport. Scale and +/-100-turn closed periodic
seams have additional continuation controls. All 24 original prism cases now
return native results; one diagonal result still rejects an unencodable STEP
vertex. These counts do not certify arbitrary combinations.

Admission requires whole-band lateral coverage, unit section winding, exact cap
transitions, shared contact topology and one connected cycle per vertex link.
A retained frame/height/region certificate owns volume and point classification.
Separate void shells and general non-axial curved embedding remain outside this
proof. Native line/circle extrusion sides publish planes/cylinders, deliberately
changing native/STEP bytes; exact finite extrusion promotion preserves supported
prism shears. Generic rational profiles retain their extrusion carrier.
Coincident-cylinder ownership retains the source UV and exact chart transition;
conflicting material sides return `CoincidentOwnershipAmbiguous`.

Public `union`, `intersection`, `difference` and `symmetric_difference` consume
two `&BodySet` operands and return `BodySet`. Empty identities retain the
surviving selection; complete subtraction produces canonical empty. Each
selection denotes the regularized union of its bodies, including overlapping
inputs. Intersection distributes over both selections, subtraction applies
every cutter successively, and native union normalization merges overlapping
results. Only selected solid subgraphs are copied when collecting components;
source curves, parameters, certificates and voids remain attached.

The [composition record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-body-set/admission.md) covers the
native domain and its tests. The earlier
[compiler reproduction](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-upstream-integration/empty-operand.log)
is historical. General mixed-surface closure, shared expression arrangements,
durable provenance receipts and complete work/memory/cancellation bounds remain
open. An unsupported constituent operation fails the complete Boolean; no
partial selection is published as success. XOR currently composes two directed
differences and their union, with the same intermediate support limits.

`boolean::{union, intersection, difference}` has a native line/arc global-z
prism route. Its profile Boolean uses the overlapping z slab for intersection,
identical slabs for union, and a covering cutter slab for directed difference.
Other routes retain their separate domains. Certified right-prism outputs with
parallel axes can continue in a common orthonormal chart after transforms or
replay; skew/nonparallel prisms are not admitted by that shortcut.

Cold global-z prism queries also consume admitted complete cap-path certificates
when native line/arc projection is unsupported. After the earlier Boolean routes
fail with incompatible slabs, a common-profile rule proves empty symmetric
difference of the complete filled profiles, including holes. Union then merges
touching/overlapping finite z intervals or retains two disjoint solids; difference
can leave zero, one or two finite products. Exact 1/10^12 residual layers remain
distinct. This completes the retained mixed-profile cut/restoration workflow;
matching outer contours or bounds cannot substitute for profile equality.
For different profiles, an empty exact material intersection also proves that
subtraction is unchanged. Union may retain both original solids only when each
profile has one material component and the exact planar union has two; shared
faces that connect material require stitching. General unequal-profile stepped
shells and arbitrary oblique products remain outside this interval rule.
See the [preceding proof and continuation review](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-native-boundary-images/review.md)
and the [selected-profile product proofs](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-profile-extrusion/proofs.md).

Parallel and antiparallel analytic cylinders are reduced through a certified
orthonormal cylinder-local frame, so the same exact line/arc region kernel is
available at arbitrary model orientation. Coaxial equal-radius cylinders also
regularize axial intervals directly: intersection, spanning union, retained
difference, two-component interior cuts, separated unions, and cap contact are
all exact.

Coincident truncated cones with the same apex, axis, and semi-angle regularize
their exact slant-parameter intervals at arbitrary orientation. The result may
be empty, one spanning/retained frustum, two disconnected frustums after an
interior cut, or a separated multi-solid union.

An axis-containing planar cutter intersects a cone carrier in two explicit
lower-bounded generator rays. Exact ray pcurves partition a bounded frustum and
the cutter without inverse fitting. Standard intersection and complementary
frustum-minus-cutter difference publish either longitudinal half-frustum with
three conical cells, two half-caps, and one axial planar face. A closed region
certificate supplies exact half-volume, conical-plus-planar area, halfspace
classification, rigid/reflection transforms, untrusted replay, and exclusion
from the whole-frustum optimization profile.

Geometrically identical ring tori support union, intersection, and difference
even when their periodic frames use opposite axis directions.

Coaxial full-turn revolutions reduce exactly to their radial/axial Hypercurve
regions after proving a common axis line. Native polygonal carriers keep their
existing path; native arcs/NURBS, analytic parallels and Boolean-selected
profiles retain their source curves and parameter identities. Union,
intersection, difference and XOR reconstruct and independently admit connected,
disconnected, holed and empty results. Opposite axis directions, shifted axial
origins and common similarities/reflections are supported. Profiles must have
strictly positive radius, and the owning curve kernel must certify the region
operation. This does not admit partial turns, singular axis contacts or general
noncoaxial surface intersections. The
[coaxial qualification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-coaxial-procedural/README.md)
records exact set oracles, repeated composition, native replay and the remaining
resource and delivery gates.

A bounded authored-axis slab can also clip one polygonal revolution through
the retained graph. Each transverse plane/profile contact is clipped across
the four periodic revolution patches with both pcurves intact. Revolution
descendants certify complete quarter-angle by represented-profile parameter
grids, and each selected planar annulus certifies two complete concentric
circle boundaries plus its effective axial normal. Standard intersection in
either operand order therefore publishes the exact retained profile band with
native area, first-moment volume, radial/profile classification, and
byte-identical untrusted replay.

Outputs may be empty, connected, disconnected, or holed. Native arcs are
retained for lenses, annuli, and later Boolean operands. Hypercurve owns planar
regularization and material/hole roles; HyperBREP owns exact extrusion and
solid validation.

For any current single-solid analytic family, a strictly separated certified
model AABB proves the non-contact Boolean directly: intersection is empty,
difference retains the first model, and union appends both complete topology
closures into one newly validated model. Touching boxes do not take this path.

Sphere/sphere Booleans additionally support equality, strict containment, and
strict partial overlap. Contained difference authors an exact inward
complete-sphere void shell. Partial overlap authors two periodic spherical-cap
faces stitched across four identity-shared intersection-circle edges, with
exact cap area, lens volume, classification, transform, and persistence
certificates. At external tangency, regularized intersection is empty and
difference retains the first sphere; point-contact union remains unsupported.
At internal tangency, union/intersection select the outer/inner operand and
inner-minus-outer is empty; outer-minus-inner remains unsupported because its
touching cavity boundary is non-manifold.

An authored-axis transverse plane/sphere circle retains a planar angular-sweep
pcurve and the exact constant-latitude sphere pcurve. The first such cut of a
boundaryless whole sphere authors two complementary periodic cap faces only
when the trim requires them. Sphere/halfspace intersection then selects one
cap and one planar disk and certifies the exact axial segment for analytic
volume, point classification, rigid/reflection transforms, operand reversal,
and persistence. A second latitude turns only the containing cap into a
periodic two-loop spherical-band face; sphere/slab intersection selects that
face with two planar disks and carries the same exact query, symmetry,
transform, and persistence guarantees without a longitude seam.

An authored-frame-aligned coaxial sphere/cylinder intersection similarly
selects two periodic spherical caps and the central four-patch cylindrical
band. Certification proves complete cylinder parameter-cell coverage, a common
axis and center line, opposite sphere latitudes, and exact agreement of both
circle radii and heights. The standard API then exposes exact face area,
`4*pi*(R³-h³)/3` volume, radial-and-spherical point classification, operand
reversal, rigid/reflection transforms, and persistence.
The complementary sphere-minus-cylinder result selects the single periodic
two-loop spherical band and reverses the same cylinder cells into an inward
band. The closed genus-one shell retains exact total face area and
`4*pi*h³/3` volume, complementary radial-and-spherical classification,
rigid/reflection transforms, and persistence. Here
`h = sqrt(R²-cylinder_radius²)`.
The opposite cylinder-minus-sphere difference is also exact when the finite
cylinder caps lie strictly beyond both sphere poles. It returns two
disconnected native components, each certified from a forward cylinder band,
one planar cap, and one reversed spherical cap. Exact volume subtracts the
spherical cap integral from the cylinder interval; point classification,
face area, rigid/reflection transforms, and persistence retain that spherical
exclusion. Mixed results never advertise the plain-cylinder optimization
profile. For a radius-2 cylinder on `[-4,4]` minus a concentric radius-3
sphere, total area is `(76-20*sqrt(5))*pi` and total volume is
`(20*sqrt(5)-12)*pi/3`.
The regularized union of that same finite cylinder and sphere is one connected
native shell: the central two-loop spherical band, complete lower and upper
cylinder grids, and both planar caps. Its exact query certificate uses a
closed sphere-region variant for finite-cylinder union, suppresses carrier
boundaries that lie strictly inside the other operand, and is excluded from
plain sphere/cylinder/prism optimization profiles. For the radius-3 sphere and
radius-2 cylinder on `[-4,4]`, exact boundary area is
`(40+4*sqrt(5))*pi` and exact volume is `(96+20*sqrt(5))*pi/3`, with
operand reversal, rigid/reflection transforms, and persistence.
Strict sphere/finite-cylinder containment is also regularized without
fabricating a carrier intersection. Union retains the outer operand,
intersection retains the inner operand, inner-minus-outer is empty, and
outer-minus-inner retains one native reversed cross-family void shell.
Containment is certified from exact radial and axial clearances, not sampled
points. A radius-3 sphere minus a centered radius-1 cylinder on `[-1,1]` has
area `42*pi` and volume `34*pi`; a radius-2 cylinder on `[-2,2]` minus a
centered radius-1 sphere has area `28*pi` and volume `44*pi/3`. Both mixed
voids retain exact classification, rigid/reflection transforms, persistence,
operand-order symmetry for union/intersection, and no false primitive profile.
These strict predicates are evaluated before carrier intersection and include
the exact cylinder-axis offset, so off-axis contained pairs have the same
operation matrix even though general off-axis sphere/cylinder surface
intersection remains unsupported. Equality does not enter the fast path;
off-axis tangency remains explicit unsupported evidence.
Finite coaxial cylinders that cross exactly one sphere/cylinder latitude also
regularize natively. For a radius-3 sphere and radius-2 cylinder on `[-4,0]`,
intersection volume is `(54-10*sqrt(5))*pi/3`, union volume is
`(102+10*sqrt(5))*pi/3`, sphere-minus-cylinder volume is
`(54+10*sqrt(5))*pi/3`, and cylinder-minus-sphere volume is
`(10*sqrt(5)-6)*pi/3`. The respective exact areas are
`(22-2*sqrt(5))*pi`, `(38+2*sqrt(5))*pi`,
`(22+10*sqrt(5))*pi`, and `(38-10*sqrt(5))*pi`. The mixed results retain
exact classification, operand symmetry, rigid/reflection transforms,
persistence, and primitive-profile isolation.

`boolean::intersection_graph` now builds the common retained face-pair graph
for any validated solids. It computes each certified face bound once, rejects
only strictly separated AABBs, retains exact supported complete-carrier
intersections and coincidence, counts exact carrier-disjoint survivors, and
keeps unsupported carrier pairs explicit. Transverse plane/plane carrier lines
are additionally projected into both faces' exact pcurve regions, clipped by
Hypercurve, and lifted back into exact `Curve3` fragments. A lack of
positive-length trim interior and an unresolved trim decision have separate
evidence; isolated contact is not conflated with an empty carrier.
When both faces are boundaryless complete carriers, the graph certifies the
entire exact relation directly; sphere/sphere circles and tangent points take
this route without inventing seam topology.
Exact isolated carrier points are also classified against planar face trims
when the opposite face is planar or boundaryless. A tangent point outside the
trim is retained as certified `NoContact`, not silently promoted to a BREP
intersection.
The point inverse extends across plane, cylinder, sphere, cone, and torus
parameterizations and replays every candidate through exact surface
evaluation before it may affect trim topology. Full circle and ellipse
carriers against planar/boundaryless face pairs are decomposed into exact
rational quadratics, clipped through Hypercurve's complete material/hole
region, and lifted as exact spatial rational Bézier fragments.
Graph instrumentation retains candidate, broad-phase rejection, exact
carrier-disjoint, exact-intersection, unsupported, clipped-fragment, and
unresolved-trim counts without requiring callers to reconstruct them.
The graph delegates each surviving candidate to `boolean::intersect_faces`;
callers working with open shells or individual patches can invoke that same
exact operation without constructing a placeholder solid.

Exact rational Bézier and NURBS tensor iso-curves can split their owning
trimmed face through `FaceRef::split_by_surface_curve`; the validator
certifies an interior homogeneous iso-curve or a boundary subrange rather than
trusting endpoint agreement. A complete rational-Bézier or NURBS graph section
on a translation tensor linear in either axis has the same topology path.
Weighted bilinear rational-Bézier sections extend it by recomposing the
complete rational-quadratic pcurve into the expected weighted rational-quartic
spatial control net. Exact pole-separated branches use the same construction
proof on their bounded graph interval; factorized and sign-definite boundary
relations retain native iso-curves. Validation reconstructs every spatial and pcurve
homogeneous control, retains the NURBS degree, knots, and native parameter
domain, proves
curved-loop pairwise simplicity, derives orientation from its monotone graph
and the selected translation-axis domain side, and persists the exact graph
pcurve through untrusted format version 5 replay. Multi-span graphs retain one
exact degree-elevated NURBS pcurve assembled from certified rational knot spans.
Partially trimmed graph sections with represented roots use the same path,
including same-boundary two-edge descendant faces. All fragments can be
applied in one call through `FaceRef::split_by_surface_curves`; an explicit
`SurfaceIntersectionOperand` selects the retained pcurve, exact unordered
endpoint sorting removes caller-order dependence, and each fragment must
belong to exactly one current descendant. Certified intersections between
materialized operand pcurves atomize represented transverse crossings and
reuse one shared crossing vertex; positive-length overlap and unresolved
algebraic contact are explicit errors. A wholly interior closed curve authors
two canonical shared edge halves, one inner wire on the surrounding material
face, and one outer wire on the enclosed descendant. Nested loops, mixed
boundary-attached traces, exact hole transfer, and caller order/direction
invariance use the same descendant arrangement. General conic clipping against
other nonplanar bounded faces and general curved regularization remain open.

## Local topology editing

`EdgeRef::split` splits a canonical edge and every incident use at an exact
interior `Parameter`. `ParameterRange` stores selected trim endpoints and
vertices store `GeometricPoint3`. Line/rational Bézier/NURBS/planar-image carriers can
retain nonmaterialized selected cuts with affine pcurve correspondence;
native angular conics still require represented parameters. Line and native circular pcurves are retained,
forward/reversed uses remain ordered, typed IDs for the first half are stable,
and the result is fully revalidated. The active prism, cylinder, frustum, and
torus solid certificates accept exact edge subdivisions rather than depending
on primitive edge counts. Angular correspondences split circular pcurves
directly in retained root sweep space; nested edits do not round-trip through
rational-Bézier parameters or accumulate equivalent expression trees.

Selected parallel-cap splits retain the complete planar source and parameter
mapping, replay without scalar endpoint reconstruction, and continue through
supported same-profile slab cuts and reunion. The [selected-image admission](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-planar-images/README.md)
records the finite domain and its independent set checks. The
[selected procedural-profile extension](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-procedural-profiles/README.md)
also constructs finite planar-image carriers with selected chart endpoints and
extrudes normalized selected procedural regions. It retains correlated UV
corners and coordinate correspondences on extrusion sides. Straight cross-cap
chains admit authored axial splits alongside selected profile splits, cold
replay, affine reflection/shear, holes/components and repeated supported slab
operations. General selected surface fields and arbitrary curved Boolean
closure remain open; exact native persistence does not establish STEP encoding.

Selected source restrictions preserve their parameter charts through repeated
splits and affine transforms, including reflection. Validation checks actual
affine endpoint mappings and retains the source's complete image/wire proofs
under exact subdivision. Arbitrary edits invalidate those proofs. Cuboid and
cylinder selected splits retain classification and volume certificates;
the [qualification cases](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-selected-topology/README.md)
also exercise rational and NURBS profile sheets. Native archive regressions
now replay those selected sheet boundaries, reflected rational planar trims,
and further splits without scalar reconstruction. Independent selected solid
replay additionally covers every cuboid edge alone or together, shear/reflection,
and one/all cylinder axial cuts. Replayed solids classify points, return exact
volume and can split and reload again. This uses reconstructed geometric proofs,
not archived solid flags; see the [evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-selected-solids/README.md).
General selected surface/contact validation remains unsupported.

Planar rational Bezier, rational quadratic and single-/multi-span NURBS
boundaries also replay native cuts followed by two selected cuts and reflection.
Complete image agreement is checked on the mapped trim interval; NURBS pcurves
may use a different affine knot chart, including negative spatial direction.
Multi-span reassembly requires a complete Hypercurve injectivity certificate,
not merely individually injective spans. Five successive selected cuboid cuts
retain one root through reload and exact solid queries. See the
[composition qualification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-spline-replay/README.md).

`FaceRef::split` splits a trimmed planar face along an exact line chord
between nonadjacent outer-boundary vertices. It authors one identity-shared
edge with opposite uses, retains the first face and outer-wire IDs, reassigns
holes by certified contour classification, updates the owning shell, and
revalidates globally. Cap-region certificates accept the resulting internal
face boundaries for line/arc prisms and native cylinders.

`FaceRef::split_by_curve` is the intersection-driven planar entry point.
For an exact straight `Curve3` fragment, each endpoint either reuses a
mathematically equal outer-boundary vertex or locates its unique represented
parameter on a canonical boundary edge and invokes `split_edge`. It then uses
the same identity-stitched face split and complete revalidation path. Whole
closed surfaces, nonplanar carriers, curved traces, and inner-wire endpoints
remain explicit unsupported cases.

`FaceRef::split_by_curves` exact-orders straight traces by canonicalized
endpoints and rejects duplicates and positive-length overlaps. Every later
trace is split at its exact finite intersections with earlier traces. Attaching
those arranged segments splits already-shared internal edges, so all incident
faces reuse one topological intersection vertex. `FacePartition` records final
descendants and one `FaceTracePartitionRef` per canonical source trace, including
its exact `Curve3` segments and corresponding local splits. Results are
byte-identical across caller order and curve direction, including several
traces concurrent at one point.

`SolidIntersectionGraph::{partition_first_faces, partition_second_faces}`
groups retained straight and surface-curve fragments by stable source
`FaceId`, combines planar supports with exact retained pcurves, and drives the
validated mixed arrangement path for either operand. Plane/extrusion line
trims retain exact pcurves on both operands. A known carrier relation without a
transferable face-local curve returns `FacePartitionUnsupported`; unresolved
trim evidence remains unresolved rather than being skipped. The narrower
`partition_{first,second}_planar_faces` methods remain available only as
explicit line-only operations.

Perpendicular plane/cylinder cuts use the same retained path rather than the
older planar-conic-only trim. The spatial latitude circle is authoritative;
the plane pcurve retains native angular-sweep correspondence and the cylinder
pcurve is the exact line `(u, constant-v)`. Quarter traces from the four native
cylinder patches coalesce into one closed planar loop, while side descendants
are certified by exact rectangular parameter-grid coverage. Axial slab
intersection is therefore supported through graph partition, all-face
selection, native curved stitching, rigid orientation, reflection, operand
reversal, and persistence.

Transverse plane/cone cuts retain the corresponding conical slant parameter
and use the same planar angular-sweep pcurve. Exact rectangular parameter-grid
coverage recertifies subdivided frustum sides, enabling native frustum/slab
intersection through both operand orders, rigid orientation, reflection, and
persistence. The z-prism specialization now requires certified translation
geometry before using a two-layer cap profile; cone frustums cannot enter that
constant-profile kernel.

Transverse plane/torus cuts retain each concentric latitude as its own exact
support, with a planar angular-sweep pcurve and constant torus-`v` pcurve.
Multiple patch-local chains coalesce independently before nested planar-loop
partitioning. Full-torus certification accepts exact latitude subdivisions by
proving complete periodic parameter-grid coverage. Axial torus-band
certification additionally proves exact latitude-cell coverage against one
authoritative `Real` interval and validates zero, one, or two native annular
cap groups. Standard torus/slab intersection therefore stitches exact central
bands and one-cap bands that close at a natural torus extremum, with analytic
volume, point classification, rigid/reflection transforms, and persistence.

Transverse plane/revolution cuts use the same authoritative spatial-circle
transfer without assuming an analytic radius law. The revolution pcurve
retains the exact profile root, side descendants tile only their represented
profile interval, and stitched planar annuli reconstruct the radial cap
segments in the certified Hypercurve profile. Normalized finite line edges are
accepted through an exact affine edge-domain/profile-domain proof, so
selection and stitching do not require the source meridian carrier to be
copied wholesale.

Closed rational-Bézier curve sweeps and translated prism shells retain their
exact solid certificate after
cap-edge subdivision, coplanar cap-face partition, and rational-tensor
side-face partition. Certification groups current coplanar caps, cancels their
internal edges, groups lateral descendants only across identity-shared
nonstandard partition edges on equivalent supports, and certifies the external
translated-shell boundary without confusing periodic seam patches. Tensor
sweeps additionally prove each side group's pcurve boundary tiles one common
exact tensor rectangle. Subdivided connector chains must tile both the
complete retained spatial subcurve and the same active tensor path interval;
homogeneous side-support restriction then reconstructs the common translated
subpath/profile directly from the resulting BREP. Exact transverse
plane-clipped sweep intersections therefore survive selection, stitching,
reflection, operand reversal, and persistence.

`SolidIntersectionGraph::{select_first_faces, select_second_faces}` retains
both immutable operand snapshots, applies all transferable partitions,
constructs certified parameter-interior witnesses from each descendant's
native pcurve region, and classifies those witnesses against the opposite
solid. Candidate generation uses exact boundary controls and dyadic points in
exact pcurve bounds; only `CurvePath2` classification proves that a candidate
is interior. Boundaryless faces use a canonical exact point from the surface
domain. `BooleanOperation` maps the resulting inside/outside evidence to
`Keep`, `KeepReversed`, or `Discard`, and `FaceSelection` accounts for every
face rather than carrying a skipped-nonplane list. Coincident planar boundaries
are overpartitioned by exact straight support arrangements and resolved from
the two oriented material sides. Cone/torus faces with exactly equal parameterizations and affine iso-boundaries
also retain finite boundary traces through native partitioning. Coincident cylinders additionally retain complete angular/axial chart transitions
and authored-periodic trim ownership. Other curved coincidence remains a typed
unsupported boundary; the completed workflow domain
is specified below.

`SolidIntersectionGraph::stitch_selected_faces` transfers the selected
faces into one new arena. It preserves source-local edge identity for every
curve family, atomizes selected edges at every represented exact opposite
endpoint, and matches cross-operand edges only when endpoints and the complete
restricted `Curve3` representation agree exactly (up to certified reversal).
It remaps differing exact edge domains, reverses pcurves for difference, finds
connected components, and publishes only fully revalidated shells.

Whole-sphere faces and intact analytic/tensor shells now transfer directly;
disjoint torus unions exercise native circular edges without a planar
fallback. Transfer regularization also proves when a rational Bézier is an
affinely parameterized line and when a complete rational-Bézier/NURBS tensor
control net is planar. Those carriers are replaced by canonical lines and
planes, with exact projected face pcurves and rebuilt affine parameter
correspondence. A curved fixed-frame sweep clipped by transverse world planes
therefore regularizes to and publishes its exact planar cuboid result through
the same selected-face API.

Planar prism-family results and arbitrary closed straight-planar outer shells
remain end-to-end supported, including strictly contained inward components
republished as exact planar void shells. Convex polyhedra use an
oriented-half-space fast certificate. General concave shells use exact
pairwise coincident-region, boundary-segment, and transverse-line material
checks; contacts without the corresponding shared topological edge or vertex
produce `SelfIntersectingSolidShell`. Outer/void and void/void pairs
additionally require exact non-contact, strict containment, and non-nesting.
Selected curved shells outside the retained analytic/sweep/loft certificates,
or outside an exact canonical reduction to one of them, remain the general
curved-shell self-intersection boundary.

## Derived adapters

With `--features tessellation`,
`tessellation::triangulate_planar_face` exactly triangulates validated
line-bounded planar faces, including holes, through HyperTRI. It returns copied
exact parameter/model-space vertices and oriented triangle indices.

`tessellation::approximate_face_chordally` is a separate, visibly lossy API
for any validated face with an explicit finite outer boundary and for
seam-free complete spherical faces. Its explicit
`ChordalApproximationPolicy` uses only integer boundary segments and midpoint
refinement levels; all generated parameters and surface evaluations use
`hyperreal::Real`. `ChordalFaceApproximation` retains parameters beside points
and certifies `ExactAtVerticesOnly`. Chord interiors have no claimed geometric
error bound. Curved pcurve trims and analytic surfaces are supported without
demoting their authoritative carriers. A whole sphere receives only an
artifact-local longitude seam and shared pole vertices; its authoritative
BREP remains seam-free. Other whole periodic faces without explicit finite
trimming, and surface-error-tolerance requests, remain rejected.

## Seam-aligned analytic cut workflows

Strict intersection (both operand orders) and source-minus-cutter complete for
the original radius-5/minor-radius-2 torus axial and longitudinal halves and
radii-5/2, height-6 cone-frustum longitudinal half, using the original 40-unit
cutters. The matrix includes scales 1/1000, 127/5000 and sqrt(2), rotations,
translations and reflections, then shared-edge splitting, reflected rescaling,
exact measurements/classification and independent archive validation.
`FaceTracePartitionRef::boundary()` retains complete line or circular pcurve
boundary coverage instead of producing a zero-area face. Periodic iso traces and full
circular image matching preserve the authored charts and traversal.

See the [native contract](ARCHITECTURE.md#seam-aligned-analytic-cuts) and
[admission evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-seam-cuts/README.md). The [repeated-composition increment](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-curved-coplanar/README.md)
adds recut, canonical-empty subtraction/recovery, and complementary-half union
for these three fixtures, in both applicable orders and six exact similarities.
Native circular overlap receipts retain actual public parameters and source
revisions. Coincident cone/torus charts require exact full parameterization
equality, including when shell faces reference different equal carrier records.
Selected quarter/band workflows are extended below. Nonlinear periodic gluing,
general torus and cone STEP exchange, full provenance/resource admission and work-package
gates remain open.

## Convex analytic sectors and axial bands

The [sector increment](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-sector-cuts/README.md) completes five
original quarter/band cases and thirty exact similarity configurations. The
corpus checks 175 nonempty Boolean continuation outputs and 35 canonical empty
results, including both applicable operand orders, complementary reconstruction,
exact independent volume/area and point tables, opposite paired edge uses,
untrusted replay, subdivision and reflected rescaling. It retains the original
40-unit cutters and includes 1e-12 classification offsets.

Native annular line partitions preserve input trace identities and model
revisions; a bridge that crosses a hole is rejected. Mixed straight/circular
traces coalesce only with complete connected-interval evidence. Clipping retains
the native spatial/UV correspondence, including nonlinear angular charts and
distinct endpoint identities at a periodic seam.

The admitted cone/torus shell proof requires a complete multiplicity-one native
UV grid inside at most two axis-containing planes and two axial bounds. It
proves the entire angular and axial occupancy, with planar completion justified
by that restricted arrangement. See [the architecture](ARCHITECTURE.md#analytic-sector-and-band-continuation).
This finite corpus does not establish arbitrary-angle or general Boolean
closure. The historical increment retained typed `UnsupportedSurface` controls.
The ring-torus exchange extension below admits its exactly representable torus
cases; cones retain typed unsupported diagnostics and same-context supported-
cylinder recovery. Broader release and resource gates remain open.

## Oblique torus bands and distant conic charts

The [periodic-parameter increment](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-oblique-cuts/README.md)
checks circle/ellipse inverse queries and plane contacts on seven charts through
+/-10^12 turns, including reversal, restriction, both complete-period seam
identities and an exact 1e-12 seam gap. Boolean atomization lifts retained native
preimages into the target angular interval; clipping reuses contacts only on a
certified equal complete parameterization.

A 45-degree relative half cut of the original torus continues into an axial
band, subtraction and complementary reconstruction in both orders. Identity,
reflected 1/1000 scale and reflected sqrt(2) scale exercise 18 nonempty outputs
and three canonical empty results, with exact independent measures/classification,
paired incidence, subdivision, transformed replay and full archive checks.
Four torus STEP attempts now reach exact scalar-encoding checks and return
`NonRational` with body/entity diagnostics; supported same-context cylinder export
recovers. Those irrational boundary values are not rounded for exchange.

The historical nine-fixture diagnostic retained cylinder, cone and torus
continuation blockers. The current 81-slot rerun has 81 native results; see the
[reviewed frontier](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-17-cone-continuation/continuation-frontier.json).
The three formerly undecided cone workflows now have independent full-size
analytic and repeated-composition acceptance.
Both formerly exhausted rationally rotated torus union orders now complete
and have independent repeated native composition tests.
Its successful body counts are not independent set proofs. The two newly
completed rational-sector slots have separate full-size exact composition probes.
General oblique closure, further torus/cone STEP domains, complete provenance/resource admission
and release gates remain open.

## Ring-torus STEP continuation

`Surface::toroidal_geometry()` exposes a borrowed exact frame and radii.
Selected advanced exchange admits ring tori with circular latitude/meridian
boundaries whose geometry has the existing exact finite-decimal encoding.
Both UV axes use angular units and preserve distinct 0/tau seams. Supplied
pcurves retain independent source scales/offsets and require native whole-image
admission. Positive horn/spindle tori and loops needing additional periodic
subdivision return explicit unsupported outcomes. General toroidal trace
families remain outside the declared exchange domain.

The [acceptance record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-16-torus-exchange/README.md) covers whole
tori, reflected/rational frames, metre/millimetre units, half/quarter recuts,
empty continuation, complementary reconstruction and an R=10, r=5 rational
axial band. Four native subdivision/archive cycles retain independent exact
measures before STEP exchange. Exact near-coincidence, malformed source maps,
both seams and exhausted import limits have rejection/recovery controls.
The necessary shared scalar inverse-angle certificate and retained native
contact/image facts have separate baseline regressions; neither changes policy
nor promotes approximate data. This domain does not establish arbitrary torus
Booleans, general STEP conformance or release/resource admission.


## Rationally rotated torus reconstruction

The original R=5,r=2 torus and 40-unit cutter rotated by exact (3/5,4/5)
produce complementary halves that reconstruct in both union orders, including
from separately reloaded operands. Two further edge-split/reload/recut/
remainder/reunion/empty-recovery cycles retain independent exact volume/area,
paired incidence and fifteen point expectations with 1/10^12 boundary offsets.
Cold replay occurs before live-object semantic queries.

Shared Hyperreal certified rounding and bounded same-function congruence
repair the reproduced chart-boundary and endpoint-incidence failures. BREP's
strict validation remains authoritative. The [admission record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-17-curved-continuation/README.md)
retains every original failure, unchanged existing receipt, repeated-growth
observation and adverse cost. This finite domain does not establish arbitrary
torus Boolean closure or whole-operation resource bounds. The later cone domain
is documented separately below.


## Rationally rotated cone continuation

The radius-5 to radius-2, height-6 cone frustum and a 40-unit cutter rotated by
exact (3/5,4/5) complete the axial band and complementary reconstruction in
both operand orders. Each workflow runs two edge-split/archive-reload/recut/
reunion/empty-recovery cycles. Exact independent volume and area, sixteen point
expectations with 1/10^12 offsets, and paired edge incidence remain required on
live and cold-replayed states.

The [admission record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-17-cone-continuation/README.md) binds
Hyperreal's shared exact relations and Hypercurve's retained circle/overlap
witnesses to this finite domain. The original partition, native incidence
capture, overlap trim and complete workflows are separate regressions. No
topology epsilon, display chord, scalar formula in BREP or serialized proof
privilege is introduced. General cone Boolean closure and standalone expanded
coordinate queries remain open. Conical LINE-revolution STEP support is
qualified separately below; native continuation alone does not establish exchange support.

## Exact conical LINE exchange

`ConicalRevolution::from_line` retains and recognizes the exact source generator
and unit axis. `ConeBranch` is relative to the apex parameter, independently of
the sign of authored V. The chart maps source angle/line parameter to native
angle/slant distance, exposes orientation, and proves point inversion away
from the apex. Skew, parallel, perpendicular, off-carrier, singular and undecided
inputs retain typed outcomes. `Surface::conical_geometry` borrows the native
frame and semi-angle.

HyperBRep IO reads and writes the selected `SURFACE_OF_REVOLUTION`/`LINE`/
`AXIS1_PLACEMENT` subset, including supplied line pcurves and exact native-to-source
face maps. Native recognition owns the proof; IO owns units, source IDs and
encoding. A finite cap anchor can encode the original height-7 cone whose apex
Z=35/3 is not a finite decimal. Neither angles nor split vertices are rounded.
One periodic conical side face and independently lifted rectangular partitions
pass native admission only with exact whole-edge images, closed orientation,
cap agreement and complete single coverage.

The [admission record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-17-cone-exchange/README.md) covers the
original full-size 5-to-2 height-6 frustum, the exact (3/5,4/5)-rotated 40-unit
cutter, halves, band/remainder, both reunion orders, recuts, empty recovery and
cold archives. Further groups cover independent manual LINE inputs, source
direction/parameter reversals, declared units, supplied pcurves, similarities,
reflections and typed failures. The [public example](../../hyperbrep-io/examples/cone_step.rs)
performs ten complete exchanges and subsequent native operations.

Direct `CONICAL_SURFACE`, B-spline generators, arbitrary cone trims, apex-ending
solids and broad revolution exchange remain open. Irrational subdivision
vertices can be valid natively while unencodable in this exact STEP subset.
Total-work/heap/cancellation admission, general Boolean closure and actual
package/binding release remain separate gates.

Full procedural revolutions require a simple profile with strictly positive
radial bounds, a common meridian plane, and exact similarity projection into its
radial/axial chart. Ordinary cold admission proves rectangular multiplicity-one
coverage of each complete source domain through one turn and reconstructs the
closed profile. Selected point comparisons also establish cavity nesting. This
is a product-shell proof; arbitrary curved-shell admission, general revolution
Booleans and procedural STEP encoding remain open. See the
[finite qualification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-procedural-revolution/README.md).

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
