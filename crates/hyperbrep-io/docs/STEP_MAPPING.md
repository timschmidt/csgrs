# Typed STEP mapping

This is the normative contract for the first geometric `hyperbrep-io::step`
boundary. It implements part of WP14a in the
[development plan](../../hyperbrep/docs/DEVELOPMENT_PLAN.md). Exactness,
completeness and performance retain that priority order under
[properties.md](../../../properties.md). New mappings follow the shared
[admission protocol](../../../docs/admission.md).

## Public boundaries and ownership

| API | Result and authority |
| --- | --- |
| `part21::parse(bytes, limits)` | Borrowed, source-scoped syntax; no schema or geometric authority. |
| `StepFile::read(bytes, options)` | Syntax plus decoded standard header and checked DATA/schema declarations. |
| `StepFile::context_units(id, limits)` | Exact declared length/angle unit graph and unapplied uncertainty metadata for a 3D context. |
| `StepFile::import_representation(id, limits, context)` | Every supported item in one selected representation, or a typed failure; native solids are published only after full builder validation. |
| `ImportedRepresentation::bodies()` | Owned native `Model` values, in representation item order, with model-bound solid/face/vertex/wire/edge-use views. |
| `interpreted_entities()` / `retained_entities()` | Sorted distinct interpreted IDs and source-order raw-retained IDs; disjoint and together covering every DATA instance. |
| `ImportedBody::edge_uses()` | Bound/loop IDs, typed polygon-segment or oriented-edge source, directed source endpoint identities and native edge-use reference. |
| `ImportedBody::explicit_edges()` | Explicit EDGE_CURVE identity, source geometry/master IDs, exact directed parameter maps for every supplied supported representation and native edge reference. |
| `write_faceted(bodies, options, context)` | All explicitly selected supported native solid occurrences as one AP214 common faceted representation/product, or a typed failure. |
| `FacetedStep::bodies()` | Borrowed model-bound source-to-output mappings for every vertex, face, wire and directed edge use. |

The file borrows its original input; imported bodies own their geometry and remain
usable after the file is dropped. Source references are file-local numeric IDs,
not global identities. Keep the file alongside the result to inspect retained
records and their spans. No native model carrier is duplicated in the IO crate.
All modeled topology goes through HyperBRep's `ModelBuilder`. Native queries,
transforms, edits and exact archives consume its ordinary `Model`.

Import is an explicit selection, not root/product/assembly traversal. A supported
selection can coexist with unknown entity types or unrelated broken
references in raw syntax, but this mapping currently requires all declared
schema names to identify the common AP203/AP214/AP242 subset. Recognized names
are `CONFIG_CONTROL_DESIGN`, `AUTOMOTIVE_DESIGN` and
`AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF`, optionally followed by a
whitespace-separated suffix. Recognizing a schema name is not conformance proof.

## Implemented mapping

The selected simple `FACETED_BREP_SHAPE_REPRESENTATION` contains one or more
simple `FACETED_BREP` items and optional `AXIS2_PLACEMENT_3D` items. Placement
items are checked metadata; they do not transform sibling items. Each body uses
one `CLOSED_SHELL`, `FACE_SURFACE` faces on `PLANE`, exactly one
`FACE_OUTER_BOUND` per face, optional inner `FACE_BOUND` records, `POLY_LOOP`
boundaries, and three-coordinate `CARTESIAN_POINT` records. Simple records have
their required inherited attributes; extra/missing attributes are errors.
Other representation items, complex geometric instances, oriented shells,
void-body entities and mapped instances are explicit unsupported boundaries.

Plane placements normalize the authored axis and the projection of the reference
direction perpendicular to it. Omitted axis defaults to Z; omitted reference
defaults to X, or Y when the axis is parallel to X. Zero and parallel supplied
directions fail. Normalization uses exact native scalar operations, including
radicals; it does not pass through binary floating point. The resulting frame
defines each face's exact local line pcurves.

`FACE_BOUND.orientation` reverses the polygon traversal when false.
`FACE_SURFACE.same_sense` controls the native face orientation independently.
Neither flag is guessed or repaired from a volume sign. All inner bounds are
constructed and validated. Within one body, equal rational coordinate triples
share a vertex and equal unordered vertex pairs share a spatial straight edge;
every source point alias and every directed use remains recorded. No epsilon
is used. Near-equal coordinates remain distinct and may make an authored shell
invalid. Sharing does not cross body boundaries.

Native validation currently admits specific planar-shell domains, including
convex shells and supported prismatic shells with holes; this importer does not
extend that domain. A syntactically valid arbitrary faceted body can therefore
return native `Build`/`Validation` errors or an unsupported-domain report.
Exact authored normal/point inconsistencies are errors even if another CAD
reader tolerates them. A future explicit repair operation must report its changes.

## Explicit planar advanced BREP

`ADVANCED_BREP_SHAPE_REPRESENTATION` selects `MANIFOLD_SOLID_BREP` bodies and
optional placement metadata. Each body has a `CLOSED_SHELL` of `ADVANCED_FACE`
records on `PLANE`. Bounds use `EDGE_LOOP`, `ORIENTED_EDGE`, `EDGE_CURVE`,
`VERTEX_POINT` and three-dimensional `CARTESIAN_POINT`. A spatial edge uses
`LINE` or `SURFACE_CURVE` whose 3D curve and all associated pcurves are lines.
Complex geometry, other curve/surface families, oriented shells, void bodies,
vertex loops and mapped/assembly instances remain typed unsupported boundaries.

Explicit vertex IDs and edge IDs determine topology. Equal coordinates or equal
unordered endpoint pairs never merge distinct explicit entities. Caches are
scoped to one body; repeated bodies sharing source records still own independent
native models. `VertexSource::VertexPoint` retains both vertex and point IDs;
the faceted case remains `VertexSource::CartesianPoint`. Every directed use
records its bound, loop, oriented-edge and edge-curve IDs plus endpoint vertex
IDs after applying bound orientation. No source-ID array index becomes a native
public handle.

A line evaluates as its exact source point plus its normalized direction times
authored vector magnitude and parameter. Length conversion scales both origin
and velocity. Each endpoint must lie exactly on that support. The parameter
order must agree with `EDGE_CURVE.same_sense`. Native edges use their authored
topological start/end with canonical unit parameterization; `SourceCurveParameters`
reports `source_parameter = start + (end - start) * s`, where `s` is the
normalized directed edge fraction (the native parameter for unit-domain lines),
including negative maps. `ORIENTED_EDGE.orientation` and `FACE_BOUND.orientation` act
independently; reversing a bound reverses order and every edge use.

Each associated `PCURVE` has a plane basis and a one-item
`DEFINITIONAL_REPRESENTATION` containing a 2D line. Its context must be parametric;
an optional geometric-context component must declare integer dimension two.
Plane parameter coordinates use the selected length unit. The mapped infinite
line must coincide with the 3D line and have the same direction. Different
positive affine parameterizations are admitted exactly. The declared
`CURVE_3D`, `PCURVE_S1` or `PCURVE_S2` selects the surface curve's parameter map;
all alternate representations are still checked and reported. Advanced-face
surface-curve associations must all be pcurves. Native local pcurves are exact
projections reparameterized to the directed edge interval. Native face validation
independently proves their spatial image on the actual face plane. Source maps
refer to the immutable imported model, not to later edited revisions.

An explicit `FACE_OUTER_BOUND` must have the material orientation. When only
`FACE_BOUND` records exist, signed contour area and face sense identify exactly
one finite material outer loop; all opposite loops become holes. This neither
reverses author intent nor chooses the first bound. Native validation then proves
loop simplicity, hole containment, manifold incidence and the supported solid
domain. General unbounded or disconnected planar face domains remain unsupported.

The API is now `import_representation`; direct callers use the typed source enums.
There is no legacy import alias or parallel geometry carrier. Full EXPRESS/AP
conformance, native arbitrary-shell completeness, global arithmetic/work/heap
admission and general curved/periodic STEP exchange remain unfinished. Independent OCCT
inputs and the exact/rejection/composition evidence are recorded in the
[advanced mapping qualification](../../hyperbrep/docs/evidence/2026-10-15-step-advanced/README.md).

## Direct circular edges and cylindrical faces

Advanced bodies additionally accept direct spatial `CIRCLE` curves and
`CYLINDRICAL_SURFACE` supports through the same explicit topology and native
builder. A circle's placement and positive radius define its exact support;
native point location must certify both endpoints. A full circular edge uses
the same explicit vertex at both endpoints and traverses exactly one period.
Different vertex IDs at equal positions remain distinct and are rejected for
this closed-circle encoding. There is no welding, synthetic seam vertex or
implicit repair. Partial arcs retain both authored vertex IDs.

The source parameter interval starts at the first endpoint's native canonical
angle in `[0, tau]`, with the native locator selecting the first seam parameter.
Its end is unwrapped in the direction of `EDGE_CURVE.same_sense`; a full edge's
directed span is `+tau` or `-tau`. Both endpoints are divided by the context's
exact radians-per-angle-unit scale. Thus the numeric source period need not be
`tau`. Native circles retain their source frame and an increasing radian domain;
native reversal supplies negative traversal. For native parameter `t`, the map
fraction is `s = (t-domain.start)/(domain.end-domain.start)`. The source/master ID
is the direct `CIRCLE` ID. Maps refer to the immutable imported revision.

Plane faces use native circular pcurves and angular-sweep correspondence.
Cylinder faces use continuous local UV charts in radians and metres, with a
separate use for each side of the periodic seam. The first point uses native
circle parameter location; subsequent segments continue the preceding lift only
after explicit vertex continuity is checked. Native whole-image validation
certifies every generated pcurve and its affine edge correspondence. The
currently admitted cylindrical boundaries are constant-height circles and axial
lines, within the existing native face/shell validation domain. This mapping
does not add arbitrary cylinder trimming or general shell validation.

Angular mapping requires a declared plane-angle unit, even when no angular
number appears in the spatial circle record. Length and angle scales remain
independent. A conversion named `degree` uses its authored exact decimal factor;
it never substitutes symbolic `pi/180`. Uncertainty metadata cannot make an
off-carrier decimal vertex incident.

Ellipses, analytic surfaces other than the ring tori described below, and spline exchange remain typed unsupported.
The planar line/supplied-pcurve mapping above is unchanged. Every selected body
must finish admission before any result is
published; all source entities remain interpreted or raw-retained. The exact
faceted writer rejects these curved models. Native archive replay is a separate
lossless operation, not a curved STEP round trip. See the
[circular mapping evidence](../../hyperbrep/docs/evidence/2026-10-15-step-circles/README.md)
and [runnable example](../examples/circular_step.rs).

## Supplied analytic pcurves

Selected `SURFACE_CURVE` edges may have a spatial `LINE` or `CIRCLE`, with one
or two `PCURVE` associations. Every supplied representation must agree in image
and sense, including non-master alternatives. `CURVE_3D`, `PCURVE_S1` and
`PCURVE_S2` choose the source parameter map; a missing nominated pcurve is an
error. The surface-curve map, spatial map and each pcurve map remain visible
through `explicit_edges()`. All maps use the normalized directed edge fraction,
including reversed edges and partial or single-vertex full circles.

Each pcurve has a one-item `DEFINITIONAL_REPRESENTATION` in a two-dimensional
parametric context. Supported basis combinations are:

| Spatial image | Pcurve basis | Surface | Coordinate conversion |
| --- | --- | --- | --- |
| Line | LINE | PLANE | Both coordinates and velocity components use the length scale |
| Circle | CIRCLE with AXIS2_PLACEMENT_2D | PLANE | Center/radius use length; circle parameter uses angle |
| Constant-height circle | LINE | CYLINDRICAL_SURFACE | U uses angle, V uses length |
| Axial line | LINE | CYLINDRICAL_SURFACE | U uses angle, V uses length |
| Torus latitude or meridian circle | LINE | TOROIDAL_SURFACE | Both U and V use angle, independently of length |

The pcurve vector is normalized in its authored parameter coordinates before
the two basis scales are applied. Plane-circle placements normalize their 2D
reference direction and use its positive perpendicular; an omitted direction
defaults to X. Source angular intervals remain unwrapped, preserving traversal
and a distinct endpoint for a full circle. Native chart coordinates are radians
and metres. The source unit's authored conversion factor remains exact even if
its name is `degree`.

For new curved combinations, the mapper stages the candidate pcurve and calls
`ModelBuilder::validate_edge_image`. That native proof checks the whole trim
and correspondence; source endpoint samples cannot replace it. Candidates may
remain as unused native geometry in the immutable model. Face construction
independently validates its generated local charts, followed by shell and model
admission. Source maps refer to the original imported revision. Native archive
replay does not itself persist the external STEP source-accounting object.
The existing planar-line specialization retains its exact infinite-line proof
and deterministic native artifact contract.

`SEAM_CURVE` checks two supplied pcurves on the same surface. For an axial
cylindrical seam their constant native U values must be 0 and exact `tau`, in
either order. Duplicate pcurves at U=0 are not opposite boundaries. A finite
decimal such as `6.283185307179586` is not exact `tau`; this encoding is rejected,
even in an ambient approximate-policy scope. This increment provides those
checks, not a successful finite-decimal exact seam fixture. Toroidal seam checks
apply to either angular axis and require both pcurves to bound that same axis.
Other seam surfaces,
trimmed/spline pcurves and explicit approximation/healing remain open.

Malformed supplied encodings now reach their referenced definitions: missing
definition IDs report `MissingEntity`, rather than the former unsupported-family
guard. Original malformed fixtures remain failures with their exact missing IDs
asserted. The [supplied fixture](../tests/fixtures/supplied-cylinder.step),
[example](../examples/supplied_step.rs) and
[acceptance evidence](../../hyperbrep/docs/evidence/2026-10-15-step-pcurves/README.md)
record exact source-map evaluation, consumption and the remaining limits.

## Units and text

Published coordinates use **metres**. Unit categories remain length, plane angle
and solid angle even though both angle categories have zero SI dimensions.
Supported units combine `NAMED_UNIT` and the category marker with `SI_UNIT` or
`CONVERSION_BASED_UNIT`. SI names are metre, radian and steradian; all twenty
STEP prefixes from yocto through yotta are supported. Conversion-based units
validate seven `DIMENSIONAL_EXPONENTS`, a positive typed measure, matching
category and an acyclic conversion chain. Simple measure subtypes and supported
complex `MEASURE_WITH_UNIT` instances are accepted. Forward references and
shared chains resolve iteratively with a per-operation cache.

The exact authored factor controls conversion: the label “inch” never substitutes
a different value. A decimal degree-to-radian factor stays that rational value;
no symbolic π is inferred from a name. Context length units are mandatory;
plane/solid-angle units are optional until a mapping needs them. Duplicate
categories, missing/unsupported units and dimension mismatches fail explicitly.
Uncertainty records retain exact SI magnitude, name and description; they never
authorize vertex merging, healing, fitting or predicate approximation.

`StepFile::read` checks the initial FILE_DESCRIPTION / FILE_NAME / FILE_SCHEMA
order, types and cardinalities, duplicate standard records, implementation levels
2;1 / 3;1 / 4;1, and section/schema declarations. Extra header records remain
raw. Calendar validity, complete schema identifiers and edition-specific rules
remain unvalidated. Accepted decoded text includes raw UTF-8, doubled apostrophes,
doubled backslashes, default/explicit ISO-8859-1 S/P directives, X octets and
X2/X4 UCS scalar groups. Surrogates, nonhex payloads and out-of-range scalars fail.
Physical ASCII control bytes are ignored inside text and its directives; encoded
control scalars survive. Explicit N/F print-layout directives between string
content units contribute no content. The syntax lexer still requires contiguous
keywords/numbers and doubled-quote escapes; general physical-control splitting
of those tokens and print directives outside strings remain unimplemented.
ISO-8859 alphabets B–I are typed unsupported; their tokens remain in raw syntax.

## Failure and resource contract

`Error` retains an entity ID when available, a byte offset (zero when no closer
site is available), and a typed cause. Missing targets include the first matching
source-reference site. Syntax/numeric admission, missing entities, invalid versus
unsupported constructs, unit cycles, limits, allocation failures, numeric causes,
native construction and native validation remain distinguishable. No error variant
publishes a partial selected representation. Solver exhaustion is the outer
`BudgetOutcome` failure and is not reclassified as geometric invalidity.

Limits are explicit and renewed for each public call:

- `StepReadOptions::syntax`: bounded input, lexemes, nesting and flat syntax storage.
- `header_text_bytes`: cumulative decoded HEADER and DATA-argument bytes.
- `ImportLimits::number`: per-lexeme exact digit/exponent admission.
- `entity_visits`, `scalars`, `text_bytes`, `edge_uses`: cumulative lookup and
  numeric-materialization attempts, emitted text bytes and directed boundary uses.
- `unit_scale_bits`: maximum numerator/denominator bit bound for SI scales and
  conversion products; sums of operand bit lengths are admitted before multiplying.
  This bound is conservative and may reject a product that would cancel.
- `OperationContext`: strict predicate authority and shared instrumented solver work.

These allowances do **not** bound all scalar work, coefficient growth outside
unit products, native validation, aggregate heap use or elapsed time. Cache hits
still charge entity lookup attempts. Dynamic IO collections reserve fallibly;
native/scalar allocation remains governed by its own implementation. A 20,000-link
unit graph exercises iterative traversal and drop on a 64 KiB native stack and
in an actual WASM runtime; this is not an arbitrary-hostile-input safety proof.

## Exact native faceted export

`StepBody` explicitly selects a validated native `SolidRef` and output name.
Native coordinates are interpreted as metres. `StepWriteOptions` supplies a
positive rational metres-per-output-unit factor, unit name, file/product metadata,
caller-authored timestamp and `WriteLimits`. The writer emits `AUTOMOTIVE_DESIGN`
with implementation level `2;1`, a product/definition/shape association and one
`FACETED_BREP_SHAPE_REPRESENTATION`. Repeating the same input solid produces a
distinct body occurrence with its own point definitions; this is not an assembly.

All outer-shell faces must use native `Plane` surfaces and all spatial edges must
use native `Line` carriers. The writer traverses every outer/inner wire in its
native directed order and emits `POLY_LOOP`, `FACE_OUTER_BOUND`/`FACE_BOUND`,
`FACE_SURFACE`, `PLANE`, `CLOSED_SHELL` and `FACETED_BREP`. Point entities are shared
by native vertex identity within each body, without coordinate welding. Every
directed use, including the closing segment, receives a polygon/segment mapping.
The two uses of a shared native edge reference the same endpoint entities in
opposite directions. Faceted STEP encodes these spatial edges implicitly.

The plane origin is the first outer-loop point, the reference direction is its
first directed edge, and the oriented normal is the complete polygon area vector
computed relative to that origin. This handles concave loops and collinear split
vertices without guessing orientation from the first three points. Both directions
are reduced to primitive integer ratios, so exact normalization would recover the
same axes without requiring serialized square roots. Bounds and face same-sense
flags are true because the frame follows the directed outer loop. The immutable
native model supplies planarity, incidence and orientation authority.

This changes parameterization explicitly: `WrittenFace` identifies the native
face and its newly emitted plane; `WrittenWire` identifies the polygon/bound;
`WrittenEdgeUse` identifies the native edge use and directed implicit segment.
Native curve domains, pcurves, authored charts, history and unrelated document
records are not encoded by selected geometry export. Keep native persistence
when those representations must remain unchanged.

Finite STEP decimal reals preserve a reduced rational only when its denominator
contains no prime other than two or five. The writer factors that denominator,
forms an integer coefficient, removes decimal trailing zeros and writes a signed
mantissa/exponent without binary floating point. `NonTerminatingDecimal` and
`NonRational` are explicit refusals; the latter means no retained exact rational
view, not a proof of mathematical irrationality. The declared unit factor must
also be exactly representable. A one-metre coordinate expressed in inches is `5000/127` and is
rejected. Neither an approximate predicate scope nor a display projection can
authorize rounding. Future approximation is a separate operation/report.

Text emits ASCII with doubled apostrophes/backslashes and X/X2/X4 escapes for
controls and Unicode. Every encoded string, including quotes, is bounded by
32,769 bytes. Metadata never obtains a current clock or process identifier.
Calendar validity and full EXPRESS/AP conformance remain unproved.

`WriteError` carries the zero-based selected body occurrence and native entity
kind/position when available; these are diagnostics scoped to that source model.
Global metadata/unit errors have no body. `UnmaterializedPoint`,
`UnsupportedSurface`, `UnsupportedCurve`, `VoidShells`, invalid inputs, numeric
causes, resource limits and allocation errors are distinct. All selected bodies
succeed together; no partial artifact escapes failure or solver-budget exhaustion.

`WriteLimits` bounds total output bytes, all DATA entities, all directed segments,
source/converted rational component bits, decimal significand/exponent sizes and
the integer buffers used by decimal/direction encoding. Denominator factor counts
also charge the exponent limit. Integer products in these encoders check a
conservative bound before multiplication; it may reject a product that would fit
after cancellation. IO collection reservations are fallible. Native unit-conversion
arithmetic, polygon area-vector arithmetic, scalar backend allocations and total
heap/time are **not** bounded by these allowances. The explicit solver scope
retains its existing instrumented-work meaning.

## Exact native analytic export

`write_advanced` accepts selected validated outer-shell solids on `Plane` and
`Cylinder` and ring `Torus` carriers, plus exact linear extrusions of lines and circles swept
normal to their plane, with `Line` and `CircleArc` edges. It emits
`ADVANCED_BREP_SHAPE_REPRESENTATION`, `MANIFOLD_SOLID_BREP`, `CLOSED_SHELL`,
`ADVANCED_FACE`, explicit bounds/loops/oriented uses, `EDGE_CURVE`,
`VERTEX_POINT`, `LINE`/`CIRCLE`, and `PLANE`/`CYLINDRICAL_SURFACE`/`TOROIDAL_SURFACE`.
Topology follows model-bound identity: shared edges stay shared, closed circles
reuse their single vertex, and two seam uses remain separate directed records.
Coincident coordinates and repeated body occurrences do not weld identities.
All inner wires and face orientations survive. Cavity shells and unmapped
carriers fail the whole selection with their source site.

The [advanced-face schema](https://steptools.com/docs/stp_aim/html/t_advanced_face.html)
permits direct line/conic edge geometry. Its supplied-pcurve requirement applies
when an edge uses a `SURFACE_CURVE`. This writer emits direct spatial curves;
it does not serialize a rounded angular period as a seam pcurve. This is a
selected AP214 encoding, not a complete EXPRESS/AP conformance claim.

The returned `WrittenAdvanced*` records identify every native vertex, edge,
directed use, wire, face and selected solid occurrence. Each edge reports the
emitted carrier's parameters at its native trim endpoints; interpolation follows
the native trim fraction. A line starts at the edge's native start point and
uses unit speed in output coordinates. A circle keeps its authored frame,
angular direction and unwrapped endpoint angles. `EDGE_CURVE.same_sense` and
`ORIENTED_EDGE.orientation` encode independent directions. Reimport may choose a
different full-period representative of the same angular phase.

Each face reports an affine native-to-output UV map. A plane retains its authored
origin and orthonormalizes its U/V frame for STEP; the reported map accounts for
skew, scale and output units. A cylinder keeps its exact origin and frame; U is
in radians and V is divided by metres-per-output-unit. The native pcurves remain
queryable through each borrowed source use. They, unused geometry, history and
unselected document entities are not persisted by geometry export; use native
archives for those representations.

An extrusion uses `Surface::extrusion_profile_and_direction` to inspect its
actual `profile(u) + v*direction` chart. Line profiles encode as planes; circular
profiles encode as cylinders only after proving the direction parallel to the
circle normal. The map retains native parameter offsets, reversed angular
traversal and signed axial scaling. `WrittenAdvancedFace::orientation` records
the emitted sense after any chart reversal; it can differ from the source
face's sense while preserving the oriented physical face. Ellipse, spline and
oblique circular extrusions retain `UnsupportedSurface` errors. No sampled
recognition, fitting or decimal approximation is performed.

This adds `WrittenSurfaceParameters::origin` and the emitted face orientation
to the public report without compatibility adapters. Existing plane/cylinder
maps have zero offset. Circle-preserving region extrusion and the containing-
slab Boolean route can intentionally change native topology and STEP bytes;
semantic, parameter and source-accounting checks are separate from deterministic
artifact migration. Native archives retain native representations; this
direct-curve STEP encoding does not serialize native pcurves or operation history.

The scalar layer's exact rational certificate can expose decimal-representable
values without rounding. Direction ratios can remove a common irrational
normalization factor by exact positive projective division; their sign is
retained and the integer encoder applies its existing bit limits. Unsupported
ratios return `NonRational`; strict unresolved decisions remain typed
`WriteErrorKind::Geometry`. As for faceted export, rational-only means the
available certified view, not a proof that another representation is irrational.
UV-map construction, scalar normalization, coefficient growth and total heap,
work and cancellation remain outside the writer's cardinality/encoding bounds.

The common input name is now `StepBody`, replacing `FacetedBody` without an alias.
`write_faceted` remains a distinct encoding with unchanged output contracts.
The [analytic admission record](../../hyperbrep/docs/evidence/2026-10-16-step-analytic-export/README.md)
retains exact graph checks, parameter maps, composition, source-bound checks and
the original native uniform-scaling failure. The subsequent
[native similarity increment](../../hyperbrep/docs/evidence/2026-10-16-analytic-similarity/README.md)
adds exact physical scaling, reflected chart transport, native replay and
unit-aware export/reimport. Physical scaling occurs in HyperBRep; output unit
selection remains an independent exchange operation.

## Ring-torus exchange

`TOROIDAL_SURFACE` uses the exact placement and major/minor radii, with native
admission requiring `major_radius > minor_radius > 0`. Both parameters are
angles. The source formula is
`C + (R + r*cos(v))*(X*cos(u) + Y*sin(u)) + Z*r*sin(v)`.
Length units apply to C, R and r; angle units independently apply to both UV
coordinates and supplied pcurve velocities. A borrowed `TorusGeometry` view
supplies writer data without fitting or a second surface carrier.

The reader constructs latitude/meridian candidates through native circle point
location and certifies the complete edge image with the ordinary model builder.
Connected loop segments retain one continuous periodic lift. A whole loop can
be translated by exactly tau into the native chart; distinct 0/tau coordinates
are preserved. A loop requiring periodic subdivision beyond this mapping is a
typed unsupported input. Endpoint agreement alone never admits a pcurve.

The writer emits direct spatial circles and an identity native-to-output UV map
in radians. It does not encode a finite decimal as tau. Existing exact decimal
and projective direction admission applies to every vertex, radius and frame;
some valid native torus bands therefore remain `NonRational` export failures.
Horn/spindle tori, other torus circle families, general trims and full EXPRESS/AP
validation remain open. The finite-decimal seam controls cover both U and V.

The [torus admission record](../../hyperbrep/docs/evidence/2026-10-16-torus-exchange/README.md)
retains the original unsupported controls and their explicit capability migration,
independent surface/measure checks, source maps, repeated modeling, native replay,
malformed inputs and resource limits. The [example](../examples/torus_step.rs)
demonstrates the public source workflow. These selected workflows do not close
WP14a/b or establish package release readiness.

## Remaining exchange work

WP14a still requires further simple analytic entities, broad independent exporter fixtures, representation
root selection, and more complete schema/resource admission. WP14b retains
spline/trim/seam/void exchange, assembly/mapped representation traversal and
explicit healing/export approximation. Header/text completeness, generic planar
shell validation, coefficient/work bounds, fuzz/ASan/lexicase qualification,
full workspace checks and release work are also open. A source syntax round trip
and native JSON replay do not count as an edited geometry STEP round trip.

The [export qualification](../../hyperbrep/docs/evidence/2026-10-15-step-export/README.md)
checks edited geometry STEP round trips, raw emitted coordinates/edge uses/oriented
normals, representability and admission, executed WASM, original native examples
read independently by OCCT, fault controls and export/import costs. These are
selected-domain observations, not a completed work package or general CAD release.

Four original fixture recipes (box, two bodies, through-hole and oblique
tetrahedron) are read by OCCT and mapped natively with exact volume and incidence
checks. The [qualification](../../hyperbrep/docs/evidence/2026-10-15-step-faceted/README.md)
retains tests, runtime probes, fault controls, source hashes and cost observations.
