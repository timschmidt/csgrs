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
| `ImportedBody::explicit_edges()` | Explicit EDGE_CURVE identity, source geometry/master IDs, exact parameter maps for every representation and native edge reference. |
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
reports `source_parameter = start + (end - start) * native_parameter`, including
negative maps. `ORIENTED_EDGE.orientation` and `FACE_BOUND.orientation` act
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
admission and curved/periodic STEP exchange remain unfinished. Independent OCCT
inputs and the exact/rejection/composition evidence are recorded in the
[advanced mapping qualification](../../hyperbrep/docs/evidence/2026-10-15-step-advanced/README.md).

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

`FacetedBody` explicitly selects a validated native `SolidRef` and output name.
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

## Remaining exchange work

WP14a still requires simple analytic entities, broad independent exporter fixtures, representation
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
