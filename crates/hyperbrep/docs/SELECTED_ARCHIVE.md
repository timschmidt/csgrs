# Selected geometry in archive version 9

Version 9 retains authored coordinates and selected curve images in the same
model. It replaces version 8; there is no migration reader.
`RawModel::from_json(json, &limits)` admits input syntax and record structure
under explicit `ArchiveLimits`, checks the format version before decoding exact
scalar data and rejects trailing input.
`RawModel::validate` is the trust boundary. Parsing alone grants no geometry,
measurement or topology authority.

## Input admission

The caller passes `ArchiveLimits` explicitly. Its defaults and units are:

| Field | Default | Counted resource |
| --- | ---: | --- |
| `input_bytes` | 16 MiB | All UTF-8 bytes, including whitespace and ignored fields |
| `nesting_depth` | 1,024 | Simultaneously open JSON objects and arrays |
| `tokens` | 1,000,000 | Opening containers, quoted strings (including keys), and primitive lexemes |
| `records` | 250,000 | Topology/geometry arena entries, selected parameters, detached point/pcurve sources, inline extrusion/revolution profiles and analytic-parallel sources |
| `degree` | 256 | Each declared NURBS axis degree, inferred Bezier axis degree, or selected-root polynomial storage degree |
| `geometry_elements` | 1,000,000 | Aggregate controls (including polynomial parallel sources), weights, knots, root coefficients, and tensor row headers |
| `topology_references` | 1,000,000 | Aggregate wire edge uses, face inner wires, shell faces and solid void shells |

Byte admission runs first. A constant-storage iterative scan then counts depth
and tokens without allocating or recursing. Closing delimiters, punctuation and
whitespace are not tokens; brackets and escaped quotes inside strings do not
increase structural depth. This scan precedes any version-specific parsing.

Next, a Serde seed carries one shared budget through record metadata. It reads
counts and degrees without constructing exact scalars, geometry or collections
of records. Top-level entries are charged before their payloads are inspected;
inline profiles and variable sizes are charged when the enclosing record's
summary is complete. Detached sources use the same budget as model arenas.
Successful inputs combine version and metadata admission in one pass. On a
metadata failure, a separate header read can still diagnose an unsupported
version regardless of field order. It never migrates an older schema. Duplicate
or absent versions are invalid JSON-model envelopes.

A control point counts once, not once per coordinate. Every tensor row also
counts, including an empty row. Bezier degrees are control counts minus one
(saturating at zero); tensor degrees use row count and maximum row width.
Rational quadratic pcurves have degree two. Root degree is coefficient count
minus one without trimming trailing zeros. Fixed-size analytic coordinates,
chart-image coefficients and fixed record references are bounded by record
counts instead of variable-size counters. Unknown fields consume syntax limits,
not schema counters. Full decoding and geometry validation remain responsible
for malformed dimensions, missing fields, invalid coefficients and references.

Exhaustion returns `PersistenceError::ResourceLimit(ArchiveLimitExceeded)` with
the resource, allowance and observed amount. This is the input length, first
disallowed syntax/record count, declared/inferred degree, or aggregate after a
record summary. Every limit is inclusive. Retrying with larger limits preserves
the original input and exact result; no data is silently omitted to fit a limit.
The one-argument API is removed; callers select their limits directly.

These limits do not bound coefficient reconstruction, geometry work, aggregate
allocations, elapsed time or native stack use. A small scalar expression can
demand substantial work. Metadata parsing and error reporting may allocate;
only the initial syntax scanner has an allocation-free claim. Native
`serde_stacker` remains in the decoder, and configurable depth alone does not
prove a safe WASM call stack. See the [syntax admission evidence](evidence/2026-10-15-archive-admission/README.md)
and [structural admission evidence](evidence/2026-10-15-archive-structure/README.md).

## Records and deterministic ordering

The existing topology arrays retain their order and references. Vertices are
`Coordinates([Real; 3])` or `CurveImage { source, parameter }`. A point source
refers either to the model's spatial-curve arena or to a detached immutable
curve in `selection.point_sources`. Detached sources have no model back-reference.
Edges store two `Scalar(Real)` or `Selected(index)` parameter records. Scalar
coefficients remain Hyperreal values; decimal/floating approximations are not
introduced.

A closed circular or elliptic edge stores the same vertex index at both ends
and two distinct parameter endpoints spanning exactly one period. It uses the
existing version-9 records. Validation reconstructs the carrier's regularity,
period and actual endpoint agreement; it does not trust an archive flag.
Circular cap and periodic cylinder edge uses also replay their complete
pcurve-image, direction and shell certificates. Distinct adjacency, parameter
maps, split/reflection and forged-record rejection are covered in the
[closed-edge evidence](evidence/2026-10-15-closed-edges/README.md).

`selection.parameters` contains two node forms:

- `Root { polynomial, interval }` supplies ascending power-basis coefficients
  and a singleton isolating interval. It does not supply a trusted root count.
- `Image { root, numerator, denominator }` supplies a flattened projective map
  `(n0 + n1*t) / (d0 + d1*t)` of an earlier root. Images cannot reference other
  images, themselves, later nodes or missing nodes.

The encoder interns root and chart records by their exact serialized contents.
An identity chart references its root directly. This also handles inverse
NURBS chart transports that retain identity evidence in memory. IDs follow
first appearance while traversing vertices, pcurves and edges; hash-map
iteration never determines output order. This is deterministic representation
ordering, not a canonical form for all mathematically equivalent polynomials.
Refinement caches, scalar reconstruction caches, addresses and trusted flags
are excluded.

Pcurves also retain `AnalyticParallel { source, distance, range, reversed }`.
The source is a polynomial quadratic/cubic control array or homogeneous rational
Bezier controls, including zero-weight controls. Range endpoints reuse the
selected-parameter graph and its flattened charts; traversal is separate from
ascending parameter order. Direct parallels and their selected-fiber
subdivisions use this definition. General non-Bezier selected parameter fields
remain unsupported by this archive. Loading reconstructs the source and checks
the complete range for poles, undefined normals and unsplit interior cusps.
It accepts certified cusp endpoints and zero-distance stationary sources.
No source regularity flag is serialized or trusted. Source records, degrees and
controls are charged before scalar decoding, including in detached pcurve sources.

This adds a family tag to version 9 without changing existing family encodings.
It preserves the current procedural geometry and selectors, not a complete
authoring-history or source-intent receipt. A saved unattached pcurve does not
establish a valid face or solid. Spatial parallel lifting, full edge/pcurve
agreement and offset-profile extrusion remain open. The
[parallel replay evidence](evidence/2026-10-18-parallel-pcurve-replay/README.md)
records the precise supported workflows and limits.

Other pcurves retain the existing native families or
`SourceRange { source, range, reversed }`. The source indexes an authored
curve in `selection.pcurve_sources`. The range is ascending in that source's
chart; traversal is independent. Sources cannot themselves be restrictions.
Even scalar range endpoints preserve their source chart. Using ordinary native
scalar subdivision here would rebase some curves and change correspondence.
Hypercurve owns this distinction through `Curve2::restrict_source` and
`Curve2::source_restriction`.

## Proof reconstruction

Each root is reconstructed using Hypercurve's checked polynomial, interval and
singleton-isolation constructors. Each image reapplies the projective map,
checking its finite denominator. References are checked before indexing; shared
roots and sources are reconstructed once and cloned as immutable handles.
Spatial points call `Curve3::point_at` with the restored selector. Pcurve ranges
are checked against their restored source and their endpoint images are rebuilt.
The archive cannot introduce a second polynomial solver or bypass Hypercurve's
strict predicate decisions.

ModelBuilder then checks endpoint incidence, ownership, connectivity and the
actual affine parameter maps. For an authored pcurve restriction, the validator
certifies the complete source image against the spatial curve and surface.
Exact restriction transfers that agreement to the trim. It does not sample a
selected point or trust a serialized image certificate.

Consecutive, equally directed restrictions can reassemble an authored boundary
image for region and area proofs. Their sources must agree structurally, their
joining parameters must compare equal, and the source must have a complete
injectivity certificate. Lines retain their direct proof. Other native sources
use Hypercurve's `Curve2::has_certified_injective_image`: one span reuses its
existing certificate; multiple spans must join at exactly equal points and be
strictly monotone in the same coordinate and direction. A missing certificate
does not assert self-intersection and does not authorize reassembly. This prevents
reassembly from hiding an internal self-contact during simplicity validation.
The same helper serves boundary queries and validation. All original vertices,
edges, uses, ranges and incidences remain in the published model. This is an
exact boundary-image proof, not a topology repair or replay of an editing history.

For a planar spline image, the validator restricts the spatial curve to the
mapped edge interval before comparing complete parameterizations. Native
Bezier subdivision may rebase a pcurve to `[0,1]`; NURBS retains its knot chart.
The lift applies the affine parameter map to every NURBS knot and reverses
controls/weights when its scale is negative. A selected restriction validates
its complete authored source on that source's mapped interval, which can be
only part of the original spatial curve after an earlier native cut. Endpoint
agreement alone never certifies the interior. These are proof views; original
geometry and topology records are retained.

## Verified coverage and remaining work

The current regressions cover selected planar line boundaries, a reflected
rational planar boundary, rational and non-unit NURBS extrusion profile sheets,
repeated subdivision, exact planar area, and subsequent subdivision after
reload. Non-affine projective selectors have a separate graph-replay regression.
Adversarial cases cover invalid singleton evidence, missing and cyclic graph
references, poles, wrong spatial selectors, reversed/out-of-domain pcurve
ranges, wrong maps, forged proof flags and trailing JSON. Native persistence
fixtures remain part of the complete HyperBRep test suite.

The [spline replay qualification](evidence/2026-10-15-spline-replay/README.md)
adds native-then-selected-then-selected cuts of rational Bezier, rational
quadratic and single-/multi-span NURBS boundaries, reflected replay, differently
authored pcurve/spatial NURBS charts with both map signs, and five successive
selected cuboid cuts with one shared root. It also rejects modified spline
interiors that preserve their endpoints. This does not establish arbitrary
degree-elevated curve equality, every nonmonotone multi-span image, or general
selected corner/surface/Boolean closure.

Independent solid reconstruction now covers cuboid edge subdivisions (each
edge and all twelve together), their affine shears/reflections, and cylinder
axial subdivisions (one and all four). The saved topology counts and selected
vertices survive; exact volume/classification, another split, another reload
and cold/warm archive-byte equality are exercised. The
[solid replay qualification](evidence/2026-10-15-selected-solids/README.md)
records these cases and their measured costs.

The [periodic-cylinder qualification](evidence/2026-10-15-periodic-cylinder/README.md)
also reconstructs cylinders with one, two, three or four rectangular side faces,
unequal angular widths, shifted periodic charts and mixed axial partitions.
One spatial seam can have two opposite uses on the same face. Replay preserves
every authored record and reconstructs the one-period coverage proof; no stored
validity flag or constructor patch count authorizes the solid. Exact area,
volume, classification, seam subdivision, reflection and coincident-cylinder
Boolean reuse are exercised. General nonrectangular periodic trims and native
single-edge closed circles are separate unfinished capabilities.

Prism extraction projects authored cap sources while retaining exact trim
ranges. Convex planar validation compares every vertex to the oriented face
planes using retained linear forms. A scalar triangle fan remains the native
signed-volume path; selected planar boundaries use
`6V_face = 2 * (origin · (u × v)) * signed_UV_area`.
Wire winding already includes face orientation and hole subtraction. Cylinder
proofs reuse checked reassembled pcurve images to certify their parameter
rectangles. Optional loft/sweep recognizers that need scalar section profiles
decline selected inputs, allowing another complete shell proof to decide them.
No successful selected input bypasses shell closure or geometric validation.

General selected solid validation, selected corners without a reconstructible
boundary image, void-shell nesting, surface/contact/fiber graphs, non-authored
Hypercurve support fragments, general Boolean transfer, and selected angular
conic images remain open. A typed failure is not a successful round trip.

This format is not the completed WP13 resource-boundary design. Aggregate
allocation and scalar-work limits, an iterative WASM-safe record decoder,
independently versioned coefficient/certificate envelopes, semantic provenance
and the optional serialization feature still need implementation. Native
`serde_stacker` handling is retained; WASM compilation does not establish a
hostile-input runtime bound. WP02/G1 and WP13 remain incomplete.

## Algorithm reference

CGAL documents the polynomial-plus-isolating-interval representation and
root-count/construction operations in its
[Algebraic Kernel manual](https://doc.cgal.org/latest/Algebraic_kernel_d/index.html).
This supports the feasibility of replaying selected-root definitions. Our
flattened chart graph and immutable source sharing are implementation choices
built on Hypercurve's existing authority, not algorithms or performance claims
attributed to CGAL. No external kernel is used in production.
