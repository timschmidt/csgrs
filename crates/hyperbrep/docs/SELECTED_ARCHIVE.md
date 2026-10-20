# Selected geometry in archive version 14

The additive `PolynomialFiber` planar-point record stores a bivariate equation,
the selected first parameter, the second-coordinate interval and an optional
exact root witness. Replay checks the singleton in that same selected field;
there is no serialized certainty or trusted root-count flag. Retained-chord
endpoints consume this point record through the public model loader. Mixed
affine scalar images of these points now use the existing `AffineForm` record,
retaining the original point and coefficients through scalar charts and spatial
planar-image replay. `RationalImage { source, map }` retains ordered rational plane images of
polynomial points. The map stores two numerator polynomials and its original
shared denominator. Replay checks every original denominator at the same joint
source before accepting the image; cancellation never removes a pole. Input admission charges both stored polynomial degrees, every
coefficient and every row header, including
empty rows, before scalar decoding. Whole polynomial branches and their spatial planar images use the shared source
definitions described below. Plane/tensor section curves retain both pcurves
through these carriers. A saved model does not serialize a complete intersection
query result or certify a trim arrangement.

Version 14 retains authored coordinates and selected curve images in the same
model. It rejects versions through 13; there is no migration reader.
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
| `records` | 250,000 | Topology/geometry arena entries, selected parameters, detached point/pcurve and polynomial sources, inline extrusion/revolution profiles, planar-image sources, rational maps and their nested point sources, translation/rotation origins, affine-form point definitions and analytic-parallel sources |
| `degree` | 256 | Each declared NURBS axis degree, inferred Bezier axis degree, stored polynomial axis degree, or declared polynomial decomposition axis/projection degree limit |
| `geometry_elements` | 1,000,000 | Aggregate controls (including polynomial parallel sources), weights, knots, root/point/branch-source and rational-map coefficients, and tensor or polynomial row headers |
| `solver_steps` | 1,024 | Each declared polynomial-source refinement-round, fiber subdivision-depth or fiber refinement-step limit, considered individually |
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
Polynomial quadratic/cubic pcurves retain their fixed degree and charge each
control, including malformed arrays. Rational quadratic pcurves have degree two.
Root degree is coefficient count minus one without trimming trailing zeros. Fixed-size analytic coordinates,
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
demand substantial work. In particular, `solver_steps` does not sum controls or
bound total solver work, live heap or cancellation. Metadata parsing and error reporting may allocate;
only the initial syntax scanner has an allocation-free claim. Native
`serde_stacker` remains in the decoder, and configurable depth alone does not
prove a safe WASM call stack. See the [syntax admission evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-archive-admission/README.md)
and [structural admission evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-archive-structure/README.md).

## Records and deterministic ordering

The existing topology arrays retain their order and references. Vertices are
`Coordinates([Real; 3])`, `CurveImage { source, parameter }`, or
`SurfaceImage { source, parameter }`. A surface image embeds the checked surface
definition and one complete planar point definition; it does not store independent
UV roots. Nested surface and point records consume admission quotas before scalar
decoding. Rebuilding the surface and evaluating the UV rechecks frame validity,
domain membership and the admitted image family. A point source
refers either to the model's spatial-curve arena or to a detached immutable
curve in `selection.point_sources`. Detached sources have no model back-reference.
Edges store two `Scalar(Real)` or `Selected(index)` parameter records. Scalar
coefficients remain Hyperreal values; decimal/floating approximations are not
introduced.

A closed circular or elliptic edge stores the same vertex index at both ends
and two distinct parameter endpoints spanning exactly one period. It uses the
existing closed-edge records. Validation reconstructs the carrier's regularity,
period and actual endpoint agreement; it does not trust an archive flag.
Circular cap and periodic cylinder edge uses also replay their complete
pcurve-image, direction and shell certificates. Distinct adjacency, parameter
maps, split/reflection and forged-record rejection are covered in the
[closed-edge evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-closed-edges/README.md).

`selection.parameters` contains three node forms:

- `Root { polynomial, interval }` supplies ascending power-basis coefficients
  and a singleton isolating interval. It does not supply a trusted root count.
- `Image { root, numerator, denominator }` supplies a flattened projective map
  `(n0 + n1*t) / (d0 + d1*t)` of an earlier root. Images cannot reference other
  images, themselves, later nodes or missing nodes.
- `AffineForm { point, coefficients }` supplies a retained planar point and
  `a*x + b*y + c`. The point definition contains authored coordinates, a rational
  or regular parallel source with its parameter, correlated rational coordinate
  polynomials in one selected parameter, a checked `PolynomialFiber`, a
  strict-compatible similarity, or a unique intersection of two exact supporting
  lines. Nested affine lifts of a selected-fiber affine form compose into the
  original source recipe during construction and replay, preserving deterministic
  bytes. Every selected reference must name an earlier node. Replay reconstructs
  the source and shared parameter field; it does not trust precomputed coordinates.

The encoder interns root and chart records by their exact serialized contents.
An identity chart references its root directly. This also handles inverse
NURBS chart transports that retain identity evidence in memory. IDs follow
first appearance while traversing vertices (including detached sources), model
curves, pcurves, surfaces and edges; hash-map
iteration never determines output order. This is deterministic representation
ordering, not a canonical form for all mathematically equivalent polynomials.
Refinement caches, scalar reconstruction caches, addresses and trusted flags
are excluded.

`selection.polynomial_sources` is required, including when empty. Each record
contains an exact bivariate `equation`, rectangular `domain`, `construction`
controls and `evaluation` controls. Construction retains
`BivariateTopologyConfig`: axis/projection degree limits, refinement rounds and
fiber controls. Evaluation retains the separate
`AlgebraicFiberRootIsolationConfig`. Equal serialized source definitions share
one reconstructed source; definitions with different controls remain distinct.
Input admission charges stored dimensions, every row and coefficient, and each
declared control before scalar reconstruction.

`PolynomialBranch { source, slab, root, range, reversed, transform, maps }` pcurves
reference that collection. Replay first runs Hypersolve's complete decomposition
from the equation, rectangle and original controls, then builds Hypercurve's
native branches. It checks the slab/root indices and ascending source range,
applies every rational map in order, then an optional exact similarity, and
restores traversal. `maps` is required even when empty. Each map must certify a
nonzero original denominator over the current conservative source box before
replay can publish a finite branch. This proves a finite parametrized image,
not injectivity; complete surface-section construction separately checks its
spatial image certificate. Branches,
restrictions and planar lifts share the reconstructed source. No event graph,
sampled contour, connectivity or certainty flag is accepted as trusted input.
Zero equations, invalid rectangles, exhausted construction controls, invalid
indices/ranges and invalid similarities retain typed failures. Loading alone
does not prove a valid face or solid; model validation remains required.

This is a breaking format. The 13→14 corpus migration changes only the version
header and appends `maps: []` to each existing polynomial-branch definition.
All other old bytes must match. New rational maps have independent semantic,
guard, composition and deterministic replay tests. The [section evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-plane-tensor-sections/README.md)
records full-image validation after affine transport and cold replay as well as
the exact corpus migration. There is no compatibility
reader. The earlier 12→13 migration added only the version header and empty
`polynomial_sources` collection; its [whole-branch replay evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-polynomial-branch-replay/README.md)
remains unchanged.

Pcurves also retain `AnalyticParallel { source, distance, range, reversed }`.
The source is a polynomial quadratic/cubic control array or homogeneous rational
Bezier controls, including zero-weight controls. Range endpoints reuse the
selected-parameter graph and its flattened charts; traversal is separate from
ascending parameter order. Direct parallels and their selected-fiber
subdivisions use this definition. Other recursive/projective selected fields without one of these
replayable definitions remain unsupported by this archive. Loading reconstructs the source and checks
the complete range for poles, undefined normals and unsplit interior cusps.
It accepts certified cusp endpoints and zero-distance stationary sources.
No source regularity flag is serialized or trusted. Source records, degrees and
controls are charged before scalar decoding, including in detached pcurve sources.

It preserves the current procedural geometry and selectors, not a complete
authoring-history or source-intent receipt. A saved unattached pcurve does not
establish a valid face or solid.

The `PlanarImage { source, origin, u, v, source_chart }` spatial record
retains a finite pcurve source and an injective affine frame. Its source chart
is `[scale, offset]`, with `t_source = scale*t_spatial + offset` and nonzero
scale. Authored endpoint intervals use a unit spatial chart; selected intervals
retain the source chart with a sign change for reversed traversal. No selected
endpoint is divided or replaced with an approximate scalar. The spatial domain
is reconstructed from the finite source and this map. Loading validates the
frame, map, endpoint membership and source constructor, then model admission
rechecks the full edge image and topology. Native `QuadraticBezier` and
`CubicBezier` pcurve tags preserve polynomial definitions used by lifts.

`Translation { origin, displacement }` retains a
straight unit-chart sweep of a point through a nonzero authored displacement.
Its origin shares the rotation origin definition: `Coordinates`, `CurveImage`
with an inline curve and parameter, or `SurfaceImage` with a surface and joint
UV point. Recursive record admission covers every embedded definition. Authored scalar evaluation, split and affine transform keep
the origin's source; an independently selected sweep parameter remains typed
unsupported. `Chord { start, end }` pcurves use checked planar point definitions
and Hypercurve's exact line constructor.

The additive schema-10 planar point record
`SupportingLineIntersection { first: [point; 2], second: [point; 2] }` retains
both source endpoint pairs, including independently selected fields. Replay
reconstructs every source, proves each support nondegenerate and proves a unique
intersection under strict predicates. Finite chord membership is not required;
parallel and coincident supports are rejected. All four endpoint definitions
consume record limits recursively before scalar decoding. No side, incidence
or certainty flags are trusted. Rational coordinate images reuse their shared
original selected root and polynomials; affine extraction uses the same owner
operation during construction and replay. Other records retain their existing
meaning. See the [coaxial qualification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-coaxial-procedural/README.md)
for nested limit/recovery checks and deterministic artifact evidence.

`Rotation` curves add an origin (`Coordinates` or `CurveImage`), authored axis
origin/unit direction, angular domain, direction sign and offset to schema 10.
Selected source and parameter identities use the existing graph. Cold admission
reconstructs the nonzero-radius proof, complete boundary images and profile-axis
clearance; no serialized certificate flag is trusted. Each origin and nested
source is charged before scalar decoding. The [revolution qualification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-revolution/README.md)
retains all prior corpus bytes and repeated new archives. No old record is
reinterpreted and no compatibility reader is introduced.

Coordinate pcurve correspondence records retain a chosen U or V coordinate and
nonzero affine scale/offset. Cold extrusion admission proves constant orthogonal
coordinate and complete profile or axial translation image. Revolution
admission proves complete retained latitude or rotated-meridian images and
rechecks the profile over its whole interval for axis contact. These records carry
no image certificate. Similarities inside affine point-query definitions are
composed when authored and replayed so both encodings use the same recipe.
Version 10 is an intentional schema break, not a byte-preserving optimization.

The [selected-patch correction](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-surface-patches/README.md)
preserves existing pcurve definitions unless analytic-arc export canonicalizes
an incident spatial vertex. This prevents reproduced growth during cold planar
replay. Schema 10 remains unchanged, but fresh scalar expression encodings may
change; exact coefficients, topology/source records, deterministic repeats and
all observed size/time regressions are audited separately. This is a correctness
repair, not a byte-preserving performance change.

Load order is selected roots/charts, authored pcurve sources, detached spatial
point sources, model spatial curves, then point images and topology. Pcurve
sources cannot refer to spatial sources or other pcurve restrictions, so the
new dependency is acyclic. Every inline planar source consumes a record and its
source-specific metadata budget before scalar decoding. This is not a bound
on scalar reconstruction or subsequent geometric validation. The
[planar lift evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-planar-curve-lift/README.md)
records the original lift workflows and costs. The
[selected-image admission](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-planar-images/README.md)
adds spatial point images at selected roots and selected topology trims. They
used the earlier source and parameter graph without a trusted flag. Version 10
extends that graph to selected carrier domains and procedural-profile construction.

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
singleton-isolation constructors. Correlated point expressions prove their
shared denominator nonzero; parallel points also prove regular source weight
and normal where required. Each image reapplies the projective map,
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

For a selected planar-image trim, the validator compares the complete retained
planar source and frame under the inverse affine source-chart map. Exact endpoint
correspondence supplies the actual restriction. This proof subsumes endpoint
sampling without requiring independent scalar coordinates. Source definitions,
parameters, frames and pcurve agreement remain mandatory on cold replay.

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

Procedural parallels use the same proof-view helper. Their supports, traversal
and joining parameters must agree, and Hypercurve must certify an injective
coordinate over the joined range with source regularity and no interior cusp.
An inadmissible joined range keeps the original pieces; instrumented budget
exhaustion remains typed. An exterior interval cannot borrow a unit-source
injectivity certificate.

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

The [spline replay qualification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-spline-replay/README.md)
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
[solid replay qualification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-selected-solids/README.md)
records these cases and their measured costs.

The [periodic-cylinder qualification](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-15-periodic-cylinder/README.md)
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
boundary image, void-shell nesting, general surface/contact/fiber graphs,
unsupported Hypercurve support fragments, general Boolean transfer, and selected angular
conic images remain open. A typed failure is not a successful round trip.

This format is not the completed WP13 resource-boundary design. Aggregate
allocation and scalar-work limits, an iterative WASM-safe record decoder,
independently versioned coefficient/certificate envelopes, semantic provenance
and the optional serialization feature still need implementation. Native
`serde_stacker` handling is retained; WASM compilation does not establish a
hostile-input runtime bound. WP02/G1 and WP13 remain incomplete.

## Finite contact consumers

Finite curve/plane contacts now feed the same selected-point and parameter
archive authority. Higher-degree native contacts retain the original spatial
curve and selected parameter; planar contacts also retain their planar source
point and frame. No new contact-specific saved model format or validity flag
is introduced. The [contact admission](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-retained-plane-contacts/README.md)
exercises actual contact parameters through shared edge splitting, cold replay,
exact reflected transforms, slab cuts, reunion and empty recovery. Finite
restrictions of signed-weight planar sources query only the consumed interval.
The four selected cube/cap STEP attempts before and after replay retain typed,
body/entity-attributed encoding failures and successful native recovery. Native
replay and exact STEP encodability remain separate claims.

## Selected procedural profiles

The selected-profile regressions retain the original nonlinear parallel cap
and its quadratic closing segments after exact clipping creates selected
boundaries. Construction retains selected vertices, profile carriers, axial
translations and side pcurves together. Cold validation proves axial chains
are nonbranching, collinear and forward, connect distinct cap boundaries, and
are wholly covered by their incident side faces. It does not assume one axial
edge per connection or authorize a solid from endpoints alone.

Tests consume these models through selected profile and authored axial splits,
affine reflection/shear, holes, disconnected components, reversed contours,
repeated slab operations, both directed set identities and empty recovery.
Malformed maps, displacements, selected references, point expressions and scalar
records must fail with typed diagnostics. Strict/approximate/strict reuse and
budget exhaustion/recovery preserve the certainty contract. The full clipped
profile currently reaches the STEP writer's attributed
`UnsupportedCurve(PlanarImage)` frontier; native replay remains separate.
Qualification and its remaining limits are tracked in the
[selected procedural-profile evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-selected-procedural-profiles/contract.md).

## Algorithm reference

CGAL documents the polynomial-plus-isolating-interval representation and
root-count/construction operations in its
[Algebraic Kernel manual](https://doc.cgal.org/latest/Algebraic_kernel_d/index.html).
This supports the feasibility of replaying selected-root definitions. Our
flattened chart graph and immutable source sharing are implementation choices
built on Hypercurve's existing authority, not algorithms or performance claims
attributed to CGAL. No external kernel is used in production.

Full procedural revolutions reuse schema 10's existing PlanarImage, Rotation,
selected graph and Coordinate records. No constructor certificate is persisted:
cold validation reconstructs the single-cover shell and positive-radius simple
profile, including selected cavity endpoints. Existing native images keep their
encoding; the [solid-revolution evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-18-procedural-revolution/README.md)
records replay and the unchanged procedural STEP boundary.

Selected iso pcurves now use a degree-one native NURBS source with the original
coordinate as its knot parameter and an exact `SourceRange` restriction.
Certified scalar bounds choose only a finite source support. They never replace
the selected endpoints. This reuses the existing archive schema and cold
validation; that increment did not admit arbitrary generated rational fragments
or whole implicit branches. Version 13 adds the polynomial branch records above.
See the [parameter-map record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-selected-pcurve-maps/README.md).

Selected section UV evaluations now feed exact chord endpoint definitions in the
existing retained point graph. Rational Bezier/NURBS section roots, restrictions
and reversal replay cold with independently checked UV images. The same models
retain their spatial selected point through transform/replay. That pcurve increment introduced no new schema.
Arbitrary generated rational curve carriers remain a separate persistence target.
See the [evaluation record](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-pcurve-evaluation/README.md).

Version 11 introduced a generic translation `origin` in place of the earlier
`source`/`parameter` pair. Plane images preserve joint polynomial-fiber and selected rational UV
evidence through point queries, affine transforms, sweeps and cold replay.
That increment covered plane point images; nonlinear surface images and whole
surface-intersection branches remained open. Existing payloads were compared
after only the declared schema migration, alongside unchanged exact geometric assertions.
See the [surface archive evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-surface-evaluation/README.md).

Version 12 added `AffineImage { source, transform }` to spatial curves. The source
is the complete embedded spatial curve definition and `transform` is an exact
4×4 affine matrix. Nested record/geometry/degree limits apply before scalar
decoding. Checked reconstruction proves the matrix affine and nonsingular,
rebuilds the source, and composes nested image matrices. No proof bit, sampled
point or prepared rational surface coefficient map is serialized.

Selected rational Bézier/NURBS tensor points and native rational-profile
extrusion points use the existing `SurfaceImage { source, parameter }` record.
Replay checks original weights, domains, knot-span ownership and joint selectors
and rebuilds coefficient maps. Affine-transformed rotation edges retain this
point authority through their embedded source. That increment rejected versions
through 11 without a legacy decoder. Its preserved archive corpus comparison
permitted only the top-level 11→12 header change and required all other bytes to match.
See the [rational surface and affine curve evidence](https://github.com/timschmidt/csgrs-evidence/blob/b20000e377626a41b6ff8e6f87aee2b20ebd7865/crates/hyperbrep/docs/evidence/2026-10-19-rational-surface-evaluation/README.md).
