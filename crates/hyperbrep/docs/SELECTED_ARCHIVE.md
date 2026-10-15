# Selected geometry in archive version 9

Version 9 retains authored coordinates and selected curve images in the same
model. It replaces version 8; there is no migration reader. `RawModel::from_json`
checks the format envelope before decoding records and rejects trailing input.
`RawModel::validate` is the trust boundary. Parsing alone grants no geometry,
measurement or topology authority.

## Records and deterministic ordering

The existing topology arrays retain their order and references. Vertices are
`Coordinates([Real; 3])` or `CurveImage { source, parameter }`. A point source
refers either to the model's spatial-curve arena or to a detached immutable
curve in `selection.point_sources`. Detached sources have no model back-reference.
Edges store two `Scalar(Real)` or `Selected(index)` parameter records. Scalar
coefficients remain Hyperreal values; decimal/floating approximations are not
introduced.

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

Pcurves retain the existing native families or
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

This format is not the completed WP13 resource-boundary design. Byte/entity,
degree and aggregate allocation limits, an iterative WASM-safe decoder,
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
