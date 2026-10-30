# Migrations

This document lists the CSGRS changes that need caller action or that change
represented geometry or serialized artifacts. The [README](../readme.md) and
the API documentation describe current behavior only. A migration is a
deliberate contract change (P27), not a performance-only change.

## Dilation of regions with holes

`CurveRegion2::offset` with a positive distance now builds the result from the
exact boundary bands, as contraction already did, when a region has a hole
that is not certified to survive. The previous boundary walk kept a hole whose
parallels had crossed: an annulus, square or slot hole dilated to or beyond its
half-width, a channel that should close between two surviving lobes, or a
convex hole edge that vanishes first. Those results were wrong and are now
correct. Circular holes wider than the distance, and strictly convex polygonal
holes whose shifted edges all keep positive length, still use the walk and
their existing representation, as do hole-free regions. Other dilations with
holes describe the same set as before when that set was already correct, but
their boundary may be split or ordered differently, so serialized bytes can
change.

## Cooperative cancellation of budgeted work

`WorkKind` has a new `Cancellation` variant; exhaustive matches on it must
handle that case. It appears only in scopes entered with
`work_budget::with_cancellable_budget` or Hypercurve's
`evaluate_with_cancellable_budget`, after their `CancellationToken` is
cancelled. Such a `BudgetExceeded` has zero `limit` and `used`. Code that only
uses `with_budget` or `evaluate_with_budget` sees no new outcome. Inner
operations report a cancelled request through their existing budget-exhausted
blockers; the outer result's kind distinguishes it from a reached limit.

## Reusing selected points and equal field generators

Polynomial branch evaluation may return an already selected point after proving
its source abscissa, defining equation, ordered branch and complete fiber census.
It no longer necessarily constructs a fresh `PolynomialFiber` definition. This
also applies in a retained similarity chart. Callers must use point semantics
and the returned definition instead of assuming that every branch evaluation
has one definition variant. Original source selections and denominator guards
remain authoritative; no coordinates are rounded or independently selected.

Selected-field joins can prove that an independently allocated generator equals
an existing parent generator. They retain both original defining equations,
selectors and source identities, and use the proved linear relation internally
to avoid an independent coordinate axis. Repeated factors remain in the original
definition. Native `JoinedField` replay validates the source selections and
re-establishes this relation; there is no serialized equality flag or new tag.
Coordinate expression shape and serialized bytes can change. Semantic replay
and deterministic re-export are separate obligations. Promotion accounting
continues to apply; these changes do not establish a total work or memory bound.

## Polynomial chord setbacks and selected chord incidence

Trim-only chamfers can cut admitted polynomial graph branches at exact Euclidean
chord setbacks. The selected circle sections retain the corner's shared fields;
retained branch endpoints and generated chord endpoints keep the same point
evidence. Existing finite-solution selection, zero-design, invalid-input and
no-solution outcomes still apply. Nonzero trim-or-extend requests require a
separate branch-continuation contract and remain explicitly unsupported.

Polynomial incidence consumes finite generated chords with selected endpoints
in the same source plane or an inverse similarity chart. It validates candidate
contacts against the original branch and domains, preserves point-based chord
parameters, and proves common-component identity before reporting overlap.
Singular tangent signs can remain unclassified even when the contact set is
complete. These additions do not establish general polynomial offsets, fillets,
whole-path SVG interchange or unrestricted repeated corner edits. The capability
matrix and work records distinguish the completed witnesses from those gaps.

## Polynomial roots with retained coefficient queries

`CurvePointDefinition2::SelectedPolynomialRoot` defines the point `(z, 0)`
from ascending `CurvePointRationalForm2` coefficients and a singleton root
selector. Exhaustive matches on the definition enum must handle this variant.
Keep each original query and its denominator, including zero coefficients;
extracting independent scalar coordinates is neither required nor equivalent.
A nonexact selector is an open interval containing exactly one distinct root.
An exact root must equal both bounds and satisfy a nonzero equation. The supplied
root-count field is not trusted. Invalid selectors and equations remain errors;
undecided predicates and reached resource limits remain typed blockers.

Native SVG path graph version 4 adds selected-coefficient point tag 28. Each
coefficient references a point definition and carries its original numerator
and denominator. HyperBRep format 14 adds `SelectedPolynomialRoot`, whose
coefficients reference checked rational-query records in the selected-parameter
graph. Older readers can reject these new variants; existing records keep their
meaning and encoding. Replay validates all references, original guards and the
root selection. Repeated coefficient sources use shared graph dependencies.

This extends exact points and their consumers. It does not establish general
implicit-region offsets, corner edits or a total work/memory bound.

## Polynomial fibers over retained query parameters

`CurvePointDefinition2::PolynomialFiber::parameter` now takes `CurveParameter2`
so that a generated selected-field query can become the base of another fiber.
Wrap existing `BezierParameter2` arguments with `.into()`. Code matching the
native representation should use `parameter.as_bezier_parameter()`; absence
of that view does not mean the exact parameter is invalid.

Native SVG path graph version 4 retains its existing fiber record (tag 14),
whose parameter reference may now name a checked query parameter (tag 27).
The same source recipe is retained by HyperBRep's planar fiber archive field.
Readers for the earlier restricted schema can reject these new dependencies;
they cannot reconstruct them by extracting independent coordinates. Existing
native-base records retain their geometry and encoding. Replay validates the
original source guards and root selector, including the retained base field.

This extends selected graph evaluation and point composition. It does not
establish general implicit-region offsets, corner edits or topology closure.

## Region query certainty after approximate construction

Geometric queries on `CurveRegion2` retain the policy required by its
normalization. Earlier membership, area, role, bounds and native-view queries
could report a strict result after an explicitly approximate construction had
removed or changed topology. These queries now return
`ExactCurveError::Blocked(Predicate)` under `STRICT` when that inherited
requirement has no strict proof. A cached value or an empty boundary does not
establish fidelity to the original source.

An explicitly authorized `evaluate_under(PredicatePolicy::APPROXIMATE_512, ...)`
readback keeps its `Approximate512Consumed` receipt and the retained geometry's
query value. Callers should keep that receipt through subsequent operations;
obtaining a strict source claim requires a new proof or strict reconstruction
from the original inputs. Exact constructions remain certified even when the
caller requests the approximate policy.

The existing approximate Boolean, miter-option and stroke-connectivity
fixtures retain their inputs, construction policies and geometric expectations.
Their readback assertions now require strict rejection followed by authorized
qualified success, then strict rejection again. The independent positive-height
cap regression prevents treating a disappeared boundary as proof of the source
empty set.

The CSGRS receipt-composition fixtures apply the same rule to boundary
readback, translation, extrusion and `GeometryOutcome::and_then`/`map`.
Strict source claims remain blocked; authorized operations retain the
approximation receipt and their original geometry expectations. Their
separate strict reconstruction control explicitly adopts the extracted
boundary as a new model. Its local proofs do not establish fidelity to the
original approximate construction. Meshes with missing source history keep
that missing-history marker rather than claiming certified ancestry.

## Finite profile projection retains topology certainty

Both finite profile projection APIs and CSGRS `try_finite_profiles` validate
the input region's retained normalization policy before using an empty result
or cached loop topology. An approximate closure or normalization therefore
remains `Approximate512Consumed` under explicitly authorized projection;
strict reuse returns a typed `Predicate` blocker unless the source claim has
a strict proof. Reusing topology avoids recomputation but does not upgrade
its certainty. This applies to nonempty profiles as well as empty results.

The existing terminal-closure profile fixture keeps its original source and
single-profile expectation. It now checks the construction receipt, strict
rejection, authorized qualified readback and repeated policy transitions.
Projection remains a boundary product with its declared sampling and coordinate
loss model; its validity does not certify fidelity to the original source.

## Exact empty SVG documents

Hypercurve and CSGRS now export a valid exact empty region without strokes as
an SVG document. Previously this returned an error because the empty set has
no finite bounding box. The document uses a unit `viewBox` for display and a
version-five native graph with no definitions or roots; the viewport creates
no geometry. Native reimport returns the empty region, which remains a valid
operand for further operations. Callers that used export failure to detect
emptiness should inspect the geometry instead.

Invalid export options still fail. A region whose empty topology depends on
an approximate decision still fails strict exact export; removing its final
boundary does not certify its source geometry.

## Fallible constructors and operations

These functions now return `Result`. They used to return empty or partial
geometry on error; now they fail. Propagate errors with `?`, or match
`ValidationError::kind()`.

| Area | Functions | Notes |
| --- | --- | --- |
| 2D primitives | rectangle, square, circle, ellipse, regular polygon, right triangle, polygon and rounded-rectangle constructors | Policy-taking and compound variants return `Result<GeometryOutcome<CurveRegion2>, ValidationError>`. |
| Sampled 2D profiles | `arrow`, `trapezoid`, `star`, `teardrop`, `egg`, `squircle`, `pie_slice`, `heart`, `supershape` | An equal-width arrow no longer emits duplicate shoulders. In WASM, star, teardrop, egg and heart throw `GeometryError` with a `code` instead of publishing failed profiles. |
| Gears and airfoils | the five gear, rack and airfoil constructors | Invalid profiles no longer become successful empty solids, including at zero thickness. |
| Splines | `bezier_path`, `bspline_path`, `bezier_region` | A missing path or empty geometry no longer stands in for failed construction. |
| Hilbert curves | `hilbert_strings` | Replaces midpoint-only clipping and empty-on-error behavior. Output segmentation may change at actual boundary contacts. |
| Text | both text constructors, `parts::annotate` | Invalid fonts, missing glyphs, failed outlines and undecided scales no longer produce empty or partial text. |
| Curve transforms | `curve::transformed`, `translated`, `rotated`, `scaled` | Failures no longer become empty regions. `contains_xy` returns `Result<Option<bool>, ValidationError>`. |
| 3D primitives | `cube`, `cuboid`, `sphere`, `cylinder`, `ellipsoid`, `frustum`, `frustum_between`, `octahedron`, `icosahedron`, `arrow`, `torus` | Return `Result<TriangleMesh, ValidationError>`. |
| Bounds | `solid::bounding_box`, `curve::bounding_box`, their `try_` variants, `SolidExt::exact_bounds`, `solid::center`, `float` | Bounds queries return `Result<Option<Aabb>, ValidationError>`. `None` is a completed query on empty geometry. |
| SDF and metaballs | the four SDF constructors, the 3D metaball constructors, `curve::metaballs` | Grid dimensions below two are rejected instead of silently increased. |
| TPMS | `gyroid`, `schwarz_p`, `schwarz_d` and their `_solid` forms | Sampling formulas and valid resolutions are unchanged. Resolutions below two are rejected instead of clamped, and undecided sampling no longer produces an empty model. |
| Merge and distribution | `solid::merge`, `distribute_linear`, `distribute_grid`, `distribute_arc` | Invalid input, undecided predicates and capacity exhaustion no longer become empty successful models. |
| Planar sections | `flatten`, `slice_z` | Also a correctness change: slice topology was corrected, and earlier slice serialization is not preserved. |

Other legacy convenience operations that replace errors with empty geometry
remain an open migration target.

## Error types

- **Retained curve policy.** `hypercurve::CurveError` gains
  `IncompatiblePredicatePolicy`; exhaustive matches must handle it. Retained
  chord and selected-fiber replay validators now distinguish policy mismatch
  from malformed topology. Principal operations expose it as
  `ExactCurveError::Blocked` with reason `Predicate`; CSGRS classifies both
  that blocker and the low-level cause as `ValidationErrorKind::Undecided`.
  Callers may retry under an explicitly permitted policy. Truly malformed
  loops remain invalid.
- **Scalar adapter Booleans.** `ScalarMesh::{union, difference, intersection,
  xor}` and their context variants report
  `AdapterError::Construction(ValidationError::Mesh(...))` instead of the
  text-only `AdapterError::Validation`.
- **C ABI Booleans.** In the C ABI and its C++, Python and Go wrappers,
  undecided mesh Booleans report `PREDICATE_UNDECIDED` (5) instead of
  `VALIDATION_FAILED` (4). Invalid input still reports (4).
- **Export failures.** PLY, glTF (including `GltfSceneObject::new`) and OBJ
  native construction failures used `IoError::Geometry` with only a string.
  Match `IoError::Construction` instead. Its display prefix is
  `"<format> construction failed:"`, followed by the native diagnostic.
  Successful serialization is unchanged.
- **glTF documents.** Empty glTF operations that returned `IoError::Geometry`
  now succeed. A document without a scene is `Unsupported` rather than
  `MalformedInput`.
- **Sweep frames.** `curve::try_sweep` reported every frame-construction
  failure as `ValidationError::InvalidArguments`. An undecided predicate now
  reports `PredicateUndecided` with the sweep step as `predicate`, for
  example `"sweep path reversal"` or `"sweep frame transport"`. Other scalar
  failures report `ValidationError::Scalar` with their cause. Invalid paths
  still report `InvalidArguments`. Code that treated `InvalidArguments` as
  every sweep failure should match `ValidationError::kind()` instead. Paths
  whose transport axis lacked a structural nonzero certificate, such as
  non-planar sampled paths, now succeed. Previously supported paths keep
  their exact rows.
- **Twisted extrusion parameters.** `curve::extrude_twisted` reported a zero
  height and every invalid end scale as `ValidationError::InvalidArguments`.
  A zero height now reports `ZeroParameter { name: "twisted-extrusion
  height" }`, and an end scale with one zero or a negative component
  `NonPositiveParameter { name: "twisted-extrusion end scale" }`. Both
  scales zero now succeed, tapering to an apex. `ValidationError::kind()`
  is `InvalidInput` for all three variants, so WASM `extrudeTwisted` keeps
  its `invalid_input` code; its message now names the parameter.
- **WASM curve meshes.** `CurveRegionJs.sweep`, `revolve` and
  `extrudeTwisted` throw a `GeometryError` with a `code`, as `extrude`
  does, instead of a bare message string for native failures.
- **Booleans of approximate results.** A Hypermesh Boolean result, and so a
  csgrs solid Boolean result, retains that its triangles meet only in shared
  vertices or edges, so a later Boolean skips testing it against itself.
  When the result's own Boolean consumed the `APPROXIMATE_512` terminal, that
  fact now keeps the approximate certainty. A later `STRICT` Boolean no
  longer takes it as proved: it tests the result against itself, as it does
  the same rows rebuilt, and can report `PredicateUndecided` where it
  reported a certified result. A later `APPROXIMATE_512` Boolean still uses
  the fact, publishes the same rows, and reports `Approximate512Consumed`
  where it could report `Certified`. Results whose Booleans were certified
  are unaffected.

## Serialized output and geometry

These changes alter represented sets, coordinates, file lengths or checksums.

- **Polynomial branch SVG replay (2026-10-29).** Versions 4 and 5 add shared
  polynomial source and selected-branch records (tags 29 and 30). The defining
  equation, original domain, construction/evaluation controls, slab/root
  indices, selected trims, traversal, rational maps and similarities are kept.
  Readers rebuild topology and validate every definition, including unused
  records. Older readers reject these tags; existing records retain their
  encoding. `SvgError::PolynomialTopology` is a new exhaustive-match arm.
  CSGRS retains that cause in `ValidationError::PolynomialTopology` and maps
  its category without converting it into a string or a successful empty result.
  Polynomial display projection now has a certified chord-error bound and
  explicit subdivision limits, while exact metadata stays authoritative.
  Conservative polynomial bounds now clip the source abscissa by certified
  trims before image transport; this can tighten exported display viewports.
  Native SVG replay now retains uncut original native lines after the same
  arrangement checks their complete source ranges. Ordinary region construction
  retains its established publication; selected chords and analytic parallels
  preserve their existing carriers.
  This extends admitted finite-fiber graph branches; it does not complete
  arbitrary implicit regions, repeated corner edits or total resource bounds.

- **Shared selected-point queries (2026-10-29).** Independent rational queries
  over admitted selected fields can now compose exactly while preserving each
  original selector and denominator guard. Callers that previously handled
  `Blocked(Unsupported)` for different selectors may receive certified results;
  opposite roots remain distinct. `CurvePointDefinition2::RationalQueryPair`
  and `CurveError::UncertainFiniteCurveProjection` require new exhaustive-match
  arms. Native SVG graph tags 26 and 27 retain query-pair points and source-bound
  query parameters. Older readers reject these tags; existing records keep
  their encoding. HyperBRep archives retain the corresponding source recipes.
  Replay validates source references, selectors and original guards, including
  unused definitions. This capability does not complete general algebraic
  region operations or their resource envelope.

- **Rational images of selected rational-curve points (2026-10-27).**
  `CurvePoint2::rational_image` accepts native selected rational-curve point
  images and repeated rational maps. Each immutable image shares its original
  point and keeps the original map and denominator. Exact point queries,
  similarities, line construction and native replay consume those definitions.
  Original poles remain invalid after algebraic cancellation. Native replay
  remains strict inside a provisional scope, and identity and constant queries
  preserve retained qualification. An actually reached work limit is reported
  before a new image is published. Existing point definitions and wire records
  suffice; no new public carrier or format version is required. Other point
  fields and general region/operation closure remain open.

- **Selected circle chart derivatives (2026-10-27).** Public derivative queries
  consume represented angular parameters and incident points in a shared
  polynomial-point field when the circle has a represented unit frame. Exact
  coordinate images preserve magnitude and every requested derivative order.
  The center stays constant in the source chart even when its coordinates and
  the queried parameter share algebraic dependencies. Retained restrictions and
  reversal keep the source parameter; similarities apply their linear part to
  vectors. Inherited frame qualification survives center cancellation. Existing
  point definitions support replay and further line construction without a new
  wire version. Other frame and point-image carriers and general region admission
  remain open; these fixtures do not establish universal operation closure.

- **Exact filled-region SVG replay (2026-10-27).** Filled exports now carry
  version 5 compound path roots over the existing checked definition graph.
  One SVG path contains all regularized material/hole boundaries with even-odd
  fill and a separate finite drawing. Hypercurve import validates definitions,
  selections and closure, then rebuilds `CurveRegion2` through its arrangement.
  Versions 3 and 4 remain readable, and standalone stroke output keeps those
  versions. Older Hypercurve readers reject version 5; other SVG viewers render
  the drawing. Serialized filled exports intentionally change. No approximate
  region is promoted by an archive: unresolved strict replay fails. Limits count
  the complete compound extension and point depth; a larger retry is supported.
  Empty compound metadata imports as empty, while whole-empty-document export
  retains its finite-bounds error. These named workflows do not complete arbitrary
  implicit-region replay, general circle incidence or total resource control.

- **Selected circle regions (2026-10-27).** Shared-field circular boundaries now
  retain selected points through winding, side classification and the existing
  Boolean arrangement. Native minor-arc predicates and region fill semantics
  remain authoritative. Generated line representatives reuse a common field,
  and circle/line endpoint proofs retain exact half-chart and transverse data.
  Named degree-seven/five fixtures cover holes, disconnected components, winding
  cancellation, mixed boundaries and a round +1/4 then -1/4 offset chain. No public
  type or wire version changes. General incidence, large erosions and exact
  filled-region native replay remain implementation gaps.

- **Selected circle points as angular authority (2026-10-27).** An exact point
  incident on a selected circle can retain its joint rational/algebraic field
  through point location, restriction, subdivision, reversal and similarities
  when the circle has a represented unit frame and both radial coordinates share
  a query field. Coordinate and angular roots need not be materialized.
  Existing circle incidence and signed half selection still gate admission.
  Native version 4 reuses its checked incident-point record and rebuilds the
  query authority from the point and circle; no new tag or version is added.
  Prior source/contact carriers retain their authority when the field pullback
  is unavailable. The later differential extension covers represented frames
  and shared polynomial-point images; general filled-region admission for
  correlated centers remains open.

- **Correlated point predicates and frame replay (2026-10-27).** Shared rational
  point-field pullbacks now serve oblique linear comparison and selected-circle
  incidence. A proved constant displacement in a represented circle frame uses
  the existing exact half-chart inverse, preserving signed radius, traversal and
  complementary-half ownership. The original degree-seven/five fiber-center
  midpoint case now continues through splitting and native replay. This does not
  provide general angular inversion in independent selected fields.
  Version 4 uses its existing checked unit-direction record when an ordinary
  endpoint normalization would reconstruct a different scalar definition.
  Readers retain the checked incoming components even if another vector has
  equal values. Reconstructible identical definitions keep their old encoding;
  affected transformed frames may gain an existing tag25 record. Geometry is
  unchanged and no format tag or version is added.

- **Selected-circle graph replay (2026-10-27).** Version 4 adds a chord-normal
  circle frame, scalar angular parameter, signed normal contact, finite circle
  range and independently incident point (tags 20–24). A checked chord unit
  direction (tag 25) preserves transported oblique frames; its reference retains
  the distinction between an infinite support and a finite chord. Native endpoint
  directions already reconstructed from represented coordinates keep their old
  encoding. Readers validate every unused definition, the literal signed
  displacement, circle incidence, selected half and ascending domain. No tangent
  or incidence Boolean is accepted as proof. Older readers reject the new tags;
  existing version 3 and version 4 records remain readable. Circle frames add one
  dependency level to `SvgOptions::max_point_definition_depth`. These witnesses
  cover named chord-normal frames, not all generated circle charts or filled
  region archives.

- **Selected-point graph replay (2026-10-27).** Version 4 metadata adds
  polynomial-fiber, rational-image, similarity, supporting-line-intersection
  and shared chord-displacement point records (tags 14–19). Existing version 3
  and version 4 records remain
  readable; older readers reject the new tags. Authored version 3 encoding is
  unchanged. Shared rational-map prefixes are encoded once per path. Check
  geometric equality separately when updating deterministic output expectations.
  `SvgOptions` adds `max_point_definition_depth` with default 256; complete Rust
  struct literals must set it or use `..SvgOptions::default()`. The new
  `CurvePointDefinition2::ChordDisplacement` variant requires exhaustive enum
  matches to add an arm. Zero allows leaf
  point definitions. The limit counts map, similarity, displacement and
  intersection nesting
  in the input graph, including dependencies reached through curve endpoints,
  even when a constructor can simplify the resulting point. It does not bound
  total arithmetic work, coefficient growth, heap use or cancellation.
  Hyperbrep native planar-point archives also add `ChordDisplacement`, replaying
  it through Hypercurve after counting its source and optional origin records.
  Older readers reject this new variant; existing archive variants keep their
  format. Hyperbrep retains its existing nested point syntax and archive budgets.
- **Selected-chord SVG replay (2026-10-27).** Version 4 native path metadata
  gains point, supporting-chord, selected-range and curve-endpoint records
  (tags 7–13). Repeated retained curves may share one definition, changing
  metadata length and checksum for those paths. Current readers still accept
  existing version 3 and version 4 records; older readers cannot consume the
  new tags. Authored version 3 encoding is unchanged. Applications comparing
  generated-path bytes should regenerate their serialized expectations while
  checking exact geometry separately. The standard SVG projection of selected
  parallel endpoints now follows a reproducible refinement sequence, so warmed
  predicate caches no longer choose different display vertices. Existing
  display files affected by that bug can change; authoritative exact geometry
  and the requested projection tolerance are preserved.
- **PLY decimal spelling.** Positions and normals used a fixed 17 fractional
  digits, which could round small nonzero values to zero. Each binary64 value
  is now spelled exactly, with up to 1074 fractional digits where needed.
  Already exact spellings, headers and face rows keep their bytes. Affected
  files change length and checksum, and some become larger.
- **AMF decimal spelling.** Vertex coordinates and color components used a
  fixed 17 fractional digits, which rounded magnitudes below `5 * 10^-18` to
  zero and could change the binary64 value of others below 0.1. Each binary64
  value is now spelled exactly, with the formatter PLY, OBJ and ASCII STL
  use. Already exact spellings, such as integers and dyadics with at most 17
  fractional binary digits, keep their bytes; other files change length and
  checksum and become larger.
- **TrueType outlines.** Native lines and quadratic/cubic Bézier curves
  replace the former eight-chord sampling. The point size uses exactly
  `127/360` millimeters per point instead of binary64 `0.3527777`. Represented
  sets, edge counts and serialized artifacts may change. The `truetype-text`
  feature now includes `curve` and no longer depends on `ttf-parser-utils`.
- **SDF meshing.** The modeling path uses Hypermesh's
  `surface_nets_standalone`. That includes every face whose four incident
  cells are present, where the chunked convention could omit final-layer faces.
- **3D metaballs.** Extraction samples the normalized field
  `F = (iso - I)/(iso + I)`. This removes the former distance cutoff, finite
  singularity sentinel and hidden `f32` vertex construction. Output
  coordinates and serialized artifacts change.
- **2D metaballs.** `curve::metaballs` samples the same normalized field. This
  removes its cutoff, center sentinel and empty-on-error fallback. 2D
  coordinates and serialized artifacts change.
- **Triangulation and linear extrusion.** Certified line profiles go through
  Hypertri without finite-coordinate projection. Exact vertices and output
  artifacts can differ from the former quantized output.
- **Sweep, twisted extrusion and revolution.** `curve::try_sweep`,
  `extrude_twisted` and `revolve` consume certified line profiles the same
  way. A profile whose vertices binary64 represents exactly keeps its rows.
  The vertices of other line profiles, such as coordinates in thirds or
  multiples of sqrt(2), are now the exact authored values, so their rows,
  exported coordinates and checksums change. Their construction evidence no longer reports
  `source_sampled()` or `coordinates_projected()`. Caps are triangulated by
  `hypercurve::triangulate_linear_contours`, as in extrusion. An undecided
  loop orientation in any of these operations, extrusion included, reports
  `PredicateUndecided` for `"profile side-ring signed area"`; extrusion
  reported `"extrusion side-ring signed area"`.
- **Mixed modeling profiles.** A curved sibling or hole no longer quantizes
  all line geometry. Triangulation, extrusion, sweep, twisted extrusion and
  revolution select exact or sampled boundaries per classified profile.
  Within sampled profiles, native line spans and representable curve endpoints
  stay exact, with one coordinate authority at each certified join. Nonlinear
  interiors and endpoints without native coordinates still use the existing
  finite sampler at a `1e-3` chord-error request. Sampling is not an analytic
  fidelity or topology certificate; the whole mesh continues to report
  `source_sampled()` and `coordinates_projected()`. Formerly rounded native
  coordinates, mesh rows, serialized sizes and checksums can change. The
  explicit `try_finite_profiles` export still quantizes every coordinate.
- **Algebraic Bézier samples.** Finite export and sampled modeling now refine
  retained parameter intervals until their source control enclosure meets the
  projection quality request. Coarse isolator midpoints no longer substitute
  for cut endpoints. Sample coordinates, counts, sizes and checksums can
  change; this remains a sampled representation without a new certified
  topology or source-fidelity bound.
- **Full revolutions across the axis.** The earlier radial-fold change
  replaced overlapping opposite-side bodies with their union, fixing winding
  two. R01 preserves that regularization while correcting the odd-segment
  phase: both signed halves now keep the placement
  `(x cos(t), y, x sin(t))` and their polyhedra are united by Hypermesh.
  Adding a distant positive component no longer rotates a negative component
  by half a turn. Mixed-sign full turns with odd segment counts intentionally
  change geometry, triangle rows, output sizes and checksums from the folded
  version. Even lattices still use the equivalent planar fold. One-sided
  profiles and partial sweeps retain their previous placement.
- **Revolution coverage and start.** Exact integer multiples of 360 degrees
  now cover one full turn with the requested number of segments and the
  authored direction; 720 and 1080 no longer fail. Oversized noninteger
  sweeps such as 450 remain invalid, preserving the previous domain instead
  of silently saturating or applying modulo reduction. An unproved integral
  turn count reports `PredicateUndecided`. `curve::revolve_from` selects an
  explicit starting meridian (degrees); `revolve` continues to start at zero.
  Mesh coverage does not represent the traversal multiplicity of a
  parameterized multi-turn surface. These are CSGRS semantics, without a
  claim of matching every OpenSCAD version or call default.
- **Imported face triangulation.** OBJ, DXF and VRML import project each
  planar polygon face onto two coordinates before triangulating it. The
  dropped coordinate is still the largest normal component whenever strict
  comparison decides it, so those faces keep their triangles. Otherwise it is
  the first component the policy proves nonzero. Under `APPROXIMATE_512`, a
  face whose largest component only the 512-bit terminal could decide now
  takes that path: its triangles can differ, and it consumes the terminal only
  if proving that component nonzero needs it. When no component is proved
  nonzero, the error detail reads "face projection normal is indeterminate"
  instead of "dominant projection axis is indeterminate".
- **Minkowski sums.** `solid::minkowski_sum` adds the represented bounded
  closed-PWN solids. The former convexification of all stored vertex sums is
  `solid::minkowski_hull`; move point-cloud or intentional convex-hull callers
  to that name.
- **Boolean batches.** Conversion from a shared Hypermesh Boolean batch gives
  empty rows zero positions as well as zero triangles.
- **Boolean output rows.** A Hypermesh Boolean result, and therefore every
  csgrs solid Boolean (`try_union`, `try_difference`, `try_intersection`,
  `try_xor` and the operations built on them), no longer keeps corefinement
  vertices that its represented set does not need. Coplanar pieces of each
  result are merged; a vertex inside one plane, or on a straight crease
  between two planes, is removed and the space around it is retriangulated
  with Hypertri. Represented sets, closure, orientation, the certainty
  receipt and the exact coordinates of the remaining vertices are unchanged,
  and no vertex is added. Vertex and triangle rows, their counts and order,
  and file lengths and checksums of exported results change, mostly to much
  smaller outputs: sixteen square holes in a block give 332 triangles instead
  of 1,600. A result triangle that covers several coplanar input triangles
  reports the least of them as its `hypermesh::TriangleSource`. Points that
  a later Boolean builds on such a result use the source face planes that the
  result retains, so symbolic coordinates keep their values but can be stored
  as smaller expressions. Where a later cut crosses an edge between two of
  the result's distinct planes, the new vertex is built from those two planes
  and the cutting plane instead of interpolated between the edge's
  endpoints: the same exact value, stored as an expression that does not
  grow with earlier operations. Each retriangulated polygon fans around its
  least vertex wherever that vertex sees it, ranking the vertices of an
  operand that retains an earlier Boolean's facts first and then by first
  use; that choice of diagonals is part of the published rows. The ranking
  changes the diagonals only when such an operand follows one that does not,
  as in `tool.union(&body)`. Coordinates of one axis that
  strict policy proves equal are stored as one expression, the smallest of
  them, so a coordinate's expression can differ from the one its
  construction produced while its value stays the same. A coordinate of a
  segment-plane crossing is stored as its exact normal form, a rational
  combination of square roots of rationals possibly times powers of pi,
  when that has fewer expression nodes; such a value can now be stored as
  an exact rational where its radicals cancel. Serialized `Real` expressions
  of those coordinates change; their values do not. Regenerate pinned
  Boolean rows, counts and checksums.

## Shared reciprocal expression artifacts

The R19 exact-completion change may construct an exact `One` computable
expression for `x * inverse(x)` when both operands share the same
retained nonzero value and a bounded audit can preserve every cancellation
handle. The exact mathematical value and certainty are unchanged, but serialized
expressions of newly constructed products can differ. This is intentional
representation migration accompanying an exact-completion fix. Regenerate
fixtures that pin those newly constructed expression bytes.
The wire schema is unchanged; cold import preserves the input expression, and
invalid or unproved children are still checked before cancellation. Distinct
nodes, conflicting handles and oversized audit graphs keep the existing path.
See the [R19 evidence and cost disclosure](../crates/hyperreal/PERFORMANCE.md#cold-domain-continuation-and-resource-audit-r19-2026-10-25).

## Cargo features

- **No dependency-named features.** Optional dependencies no longer create
  features of their own, so `stl_io`, `chrono`, `dxf`, `base64`,
  `serde_json`, `gerber-types`, `gerber_parser`, `ttf-parser`, `image`,
  `hypersdf`, `wasm-bindgen`, `js-sys`, `serde-wasm-bindgen` and `serde`
  are not valid `csgrs` features. Enable the capability feature instead, for
  example `stl-io` for `stl_io`, `image-io` for `image` and `sdf` or
  `metaballs` for `hypersdf`. The [README](../readme.md#features) lists them.
- **`image-io` enables no image codecs.** `image-io` depends on `image`
  without its default features. `curve::from_image` still takes an
  already-decoded `image::GrayImage`, and its regions are unchanged. CSGRS no
  longer turns on `image`'s format decoders and encoders or its `rayon`
  feature. A program that decodes or encodes image files, and relied on CSGRS
  to enable those formats, must request them in its own manifest, for example
  `image = { version = "0.25", default-features = false, features = ["png"] }`;
  Cargo unifies them with the CSGRS dependency. Without them, `image::open`
  and `image::load_from_memory` report an unsupported format at run time
  rather than failing to compile. DXF thumbnail reading under `dxf-io` is
  unaffected: the `dxf` crate requests the BMP codec it uses itself.

## Successive-operation research records (2026-10-25)

`SUCCESSIVE_OPERATIONS_RAW` now requires a new path and syncs each completed
row before the next step. The 18 TSV columns are unchanged; metadata comments
now contain an embedded build record and a final completion marker. Direct
Cargo builds identify themselves as unattributed. Use
[`scripts/successive-operations.py`](../scripts/successive-operations.py) for
source/compiler attribution, actual executable hashes and interruption status.
Historical timing records retain their original, weaker attribution.
