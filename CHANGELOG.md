# Changelog

## [0.23.0] - Unreleased

Changes since 0.20.1; versions 0.21 and 0.22 were not published. Caller-facing
migrations, including changed serialized output, are listed in
[docs/MIGRATIONS.md](docs/MIGRATIONS.md).

- A unit cube rotated successively by the independently imported binary64
  angles 13.1, 17.3, -13.1 and -17.3 degrees about one axis can now complete
  a certified union with the original cube and subtract back to empty.
- Mixed-sign full revolutions preserve each authored meridian for odd segment
  counts while regularizing overlapping material. Integer full-turn multiples
  share one coverage lattice; `curve::revolve_from` accepts an explicit start.
- Triangulation, extrusion, sweep, twist and revolution retain exact line
  members and native curve endpoints in mixed profiles. Curved interiors
  remain explicitly sampled; narrow exact members no longer collapse merely
  because another boundary is curved.
- Transforms of terminal-qualified closed curves preserve their operation
  receipt. A strict replay blocked by retained policy now reports an undecided
  predicate instead of invalid geometry. Native line-profile extrusion of these
  qualified loops remains an open consumer.

### Breaking

- The BSP-tree polygon model is replaced by the Hyper geometry stack. The
  `mesh`, `sketch` and `traits` modules, including `Mesh<S>`, `Sketch<S>`,
  `Polygon`, `Vertex`, `Plane`, the BSP trees and the `CSG` trait, are
  removed. Solids are now `csgrs::TriangleMesh` (Hypermesh indexed
  triangles) and planar geometry is `hypercurve::CurveRegion2`,
  `CurveString2` or `CurvePath2`. Operations are free functions in
  `csgrs::solid` and `csgrs::curve`, plus the fluent `solid::SolidExt` trait.
  Solid Booleans are evaluated by Hypermesh, which validates that inputs are
  finite, closed, oriented solids.
- `csgrs::Real` is now `hyperreal::Real` instead of an `f64`/`f32` alias.
  The `float_types` module (including `EPSILON`, `PI`, the unit constants
  and the `parry3d`/`rapier3d` re-exports) is removed, and `nalgebra` point,
  vector and matrix types are replaced by `hyperlattice` types over `Real`.
  There is no global tolerance: topology decisions use exact or certified
  predicates, and primitive floats appear only at adapter, I/O, FFI and
  WebAssembly boundaries.
- Constructors, transforms, bounds, sections and text return `Result` with a
  typed `ValidationError` (match `kind()`) instead of empty or partial
  geometry, and the solid Booleans (`SolidExt::try_union`, `try_difference`,
  `try_intersection`, `try_xor`) return `hypermesh::HypermeshError` failures.
  Bounds queries return `Result<Option<Aabb>, _>`, with `None` for empty
  geometry.
- Generic per-polygon and per-object metadata `S` is removed. Face metadata
  now lives in the optional `AttributedMesh<M>` sidecar (`attributed`
  feature), with one value per triangle and optional authored normals.
- File-format functions are free functions in public `csgrs::io::{stl, obj,
  ply, amf, dxf, svg}` modules, for example
  `io::stl::to_stl_binary(&mesh, name)`, instead of `Mesh` and `Sketch`
  methods, and report `csgrs::io::IoError`. SVG import and export go through
  Hypercurve.
- Planar operations moved to native curve APIs. `Sketch::offset` and
  `offset_rounded` become `CurveRegion2::offset` with an explicit
  `hypercurve::OffsetCornerStyle2`. Extrusions are `curve::try_extrude`,
  `try_extrude_vector`, `extrude_twisted`, `revolve` and `try_sweep`, and
  lofting is `solid::loft`. Bézier and B-spline input is
  `curve::bezier_region`, `bezier_path` and `bspline_path`. Text is
  `curve::truetype_text` and `hershey_strings`, with the Hershey fonts
  compiled into Hypercurve (`hypercurve::hershey::fonts`).
- Solid queries moved as well. `Mesh::slice(plane)` becomes
  `solid::slice_z(mesh, z)`, which returns filled sections, open strings and
  paths separately, and `flatten` returns a `CurveRegion2`. `contains_vertex`
  becomes `solid::contains_point`, `is_manifold` becomes
  `solid::is_closed_manifold`, and `Mesh::mass_properties` becomes
  `solid::exact_mass_properties`, computed exactly by Hyperphysics.
- The `f64`, `f32`, `parallel`, `chull-io`, `delaunay` and `earcut` features
  are removed. Convex hulls and Minkowski sums are always available, and
  triangulation uses Hypertri. The planar API needs the new default `curve`
  feature.
- Optional dependencies no longer create implicit Cargo features. Enable
  `stl-io`, `dxf-io`, `image-io`, `truetype-text` or `wasm` instead of
  `stl_io`, `dxf`, `image`, `ttf-parser` or `wasm-bindgen`. Features named
  after removed dependencies, such as `rayon`, `chull` and `parry3d-f64`, are
  also gone. See "Cargo features" in [docs/MIGRATIONS.md](docs/MIGRATIONS.md).

### Added

- Explicit predicate policy. `*_with_context` variants take a
  `GeometryContext` (`STRICT` or `APPROXIMATE_512`) and return a
  `GeometryOutcome<T>` with a `GeometryCertainty` receipt (`Certified` or
  `Approximate512Consumed`). The convenience entry points use `STRICT`.
  `GeometryOutcome::and_then` runs a dependent operation and combines both
  receipts, so a chain of curve operations, whose regions carry no inherited
  evidence, keeps its weakest certainty; `GeometryCertainty::combine`
  combines two receipts.
- glTF/GLB import and export (`gltf-io`), VRML 2.0 import (`vrml-io`) and
  Gerber RS-274X import and export (`gerber-io`), all enabled by default.
  OBJ import can attach face metadata (`io::obj::from_obj_attributed`).
- Solid operations `solid::minkowski_hull`, `minkowski_sum_with_options`
  (count limits and cooperative cancellation), `polyline_intersections`,
  `dihedral_angle` and `materialize_finite`; the bounded TPMS wall solids
  `gyroid_solid`, `schwarz_p_solid` and `schwarz_d_solid`; SDF meshing from
  retained `hypersdf::SdfExpr` values (`sdf_expr`); and `*_with_diagnostics`
  variants of SDF and metaball sampling.
- Hilbert-curve infill clipped to a filled region: `curve::hilbert_paths`
  keeps trim records, and `hilbert_strings` returns plain curve strings.
- `curve::extrude_twisted` with both end scales zero tapers the profile to
  the apex `(0, 0, height)`, as OpenSCAD's `linear_extrude(scale = 0)` does,
  instead of returning `InvalidArguments`. A holed profile gives a pyramid
  whose hole's pyramid meets it only at the apex.
- `csgrs::adapter` provides primitive-scalar wrappers (`ScalarMesh<A>` and
  `ScalarCurve<A>` over `RawReal`, `F32`, `F64` and `I128`) that convert only
  at ingress and egress.
- `solid::cache::with_policy`, `CachePolicy`, `usage` and `clear` set,
  inspect and clear the entry limits of the construction caches, which apply
  per thread.
- WebAssembly bindings (`wasm` feature) for mesh, curve, point, matrix and
  plane values, with certainty receipts on results. Failures of primitive
  constructors, transforms and bounds, among others, are thrown as a
  `GeometryError` with a machine-readable `code`. The C ABI is provided by
  the new `csgrs-ffi` crate, with C++, Go, Python and TypeScript wrappers in
  `bindings/`.

### Changed

- TrueType outlines are native lines and quadratic/cubic Bézier curves
  instead of curves flattened into eight segments, and use exactly `127/360`
  mm per point. `truetype-text` now enables `curve` and no longer depends on
  `ttf-parser-utils`.
- SDF and metaball meshing use Hypermesh's native-`Real` Surface Nets instead
  of `fast-surface-nets`. Metaball extraction samples the normalized field
  `(iso - I) / (iso + I)` of the exact inverse-square influence `I = r² / d²`,
  without the former distance cutoff or epsilon-smoothed denominator, and
  `MetaBall::influence` returns an error at a ball's center.
- Solid Booleans return only the vertices their result needs. Hypermesh
  merges coplanar pieces of each result and removes vertices inside one plane
  or on a straight crease between two, so results of successive Booleans stay
  proportional to the represented set: sixteen square holes through a block
  give 332 triangles instead of 1,600, and drill chains through sampled
  polygons keep constant coordinate size. Cuts that overlap earlier symbolic
  cuts build their new corners from the earlier results' face planes, so
  rotated-cut chains also keep constant coordinate size, and coplanar faces
  merged by repeated unions are triangulated from their oldest end, so a
  further union's face splitting no longer grows with the number of earlier
  ones. Each result stores equal symbolic coordinates as one expression, so
  later Booleans compare them structurally instead of proving them equal
  again; a further union of translated icosahedra costs about the same at
  the twelfth union as at the fourth. Represented sets and certainty are
  unchanged; triangle rows,
  counts and exported checksums change (see "Boolean output rows" in
  [docs/MIGRATIONS.md](docs/MIGRATIONS.md)).
- `image-io` depends on `image` without its default features, so CSGRS no
  longer builds image codecs, `rayon` or the AVIF encoder for raster tracing,
  which consumes an already-decoded `GrayImage`. A default build compiles 84
  packages instead of 146. `curve::from_image` is unchanged; programs that
  decode images enable the codecs they need in their own manifest (see
  "Cargo features" in [docs/MIGRATIONS.md](docs/MIGRATIONS.md)).
- `wasm32` builds compile `uuid`, which only the DXF and Gerber parsers
  need, only when `dxf-io` or `gerber-io` is enabled.
- `curve::extrude_twisted` names its invalid parameters: a zero height
  returns `ValidationError::ZeroParameter`, as a zero height does in
  `try_extrude`, and an end scale with one zero or a negative component
  returns `NonPositiveParameter`. Both returned `InvalidArguments`; the
  error kind is still `InvalidInput`.
- `bevymesh` targets Bevy 0.18 (was 0.16) through `solid::to_bevy_mesh`.
- Raises the minimum supported Rust version (MSRV) from 1.85.1 to 1.95.0.

### Fixed

- Exporting a reimported OBJ file reproduces it exactly, as `to_obj`
  documents. A coordinate whose binary64 value has a long decimal, such as a
  Steiner vertex with denominator 3 or 6, could be written one ulp away the
  second time, because hyperreal's float conversion depended on how the
  value was stored.
- AMF export writes each coordinate's binary64 value exactly, as PLY, OBJ
  and ASCII STL do. It used 17 fixed fractional digits, so a coordinate of
  magnitude below `5 * 10^-18` became zero and the 2^-70 cube was written
  with every vertex at the origin; coordinates below 0.1 could also change
  their binary64 value. The serialization change is listed in
  [docs/MIGRATIONS.md](docs/MIGRATIONS.md).
- The involute and cycloidal gear and rack constructors, marked "under
  construction" in 0.20.1, now produce corrected tooth profiles and validate
  their parameters.
- TrueType text now scales by the font's `units_per_em`, where 0.20.1 assumed
  2048. Glyphs advance by the font's horizontal advance and pair kerning
  rather than their bounding-box width, and newlines and tabs are laid out.
- Planar transforms by a `Matrix4` no longer silently drop the out-of-plane
  and homogeneous components. The homogeneous weight is divided out, and
  perspective or out-of-plane images return `UnsupportedTransform`.
- `curve::try_sweep` completes under `STRICT` along non-planar sampled paths,
  such as the helix of the `extrude` example, which formerly failed with
  `InvalidArguments`. A sweep reports its true outcome: an undecided
  predicate returns `ValidationError::PredicateUndecided` naming the sweep
  step, another scalar failure returns `ValidationError::Scalar`, and only an
  invalid path returns `InvalidArguments`. Paths that already succeeded keep
  their exact output. Coordinates along a general path are nested square
  roots, so later strict operations on them can be undecidable.
- In WebAssembly, `CurveRegionJs.sweep`, `revolve` and `extrudeTwisted` throw
  a `GeometryError` with a `code`, as `extrude` does, instead of a bare
  message string.
- `curve::try_sweep`, `extrude_twisted` and `revolve` use an exact line
  profile's own coordinates, as `try_extrude` does, instead of rounding them
  to binary64. Widths of 1/3, sqrt(2) and 2^-80, holes 2^-80 apart after a
  2^60 translation, and the authored meridian of a partial revolution stay
  exact. These results no longer report sampling or coordinate projection.
  Previously a 2^-80 sliver swept into a certified closed surface of zero
  volume, and the separated holes failed to sweep. A partial revolution of a
  profile at negative radius, which failed, now completes. Profiles with arc
  or curve boundaries are still sampled, at the chord error now published as
  `curve::SAMPLED_PROFILE_CHORD_ERROR`, and their results still say so.
- A full turn of `curve::revolve` whose profile has material on both sides
  of the axis revolves the profile's fold: the material at nonnegative
  radius united, exactly, with the mirror image of the material at
  nonpositive radius, which a full turn sweeps through the same circles.
  Opposite loops were revolved separately, so where their mirror images
  overlapped the result was a closed mesh that counted the overlap twice:
  `[1, 3] x [0, 2]` with its mirror image `[-3, -1] x [0, 2]` in three
  segments had twice the volume of one loop, and in four segments, where the
  copies coincide, failed with a `Geometry` error. A loop crossing the axis,
  or a hole straddling it, returned `InvalidArguments` and now revolves. A
  partial turn still requires each loop to lie on one side of the axis;
  beyond a half turn, opposite loops whose mirror images overlap still give
  a closed mesh with winding number two in the overlap, as now documented.
- Importing a planar polygon face no longer requires deciding which normal
  component is largest. Any component the policy proves nonzero gives the
  projection used to triangulate it. A VRML face turned about an axis in its
  plane by pi/4 written to 200 decimal digits failed under `STRICT` with
  "dominant projection axis is indeterminate", and under `APPROXIMATE_512`
  consumed the approximate terminal; it now imports certified under both.
  Faces whose largest component strict comparison decides keep their
  triangles, which covers every OBJ and DXF face, since their coordinates
  are rational.

- Strict Booleans and convex hulls of sampled spheres complete. The
  `boolean_operations` and `convex_hull` examples stopped under `STRICT` on
  exact zeros of the 32 x 16 sphere's trigonometric coordinates; hyperreal now
  proves them. Odd sample counts such as 17 x 9 and further operations on the
  results also decide.
- Rotations by angles such as 0.0001 or 12.345 degrees, whose sine and cosine
  lie beyond the cyclotomic proof's order limit, decide under `STRICT` where
  they need only `sin^2 + cos^2 = 1`: the volume of the rotated solid, a
  rotation followed by its inverse, and a cube's union, intersection and
  difference with itself rotated by 0.0001 degree about a corner axis. These
  ended `PredicateUndecided`. Booleans of a sampled cylinder with its copy
  rotated by 0.0001 degree about a coordinate axis also complete, because
  Boolean arrangements now take incidences their points have by construction
  instead of proving the zeros; rotations about a skew axis still end
  `PredicateUndecided`
  ([hypermesh future work](crates/hypermesh/FUTURE_WORK.md#near-coincident-rotations-beyond-the-cyclotomic-domain)).

### Removed

- Rapier and Parry integration: `Mesh::to_rapier_shape`, `to_trimesh` and
  `to_rigid_body`.
- Mesh smoothing and quality analysis: `laplacian_smooth`, `taubin_smooth`,
  `adaptive_refine`, `remove_poor_triangles`, `analyze_triangle_quality` and
  `compute_mesh_quality`.

For 0.20.1 and earlier, see [crates.io](https://crates.io/crates/csgrs/versions).
