# Migrations

This document lists the CSGRS changes that need caller action or that change
represented geometry or serialized artifacts. The [README](../readme.md) and
the API documentation describe current behavior only. A migration is a
deliberate contract change (P27), not a performance-only change.

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
- **WASM curve meshes.** `CurveRegionJs.sweep`, `revolve` and
  `extrudeTwisted` throw a `GeometryError` with a `code`, as `extrude`
  does, instead of a bare message string for native failures.

## Serialized output and geometry

These changes alter represented sets, coordinates, file lengths or checksums.

- **PLY decimal spelling.** Positions and normals used a fixed 17 fractional
  digits, which could round small nonzero values to zero. Each binary64 value
  is now spelled exactly, with up to 1074 fractional digits where needed.
  Already exact spellings, headers and face rows keep their bytes. Affected
  files change length and checksum, and some become larger.
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
  reports the least of them as its `hypermesh::TriangleSource`. Regenerate
  pinned Boolean rows, counts and checksums.

## Cargo features

- **No dependency-named features.** Optional dependencies no longer create
  features of their own, so `stl_io`, `chrono`, `dxf`, `base64`,
  `serde_json`, `gerber-types`, `gerber_parser`, `ttf-parser`, `image`,
  `hypersdf`, `wasm-bindgen`, `js-sys`, `serde-wasm-bindgen` and `serde`
  are not valid `csgrs` features. Enable the capability feature instead, for
  example `stl-io` for `stl_io`, `image-io` for `image` and `sdf` or
  `metaballs` for `hypersdf`. The [README](../readme.md#features) lists them.
