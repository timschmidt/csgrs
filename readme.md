# CSGRS

CSGRS is the constructive-solid-geometry grammar for the Hyper geometry
workspace. It provides an OpenSCAD-like vocabulary for primitives, Boolean
composition, transforms, extrusion, revolution, sweep, loft, conversion, and
CAD interchange without introducing another geometry kernel.

- [Hypercurve](https://github.com/timschmidt/hypercurve) owns curves, paths, filled regions, and
  certified planar operations.
- [Hypermesh](https://github.com/timschmidt/hypermesh) owns indexed triangle geometry, certified
  solid Booleans, topology facts, and mesh queries.
- [Hypertri](https://github.com/timschmidt/hypertri) owns triangulation.
- [Hyperphysics](https://github.com/timschmidt/hyperphysics) owns physical-property analysis.
- CSGRS supplies the modeling language that composes those native types and
  connects them to file formats and application boundaries.

Core coordinates use `hyperreal::Real`. Primitive floats occur only at
explicit rendering, serialization, adapter, FFI, and WebAssembly boundaries.

![A composed CSG model](docs/csg.png)

## Primary types

CSGRS operates directly on its dependencies' native geometry:

```text
CurveRegion2 ── extrude / revolve / sweep ──► TriangleMesh
     │                                             │
     ├─ CurveString2 and CurvePath2 for open work  ├─ Hypermesh Booleans
     └─ Hypercurve topology and Booleans           └─ queries and interchange
```

- `hypercurve::CurveRegion2` is filled planar material, including holes.
- `hypercurve::CurveString2` is an open or closed curve string without implied
  fill.
- `hypercurve::CurvePath2` preserves higher-order path topology.
- `hypermesh::TriangleMesh`, re-exported as `csgrs::TriangleMesh`, is reusable
  indexed triangle geometry and the input/output type for solid modeling.
- `csgrs::AttributedMesh<M>` is an optional, feature-gated boundary sidecar
  with one metadata value per triangle and optional authored normals. Modeling
  operations still consume its `TriangleMesh` geometry.
- `Real` is CSGRS's re-export of the exact-aware Hyperreal scalar.

Use Hypercurve or Hypermesh directly for kernel-level work. Use CSGRS when the
task is to describe a model, cross between curve and solid representations, or
exchange geometry with another format.

## Quick start

Install Rust with [rustup](https://rustup.rs/), then:

```sh
cargo new drilled-cube
cd drilled-cube
cargo add csgrs
```

Replace `src/main.rs` with:

The equivalent dependency entry is:

```toml
[dependencies]
csgrs = "0.23.0"
```

<!-- quickstart:start -->
```rust
use csgrs::{
    GeometryContext, Real, curve,
    io::stl::to_stl_binary,
    solid::{self, SolidExt},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let body = solid::cube(Real::from(12))?;
    let opening = curve::try_extrude(
        &curve::square(Real::from(4))?,
        Real::from(16),
        &GeometryContext::STRICT,
    )?
    .into_value()
    .translated(Real::from(4), Real::from(4), Real::from(-2))?;

    let part = body.try_difference(&opening)?;
    std::fs::write("drilled_cube.stl", to_stl_binary(&part, "drilled_cube")?)?;
    Ok(())
}
```
<!-- quickstart:end -->

Run it with `cargo run`. It writes `drilled_cube.stl`. The same program is
included as `examples/basic.rs` and runs with:

```sh
cargo run --example basic
```

## Modeling API

The following is the useful public modeling surface. Generated API
documentation contains complete signatures, feature conditions, and errors.

### Filled regions and open curves

Functions in `csgrs::curve` return native Hypercurve types.

Rectangle, square, circle, ellipse, regular polygon, right triangle, polygon,
and rounded-rectangle constructors now return `Result<CurveRegion2,
ValidationError>`. Their policy-taking variants and the compound constructors
return `Result<GeometryOutcome<CurveRegion2>, ValidationError>`. This API
migration replaces empty-on-error behavior: propagate `?` or handle the typed
error explicitly. Negative dimensions, malformed rings, unresolved predicates,
and checked capacity/allocation failures remain failures through scalar
adapters and bindings.

Nonnegative primitive dimensions may produce canonical empty geometry when a
dimension is certified zero, after all arguments are checked. Sampled circles
and ellipses still require at least three segments; polygons require a valid
ring with at least three vertices. Use `curve::empty()` for explicit absence.
Compound constructors retain their documented positive-dimension/range
restrictions. A zero rounded-corner segment count selects the sharp rectangle;
corner radii are clamped to half the smaller dimension. Sampling still defines
a polygon and provides no analytic-surface fidelity bound.

Rounded rectangles retain every nonzero straight side between their tangent
endpoints. A positive `corner_segments = n` gives `n` equal-angle chords per
quarter-circle and at most `4 * (n + 1)` vertices; only proven coincident
endpoints are omitted. This corrects an earlier sampler that omitted distinct
tangent endpoints, cutting away part of the intended polygon. For example,
`rounded_rectangle(4, 2, 1/2, 1)` has area `15/2`, rather than `4`.
Existing rounded-rectangle and Gerber obround outputs may therefore gain
vertices and change exported bytes or previews. Regenerate derived artifacts;
this is a geometry correction, not a performance-only change.

The ring cache retains only successful strict constructions, keyed by owned
exact coordinate rows, in a thread-local FIFO of at most 64 entries. Approximate
requests may reuse those proofs but do not populate the cache. Failures are not
cached. The limit counts entries, not heap bytes; scalar growth, secondary
kernel allocation, total work and cancellation remain unbounded internally.
The sampled `arrow`, `trapezoid`, `star`, `teardrop`, `egg`, `squircle`,
`pie_slice`, `heart`, and `supershape` constructors also return `Result`; each
has a `_with_context` variant returning `GeometryOutcome`. Their existing
positive-dimension restrictions and sampling formulas remain. An equal-width
arrow now omits its duplicate shoulders and retains its shaft and triangular
head. `teardrop_cylinder` propagates profile errors and joins construction and
extrusion certainty. WASM star, teardrop, egg and heart constructors throw
`GeometryError` with a diagnostic `code` instead of publishing failed profiles.

Stars require at least three tips and `outer_radius > inner_radius > 0`;
teardrops require `length > width / 2` and at least two semicircle chords.
Eggs and squircles require at least three samples; hearts require eight.
Sectors require a positive radius, a nonzero signed sweep of at most 360 degrees,
and at least one chord. A full turn keeps the existing minimum of three circle
samples. A successfully regularized zero-area sample, such as a one-chord
semicircle, is canonical empty geometry. Supershapes require positive axes,
nonzero `n1`, at least three samples and defined powers at every sample.
Invalid input, undecided predicates, scalar-domain errors and capacity failures
remain distinct from successful empty results. Native topology failures retain
their cause. These sampled polygons carry no analytic-fidelity bound.

Gear, text, airfoil and other legacy convenience operations still need their
own typed-failure migration. End-to-end provenance remains a separate open
contract from these operation-local certainty receipts.

Rendered filled constructors:

| Preview | Function |
|---|---|
| <img src="docs/square.png" width="144" alt="Filled square with visible edges and vertices"/> | `curve::square(width: Real) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/rectangle.png" width="144" alt="Filled rectangle with visible edges and vertices"/> | `curve::rectangle(width: Real, length: Real) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/circle.png" width="144" alt="Filled sampled circle with visible edges and vertices"/> | `curve::circle(radius: Real, segments: usize) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/right_triangle.png" width="144" alt="Filled right triangle with visible edges and vertices"/> | `curve::right_triangle(width: Real, height: Real) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/polygon.png" width="144" alt="Filled polygon with visible edges and vertices"/> | `curve::polygon(points: &[[Real; 2]]) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/rounded_rectangle.png" width="144" alt="Filled rounded rectangle with visible edges and vertices"/> | `curve::rounded_rectangle(width: Real, height: Real, corner_radius: Real, corner_segments: usize) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/ellipse.png" width="144" alt="Filled sampled ellipse with visible edges and vertices"/> | `curve::ellipse(width: Real, height: Real, segments: usize) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/regular_ngon.png" width="144" alt="Filled regular polygon with visible edges and vertices"/> | `curve::regular_ngon(sides: usize, radius: Real) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/curve_arrow.png" width="144" alt="Filled planar arrow with visible edges and vertices"/> | `curve::arrow(shaft_length: Real, shaft_width: Real, head_length: Real, head_width: Real) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/trapezoid.png" width="144" alt="Filled trapezoid with visible edges and vertices"/> | `curve::trapezoid(top_width: Real, bottom_width: Real, height: Real, top_offset: Real) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/star.png" width="144" alt="Filled star with visible edges and vertices"/> | `curve::star(num_points: usize, outer_radius: Real, inner_radius: Real) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/teardrop.png" width="144" alt="Filled teardrop with visible edges and vertices"/> | `curve::teardrop(width: Real, length: Real, segments: usize) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/curve_egg.png" width="144" alt="Filled egg with visible edges and vertices"/> | `curve::egg(width: Real, length: Real, segments: usize) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/squircle.png" width="144" alt="Filled squircle with visible edges and vertices"/> | `curve::squircle(width: Real, height: Real, segments: usize) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/keyhole.png" width="144" alt="Filled keyhole with visible edges and vertices"/> | `curve::keyhole(circle_radius: Real, handle_width: Real, handle_height: Real, segments: usize, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, ValidationError>` |
| <img src="docs/reuleaux.png" width="144" alt="Filled Reuleaux polygon with visible exact curve edges and vertices"/> | `curve::reuleaux(sides: usize, diameter: Real, circle_segments: usize, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, ValidationError>` |
| <img src="docs/heart.png" width="144" alt="Filled heart with visible edges and vertices"/> | `curve::heart(width: Real, height: Real, segments: usize) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/ring.png" width="144" alt="Filled annulus with visible exact curve edges and vertices"/> | `curve::ring(inner_diameter: Real, thickness: Real, segments: usize, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, ValidationError>` |
| <img src="docs/pie_slice.png" width="144" alt="Filled circular sector with visible edges and vertices"/> | `curve::pie_slice(radius: Real, start_angle_deg: Real, end_angle_deg: Real, segments: usize) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/crescent.png" width="144" alt="Filled crescent with visible exact curve edges and vertices"/> | `curve::crescent(outer_radius: Real, inner_radius: Real, offset: Real, segments: usize, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, ValidationError>` |
| <img src="docs/supershape.png" width="144" alt="Filled superformula shape with visible edges and vertices"/> | `curve::supershape(a: Real, b: Real, m: Real, n1: Real, n2: Real, n3: Real, segments: usize) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/circle_with_keyway.png" width="144" alt="Circle with a keyway and visible exact curve edges"/> | `curve::circle_with_keyway(radius: Real, segments: usize, key_width: Real, key_depth: Real, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, ValidationError>` |
| <img src="docs/circle_with_flat.png" width="144" alt="Circle with one flat and visible exact curve edges"/> | `curve::circle_with_flat(radius: Real, segments: usize, flat_distance: Real, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, ValidationError>` |
| <img src="docs/circle_with_two_flats.png" width="144" alt="Circle with two flats and visible exact curve edges"/> | `curve::circle_with_two_flats(radius: Real, segments: usize, flat_distance: Real, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, ValidationError>` |
| <img src="docs/involute_gear.png" width="144" alt="Filled involute gear profile with visible edges and vertices"/> | `curve::involute_gear(module: Real, teeth: usize, pressure_angle_degrees: Real, clearance: Real, backlash: Real, segments_per_flank: usize) -> CurveRegion2` |
| <img src="docs/cycloidal_gear.png" width="144" alt="Filled cycloidal gear profile with visible edges and vertices"/> | `curve::cycloidal_gear(module: Real, teeth: usize, generating_radius: Real, clearance: Real, segments_per_flank: usize) -> CurveRegion2` |
| <img src="docs/involute_rack.png" width="144" alt="Filled involute rack profile with visible edges and vertices"/> | `curve::involute_rack(module: Real, teeth: usize, pressure_angle_degrees: Real, clearance: Real, backlash: Real) -> CurveRegion2` |
| <img src="docs/cycloidal_rack.png" width="144" alt="Filled cycloidal rack profile with visible edges and vertices"/> | `curve::cycloidal_rack(module: Real, teeth: usize, clearance: Real, segments_per_flank: usize) -> CurveRegion2` |
| <img src="docs/airfoil_naca4.png" width="144" alt="Filled NACA four-digit airfoil with visible edges and vertices"/> | `curve::airfoil_naca4(max_camber: Real, camber_position: Real, thickness: Real, chord: Real, samples: usize) -> CurveRegion2` |
| <img src="docs/bezier.png" width="144" alt="Filled exact Bezier boundary with visible edge and vertex"/> | `curve::bezier_region(control: &[[Real; 2]], display_segments: usize) -> CurveRegion2` |
| <img src="docs/metaballs_2d.png" width="144" alt="Filled planar metaball contours with visible edges and vertices"/> | `curve::metaballs(balls: &[(Point2, Real)], resolution: (usize, usize), iso_value: Real, padding: Real) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/from_image.png" width="144" alt="Filled raster-traced region with visible edges and vertices"/> | `curve::from_image(image: &GrayImage, threshold: u8) -> Result<CurveRegion2, RasterTraceError>` |
| <img src="docs/truetype.png" width="144" alt="Filled TrueType glyph regions with visible exact edges and vertices"/> | `curve::truetype_text(text: &str, font_data: &[u8], scale: Real) -> CurveRegion2` |

`curve::empty()` is intentionally image-free because it contains no visible
geometry. `polygon_points` constructs the same polygon family from native
Hypercurve `Point2` values.

Open constructors retain their native path/string topology:

| Preview | Function |
|---|---|
| <img src="docs/bezier_path.png" width="144" alt="Exact open Bezier path with visible edge and authored vertices"/> | `curve::bezier_path(control: &[[Real; 2]], display_segments: usize) -> Option<CurvePath2>` |
| <img src="docs/bspline_path.png" width="144" alt="Exact open B-spline path with visible edge and authored vertices"/> | `curve::bspline_path(control: &[[Real; 2]], degree: usize, display_segments_per_span: usize) -> Option<CurvePath2>` |
| <img src="docs/hilbert_strings.png" width="144" alt="Open Hilbert curve strings with visible edges and vertices"/> | `curve::hilbert_strings(boundary: &CurveRegion2, order: usize, padding: Real) -> Vec<CurveString2>` |
| <img src="docs/hershey_strings.png" width="144" alt="Open Hershey text curve strings with visible edges and vertices"/> | `curve::hershey_strings(text: &str, font: &hypercurve::hershey::Font<'_>, size: Real) -> Vec<CurveString2>` |

The Hershey catalog is compiled into Hypercurve and exposed as
`hypercurve::hershey::fonts`; no font files or runtime parser are required.

Planar operations:

- `CurveRegion2::boolean_region(&other, BooleanOp::{Union, Difference, Intersection, Xor})`
  from Hypercurve, or `boolean_regions` for all four sharing one arrangement;
- `offset` and `offset_rounded`;
- `transformed`, `translated`, `rotated`, and `scaled`;
- `contains_xy`, `bounding_box`, `try_finite_profiles`, and `try_triangulate`.

`try_finite_profiles` is an explicit display/interchange projection. It is not a
replacement for the native curve topology.

The four `curve::transformed`/`translated`/`rotated`/`scaled` functions now
return `Result<CurveRegion2, ValidationError>`; failures no longer become empty
regions. Their `try_` aliases and context variants use the same error type,
retaining native causes in `ValidationError::ExactCurve`. Context variants
return operation-local certainty. `contains_xy` now returns
`Result<Option<bool>, ValidationError>`: `Some(true)`/`Some(false)` classify
inside/outside, `Ok(None)` means a proved boundary, and an undecided query is
an error. Canonical empty regions classify outside and remain composable under
valid transforms. Invalid transform arguments are checked even for empty input.

A `Matrix4` applied to a planar region must restrict to a nonsingular affine
map on `z = 0`, keep that plane in `z = 0`, and have constant nonzero homogeneous
weight. The weight is divided out, including negative or nonunit values.
The third column has no effect on an XY input. Perspective and out-of-plane
images return `UnsupportedTransform`; a constant zero weight with a nonzero
coordinate image returns `UnboundedTransform`. An all-zero homogeneous image
is invalid input. Unproved constraints remain undecided, or explicitly
qualified when the selected policy consumes approximate evidence. This corrects
the old behavior that ignored these components and could publish a different set.

Scalar adapters and native bindings share this admission path. They validate
all three scalar inputs: Z translation must preserve the plane; Z scale has no
effect on XY geometry; X/Y/Z rotations are applied in that order and their
combined image must remain planar. An X half-turn is therefore a reflection,
while an X quarter-turn is unsupported. WASM transforms and strict point queries
throw `GeometryError` with a diagnostic `code`; contextual queries retain their
explicit boundary/uncertain status. These receipts do not establish end-to-end
provenance or an internal work, retained-memory, or cancellation bound.

### Curves to solids

These functions consume a `CurveRegion2` and return a native `TriangleMesh`:

| Preview | Function |
|---|---|
| <img src="docs/extrude.png" width="144" alt="Planar star extruded into a solid with visible faces, edges, and vertices"/> | `curve::try_extrude(region: &CurveRegion2, height: Real, context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/extrude_vector.png" width="144" alt="Planar star vector-extruded into a solid with visible faces, edges, and vertices"/> | `curve::try_extrude_vector(region: &CurveRegion2, direction: Vector3, context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/extrude_twisted.png" width="144" alt="Planar star twist-extruded into a solid with visible faces, edges, and vertices"/> | `curve::extrude_twisted(input: &CurveRegion2, height: Real, twist_degrees: Real, end_scale: [Real; 2], slices: usize, context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/revolve.png" width="144" alt="Planar region revolved into a solid with visible faces, edges, and vertices"/> | `curve::revolve(region: &CurveRegion2, angle_degrees: Real, segments: usize, context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/sweep.png" width="144" alt="Planar region swept along a path into a solid with visible faces, edges, and vertices"/> | `curve::try_sweep(region: &CurveRegion2, path: &[Point3], context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/loft.png" width="144" alt="Corresponding planar sections lofted into a solid with visible faces, edges, and vertices"/> | `solid::loft(sections: &[Vec<Point3>]) -> Result<TriangleMesh, ValidationError>` |

### Solid construction

Functions in `csgrs::solid` return `TriangleMesh`:

| Preview | Function |
|---|---|
| <img src="docs/cube.png" width="144" alt="Cube with visible faces, edges, and vertices"/> | `solid::cube(width: Real) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/cuboid.png" width="144" alt="Cuboid with visible faces, edges, and vertices"/> | `solid::cuboid(width: Real, length: Real, height: Real) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/sphere.png" width="144" alt="Sampled sphere with visible faces, edges, and vertices"/> | `solid::sphere(radius: Real, segments: usize, stacks: usize) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/cylinder.png" width="144" alt="Sampled cylinder with visible faces, edges, and vertices"/> | `solid::cylinder(radius: Real, height: Real, segments: usize) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/ellipsoid.png" width="144" alt="Sampled ellipsoid with visible faces, edges, and vertices"/> | `solid::ellipsoid(radius_x: Real, radius_y: Real, radius_z: Real, segments: usize, stacks: usize) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/frustum.png" width="144" alt="Conical frustum with visible faces, edges, and vertices"/> | `solid::frustum(radius_bottom: Real, radius_top: Real, height: Real, segments: usize) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/frustum_between.png" width="144" alt="Conical frustum between two points with visible faces, edges, and vertices"/> | `solid::frustum_between(start: Point3, end: Point3, start_radius: Real, end_radius: Real, segments: usize) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/polyhedron.png" width="144" alt="Indexed polyhedron with visible faces, edges, and vertices"/> | `solid::polyhedron(points: &[[Real; 3]], faces: &[&[usize]]) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/octahedron.png" width="144" alt="Regular octahedron with visible faces, edges, and vertices"/> | `solid::octahedron(radius: Real) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/icosahedron.png" width="144" alt="Regular icosahedron with visible faces, edges, and vertices"/> | `solid::icosahedron(radius: Real) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/mesh_arrow.png" width="144" alt="Three-dimensional arrow with visible faces, edges, and vertices"/> | `solid::arrow(start: Point3, direction: Vector3, segments: usize, mirrored: bool) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/torus.png" width="144" alt="Sampled torus with visible faces, edges, and vertices"/> | `solid::torus(major_radius: Real, minor_radius: Real, major_segments: usize, minor_segments: usize) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/teardrop_cylinder.png" width="144" alt="Extruded teardrop solid with visible faces, edges, and vertices"/> | `solid::teardrop_cylinder(width: Real, length: Real, height: Real, shape_segments: usize, context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/spur_gear_involute.png" width="144" alt="Involute spur gear with visible faces, edges, and vertices"/> | `solid::spur_gear_involute(module: Real, teeth: usize, pressure_angle_degrees: Real, clearance: Real, backlash: Real, segments_per_flank: usize, thickness: Real, context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/spur_gear_cycloid.png" width="144" alt="Cycloidal spur gear with visible faces, edges, and vertices"/> | `solid::spur_gear_cycloid(module: Real, teeth: usize, generating_radius: Real, clearance: Real, segments_per_flank: usize, thickness: Real, context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/helical_involute_gear.png" width="144" alt="Helical involute gear with visible faces, edges, and vertices"/> | `solid::helical_involute_gear(module: Real, teeth: usize, pressure_angle_degrees: Real, clearance: Real, backlash: Real, segments_per_flank: usize, thickness: Real, helix_angle_degrees: Real, slices: usize, context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/metaballs_3d.png" width="144" alt="Sampled metaball solid with visible faces, edges, and vertices"/> | `solid::metaballs(balls: &[MetaBall], resolution: (usize, usize, usize), iso_value: Real, padding: Real) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/sdf.png" width="144" alt="Sampled signed-distance-field solid with visible faces, edges, and vertices"/> | `solid::sdf(field: impl Fn(&Point3) -> Real, resolution: (usize, usize, usize), min: Point3, max: Point3, iso_value: Real) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/gyroid.png" width="144" alt="Bounded gyroid solid with visible faces, edges, and vertices"/> | `solid::gyroid_solid(bounds: &TriangleMesh, resolution: usize, scale: Real, iso_value: Real, wall_thickness: Real) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/schwarz_p.png" width="144" alt="Bounded Schwarz-P solid with visible faces, edges, and vertices"/> | `solid::schwarz_p_solid(bounds: &TriangleMesh, resolution: usize, scale: Real, iso_value: Real, wall_thickness: Real) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/schwarz_d.png" width="144" alt="Bounded Schwarz-D solid with visible faces, edges, and vertices"/> | `solid::schwarz_d_solid(bounds: &TriangleMesh, resolution: usize, scale: Real, iso_value: Real, wall_thickness: Real) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/gyroid_surface.png" width="144" alt="Bounded gyroid level surface with visible faces, edges, and vertices"/> | `solid::gyroid(bounds: &TriangleMesh, resolution: usize, scale: Real, iso_value: Real) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/schwarz_p_surface.png" width="144" alt="Bounded Schwarz-P level surface with visible faces, edges, and vertices"/> | `solid::schwarz_p(bounds: &TriangleMesh, resolution: usize, scale: Real, iso_value: Real) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/schwarz_d_surface.png" width="144" alt="Bounded Schwarz-D level surface with visible faces, edges, and vertices"/> | `solid::schwarz_d(bounds: &TriangleMesh, resolution: usize, scale: Real, iso_value: Real) -> Result<TriangleMesh, ValidationError>` |

`solid::empty()` is intentionally image-free. Diagnostic variants
`metaballs_with_diagnostics` and `sdf_with_diagnostics` produce the same shape
families while returning sampling reports. `sdf_expr` and
`sdf_expr_with_diagnostics` accept retained `hypersdf::SdfExpr` values and
produce the same SDF shape family. All four SDF constructors now return `Result`:
use `?` to propagate failures before passing a mesh downstream. Successful
empty level surfaces remain `Ok`; invalid expression domains, undecided
construction or extraction, and capacity exhaustion are errors with retained
native causes. `ValidationError::SdfSample` also records the failing grid index.
Grid dimensions below two are rejected instead of silently increased. Diagnostic
queries also fail when their requested sign or extrema cannot be established.

`*_with_context` variants retain the certainty of the decisions consumed under
the selected policy, including `sdf_expr_with_diagnostics_and_context`.
Expression construction remains strict under either meshing policy. Diagnostic
previews remain lossy and preserve only strict sign evidence; they may withhold
samples accepted approximately by the mesh operation. The display backend also
uses a chunk boundary convention that can omit final-layer faces. The modeling
path now uses Hypermesh's `surface_nets_standalone`, which includes all faces
whose four incident cells are present in the grid. Hypermesh's existing
`surface_nets` keeps its documented chunk convention. The modeling output
correction is a contract migration, not a performance-only change.
Numerical certification
of the sampled level surface establishes neither closed topology nor fidelity
to the continuous field. Check required topology separately and retain the
source expression when that distinction matters.

The SDF sampling path checks primary buffer capacities and reservations. Scalar
expression growth, secondary extraction/preview allocations, total work and
cancellation do not yet have internal bounds. The 3D metaball constructors now share this native sampler and return `Result`.
`MetaBall` reexports Hypersdf's retained `Metaball`; its `influence` returns a
fallible exact inverse-square value, with no distant cutoff and a typed pole
error at the center. `metaballs_with_context` and
`metaballs_with_diagnostics_and_context` retain the certainty actually consumed.
Radii and iso values must be positive, padding nonnegative, and every requested
grid dimension at least two, including for empty inputs.

This is an explicit geometry migration: extraction samples
`F = (iso - I)/(iso + I)`, where `I = sum(r² / distance²)`, and uses the exact
continuous limit `F = -1` at centers. The normalization preserves the intended
level set while making finite interpolation well-defined, including for
coincident centers. The former cutoff, finite singularity sentinel and hidden
`f32` vertex construction are removed from 3D modeling. Grid locations retain
the existing conservative bounds and one-cell exterior border. Diagnostic sign
counts and extrema describe F, with negative meaning inside. Output coordinates
and serialized artifacts change; this is not a performance-only change or an
analytic-surface fidelity guarantee.

The 2D `curve::metaballs` now returns `Result` and samples the same normalized
native field, removing its cutoff, center sentinel, and empty-on-error fallback.
`curve::metaballs_with_context` also retains predicate certainty. It preserves
the existing core grid within the conservative bounds, without the 3D exterior
border. Hypercurve owns exact marching-squares edge crossings and material/hole
arrangement. Ambiguous cells retain the center-sample-sum sign rule (zero is
inside). Chords define a deterministic polygonal proposal, not an exact bilinear
level curve or certified approximation of the continuous field. Boundary samples
must be strictly exterior; unresolved decisions and unsupported arrangements
remain typed failures. This migration changes 2D coordinates and serialized
artifacts as well. Primary grid buffers are checked; total work, secondary
allocations, coefficient growth and cancellation remain internally unbounded.

Triangulation and linear extrusion now use native certified line profiles and
Hypertri without finite-coordinate projection, including line-image Bézier
boundaries, holes, rational coordinates, and coordinates beyond `f64` range.
Their exact vertices and output artifacts can therefore differ from the former
quantized output. Other profile representations retain the existing sampled
finite path; source-curve fidelity and end-to-end provenance for that path remain
open targets. Rust extrusion adapters retain native errors, C-family bindings
retain status categories, and WASM extrusion/triangulation expose `GeometryError`
codes. Required topology is checked separately from numerical certainty.

Sampling arguments such as segments, stacks, slices, and resolution are
explicit tessellation choices; CSGRS has no global tolerance setting.

### Solid operations and queries

`solid::SolidExt` supplies fluent native operations:

- `try_union`, `try_difference`, `try_intersection`, and `try_xor` use
  `GeometryContext::STRICT`;
- `try_union_with_context`, `try_difference_with_context`,
  `try_intersection_with_context`, and `try_xor_with_context` accept
  `&GeometryContext` and return `HypermeshResult<GeometryOutcome<TriangleMesh>>`;
- `transformed` and `translated`;
- `exact_bounds`;
- `visit_native_triangles`.

The corresponding free functions and additional operations are:

- Booleans and transforms: `boolean`, `boolean_with_context`, `transform`,
  `rotate`, `scale`, `mirror`, `inverse`, `translation`, and `translation_to`;
- positioning and repetition: `bounding_box`, `center`, `float`, `merge`,
  `distribute_linear`, `distribute_grid`, and `distribute_arc`;
- topology/refinement: `subdivide`, `renormalized`, `materialize_finite`,
  `is_closed_manifold`, `convex_hull`, and `minkowski_sum`;
- geometric queries: `contains_point`, `ray_intersections`,
  `polyline_intersections`, and `dihedral_angle`;
- projections: `flatten` and `slice_z`;
- analysis: `exact_mass_properties`;
- application conversion: `to_bevy_mesh` when `bevymesh` is enabled;
- surface forms of implicit fields: `gyroid`, `schwarz_p`, and `schwarz_d`.

Hypermesh retains reusable bounds, topology, transform, convexity, and
Boolean facts on `TriangleMesh`. Cloning a mesh shares its immutable geometry
and retained facts.

`solid::empty()` is a valid operand for every modeling Boolean. Union can
accumulate from empty, and a subtraction that removes everything can feed the
next operation. Canonical empty geometry has neither positions nor triangles;
a mesh missing just one of those arrays remains an input error. The other
operand is still validated and regularized under the selected policy, even
when the result is necessarily empty. Conversion from a shared Hypermesh
Boolean batch now gives empty rows zero positions as well as zero triangles;
nonempty rows continue sharing the batch's position allocation.

`ScalarMesh::{union, difference, intersection, xor}` and their context variants
now preserve native Boolean failures as
`AdapterError::Construction(ValidationError::Mesh(...))`, replacing text-only
`AdapterError::Validation`. Callers that match errors should inspect
`ValidationError::kind()` or follow `std::error::Error::source`; the native
diagnostic text is retained. In the C ABI and its C++/Python/Go wrappers,
undecided mesh Booleans now report the existing `PREDICATE_UNDECIDED` (5) status
instead of generic `VALIDATION_FAILED` (4). Invalid input still reports (4),
including malformed operands paired with empty geometry. No failure publishes
a successful model.

Primitive constructors (`cube`, `cuboid`, `sphere`, `cylinder`, `ellipsoid`,
`frustum`, `frustum_between`, `octahedron`, `icosahedron`, `arrow`, and `torus`)
now return `Result<TriangleMesh, ValidationError>`. This is an API migration:
propagate construction errors with `?` before transforming or composing a model.
Negative dimensions/radii, invalid sample counts, undecided strict predicates,
unsupported torus representations, and unrepresentable buffer capacities are
errors, with structured variants retained by `AdapterError::Construction`.
Proven zero-size primitives remain canonical empty sets after validation of all
parameters. Cylinder/frustum height is directed, so negative height remains
supported. A cone may have one zero radius. Sample counts require at least three
segments and, for spheres/ellipsoids, two stacks. Nonempty tori require
`major_radius > minor_radius`; a zero minor radius represents the empty set.
Only successful constructions enter the primitive cache.

Modeling transforms (`solid::transform`, `rotate`, `scale`, `mirror`, and
`SolidExt::{transformed, translated}`) also return `Result<TriangleMesh,
ValidationError>`; propagate them with `?`. Their existing `try_` counterparts
remain equivalent. Invalid indices, singular transforms, invalid planes,
undecided predicates, and projective images crossing infinity fail explicitly.
The shared Hypermesh implementation validates indices and certifies a nonzero
homogeneous scale of constant sign on each triangle. Separate components may
have opposite signs. Native diagnostics remain available in
`ValidationError::Mesh` and `AdapterError::Construction`. Open surfaces and point
sets remain supported transform inputs; a transform does not certify topology.
Valid transforms of canonical empty geometry remain canonical empty.
Context variants report operation-local predicate certainty; they do not certify
the provenance of an earlier approximate construction. Transform work and
allocation do not yet have an internal total-work, heap-byte, or cancellation bound.

Bounds queries (`solid::bounding_box`, `curve::bounding_box`, their `try_`
variants, and `SolidExt::exact_bounds`) now return `Result<Option<Aabb>,
ValidationError>`. Propagate errors with `?`, then handle `None` as a completed
query on empty geometry. A point at the origin has `Some` zero-size bounds.
Context queries retain the certainty receipt around the optional bounds.
Scalar adapters follow the same optional-result contract. Mesh bounds validate
indices before reusing retained position facts; a bounds query does not validate
closed-solid topology. `solid::center` and `float` now return `Result`; they
preserve canonical empty geometry and propagate bounds failures.

The six TPMS constructors (`gyroid`, `schwarz_p`, `schwarz_d`, and their `_solid`
forms) also return `Result<TriangleMesh, ValidationError>`. Their sampling formulas
and valid resolutions are unchanged. Resolutions below two are now rejected
instead of silently clamped. Periods must be proved positive and solid wall
thickness must be proved nonnegative. Empty bounds and zero wall thickness
produce canonical empty geometry after parameter validation. Undecided bounds,
sample signs, interpolation, orientation, and native Surface Nets failures remain
errors with their diagnostic category; they no longer produce an empty model.
Certification concerns the sampled representation, not the continuous TPMS field.

These constructors make strict numerical decisions and retain exact sampled
coordinates. They do not certify fidelity to an analytic surface or attach
end-to-end provenance to a mesh. Buffer capacity checks and fallible reservations
report representational overflow and reservation failure. They do not bound total
work, scalar expression growth, peak heap usage, or allocator/process termination;
there is no constructor cancellation contract yet. Use external containment for
large or adversarial sample counts. Other legacy convenience operations that
replace errors with empty geometry remain an open migration target.

For sampled curved solids whose predicates remain undecided in strict mode,
select `GeometryContext::APPROXIMATE_512` explicitly:

```rust
use csgrs::{GeometryContext, Real, solid::{self, SolidExt}};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let body = solid::cuboid(Real::from(12), Real::from(12), Real::from(4))?;
    let drill = solid::cylinder(Real::from(2), Real::from(6), 16)?
        .translated(Real::from(6), Real::from(6), Real::from(-1))?;
    let result = body.try_difference_with_context(
        &drill,
        &GeometryContext::APPROXIMATE_512,
    )?;
    println!("Predicate certainty: {:?}", result.certainty);
    let drilled_body = result.into_value();
    println!("Output triangles: {}", drilled_body.triangles.len());
    Ok(())
}
```

The context reaches Hypermesh's input validation, Boolean evaluation, and
output certification. `GeometryCertainty::Approximate512Consumed` means at
least one predicate used the terminal 512-bit interpretation. Operations
whose decisions are all certified still report `Certified` with this context.

Context-bearing variants also cover the operations below. Each returns a
`Result<GeometryOutcome<T>, _>`; `T` is the mesh, region, query value, or import
report. The existing convenience entry points retain their strict default.

| Operation | Context-bearing API |
|---|---|
| Mesh transforms and positioning | `solid::try_transform_with_context`, `try_rotate_with_context`, `try_scale_with_context`, `try_mirror_with_context`, `try_center_with_context`, `try_float_with_context` |
| Mesh bounds and queries | `solid::try_bounding_box_with_context`, `contains_point_with_context`, `polyline_intersections_with_context`, `dihedral_angle_with_context` |
| Hulls, copies, and polygon caps | `solid::convex_hull_with_context`, `minkowski_sum_with_context`, `distribute_linear_with_context`, `distribute_grid_with_context`, `distribute_arc_with_context`, `polyhedron_with_context`, `loft_with_context` |
| Projection and slicing | `solid::flatten_with_context`, `slice_z_with_context` |
| Native Real metaball meshing | `solid::metaballs_with_context`, `metaballs_with_diagnostics_and_context` |
| Native Real SDF meshing | `solid::sdf_with_context`, `sdf_with_diagnostics_and_context`, `sdf_expr_with_context`, `sdf_expr_with_diagnostics_and_context` |
| Curve construction | `curve::polygon_with_context`, `rectangle_with_context`, `square_with_context`, `circle_with_context`, `ellipse_with_context`, `regular_ngon_with_context`, `rounded_rectangle_with_context`, `bezier_region_with_context`, `bspline_path_with_context` |
| Curve transforms and queries | `curve::try_transformed_with_context`, `try_translated_with_context`, `try_rotated_with_context`, `try_scaled_with_context`, `try_bounding_box_with_context`, `contains_xy_with_context`, `hilbert_strings_with_context` |
| Mesh imports | `io::obj::from_obj_with_context`, `from_obj_attributed_with_context`, `io::dxf::from_dxf_with_context`, `io::vrml::from_vrml_with_context` |
| Gerber conversion | `io::gerber::import_gerber_with_context`, `export_gerber_with_options_and_context` |

Curve composites such as `ring`, `crescent`, and `circle_with_keyway` pass their
existing context through component construction, transforms, and Booleans.
Mesh distributions, flattening, and slicing retain certainty in their caches
and include the policy in cache keys. Their context-bearing APIs report
undecided predicates as errors rather than substituting empty geometry.

Scalar mesh adapters expose `union_with_context`, `difference_with_context`,
`intersection_with_context`, `xor_with_context`, and context-bearing transform,
positioning, bounds, and polyhedron methods. Scalar curve adapters expose
context-bearing transforms, bounds, and polygon construction. JavaScript mesh,
curve, and plane wrappers provide corresponding `*WithContext` methods; the
last argument selects the approximate terminal and the result exposes
`approximate512Consumed` alongside its value. Selecting approximation does not
suppress invalid-input or unsupported-operation errors.

Rendered solid operations:

| Preview | Function |
|---|---|
| <img src="docs/inverse.png" width="144" alt="Solid with inverted triangle winding and visible topology"/> | `solid::inverse(mesh: &TriangleMesh) -> TriangleMesh` |
| <img src="docs/convex_hull.png" width="144" alt="Convex hull with visible faces, edges, and vertices"/> | `solid::convex_hull(mesh: &TriangleMesh) -> HypermeshResult<TriangleMesh>` |
| <img src="docs/minkowski_sum.png" width="144" alt="Minkowski sum with visible faces, edges, and vertices"/> | `solid::minkowski_sum(left: &TriangleMesh, right: &TriangleMesh) -> Result<TriangleMesh, ValidationError>` |

### Solids to planar geometry

The inverse dimensional conversions preserve native Hypercurve topology:

| Preview | Function |
|---|---|
| <img src="docs/flatten.png" width="144" alt="Filled planar projection of a three-dimensional solid with visible exact edges and vertices"/> | `solid::flatten(mesh: &TriangleMesh) -> Result<CurveRegion2, ValidationError>` |
| <img src="docs/slice_z.png" width="144" alt="Filled planar slice of a three-dimensional solid with visible exact edges and vertices"/> | `solid::slice_z(mesh: &TriangleMesh, z: Real) -> Result<SliceResult, ValidationError>` |

`flatten` unions the XY projections of all triangle faces, omitting projected
faces of zero area. `slice_z` returns filled closed sections, open curve strings,
and higher-order paths separately. Closed sections and coplanar faces use
signed nonzero winding: holes stay empty, equally oriented shells accumulate,
and opposite traversals cancel. The sections approached from above and below
are united, preserving cuts through shared edges and coplanar caps. Isolated
point contacts are omitted by this regularized area/wire carrier.

Migration: both primary functions now return `Result`; propagate errors with
`?` or handle them explicitly. Invalid indices and undecided predicates no
longer become empty geometry. This is a correctness and API change, including
corrected slice topology; prior slice serialization is not preserved. The
context variants retain aggregate operation-local certainty. They do not
certify inherited construction intent or input manifoldness, nor impose an
internal total-work, memory-byte or cancellation bound. Native Hypermesh
section records retain source triangles, edges and exact interpolation
parameters; the planar return value does not expose those mesh-source records.

### Metadata

Enable `attributed` when a renderer or interchange boundary needs aligned
face information:

```rust
use csgrs::{AttributedMesh, Real, solid};

let geometry = solid::cube(Real::from(2)).expect("positive cube width");
let attributed = AttributedMesh::from_uniform(geometry, "housing");
assert_eq!(
    attributed.face_metadata().len(),
    attributed.geometry().triangles.len()
);
```

`AttributedMesh::new` validates one metadata row per triangle.
`with_authored_normals` additionally validates one normal per position.
`geometry`, `face_metadata`, `authored_normals`, `into_parts`, and
`map_metadata` expose the sidecar without duplicating modeling behavior.

## File formats

Feature-gated functions live under `csgrs::io`:

| Format | Import | Export |
|---|---|---|
| STL | `stl::from_stl` | `to_stl_ascii`, `to_stl_binary` |
| OBJ | `obj::from_obj` | `to_obj`, `write_obj` |
| PLY | — | `to_ply`, `write_ply` |
| AMF | — | `to_amf`, `to_amf_with_color`, `write_amf` |
| glTF/GLB | `gltf::from_gltf` | `to_gltf`, `write_gltf`, `to_gltf_scene`, `write_gltf_scene` |
| VRML 2.0 | `vrml::from_vrml` | — |
| DXF | `dxf::from_dxf` | `dxf::to_dxf` |
| SVG | `svg::import_svg` | `svg::export_svg` |
| Gerber | `gerber::import_gerber` | `gerber::export_gerber` |

Triangle importers return `TriangleMesh`. Curve importers return distinct
filled, string, and path carriers where the source format can contain more
than one topology kind. Mesh exporters accept `TriangleMesh` directly, and
planar exporters accept native `CurveRegion2` values.

## Boundary adapters

`csgrs::adapter` provides opt-in primitive-scalar wrappers over native
geometry:

- scalar policies: `RawReal`, `F32`, `F64`, and `I128`;
- solid boundary: `ScalarMesh<A>`;
- curve boundary: `ScalarCurve<A>`;
- result carriers: `GraphicsMesh`, `IndexedMeshBuffers`, and `Aabb3`.

Conversions happen only at ingress and egress. Float output is fallible when
coordinates are not finite; integer output requires an exact, in-range
integer. The adapters do not implement another geometry kernel.

The C ABI is the sibling
[`csgrs-ffi`](https://github.com/timschmidt/csgrs-ffi) crate. C++, Go, Python,
and TypeScript wrappers are documented in [bindings](bindings/README.md).
The `wasm` feature provides browser-oriented wrappers over the same core;
the [WASM binding guide](bindings/js/WASM.md) covers ownership, named certainty
receipts, and decided/boundary/uncertain point-query status.

## Features

Default features provide the common modeling and interchange surface. For
smaller builds use `default-features = false`.

| Feature | Adds |
|---|---|
| `curve` | native curve grammar and curve-to-solid operations |
| `attributed` | aligned metadata/authored-normal boundary carrier |
| `stl-io`, `obj-io`, `ply-io`, `amf-io` | triangle interchange |
| `gltf-io`, `vrml-io`, `dxf-io` | scene and CAD interchange |
| `svg-io`, `gerber-io` | curve interchange |
| `offset` | regularized planar offsets |
| `truetype-text`, `hershey-text`, `image-io` | text and raster curve construction |
| `metaballs`, `sdf` | sampled implicit geometry |
| `bevymesh` | Bevy conversion |
| `wasm` | WebAssembly/browser wrappers |
| `dispatch-trace` | computation-path evidence for tests and benchmarks |

A minimal curve-and-solid build is:

```toml
[dependencies]
csgrs = { version = "0.23.0", default-features = false, features = ["curve", "stl-io"] }
```

For WebAssembly:

```sh
cargo install wasm-pack
wasm-pack build --profile wasm --target bundler --out-dir pkg -- --features wasm
```

## Correctness boundaries

The stack-wide [admission protocol](docs/admission.md) defines the proof,
certainty, cache, boundary, and validation requirements for extensions.
The [properties execution ledger](docs/properties-status.json) records the
remaining work and its evidence; unverified items are not completion claims.

- Topology-sensitive decisions use `Real` and the Hyper predicate stack.
- Boolean inputs must meet Hypermesh's documented finite, closed, oriented
  solid preconditions. Keep structured `try_*` errors.
- Sampled circles, text, raster contours, implicit fields, and graphics
  buffers remain explicit approximations at caller-chosen resolutions.
- `CurveRegion2`, `CurveString2`, and `CurvePath2` have different meanings;
  CSGRS does not hide them behind a combined 2D facade.
- `TriangleMesh` is authoritative geometry. `AttributedMesh` stores only
  aligned boundary attributes.
- Bevy, WebAssembly, scalar adapters, FFI, and serialization are application
  boundaries; their finite projections never become topology truth.

`solid::cache::with_policy` gives synchronous callers explicit per-cache entry
limits. `CachePolicy::DISABLED` clears grammar caches and retains no new entries;
`policy`, `usage`, and `clear` expose their state. Scopes restore the prior policy
on return or unwinding, and limits apply independently on each thread. These are
entry budgets, not byte budgets. See the [cache contract](src/solid/cache.rs) for
key identity, FIFO eviction, nesting, and certainty semantics.

## Performance and hard tests

The [exactCorelib comparison harness](crates/comparisons/INTEGRATION.md) keeps
the pinned historical audit archive beside runnable native comparisons.
Its validation record distinguishes current test and benchmark execution
checks from historical timing, memory, and numerical claims.

The reproducible harness, CSV schema, dependency policy, and interpretation
rules are in [benchmarks/README.md](benchmarks/README.md). Retained
measurements and optimization notes are in [PERFORMANCE.md](PERFORMANCE.md).

The serialized suite compares CSGRS, CGAL EPECK, and tight-tolerance
OpenCascade across 66 shared construction, transform, Boolean, analysis, and
I/O workloads. That includes deterministic 8,020-triangle concave-labyrinth
and 36,096-triangle level-3 Sierpiński-foam meshes. Rust comparisons add
`boolmesh`, `manifold-rust`, and `tri-mesh` where their representations
overlap.

Set `YEAHRIGHT_BENCH=1` for benchmark runs that should download the optional
public-domain YeahRight corpus into `target/benchmark-fixtures/yeahright`.

## Examples

The `examples` directory covers primitives, native curve extrusion, Booleans,
transforms, hulls, Minkowski sums, metadata, adapters, and interchange. The
Nuxt WebAssembly demo is under
[examples/wasm-nuxt4](examples/wasm-nuxt4/README.md).

Community integrations:

- [csgrs-bevy-example](https://github.com/timschmidt/csgrs-bevy-example)
- [csgrs-egui-example](https://github.com/timschmidt/csgrs-egui-example)
- [csgrs-egui-wasm-example](https://github.com/timschmidt/csgrs-egui-wasm-example)
- [csgrs-druid-example](https://github.com/timschmidt/csgrs-druid-example)

## References

These are the authoritative mathematical and interchange references for the
crate:

- Nicholas M. Patrikalakis, Takashi Maekawa, and Wonjoon Cho,
  [*Shape Interrogation for Computer Aided Design and Manufacturing*](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/),
  MIT Press. The curve/surface, intersection, robustness, and offset chapters
  motivate the separation of representation, interrogation, and presentation.
- Jonathan Richard Shewchuk,
  [“Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates”](https://www.cs.cmu.edu/~quake/robust.html),
  *Discrete & Computational Geometry* 18, 1997, pp. 305–363. This is the
  primary reference for sign-correct adaptive predicates.
- Chee K. Yap,
  [“Towards Exact Geometric Computation”](https://doi.org/10.1016/0925-7721(95)00040-2),
  *Computational Geometry* 7, 1997, pp. 3–23. This informs the separation
  between exact decisions and explicit approximation boundaries.
- Ucamco,
  [Gerber Layer Format specifications](https://www.ucamco.com/en/gerber/downloads),
  the normative source for the Gerber adapter.
- W3C, [Scalable Vector Graphics (SVG) 2](https://www.w3.org/TR/SVG2/),
  the normative source format adapted through Hypercurve.
- Khronos Group,
  [glTF 2.0 specification](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html),
  the normative scene and buffer format for the glTF adapter.
## Acknowledgements

CSGRS began as a Rust translation of Evan Wallace's
[CSG.js](https://github.com/evanw/csg.js), released under the MIT license.
That lineage and copyright notice remain in [LICENSE](LICENSE).

The crate builds on work by the Hyperreal, Hyperlattice, Hyperlimit,
Hypercurve, Hypertri, Hypermesh, Hyperphysics, and Hypersdf contributors and
the upstream format/rendering crates named in `Cargo.toml`.

The compiled single-stroke font catalog was created by Dr. A. V. Hershey at
the U.S. National Bureau of Standards. Its source distribution format was
created by James Hurt of Cognition, Inc.; the integrated Rust representation
is not the U.S. NTIS distribution format.

## Contributing and license

Run the release checks before submitting changes:

```sh
cargo fmt --all -- --check
cargo test --all-features
cargo clippy --all-features --all-targets -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --all-features --no-deps
cargo check --all-features --benches
```

Discussion is available in the
[CSGRS Discord community](https://discord.gg/9WkD3WFxMC). CSGRS is available
under the [MIT License](LICENSE).
