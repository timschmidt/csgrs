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
    let body = solid::cube(Real::from(12));
    let opening = curve::try_extrude(
        &curve::square(Real::from(4)),
        Real::from(16),
        &GeometryContext::STRICT,
    )?
    .into_value()
    .translated(Real::from(4), Real::from(4), Real::from(-2));

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

Rendered filled constructors:

| Preview | Function |
|---|---|
| <img src="docs/square.png" width="144" alt="Filled square with visible edges and vertices"/> | `curve::square(width: Real) -> CurveRegion2` |
| <img src="docs/rectangle.png" width="144" alt="Filled rectangle with visible edges and vertices"/> | `curve::rectangle(width: Real, length: Real) -> CurveRegion2` |
| <img src="docs/circle.png" width="144" alt="Filled sampled circle with visible edges and vertices"/> | `curve::circle(radius: Real, segments: usize) -> CurveRegion2` |
| <img src="docs/right_triangle.png" width="144" alt="Filled right triangle with visible edges and vertices"/> | `curve::right_triangle(width: Real, height: Real) -> CurveRegion2` |
| <img src="docs/polygon.png" width="144" alt="Filled polygon with visible edges and vertices"/> | `curve::polygon(points: &[[Real; 2]]) -> CurveRegion2` |
| <img src="docs/rounded_rectangle.png" width="144" alt="Filled rounded rectangle with visible edges and vertices"/> | `curve::rounded_rectangle(width: Real, height: Real, corner_radius: Real, corner_segments: usize) -> CurveRegion2` |
| <img src="docs/ellipse.png" width="144" alt="Filled sampled ellipse with visible edges and vertices"/> | `curve::ellipse(width: Real, height: Real, segments: usize) -> CurveRegion2` |
| <img src="docs/regular_ngon.png" width="144" alt="Filled regular polygon with visible edges and vertices"/> | `curve::regular_ngon(sides: usize, radius: Real) -> CurveRegion2` |
| <img src="docs/curve_arrow.png" width="144" alt="Filled planar arrow with visible edges and vertices"/> | `curve::arrow(shaft_length: Real, shaft_width: Real, head_length: Real, head_width: Real) -> CurveRegion2` |
| <img src="docs/trapezoid.png" width="144" alt="Filled trapezoid with visible edges and vertices"/> | `curve::trapezoid(top_width: Real, bottom_width: Real, height: Real, top_offset: Real) -> CurveRegion2` |
| <img src="docs/star.png" width="144" alt="Filled star with visible edges and vertices"/> | `curve::star(num_points: usize, outer_radius: Real, inner_radius: Real) -> CurveRegion2` |
| <img src="docs/teardrop.png" width="144" alt="Filled teardrop with visible edges and vertices"/> | `curve::teardrop(width: Real, length: Real, segments: usize) -> CurveRegion2` |
| <img src="docs/curve_egg.png" width="144" alt="Filled egg with visible edges and vertices"/> | `curve::egg(width: Real, length: Real, segments: usize) -> CurveRegion2` |
| <img src="docs/squircle.png" width="144" alt="Filled squircle with visible edges and vertices"/> | `curve::squircle(width: Real, height: Real, segments: usize) -> CurveRegion2` |
| <img src="docs/keyhole.png" width="144" alt="Filled keyhole with visible edges and vertices"/> | `curve::keyhole(circle_radius: Real, handle_width: Real, handle_height: Real, segments: usize, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError>` |
| <img src="docs/reuleaux.png" width="144" alt="Filled Reuleaux polygon with visible exact curve edges and vertices"/> | `curve::reuleaux(sides: usize, diameter: Real, circle_segments: usize, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError>` |
| <img src="docs/heart.png" width="144" alt="Filled heart with visible edges and vertices"/> | `curve::heart(width: Real, height: Real, segments: usize) -> CurveRegion2` |
| <img src="docs/ring.png" width="144" alt="Filled annulus with visible exact curve edges and vertices"/> | `curve::ring(inner_diameter: Real, thickness: Real, segments: usize, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError>` |
| <img src="docs/pie_slice.png" width="144" alt="Filled circular sector with visible edges and vertices"/> | `curve::pie_slice(radius: Real, start_angle_deg: Real, end_angle_deg: Real, segments: usize) -> CurveRegion2` |
| <img src="docs/crescent.png" width="144" alt="Filled crescent with visible exact curve edges and vertices"/> | `curve::crescent(outer_radius: Real, inner_radius: Real, offset: Real, segments: usize, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError>` |
| <img src="docs/supershape.png" width="144" alt="Filled superformula shape with visible edges and vertices"/> | `curve::supershape(a: Real, b: Real, m: Real, n1: Real, n2: Real, n3: Real, segments: usize) -> CurveRegion2` |
| <img src="docs/circle_with_keyway.png" width="144" alt="Circle with a keyway and visible exact curve edges"/> | `curve::circle_with_keyway(radius: Real, segments: usize, key_width: Real, key_depth: Real, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError>` |
| <img src="docs/circle_with_flat.png" width="144" alt="Circle with one flat and visible exact curve edges"/> | `curve::circle_with_flat(radius: Real, segments: usize, flat_distance: Real, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError>` |
| <img src="docs/circle_with_two_flats.png" width="144" alt="Circle with two flats and visible exact curve edges"/> | `curve::circle_with_two_flats(radius: Real, segments: usize, flat_distance: Real, context: &GeometryContext) -> Result<GeometryOutcome<CurveRegion2>, CurveBooleanError>` |
| <img src="docs/involute_gear.png" width="144" alt="Filled involute gear profile with visible edges and vertices"/> | `curve::involute_gear(module: Real, teeth: usize, pressure_angle_degrees: Real, clearance: Real, backlash: Real, segments_per_flank: usize) -> CurveRegion2` |
| <img src="docs/cycloidal_gear.png" width="144" alt="Filled cycloidal gear profile with visible edges and vertices"/> | `curve::cycloidal_gear(module: Real, teeth: usize, generating_radius: Real, clearance: Real, segments_per_flank: usize) -> CurveRegion2` |
| <img src="docs/involute_rack.png" width="144" alt="Filled involute rack profile with visible edges and vertices"/> | `curve::involute_rack(module: Real, teeth: usize, pressure_angle_degrees: Real, clearance: Real, backlash: Real) -> CurveRegion2` |
| <img src="docs/cycloidal_rack.png" width="144" alt="Filled cycloidal rack profile with visible edges and vertices"/> | `curve::cycloidal_rack(module: Real, teeth: usize, clearance: Real, segments_per_flank: usize) -> CurveRegion2` |
| <img src="docs/airfoil_naca4.png" width="144" alt="Filled NACA four-digit airfoil with visible edges and vertices"/> | `curve::airfoil_naca4(max_camber: Real, camber_position: Real, thickness: Real, chord: Real, samples: usize) -> CurveRegion2` |
| <img src="docs/bezier.png" width="144" alt="Filled exact Bezier boundary with visible edge and vertex"/> | `curve::bezier_region(control: &[[Real; 2]], display_segments: usize) -> CurveRegion2` |
| <img src="docs/metaballs_2d.png" width="144" alt="Filled planar metaball contours with visible edges and vertices"/> | `curve::metaballs(balls: &[(Point2, Real)], resolution: (usize, usize), iso_value: Real, padding: Real) -> CurveRegion2` |
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

- `CurveRegionExt::{try_union, try_difference, try_intersection, try_xor}`;
- `offset` and `offset_rounded`;
- `transformed`, `translated`, `rotated`, and `scaled`;
- `contains_xy`, `bounding_box`, `finite_profiles`, and `triangulate`.

`finite_profiles` is an explicit display/interchange projection. It is not a
replacement for the native curve topology.

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
| <img src="docs/cube.png" width="144" alt="Cube with visible faces, edges, and vertices"/> | `solid::cube(width: Real) -> TriangleMesh` |
| <img src="docs/cuboid.png" width="144" alt="Cuboid with visible faces, edges, and vertices"/> | `solid::cuboid(width: Real, length: Real, height: Real) -> TriangleMesh` |
| <img src="docs/sphere.png" width="144" alt="Sampled sphere with visible faces, edges, and vertices"/> | `solid::sphere(radius: Real, segments: usize, stacks: usize) -> TriangleMesh` |
| <img src="docs/cylinder.png" width="144" alt="Sampled cylinder with visible faces, edges, and vertices"/> | `solid::cylinder(radius: Real, height: Real, segments: usize) -> TriangleMesh` |
| <img src="docs/ellipsoid.png" width="144" alt="Sampled ellipsoid with visible faces, edges, and vertices"/> | `solid::ellipsoid(radius_x: Real, radius_y: Real, radius_z: Real, segments: usize, stacks: usize) -> TriangleMesh` |
| <img src="docs/frustum.png" width="144" alt="Conical frustum with visible faces, edges, and vertices"/> | `solid::frustum(radius_bottom: Real, radius_top: Real, height: Real, segments: usize) -> TriangleMesh` |
| <img src="docs/frustum_between.png" width="144" alt="Conical frustum between two points with visible faces, edges, and vertices"/> | `solid::frustum_between(start: Point3, end: Point3, start_radius: Real, end_radius: Real, segments: usize) -> TriangleMesh` |
| <img src="docs/polyhedron.png" width="144" alt="Indexed polyhedron with visible faces, edges, and vertices"/> | `solid::polyhedron(points: &[[Real; 3]], faces: &[&[usize]]) -> Result<TriangleMesh, ValidationError>` |
| <img src="docs/octahedron.png" width="144" alt="Regular octahedron with visible faces, edges, and vertices"/> | `solid::octahedron(radius: Real) -> TriangleMesh` |
| <img src="docs/icosahedron.png" width="144" alt="Regular icosahedron with visible faces, edges, and vertices"/> | `solid::icosahedron(radius: Real) -> TriangleMesh` |
| <img src="docs/mesh_arrow.png" width="144" alt="Three-dimensional arrow with visible faces, edges, and vertices"/> | `solid::arrow(start: Point3, direction: Vector3, segments: usize, mirrored: bool) -> TriangleMesh` |
| <img src="docs/torus.png" width="144" alt="Sampled torus with visible faces, edges, and vertices"/> | `solid::torus(major_radius: Real, minor_radius: Real, major_segments: usize, minor_segments: usize) -> TriangleMesh` |
| <img src="docs/teardrop_cylinder.png" width="144" alt="Extruded teardrop solid with visible faces, edges, and vertices"/> | `solid::teardrop_cylinder(width: Real, length: Real, height: Real, shape_segments: usize, context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/spur_gear_involute.png" width="144" alt="Involute spur gear with visible faces, edges, and vertices"/> | `solid::spur_gear_involute(module: Real, teeth: usize, pressure_angle_degrees: Real, clearance: Real, backlash: Real, segments_per_flank: usize, thickness: Real, context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/spur_gear_cycloid.png" width="144" alt="Cycloidal spur gear with visible faces, edges, and vertices"/> | `solid::spur_gear_cycloid(module: Real, teeth: usize, generating_radius: Real, clearance: Real, segments_per_flank: usize, thickness: Real, context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/helical_involute_gear.png" width="144" alt="Helical involute gear with visible faces, edges, and vertices"/> | `solid::helical_involute_gear(module: Real, teeth: usize, pressure_angle_degrees: Real, clearance: Real, backlash: Real, segments_per_flank: usize, thickness: Real, helix_angle_degrees: Real, slices: usize, context: &GeometryContext) -> Result<GeometryOutcome<TriangleMesh>, ValidationError>` |
| <img src="docs/metaballs_3d.png" width="144" alt="Sampled metaball solid with visible faces, edges, and vertices"/> | `solid::metaballs(balls: &[MetaBall], resolution: (usize, usize, usize), iso_value: Real, padding: Real) -> TriangleMesh` |
| <img src="docs/sdf.png" width="144" alt="Sampled signed-distance-field solid with visible faces, edges, and vertices"/> | `solid::sdf(field: impl Fn(&Point3) -> Real, resolution: (usize, usize, usize), min: Point3, max: Point3, iso_value: Real) -> TriangleMesh` |
| <img src="docs/gyroid.png" width="144" alt="Bounded gyroid solid with visible faces, edges, and vertices"/> | `solid::gyroid_solid(bounds: &TriangleMesh, resolution: usize, scale: Real, iso_value: Real, wall_thickness: Real) -> TriangleMesh` |
| <img src="docs/schwarz_p.png" width="144" alt="Bounded Schwarz-P solid with visible faces, edges, and vertices"/> | `solid::schwarz_p_solid(bounds: &TriangleMesh, resolution: usize, scale: Real, iso_value: Real, wall_thickness: Real) -> TriangleMesh` |
| <img src="docs/schwarz_d.png" width="144" alt="Bounded Schwarz-D solid with visible faces, edges, and vertices"/> | `solid::schwarz_d_solid(bounds: &TriangleMesh, resolution: usize, scale: Real, iso_value: Real, wall_thickness: Real) -> TriangleMesh` |
| <img src="docs/gyroid_surface.png" width="144" alt="Bounded gyroid level surface with visible faces, edges, and vertices"/> | `solid::gyroid(bounds: &TriangleMesh, resolution: usize, scale: Real, iso_value: Real) -> TriangleMesh` |
| <img src="docs/schwarz_p_surface.png" width="144" alt="Bounded Schwarz-P level surface with visible faces, edges, and vertices"/> | `solid::schwarz_p(bounds: &TriangleMesh, resolution: usize, scale: Real, iso_value: Real) -> TriangleMesh` |
| <img src="docs/schwarz_d_surface.png" width="144" alt="Bounded Schwarz-D level surface with visible faces, edges, and vertices"/> | `solid::schwarz_d(bounds: &TriangleMesh, resolution: usize, scale: Real, iso_value: Real) -> TriangleMesh` |

`solid::empty()` is intentionally image-free. Diagnostic variants
`metaballs_with_diagnostics` and `sdf_with_diagnostics` produce the same shape
families while returning sampling reports. `sdf_expr` and
`sdf_expr_with_diagnostics` accept retained `hypersdf::SdfExpr` values and
produce the same SDF shape family.

Sampling arguments such as segments, stacks, slices, and resolution are
explicit tessellation choices; CSGRS has no global tolerance setting.

### Solid operations and queries

`solid::SolidExt` supplies fluent native operations:

- `try_union`, `try_difference`, `try_intersection`, and `try_xor`;
- `transformed` and `translated`;
- `exact_bounds`;
- `visit_native_triangles`.

The corresponding free functions and additional operations are:

- Booleans and transforms: `boolean`, `transform`, `rotate`, `scale`,
  `mirror`, `inverse`, `translation`, and `translation_to`;
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
| <img src="docs/flatten.png" width="144" alt="Filled planar projection of a three-dimensional solid with visible exact edges and vertices"/> | `solid::flatten(mesh: &TriangleMesh) -> CurveRegion2` |
| <img src="docs/slice_z.png" width="144" alt="Filled planar slice of a three-dimensional solid with visible exact edges and vertices"/> | `solid::slice_z(mesh: &TriangleMesh, z: Real) -> (CurveRegion2, Vec<CurveString2>, Vec<CurvePath2>)` |

`flatten` unions the XY projections of all triangle faces. `slice_z` returns
filled closed sections, open curve strings, and higher-order paths separately
instead of discarding their topology roles.

### Metadata

Enable `attributed` when a renderer or interchange boundary needs aligned
face information:

```rust
use csgrs::{AttributedMesh, Real, solid};

let geometry = solid::cube(Real::from(2));
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
The `wasm` feature provides browser-oriented wrappers over the same core.

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
wasm-pack build --release --target bundler --out-dir pkg -- --features wasm
```

## Correctness boundaries

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

## Performance and hard tests

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
