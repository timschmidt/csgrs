# csgrs

A fast, optionally multithreaded **Constructive Solid Geometry (CSG)**
library in Rust, built around Boolean operations (*union*, *difference*,
*intersection*, *xor*) on several different internal geometry representations.
**csgrs** provides data structures and methods for constructing 2D and 3D geometry
with an [OpenSCAD](https://openscad.org/)-like syntax.  Our aim is for **csgrs**
to be light weight and full featured through integration with the local hyper
geometry crates for exact-aware planar regions, curves, triangulation, mesh
topology, and physical-property calculations. The crate accepts hyperreal
geometry values internally, keeps primitive floats at IO/JS/adapter boundaries,
and can be built for WASM. Dependencies are 100% Rust and nearly all optional.

The core API accepts raw `hyperreal::Real` values. Primitive scalar adapters live
outside the core in [csgrs-adapter](adapters/README.md), where Rust `f32`,
`f64`, `i128`, and raw hyperreal entry points convert at ingress and egress while
all geometry remains hyperreal underneath. Polygon triangulation uses
[hypertri](../hypertri/README.md): **csgrs** rotates 3D polygons into 2D, lifts
projected coordinates to hyperreals, runs exact predicate triangulation, then
maps triangle indices back onto the original 3D vertices.

![Example CSG output](docs/csg.png)

## Hyper Stack Links

- [hyperreal](../hyperreal/README.md): exact rational, symbolic, and computable
  real arithmetic.
- [hyperlimit](../hyperlimit/README.md): exact predicate policy and certified
  geometric decisions.
- [hyperlattice](../hyperlattice/README.md): small exact vector, matrix, and
  transform algebra.
- [hypercurve](../hypercurve/README.md): planar curve, contour, region, and
  boolean geometry.
- [hypertri](../hypertri/README.md): exact polygon triangulation and constrained
  Delaunay topology.
- [hypermesh](../hypermesh/README.md): exact-aware indexed mesh validation,
  topology, and 3D Boolean operations.
- [hypersolve](../hypersolve/README.md): experimental exact-aware solver layer.
- [hyperdrc](../hyperdrc/README.md): PCB design-readiness checks over exact-aware
  geometry adapters.
- [hyperphysics](../hyperphysics/README.md): placeholder physics-domain crate
  for the exact geometry stack.
- [csgrs](../csgrs/readme.md): constructive solid geometry and polygon boolean
  engine used by HyperDRC and available as an interop target.

## Community
[![](https://dcbadge.limes.pink/api/server/https://discord.gg/9WkD3WFxMC)](https://discord.gg/9WkD3WFxMC)

## Acknowledgments

Thanks to the project contributors whose code, documentation, examples, issue
reports, review, and implementation guidance have helped shape **csgrs**:
[Andrei Stepanenko](https://github.com/ftvkyo), Bruce Mitchener, Connor K,
[Ed Swartz](https://github.com/eswartz), [Hattiffnat](https://github.com/Hattiffnat),
[Mark van der Net](https://github.com/mvdnet), Markus Sprecher,
[Naseschwarz](https://github.com/Naseschwarz), Nathan Fenner,
[PJB3005](https://github.com/PJB3005), [qthree](https://github.com/qthree),
[Ricardo C. Oliveira](https://github.com/Ricardo-C-Oliveira), Robin Miller, Ryan,
[TimTheBig](https://github.com/TimTheBig),
[uttarayan21](https://github.com/uttarayan21), and Wink Saville.

## Getting started

### A simple CSG example

Install the [Rust](https://www.rust-lang.org/) language tools from
[rustup.rs](https://rustup.rs/).

Use cargo to create a new project, `my_cad_project`, and add the `csgrs` dependency:
```shell
cargo new my_cad_project
cd my_cad_project
cargo add csgrs
```

### main.rs

Change `src/main.rs` to the following code:
```rust
use csgrs::csg::CSG;
use csgrs::Real;

type Mesh = csgrs::mesh::Mesh<()>;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create a cube
    let cube: Mesh = Mesh::cube(Real::from(2), ()); // 2×2×2 cube at origin, no metadata

    // Create sphere at (1, 1, 1) with radius 1.25:
    let radius = (Real::from(5) / Real::from(4)).unwrap();
    let sphere: Mesh = Mesh::sphere(radius, 16, 8, ())
        .translate(Real::one(), Real::one(), Real::one());

    // Perform a difference operation:
    let result = cube.try_difference(&sphere).expect("certified mesh difference");

    // Write the result as an ASCII STL:
    let stl = result.to_stl_ascii("cube_minus_sphere")?;
    std::fs::write("cube_sphere_difference.stl", stl)?;
    Ok(())
}
```

### Build and run

```shell
cargo build
cargo run
```

This results in a file named `cube_sphere_difference.stl` in the current directory
and it can be viewed in a STL viewer such as [f3d](https://github.com/f3d-app/f3d)
with, `f3d cube_sphere_difference.stl`.

### Primitive scalar adapters

Use [csgrs-adapter](adapters/README.md) when an application wants a primitive
Rust scalar surface while keeping `csgrs` exact internally:

```rust
use csgrs_adapter::{F64, Mesh};

type MeshF64 = Mesh<F64, ()>;

fn main() -> Result<(), csgrs_adapter::AdapterError> {
    let cube = MeshF64::cube(2.0, ())?;
    let moved = cube.translate(1.0, 0.0, 0.0)?;
    let _bounds = moved.bounding_box()?;
    Ok(())
}
```

### Building for WASM

```shell
cargo install wasm-pack
wasm-pack build --release --target bundler --out-dir pkg -- --features wasm
```

## Features and Structures

### Profile Structure

- **`Profile<M>`** stores and manipulates 2D CAD geometry as hypercurve topology. It contains:
  - a [`hypercurve::Region2`](https://docs.rs/hypercurve/latest/hypercurve/struct.Region2.html) for filled material and hole contours
  - native [`hypercurve::CurveString2`](https://docs.rs/hypercurve/latest/hypercurve/struct.CurveString2.html) wires for open paths
  - finite boundary projections used only at IO/API edges
  - an internal cached bounding box for finite boundary projections
  - caller-owned metadata of type `M`, accessed through `metadata`, `metadata_mut`, `set_metadata`, `with_metadata`, and `map_metadata`

`Profile<M>` provides methods for working with planar XY shapes made of filled regions and open paths.
Prefer `Profile::from_region`, `Profile::from_wires`, `Profile::from_region_and_wires`,
`as_region`, `wires`, `into_region_and_wires`, `region_profiles`, and
`wire_polylines` so topology stays in hypercurve objects instead of a finite
boundary projection. The old `geo`-based Profile constructors and accessors
are no longer public API, and the crate no longer depends on `geo`.
`Profile` values are triangulated with hypertri when exported as STL or converted into a `Mesh<M>`.

### 2D Shapes in Profile

- <img src="docs/square.png" width="128" alt="top down view of a square"/> **`Profile::square(width: Real, metadata: M)`**
- <img src="docs/rectangle.png" width="128" alt="top down view of a rectangle"/> **`Profile::rectangle(width: Real, length: Real, metadata: M)`**
- <img src="docs/circle.png" width="128" alt="top down view of a circle"/> **`Profile::circle(radius: Real, segments: usize, metadata: M)`**
- <img src="docs/polygon.png" width="128" alt="top down view of a triangle"/> **`Profile::polygon(&[[x1,y1],[x2,y2],...], metadata: M)`**
- <img src="docs/rounded_rectangle.png" width="128" alt="top down view of a rectangle with rounded corners"/> **`Profile::rounded_rectangle(width: Real, height: Real, corner_radius: Real, corner_segments: usize, metadata: M)`**
- <img src="docs/ellipse.png" width="128" alt="top down view of an ellipse"/> **`Profile::ellipse(width: Real, height: Real, segments: usize, metadata: M)`**
- <img src="docs/regular_ngon.png" width="128" alt="top down view of a 6 sided n-gon"/> **`Profile::regular_ngon(sides: usize, radius: Real, metadata: M)`**
- <img src="docs/sketch_arrow.png" width="128" alt="top down view of a 2D arrow"/> **`Profile::arrow(shaft_length: Real, shaft_width: Real, head_length: Real, head_width: Real, metadata: M)`**
- <img src="docs/right_triangle.png" width="128" alt="top down view of a right triangle"/> **`Profile::right_triangle(width: Real, height: Real, metadata: M)`**
- <img src="docs/trapezoid.png" width="128" alt="top down view of trapezoid"/> **`Profile::trapezoid(top_width: Real, bottom_width: Real, height: Real, top_offset: Real, metadata: M)`**
- <img src="docs/star.png" width="128" alt="top down view of star"/> **`Profile::star(num_points: usize, outer_radius: Real, inner_radius: Real, metadata: M)`**
- <img src="docs/teardrop.png" width="128" alt="top down view of a teardrop"/> **`Profile::teardrop(width: Real, height: Real, segments: usize, metadata: M)`**
- <img src="docs/sketch_egg.png" width="128" alt="top down view of an egg shape"/> **`Profile::egg(width: Real, length: Real, segments: usize, metadata: M)`**
- <img src="docs/squircle.png" width="128" alt="top down view of a squircle"/> **`Profile::squircle(width: Real, height: Real, segments: usize, metadata: M)`**
- <img src="docs/keyhole.png" width="128" alt="top down view of a keyhole"/> **`Profile::keyhole(circle_radius: Real, handle_width: Real, handle_height: Real, segments: usize, metadata: M)`**
- <img src="docs/reuleaux.png" width="128"/> **`Profile::reuleaux(sides: usize, radius: Real, arc_segments_per_side: usize, metadata: M)`**
- <img src="docs/ring.png" width="128" alt="top down view of a ring"/> **`Profile::ring(id: Real, thickness: Real, segments: usize, metadata: M)`**
- <img src="docs/pie_slice.png" width="128" alt="top down view of a slice of a circle"/> **`Profile::pie_slice(radius: Real, start_angle_deg: Real, end_angle_deg: Real, segments: usize, metadata: M)`**
- <img src="docs/supershape.png" width="128"/> **`Profile::supershape(a: Real, b: Real, m: Real, n1: Real, n2: Real, n3: Real, segments: usize, metadata: M)`**
- <img src="docs/circle_with_keyway.png" width="128" alt="top down view of a circle with a notch taken out of it"/> **`Profile::circle_with_keyway(radius: Real, segments: usize, key_width: Real, key_depth: Real, metadata: M)`**
- <img src="docs/circle_with_flat.png" width="128" alt="top down view of a circle with a flat edge"/> **`Profile::circle_with_flat(radius: Real, segments: usize, flat_dist: Real, metadata: M)`**
- <img src="docs/circle_with_two_flats.png" width="128" alt="top down view of a circle with two flat edges"/> **`Profile::circle_with_two_flats(radius: Real, segments: usize, flat_dist: Real, metadata: M)`**
- <img src="docs/from_image.png" width="128" alt="top down view of a pixleated circle"/> **`Profile::from_image(img: &GrayImage, threshold: u8, closepaths: bool, metadata: M)`** - Builds a new CSG from the “on” pixels of a grayscale image
- <img src="docs/text.png" width="128" alt="top down view of the text 'HELLO'"/> **`Profile::text(text: &str, font_data: &[u8], size: Real, metadata: M)`** - generate 2D text geometry in the XY plane from TTF fonts
- <img src="docs/metaballs_2d.png" width="128" alt="top down view of three metaballs merged"/> **`Profile::metaballs(balls: &[(hypercurve::Point2, Real)], resolution: (usize, usize), iso_value: Real, padding: Real, metadata: M)`**
- <img src="docs/airfoil_naca4.png" width="128" alt="a side view of an airfoil"/> **`Profile::airfoil_naca4(max_camber: Real, camber_position: Real, thickness: Real, chord: Real, samples: usize, metadata: M)`** - [NACA 4 digit](https://en.wikipedia.org/wiki/NACA_airfoil#Four-digit_series) airfoil
- <img src="docs/bezier.png" width="128" alt="top down view of a bezier curve"/> **`Profile::bezier(control: &[[Real; 2]], segments: usize, metadata: M)`**
- <img src="docs/bspline.png" width="128" alt="top down view of a neer semi-circle shape"/> **`Profile::bspline(control: &[[Real; 2]], p: usize, segments_per_span: usize, metadata: M)`**
- <img src="docs/heart.png" width="128" alt="top down view of a cartune heart"/> **`Profile::heart(width: Real, height: Real, segments: usize, metadata: M)`**
- <img src="docs/crescent.png" width="128" alt="top down view of a crescent"/> **`Profile::crescent(outer_r: Real, inner_r: Real, offset: Real, segments: usize, metadata: M)`** -
- <img src="docs/hilbert_curve.png" width="128" alt="top down view of a hilbert curve"/> **`Profile::hilbert(order: usize, padding: Real)`** - fill an existing Profile with a hilbert curve
- <img src="docs/involute_gear.png" width="128" alt="top down view of a involute gear profile"/> **`Profile::involute_gear(module: Real, teeth: usize, pressure_angle_deg: Real, clearance: Real, backlash: Real, segments_per_flank: usize, metadata: M)`**
- **`Profile::cycloidal_gear(module_: Real, teeth: usize, pin_teeth: usize, clearance: Real, segments_per_flank: usize, metadata: M)`** - under construction
- **`Profile::involute_rack(module_: Real, num_teeth: usize, pressure_angle_deg: Real, clearance: Real, backlash: Real, metadata: M)`** - under construction
- **`Profile::cycloidal_rack(module_: Real, num_teeth: usize, generating_radius: Real, clearance: Real, segments_per_flank: usize, metadata: M)`** - under construction

```rust
// Alias the library’s generic Profile type with unit metadata:
type Profile = csgrs::profile::Profile<()>;
use csgrs::Real;

let square = Profile::square(Real::one(), ());
let rect = Profile::rectangle(Real::from(2), Real::from(4), ());
let circle = Profile::circle(Real::one(), 32, ());
let circle2 = Profile::circle(Real::from(2), 64, ());

let font_data = include_bytes!("../fonts/MyFont.ttf");
let sketch_text = Profile::text("Hello!", font_data, Real::from(20), ());

// Then extrude the text to make it 3D:
let text_3d = sketch_text.extrude(Real::one());
```

### Extrusions and Revolves

Extrusions build 3D polygons from 2D Geometries.

- <img src="docs/extrude.png" width="128" alt="an angled view of an extruded star"/> **`Profile::extrude(height: Real)`** - Simple extrude in Z+
- <img src="docs/extrude_vector.png" width="128"  alt="an angled view of a star extruded at an angle"/> **`Profile::extrude_vector(direction: Vector3)`** - Extrude along Vector3 direction
- <img src="docs/revolve.png" width="128"  alt="an arch with round ends"/> **`Profile::revolve(angle_degs, segments)`** - Extrude while rotating around the Y axis
- <img src="docs/loft.png" width="128" alt="an angled view of a lofted tapered prism"/> **`Profile::loft(&bottom_polygon, &top_polygon, false)`** - Helper function which extrudes between two Mesh Polygons, optionally with caps
- <img src="docs/sweep.png" width="128" alt="a Profile swept along a 3D path"/> **`Profile::sweep(path: &[Point3<Real>])`** - Sweep a Profile along a path defined by a series of Points

```rust
let square = Profile::square(2.0, ());
let prism = square.extrude(5.0);

let revolve_shape = square.revolve(360.0, 16);

let bottom = Profile::circle(2.0, 64, ());
let top = bottom.translate(0.0, 0.0, 5.0);
let lofted = Profile::loft(&bottom.polygons[0], &top.polygons[0], false);
```

### Misc Profile operations

- **`Profile::offset(distance)`** - outward (or inward) offset in 2D. Certified simple sharp offsets use hypercurve directly; remaining regularized offset cases are behind the optional `offset` compatibility feature and are recomposed into native Profile topology.
- **`Profile::offset_rounded(distance)`** - outward (or inward) rounded offset in 2D behind the optional `offset` compatibility feature, with results recomposed into native Profile topology.
- **`Profile::straight_skeleton(&self, orientation: bool)`** - returns a Profile containing the inside (orientation: true) or outside (orientation: false) straight skeleton behind the optional `offset` compatibility feature.
- **`Profile::bounding_box()`** - computes the bounding box of the shape.
- **`Profile::invalidate_bounding_box()`** - invalidates the bounding box of the shape, causing it to be recomputed on next access
- **`Profile::triangulate()`** - subdivides the Profile into triangles

### Mesh Structure

- **`Mesh<M>`** is the type which stores and manipulates 3D polygonal geometry.  It contains:
  - a `Vec<Polygon<M>>` polygons, describing 3D shapes, each `Polygon<M>` holds:
    - a `Vec<Vertex>` (positions + normals),
    - a `Plane` describing the polygon’s orientation in 3D.
    - a lazily cached polygon bounding box (`OnceLock<Aabb>`) used by boolean and query code.
    - a metadata field of type `M` defined by you
  - a bounding box wrapped in a OnceLock (bounding_box: OnceLock<Aabb>)
  - lazily retained connectivity/query state where repeated operations benefit
  - another metadata field of type `M` also defined by you

`Mesh<M>` provides methods for working with 3D shapes. You can build a
`Mesh<M>` from polygons with `Mesh::from_polygons(...)`.
Polygons must be closed, planar, and have 3 or more vertices.
Polygons are triangulated when being exported or converted to query meshes.

### 3D Shapes in Mesh

- <img src="docs/cube.png" width="128" alt="an angled view of a cube"/> **`Mesh::cube(width: Real, metadata: M)`**
- <img src="docs/cuboid.png" width="128" alt="an angled view of a cuboid"/> **`Mesh::cuboid(width: Real, length: Real, height: Real, metadata: M)`**
- <img src="docs/sphere.png" width="128" alt="an angled view of a sphere"/> **`Mesh::sphere(radius: Real, segments: usize, stacks: usize, metadata: M)`**
- <img src="docs/cylinder.png" width="128" alt="an angled view of a cylinder"/> **`Mesh::cylinder(radius: Real, height: Real, segments: usize, metadata: M)`**
- <img src="docs/frustum.png" width="128"/> **`Mesh::frustum(radius1: Real, radius2: Real, height: Real, segments: usize, metadata: M)`** -
Construct a frustum at origin with height and `radius1` and `radius2`.
If either radius is within `tolerance()` of 0.0, a cone terminating at a point is constructed.
- <img src="docs/frustum_ptp.png" width="128"/> **`Mesh::frustum_ptp(start: Point3, end: Point3, radius1: Real, radius2: Real, segments:
usize, metadata: M)`** -
Construct a frustum from `start` to `end` with `radius1` and `radius2`.
If either radius is within `tolerance()` of 0.0, a cone terminating at a point is constructed.
- <img src="docs/polyhedron.png" width="128"/> **`Mesh::polyhedron(points: &[[Real; 3]], faces: &[Vec<usize>], metadata: M)`**
- <img src="docs/octahedron.png" width="128"/> **`Mesh::octahedron(radius: Real, metadata: M)`** -
- <img src="docs/icosahedron.png" width="128"/> **`Mesh::icosahedron(radius: Real, metadata: M)`** -
- <img src="docs/torus.png" width="128"/> **`Mesh::torus(major_r: Real, minor_r: Real, segments_major: usize, segments_minor: usize, metadata: M)`** -
- <img src="docs/mesh_egg.png" width="128"/> **`Mesh::egg(width: Real, length: Real, revolve_segments: usize, outline_segments: usize, metadata: M)`**
- <img src="docs/mesh_teardrop.png" width="128"/> **`Mesh::teardrop(width: Real, height: Real, revolve_segments: usize, shape_segments: usize, metadata: M)`**
- <img src="docs/teardrop_cylinder.png" width="128"/> **`Mesh::teardrop_cylinder(width: Real, length: Real, height: Real, shape_segments: usize, metadata: M)`**
- <img src="docs/ellipsoid.png" width="128"/> **`Mesh::ellipsoid(rx: Real, ry: Real, rz: Real, segments: usize, stacks: usize, metadata: M)`**
- <img src="docs/metaballs_3d.png" width="128"/> **`Mesh::metaballs(balls: &[MetaBall], resolution: (usize, usize, usize), iso_value: Real, padding: Real, metadata: M)`**
- <img src="docs/sdf.png" width="128"/> **`Mesh::sdf<F>(sdf: F, resolution: (usize, usize, usize), min_pt: Point3, max_pt: Point3, iso_value: Real, metadata: M)`** - Return a CSG created by meshing a signed distance field within a bounding box
- <img src="docs/mesh_arrow.png" width="128"/> **`Mesh::arrow(start: Point3, direction: Vector3, segments: usize, orientation: bool, metadata: M)`** - Create an arrow at start, pointing along direction
- <img src="docs/gyroid.png" width="128"/> **`Mesh::gyroid_solid(resolution: usize, period: Real, iso_value: Real, thickness: Real, metadata: M)`** - Generate a capped, finite-thickness Gyroid solid inside the bounding box of `self`
- <img src="docs/schwarz_p.png" width="128"/> **`Mesh::schwarz_p_solid(resolution: usize, period: Real, iso_value: Real, thickness: Real, metadata: M)`** - Generate a capped, finite-thickness Schwarz P solid inside the bounding box of `self`
- <img src="docs/schwarz_d.png" width="128"/> **`Mesh::schwarz_d_solid(resolution: usize, period: Real, iso_value: Real, thickness: Real, metadata: M)`** - Generate a capped, finite-thickness Schwarz D solid inside the bounding box of `self`
- <img src="docs/spur_gear_involute.png" width="128"/> **`Mesh::spur_gear_involute(module: Real, teeth: usize, pressure_angle_deg: Real, clearance: Real, backlash: Real, segments_per_flank: usize, thickness: Real, helix_angle_deg: Real, slices: usize, metadata: M,)`** - Generate an involute spur gear
- **`Mesh::helical_involute_gear(module_: Real, teeth: usize, pressure_angle_deg: Real, clearance: Real, backlash: Real, segments_per_flank: usize, thickness: Real, helix_angle_deg: Real, slices: usize, metadata: M)`** - under construction

```rust
// Unit cube at origin, no metadata
let cube = Mesh::cube(1.0, ());

// Sphere of radius=2 at origin with 32 segments and 16 stacks
let sphere = Mesh::sphere(2.0, 32, 16, ());

// Cylinder from radius=1, height=2, 16 segments, and no metadata
let cyl = Mesh::cylinder(1.0, 2.0, 16, ());

// Create a custom polyhedron from points and face indices:
let points = &[
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.5, 0.5, 1.0],
];
let faces = vec![
    vec![0, 1, 2, 3], // base rectangle
    vec![0, 1, 4],    // triangular side
    vec![1, 2, 4],
    vec![2, 3, 4],
    vec![3, 0, 4],
];
let pyramid = Mesh::<()>::polyhedron(points, &faces, ());

// Metaballs https://en.wikipedia.org/wiki/Metaballs
use csgrs::mesh::metaballs::MetaBall;
let balls = vec![
    MetaBall::new(Point3::origin(), 1.0),
    MetaBall::new(Point3::new(1.5, 0.0, 0.0), 1.0),
];

let resolution = (60, 60, 60);
let iso_value = 1.0;
let padding = 1.0;

let metaball_csg = Mesh::<()>::metaballs(
    &balls,
    resolution,
    iso_value,
    padding,
    (),
);

// Example Signed Distance Field for a sphere of radius 1.5 centered at (0,0,0)
let my_sdf = |p: &Point3<Real>| p.coords.norm() - 1.5;

let resolution = (60, 60, 60);
let min_pt = Point3::new(-2.0, -2.0, -2.0);
let max_pt = Point3::new( 2.0,  2.0,  2.0);
let iso_value = 0.0; // Typically zero for SDF-based surfaces

let csg_shape = Mesh::<()>::sdf(my_sdf, resolution, min_pt, max_pt, iso_value, ());
```

### CSG Boolean Operations

```rust
let union_result = cube.try_union(&sphere)?;
let difference_result = cube.try_difference(&sphere)?;
let intersection_result = cylinder.try_intersection(&sphere)?;
let xor_result = cylinder.try_xor(&sphere)?;
```

`Mesh<M>` and `Profile<M>` provide typed `try_union`, `try_difference`,
`try_intersection`, and `try_xor` methods. The methods on `csgrs::csg::CSG`
remain compatibility conveniences and panic when a certified Boolean reports
an error or uncertainty.

Types implementing the CSG trait also provide the following transformation functions:

### Transformations

- **`::translate(x: Real, y: Real, z: Real)`** - Returns the CSG translated by x, y, and z
- **`::translate_vector(vector: Vector3)`** - Returns the CSG translated by vector
- **`::rotate(x_deg, y_deg, z_deg)`** - Returns the CSG rotated in x, y, and z
- **`::scale(scale_x, scale_y, scale_z)`** - Returns the CSG scaled in x, y, and z
- **`::mirror(plane: Plane)`** - Returns the CSG mirrored across plane
- **`::center()`** - Returns the CSG centered at the origin
- **`::float()`** - Returns the CSG translated so that its bottommost point(s) sit exactly at z=0
- **`::transform(&Matrix4)`** - Returns the CSG after applying arbitrary affine transforms
- <img src="docs/distribute_arc.png" width="128"/> **`::distribute_arc(count: usize, radius: Real, start_angle_deg: Real, end_angle_deg: Real)`**
- <img src="docs/distribute_linear.png" width="128"/> **`::distribute_linear(count: usize, dir: hyperlattice::Vector3, spacing: Real)`**
- <img src="docs/distribute_grid.png" width="128"/> **`::distribute_grid(rows: usize, cols: usize, dx: Real, dy: Real)`**
- <img src="docs/inverse.png" width="128"/> **`::inverse()`** - flips the inside/outside orientation.

```rust
use hyperlattice::{Real, Vector3};
use csgrs::mesh::plane::Plane;
use csgrs::csg::CSG;

fn r(value: f64) -> Real {
    Real::try_from(value).expect("finite example scalar")
}

let moved = cube.translate(r(3.0), r(0.0), r(0.0));
let moved2 = cube.translate_vector(Vector3::from_xyz(r(3.0), r(0.0), r(0.0)));
let rotated = sphere.rotate(r(0.0), r(45.0), r(90.0));
let scaled = cylinder.scale(r(2.0), r(1.0), r(1.0));
let plane_x = Plane::from_normal(Vector3::x(), r(0.0)); // x=0 plane
let plane_y = Plane { normal: Vector3::y(), w: 0.0 }; // y=0 plane
let plane_z = Plane { normal: Vector3::z(), w: 0.0 }; // z=0 plane
let mirrored = cube.mirror(plane_x);
```

### Miscellaneous Mesh Operations

- **`Mesh::vertices()`** - collect all vertices from the `Mesh`
- <img src="docs/convex_hull.png" width="128"/> **`Mesh::convex_hull()`** - uses [`chull`](https://crates.io/crates/chull) to generate a 3D convex hull.
- <img src="docs/minkowski_sum.png" width="128"/> **`Mesh::minkowski_sum(&other)`** - naive Minkowski sum, then takes the hull.
- **`Mesh::ray_intersections(origin, direction)`** — returns all intersection points and distances.
- **`Mesh::flatten()`** - flattens a 3D shape into 2D (on the XY plane), unions the outlines.
- **`Mesh::slice(plane)`** - slices the CSG by a plane and returns the cross-section polygons.
- <img src="docs/subdivide_triangles.png" width="128"/> **`Mesh::subdivide_triangles(subdivisions)`** - subdivides each polygon’s triangles, increasing mesh density.
- **`Mesh::renormalize()`** - re-computes each polygon’s plane from its vertices, resetting all normals.
- **`Mesh::bounding_box()`** - computes the bounding box of the shape.
- **`Mesh::invalidate_bounding_box()`** - invalidates the bounding box of the shape, causing it to be recomputed on next access
- **`Mesh::triangulate()`** - triangulates all polygons returning a CSG containing triangles.
- **`Mesh::from_polygons(polygons: &[Polygon<M>])`** - create a new CSG from Polygons.

### STL

- **Export ASCII STL**: `mesh.to_stl_ascii("solid_name") -> Result<String, IoError>`
- **Export Binary STL**: `mesh.to_stl_binary("solid_name") -> Result<Vec<u8>, IoError>`
- **Import STL**: `Mesh::from_stl(&stl_data, metadata) -> Result<Mesh<M>, IoError>`

```rust
// Save to ASCII STL
let stl_text = csg_union.to_stl_ascii("union_solid")?;
std::fs::write("union_ascii.stl", stl_text).unwrap();

// Save to binary STL
let stl_bytes = csg_union.to_stl_binary("union_solid")?;
std::fs::write("union_bin.stl", stl_bytes)?;

// Load from an STL file on disk
let file_data = std::fs::read("some_file.stl")?;
let imported_mesh = Mesh::from_stl(&file_data, ())?;
```

### DXF

- **Export**: `mesh.to_dxf() -> Result<Vec<u8>, IoError>`
- **Import**: `Mesh::from_dxf(&dxf_data, metadata) -> Result<Mesh<M>, IoError>`

```rust
// Export DXF
let dxf_bytes = csg_obj.to_dxf()?;
std::fs::write("output.dxf", dxf_bytes)?;

// Import DXF
let dxf_data = std::fs::read("some_file.dxf")?;
let mesh_dxf = Mesh::from_dxf(&dxf_data, ())?;
```

### Other interchange formats

All interchange exporters are fallible. They reject non-finite or unrepresentable
coordinates and metadata that cannot be encoded safely instead of substituting
values or emitting a partial document.

- **OBJ**: `mesh.to_obj(name)` and `Mesh::from_obj(reader, metadata)`
- **PLY**: `mesh.to_ply(comment)`
- **AMF**: `mesh.to_amf(name, units)` and `mesh.to_amf_with_color(name, units, color)`
- **glTF 2.0**: `mesh.to_gltf(name)`
- **SVG profiles**: `Profile::from_svg(document, metadata)` and `profile.to_svg()`
- **Gerber profiles**: `Profile::from_gerber(data, metadata)` and `profile.to_gerber()`

Each method returns `Result<_, csgrs::io::IoError>`. OBJ import deliberately
rejects texture-coordinate data, STL import validates closed and consistently
oriented topology, and DXF import reports unsupported entity types explicitly.

### Hershey Text

Hershey fonts are single stroke fonts which produce open ended polylines in the XY plane via [`hershey`](https://crates.io/crates/hershey):

```rust
let font_data = include_bytes("../fonts/myfont.jhf");
let csg_text = Profile::from_hershey("Hello!", font_data, 20.0, ());
```

### Create a Bevy `Mesh`

`csg.to_bevy_mesh()` returns a Bevy [`Mesh`](https://docs.rs/bevy/latest/bevy/prelude/struct.Mesh.html).

```rust
use bevy::{prelude::*, render::render_asset::RenderAssetUsages, render::mesh::{Indices, PrimitiveTopology}};

let bevy_mesh = mesh_obj.to_bevy_mesh();
```

### Mesh Queries

Mesh queries such as `Mesh::intersect_polyline` and `Mesh::contains_vertex`
operate on hyperlattice points and vectors directly.

### Mass Properties

```rust
let density = 1.0;
let (mass, com, inertia_frame) = mesh_obj.mass_properties(density);
println!("Mass: {}", mass);
println!("Center of Mass: {:?}", com);
println!("Inertia local frame: {:?}", inertia_frame);
```

### Manifold Check

`mesh.is_manifold()` triangulates the CSG, builds a HashMap of all edges (pairs of vertices), and checks that each is used exactly twice. Returns `true` if manifold, `false` if not.

```rust
if (mesh_obj.is_manifold()){
    println!("Mesh is manifold!");
} else {
    println!("Not manifold.");
}
```

## Tolerance

Primitive `f32`/`f64` values are limited to IO, rendering, and JS boundary
adapters, then promoted into hyperreal-backed geometry for topology-sensitive
work. There are no Cargo precision flags and no global float tolerance setter.

## Working with Metadata

`Mesh<M>` and `Profile<M>` are generic over `M: Clone`. Each polygon in a `Mesh<M>` and each `Mesh<M>` and `Profile<M>` carry caller-owned metadata. Use `M = ()` for no metadata or `M = Option<YourMetadata>` for optional metadata.
Use cases include storing color, ID, or layer info.

```rust
use csgrs::polygon::Polygon;
use csgrs::vertex::Vertex;
use hyperlattice::{Point3, Real, Vector3};

#[derive(Clone)]
struct MyMetadata {
    color: (u8, u8, u8),
    label: String,
}

type Mesh = csgrs::mesh::Mesh<Option<MyMetadata>>;

// For a single polygon:
let mut poly = Polygon::new(
    vec![
        Vertex::new(Point3::origin(), Vector3::z()),
        Vertex::new(Point3::new(1.0, 0.0, 0.0), Vector3::z()),
        Vertex::new(Point3::new(0.0, 1.0, 0.0), Vector3::z()),
    ],
    Some(MyMetadata {
        color: (255, 0, 0),
        label: "Triangle".into(),
    }),
);

// Retrieve metadata
if let Some(data) = poly.metadata() {
    println!("This polygon is labeled {}", data.label);
}

// Mutate metadata
if let Some(data_mut) = poly.metadata_mut() {
    data_mut.label.push_str("_extended");
}
```

## Examples
- [csgrs-bevy-example](https://github.com/timschmidt/csgrs-bevy-example)
- [csgrs-egui-example](https://github.com/timschmidt/csgrs-egui-example)
- [csgrs-egui-wasm-example](https://github.com/timschmidt/csgrs-egui-wasm-example)
- [csgrs-druid-example](https://github.com/timschmidt/csgrs-druid-example)

## Project Status

`csgrs` is usable today as a Rust-first CSG and geometry toolkit. The stable
core is polygon-backed `Mesh` with hypermesh Boolean conversion,
hypercurve-backed `Profile`, hypertri polygon
triangulation, common 2D/3D primitive construction, extrusion/revolve/sweep/loft
operations, transformations, metadata propagation, cached bounding boxes,
TriMesh query conversion, ray intersection helpers, and
import/export for common mesh and manufacturing formats behind Cargo features.

The project is also intentionally experimental in several areas:

- **Boolean kernels**: `Mesh` booleans now route through `hypermesh` rather
  than a csgrs-owned partition tree. Nontrivial `union`, `difference`,
  `intersection`, and `xor` calls import both operands as `hypermesh::Mesh` and
  materialize the hypermesh result directly. Use the `try_*` boolean methods
  when callers need the typed hypermesh import, operation, or materialization
  error; those errors are not hidden by a legacy mesh-boolean fallback.
- **Triangulation**: hypertri is the single triangulation backend. It replaces
  the former Spade, Earcut, and `delaunay` dependency matrix.
- **Numeric model**: hyperreals are the core scalar model. Primitive `f32` and
  `f64` entry points live at IO, JavaScript, FFI, and `csgrs-adapter`
  boundaries, which promote values before topology-sensitive work.
- **Curves and offsets**: hypercurve is the arc-preserving 2D backend. The
  former Curvo-backed NURBS integration layer has been removed, and remaining
  finite offset compatibility is isolated behind the optional `offset` feature.
- **WASM**: JavaScript bindings cover the core mesh/sketch APIs and are useful
  for browser previews, but the Rust API remains the source of truth.

## README Renders

The images in this README are generated by the pure-Rust example renderer:

```shell
cargo run --example readme_renders
```

The generator writes every referenced PNG under `docs/` and intentionally covers
each documented primitive or operation that has a visual representation. It uses
the existing `image` dependency through the `image-io` feature, frames meshes
with cached bounding boxes, and deliberately uses a slower projected-triangle
software path for consistent README previews.

To render into a scratch directory without touching the checked-in baselines,
set `README_RENDER_OUTPUT_DIR`:

```shell
README_RENDER_OUTPUT_DIR=/tmp/csgrs-readme-renders cargo run --example readme_renders
```

## Roadmap

- **Boolean robustness**: extend the direct `hypermesh` solid boolean path with
  more topology coverage and add indexed mesh conversion/merge utilities.
- **Numeric backend**: continue pushing topology-sensitive predicates into
  hyperreal-backed hyper geometry, while keeping primitive `f32`/`f64`
  conversions at API and IO boundaries.
- **Triangulation and polygon validity**: finish T-junction repair, coplanar
  polygon merging, validation hooks, and richer hypertri constrained
  triangulation integration.
- **Curves and offsets**: harden Bezier, B-spline, hypercurve offsets,
  straight skeleton, and TrueType text paths.
- **Feature-complete modeling operations**: add rounded cuboids, chamfers,
  fillets, 3D offsets, bending, threaded parts, richer gear options, and
  attachment/alignment helpers.
- **Representations**: evaluate native indexed `hypermesh` storage where it
  improves ownership and conversion costs, keep `Profile` backed by hypercurve,
  add metadata deduplication, UV support,
  and better conversion among hyper geometry types, query `TriMesh`, and file
  formats.
- **Performance**: broaden Rayon use in operations that are embarrassingly
  parallel, preserve borrowed-slice APIs, minimize allocations, and use cached
  AABBs/TriMesh acceleration structures in query-heavy code.
- **I/O and integration**: improve glTF/Gerber output, add STEP/IGES research,
  continue Bevy/WASM interop, and keep boundary bindings aligned with the Rust API.
- **Testing**: keep expanding adversarial, fuzz, and property tests around
  degeneracy, invalid geometry, boolean edge cases, extrusion/sweep/revolve,
  import/export round trips, and tolerance boundaries.

## References

> Chernyaev, Evgeni. “Marching Cubes 33: Construction of Topologically Correct
> Isosurfaces.” *CERN Document Server*, 1995,
> https://cds.cern.ch/record/292771.

> “CSG.js.” *GitHub*, Evan Wallace, 2011,
> https://github.com/evanw/csg.js.

> de Berg, Mark, et al. *Computational Geometry: Algorithms and Applications*.
> 3rd ed., Springer, 2008.

> Meisters, Gary H. “Polygons Have Ears.” *The American Mathematical Monthly*,
> vol. 82, no. 6, 1975, pp. 648-651, https://doi.org/10.2307/2319703.

> Fabien Sanglard. “Floating Point Visually Explained.” *Fabien Sanglard's
> Website*, 2020, https://fabiensanglard.net/floating_point_visually_explained/.

> Farouki, Rida T. “The Bernstein Polynomial Basis: A Centennial Retrospective.”
> *Computer Aided Geometric Design*, vol. 29, no. 6, 2012, pp. 379-419.

> Held, Martin. “FIST: Fast Industrial-Strength Triangulation of Polygons.”
> *Algorithmica*, vol. 30, no. 4, 2001, pp. 563-596.

> Hershey, A. V. *Calligraphy for Computers*. U.S. Naval Weapons Laboratory,
> 1967.

> Lorensen, William E., and Harvey E. Cline. “Marching Cubes: A High Resolution
> 3D Surface Construction Algorithm.” *Computer Graphics*, vol. 21, no. 4, 1987,
> pp. 163-169.

> Patrikalakis, Nicholas M., Takashi Maekawa, and Wonjoon Cho. *Shape
> Interrogation for Computer Aided Design and Manufacturing*. Hyperbook ed., MIT,
> https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/.

> Pharr, Matt, Wenzel Jakob, and Greg Humphreys. *Physically Based Rendering:
> From Theory to Implementation*. 4th ed., MIT Press, 2023,
> https://pbr-book.org/.

> Prade, Inigo Quilez. “Distance Functions.” *Inigo Quilez*,
> https://iquilezles.org/articles/distfunctions/.

> Schoen, Alan H. *Infinite Periodic Minimal Surfaces without Self-Intersections*.
> NASA Technical Note D-5541, 1970.

> Shewchuk, Jonathan Richard. “Adaptive Precision Floating-Point Arithmetic and
> Fast Robust Geometric Predicates.” *Discrete & Computational Geometry*, vol.
> 18, no. 3, 1997, pp. 305-363, https://doi.org/10.1007/PL00009321.

> Shewchuk, Jonathan Richard. “Robust Adaptive Floating-Point Geometric
> Predicates.” *Proceedings of the Twelfth Annual Symposium on Computational
> Geometry*, ACM, 1996, pp. 141-150.

> Yap, Chee K. “Towards Exact Geometric Computation.” *Computational Geometry*,
> vol. 7, nos. 1-2, 1997, pp. 3-23,
> https://doi.org/10.1016/0925-7721(95)00040-2.

> “The OpenSCAD Language.” *OpenSCAD*, https://openscad.org/documentation.html.

> TrueType Reference Manual. *Apple Developer Documentation*,
> https://developer.apple.com/fonts/TrueType-Reference-Manual/.

## License

```
MIT License

Copyright (c) 2025 Timothy Schmidt

Permission is hereby granted, free of charge, to any person obtaining a copy of this 
software and associated documentation files (the "Software"), to deal in the Software 
without restriction, including without limitation the rights to use, copy, modify, merge, 
publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons 
to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

This library initially based on a translation of **CSG.js** © 2011 Evan Wallace, under the MIT license.  

---

If you find issues, please file an [issue](https://github.com/timschmidt/csgrs/issues) or submit a pull request. Feedback and contributions are welcome!

**Have fun building geometry in Rust!**
