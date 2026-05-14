# csgrs

A fast, optionally multithreaded **Constructive Solid Geometry (CSG)**
library in Rust, built around Boolean operations (*union*, *difference*,
*intersection*, *xor*) on several different internal geometry representations.
**csgrs** provides data structures and methods for constructing 2D and 3D geometry
with an [OpenSCAD](https://openscad.org/)-like syntax.  Our aim is for **csgrs**
to be light weight and full featured through integration with the
[Dimforge](https://www.dimforge.com/) ecosystem
(e.g., [`nalgebra`](https://crates.io/crates/nalgebra),
[`Parry`](https://crates.io/crates/parry3d),
and [`Rapier`](https://crates.io/crates/rapier3d)) and
[`geo`](https://crates.io/crates/geo) for robust processing of
[Simple Features](https://en.wikipedia.org/wiki/Simple_Features).
**csgrs** has a number of functions useful for generating CNC toolpaths.  The
library can be built for 32bit or 64bit floats, and for WASM.  Dependencies are
100% rust and nearly all optional.

[Earcut](https://docs.rs/geo/latest/geo/algorithm/triangulate_earcut/trait.TriangulateEarcut.html)
and
[constrained delaunay](https://docs.rs/geo/latest/geo/algorithm/triangulate_delaunay/trait.TriangulateDelaunay.html#method.constrained_triangulation)
and
[delaunay](https://crates.io/crates/delaunay)
algorithms used for triangulation work only in 2D, so **csgrs** rotates
3D polygons into 2D for triangulation then back to 3D. Enable exactly one
triangulation feature: `delaunay` (spade), `earcut`, or `delaunay-rs`.

![Example CSG output](docs/csg.png)

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
use csgrs::traits::CSG;

type Mesh = csgrs::mesh::Mesh<()>;

fn main() {
    // Create a cube
    let cube: Mesh = Mesh::cube(2.0, None); // 2×2×2 cube at origin, no metadata

    // Create sphere at (1, 1, 1) with radius 1.25:
    let sphere: Mesh = Mesh::sphere(1.25, 16, 8, None).translate(1.0, 1.0, 1.0);

    // Perform a difference operation:
    let result = cube.difference(&sphere);

    // Write the result as an ASCII STL:
    let stl = result.to_stl_ascii("cube_minus_sphere");
    std::fs::write("cube_sphere_difference.stl", stl).unwrap();
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

### Building for WASM

```shell
cargo install wasm-pack
wasm-pack build --release --target bundler --out-dir pkg -- --features wasm
```

## Features and Structures

### Sketch Structure

- **`Sketch<S>`** is the type which stores and manipulates 2D polygonal geometry.  It contains:
  - a [`geo`](https://crates.io/crates/geo) [`GeometryCollection<Real>`](https://docs.rs/geo/latest/geo/geometry/struct.GeometryCollection.html)
  - a bounding box wrapped in a OnceLock (bounding_box: OnceLock<Aabb>)
  - an optional metadata field (`Option<S>`) also defined by you

`Sketch<S>` provides methods for working with 2D shapes made of points and lines.
You can build a `Sketch<S>` from geo Geometries with `Sketch::from_geo(...)`.
Geometries can be open or closed, and can have holes, but must be planar in the XY.
`Sketch`'s are triangulated when exported as an STL, or when a Geometry is
converted into a `Mesh<S>`.

### 2D Shapes in Sketch

- <img src="docs/square.png" width="128" alt="top down view of a square"/> **`Sketch::square(width: Real, metadata: Option<S>)`**
- <img src="docs/rectangle.png" width="128" alt="top down view of a rectangle"/> **`Sketch::rectangle(width: Real, length: Real, metadata: Option<S>)`**
- <img src="docs/circle.png" width="128" alt="top down view of a circle"/> **`Sketch::circle(radius: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/polygon.png" width="128" alt="top down view of a triangle"/> **`Sketch::polygon(&[[x1,y1],[x2,y2],...], metadata: Option<S>)`**
- <img src="docs/rounded_rectangle.png" width="128" alt="top down view of a rectangle with rounded corners"/> **`Sketch::rounded_rectangle(width: Real, height: Real, corner_radius: Real, corner_segments: usize, metadata: Option<S>)`**
- <img src="docs/ellipse.png" width="128" alt="top down view of an ellipse"/> **`Sketch::ellipse(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/regular_ngon.png" width="128" alt="top down view of a 6 sided n-gon"/> **`Sketch::regular_ngon(sides: usize, radius: Real, metadata: Option<S>)`**
- <img src="docs/sketch_arrow.png" width="128" alt="top down view of a 2D arrow"/> **`Sketch::arrow(shaft_length: Real, shaft_width: Real, head_length: Real, head_width: Real, metadata: Option<S>)`**
- <img src="docs/right_triangle.png" width="128" alt="top down view of a right triangle"/> **`Sketch::right_triangle(width: Real, height: Real, metadata: Option<S>)`**
- <img src="docs/trapezoid.png" width="128" alt="top down view of trapezoid"/> **`Sketch::trapezoid(top_width: Real, bottom_width: Real, height: Real, top_offset: Real, metadata: Option<S>)`**
- <img src="docs/star.png" width="128" alt="top down view of star"/> **`Sketch::star(num_points: usize, outer_radius: Real, inner_radius: Real, metadata: Option<S>)`**
- <img src="docs/teardrop.png" width="128" alt="top down view of a teardrop"/> **`Sketch::teardrop(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/sketch_egg.png" width="128" alt="top down view of an egg shape"/> **`Sketch::egg(width: Real, length: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/squircle.png" width="128" alt="top down view of a squircle"/> **`Sketch::squircle(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/keyhole.png" width="128" alt="top down view of a keyhole"/> **`Sketch::keyhole(circle_radius: Real, handle_width: Real, handle_height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/reuleaux.png" width="128"/> **`Sketch::reuleaux(sides: usize, radius: Real, arc_segments_per_side: usize, metadata: Option<S>)`**
- <img src="docs/ring.png" width="128" alt="top down view of a ring"/> **`Sketch::ring(id: Real, thickness: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/pie_slice.png" width="128" alt="top down view of a slice of a circle"/> **`Sketch::pie_slice(radius: Real, start_angle_deg: Real, end_angle_deg: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/supershape.png" width="128"/> **`Sketch::supershape(a: Real, b: Real, m: Real, n1: Real, n2: Real, n3: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/circle_with_keyway.png" width="128" alt="top down view of a circle with a notch taken out of it"/> **`Sketch::circle_with_keyway(radius: Real, segments: usize, key_width: Real, key_depth: Real, metadata: Option<S>)`**
- <img src="docs/circle_with_flat.png" width="128" alt="top down view of a circle with a flat edge"/> **`Sketch::circle_with_flat(radius: Real, segments: usize, flat_dist: Real, metadata: Option<S>)`**
- <img src="docs/circle_with_two_flats.png" width="128" alt="top down view of a circle with two flat edges"/> **`Sketch::circle_with_two_flats(radius: Real, segments: usize, flat_dist: Real, metadata: Option<S>)`**
- <img src="docs/from_image.png" width="128" alt="top down view of a pixleated circle"/> **`Sketch::from_image(img: &GrayImage, threshold: u8, closepaths: bool, metadata: Option<S>)`** - Builds a new CSG from the “on” pixels of a grayscale image
- <img src="docs/text.png" width="128" alt="top down view of the text 'HELLO'"/> **`Sketch::text(text: &str, font_data: &[u8], size: Real, metadata: Option<S>)`** - generate 2D text geometry in the XY plane from TTF fonts
- <img src="docs/metaballs_2d.png" width="128" alt="top down view of three metaballs merged"/> **`Sketch::metaballs(balls: &[(nalgebra::Point2<Real>, Real)], resolution: (usize, usize), iso_value: Real, padding: Real, metadata: Option<S>)`**
- <img src="docs/airfoil_naca4.png" width="128" alt="a side view of an airfoil"/> **`Sketch::airfoil_naca4(max_camber: Real, camber_position: Real, thickness: Real, chord: Real, samples: usize, metadata: Option<S>)`** - [NACA 4 digit](https://en.wikipedia.org/wiki/NACA_airfoil#Four-digit_series) airfoil
- <img src="docs/bezier.png" width="128" alt="top down view of a bezier curve"/> **`Sketch::bezier(control: &[[Real; 2]], segments: usize, metadata: Option<S>)`**
- <img src="docs/bspline.png" width="128" alt="top down view of a neer semi-circle shape"/> **`Sketch::bspline(control: &[[Real; 2]], p: usize, segments_per_span: usize, metadata: Option<S>)`**
- <img src="docs/heart.png" width="128" alt="top down view of a cartune heart"/> **`Sketch::heart(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/crescent.png" width="128" alt="top down view of a crescent"/> **`Sketch::crescent(outer_r: Real, inner_r: Real, offset: Real, segments: usize, metadata: Option<S>)`** - 
- <img src="docs/hilbert_curve.png" width="128" alt="top down view of a hilbert curve"/> **`Sketch::hilbert(order: usize, padding: Real)`** - fill an existing Sketch with a hilbert curve
- <img src="docs/involute_gear.png" width="128" alt="top down view of a involute gear profile"/> **`Sketch::involute_gear(module: Real, teeth: usize, pressure_angle_deg: Real, clearance: Real, backlash: Real, segments_per_flank: usize, metadata: Option<S>)`**
- **`Sketch::cycloidal_gear(module_: Real, teeth: usize, pin_teeth: usize, clearance: Real, segments_per_flank: usize, metadata: Option<S>)`** - under construction
- **`Sketch::involute_rack(module_: Real, num_teeth: usize, pressure_angle_deg: Real, clearance: Real, backlash: Real, metadata: Option<S>)`** - under construction
- **`Sketch::cycloidal_rack(module_: Real, num_teeth: usize, generating_radius: Real, clearance: Real, segments_per_flank: usize, metadata: Option<S>)`** - under construction

```rust
// Alias the library’s generic Sketch type with empty metadata:
type Sketch = csgrs::sketch::Sketch<()>;

let square = Sketch::square(1.0, None); // 1×1 at origin
let rect = Sketch::rectangle(2.0, 4.0, None);
let circle = Sketch::circle(1.0, 32, None); // radius=1, 32 segments
let circle2 = Sketch::circle(2.0, 64, None);

let font_data = include_bytes!("../fonts/MyFont.ttf");
let sketch_text = Sketch::text("Hello!", font_data, 20.0, None);

// Then extrude the text to make it 3D:
let text_3d = sketch_text.extrude(1.0);
```

### Extrusions and Revolves

Extrusions build 3D polygons from 2D Geometries.

- <img src="docs/extrude.png" width="128" alt="an angled view of an extruded star"/> **`Sketch::extrude(height: Real)`** - Simple extrude in Z+
- <img src="docs/extrude_vector.png" width="128"  alt="an angled view of a star extruded at an angle"/> **`Sketch::extrude_vector(direction: Vector3)`** - Extrude along Vector3 direction
- <img src="docs/revolve.png" width="128"  alt="an arch with round ends"/> **`Sketch::revolve(angle_degs, segments)`** - Extrude while rotating around the Y axis
- <img src="docs/loft.png" width="128" alt="an angled view of a lofted tapered prism"/> **`Sketch::loft(&bottom_polygon, &top_polygon, false)`** - Helper function which extrudes between two Mesh Polygons, optionally with caps
- <img src="docs/sweep.png" width="128" alt="a Sketch swept along a 3D path"/> **`Sketch::sweep(path: &[Point3<Real>])`** - Sweep a Sketch along a path defined by a series of Points

```rust
let square = Sketch::square(2.0, None);
let prism = square.extrude(5.0);

let revolve_shape = square.revolve(360.0, 16);

let bottom = Sketch::circle(2.0, 64, None);
let top = bottom.translate(0.0, 0.0, 5.0);
let lofted = Sketch::loft(&bottom.polygons[0], &top.polygons[0], false);
```

### Misc Sketch operations

- **`Sketch::offset(distance)`** - outward (or inward) offset in 2D using [`geo-offset`](https://crates.io/crates/geo-offset).
- **`Sketch::offset_rounded(distance)`** - outward (or inward) offset in 2D using [`geo-offset`](https://crates.io/crates/geo-offset).
- **`Sketch::straight_skeleton(&self, orientation: bool)`** - returns a Sketch containing the inside (orientation: true) or outside (orientation: false) straight skeleton
- **`Sketch::bounding_box()`** - computes the bounding box of the shape.
- **`Sketch::invalidate_bounding_box()`** - invalidates the bounding box of the shape, causing it to be recomputed on next access
- **`Sketch::triangulate()`** - subdivides the Sketch into triangles

### Mesh Structure

- **`Mesh<S>`** is the type which stores and manipulates 3D polygonal geometry.  It contains:
  - a `Vec<Polygon<S>>` polygons, describing 3D shapes, each `Polygon<S>` holds:
    - a `Vec<Vertex>` (positions + normals),
    - a `Plane` describing the polygon’s orientation in 3D.
    - a lazily cached polygon bounding box (`OnceLock<Aabb>`) used by boolean and query code.
    - an optional metadata field (`Option<S>`) defined by you
  - a bounding box wrapped in a OnceLock (bounding_box: OnceLock<Aabb>)
  - a lazily built Parry `TriMesh` cache for repeated point/ray query operations
  - another optional metadata field (`Option<S>`) also defined by you

`Mesh<S>` provides methods for working with 3D shapes. You can build a
`Mesh<S>` from polygons with `Mesh::from_polygons(...)`.
Polygons must be closed, planar, and have 3 or more vertices.
Polygons are triangulated when being exported or converted to query meshes.

### 3D Shapes in Mesh

- <img src="docs/cube.png" width="128" alt="an angled view of a cube"/> **`Mesh::cube(width: Real, metadata: Option<S>)`**
- <img src="docs/cuboid.png" width="128" alt="an angled view of a cuboid"/> **`Mesh::cuboid(width: Real, length: Real, height: Real, metadata: Option<S>)`**
- <img src="docs/sphere.png" width="128" alt="an angled view of a sphere"/> **`Mesh::sphere(radius: Real, segments: usize, stacks: usize, metadata: Option<S>)`**
- <img src="docs/cylinder.png" width="128" alt="an angled view of a cylinder"/> **`Mesh::cylinder(radius: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/frustum.png" width="128"/> **`Mesh::frustum(radius1: Real, radius2: Real, height: Real, segments: usize, metadata: Option<S>)`** -
Construct a frustum at origin with height and `radius1` and `radius2`.
If either radius is within EPSILON of 0.0, a cone terminating at a point is constructed.
- <img src="docs/frustum_ptp.png" width="128"/> **`Mesh::frustum_ptp(start: Point3, end: Point3, radius1: Real, radius2: Real, segments:
usize, metadata: Option<S>)`** -
Construct a frustum from `start` to `end` with `radius1` and `radius2`.
If either radius is within EPSILON of 0.0, a cone terminating at a point is constructed.
- <img src="docs/polyhedron.png" width="128"/> **`Mesh::polyhedron(points: &[[Real; 3]], faces: &[Vec<usize>], metadata: Option<S>)`**
- <img src="docs/octahedron.png" width="128"/> **`Mesh::octahedron(radius: Real, metadata: Option<S>)`** -
- <img src="docs/icosahedron.png" width="128"/> **`Mesh::icosahedron(radius: Real, metadata: Option<S>)`** -
- <img src="docs/torus.png" width="128"/> **`Mesh::torus(major_r: Real, minor_r: Real, segments_major: usize, segments_minor: usize, metadata: Option<S>)`** -
- <img src="docs/mesh_egg.png" width="128"/> **`Mesh::egg(width: Real, length: Real, revolve_segments: usize, outline_segments: usize, metadata: Option<S>)`**
- <img src="docs/mesh_teardrop.png" width="128"/> **`Mesh::teardrop(width: Real, height: Real, revolve_segments: usize, shape_segments: usize, metadata: Option<S>)`**
- <img src="docs/teardrop_cylinder.png" width="128"/> **`Mesh::teardrop_cylinder(width: Real, length: Real, height: Real, shape_segments: usize, metadata: Option<S>)`**
- <img src="docs/ellipsoid.png" width="128"/> **`Mesh::ellipsoid(rx: Real, ry: Real, rz: Real, segments: usize, stacks: usize, metadata: Option<S>)`**
- <img src="docs/metaballs_3d.png" width="128"/> **`Mesh::metaballs(balls: &[MetaBall], resolution: (usize, usize, usize), iso_value: Real, padding: Real, metadata: Option<S>)`**
- <img src="docs/sdf.png" width="128"/> **`Mesh::sdf<F>(sdf: F, resolution: (usize, usize, usize), min_pt: Point3, max_pt: Point3, iso_value: Real, metadata: Option<S>)`** - Return a CSG created by meshing a signed distance field within a bounding box
- <img src="docs/mesh_arrow.png" width="128"/> **`Mesh::arrow(start: Point3, direction: Vector3, segments: usize, orientation: bool, metadata: Option<S>)`** - Create an arrow at start, pointing along direction
- <img src="docs/gyroid.png" width="128"/> **`Mesh::gyroid(resolution: usize, period: Real, iso_value: Real, metadata: Option<S>)`** - Generate a Triply Periodic Minimal Surface (Gyroid) inside the volume of `self`
- <img src="docs/schwarz_p.png" width="128"/> **`Mesh::schwarz_p(resolution: usize, period: Real, iso_value: Real, metadata: Option<S>)`** - Generate a Triply Periodic Minimal Surface (Schwarz P) inside the volume of `self`
- <img src="docs/schwarz_d.png" width="128"/> **`Mesh::schwarz_d(resolution: usize, period: Real, iso_value: Real, metadata: Option<S>)`** - Generate a Triply Periodic Minimal Surface (Schwarz D) inside the volume of `self`
- <img src="docs/spur_gear_involute.png" width="128"/> **`Mesh::spur_gear_involute(module: Real, teeth: usize, pressure_angle_deg: Real, clearance: Real, backlash: Real, segments_per_flank: usize, thickness: Real, helix_angle_deg: Real, slices: usize, metadata: Option<S>,)`** - Generate an involute spur gear
- **`Mesh::helical_involute_gear(module_: Real, teeth: usize, pressure_angle_deg: Real, clearance: Real, backlash: Real, segments_per_flank: usize, thickness: Real, helix_angle_deg: Real, slices: usize, metadata: Option<S>)`** - under construction

```rust
// Unit cube at origin, no metadata
let cube = Mesh::cube(1.0, None);

// Sphere of radius=2 at origin with 32 segments and 16 stacks
let sphere = Mesh::sphere(2.0, 32, 16, None);

// Cylinder from radius=1, height=2, 16 segments, and no metadata
let cyl = Mesh::cylinder(1.0, 2.0, 16, None);

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
let pyramid = Mesh::polyhedron(points, &faces, None);

// Metaballs https://en.wikipedia.org/wiki/Metaballs
use csgrs::mesh::metaballs::MetaBall;
let balls = vec![
    MetaBall::new(Point3::origin(), 1.0),
    MetaBall::new(Point3::new(1.5, 0.0, 0.0), 1.0),
];

let resolution = (60, 60, 60);
let iso_value = 1.0;
let padding = 1.0;

let metaball_csg = CSG::from_metaballs(
    &balls,
    resolution,
    iso_value,
    padding,
    None,
);

// Example Signed Distance Field for a sphere of radius 1.5 centered at (0,0,0)
let my_sdf = |p: &Point3<Real>| p.coords.norm() - 1.5;

let resolution = (60, 60, 60);
let min_pt = Point3::new(-2.0, -2.0, -2.0);
let max_pt = Point3::new( 2.0,  2.0,  2.0);
let iso_value = 0.0; // Typically zero for SDF-based surfaces

let csg_shape = Mesh::from_sdf(my_sdf, resolution, min_pt, max_pt, iso_value, None);
```

### CSG Boolean Operations

```rust
use csgrs::traits::CSG;

let union_result = cube.union(&sphere);
let difference_result = cube.difference(&sphere);
let intersection_result = cylinder.intersection(&sphere);
```

Booleans on any type implementing the CSG trait such as `Mesh<S>` or `Sketch<S>` return their own type.
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
- <img src="docs/distribute_linear.png" width="128"/> **`::distribute_linear(count: usize, dir: nalgebra::Vector3, spacing: Real)`**
- <img src="docs/distribute_grid.png" width="128"/> **`::distribute_grid(rows: usize, cols: usize, dx: Real, dy: Real)`**
- <img src="docs/inverse.png" width="128"/> **`::inverse()`** - flips the inside/outside orientation.

```rust
use nalgebra::Vector3;
use csgrs::mesh::plane::Plane;
use csgrs::traits::CSG;

let moved = cube.translate(3.0, 0.0, 0.0);
let moved2 = cube.translate_vector(Vector3::new(3.0, 0.0, 0.0));
let rotated = sphere.rotate(0.0, 45.0, 90.0);
let scaled = cylinder.scale(2.0, 1.0, 1.0);
let plane_x = Plane { normal: Vector3::x(), w: 0.0 }; // x=0 plane
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
- **`Mesh::from_polygons(polygons: &[Polygon<S>])`** - create a new CSG from Polygons.

### STL

- **Export ASCII STL**: `csg.to_stl_ascii("solid_name") -> String`
- **Export Binary STL**: `csg.to_stl_binary("solid_name") -> io::Result<Vec<u8>>`
- **Import STL**: `Mesh::from_stl(&stl_data) -> io::Result<CSG<S>>`

```rust
// Save to ASCII STL
let stl_text = csg_union.to_stl_ascii("union_solid");
std::fs::write("union_ascii.stl", stl_text).unwrap();

// Save to binary STL
let stl_bytes = csg_union.to_stl_binary("union_solid").unwrap();
std::fs::write("union_bin.stl", stl_bytes).unwrap();

// Load from an STL file on disk
let file_data = std::fs::read("some_file.stl")?;
let imported_mesh = Mesh::from_stl(&file_data)?;
```

### DXF

- **Export**: `csg.to_dxf() -> Result<Vec<u8>, Box<dyn Error>>`
- **Import**: `Mesh::from_dxf(&dxf_data) -> Result<CSG<S>, Box<dyn Error>>`

```rust
// Export DXF
let dxf_bytes = csg_obj.to_dxf()?;
std::fs::write("output.dxf", dxf_bytes)?;

// Import DXF
let dxf_data = std::fs::read("some_file.dxf")?;
let csg_dxf = CSG::from_dxf(&dxf_data)?;
```

### Hershey Text

Hershey fonts are single stroke fonts which produce open ended polylines in the XY plane via [`hershey`](https://crates.io/crates/hershey):

```rust
let font_data = include_bytes("../fonts/myfont.jhf");
let csg_text = Sketch::from_hershey("Hello!", font_data, 20.0, None);
```

### Create a Bevy `Mesh`

`csg.to_bevy_mesh()` returns a Bevy [`Mesh`](https://docs.rs/bevy/latest/bevy/prelude/struct.Mesh.html).

```rust
use bevy::{prelude::*, render::render_asset::RenderAssetUsages, render::mesh::{Indices, PrimitiveTopology}};

let bevy_mesh = mesh_obj.to_bevy_mesh();
```

### Create a Parry `TriMesh`

`mesh.to_trimesh()` returns an `Option<TriMesh<Real>>`. `mesh.to_rapier_shape()`
wraps the same triangle data in a Rapier `SharedShape`. `Mesh::ray_intersections`,
`Mesh::intersect_polyline`, and `Mesh::contains_vertex` use Parry ray and point
queries, and the mesh stores a cached query `TriMesh` where the operation can
reuse it.

```rust
use csgrs::float_types::parry3d::query::{Ray, RayCast};
use nalgebra::{Point3, Vector3};

let trimesh = mesh_obj.to_trimesh().expect("valid triangle mesh");
let ray = Ray::new(Point3::new(-10.0, 0.0, 0.0), Vector3::x());
let hit = trimesh.cast_local_ray(&ray, 100.0, false);
```

### Create a Rapier Rigid Body

`csg.to_rigid_body(rb_set, co_set, translation, rotation, density)` helps build and insert both a rigid body and a collider:

```rust
use nalgebra::Vector3;
use csgrs::float_types::rapier3d::prelude::*;  // re-exported for f32/f64 support
use csgrs::float_types::FRAC_PI_2;
use csgrs::traits::CSG;
use csgrs::mesh::Mesh;

let mut rb_set = RigidBodySet::new();
let mut co_set = ColliderSet::new();

let axis_angle = Vector3::z() * FRAC_PI_2; // 90° around Z
let rb_handle = mesh_obj.to_rigid_body(
    &mut rb_set,
    &mut co_set,
    Vector3::new(0.0, 0.0, 0.0), // translation
    axis_angle,                  // axis-angle
    1.0,                         // density
);
```

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

To account for error inherent in the storage of IEEE754 floating point numbers, and to maintain the speed of geometric calculations, `csgrs` provides a tunable tolerance value which can be set at compile time or once at the beginning of runtime as follows:

* **Shell**:

  ```bash
  CSGRS_TOLERANCE=1e-6 cargo build
  ```

* **Cargo config** (project- or user-level `.cargo/config.toml`):

  ```toml
  [env]
  CSGRS_TOLERANCE = "1e-6"
  ```

* **Build script in the *dependent* crate** (`build.rs`):

  ```rust
  fn main() {
      println!("cargo:rustc-env=CSGRS_TOLERANCE=1e-6");
  }
  ```

* **Runtime override** (in their `main`):

  ```rust
  fn main() {
      csgrs::float_types::set_tolerance(1e-6);
      // ... rest of the program ...
  }
  ```

> `set_tolerance` is a one-shot setter; subsequent calls are ignored. Use it at program start.

## Working with Metadata

`Mesh<S>` and `Sketch<S>` are generic over `S: Clone`. Each polygon in a `Mesh<S>` and each `Mesh<S>` and `Sketch<S>` have an optional `metadata: Option<S>`.  
Use cases include storing color, ID, or layer info.

```rust
use csgrs::polygon::Polygon;
use csgrs::vertex::Vertex;
use nalgebra::{Point3, Vector3};

#[derive(Clone)]
struct MyMetadata {
    color: (u8, u8, u8),
    label: String,
}

type Mesh = csgrs::mesh::Mesh<MyMetadata>;

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
core is BSP-backed `Mesh`, `geo`-backed `Sketch`, polygon triangulation, common
2D/3D primitive construction, extrusion/revolve/sweep/loft operations,
transformations, metadata propagation, cached bounding boxes, TriMesh query
conversion, ray intersection helpers, mesh quality utilities, and import/export
for common mesh and manufacturing formats behind Cargo features.

The project is also intentionally experimental in several areas:

- **Boolean kernels**: BSP booleans are the compatibility baseline. `BMesh`
  exposes a boolmesh-backed layer behind the `bmesh` feature, and future work is
  aimed at making indexed mesh booleans lower cost and more robust.
- **Triangulation**: exactly one of `delaunay`, `earcut`, or `delaunay-rs` must
  be selected. Delaunay through `geo`/Spade is the current default.
- **Numeric model**: `Real` is selected by `f64` or `f32`, with `f64` default.
  The tolerance can be configured through `CSGRS_TOLERANCE` or
  `set_tolerance`.
- **NURBS and curves**: Curvo-backed NURBS support is available behind the
  `nurbs` feature, but it should be treated as an integration layer rather than
  the final curve kernel.
- **WASM**: JavaScript bindings cover the core mesh/sketch APIs and are useful
  for browser previews, but the Rust API remains the source of truth.
- **Toolpaths**: contour, pocket, FDM layer, lathe roughing, and G-code helpers
  exist, with more CAM robustness work still needed.

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

## Roadmap

- **Boolean robustness**: keep BSP as the early verification layer, complete the
  boolmesh-backed boolean path, reduce excess polygon production by testing
  only polygons whose cached bounding boxes intersect, and add indexed mesh
  conversion/merge utilities.
- **Numeric backend**: continue the f64 default path while preparing a cleaner
  tolerance/ordering layer around `Real`, including better use of exact or
  robust predicates where the current tolerance model is weak.
- **Triangulation and polygon validity**: finish T-junction repair, coplanar
  polygon merging, validation hooks, and consistent triangulation through Spade,
  Earcut, or Delaunay backends.
- **Curves and offsets**: harden Bezier, B-spline, NURBS, offset, straight
  skeleton, and TrueType text paths; keep Curvo useful immediately while
  harvesting algorithms that can be owned with a lower maintenance cost.
- **Feature-complete modeling operations**: add rounded cuboids, chamfers,
  fillets, 3D offsets, bending, threaded parts, richer gear options, and
  attachment/alignment helpers.
- **Representations**: add indexed mesh APIs, half-edge/radial-edge adapters,
  metadata deduplication, UV support, and better conversion among `Mesh`,
  `BMesh`, `Sketch`, `Nurbs`, `TriMesh`, and file formats.
- **Performance**: broaden Rayon use in operations that are embarrassingly
  parallel, preserve borrowed-slice APIs, minimize allocations, and use cached
  AABBs/TriMesh acceleration structures in query-heavy code.
- **I/O and integration**: improve glTF/Gerber output, add STEP/IGES research,
  continue Bevy/Rapier interop, and keep WASM bindings aligned with the Rust API.
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

> “Earcut.” *GitHub*, Mapbox, https://github.com/mapbox/earcut.

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

> “nalgebra.” *Dimforge*, https://nalgebra.org/.

> Patrikalakis, Nicholas M., Takashi Maekawa, and Wen-Chyung Cho. *Shape
> Interrogation for Computer Aided Design and Manufacturing*. MIT,
> https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/.

> “Parry: 2D and 3D Collision Detection Libraries for Rust.” *Dimforge*,
> https://parry.rs/.

> Pharr, Matt, Wenzel Jakob, and Greg Humphreys. *Physically Based Rendering:
> From Theory to Implementation*. 4th ed., MIT Press, 2023,
> https://pbr-book.org/.

> Prade, Inigo Quilez. “Distance Functions.” *Inigo Quilez*,
> https://iquilezles.org/articles/distfunctions/.

> “Rapier: 2D and 3D Physics Engines for Rust.” *Dimforge*, https://rapier.rs/.

> Schoen, Alan H. *Infinite Periodic Minimal Surfaces without Self-Intersections*.
> NASA Technical Note D-5541, 1970.

> Shewchuk, Jonathan Richard. “Adaptive Precision Floating-Point Arithmetic and
> Fast Robust Geometric Predicates.” *Discrete & Computational Geometry*, vol.
> 18, no. 3, 1997, pp. 305-363.

> Shewchuk, Jonathan Richard. “Robust Adaptive Floating-Point Geometric
> Predicates.” *Proceedings of the Twelfth Annual Symposium on Computational
> Geometry*, ACM, 1996, pp. 141-150.

> “Spade.” *crates.io*, https://crates.io/crates/spade.

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
