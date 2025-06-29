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
algorithms used for triangulation only work in 2D, so **csgrs** rotates
3D polygons into 2D for triangulation then back to 3D.

![Example CSG output](docs/csg.png)

## Community
[![](https://dcbadge.limes.pink/api/server/https://discord.gg/9WkD3WFxMC)](https://discord.gg/9WkD3WFxMC)

## Getting started

Install the [Rust](https://www.rust-lang.org/) language tools from
[rustup.rs](https://rustup.rs/).

```shell
cargo new my_cad_project
cd my_cad_project
cargo add csgrs
```

### Example main.rs

```rust
// Alias the library’s generic Mesh type with empty metadata:
type Mesh = csgrs::mesh::Mesh<()>;

// Create two shapes:
let cube = Mesh::cube(2.0, None);  // 2×2×2 cube at origin, no metadata
let sphere = Mesh::sphere(1.0, 16, 8, None); // sphere of radius=1 at origin, no metadata

// Difference one from the other:
let difference_result = cube.difference(&sphere);

// Write the result as an ASCII STL:
let stl = difference_result.to_stl_ascii("cube_minus_sphere");
std::fs::write("cube_sphere_difference.stl", stl).unwrap();
```

### Building for WASM

```shell
cargo build --features="wasm" --target=wasm32-unknown-unknown --release
```

### Sketch Structure

- **`Sketch<S>`** is the type which stores and manipulates 2D polygonal geometry.  It contains:
  - a [`geo`](https://crates.io/crates/geo) [`GeometryCollection<Real>`](https://docs.rs/geo/latest/geo/geometry/struct.GeometryCollection.html)
  - a bounding box wrapped in a OnceLock (bounding_box: OnceLock<Aabb>)
  - an optional metadata field (`Option<S>`) also defined by you

`Sketch<S>` provides methods for working with 2D shapes made of points and lines.
You can build a `Sketch<S>` geo Geometries with `Sketch::from_geo(...)`.
Geometries can be open or closed, have holes, but must be planar in the XY.
`Sketch`'s are triangulated when exported as an STL, or when a Geometry is
converted into a `Mesh<S>`.

### 2D Shapes in Sketch

- <img src="docs/square.png" width="128" alt="top down view of a square"/> **`Sketch::square(width: Real, metadata: Option<S>)`**
- <img src="docs/square.png" width="128" alt="top down view of a square"/> **`Sketch::rectangle(width: Real, length: Real, metadata: Option<S>)`**
- <img src="docs/circle.png" width="128" alt="top down view of a circle"/> **`Sketch::circle(radius: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/polygon.png" width="128" alt="top down view of a triangle"/> **`Sketch::polygon(&[[x1,y1],[x2,y2],...], metadata: Option<S>)`**
- <img src="docs/rounded_rectangle.png" width="128" alt="top down view of a rectangle with rounded corners"/> **`Sketch::rounded_rectangle(width: Real, height: Real, corner_radius: Real, corner_segments: usize, metadata: Option<S>)`**
- <img src="docs/ellipse.png" width="128" alt="top down view of an ellipse"/> **`Sketch::ellipse(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/ngon.png" width="128" alt="top down view of a 6 sided n-gon"/> **`Sketch::regular_ngon(sides: usize, radius: Real, metadata: Option<S>)`**
- <img src="docs/right_triangle.png" width="128" alt="top down view of a right triangle"/> **`Sketch::right_triangle(width: Real, height: Real, metadata: Option<S>)`**
- <img src="docs/trapezoid.png" width="128" alt="top down view of trapezoid"/> **`Sketch::trapezoid(top_width: Real, bottom_width: Real, height: Real, top_offset: Real, metadata: Option<S>)`**
- <img src="docs/star.png" width="128" alt="top down view of star"/> **`Sketch::star(num_points: usize, outer_radius: Real, inner_radius: Real, metadata: Option<S>)`**
- <img src="docs/teardrop.png" width="128" alt="top down view of a teardrop"/> **`Sketch::teardrop(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/egg_outline.png" width="128" alt="top down view of an egg shape"/> **`Sketch::egg(width: Real, length: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/squircle.png" width="128" alt="top down view of a squircle"/> **`Sketch::squircle(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/keyhole.png" width="128" alt="top down view of a keyhole"/> **`Sketch::keyhole(circle_radius: Real, handle_width: Real, handle_height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/reuleaux3.png" width="128"/> **`Sketch::reuleaux(sides: usize, radius: Real, arc_segments_per_side: usize, metadata: Option<S>)`**
- <img src="docs/ring.png" width="128" alt="top down view of a ring"/> **`Sketch::ring(id: Real, thickness: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/pie_slice.png" width="128" alt="top down view of a slice of a circle"/> **`Sketch::pie_slice(radius: Real, start_angle_deg: Real, end_angle_deg: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/supershape.png" width="128"/> **`Sketch::supershape(a: Real, b: Real, m: Real, n1: Real, n2: Real, n3: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/circle_with_keyway.png" width="128" alt="top down view of a circle with a notch taken out of it"/> **`Sketch::circle_with_keyway(radius: Real, segments: usize, key_width: Real, key_depth: Real, metadata: Option<S>)`**
- <img src="docs/d.png" width="128" alt="top down view of a circle with a flat edge"/> **`Sketch::circle_with_flat(radius: Real, segments: usize, flat_dist: Real, metadata: Option<S>)`**
- <img src="docs/double_flat.png" width="128" alt="top down view of a circle with two flat edges"/> **`Sketch::circle_with_two_flats(radius: Real, segments: usize, flat_dist: Real, metadata: Option<S>)`**
- <img src="docs/from_image.png" width="128" alt="top down view of a pixleated circle"/> **`Sketch::from_image(img: &GrayImage, threshold: u8, closepaths: bool, metadata: Option<S>)`** - Builds a new CSG from the “on” pixels of a grayscale image
- <img src="docs/truetype.png" width="128" alt="top down view of the text 'HELLO'"/> **`Sketch::text(text: &str, font_data: &[u8], size: Real, metadata: Option<S>)`** - generate 2D text geometry in the XY plane from TTF fonts
- <img src="docs/metaballs_2d.png" width="128"/> **`Sketch::metaballs(balls: &[(nalgebra::Point2<Real>, Real)], resolution: (usize, usize), iso_value: Real, padding: Real, metadata: Option<S>)`**
- <img src="docs/airfoil.png" width="128" alt="a side view of an airfoil"/> **`Sketch::airfoil(code: &str, chord: Real, samples: usize, metadata: Option<S>)`**
- <img src="docs/bezier_extruded.png" width="128" alt="an angled view of a bezier cirve"/> **`Sketch::bezier(control: &[[Real; 2]], segments: usize, metadata: Option<S>)`**
- <img src="docs/bspline.png" width="128" alt="top down view of a neer semi-circle shape"/> **`Sketch::bspline(control: &[[Real; 2]], p: usize, segments_per_span: usize, metadata: Option<S>)`**
- <img src="docs/heart.png" width="128" alt="top down view of a cartune heart"/> **`Sketch::heart(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/crescent.png" width="128"/> **`Sketch::crescent(outer_r: Real, inner_r: Real, offset: Real, segments: usize, metadata: Option<S>)`** - 
- **`Sketch::involute_gear(module_: Real, teeth: usize, pressure_angle_deg: Real, clearance: Real, backlash: Real, segments_per_flank: usize, metadata: Option<S>)`** - under construction
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
- <img src="docs/rotate_extrude.png" width="128"  alt="an arch with round ends"/> **`Sketch::revolve(angle_degs, segments)`** - Extrude while rotating around the Y axis
- **`Sketch::loft(&bottom_polygon, &top_polygon, false)`** - Helper function which extrudes between two Mesh Polygons, optionally with caps

```rust
let square = Sketch::square(2.0, None);
let prism = square.extrude(5.0);

let revolve_shape = square.revolve(360.0, 16);

let bottom = Sketch::circle(2.0, 64, None);
let top = bottom.translate(0.0, 0.0, 5.0);
let lofted = Sketch::loft(&bottom.polygons[0], &top.polygons[0], false);
```

### Mesh Structure

- **`Mesh<S>`** is the type which stores and manipulates 3D polygonal geometry.  It contains:
  - a `Vec<Polygon<S>>` polygons, describing 3D shapes, each `Polygon<S>` holds:
    - a `Vec<Vertex>` (positions + normals),
    - a `Plane` describing the polygon’s orientation in 3D.
    - an optional metadata field (`Option<S>`) defined by you
  - a bounding box wrapped in a OnceLock (bounding_box: OnceLock<Aabb>)
  - another optional metadata field (`Option<S>`) also defined by you

`Mesh<S>` provides methods for working with 3D shapes. You can build a
`Mesh<S>` from polygons with `Mesh::from_polygons(...)`.
Polygons must be closed, planar, have 3 or more vertices.
Polygons are triangulated when being exported as an STL.

### 3D Shapes in Mesh

- <img src="docs/cube.png" width="128" alt="an angled view of a cube"/> **`Mesh::cube(width: Real, metadata: Option<S>)`**
- <img src="docs/cube.png" width="128" alt="an angled view of a cube"/> **`Mesh::cuboid(width: Real, length: Real, height: Real, metadata: Option<S>)`**
- <img src="docs/sphere.png" width="128" alt="an angled view of a sphere"/> **`Mesh::sphere(radius: Real, segments: usize, stacks: usize, metadata: Option<S>)`**
- <img src="docs/cylinder.png" width="128" alt="an angled view of a cylinder"/> **`Mesh::cylinder(radius: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/frustum.png" width="128"/> **`Mesh::frustum(radius1: Real, radius2: Real, height: Real, segments: usize, metadata: Option<S>)`** -
Construct a frustum at origin with height and `radius1` and `radius2`.
If either radius is within EPSILON of 0.0, a cone terminating at a point is constructed.
- <img src="docs/frustum.png" width="128"/> **`Mesh::frustum_ptp(start: Point3, end: Point3, radius1: Real, radius2: Real, segments:
usize, metadata: Option<S>)`** -
Construct a frustum from `start` to `end` with `radius1` and `radius2`.
If either radius is within EPSILON of 0.0, a cone terminating at a point is constructed.
- <img src="docs/polyhedron.png" width="128"/> **`Mesh::polyhedron(points: &[[Real; 3]], faces: &[Vec<usize>], metadata: Option<S>)`**
- <img src="docs/octahedron.png" width="128"/> **`Mesh::octahedron(radius: Real, metadata: Option<S>)`** -
- <img src="docs/icosahedron.png" width="128"/> **`Mesh::icosahedron(radius: Real, metadata: Option<S>)`** -
- <img src="docs/torus.png" width="128"/> **`Mesh::torus(major_r: Real, minor_r: Real, segments_major: usize, segments_minor: usize, metadata: Option<S>)`** -
- <img src="docs/egg.png" width="128"/> **`Mesh::egg(width: Real, length: Real, revolve_segments: usize, outline_segments: usize, metadata: Option<S>)`**
- <img src="docs/teardrop3d.png" width="128"/> **`Mesh::teardrop(width: Real, height: Real, revolve_segments: usize, shape_segments: usize, metadata: Option<S>)`**
- <img src="docs/teardrop_cylinder.png" width="128"/> **`Mesh::teardrop_cylinder(width: Real, length: Real, height: Real, shape_segments: usize, metadata: Option<S>)`**
- <img src="docs/ellipsoid.png" width="128"/> **`Mesh::ellipsoid(rx: Real, ry: Real, rz: Real, segments: usize, stacks: usize, metadata: Option<S>)`**
- <img src="docs/metaballs.png" width="128"/> **`Mesh::metaballs(balls: &[MetaBall], resolution: (usize, usize, usize), iso_value: Real, padding: Real, metadata: Option<S>)`**
- <img src="docs/sdf-sphere.png" width="128"/> **`Mesh::sdf<F>(sdf: F, resolution: (usize, usize, usize), min_pt: Point3, max_pt: Point3, iso_value: Real, metadata: Option<S>)`** - Return a CSG created by meshing a signed distance field within a bounding box
- <img src="docs/arrow_to.png" width="128"/> **`Mesh::arrow(start: Point3, direction: Vector3, segments: usize, orientation: bool, metadata: Option<S>)`** - Create an arrow at start, pointing along direction
- <img src="docs/gyroid.png" width="128"/> **`Mesh::gyroid(resolution: usize, period: Real, iso_value: Real, metadata: Option<S>)`** - Generate a Triply Periodic Minimal Surface (Gyroid) inside the volume of `self`
- <img src="docs/schwarzp.png" width="128"/> **`Mesh::schwarz_p(resolution: usize, period: Real, iso_value: Real, metadata: Option<S>)`** - Generate a Triply Periodic Minimal Surface (Schwarz P) inside the volume of `self`
- <img src="docs/schwarzd.png" width="128"/> **`Mesh::schwarz_d(resolution: usize, period: Real, iso_value: Real, metadata: Option<S>)`** - Generate a Triply Periodic Minimal Surface (Schwarz D) inside the volume of `self`
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

### CSGOps Boolean Operations

```rust
use csgrs::traits::CSGOps;

let union_result = cube.union(&sphere);
let difference_result = cube.difference(&sphere);
let intersection_result = cylinder.intersection(&sphere);
```

Booleans on any type implementing the CSGOps trait such as `Mesh<S>` or `Sketch<S>` return their own type.
Types implementing the CSGOps trait also provide the following transformation functions:

### Transformations

- **`::translate(x: Real, y: Real, z: Real)`** - Returns the CSG translated by x, y, and z
- **`::translate_vector(vector: Vector3)`** - Returns the CSG translated by vector
- **`::rotate(x_deg, y_deg, z_deg)`** - Returns the CSG rotated in x, y, and z
- **`::scale(scale_x, scale_y, scale_z)`** - Returns the CSG scaled in x, y, and z
- **`::mirror(plane: Plane)`** - Returns the CSG mirrored across plane
- **`::center()`** - Returns the CSG centered at the origin
- **`::float()`** - Returns the CSG translated so that its bottommost point(s) sit exactly at z=0
- **`::transform(&Matrix4)`** - Returns the CSG after applying arbitrary affine transforms
- <img src="docs/distribute_arc.png" width="128"/> **`CSG::distribute_arc(count: usize, radius: Real, start_angle_deg: Real, end_angle_deg: Real)`**
- <img src="docs/distribute_line.png" width="128"/> **`CSG::distribute_linear(count: usize, dir: nalgebra::Vector3, spacing: Real)`**
- <img src="docs/distribute_grid.png" width="128"/> **`CSG::distribute_grid(rows: usize, cols: usize, dx: Real, dy: Real)`**

```rust
use nalgebra::Vector3;
use csgrs::mesh::plane::Plane;
use csgrs::traits::CSGOps;

let moved = cube.translate(3.0, 0.0, 0.0);
let moved2 = cube.translate_vector(Vector3::new(3.0, 0.0, 0.0));
let rotated = sphere.rotate(0.0, 45.0, 90.0);
let scaled = cylinder.scale(2.0, 1.0, 1.0);
let plane_x = Plane { normal: Vector3::x(), w: 0.0 }; // x=0 plane
let plane_y = Plane { normal: Vector3::y(), w: 0.0 }; // y=0 plane
let plane_z = Plane { normal: Vector3::z(), w: 0.0 }; // z=0 plane
let mirrored = cube.mirror(plane_x);
```

### Miscellaneous Operations

- **`Mesh::vertices()`** — collect all vertices from the CSG
- <img src="docs/inverse_sphere.png" width="128"/> **`::inverse()`** — flips the inside/outside orientation.
- <img src="docs/convex_hull.png" width="128"/> **`Mesh::convex_hull()`** — uses [`chull`](https://crates.io/crates/chull) to generate a 3D convex hull.
- <img src="docs/minkowski.png" width="128"/> **`Mesh::minkowski_sum(&other)`** — naive Minkowski sum, then takes the hull.
- **`Mesh::ray_intersections(origin, direction)`** — returns all intersection points and distances.
- **`Mesh::flatten()`** — flattens a 3D shape into 2D (on the XY plane), unions the outlines.
- **`Mesh::slice(plane)`** — slices the CSG by a plane and returns the cross-section polygons.
- **`Sketch::offset(distance)`** — outward (or inward) offset in 2D using [`geo-offset`](https://crates.io/crates/geo-offset).
- **`Sketch::offset_rounded(distance)`** — outward (or inward) offset in 2D using [`geo-offset`](https://crates.io/crates/geo-offset).
- <img src="docs/subdivided.png" width="128"/> **`CSG::subdivide_triangles(subdivisions)`** — subdivides each polygon’s triangles, increasing mesh density.
- **`Mesh::renormalize()`** — re-computes each polygon’s plane from its vertices, resetting all normals.
- **`::bounding_box()`** — computes the bounding box of the shape.
- **`::invalidate_bounding_box()`** — invalidates the bounding box of the shape, causing it to be recomputed on next access
- **`Mesh::tessellate()`** — triangulates all polygons returning a CSG containing triangles.
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

`csg.to_trimesh()` returns a `SharedShape` containing a `TriMesh<Real>`.

```rust
use csgrs::float_types::rapier3d::prelude::*;  // re-exported for f32/f64 support

let trimesh_shape = mesh_obj.to_trimesh(); // SharedShape with a TriMesh
```

### Create a Rapier Rigid Body

`csg.to_rigid_body(rb_set, co_set, translation, rotation, density)` helps build and insert both a rigid body and a collider:

```rust
use nalgebra::Vector3;
use csgrs::float_types::rapier3d::prelude::*;  // re-exported for f32/f64 support
use csgrs::float_types::FRAC_PI_2;
use csgrs::traits::CSGOps;
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

## Build tests
A cargo xtask is included in the repository for testing building with various
combinations of feature flags.  To use it, you must install cargo xtask:
```rust
cargo install xtask
```

To run the tests:
```rust
cargo xtask test-all
```

## Roadmap
- **Attachments** Unless you make models containing just one object attachments features can revolutionize your modeling. They will let you position components of a model relative to other components so you don't have to keep track of the positions and orientations of parts of the model. You can instead place something on the TOP of something else, perhaps aligned to the RIGHT.
- **Rounding and filleting** Provide modules like cuboid() to make a cube with any of the edges rounded, offset_sweep() to round the ends of a linear extrusion, and prism_connector() which works with the attachments feature to create filleted prisms between a variety of objects, or even rounded holes through a single object. Also edge_profile() to apply a variety of different mask profiles to chosen edges of a cubic shape, or directly subtract 3d mask shapes from an edge of objects that are not cubes.
- **Complex object support** The path_sweep() function/module takes a 2d polygon moves it through space along a path and sweeps out a 3d shape as it moves. Link together a series of arbitrary polygons with skin() or vnf_vertex_array().  Build parts of an object in multiple different representations and combine.
- **Texturing** Apply textures to many kinds of objects. Create knurling or any repeating pattern.  Applying a texture can actually replace the base object with something different based on repeating copies of the texture element. A texture can also be an image; using texturing you can emboss an arbitrary image onto your model.
- **Parts library** The parts library will include many useful specific functional parts including gears, generic threading, and specific threading to match plastic bottles, pipe fittings, and standard screws. Also clips, hinges, and dovetail joints, aluminum extrusion, bearings, nuts, bolts, washers, etc.
- **Shorthands** Shorthands to make your code a little shorter, and more importantly, make it significantly easier to read. Compare up(x) to translate([0,0,x]). Shorthands will include operations for creating copies of objects and for applying transformations to objects.  Drawing like turtle graphics will be possible.
- **Non-linear solver** Composed of a tree which can contain operations and variables representing systems of equations describing constraints, and functionality to perterb variables, sample the solution space described by the tree expression, determine the local slope, and hill climb toward a solution.

## Performance
Patterns we work to follow throughout the library to improve performance
and memory usage:
- functions should accept borrowed slices, this permits easy use of iterators
- iterators should be used wherever parallelism may help (and rayon's par_iter)
- allocations should be kept to a minimum.  Memory should be read-only if
possible, clone if necessary, and offer the choice of transmut in place or
create new copy via appropriate functions

## Todo
- when tessellating, detect T junctions with other polygons with shared edges,
and insert splitting vertices into polygons to correct
- implement as_indexed, from_indexed, and merge_vertices (using hashbrown, and a
string expression of each float out to EPSILON significant digits)
- ensure re-triangulate unions all coplanar polygons
- evaluate https://docs.rs/parry3d/latest/parry3d/shape/struct.HalfSpace.html and
https://docs.rs/parry3d/latest/parry3d/query/point/trait.PointQuery.html#method.contains_point
for plane splitting
- evaluate https://docs.rs/parry3d/latest/parry3d/shape/struct.Polyline.html
for Polygon
- evaluate https://docs.rs/parry3d/latest/parry3d/shape/struct.Segment.html
- evaluate https://docs.rs/nalgebra/latest/nalgebra/geometry/struct.Rotation.html#method.rotation_between-1 
- evaluate https://docs.rs/parry3d/latest/parry3d/shape/struct.Triangle.html
- evaluate https://docs.rs/parry3d/latest/parry3d/shape/struct.Segment.html#method.local_split_and_get_intersection in plane splitting and slicing
- evaluate https://github.com/dimforge/parry/blob/master/src/query/clip/clip_halfspace_polygon.rs
- evaluate https://github.com/dimforge/parry/blob/master/src/query/clip/clip_segment_segment.rs
- evaluate https://github.com/dimforge/parry/blob/master/src/transformation/voxelization/voxel_set.rs and https://github.com/dimforge/parry/blob/master/src/transformation/voxelization/voxelized_volume.rs
- evaluate https://github.com/dimforge/parry/blob/master/src/transformation/convex_hull3/convex_hull.rs instead of chull
- evaluate https://github.com/dimforge/parry/blob/master/src/utils/ccw_face_normal.rs for normalization
- implement wavefront obj output using https://github.com/dimforge/parry/blob/master/src/transformation/wavefront.rs
- transition sweep, linear_extrude, over to Polygon/Multipolygon native / polygon secondary
- disengage chulls on 2D->3D shapes
- fix subtract_cube_sphere in main.rs - shapes are out of proximity
- fix up error handling with result types, eliminate panics
- ray intersection (singular)
- expose geo traits on 2D shapes
- https://www.nalgebra.org/docs/user_guide/projections/ for 2d and 3d
- convert more for loops to iterators - csg::transform
- polygons_by_metadata public function of a CSG
  - draft implementation done, pending API discussion
- document coordinate system / coordinate transformations / compounded transformations
- determine why flattened_cube.stl produces invalid output with to_stl_binary but not to_stl_ascii
- determine why square_2d_shrink.stl produces invalid output with to_stl_binary but not to_stl_ascii
- determine why square_2d produces invalid output with to_stl_binary but not to_stl_ascii
- bending
- lead-ins, lead-outs
- gpu acceleration
  - https://github.com/dimforge/wgmath
  - https://github.com/pcwalton/pathfinder
- reduce dependency feature sets
- space filling curves, hilbert sort polygons / points
- identify more candidates for par_iter: minkowski, polygon_from_slice, is_manifold
- http://www.ofitselfso.com/MiscNotes/CAMBamStickFonts.php
- screw threads
- support scale and translation along a vector in rotate extrude
- reimplement 3D offsetting with https://github.com/u65xhd/meshvox or https://docs.rs/parry3d/latest/parry3d/transformation/vhacd/struct.VHACD.html or https://github.com/komadori/bevy_mod_outline/
- implement 2d/3d convex decomposition with https://docs.rs/parry3d-f64/latest/parry3d_f64/transformation/vhacd/struct.VHACD.html
  - https://github.com/dimforge/parry/blob/master/src/transformation/hertel_mehlhorn.rs for convex partitioning
- reimplement transformations and shapes with https://docs.rs/parry3d/latest/parry3d/transformation/utils/index.html
  - https://github.com/dimforge/parry/tree/master/src/transformation/to_outline or to_polyline
- std::io::Cursor, std::error::Error - core2 no_std transition
- https://crates.io/crates/polylabel
  - pull in https://github.com/fschutt/polylabel-mini/blob/master/src/lib.rs and adjust f64 -> Real
- reduce allocations
- history tree
  - STEP/IGES import / export
  - curves?
- constraintt solving tree
- test geo_booleanop as alternative to geo's built-in boolean ops.
- rethink metadata
  - support storing UV[W] coordinates with vertices at compile time (try to keep runtime cost low too)
  - accomplish equivalence checks and memory usage reduction by using a hashmap or references instead of storing metadata with each node
  - with equivalence checks, returning sorted metadata becomes easy
- implement half-edge, radial edge, etc to and from adapters
  - chamfers
  - fillets
  - manifold tests
  - 3D offset
  - attachments
- align_x_pos, align_x_neg, align_y_pos, align_y_neg, align_z_pos, align_z_neg, center_x, center_y, center_z,
- attachment points / rapier integration
  - attachment is a Vertex (Point + normal)
  - attachments Vec in CSG datastructure
  - make corners and centers of bb accessible by default, even in empty CSG
  - make corners, edge midpoints, and centroids of polygons accessible by default (calculate on demand using an iterator)
  - align_to_attachment(name, csg2, name2)
- implement C FFI using https://rust-lang.github.io/rust-bindgen/
- pull in https://crates.io/crates/geo-uom for units and dimensional analysis
- https://proptest-rs.github.io/proptest/intro.html
- https://crates.io/crates/geo-validity-check as compile time option
- https://crates.io/crates/geo-index - 2D only :(
- https://github.com/lelongg/geo-rand
- renderer integration
  - blueprint renders
  - exploded renders - installation vector
- implement 2D line, point, LineString functions for Sketch
- https://github.com/hmeyer/tessellation
- emit TrueType glyphs into the same MultiPolygon for each call of text()
- evaluate using approx crate
- evaluate using https://docs.rs/nalgebra/latest/nalgebra/trait.RealField.html instead of float_types::Real
- mutable API for transmute, etc.
- implement trait geo::MetricSpace on nalgebra::Point, Point2, Point3
- gltf output
- gerber output
- rework bezier and bspline using https://github.com/mattatz/curvo
  - import functions from https://github.com/nical/lyon/tree/main/crates/geom/src for cubic and quadratic bezier
- https://docs.rs/rgeometry/latest/rgeometry/algorithms/polygonization/fn.two_opt_moves.html and other algorithms from rgeometry crate

## Todo shapes
- geodesic domes / goldberg polyhedra
- uniform polyhedra
- molecular models
- kepler-poinsot polyhedra
- dodecahedron
- Archimedean / Catalan solids
- Johnson solids, near-miss johnson solids
- deltahedrons
- regular polytopes
- regular skew polyhedra
- toroidal polyhedra
- shapes from https://iquilezles.org/articles/

## Todo easy
- finish naca airfoil implementations
- stack transformation
- additional renders

## Todo maybe
- https://github.com/PsichiX/density-mesh
- https://github.com/asny/tri-mesh port
- https://crates.io/crates/flo_curves
- port https://github.com/21re/rust-geo-booleanop to cavalier_contours
- hyperbolic geometry: https://github.com/agerasev/ccgeom/tree/master/src/hyperbolic
- https://crates.io/crates/spherical_geometry
- https://crates.io/crates/miniproj
- examine https://crates.io/crates/geo-aid constraint solver
- examine https://cadquery.readthedocs.io/en/latest/apireference.html for function ideas
- https://github.com/tscircuit/tscircuit

## References
> [Shape Interrogation for Computer Aided Design and Manufacturing](https://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/)

> [Shewchuk, J.R., 1997. Adaptive precision floating-point arithmetic and fast robust geometric predicates. Discrete & Computational Geometry, 18(3), pp.305-363.](https://link.springer.com/content/pdf/10.1007/PL00009321.pdf)

> [Shewchuk, J.R., 1996, May. Robust adaptive floating-point geometric predicates. In Proceedings of the twelfth annual symposium on Computational geometry (pp. 141-150).](https://dl.acm.org/doi/abs/10.1145/237218.237337)

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
