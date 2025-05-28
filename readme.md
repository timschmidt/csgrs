# csgrs

A fast, optionally multithreaded **Constructive Solid Geometry (CSG)**
library in Rust, built around Boolean operations (*union*, *difference*,
*intersection*, *xor*) on several different internal geometry representations.
**csgrs** provides data structures and methods for constructing 2D and 3D geometry
with an [OpenSCAD](https://openscad.org/)-like syntax.  **csgrs** aims to be light
weight and full featured through integration with the
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
// Alias the library’s generic CSG type with empty metadata:
type CSG = csgrs::csg::CSG<()>;

// Create two shapes:
let cube = CSG::cube(2.0, 2.0, 2.0, None);  // 2×2×2 cube at origin, no metadata
let sphere = CSG::sphere(1.0, 16, 8, None); // sphere of radius=1 at origin, no metadata

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

### CSG Structure

- **`CSG<S>`** is the main type. It stores:
  - a `Vec<Polygon<S>>` polygons, describing 3D shapes, each `Polygon<S>` holds:
    - a `Vec<Vertex>` (positions + normals),
    - a `Plane` describing the polygon’s orientation in 3D.
    - an optional metadata field (`Option<S>`) defined by you
  - a [`geo`](https://crates.io/crates/geo) [`GeometryCollection<Real>`](https://docs.rs/geo/latest/geo/geometry/struct.GeometryCollection.html)
  - another optional metadata field (`Option<S>`) also defined by you

`CSG<S>` provides methods for working with 2D and 3D shapes. You can build a
`CSG<S>` from polygons with `CSG::from_polygons(...)` or from geo Geometries with
`CSG::from_geo(...)`.  Polygons must be closed, planar, have 3 or more vertices,
and are 3D.  Geometries can be open or closed, have holes, but must be planar in
the XY.  Operations work on both 2D and 3D shapes though they generally do not
interact except where one is explicitly transformed into the other as in extrude
or slice.  Polygons and Geometries are triangulated when being exported as an STL,
or when a Geometry is converted into polygons using `CSG::to_polygons(...)`.

### 2D Shapes

- <img src="docs/square.png" width="128"/> **`CSG::square(width: Real, length: Real, metadata: Option<S>)`**
- <img src="docs/circle.png" width="128"/> **`CSG::circle(radius: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/polygon.png" width="128"/> **`CSG::polygon(&[[x1,y1],[x2,y2],...], metadata: Option<S>)`**
- <img src="docs/rounded_rectangle.png" width="128"/> **`CSG::rounded_rectangle(width: Real, height: Real, corner_radius: Real, corner_segments: usize, metadata: Option<S>)`**
- <img src="docs/ellipse.png" width="128"/> **`CSG::ellipse(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/ngon.png" width="128"/> **`CSG::regular_ngon(sides: usize, radius: Real, metadata: Option<S>)`**
- <img src="docs/right_triangle.png" width="128"/> **`CSG::right_triangle(width: Real, height: Real, metadata: Option<S>)`**
- <img src="docs/trapezoid.png" width="128"/> **`CSG::trapezoid(top_width: Real, bottom_width: Real, height: Real, top_offset: Real, metadata: Option<S>)`**
- <img src="docs/star.png" width="128"/> **`CSG::star(num_points: usize, outer_radius: Real, inner_radius: Real, metadata: Option<S>)`**
- <img src="docs/teardrop.png" width="128"/> **`CSG::teardrop(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/egg_outline.png" width="128"/> **`CSG::egg_outline(width: Real, length: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/squircle.png" width="128"/> **`CSG::squircle(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/keyhole.png" width="128"/> **`CSG::keyhole(circle_radius: Real, handle_width: Real, handle_height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/reuleaux3.png" width="128"/> **`CSG::reuleaux(sides: usize, radius: Real, arc_segments_per_side: usize, metadata: Option<S>)`**
- <img src="docs/ring.png" width="128"/> **`CSG::ring(id: Real, thickness: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/pie_slice.png" width="128"/> **`CSG::pie_slice(radius: Real, start_angle_deg: Real, end_angle_deg: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/supershape.png" width="128"/> **`CSG::supershape(a: Real, b: Real, m: Real, n1: Real, n2: Real, n3: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/circle_with_keyway.png" width="128"/> **`CSG::circle_with_keyway(radius: Real, segments: usize, key_width: Real, key_depth: Real, metadata: Option<S>)`**
- <img src="docs/d.png" width="128"/> **`CSG::circle_with_flat(radius: Real, segments: usize, flat_dist: Real, metadata: Option<S>)`**
- <img src="docs/double_flat.png" width="128"/> **`CSG::circle_with_two_flats(radius: Real, segments: usize, flat_dist: Real, metadata: Option<S>)`**
- <img src="docs/from_image.png" width="128"/> **`CSG::from_image(img: &GrayImage, threshold: u8, closepaths: bool, metadata: Option<S>)`** - Builds a new CSG from the “on” pixels of a grayscale image
- <img src="docs/truetype.png" width="128"/> **`CSG::text(text: &str, font_data: &[u8], size: Real, metadata: Option<S>)`** - generate 2D text geometry in the XY plane from TTF fonts
- <img src="docs/metaballs_2d.png" width="128"/> **`CSG::metaballs2d(balls: &[(nalgebra::Point2<Real>, Real)], resolution: (usize, usize), iso_value: Real, padding: Real, metadata: Option<S>)`** - 
- <img src="docs/airfoil.png" width="128"/> **`CSG::airfoil(code: &str, chord: Real, samples: usize, metadata: Option<S>)`** -
- <img src="docs/bezier_extruded.png" width="128"/> **`CSG::bezier(control: &[[Real; 2]], segments: usize, metadata: Option<S>)`**
- <img src="docs/bspline.png" width="128"/> **`CSG::bspline(control: &[[Real; 2]], p: usize, segments_per_span: usize, metadata: Option<S>)`**
- <img src="docs/heart.png" width="128"/> **`CSG::heart(width: Real, height: Real, segments: usize, metadata: Option<S>)`** - 
- <img src="docs/crescent.png" width="128"/> **`CSG::crescent(outer_r: Real, inner_r: Real, offset: Real, segments: usize, metadata: Option<S>)`** - 
- **`CSG::involute_gear_2d(module_: Real, teeth: usize, pressure_angle_deg: Real, clearance: Real, backlash: Real, segments_per_flank: usize, metadata: Option<S>)`** - under construction
- **`CSG::cycloidal_gear_2d(module_: Real, teeth: usize, pin_teeth: usize, clearance: Real, segments_per_flank: usize, metadata: Option<S>)`** - under construction
- **`CSG::involute_rack_2d(module_: Real, num_teeth: usize, pressure_angle_deg: Real, clearance: Real, backlash: Real, metadata: Option<S>)`** - under construction
- **`CSG::cycloidal_rack_2d(module_: Real, num_teeth: usize, generating_radius: Real, clearance: Real, segments_per_flank: usize, metadata: Option<S>)`** - under construction

```rust
let square = CSG::square(1.0, 1.0, None); // 1×1 at origin
let rect = CSG::square(2.0, 4.0, None);
let circle = CSG::circle(1.0, 32, None); // radius=1, 32 segments
let circle2 = CSG::circle(2.0, 64, None);

let font_data = include_bytes!("../fonts/MyFont.ttf");
let csg_text = CSG::text("Hello!", font_data, 20.0, None);

// Then extrude the text to make it 3D:
let text_3d = csg_text.extrude(1.0);
```

### Extrusions and Revolves

Extrusions build 3D polygons from 2D Geometries.

- <img src="docs/extrude.png" width="128"/> **`CSG::extrude(height: Real)`** - Simple extrude in Z+
- <img src="docs/extrude_vector.png" width="128"/> **`CSG::extrude_vector(direction: Vector3)`** - Extrude along Vector3 direction
- **`CSG::extrude_between(&polygon_bottom.polygons[0], &polygon_top.polygons[0], false)`** - Extrude Between Two BSP Polygons
- <img src="docs/rotate_extrude.png" width="128"/> **`CSG::rotate_extrude(angle_degs, segments)`** - Extrude while rotating around the Y axis

```rust
let square = CSG::square(2.0, 2.0, None);
let prism = square.extrude(5.0);

let revolve_shape = square.rotate_extrude(360.0, 16);

let polygon_bottom = CSG::circle(2.0, 64, None);
let polygon_top = polygon_bottom.translate(0.0, 0.0, 5.0);
let lofted = CSG::extrude_between(&polygon_bottom.polygons[0], &polygon_top.polygons[0], false);
```

### 3D Shapes

- <img src="docs/cube.png" width="128"/> **`CSG::cube(width: Real, length: Real, height: Real, metadata: Option<S>)`**
- <img src="docs/sphere.png" width="128"/> **`CSG::sphere(radius: Real, segments: usize, stacks: usize, metadata: Option<S>)`**
- <img src="docs/cylinder.png" width="128"/> **`CSG::cylinder(radius: Real, height: Real, segments: usize, metadata: Option<S>)`**
- <img src="docs/frustum.png" width="128"/> **`CSG::frustum(radius1: Real, radius2: Real, height: Real, segments: usize, metadata: Option<S>)`** -
Construct a frustum at origin with height and `radius1` and `radius2`.
If either radius is within EPSILON of 0.0, a cone terminating at a point is constructed.
- <img src="docs/frustum.png" width="128"/> **`CSG::frustum_ptp(start: Point3, end: Point3, radius1: Real, radius2: Real, segments:
usize, metadata: Option<S>)`** -
Construct a frustum from `start` to `end` with `radius1` and `radius2`.
If either radius is within EPSILON of 0.0, a cone terminating at a point is constructed.
- <img src="docs/polyhedron.png" width="128"/> **`CSG::polyhedron(points: &[[Real; 3]], faces: &[Vec<usize>], metadata: Option<S>)`**
- <img src="docs/octahedron.png" width="128"/> **`CSG::octahedron(radius: Real, metadata: Option<S>)`** -
- <img src="docs/icosahedron.png" width="128"/> **`CSG::icosahedron(radius: Real, metadata: Option<S>)`** -
- <img src="docs/torus.png" width="128"/> **`CSG::torus(major_r: Real, minor_r: Real, segments_major: usize, segments_minor: usize, metadata: Option<S>)`** -
- <img src="docs/egg.png" width="128"/> **`CSG::egg(width: Real, length: Real, revolve_segments: usize, outline_segments: usize, metadata: Option<S>)`**
- <img src="docs/teardrop3d.png" width="128"/> **`CSG::teardrop(width: Real, height: Real, revolve_segments: usize, shape_segments: usize, metadata: Option<S>)`**
- <img src="docs/teardrop_cylinder.png" width="128"/> **`CSG::teardrop_cylinder(width: Real, length: Real, height: Real, shape_segments: usize, metadata: Option<S>)`**
- <img src="docs/ellipsoid.png" width="128"/> **`CSG::ellipsoid(rx: Real, ry: Real, rz: Real, segments: usize, stacks: usize, metadata: Option<S>)`**
- <img src="docs/metaballs.png" width="128"/> **`CSG::metaballs(balls: &[MetaBall], resolution: (usize, usize, usize), iso_value: Real, padding: Real, metadata: Option<S>)`**
- <img src="docs/sdf-sphere.png" width="128"/> **`CSG::sdf<F>(sdf: F, resolution: (usize, usize, usize), min_pt: Point3, max_pt: Point3, iso_value: Real, metadata: Option<S>)`** - Return a CSG created by meshing a signed distance field within a bounding box
- <img src="docs/arrow_to.png" width="128"/> **`CSG::arrow(start: Point3, direction: Vector3, segments: usize, orientation: bool, metadata: Option<S>)`** - Create an arrow at start, pointing along direction
- <img src="docs/gyroid.png" width="128"/> **`CSG::gyroid(resolution: usize, period: Real, iso_value: Real, metadata: Option<S>)`** - Generate a Triply Periodic Minimal Surface (Gyroid) inside the volume of `self`
- <img src="docs/schwarzp.png" width="128"/> **`CSG::schwarz_p(resolution: usize, period: Real, iso_value: Real, metadata: Option<S>)`** - Generate a Triply Periodic Minimal Surface (Schwarz P) inside the volume of `self`
- <img src="docs/schwarzd.png" width="128"/> **`CSG::schwarz_d(resolution: usize, period: Real, iso_value: Real, metadata: Option<S>)`** - Generate a Triply Periodic Minimal Surface (Schwarz D) inside the volume of `self`
- **`CSG::helical_involute_gear(module_: Real, teeth: usize, pressure_angle_deg: Real, clearance: Real, backlash: Real, segments_per_flank: usize, thickness: Real, helix_angle_deg: Real, slices: usize, metadata: Option<S>)`** - under construction

```rust
// Unit cube at origin, no metadata
let cube = CSG::cube(1.0, 1.0, 1.0, None);

// Sphere of radius=2 at origin with 32 segments and 16 stacks
let sphere = CSG::sphere(2.0, 32, 16, None);

// Cylinder from radius=1, height=2, 16 segments, and no metadata
let cyl = CSG::cylinder(1.0, 2.0, 16, None);

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
let pyramid = CSG::polyhedron(points, &faces, None);

// Metaballs https://en.wikipedia.org/wiki/Metaballs
use csgrs::csg::MetaBall;
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

let csg_shape = CSG::from_sdf(my_sdf, resolution, min_pt, max_pt, iso_value, None);
```

### CSG Boolean Operations

```rust
let union_result = cube.union(&sphere);
let difference_result = cube.difference(&sphere);
let intersection_result = cylinder.intersection(&sphere);
```

They all return a new `CSG<S>`

### Transformations

- **`CSG::translate(x: Real, y: Real, z: Real)`** - Returns the CSG translated by x, y, and z
- **`CSG::translate_vector(vector: Vector3)`** - Returns the CSG translated by vector
- **`CSG::rotate(x_deg, y_deg, z_deg)`** - Returns the CSG rotated in x, y, and z
- **`CSG::scale(scale_x, scale_y, scale_z)`** - Returns the CSG scaled in x, y, and z
- **`CSG::mirror(plane: Plane)`** - Returns the CSG mirrored across plane
- **`CSG::center()`** - Returns the CSG centered at the origin
- **`CSG::float()`** - Returns the CSG translated so that its bottommost point(s) sit exactly at z=0
- **`CSG::transform(&Matrix4)`** - Returns the CSG after applying arbitrary affine transforms
- <img src="docs/distribute_arc.png" width="128"/> **`CSG::distribute_arc(count: usize, radius: Real, start_angle_deg: Real, end_angle_deg: Real)`**
- <img src="docs/distribute_line.png" width="128"/> **`CSG::distribute_linear(count: usize, dir: nalgebra::Vector3, spacing: Real)`**
- <img src="docs/distribute_grid.png" width="128"/> **`CSG::distribute_grid(rows: usize, cols: usize, dx: Real, dy: Real)`**

```rust
use nalgebra::Vector3;
use csgrs::plane::Plane;

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

- **`CSG::vertices()`** — collect all vertices from the CSG
- <img src="docs/inverse_sphere.png" width="128"/> **`CSG::inverse()`** — flips the inside/outside orientation.
- <img src="docs/convex_hull.png" width="128"/> **`CSG::convex_hull()`** — uses [`chull`](https://crates.io/crates/chull) to generate a 3D convex hull.
- <img src="docs/minkowski.png" width="128"/> **`CSG::minkowski_sum(&other)`** — naive Minkowski sum, then takes the hull.
- **`CSG::ray_intersections(origin, direction)`** — returns all intersection points and distances.
- **`CSG::flatten()`** — flattens a 3D shape into 2D (on the XY plane), unions the outlines.
- **`CSG::slice(plane)`** — slices the CSG by a plane and returns the cross-section polygons.
- **`CSG::offset(distance)`** — outward (or inward) offset in 2D using [`geo-offset`](https://crates.io/crates/geo-offset).
- <img src="docs/subdivided.png" width="128"/> **`CSG::subdivide_triangles(subdivisions)`** — subdivides each polygon’s triangles, increasing mesh density.
- **`CSG::renormalize()`** — re-computes each polygon’s plane from its vertices, resetting all normals.
- **`CSG::bounding_box()`** — computes the bounding box of the shape.
- **`CSG::tessellate()`** — triangulates all polygons returning a CSG containing triangles.
- **`CSG::from_polygons(polygons: &[Polygon<S>])`** - create a new CSG from Polygons.

### STL

- **Export ASCII STL**: `csg.to_stl_ascii("solid_name") -> String`
- **Export Binary STL**: `csg.to_stl_binary("solid_name") -> io::Result<Vec<u8>>`
- **Import STL**: `CSG::from_stl(&stl_data) -> io::Result<CSG<S>>`

```rust
// Save to ASCII STL
let stl_text = csg_union.to_stl_ascii("union_solid");
std::fs::write("union_ascii.stl", stl_text).unwrap();

// Save to binary STL
let stl_bytes = csg_union.to_stl_binary("union_solid").unwrap();
std::fs::write("union_bin.stl", stl_bytes).unwrap();

// Load from an STL file on disk
let file_data = std::fs::read("some_file.stl")?;
let imported_csg = CSG::from_stl(&file_data)?;
```

### DXF

- **Export**: `csg.to_dxf() -> Result<Vec<u8>, Box<dyn Error>>`
- **Import**: `CSG::from_dxf(&dxf_data) -> Result<CSG<S>, Box<dyn Error>>`

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
let csg_text = CSG::from_hershey("Hello!", font_data, 20.0, None);
```

### Create a Bevy `Mesh`

`csg.to_bevy_mesh()` returns a Bevy [`Mesh`](https://docs.rs/bevy/latest/bevy/prelude/struct.Mesh.html).

```rust
use csgrs::csg::CSG;
use bevy::{prelude::*, render::render_asset::RenderAssetUsages, render::mesh::{Indices, PrimitiveTopology}};

let bevy_mesh = csg_obj.to_bevy_mesh();
```

### Create a Parry `TriMesh`

`csg.to_trimesh()` returns a `SharedShape` containing a `TriMesh<Real>`.

```rust
use csgrs::csg::CSG;
use csgrs::float_types::rapier3d::prelude::*;  // re-exported for f32/f64 support

let trimesh_shape = csg_obj.to_trimesh(); // SharedShape with a TriMesh
```

### Create a Rapier Rigid Body

`csg.to_rigid_body(rb_set, co_set, translation, rotation, density)` helps build and insert both a rigid body and a collider:

```rust
use nalgebra::Vector3;
use csgrs::float_types::rapier3d::prelude::*;  // re-exported for f32/f64 support
use csgrs::float_types::FRAC_PI_2;
use csgrs::csg::CSG;

let mut rb_set = RigidBodySet::new();
let mut co_set = ColliderSet::new();

let axis_angle = Vector3::z() * FRAC_PI_2; // 90° around Z
let rb_handle = csg_obj.to_rigid_body(
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
let (mass, com, inertia_frame) = csg_obj.mass_properties(density);
println!("Mass: {}", mass);
println!("Center of Mass: {:?}", com);
println!("Inertia local frame: {:?}", inertia_frame);
```

### Manifold Check

`csg.is_manifold()` triangulates the CSG, builds a HashMap of all edges (pairs of vertices), and checks that each is used exactly twice. Returns `true` if manifold, `false` if not.

```rust
if (csg_obj.is_manifold()){
    println!("CSG is manifold!");
} else {
    println!("Not manifold.");
}
```

## Working with Metadata

`CSG<S>` is generic over `S: Clone`. Each polygon has an optional `metadata: Option<S>`.  
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

type CSG = csgrs::CSG<MyMetadata>;

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

## Performance
Patterns we work to follow throughout the library to improve performance
and memory usage:
- functions should accept borrowed slices, this permits easy use of iterators
- iterators should be used wherever parallelism may help (and rayon's par_iter)
- allocations should be kept to a minimum.  Memory should be read-only if
possible, clone if necessary, and offer the choice of transmut in place or
create new copy via appropriate functions

## Roadmap / Todo
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
- gpu accelleration
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
- adapt cavalier_contours demo application
- rethink metadata
  - support storing UV[W] coordinates with vertexes at compile time (try to keep runtime cost low too)
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
- implement 2D line, point, LineString functions for CSG
- https://github.com/hmeyer/tessellation
- emit TrueType glyphs into the same MultiPolygon for each call of text()
- evaluate using approx crate
- evaluate using https://docs.rs/nalgebra/latest/nalgebra/trait.RealField.html instead of float_types::Real
- mutable API for transmute, etc.
- implement trait geo::MetricSpace on nalgebra::Point, Point2, Point3
- investigate https://github.com/TimTheBig/geo-3d for useful functions
- gltf output
- gerber output
- rework bezier and bspline using https://github.com/mattatz/curvo
  - import functions from https://github.com/nical/lyon/tree/main/crates/geom/src for cubic and quadratic bezier

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
