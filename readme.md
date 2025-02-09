# csgrs

A **Constructive Solid Geometry (CSG)** library in Rust, built around Boolean operations (*union*, *difference*, *intersection*) on sets of polygons stored in BSP trees. **csgrs** enables you to construct 2D and 3D geometry with an [OpenSCAD](https://openscad.org/)-like syntax, and to transform, interrogate, and simulate those shapes without leaving Rust.

This library aims to integrate cleanly with the [Dimforge](https://www.dimforge.com/) ecosystem (e.g., [`nalgebra`](https://crates.io/crates/nalgebra), [`Parry`](https://crates.io/crates/parry3d), and [`Rapier`](https://crates.io/crates/rapier3d)), leverage [`earclip`](https://crates.io/crates/earclip) and [`cavalier_contours`](https://crates.io/crates/cavalier_contours) for robust mesh and line processing, be reasonably performant on a wide variety of targets, and provide an extensible, type-safe API.

![Example CSG output](docs/csg.png)

## Getting started

Install the [Rust](https://www.rust-lang.org/) language tools from [rustup.rs](https://rustup.rs/).

```shell
cargo new my_cad_project
cd my_cad_project
cargo add csgrs
cargo add nalgebra // provides Points, Vectors, etc. 
```

### Example main.rs

```rust
// Alias the library’s generic CSG type with empty metadata:
type CSG = csgrs::csg::CSG<()>;

// Create two shapes:
let cube = CSG::cube(2.0, 2.0, 2.0, None);  // 2×2×2 cube at origin, no metadata
let sphere = CSG::sphere(1.0, 16, 8, None); // sphere of radius=1 at origin, no metadata

// Subtract one from the other:
let subtract_result = cube.subtract(&sphere);

// Write the result as an ASCII STL:
let stl = subtract_result.to_stl_ascii("cube_minus_sphere");
std::fs::write("cube_sphere_difference.stl", stl).unwrap();
```

### CSG and Polygon Structures

- **`CSG<S>`** is the main type. It stores a list of **polygons** (`Vec<Polygon<S>>`).
- **`Polygon<S>`** holds:
  - a `Vec<Vertex>` (positions + normals),
  - a `bool` indicating whether the polyline is open or closed,
  - an optional metadata field (`Option<S>`), and
  - a `Plane` describing the polygon’s orientation in 3D.

`CSG<S>` provides methods for working with 3D shapes, `Polygon<S>` provides methods for working with 2D shapes. You can build a `CSG<S>` from polygons with `CSG::from_polygons(...)`.  Some 2D functions are re-exported by `CSG<S>` for ease of use.

### 2D Shapes

Helper constructors for 2D shapes in the XY plane:

- `CSG::square(width: Real, length: Real, metadata: Option<S>)`
- `CSG::circle(radius: Real, segments: usize, metadata: Option<S>)`
- `CSG::polygon_2d(&[[x1,y1],[x2,y2],...], metadata: Option<S>)`

```rust
let square = CSG::square(1.0, 1.0, None); // 1×1 at origin
let rect = CSG::square(2.0, 4.0, None);
let circle = CSG::circle(1.0, 32, None); // radius=1, 32 segments
let circle2 = CSG::circle(2.0, 64, None);
```

### 3D Shapes

Similarly, you can create standard 3D primitives:

- `CSG::cube(width: Real, length: Real, height: Real, metadata: Option<S>)`
- `CSG::sphere(radius: Real, segments: usize, stacks: usize, metadata: Option<S>)`
- `CSG::cylinder(radius: Real, height: Real, segments: usize, metadata: Option<S>)`
- `CSG::cylinder_ptp(start: Point3, end: Point3, radius: Real, segments: usize, metadata: Option<S>)`
- `CSG::polyhedron(points: &[[Real; 3]], faces: &[Vec<usize>], metadata: Option<S>)`

```rust
// Unit cube at origin, no metadata
let cube = CSG::cube(1.0, 1.0, 1.0, None);

// Sphere of radius=2 at origin with 32 segments and 16 stacks
let sphere = CSG::sphere(2.0, 32, 16, None);

// Cylinder from radius=1, height=2, 16 slices, and no metadata
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
```

### Boolean Operations

Three primary operations:

1. **Union**: `a.union(&b)`
2. **Difference**: `a.subtract(&b)`
3. **Intersection**: `a.intersect(&b)`

They all return a new `CSG<S>`.

```rust
let union_result = cube.union(&sphere);
let subtraction_result = cube.subtract(&sphere);
let intersection_result = cylinder.intersect(&sphere);
```

### Transformations

- `translate(vector: Vector3<Real>)`
- `rotate(x_deg, y_deg, z_deg)`
- `scale(scale_x, scale_y, scale_z)`
- `mirror(Axis::X | Axis::Y | Axis::Z)`
- `transform(&Matrix4<Real>)` for arbitrary affine transforms.

```rust
use nalgebra::Vector3;

let moved = cube.translate(Vector3::new(3.0, 0.0, 0.0));
let rotated = sphere.rotate(0.0, 45.0, 90.0);
let scaled = cylinder.scale(2.0, 1.0, 1.0);
let mirrored = cube.mirror(Axis::Z);
```

### Extrusions and Revolves

- **Linear Extrude**: 
  - `my_2d_shape.extrude(height: Real)`  
  - `my_2d_shape.extrude_vector(direction: Vector3<Real>)`  
- **Extrude Between Two Polygons**:  
  ```rust
  let polygon_bottom = CSG::circle(2.0, 64, None);
  let polygon_top = polygon_bottom.translate(Vector3::new(0.0, 0.0, 5.0));
  let lofted = CSG::extrude_between(&polygon_bottom.polygons[0],
                                      &polygon_top.polygons[0],
                                      false);
  ```
- **Rotate-Extrude (Revolve)**: `my_2d_shape.rotate_extrude(angle_degs, segments)`

```rust
let square = CSG::square(2.0, 2.0, None);
let prism = square.extrude(5.0);

let revolve_shape = square.rotate_extrude(360.0, 16);
```

### Miscellaneous Operations

- **`CSG::inverse()`** — flips the inside/outside orientation.
- **`CSG::convex_hull()`** — uses [`chull`](https://crates.io/crates/chull) to generate a 3D convex hull.
- **`CSG::minkowski_sum(&other)`** — naive Minkowski sum, then takes the hull.
- **`CSG::ray_intersections(origin, direction)`** — returns all intersection points and distances.
- **`CSG::flatten()`** — flattens a 3D shape into 2D (on the XY plane), unions the outlines.
- **`CSG::slice(Some(plane))`** — slices the CSG by a plane and returns the cross-section polygons.
- **`CSG::offset_2d(distance)`** — outward (or inward) offset in 2D using [cavalier_contours](https://crates.io/crates/cavalier_contours).
- **`CSG::subdivide_triangles(subdivisions)`** — subdivides each polygon’s triangles, increasing mesh density.
- **`CSG::renormalize()`** — re-computes each polygon’s plane from its vertices, resetting all normals.
- **`CSG::reconstruct_polyline_3d(polylines: &[Polygon<S>])`** — reconstructs a 3d polyline from 2d polylines with matching start/end points
- **`CSG::bounding_box()`** — computes the bounding box of the shape
- **`CSG::retriangulate()`** — retriangulates all polygons with [`earclip`](https://crates.io/crates/earclip)
- **`CSG::extrude_polyline(poly: Polyline<Real>, direction: Vector3<Real>, metadata: Option<S>)`** — extrude an open ended polyline to create a surface
- **`CSG::from_polylines(polylines: Vec<Polyline<Real>>, metadata: Option<S>)`** — create a new CSG from [cavalier_contours](https://crates.io/crates/cavalier_contours) polylines
- **`CSG::vertices()`** — collect all vertices from the CSG

### Working with Metadata

`CSG<S>` is generic over `S: Clone`. Each polygon has an optional `metadata: Option<S>`.  
Use cases include storing color, ID, or layer info.

```rust
use csgrs::{Polygon, Vertex};
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
        Vertex::new(Point3::new(0.0, 0.0, 0.0), Vector3::z()),
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

### TrueType Text

You can generate 2D text geometry in the XY plane from TTF fonts via [`meshtext`](https://crates.io/crates/meshtext):

```rust
let font_data = include_bytes!("../fonts/MyFont.ttf");
let csg_text = CSG::text("Hello!", font_data, Some(20.0), None);

// Then extrude the text to make it 3D:
let text_3d = csg_text.extrude(1.0);
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
    true => println!("CSG is manifold!"),
} else {
    false => println!("Not manifold."),
}
```

---
## 2D Subsystem and Polygon‐Level 2D Operations

Although **CSG** typically focuses on three‐dimensional Boolean operations, this library also provides a robust **2D subsystem** built on top of [cavalier_contours](https://crates.io/crates/cavalier_contours). Each `Polygon<S>` in 3D can be **projected** into 2D (its own local XY plane) for 2D boolean operations such as **union**, **difference**, **intersection**, and **xor**. These are especially handy if you’re offsetting shapes, working with complex polygons, or just want 2D output.

Below is a quick overview of the **2D‐related methods** you’ll find on `Polygon<S>`:

### Polygon::to_2d() and Polygon::from_2d(...)
- **`to_2d()`**  
  Projects the polygon from its 3D plane into a 2D [`Polyline<f64>`](https://docs.rs/cavalier_contours/latest/cavalier_contours/polyline/struct.Polyline.html).  
  Internally:
  1. Finds a transform that sends `polygon.plane.normal` to the +Z axis.
  2. Transforms each vertex into that local coordinate system (so the polygon lies at *z = 0*).
  3. Returns a 2D `Polyline<f64>` of `(x, y, bulge)` points (here, `bulge` is set to `0.0` by default).

- **`from_2d(polyline, Some(metadata))`**  
  The inverse of `to_2d()`, creating a 3D `Polygon` from a 2D `Polyline<Real>`. This method uses the **same** plane as the polygon on which you called `from_2d()`. That is, it takes `(x, y)` points in the local XY plane of `self.plane` and lifts them back into 3D space.

These two functions let you cleanly convert between a 3D polygon and a pure 2D representation whenever you need to do 2D manipulations.  

> **Tip**: If your polygons truly are already in the global XY plane (i.e., `z ≈ 0`), or you would like to flatten them without adjusting for their reference plane, you can use `Polygon::to_polyline()` and `Polygon::from_polyline(...)`. Those skip the plane‐based transform and simply store or read `(x, y, 0.0)` directly.

### 2D Boolean Operations

A `Polygon<S>` supports **union**, **difference**, **intersection**, and **xor** in 2D. Each of these methods:
- Projects **both** polygons into 2D via `to_2d()`.
- Invokes [cavalier_contours](https://crates.io/crates/cavalier_contours) to compute the boolean operation.
- Reconstructs one or more resulting polygons in 3D using `from_2d(...)`.

Each operation returns a `Vec<Polygon<S>>` rather than a single polygon, because the result may split into multiple disjoint pieces.  

- **`union(&other) -> Vec<Polygon<S>>`**  
  `self ∪ other`. Merges overlapping or adjacent areas.

- **`intersection(&other) -> Vec<Polygon<S>>`**  
  `self ∩ other`. Keeps only overlapping regions.

- **`difference(&other) -> Vec<Polygon<S>>`**  
  `self \ other`. Subtracts `other` from `self`.

- **`xor(&other) -> Vec<Polygon<S>>`**  
  Symmetric difference `(self ∪ other) \ (self ∩ other)`—keeps regions that belong to exactly one polygon.

Example usage:
```rust
let p1 = polygon_a.union(&polygon_b);          // 2D union
let p2 = polygon_a.intersection(&polygon_b);   // 2D intersection
let p3 = polygon_a.difference(&polygon_b);     // 2D difference
let p4 = polygon_a.xor(&polygon_b);            // 2D xor
```

### Transformations

- `translate(v: Vector3<Real>)`
- `rotate(axis: Vector3<Real>, angle: Real, center: Option<Point3<Real>>)`
- `scale(factor: Real)`
- `mirror(Axis::X | Axis::Y | Axis::Z)`
- `transform(&Matrix4<Real>)` for arbitrary affine transforms.
- `flip()`
- `convex_hull()`
- `minkowski_sum(other: Polygon<S>)`

### Misc functions

- `subdivide_triangles()`
- `calculate_new_normal()`
- `triangulate()`
- `recalc_plane_and_normals()`
- `offset_open(distance)`
- `reconstruct_arcs(min_match: usize, rms_limit: Real, angle_limit_degs: Real, offset_limit: Real)`
- `check_coordinates_finite()`
- `check_repeated_points()`
- `check_ring_closed()`
- `check_minimum_ring_size()`
- `check_ring_self_intersection()`

### Signed Area (Shoelace)
The `pline_area` function computes the signed area of a closed `Polyline<Real>`:
- **Positive** if the points are in **counterclockwise (CCW)** order.
- **Negative** if the points are in **clockwise (CW)** order.
- Near‐zero for degenerate or collinear loops.

---

## Roadmap / Todo
- fix up error handling with result types
- sweep following path
- convert more for loops to iterators
- file formats behind a feature flag
- parry, rapier behind feature flags
- polygons_by_metadata public function of a CSG
  - draft implementation done, pending API discussion
- extend flatten to work with arbitrary planes
- overwrite polygon metadata correctly in difference, intersection, etc
- fix normals on rotate_extrude
- determine why flattened_cube.stl produces invalid output with to_stl_binary but not to_stl_ascii
- determine why square_2d_shrink.stl produces invalid output with to_stl_binary but not to_stl_ascii
- determine why square_2d produces invalid output with to_stl_binary but not to_stl_ascii
- remaining 2d functions to finalize: signed area, is_ccw, line/line intersection
  - tests
- lazy transformations?
- invert Polygon::open to match cavalier_contours
- use &[T] over Vec<T> where possible
- vector font for machining / svg import/export
  - https://github.com/kamalmostafa/hershey-fonts
    - https://github.com/kicad-rs/hershey/blob/main/src/lib.rs
  - http://www.ofitselfso.com/MiscNotes/CAMBamStickFonts.php
- https://crates.io/crates/contour_tracing
- https://github.com/PsichiX/density-mesh
- evaluate https://github.com/MIERUNE/earcut-rs vs earclip
- screw threads
- implement 2d offsetting with these for testing against cavalier_contours
  - https://github.com/Akirami/polygon-offsetting
  - https://github.com/anlumo/offset_polygon
- support twist and scale in linear extrude like openscad
- support scale and translation along a vector in rotate extrude
- fill
- implement .center()
- parallelize clip_to and invert with rayon and par_iter
- identify more candidates for par_iter
- reimplement 3D offsetting with voxelcsgrs or https://docs.rs/parry3d/latest/parry3d/transformation/vhacd/struct.VHACD.html
- reimplement convex hull with https://docs.rs/parry3d-f64/latest/parry3d_f64/transformation/fn.convex_hull.html
- implement 2d/3d convex decomposition with https://docs.rs/parry3d-f64/latest/parry3d_f64/transformation/vhacd/struct.VHACD.html
- reimplement transformations and shapes with https://docs.rs/parry3d/latest/parry3d/transformation/utils/index.html
- evaluate https://github.com/asny/tri-mesh for useful functions
- identify blockers for no-std
- identify opportunities to use parry2d_f64 and parry3d_f64 modules and functions to simplify and enhance our own
  - https://docs.rs/parry2d-f64/latest/parry2d_f64/index.html
  - https://docs.rs/parry3d-f64/latest/parry3d_f64/index.html

## Todo maybe
- implement constant radius arc support in 2d using cavalier_contours, interpolate/tessellate in from_polygons
- extend Polygon to allow edges to store bulge like cavalier_contours and update split_polygon to handle line/arc intersections.

---

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
