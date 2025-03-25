# Shapes

### 2D Shapes

- **`CSG::square(width: Real, length: Real, metadata: Option<S>)`**
- **`CSG::circle(radius: Real, segments: usize, metadata: Option<S>)`**
- **`CSG::polygon(&[[x1,y1],[x2,y2],...], metadata: Option<S>)`**
- **`CSG::rounded_rectangle(width: Real, height: Real, corner_radius: Real, corner_segments: usize, metadata: Option<S>)`**
- **`CSG::ellipse(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- **`CSG::regular_ngon(sides: usize, radius: Real, metadata: Option<S>)`**
- **`CSG::right_triangle(width: Real, height: Real, metadata: Option<S>)`**
- **`CSG::trapezoid(top_width: Real, bottom_width: Real, height: Real, top_offset: Real, metadata: Option<S>)`**
- **`CSG::star(num_points: usize, outer_radius: Real, inner_radius: Real, metadata: Option<S>)`**
- **`CSG::teardrop(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- **`CSG::egg_outline(width: Real, length: Real, segments: usize, metadata: Option<S>)`**
- **`CSG::squircle(width: Real, height: Real, segments: usize, metadata: Option<S>)`**
- **`CSG::keyhole(circle_radius: Real, handle_width: Real, handle_height: Real, segments: usize, metadata: Option<S>)`**
- **`CSG::reuleaux_polygon(sides: usize, radius: Real, arc_segments_per_side: usize, metadata: Option<S>)`** final shape not yet achieved
- **`CSG::ring(id: Real, thickness: Real, segments: usize, metadata: Option<S>)`**
- **`CSG::pie_slice(radius: Real, start_angle_deg: Real, end_angle_deg: Real, segments: usize, metadata: Option<S>)`**
- **`CSG::metaball_2d(balls: &[(nalgebra::Point2<Real>, Real)], resolution: (usize, usize), iso_value: Real, padding: Real, metadata: Option<S>)`** failing at the moment, pending rework
- **`CSG::supershape(a: Real, b: Real, m: Real, n1: Real, n2: Real, n3: Real, segments: usize, metadata: Option<S>)`**
- **`CSG::circle_with_keyway(radius: Real, segments: usize, key_width: Real, key_depth: Real, metadata: Option<S>)`**
- **`CSG::circle_with_flat(radius: Real, segments: usize, flat_dist: Real, metadata: Option<S>)`**
- **`CSG::circle_with_two_flats(radius: Real, segments: usize, flat_dist: Real, metadata: Option<S>)`**
- **`CSG::from_image(img: &GrayImage, threshold: u8, closepaths: bool, metadata: Option<S>)`** - Builds a new CSG from the “on” pixels of a grayscale image
- **`CSG::text(text: &str, font_data: &[u8], size: Real, metadata: Option<S>)`** - generate 2D text geometry in the XY plane from TTF fonts via [`meshtext`](https://crates.io/crates/meshtext)

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

### 3D Shapes

- **`CSG::cube(width: Real, length: Real, height: Real, metadata: Option<S>)`**
- **`CSG::sphere(radius: Real, segments: usize, stacks: usize, metadata: Option<S>)`**
- **`CSG::cylinder(radius: Real, height: Real, segments: usize, metadata: Option<S>)`**
- **`CSG::frustum(radius1: Real, radius2: Real, height: Real, segments: usize, metadata: Option<S>)`** - Construct a frustum at origin with height and `radius1` and `radius2`.  If either radius is within EPSILON of 0.0, a cone terminating at a point is constructed.
- **`CSG::frustum_ptp(start: Point3, end: Point3, radius1: Real, radius2: Real, segments: usize, metadata: Option<S>)`** - Construct a frustum from `start` to `end` with `radius1` and `radius2`.  If either radius is within EPSILON of 0.0, a cone terminating at a point is constructed.
- **`CSG::polyhedron(points: &[[Real; 3]], faces: &[Vec<usize>], metadata: Option<S>)`**
- **`CSG::egg(width: Real, length: Real, revolve_segments: usize, outline_segments: usize, metadata: Option<S>)`**
- **`CSG::teardrop(width: Real, height: Real, revolve_segments: usize, shape_segments: usize, metadata: Option<S>)`**
- **`CSG::teardrop_cylinder(width: Real, length: Real, height: Real, shape_segments: usize, metadata: Option<S>)`**
- **`CSG::ellipsoid(rx: Real, ry: Real, rz: Real, segments: usize, stacks: usize, metadata: Option<S>)`**
- **`CSG::metaballs(balls: &[MetaBall], resolution: (usize, usize, usize), iso_value: Real, padding: Real, metadata: Option<S>)`**
- **`CSG::sdf<F>(sdf: F, resolution: (usize, usize, usize), min_pt: Point3, max_pt: Point3, iso_value: Real, metadata: Option<S>)`** - Return a CSG created by meshing a signed distance field within a bounding box
- **`CSG::arrow(start: Point3, direction: Vector3, segments: usize, orientation: bool, metadata: Option<S>)`** - Create an arrow at start, pointing along direction
- **`CSG::gyroid(resolution: usize, period: Real, iso_value: Real, metadata: Option<S>)`** - Generate a Triply Periodic Minimal Surface (Gyroid) inside the volume of `self` - not yet working

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
