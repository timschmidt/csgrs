# csgrs

Constructive Solid Geometry (CSG) is a modeling technique that uses Boolean operations like union and intersection to combine 3D solids. This library implements CSG operations on meshes simply using BSP trees.  It is meant to add CSG to the larger [Dimforge](https://www.dimforge.com/) ecosystem, bring the [OpenSCAD](https://openscad.org/) feature set into Rust, work in a wide variety of environments, and be reasonably performant.

![Example CSG output](docs/csg.png)

## Use the library:

    use csgrs::CSG;
    
    // Create a type alias for easy usage
    type MyCSG = CSG<()>;

## Construct a 2D shape:

    let square = MyCSG::square(None);
    let square2 = MyCSG::square(Some(([2.0, 3.0], true)));
    let circle = MyCSG::circle(None);
    let circle2 = MyCSG::circle(Some((2.0, 64)));
    
    let points = vec![[0.0, 0.0], [2.0, 0.0], [1.0, 1.5]];
    let polygon2d = MyCSG::polygon_2d(&points);

## Construct a 3D shape:

    let cube = MyCSG::cube(None);
    let cube2 = MyCSG::cube(Some([0.0, 0.0, 0.0], [1.0, 1.0, 1.0])); // center, radius
    let sphere = MyCSG::sphere(None);
    let sphere2 = MyCSG::sphere(Some([0.0, 0.0, 0.0], 1.0, 16, 8)); // center, radius, slices, stacks
    let cylinder = MyCSG::cylinder(None);
    let cylinder2 = MyCSG::cylinder(Some([0.0, -1.0, 0.0], [0.0, 1.0, 0.0], 1.0, 16)); // start, end, radius, slices
    
    // A simple triangular prism
    let points = [
        [0.0, 0.0, 0.0], // 0
        [1.0, 0.0, 0.0], // 1
        [0.0, 1.0, 0.0], // 2
        [0.0, 0.0, 1.0], // 3
        [1.0, 0.0, 1.0], // 4
        [0.0, 1.0, 1.0], // 5
    ];
    // Faces: bottom triangle, top triangle, and 3 rectangular sides
    let faces = vec![
        vec![0, 1, 2],    // bottom
        vec![3, 5, 4],    // top
        vec![0, 2, 5, 3], // side
        vec![0, 3, 4, 1], // side
        vec![1, 4, 5, 2], // side
    ];
    let prism = MyCSG::polyhedron(&points, &faces);

## Combine shapes:

    let union_result = cube.union(&sphere);
    let subtraction_result = cube.subtract(&sphere);
    let intersection_result = cylinder.intersect(&sphere);

## Extract polygons:

    let polygons = union_result.to_polygons();
    println!("Polygon count = {}", polygons.len());

## Translate:

    let translation_result = cube.translate(Vector3::new(3.0, 2.0, 1.0));

## Rotate:

    let rotation_result = cube.rotate(15.0, 45.0, 0.0);

## Scale:

    let scale_result = cube.scale(2.0, 1.0, 3.0);

## Mirror:

    let mirror_result = cube.mirror(Axis::Y);
    
## Convex hull:

    let hull = cube.convex_hull();

## Minkowski sum:

    let rounded_cube = cube.minkowski_sum(&sphere);
    
## Project a 3D shape into 2D

    let cube = MyCSG::cube(None);
    let projection = cube.project();
    
## Cut a shape with a plane:

    let cut = cube.cut(None); // cut at z=0
    
    let plane = Plane {
        normal: nalgebra::Vector3::new(0.0, 0.0, 1.0),
        w: 1.0,
    }
    let slice = cube.cut(plane); // cut at z=1
    
## Extrude a 2D shape:

    let square = MyCSG::square(Some(([2.0, 2.0], true)));
    let prism = square.extrude(5.0);
    
## Extrude along a vector:

    // Extrude along the +Y direction by 5 units:
    let extruded_y = my_2d_shape.extrude_along(Vector3::new(0.0, 5.0, 0.0));
    
    // Extrude along some arbitrary vector, say (1.0, 2.0, 3.0):
    let extruded_diagonal = my_2d_shape.extrude_along(Vector3::new(1.0, 2.0, 3.0));
    
## Extrude between two polygons:

    let circle = MyCSG::circle(Some((2.0, 64)));
    let circle2 = MyCSG::circle(Some((2.0, 64)));
    let solid = CSG::extrude_between(circle, circle2.translate(Vector3::new(3.0, 2.0, 5.0)));
    
## Rotate extrude:

    let polygon = MyCSG::polygon_2d(&[
        [1.0, 0.0],
        [1.0, 2.0],
        [0.5, 2.5],
    ]);
    let revolve_shape = polygon.rotate_extrude(360.0, 16); // degrees, steps
    
## [Transform](https://en.wikipedia.org/wiki/Transformation_matrix#Affine_transformations):

    // Scale X, Shear X along Y, Shear X along Z, Translate X
    // Shear Y along X, Scale Y, Shear Y along Z, Translate Y
    // Shear Z along X, Shear Z along Y, Scale Z, Translate Z
    // The last row are clamped to 0,0,0,1 in OpenSCAD
    
    cube.transform(Matrix4x4::new(11, 12, 13, 14,
                                  21, 22, 23, 24,
                                  21, 22, 23, 24,
                                  0, 0, 0, 1));
    
## Bounding box:

    let aabb = cube.bounding_box();
    println!("Axis-aligned bounding box mins: {:?}", aabb.mins);
    println!("Axis-aligned bounding box maxs: {:?}", aabb.maxs);
    
## Offset a 3D shape: (bugged atm)

    let grown_cube = cube.grow(4.0);
    let shrunk_cube = cube.shrink(4.0);

## Offset a 2D shape:

    let grown_square = square.offset_2d(4.0);
    let shrunk_square = square.offset_2d(-4.0);
    
## Text:

    let font_data = include_bytes!("my_font.ttf");

    // Generate a simple "Hello" text in the XY plane
    let csg_text = MyCSG::text_mesh("Hello", font_data, Some(10.0));
    
## Subdivide triangles:

    let subdivisions = 2;
    let subdivided_csg = rounded_cube.subdivide_triangles(subdivisions);
    
## Renormalize:

    let renormalized_csg = cube.renormalize();
    
## Compute all ray intersections for measurement (expensive):

    let cube = MyCSG::cube(None);
    let ray_origin = nalgebra::Point3::new(-5.0, 0.0, 0.0);
    let ray_dir    = nalgebra::Vector3::new(1.0, 0.0, 0.0);

    let intersections = cube.ray_intersections(&ray_origin, &ray_dir);
    println!("Found {} intersections:", intersections.len());
    for (point, dist) in intersections {
        println!("  t = {:.4}, point = {:?}", dist, point); // distance to 4 decimal places
    }

## Create a [Parry](https://parry.rs/) TriMesh:

    let trimesh = my_csg.to_trimesh();

## Create a [Rapier](https://rapier.rs/) rigid body:

    // 90 degrees in radians
    let angle = std::f64::consts::FRAC_PI_2;
    // Axis-angle: direction = Z, magnitude = angle
    let axis_angle = Vector3::z() * angle;
    
    let rigid_body = my_csg.to_rigid_body(
        &mut rigid_body_set,
        &mut collider_set,
        Vector3::new(0.0, 0.0, 0.0), // translation
        axis_angle,                  // 90° around Z
        1.0,                         // density
    );
    
## Collect mass properties of a shape:

    let density = 1.0;
    let (mass, center_of_mass, inertia_frame) = my_csg.mass_properties(density);

## Export an ASCII STL:

    let stl_data = union_result.to_stl_ascii("cube_minus_sphere");
    std::fs::write("output.stl", stl_data.as_bytes())?;
    
## Export a binary STL:

    let bytes = union_result.to_stl_binary("my_solid")?;
    std::fs::write("output.stl", bytes)?;

## Import an STL:

    let stl_data: Vec<u8> = std::fs::read("path_to_stl_file.stl")?;
    let csg = MyCSG::from_stl(&stl_data)?;
    
## Generic per-object and per-polygon metadata:

In order to allow you to store custom per-polygon metadata (colors, IDs, etc.), `csgrs` now has a generic type parameter `S: Clone` on both `CSG<S>` and `Polygon<S>`.  If you don’t need custom data, you can simply use `()`, an empty type, for `S`.

    // No metadata:
    type MyCSG = CSG<()>;
    let cube = MyCSG::cube(None);

If you do want custom data, define your own type that implements Clone:

    #[derive(Clone)]
    struct MyMetadata {
        color: (u8, u8, u8),
        layer_id: u32,
        // etc.
    }
    
    // Then alias with the custom type:
    type MyCSG = CSG<MyMetadata>;
    
    // Or instantiate directly:
    let mut csg = CSG::<MyMetadata>::new();

The various shape functions (`cube`, `sphere`, etc.) produce polygons whose `shared` field is `None` by default.

## Getting and setting metadata:

Once you have a `CSG<S>`, you can access its polygons (either via `csg.polygons` or `csg.to_polygons()`) and use the following helper methods on each `Polygon<S>`:

    shared_data() -> Option<&S>: Returns a reference to the metadata if present.
    shared_data_mut() -> Option<&mut S>: Returns a mutable reference to the metadata.
    set_shared_data(value: S): Overwrites the metadata with a new value.
    
    // Create a CSG with a single polygon that has a string metadata value:
    let mut poly = Polygon::new(
        vec![
            Vertex::new(Point3::new(0.0, 0.0, 0.0), nalgebra::Vector3::z()),
            Vertex::new(Point3::new(1.0, 0.0, 0.0), nalgebra::Vector3::z()),
            Vertex::new(Point3::new(0.0, 1.0, 0.0), nalgebra::Vector3::z()),
        ],
        Some("MyTriangle".to_string()),
    );
    
    // Access the data
    if let Some(data) = poly.shared_data() {
        println!("Metadata data is: {}", data);
    }
    
    // Mutably modify
    if let Some(data_mut) = poly.shared_data_mut() {
        data_mut.push_str("_extended");
    }
    
    // Or directly set
    poly.set_shared_data("OverwrittenData".to_string());
    
    // Make a CSG from polygons
    let csg = CSG::from_polygons(vec![poly]);

## Implementation Details

All CSG operations are implemented in terms of two functions, `clip_to()` and `invert()`, which remove parts of a BSP tree inside another BSP tree and swap solid and empty space, respectively. To find the union of `a` and `b`, we want to remove everything in `a` inside `b` and everything in `b` inside `a`, then combine polygons from `a` and `b` into one solid:

    a.clip_to(&b);
    b.clip_to(&a);
    a.build(&b.all_polygons());

The only tricky part is handling overlapping coplanar polygons in both trees. The code above keeps both copies, but we need to keep them in one tree and remove them in the other tree. To remove them from `b` we can clip the inverse of `b` against `a`. The code for union now looks like this:

    a.clip_to(&b);
    b.clip_to(&a);
    b.invert();
    b.clip_to(&a);
    b.invert();
    a.build(&b.all_polygons());

Subtraction and intersection naturally follow from set operations. If union is `A | B`, subtraction is `A - B = ~(~A | B)` and intersection is `A & B = ~(~A | ~B)` where `~` is the complement operator.

## Todo
- rename shared data functions to metadata functions
- fix normals on rotate_extrude
- fix normal on bottom face of extrude
- determine why flattened_cube.stl produces invalid output with to_stl_binary but not to_stl_ascii
- determine why square_2d_shrink.stl produces invalid output with to_stl_binary but not to_stl_ascii
- determine why square_2d produces invalid output with to_stl_binary but not to_stl_ascii
- 2d boolean ops
  - functions: signed area, is_ccw, line/line intersection, intersection, union, difference
  - tests / implementation with cavalier_contours
- vector font for machining
  - https://github.com/kamalmostafa/hershey-fonts
    - https://github.com/kicad-rs/hershey/blob/main/src/lib.rs
  - http://www.ofitselfso.com/MiscNotes/CAMBamStickFonts.php
- https://crates.io/crates/contour_tracing
- evaluate https://github.com/gfx-rs/genmesh
- https://github.com/PsichiX/density-mesh
- implement 2d offsetting with these for testing against cavalier_contours
  - https://github.com/Akirami/polygon-offsetting
  - https://github.com/anlumo/offset_polygon
- support twist and scale in linear extrude like openscad
- support scale and translation along a vector in rotate extrude
- extruding a line does not currently result in a 2D shape as it has fewer than three points
- dxf/svg import/export
- fragments (circle, sphere, regularize with rotate_extrude)
- extend polygon to handle multiple loops, outside and holes, using earclip on from_polygons, plane splitting
- fill
- 32bit / 64bit feature
- parallelize clip_to and invert with rayon and par_iter
- identify more candidates for par_iter
- manifoldness tests / fixes - in stl_io library
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
- implement arc support in 2d using cavalier_contours, tessellate in from_polygons
- reconstruct arcs from polylines using 
- extend Polygon to allow edges to store arc parameters and bulge like cavalier_contours and update split_polygon to handle line/arc intersections.

# License

Copyright (c) 2025 Timothy Schmidt, initially based on a translation of CSG.js Copyright (c) 2011 Evan Wallace, under the [MIT license](http://www.opensource.org/licenses/mit-license.php).
