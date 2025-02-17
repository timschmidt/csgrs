// main.rs
//
// Minimal example of each function of csgrs (which is now generic over the shared-data type S).
// Here, we do not use any shared data, so we'll bind the generic S to ().

use std::fs;
use nalgebra::{Vector3, Point3};
use csgrs::plane::Plane;
#[cfg(feature = "metaballs")]
use csgrs::csg::MetaBall;

// A type alias for convenience: no shared data, i.e. S = ()
type CSG = csgrs::csg::CSG<()>;

fn main() {
    // Ensure the /stls folder exists
    let _ = fs::create_dir_all("stl");

    // 1) Basic shapes: cube, sphere, cylinder
    let cube = CSG::cube(2.0, 2.0, 2.0, None);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/cube.stl", cube.to_stl_binary("cube").unwrap());

    let sphere = CSG::sphere(1.0, 16, 8, None); // center=(0,0,0), radius=1, slices=16, stacks=8, no metadata
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/sphere.stl", sphere.to_stl_binary("sphere").unwrap());

    let cylinder = CSG::cylinder(1.0, 2.0, 32, None); // start=(0,-1,0), end=(0,1,0), radius=1.0, slices=32
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/cylinder.stl", cylinder.to_stl_binary("cylinder").unwrap());

    // 2) Transformations: Translate, Rotate, Scale, Mirror
    let moved_cube = cube
        .translate(Vector3::new(1.0, 0.0, 0.0))
        .rotate(0.0, 45.0, 0.0)
        .scale(1.0, 0.5, 2.0);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/cube_transformed.stl", moved_cube.to_stl_binary("cube_transformed").unwrap());

    let plane_x = Plane { normal: Vector3::x(), w: 0.0 };
    let mirrored_cube = cube.mirror(plane_x);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/cube_mirrored_x.stl", mirrored_cube.to_stl_binary("cube_mirrored_x").unwrap());

    // 3) Boolean operations: Union, Subtract, Intersect
    let union_shape = moved_cube.union(&sphere);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/union_cube_sphere.stl", union_shape.to_stl_binary("union_cube_sphere").unwrap());

    let subtract_shape = moved_cube.difference(&sphere);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/subtract_cube_sphere.stl", subtract_shape.to_stl_binary("subtract_cube_sphere").unwrap());

    let intersect_shape = moved_cube.intersection(&sphere);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/intersect_cube_sphere.stl", intersect_shape.to_stl_binary("intersect_cube_sphere").unwrap());

    // 4) Convex hull
    #[cfg(feature = "chull-io")]
    let hull_of_union = union_shape.convex_hull();
    #[cfg(feature = "stl-io")]
    #[cfg(feature = "chull-io")]
    let _ = fs::write("stl/hull_union.stl", hull_of_union.to_stl_binary("hull_union").unwrap());

    // 5) Minkowski sum
    #[cfg(feature = "chull-io")]
    let minkowski = cube.minkowski_sum(&sphere);
    #[cfg(feature = "stl-io")]
    #[cfg(feature = "chull-io")]
    let _ = fs::write("stl/minkowski_cube_sphere.stl", minkowski.to_stl_binary("minkowski_cube_sphere").unwrap());

    // 7) 2D shapes and 2D offsetting
    let square_2d = CSG::square(2.0, 2.0, None); // 2x2 square, centered
    let _ = fs::write("stl/square_2d.stl", square_2d.to_stl_ascii("square_2d"));

    let circle_2d = CSG::circle(1.0, 32, None);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/circle_2d.stl", circle_2d.to_stl_binary("circle_2d").unwrap());

    let grown_2d = square_2d.offset_2d(0.5);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/square_2d_grow_0_5.stl", grown_2d.to_stl_binary("square_2d_grow_0_5").unwrap());

    let shrunk_2d = square_2d.offset_2d(-0.5);
    let _ = fs::write("stl/square_2d_shrink_0_5.stl", shrunk_2d.to_stl_ascii("square_2d_shrink_0_5"));

    // 8) Extrude & Rotate-Extrude
    let extruded_square = square_2d.extrude(1.0);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/square_extrude.stl", extruded_square.to_stl_binary("square_extrude").unwrap());

    let revolve_circle = circle_2d.rotate(-90.0,0.0,0.0).translate(Vector3::new(10.0, 0.0, 0.0)).rotate_extrude(360.0, 32);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/circle_revolve_360.stl", revolve_circle.to_stl_binary("circle_revolve_360").unwrap());

    let partial_revolve = circle_2d.rotate(-90.0,0.0,0.0).translate(Vector3::new(10.0, 0.0, 0.0)).rotate_extrude(180.0, 32);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/circle_revolve_180.stl", partial_revolve.to_stl_binary("circle_revolve_180").unwrap());

    // 9) Subdivide triangles (for smoother sphere or shapes):
    let subdiv_sphere = sphere.subdivide_triangles(2); // 2 subdivision levels
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/sphere_subdiv2.stl", subdiv_sphere.to_stl_binary("sphere_subdiv2").unwrap());

    // 10) Renormalize polygons (flat shading):
    let mut union_clone = union_shape.clone();
    union_clone.renormalize();
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/union_renormalized.stl", union_clone.to_stl_binary("union_renormalized").unwrap());

    // 11) Ray intersection demo (just printing the results)
    {
        let ray_origin = Point3::new(0.0, 0.0, -5.0);
        let ray_dir = Vector3::new(0.0, 0.0, 1.0); // pointing along +Z
        let hits = cube.ray_intersections(&ray_origin, &ray_dir);
        println!("Ray hits on the cube: {:?}", hits);
    }

    // 12) Polyhedron example (simple tetrahedron):
    let points = &[
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.5, 1.0, 0.0],
        [0.5, 0.5, 1.0],
    ];
    let faces = vec![
        vec![0, 1, 2], // base triangle
        vec![0, 1, 3], // side
        vec![1, 2, 3],
        vec![2, 0, 3],
    ];
    let poly = CSG::polyhedron(points, &faces, None);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/tetrahedron.stl", poly.to_stl_binary("tetrahedron").unwrap());

    // 13) Text example (2D). Provide a valid TTF font data below:
    // (Replace "asar.ttf" with a real .ttf file in your project.)
    let font_data = include_bytes!("../asar.ttf");
    #[cfg(feature = "truetype-text")]
    let text_csg = CSG::text("HELLO", font_data, 15.0, None);
    #[cfg(feature = "stl-io")]
    #[cfg(feature = "truetype-text")]
    let _ = fs::write("stl/text_hello_2d.stl", text_csg.to_stl_binary("text_hello_2d").unwrap());

    // Optionally extrude the text:
    #[cfg(feature = "truetype-text")]
    let text_extruded = text_csg.extrude(2.0);
    #[cfg(feature = "stl-io")]
    #[cfg(feature = "truetype-text")]
    let _ = fs::write("stl/text_hello_extruded.stl", text_extruded.to_stl_binary("text_hello_extruded").unwrap());

    // 14) Mass properties (just printing them)
    let (mass, com, principal_frame) = cube.mass_properties(1.0);
    println!("Cube mass = {}", mass);
    println!("Cube center of mass = {:?}", com);
    println!("Cube principal inertia local frame = {:?}", principal_frame);
    
    // 1) Create a cube from (-1,-1,-1) to (+1,+1,+1)
    //    (By default, CSG::cube(None) is from -1..+1 if the "radius" is [1,1,1].)
    let cube = CSG::cube(1.0, 1.0, 1.0, None);
    // 2) Flatten into the XY plane
    let flattened = cube.flatten();
    let _ = fs::write("stl/flattened_cube.stl", flattened.to_stl_ascii("flattened_cube"));
    
    // Create a frustrum (start=-2, end=+2) with radius1 = 1, radius2 = 2, 32 slices
    let frustrum = CSG::frustrum_ptp(Point3::new(0.0, 0.0, -2.0), Point3::new(0.0, 0.0, 2.0), 1.0, 2.0, 32, None);
    let _ = fs::write("stl/frustrum.stl", frustrum.to_stl_ascii("frustrum"));
    
    // 1) Create a cylinder (start=-1, end=+1) with radius=1, 32 slices
    let cyl = CSG::frustrum_ptp(Point3::new(0.0, 0.0, -1.0), Point3::new(0.0, 0.0, 1.0), 1.0, 1.0, 32, None);
    // 2) Slice at z=0
    let cross_section = cyl.slice(Plane { normal: Vector3::z(), w: 0.0 });
    let _ = fs::write("stl/sliced_cylinder.stl", cyl.to_stl_ascii("sliced_cylinder"));
    let _ = fs::write("stl/sliced_cylinder_slice.stl", cross_section.to_stl_ascii("sliced_cylinder_slice"));
    
    let poor_geometry_shape = moved_cube.difference(&sphere);
    #[cfg(feature = "earclip-io")]
    let retriangulated_shape = poor_geometry_shape.triangulate_earclip();
    #[cfg(feature = "earclip-io")]
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/retriangulated.stl", retriangulated_shape.to_stl_binary("retriangulated").unwrap());

    let sphere_test = CSG::sphere(1.0, 16, 8, None);
    let cube_test = CSG::cube(1.0, 1.0, 1.0, None);
    let res = cube_test.difference(&sphere_test);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/sphere_cube_test.stl", res.to_stl_binary("sphere_cube_test").unwrap());
    assert_eq!(res.bounding_box(), cube_test.bounding_box());

    // Suppose we want two overlapping metaballs
    #[cfg(feature = "metaballs")]
    let balls = vec![
        MetaBall::new(Point3::new(0.0, 0.0, 0.0), 1.0),
        MetaBall::new(Point3::new(1.5, 0.0, 0.0), 1.0),
    ];

    let resolution = (60, 60, 60);
    let iso_value = 1.0;
    let padding = 1.0;

    #[cfg(feature = "metaballs")]
    let metaball_csg = CSG::from_metaballs(
        &balls,
        resolution,
        iso_value,
        padding,
    );
    
    // For instance, save to STL
    #[cfg(feature = "metaballs")]
    #[cfg(feature = "stl-io")]
    let stl_data = metaball_csg.to_stl_binary("my_metaballs").unwrap();
    #[cfg(feature = "metaballs")]
    std::fs::write("stl/metaballs.stl", stl_data)
        .expect("Failed to write metaballs.stl");
}

