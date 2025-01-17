// main.rs
//
// Minimal example of each function of csgrs

use std::fs;
use nalgebra::{Vector3, Point3};
use csgrs::{CSG, Axis};

fn main() {
    // 1) Basic shapes: cube, sphere, cylinder
    let cube = CSG::cube(None); // center=(0,0,0), radius=(1,1,1) by default
    let _ = fs::write("stl/cube.stl", cube.to_stl("cube"));

    let sphere = CSG::sphere(None); // center=(0,0,0), radius=1, slices=16, stacks=8
    let _ = fs::write("stl/sphere.stl", sphere.to_stl("sphere"));

    let cylinder = CSG::cylinder(None); // start=(0,-1,0), end=(0,1,0), radius=1.0, slices=16
    let _ = fs::write("stl/cylinder.stl", cylinder.to_stl("cylinder"));

    // 2) Transformations: Translate, Rotate, Scale, Mirror
    let moved_cube = cube
        .translate(Vector3::new(2.0, 0.0, 0.0))
        .rotate(0.0, 45.0, 0.0)
        .scale(1.0, 0.5, 2.0);
    let _ = fs::write("stl/cube_transformed.stl", moved_cube.to_stl("cube_transformed"));

    let mirrored_cube = cube.mirror(Axis::X);
    let _ = fs::write("stl/cube_mirrored_x.stl", mirrored_cube.to_stl("cube_mirrored_x"));

    // 3) Boolean operations: Union, Subtract, Intersect
    let union_shape = cube.union(&sphere);
    let _ = fs::write("stl/union_cube_sphere.stl", union_shape.to_stl("union_cube_sphere"));

    let subtract_shape = cube.subtract(&sphere);
    let _ = fs::write("stl/subtract_cube_sphere.stl", subtract_shape.to_stl("subtract_cube_sphere"));

    let intersect_shape = cube.intersect(&sphere);
    let _ = fs::write("stl/intersect_cube_sphere.stl", intersect_shape.to_stl("intersect_cube_sphere"));

    // 4) Convex hull
    let hull_of_union = union_shape.convex_hull();
    let _ = fs::write("stl/hull_union.stl", hull_of_union.to_stl("hull_union"));

    // 5) Minkowski sum
    let minkowski = cube.minkowski_sum(&sphere);
    let _ = fs::write("stl/minkowski_cube_sphere.stl", minkowski.to_stl("minkowski_cube_sphere"));

    // 6) Grow & Shrink (3D offsetting)
    let grown_cube = cube.grow(0.2);   // approximate outward offset
    let _ = fs::write("stl/cube_grow_0_2.stl", grown_cube.to_stl("cube_grow_0_2"));

    let shrunk_cube = cube.shrink(0.2); // approximate inward offset
    let _ = fs::write("stl/cube_shrink_0_2.stl", shrunk_cube.to_stl("cube_shrink_0_2"));

    // 7) 2D shapes and 2D offsetting
    let square_2d = CSG::square(Some(([2.0, 2.0], true))); // 2x2 square, centered
    let _ = fs::write("stl/square_2d.stl", square_2d.to_stl("square_2d"));

    let circle_2d = CSG::circle(Some((1.0, 32)));
    let _ = fs::write("stl/circle_2d.stl", circle_2d.to_stl("circle_2d"));

    let grown_2d = square_2d.grow_2d(0.5);
    let _ = fs::write("stl/square_2d_grow_0_5.stl", grown_2d.to_stl("square_2d_grow_0_5"));

    let shrunk_2d = square_2d.shrink_2d(0.5);
    let _ = fs::write("stl/square_2d_shrink_0_5.stl", shrunk_2d.to_stl("square_2d_shrink_0_5"));

    // 8) Extrude & Rotate-Extrude
    let extruded_square = square_2d.extrude(1.0);
    let _ = fs::write("stl/square_extrude.stl", extruded_square.to_stl("square_extrude"));

    let revolve_circle = circle_2d.rotate_extrude(360.0, 32);
    let _ = fs::write("stl/circle_revolve_360.stl", revolve_circle.to_stl("circle_revolve_360"));

    let partial_revolve = circle_2d.rotate_extrude(180.0, 32);
    let _ = fs::write("stl/circle_revolve_180.stl", partial_revolve.to_stl("circle_revolve_180"));

    // 9) Subdivide triangles (for smoother sphere or shapes):
    let subdiv_sphere = sphere.subdivide_triangles(2); // 2 subdivision levels
    let _ = fs::write("stl/sphere_subdiv2.stl", subdiv_sphere.to_stl("sphere_subdiv2"));

    // 10) Renormalize polygons (flat shading):
    let mut union_clone = union_shape.clone();
    union_clone.renormalize();
    let _ = fs::write("stl/union_renormalized.stl", union_clone.to_stl("union_renormalized"));

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
    let poly = CSG::polyhedron(points, &faces);
    let _ = fs::write("stl/tetrahedron.stl", poly.to_stl("tetrahedron"));

    // 13) Text example (2D). Provide a valid TTF font data below:
    // (Replace "SomeFont.ttf" with a real .ttf file in your project.)
    let font_data = include_bytes!("../asar.ttf");
    let text_csg = CSG::text_mesh("HELLO", font_data, Some(15.0));
    let _ = fs::write("stl/text_hello_2d.stl", text_csg.to_stl("text_hello_2d"));

    // Optionally extrude the text:
    let text_extruded = text_csg.extrude(2.0);
    let _ = fs::write("stl/text_hello_extruded.stl", text_extruded.to_stl("text_hello_extruded"));

    // 14) Mass properties (just printing them)
    let (mass, com, principal_frame) = cube.mass_properties(1.0); 
    println!("Cube mass = {}", mass);
    println!("Cube center of mass = {:?}", com);
    println!("Cube principal inertia local frame = {:?}", principal_frame);
    
}
