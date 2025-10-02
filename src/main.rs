// main.rs
//
// Minimal example of each function of csgrs (which is now generic over the shared-data type S).
// Here, we do not use any shared data, so we'll bind the generic S to ().

#[cfg(feature = "sdf")]
use csgrs::float_types::Real;

use csgrs::mesh::plane::Plane;
use csgrs::traits::CSG;
use nalgebra::{Point3, Vector3};
use std::fs;

#[cfg(feature = "image")]
use image::{GrayImage, ImageBuffer};

#[cfg(feature = "metaballs")]
use csgrs::mesh::metaballs::MetaBall;

type Mesh = csgrs::mesh::Mesh<()>;
type Sketch = csgrs::sketch::Sketch<()>;

fn main() {
    // Ensure the /stls folder exists
    let _ = fs::create_dir_all("stl");

    // 1) Basic shapes: cube, sphere, cylinder
    let cube = Mesh::cube(2.0, None);

    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/cube.stl", cube.to_stl_binary("cube").unwrap());

    #[cfg(feature = "stl-io")]
    {
        let sphere = Mesh::sphere(1.0, 16, 8, None); // center=(0,0,0), radius=1, slices=16, stacks=8, no metadata
        let _ = fs::write("stl/sphere.stl", sphere.to_stl_binary("sphere").unwrap());
    }

    #[cfg(feature = "stl-io")]
    {
        let cylinder = Mesh::cylinder(1.0, 2.0, 32, None); // start=(0,-1,0), end=(0,1,0), radius=1.0, slices=32
        let _ = fs::write(
            "stl/cylinder.stl",
            cylinder.to_stl_binary("cylinder").unwrap(),
        );
    }

    // 2) Transformations: Translate, Rotate, Scale, Mirror
    #[cfg(feature = "stl-io")]
    {
        let moved_cube = cube
            .translate(1.0, 0.0, 0.0)
            .rotate(0.0, 45.0, 0.0)
            .scale(1.0, 0.5, 2.0);
        let _ = fs::write(
            "stl/cube_transformed.stl",
            moved_cube.to_stl_binary("cube_transformed").unwrap(),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let plane_x = Plane::from_normal(Vector3::x(), 0.0);
        let mirrored_cube = cube.mirror(plane_x);
        let _ = fs::write(
            "stl/cube_mirrored_x.stl",
            mirrored_cube.to_stl_binary("cube_mirrored_x").unwrap(),
        );
    }

    // 3) Boolean operations: Union, Subtract, Intersect
    #[cfg(feature = "stl-io")]
    {
        let moved_cube = cube
            .translate(1.0, 0.0, 0.0)
            .rotate(0.0, 45.0, 0.0)
            .scale(1.0, 0.5, 2.0);
        let sphere = Mesh::sphere(1.0, 16, 8, None);
        let union_shape = moved_cube.translate(-1.0, 0.0, 0.0).union(&sphere);
        let _ = fs::write(
            "stl/union_cube_sphere.stl",
            union_shape.to_stl_binary("union_cube_sphere").unwrap(),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let moved_cube = cube
            .translate(1.0, 0.0, 0.0)
            .rotate(0.0, 45.0, 0.0)
            .scale(1.0, 0.5, 2.0);
        let sphere = Mesh::sphere(1.0, 16, 8, None);
        let subtract_shape = moved_cube.difference(&sphere);
        let _ = fs::write(
            "stl/subtract_cube_sphere.stl",
            subtract_shape.to_stl_binary("subtract_cube_sphere").unwrap(),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let sphere = Mesh::sphere(1.0, 16, 8, None);
        let intersect_shape = cube.intersection(&sphere);
        let _ = fs::write(
            "stl/intersect_cube_sphere.stl",
            intersect_shape
                .to_stl_binary("intersect_cube_sphere")
                .unwrap(),
        );
    }

    // 4) Convex hull
    #[cfg(all(feature = "chull-io", feature = "stl-io"))]
    {
        let moved_cube = cube
            .translate(1.0, 0.0, 0.0)
            .rotate(0.0, 45.0, 0.0)
            .scale(1.0, 0.5, 2.0);
        let sphere = Mesh::sphere(1.0, 16, 8, None);
        let union_shape = moved_cube.translate(-1.0, 0.0, 0.0).union(&sphere);
        let hull_of_union = union_shape.convex_hull();
        let _ = fs::write(
            "stl/hull_union.stl",
            hull_of_union.to_stl_binary("hull_union").unwrap(),
        );
    }

    // 5) Minkowski sum
    #[cfg(all(feature = "stl-io", feature = "chull-io"))]
    {
        let sphere = Mesh::sphere(1.0, 16, 8, None);
        let minkowski = cube.minkowski_sum(&sphere);
        let _ = fs::write(
            "stl/minkowski_cube_sphere.stl",
            minkowski.to_stl_binary("minkowski_cube_sphere").unwrap(),
        );
    }

    // 7) 2D shapes and 2D offsetting
    #[cfg(feature = "stl-io")]
    {
        let square_2d = Sketch::square(2.0, None); // 2x2 square, centered
        let _ = fs::write("stl/square_2d.stl", square_2d.to_stl_ascii("square_2d"));
    }

    let circle_2d = Sketch::circle(1.0, 32, None);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        "stl/circle_2d.stl",
        circle_2d.to_stl_binary("circle_2d").unwrap(),
    );

    #[cfg(all(feature = "offset", feature = "stl-io"))]
    {
        let square_2d = Sketch::square(2.0, None);
        let grown_2d = square_2d.offset(0.5);
        let _ = fs::write(
            "stl/square_2d_grow_0_5.stl",
            grown_2d.to_stl_ascii("square_2d_grow_0_5"),
        );

        let shrunk_2d = square_2d.offset(-0.5);
        let _ = fs::write(
            "stl/square_2d_shrink_0_5.stl",
            shrunk_2d.to_stl_ascii("square_2d_shrink_0_5"),
        );
    }

    // star(num_points, outer_radius, inner_radius)
    #[cfg(feature = "stl-io")]
    {
        let star_2d = Sketch::star(5, 2.0, 0.8, None);
        let _ = fs::write("stl/star_2d.stl", star_2d.to_stl_ascii("star_2d"));
    }

    // Extrude & Rotate-Extrude
    #[cfg(feature = "stl-io")]
    {
        let star_2d = Sketch::star(5, 2.0, 0.8, None);
        let extruded_star = star_2d.extrude(1.0);
        let _ = fs::write(
            "stl/star_extrude.stl",
            extruded_star.to_stl_binary("star_extrude").unwrap(),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let star_2d = Sketch::star(5, 2.0, 0.8, None);
        let vector_extruded_star = star_2d.extrude_vector(Vector3::new(2.0, 1.0, 1.0));
        let _ = fs::write(
            "stl/star_vec_extrude.stl",
            vector_extruded_star.to_stl_binary("star_extrude").unwrap(),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let revolve_circle = circle_2d
            .translate(10.0, 0.0, 0.0)
            .revolve(360.0, 32)
            .expect("Revolve failed");
        #[cfg(feature = "stl-io")]
        let _ = fs::write(
            "stl/circle_revolve_360.stl",
            revolve_circle.to_stl_binary("circle_revolve_360").unwrap(),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let partial_revolve = circle_2d
            .translate(10.0, 0.0, 0.0)
            .revolve(180.0, 32)
            .expect("Revolve failed");
        let _ = fs::write(
            "stl/circle_revolve_180.stl",
            partial_revolve.to_stl_binary("circle_revolve_180").unwrap(),
        );
    }

    // 9) Subdivide triangles (for smoother sphere or shapes):
    #[cfg(feature = "stl-io")]
    {
        let sphere = Mesh::sphere(1.0, 16, 8, None);
        let subdiv_sphere = sphere.subdivide_triangles(2.try_into().expect("not 0")); // 2 subdivision levels
        let _ = fs::write(
            "stl/sphere_subdiv2.stl",
            subdiv_sphere.to_stl_binary("sphere_subdiv2").unwrap(),
        );
    }

    // 10) Renormalize polygons (flat shading):
    #[cfg(feature = "stl-io")]
    {
        let moved_cube = cube
            .translate(1.0, 0.0, 0.0)
            .rotate(0.0, 45.0, 0.0)
            .scale(1.0, 0.5, 2.0);
        let sphere = Mesh::sphere(1.0, 16, 8, None);
        let union_shape = moved_cube.translate(-1.0, 0.0, 0.0).union(&sphere);
        let mut union_clone = union_shape.clone();
        union_clone.renormalize();
        let _ = fs::write(
            "stl/union_renormalized.stl",
            union_clone.to_stl_binary("union_renormalized").unwrap(),
        );
    }

    // 11) Ray intersection demo (just printing the results)
    {
        let ray_origin = Point3::new(0.0, 0.0, -5.0);
        let ray_dir = Vector3::new(0.0, 0.0, 1.0); // pointing along +Z
        let hits = cube.ray_intersections(&ray_origin, &ray_dir);
        println!("Ray hits on the cube: {:?}", hits);
    }

    // 12) Polyhedron example (simple tetrahedron):
    let points = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.5, 1.0, 0.0],
        [0.5, 0.5, 1.0],
    ];
    let faces: [&[usize]; 4] = [
        &[0, 2, 1], // base triangle
        &[0, 1, 3], // side
        &[1, 2, 3],
        &[2, 0, 3],
    ];
    let poly = Mesh::polyhedron(&points, &faces, None).unwrap();
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/tetrahedron.stl", poly.to_stl_ascii("tetrahedron"));

    // 13) Text example (2D). Provide a valid TTF font data below:
    // (Replace "asar.ttf" with a real .ttf file in your project.)
    #[cfg(feature = "truetype-text")]
    let font_data = include_bytes!("../asar.ttf");
    #[cfg(feature = "truetype-text")]
    let text_csg = Sketch::text("HELLO", font_data, 15.0, None);
    #[cfg(feature = "stl-io")]
    #[cfg(feature = "truetype-text")]
    let _ = fs::write(
        "stl/text_hello_2d.stl",
        text_csg.to_stl_binary("text_hello_2d").unwrap(),
    );

    // Optionally extrude the text:
    #[cfg(feature = "truetype-text")]
    let text_extruded = text_csg.extrude(2.0);
    #[cfg(feature = "stl-io")]
    #[cfg(feature = "truetype-text")]
    let _ = fs::write(
        "stl/text_hello_extruded.stl",
        text_extruded.to_stl_binary("text_hello_extruded").unwrap(),
    );

    // 14) Mass properties (just printing them)
    let (mass, com, principal_frame) = cube.mass_properties(1.0);
    println!("Cube mass = {}", mass);
    println!("Cube center of mass = {:?}", com);
    println!("Cube principal inertia local frame = {:?}", principal_frame);

    // 1) Create a cube from (-1,-1,-1) to (+1,+1,+1)
    //    (By default, CSG::cube(None) is from -1..+1 if the "radius" is [1,1,1].)
    let cube = Mesh::cube(100.0, None);

    // 2) Flatten into the XY plane
    #[cfg(feature = "stl-io")]
    {
        let flattened = cube.flatten();
        let _ = fs::write(
            "stl/flattened_cube.stl",
            flattened.to_stl_ascii("flattened_cube"),
        );
    }

    // Create a frustum (start=-2, end=+2) with radius1 = 1, radius2 = 2, 32 slices
    #[cfg(feature = "stl-io")]
    {
        let frustum = Mesh::frustum_ptp(
            Point3::new(0.0, 0.0, -2.0),
            Point3::new(0.0, 0.0, 2.0),
            1.0,
            2.0,
            32,
            None,
        );
        let _ = fs::write("stl/frustum.stl", frustum.to_stl_ascii("frustum"));
    }

    // 1) Create a cylinder (start=-1, end=+1) with radius=1, 32 slices
    #[cfg(feature = "stl-io")]
    {
        let cyl = Mesh::frustum_ptp(
            Point3::new(0.0, 0.0, -1.0),
            Point3::new(0.0, 0.0, 1.0),
            1.0,
            1.0,
            32,
            None,
        );
        // 2) Slice at z=0
        let cross_section = cyl.slice(Plane::from_normal(Vector3::z(), 0.0));
        let _ = fs::write("stl/sliced_cylinder.stl", cyl.to_stl_ascii("sliced_cylinder"));
        let _ = fs::write(
            "stl/sliced_cylinder_slice.stl",
            cross_section.to_stl_ascii("sliced_cylinder_slice"),
        );
    }

    //let poor_geometry_shape = moved_cube.difference(&sphere);
    //#[cfg(feature = "earclip-io")]
    //let retriangulated_shape = poor_geometry_shape.triangulate_earclip();
    //#[cfg(all(feature = "earclip-io", feature = "stl-io"))]
    //let _ = fs::write("stl/retriangulated.stl", retriangulated_shape.to_stl_binary("retriangulated").unwrap());

    let sphere_test = Mesh::sphere(1.0, 16, 8, None);
    let cube_test = Mesh::cube(1.0, None);
    let res = cube_test.difference(&sphere_test);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        "stl/sphere_cube_test.stl",
        res.to_stl_binary("sphere_cube_test").unwrap(),
    );
    assert_eq!(res.bounding_box(), cube_test.bounding_box());

    #[cfg(all(feature = "stl-io", feature = "metaballs"))]
    {
        // Suppose we want two overlapping metaballs
        let balls = vec![
            MetaBall::new(Point3::origin(), 1.0),
            MetaBall::new(Point3::new(1.75, 0.0, 0.0), 1.0),
        ];

        let resolution = (60, 60, 60);
        let iso_value = 1.0;
        let padding = 1.0;

        #[cfg(feature = "metaballs")]
        let metaball_csg = Mesh::metaballs(&balls, resolution, iso_value, padding, None);

        // For instance, save to STL
        let stl_data = metaball_csg.to_stl_binary("my_metaballs").unwrap();
        std::fs::write("stl/metaballs.stl", stl_data).expect("Failed to write metaballs.stl");
    }

    #[cfg(feature = "sdf")]
    {
        // Example SDF for a sphere of radius 1.5 centered at (0,0,0)
        let my_sdf = |p: &Point3<Real>| p.coords.norm() - 1.5;

        let resolution = (60, 60, 60);
        let min_pt = Point3::new(-2.0, -2.0, -2.0);
        let max_pt = Point3::new(2.0, 2.0, 2.0);
        let iso_value = 0.0; // Typically zero for SDF-based surfaces

        let csg_shape = Mesh::sdf(my_sdf, resolution, min_pt, max_pt, iso_value, None);

        // Now `csg_shape` is your polygon mesh as a CSG you can union, subtract, or export:
        #[cfg(feature = "stl-io")]
        let _ = std::fs::write(
            "stl/sdf_sphere.stl",
            csg_shape.to_stl_binary("sdf_sphere").unwrap(),
        );
    }

    // Create a pie slice of radius 2, from 0 to 90 degrees
    #[cfg(feature = "stl-io")]
    {
        let wedge = Sketch::pie_slice(2.0, 0.0, 90.0, 16, None);
        let _ = fs::write("stl/pie_slice.stl", wedge.to_stl_ascii("pie_slice"));
    }

    // Create a 2D "metaball" shape from 3 circles
    #[cfg(all(feature = "metaballs", feature = "stl-io"))]
    {
        use nalgebra::Point2;
        let balls_2d = vec![
            (Point2::new(0.0, 0.0), 1.0),
            (Point2::new(1.5, 0.0), 1.0),
            (Point2::new(0.75, 1.0), 0.5),
        ];
        let mb2d = Sketch::metaballs(&balls_2d, (100, 100), 1.0, 0.25, None);
        let _ = fs::write("stl/mb2d.stl", mb2d.to_stl_ascii("metaballs2d"));
    }

    // Create a supershape
    #[cfg(feature = "stl-io")]
    {
        let sshape = Sketch::supershape(1.0, 1.0, 6.0, 1.0, 1.0, 1.0, 128, None);
        let _ = fs::write("stl/supershape.stl", sshape.to_stl_ascii("supershape"));
    }

    // Distribute a square along an arc
    #[cfg(feature = "stl-io")]
    {
        let square = Sketch::circle(1.0, 32, None);
        let arc_array = square.distribute_arc(5, 5.0, 0.0, 180.0);
        let _ = fs::write("stl/arc_array.stl", arc_array.to_stl_ascii("arc_array"));
    }

    // Distribute that wedge along a linear axis
    #[cfg(feature = "stl-io")]
    {
        let wedge = Sketch::pie_slice(2.0, 0.0, 90.0, 16, None);
        let wedge_line =
            wedge.distribute_linear(4, nalgebra::Vector3::new(1.0, 0.0, 0.0), 3.0);
        let _ = fs::write("stl/wedge_line.stl", wedge_line.to_stl_ascii("wedge_line"));
    }

    // Make a 4x4 grid of the supershape
    #[cfg(feature = "stl-io")]
    {
        let sshape = Sketch::supershape(1.0, 1.0, 6.0, 1.0, 1.0, 1.0, 128, None);
        let grid_of_ss = sshape.distribute_grid(4, 4, 3.0, 3.0);
        let _ = fs::write("stl/grid_of_ss.stl", grid_of_ss.to_stl_ascii("grid_of_ss"));
    }

    // 1. Circle with keyway
    #[cfg(feature = "stl-io")]
    {
        let keyway_shape = Sketch::circle_with_keyway(10.0, 64, 2.0, 3.0, None);
        let _ = fs::write(
            "stl/keyway_shape.stl",
            keyway_shape.to_stl_ascii("keyway_shape"),
        );
        // Extrude it 2 units:
        let keyway_3d = keyway_shape.extrude(2.0);
        let _ = fs::write("stl/keyway_3d.stl", keyway_3d.to_stl_ascii("keyway_3d"));
    }

    // 2. D-shape
    #[cfg(feature = "stl-io")]
    {
        let d_shape = Sketch::circle_with_flat(5.0, 32, 2.0, None);
        let _ = fs::write("stl/d_shape.stl", d_shape.to_stl_ascii("d_shape"));
        let d_3d = d_shape.extrude(1.0);
        let _ = fs::write("stl/d_3d.stl", d_3d.to_stl_ascii("d_3d"));
    }

    // 3. Double-flat circle
    #[cfg(feature = "stl-io")]
    {
        let double_flat = Sketch::circle_with_two_flats(8.0, 64, 3.0, None);
        let _ = fs::write("stl/double_flat.stl", double_flat.to_stl_ascii("double_flat"));
        let df_3d = double_flat.extrude(0.5);
        let _ = fs::write("stl/df_3d.stl", df_3d.to_stl_ascii("df_3d"));
    }

    // A 3D teardrop shape
    #[cfg(all(feature = "chull", feature = "stl-io"))]
    {
        let teardrop_solid = Mesh::teardrop(3.0, 5.0, 32, 32, None);
        let _ = fs::write(
            "stl/teardrop_solid.stl",
            teardrop_solid.to_stl_ascii("teardrop_solid"),
        );

        // A 3D egg shape
        let egg_solid = Mesh::egg(2.0, 4.0, 8, 16, None);
        let _ = fs::write("stl/egg_solid.stl", egg_solid.to_stl_ascii("egg_solid"));
    }

    // An ellipsoid with X radius=2, Y radius=1, Z radius=3
    #[cfg(feature = "stl-io")]
    {
        let ellipsoid = Mesh::ellipsoid(2.0, 1.0, 3.0, 16, 8, None);
        let _ = fs::write("stl/ellipsoid.stl", ellipsoid.to_stl_ascii("ellipsoid"));
    }

    // A teardrop 'blank' hole
    #[cfg(feature = "stl-io")]
    {
        let teardrop_cylinder = Mesh::teardrop_cylinder(2.0, 4.0, 32.0, 16, None);
        let _ = fs::write(
            "stl/teardrop_cylinder.stl",
            teardrop_cylinder.to_stl_ascii("teardrop_cylinder"),
        );
    }

    // 1) polygon()
    #[cfg(feature = "stl-io")]
    {
        let polygon_2d =
            Sketch::polygon(&[[0.0, 0.0], [2.0, 0.0], [1.5, 1.0], [1.0, 2.0]], None);
        let _ = fs::write("stl/polygon_2d.stl", polygon_2d.to_stl_ascii("polygon_2d"));
    }

    // 2) rounded_rectangle(width, height, corner_radius, corner_segments)
    #[cfg(feature = "stl-io")]
    {
        let rrect_2d = Sketch::rounded_rectangle(4.0, 2.0, 0.3, 8, None);
        let _ = fs::write(
            "stl/rounded_rectangle_2d.stl",
            rrect_2d.to_stl_ascii("rounded_rectangle_2d"),
        );
    }

    // 3) ellipse(width, height, segments)
    #[cfg(feature = "stl-io")]
    {
        let ellipse = Sketch::ellipse(3.0, 1.5, 32, None);
        let _ = fs::write("stl/ellipse.stl", ellipse.to_stl_ascii("ellipse"));
    }

    // 4) regular_ngon(sides, radius)
    #[cfg(feature = "stl-io")]
    {
        let ngon_2d = Sketch::regular_ngon(6, 1.0, None); // Hexagon
        let _ = fs::write("stl/ngon_2d.stl", ngon_2d.to_stl_ascii("ngon_2d"));
    }

    // 6) trapezoid(top_width, bottom_width, height)
    #[cfg(feature = "stl-io")]
    {
        let trap_2d = Sketch::trapezoid(1.0, 2.0, 2.0, 0.5, None);
        let _ = fs::write("stl/trapezoid_2d.stl", trap_2d.to_stl_ascii("trapezoid_2d"));
    }

    // 8) teardrop(width, height, segments) [2D shape]
    #[cfg(feature = "stl-io")]
    {
        let teardrop_2d = Sketch::teardrop(2.0, 3.0, 16, None);
        let _ = fs::write("stl/teardrop_2d.stl", teardrop_2d.to_stl_ascii("teardrop_2d"));
    }

    // 9) egg_outline(width, length, segments) [2D shape]
    #[cfg(feature = "stl-io")]
    {
        let egg_2d = Sketch::egg(2.0, 4.0, 32, None);
        let _ = fs::write(
            "stl/egg_outline_2d.stl",
            egg_2d.to_stl_ascii("egg_outline_2d"),
        );
    }

    // 10) squircle(width, height, segments)
    #[cfg(feature = "stl-io")]
    {
        let squircle_2d = Sketch::squircle(3.0, 3.0, 32, None);
        let _ = fs::write("stl/squircle_2d.stl", squircle_2d.to_stl_ascii("squircle_2d"));
    }

    // 11) keyhole(circle_radius, handle_width, handle_height, segments)
    #[cfg(feature = "stl-io")]
    {
        let keyhole_2d = Sketch::keyhole(1.0, 1.0, 2.0, 16, None);
        let _ = fs::write("stl/keyhole_2d.stl", keyhole_2d.to_stl_ascii("keyhole_2d"));
    }

    // 12) reuleaux_polygon(sides, side_len, segments)
    #[cfg(feature = "stl-io")]
    {
        let reuleaux3_2d = Sketch::reuleaux(3, 2.0, 64, None); // Reuleaux triangle
        let _ = fs::write(
            "stl/reuleaux3_2d.stl",
            reuleaux3_2d.to_stl_ascii("reuleaux_2d"),
        );
    }

    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    #[cfg(feature = "stl-io")]
    {
        let reuleaux4_2d = Sketch::reuleaux(4, 2.0, 64, None); // Reuleaux triangle
        let _ = fs::write(
            "stl/reuleaux4_2d.stl",
            reuleaux4_2d.to_stl_ascii("reuleaux_2d"),
        );
    }

    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    #[cfg(feature = "stl-io")]
    {
        let reuleaux5_2d = Sketch::reuleaux(5, 2.0, 64, None); // Reuleaux triangle
        let _ = fs::write(
            "stl/reuleaux5_2d.stl",
            reuleaux5_2d.to_stl_ascii("reuleaux_2d"),
        );
    }

    // 13) ring(inner_diam, thickness, segments)
    #[cfg(feature = "stl-io")]
    {
        let ring_2d = Sketch::ring(5.0, 1.0, 32, None);
        let _ = fs::write("stl/ring_2d.stl", ring_2d.to_stl_ascii("ring_2d"));
    }

    // 15) from_image(img, threshold, closepaths, metadata) [requires "image" feature]
    #[cfg(feature = "image")]
    {
        // Make a simple 64x64 gray image with a circle in the center
        let mut img: GrayImage = ImageBuffer::new(64, 64);
        // Fill a small circle of "white" pixels in the middle
        let center = (32, 32);
        for y in 0..64 {
            for x in 0..64 {
                let dx = x as i32 - center.0;
                let dy = y as i32 - center.1;
                if dx * dx + dy * dy < 15 * 15 {
                    img.put_pixel(x, y, image::Luma([255u8]));
                }
            }
        }
        let csg_img = Sketch::from_image(&img, 128, true, None).center();
        let _ = fs::write("stl/from_image.stl", csg_img.to_stl_ascii("from_image"));
    }

    // 16) gyroid(...) – uses the current CSG volume as a bounding region
    // Let's reuse the `cube` from above:
    #[cfg(all(feature = "stl-io", feature = "sdf"))]
    {
        let tpms_volume = Mesh::sphere(20.0, 10, 10, None);
        let gyroid_inside_sphere = tpms_volume.gyroid(64, 2.0, 0.0, None);
        let _ = fs::write(
            "stl/gyroid_sphere.stl",
            gyroid_inside_sphere.to_stl_binary("gyroid_cube").unwrap(),
        );

        let schwarzp_inside_sphere = tpms_volume.schwarz_p(64, 2.0, 0.0, None);
        let _ = fs::write(
            "stl/schwarz_p_sphere.stl",
            schwarzp_inside_sphere
                .to_stl_binary("schwarz_p_cube")
                .unwrap(),
        );

        let schwarzd_inside_sphere = tpms_volume.schwarz_d(64, 2.0, 0.0, None);
        let _ = fs::write(
            "stl/schwarz_d_sphere.stl",
            schwarzd_inside_sphere
                .to_stl_binary("schwarz_d_cube")
                .unwrap(),
        );
    }

    // Define the start point and the arrow direction vector.
    // The arrow’s length is the norm of the direction vector.
    let start = Point3::new(1.0, 1.0, 1.0);
    let direction = Vector3::new(10.0, 5.0, 20.0);

    // Define the resolution (number of segments for the cylindrical shaft and head).
    let segments = 16;

    // Create the arrow. We pass `None` for metadata.
    #[cfg(feature = "stl-io")]
    {
        let arrow_csg = Mesh::arrow(start, direction, segments, true, None::<()>);
        let _ = fs::write("stl/arrow.stl", arrow_csg.to_stl_ascii("arrow_example"));
    }

    #[cfg(feature = "stl-io")]
    {
        let arrow_reversed_csg = Mesh::arrow(start, direction, segments, false, None::<()>);
        let _ = fs::write(
            "stl/arrow_reversed.stl",
            arrow_reversed_csg.to_stl_ascii("arrow_example"),
        );
    }

    // 2-D profile for NACA 2412, 1 m chord, 100 pts / surface
    #[cfg(feature = "stl-io")]
    {
        let naca2412 = Sketch::airfoil_naca4(2.0, 4.0, 12.0, 1.0, 100, None);
        let _ = fs::write("stl/naca2412.stl", naca2412.to_stl_ascii("2412"));
    }

    // quick solid wing rib 5 mm thick
    #[cfg(feature = "stl-io")]
    {
        let naca2412 = Sketch::airfoil_naca4(2.0, 4.0, 12.0, 1.0, 100, None);
        let rib = naca2412.extrude(0.005);
        let _ = fs::write("stl/naca2412_extruded.stl", rib.to_stl_ascii("2412_extruded"));
    }

    // symmetric foil for a centerboard
    #[cfg(feature = "stl-io")]
    {
        let naca0015 = Sketch::airfoil_naca4(0.0, 0.0, 15.0, 0.3, 80, None)
            .extrude_vector(nalgebra::Vector3::new(0.0, 0.0, 1.2));
        let _ = fs::write("stl/naca0015.stl", naca0015.to_stl_ascii("naca0015"));

        let oct = Mesh::octahedron(10.0, None);
        let _ = fs::write("stl/octahedron.stl", oct.to_stl_ascii("octahedron"));
    }

    //let dodec = CSG::dodecahedron(15.0, None);
    //let _ = fs::write("stl/dodecahedron.stl", dodec.to_stl_ascii(""));

    #[cfg(feature = "stl-io")]
    {
        let ico = Mesh::icosahedron(12.0, None);
        let _ = fs::write("stl/icosahedron.stl", ico.to_stl_ascii(""));
    }

    #[cfg(feature = "stl-io")]
    {
        let torus = Mesh::torus(20.0, 5.0, 48, 24, None);
        let _ = fs::write("stl/torus.stl", torus.to_stl_ascii(""));
    }

    #[cfg(feature = "stl-io")]
    {
        let heart2d = Sketch::heart(30.0, 25.0, 128, None);
        let _ = fs::write("stl/heart2d.stl", heart2d.to_stl_ascii(""));
    }

    #[cfg(feature = "stl-io")]
    {
        let crescent2d = Sketch::crescent(10.0, 7.0, 4.0, 64, None);
        let _ = fs::write("stl/crescent2d.stl", crescent2d.to_stl_ascii(""));
    }

    // ---------------------------------------------------------
    // Additional “SCENES” Demonstrating Each Function Minimally
    //
    // In these scenes, we typically:
    //   1) Create the shape
    //   2) Extrude (if 2D) so we can save an STL
    //   3) Optionally union with a small arrow that points to
    //      a location of interest in the shape
    //   4) Save the result as an STL, e.g. "scene_XX_something.stl"
    //
    // Because many shapes are already shown above, these are
    // just short examples to help with explanation.
    // ---------------------------------------------------------

    // Scene A: Demonstrate a right_triangle(width=2, height=1)
    #[cfg(feature = "stl-io")]
    {
        let tri_2d = Sketch::right_triangle(2.0, 1.0, None);
        let arrow = Mesh::arrow(
            Point3::new(0.0, 0.0, 0.1), // at corner
            Vector3::new(0.5, 0.0, 0.0),
            8,
            true,
            None::<()>,
        );
        let scene = tri_2d.extrude(0.1).union(&arrow);
        let _ = fs::write(
            "stl/scene_right_triangle.stl",
            scene.to_stl_ascii("scene_right_triangle"),
        );
    }

    // Scene B: Demonstrate extrude_vector(direction)
    #[cfg(feature = "stl-io")]
    {
        let circle2d = Sketch::circle(1.0, 32, None);
        // extrude along an arbitrary vector
        let extruded_along_vec = circle2d.extrude_vector(Vector3::new(0.0, 0.0, 2.0));
        let _ = fs::write(
            "stl/scene_extrude_vector.stl",
            extruded_along_vec.to_stl_ascii("scene_extrude_vector"),
        );
    }

    // Scene E: Demonstrate center() (moves shape so bounding box is centered on the origin)
    #[cfg(feature = "stl-io")]
    {
        let off_center_circle = Sketch::circle(1.0, 32, None)
            .translate(5.0, 2.0, 0.0)
            .extrude(0.1);
        let centered_circle = off_center_circle.center();
        let _ = fs::write(
            "stl/scene_circle_off_center.stl",
            off_center_circle.to_stl_ascii("scene_circle_off_center"),
        );
        let _ = fs::write(
            "stl/scene_circle_centered.stl",
            centered_circle.to_stl_ascii("scene_circle_centered"),
        );
    }

    // Scene F: Demonstrate float() (moves shape so bottom is at z=0)
    #[cfg(feature = "stl-io")]
    {
        let sphere_for_float = Mesh::sphere(1.0, 16, 8, None).translate(0.0, 0.0, -1.5);
        let floated = sphere_for_float.float();
        let _ = fs::write(
            "stl/scene_sphere_before_float.stl",
            sphere_for_float.to_stl_ascii("scene_sphere_before_float"),
        );
        let _ = fs::write(
            "stl/scene_sphere_floated.stl",
            floated.to_stl_ascii("scene_sphere_floated"),
        );
    }

    // Scene G: Demonstrate inverse() (flips inside/outside)
    #[cfg(feature = "stl-io")]
    {
        let sphere = Mesh::sphere(1.0, 16, 8, None);
        let inv_sphere = sphere.inverse();
        #[cfg(feature = "stl-io")]
        let _ = fs::write(
            "stl/scene_inverse_sphere.stl",
            inv_sphere.to_stl_binary("scene_inverse_sphere").unwrap(),
        );
    }

    // Scene H: Demonstrate tessellate() (forces triangulation)
    #[cfg(feature = "stl-io")]
    {
        let sphere = Mesh::sphere(1.0, 16, 8, None);
        let tri_sphere = sphere.triangulate();
        #[cfg(feature = "stl-io")]
        let _ = fs::write(
            "stl/scene_tessellate_sphere.stl",
            tri_sphere.to_stl_binary("scene_tessellate_sphere").unwrap(),
        );
    }

    // Scene I: Demonstrate slice(plane) – slice a cube at z=0
    #[cfg(feature = "stl-io")]
    {
        let plane_z = Plane::from_normal(Vector3::z(), 0.5);
        let sliced_polygons = cube.slice(plane_z);
        let _ = fs::write("stl/scene_sliced_cube.stl", cube.to_stl_ascii("sliced_cube"));
        // Save cross-section as well
        let _ = fs::write(
            "stl/scene_sliced_cube_section.stl",
            sliced_polygons.to_stl_ascii("sliced_cube_section"),
        );
    }

    // Scene J: Demonstrate re-computing vertices() or printing them
    #[cfg(feature = "stl-io")]
    {
        let circle_extruded = Sketch::circle(1.0, 32, None).extrude(0.5);
        let verts = circle_extruded.vertices();
        println!("Scene J circle_extruded has {} vertices", verts.len());
        // We'll still save an STL so there's a visual
        let _ = fs::write(
            "stl/scene_j_circle_extruded.stl",
            circle_extruded.to_stl_ascii("scene_j_circle_extruded"),
        );
    }

    // Scene K: Demonstrate reuleaux_polygon with a typical triangle shape
    // (already used sides=4 above, so let's do sides=3 here)
    #[cfg(feature = "stl-io")]
    {
        let reuleaux_tri = Sketch::reuleaux(3, 2.0, 16, None).extrude(0.1);
        let _ = fs::write(
            "stl/scene_reuleaux_triangle.stl",
            reuleaux_tri.to_stl_ascii("scene_reuleaux_triangle"),
        );
    }

    // Scene L: Demonstrate revolve (360 deg) on a square
    #[cfg(feature = "stl-io")]
    {
        let small_square = Sketch::square(1.0, None).translate(2.0, 0.0, 0.0);
        let revolve = small_square.revolve(360.0, 24).expect("Revolve failed");
        let _ = fs::write(
            "stl/scene_square_revolve_360.stl",
            revolve.to_stl_ascii("scene_square_revolve_360"),
        );
    }

    // Scene M: Demonstrate “mirror” across a Y=0 plane
    #[cfg(feature = "stl-io")]
    {
        let plane_y = Plane::from_normal(Vector3::y(), 0.0);
        let shape = Sketch::rectangle(2.0, 1.0, None)
            .translate(1.0, 1.0, 0.0)
            .extrude(0.1);
        let mirrored = shape.mirror(plane_y);
        let _ = fs::write(
            "stl/scene_square_mirrored_y.stl",
            mirrored.to_stl_ascii("scene_square_mirrored_y"),
        );
    }

    // Scene N: Demonstrate scale()
    #[cfg(feature = "stl-io")]
    {
        let sphere = Mesh::sphere(1.0, 16, 8, None);
        let scaled = sphere.scale(1.0, 2.0, 0.5);
        let _ = fs::write(
            "stl/scene_scaled_sphere.stl",
            scaled.to_stl_binary("scene_scaled_sphere").unwrap(),
        );
    }

    // Scene O: Demonstrate transform() with an arbitrary affine matrix
    #[cfg(feature = "stl-io")]
    {
        use nalgebra::{Matrix4, Translation3};
        let xlate = Translation3::new(2.0, 0.0, 1.0).to_homogeneous();
        // Scale matrix
        let scale_mat = Matrix4::new_scaling(0.5);
        // Combine
        let transform_mat = xlate * scale_mat;
        let shape = Mesh::cube(1.0, None).transform(&transform_mat);
        let _ = fs::write(
            "stl/scene_transform_cube.stl",
            shape.to_stl_ascii("scene_transform_cube"),
        );
    }

    // Scene P: Demonstrate offset(distance)
    #[cfg(all(feature = "offset", feature = "stl-io"))]
    {
        let poly_2d = Sketch::polygon(&[[0.0, 0.0], [2.0, 0.0], [1.0, 1.5]], None);
        let grown = poly_2d.offset(0.2);
        let scene = grown.extrude(0.1);
        let _ = fs::write(
            "stl/scene_offset_grown.stl",
            scene.to_stl_ascii("scene_offset_grown"),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let gear_involute_2d = Sketch::involute_gear(
            2.0,  // module [mm]
            20,   // z – number of teeth
            20.0, // α – pressure angle [deg]
            0.05, // radial clearance
            0.02, // backlash at pitch line
            14,   // segments per involute flank
            None,
        );
        let _ = fs::write(
            "stl/gear_involute_2d.stl",
            gear_involute_2d.to_stl_ascii("gear_involute_2d"),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let gear_cycloid_2d = Sketch::cycloidal_gear(
            2.0,  // module
            17,   // gear teeth
            18,   // mating pin-wheel teeth (zₚ = z±1)
            0.05, // clearance
            20,   // segments per flank
            None,
        );
        let _ = fs::write(
            "stl/gear_cycloid_2d.stl",
            gear_cycloid_2d.to_stl_ascii("gear_cycloid_2d"),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let rack_involute = Sketch::involute_rack(
            2.0,  // module
            12,   // number of rack teeth to generate
            20.0, // pressure angle
            0.05, // clearance
            0.02, // backlash
            None,
        );
        let _ = fs::write(
            "stl/rack_involute.stl",
            rack_involute.to_stl_ascii("rack_involute"),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let rack_cycloid = Sketch::cycloidal_rack(
            2.0,  // module
            12,   // teeth
            1.0,  // generating-circle radius  (≈ m/2 for a conventional pin-rack)
            0.05, // clearance
            24,   // segments per flank
            None,
        );
        let _ = fs::write(
            "stl/rack_cycloid.stl",
            rack_cycloid.to_stl_ascii("rack_cycloid"),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let spur_involute = Mesh::spur_gear_involute(
            2.0, 20, 20.0, 0.05, 0.02, 14, 12.0, // face-width (extrusion thickness)
            None,
        );
        let _ = fs::write(
            "stl/spur_involute.stl",
            spur_involute.to_stl_ascii("spur_involute"),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let spur_cycloid = Mesh::spur_gear_cycloid(
            2.0, 17, 18, 0.05, 20, 12.0, // thickness
            None,
        );
        let _ = fs::write(
            "stl/spur_cycloid.stl",
            spur_cycloid.to_stl_ascii("spur_cycloid"),
        );
    }

    /*
    let helical = CSG::helical_involute_gear(
        2.0,   // module
        20,    // z
        20.0,  // pressure angle
        0.05, 0.02, 14,
        25.0,   // face-width
        15.0,   // helix angle β [deg]
        40,     // axial slices (resolution of the twist)
        None,
    );
    let _ = fs::write("stl/helical.stl", helical.to_stl_ascii("helical"));
    */

    // Bézier curve demo
    #[cfg(feature = "stl-io")]
    {
        let bezier_ctrl = &[
            [0.0, 0.0], // P0
            [1.0, 2.0], // P1
            [3.0, 3.0], // P2
            [4.0, 0.0], // P3
        ];
        let bezier_2d = Sketch::bezier(bezier_ctrl, 128, None);
        let _ = fs::write("stl/bezier_2d.stl", bezier_2d.to_stl_ascii("bezier_2d"));

        // give it a little “body” so we can see it in a solid viewer
        let bezier_3d = bezier_2d.extrude(0.25);
        let _ = fs::write(
            "stl/bezier_extruded.stl",
            bezier_3d.to_stl_ascii("bezier_extruded"),
        );
    }

    // B-spline demo
    #[cfg(feature = "stl-io")]
    {
        let bspline_ctrl = &[[0.0, 0.0], [1.0, 2.5], [3.0, 3.0], [5.0, 0.0], [6.0, -1.5]];
        let bspline_2d = Sketch::bspline(
            bspline_ctrl,
            /* degree p = */ 3,
            /* seg/span */ 32,
            None,
        );
        let _ = fs::write("stl/bspline_2d.stl", bspline_2d.to_stl_ascii("bspline_2d"));
    }

    #[cfg(feature = "bevymesh")]
    println!("{:#?}", bezier_3d.to_bevy_mesh());

    // a quick thickening just like the Bézier
    //let bspline_3d = bspline_2d.extrude(0.25);
    //let _ = fs::write(
    //    "stl/bspline_extruded.stl",
    //    bspline_3d.to_stl_ascii("bspline_extruded"),
    //);

    // Done!
    println!(
        "All scenes have been created and written to the 'stl' folder (where applicable)."
    );

    #[cfg(feature = "stl-io")]
    {
        let cube1 = Mesh::cube(3.0, None).translate(1.0, 1.0, 1.0);
        let cube2 = cube1.translate(2.0, 2.0, 2.0);
        let result = cube1.intersection(&cube2.inverse());
        let _ = fs::write(
            "stl/cube_difference.stl",
            result.to_stl_ascii("cube difference"),
        );
    }

    #[cfg(feature = "stl-io")]
    {
        let outer = Sketch::square(200.0, None).center();
        let inner = Sketch::square(160.0, None).center();
        let profile = outer.difference(&inner);

        let path: Vec<Point3<Real>> = vec![
            Point3::new(0.0, 0.0, 0.0), // start
            Point3::new(150.0, 20.0, 20.0),
            Point3::new(300.0, -20.0, -20.0),
            Point3::new(450.0, 20.0, 20.0), // end
        ];

        let tube: Mesh = profile.sweep(&path);
        let _ = fs::write("stl/swept_tube.stl", tube.to_stl_ascii("swept_tube"));
    }

    #[cfg(feature = "stl-io")]
    {
        let rect = Sketch::rectangle(100.0, 60.0, None);
        let hilbert = rect.hilbert_curve(6, 2.0);
        let hilbert_extruded = hilbert.extrude(10.0);
        let _ = fs::write(
            "stl/hilbert_extruded.stl",
            hilbert_extruded.to_stl_ascii("hilbert"),
        );
    }

    let right_triangle = Sketch::right_triangle(10.0, 5.0, None);
    let _ = fs::write(
        "stl/right_triangle.stl",
        right_triangle.to_stl_ascii("right_triangle"),
    );

    let arrow_2d = Sketch::arrow(20.0, 2.0, 10.0, 10.0, None);
    let _ = fs::write("stl/arrow_2d.stl", arrow_2d.to_stl_ascii("arrow"));
}
