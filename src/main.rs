// main.rs
//
// Minimal example of each function of csgrs (which is now generic over the shared-data type S).
// Here, we do not use any shared data, so we'll bind the generic S to ().

use csgrs::float_types::Real;
use std::fs;
use nalgebra::{Vector3, Point3};
use csgrs::plane::Plane;

#[cfg(feature = "image")]
use image::{GrayImage, ImageBuffer};

#[cfg(feature = "metaballs")]
use csgrs::metaballs::MetaBall;

// A type alias for convenience: no shared data, i.e. S = ()
#[allow(clippy::upper_case_acronyms)]
type CSG = csgrs::csg::CSG<()>;

fn main() {
    // Ensure the /stls folder exists
    let _ = fs::create_dir_all("stl");

    // 1) Basic shapes: cube, sphere, cylinder
    let cube = CSG::cube(2.0, 2.0, 2.0, None);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/cube.stl", cube.to_stl_binary("cube").unwrap());

    // center=(0,0,0), radius=1, slices=16, stacks=8, no metadata
    let sphere = CSG::sphere(1.0, 16, 8, None);


    let cylinder = CSG::cylinder(1.0, 2.0, 32, None); // start=(0,-1,0), end=(0,1,0), radius=1.0, slices=32
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/cylinder.stl", cylinder.to_stl_binary("cylinder").unwrap());

    // 1) Create a cube from (-1,-1,-1) to (+1,+1,+1)
    //    (By default, CSG::cube(None) is from -1..+1 if the "radius" is [1,1,1].)
    let cube = CSG::cube(1.0, 1.0, 1.0, None);
    // 2) Flatten into the XY plane
    let flattened = cube.clone().flatten();
    let _ = fs::write("stl/flattened_cube.stl", flattened.to_stl_ascii("flattened_cube"));

    // Create a frustum (start=-2, end=+2) with radius1 = 1, radius2 = 2, 32 slices
    let frustum = CSG::frustum_ptp(Point3::new(0.0, 0.0, -2.0), Point3::new(0.0, 0.0, 2.0), 1.0, 2.0, 32, None);
    let _ = fs::write("stl/frustum.stl", frustum.to_stl_ascii("frustum"));

    // 1) Create a cylinder (start=-1, end=+1) with radius=1, 32 slices
    let cyl = CSG::frustum_ptp(Point3::new(0.0, 0.0, -1.0), Point3::new(0.0, 0.0, 1.0), 1.0, 1.0, 32, None);
    // 2) Slice at z=0
    #[cfg(feature = "hashmap")]
    {
    let cross_section = cyl.slice(Plane { normal: Vector3::z(), intercept: 0.0 });
    let _ = fs::write("stl/sliced_cylinder.stl", cyl.to_stl_ascii("sliced_cylinder"));
    let _ = fs::write("stl/sliced_cylinder_slice.stl", cross_section.to_stl_ascii("sliced_cylinder_slice"));
    }

    // let poor_geometry_shape = moved_cube.difference(&sphere);
    // #[cfg(feature = "earclip-io")]
    // let retriangulated_shape = poor_geometry_shape.triangulate_earclip();
    // #[cfg(all(feature = "earclip-io", feature = "stl-io"))]
    // let _ = fs::write("stl/retriangulated.stl", retriangulated_shape.to_stl_binary("retriangulated").unwrap());

    let sphere_test = CSG::sphere(1.0, 16, 8, None);
    let cube_test = CSG::cube(1.0, 1.0, 1.0, None);
    let res = cube_test.difference(&sphere_test);
    #[cfg(feature = "stl-io")]
    let _ = fs::write("stl/sphere_cube_test.stl", res.to_stl_binary("sphere_cube_test").unwrap());
    assert_eq!(res.bounding_box(), cube_test.bounding_box());

    // Create a pie slice of radius 2, from 0 to 90 degrees
    let wedge = CSG::pie_slice(2.0, 0.0, 90.0, 16, None);
    let _ = fs::write("stl/pie_slice.stl", wedge.to_stl_ascii("pie_slice"));

    // Create a 2D "metaball" shape from 3 circles
    use nalgebra::Point2;
    let balls_2d = vec![
        (Point2::new(0.0, 0.0), 1.0),
        (Point2::new(1.5, 0.0), 1.0),
        (Point2::new(0.75, 1.0), 0.5),
    ];
    let mb2d = CSG::metaball_2d(&balls_2d, (100, 100), 1.0, 0.25, None);
    let _ = fs::write("stl/mb2d.stl", mb2d.to_stl_ascii("metaballs2d"));

    // Create a supershape
    let sshape = CSG::supershape(1.0, 1.0, 6.0, 1.0, 1.0, 1.0, 128, None);
    let _ = fs::write("stl/supershape.stl", sshape.to_stl_ascii("supershape"));

    // Distribute a square along an arc
    let square = CSG::circle(1.0, 32, None);
    let arc_array = square.distribute_arc(5, 5.0, 0.0, 180.0)
        .expect("count is not less then 1");
    let _ = fs::write("stl/arc_array.stl", arc_array.to_stl_ascii("arc_array"));

    // Distribute that wedge along a linear axis
    let wedge_line = wedge.distribute_linear(4, nalgebra::Vector3::new(1.0, 0.0, 0.0), 3.0)
        .expect("count is not less then 1");
    let _ = fs::write("stl/wedge_line.stl", wedge_line.to_stl_ascii("wedge_line"));

    // Make a 4x4 grid of the supershape
    let grid_of_ss = sshape.distribute_grid(4, 4, 3.0, 3.0)
        .expect("count is not less then 1");
    let _ = fs::write("stl/grid_of_ss.stl", grid_of_ss.to_stl_ascii("grid_of_ss"));

    // 1. Circle with keyway
    let keyway_shape = CSG::circle_with_keyway(10.0, 64, 2.0, 3.0, None);
    let _ = fs::write("stl/keyway_shape.stl", keyway_shape.to_stl_ascii("keyway_shape"));
    // Extrude it 2 units:
    let keyway_3d = keyway_shape.extrude(2.0);
    let _ = fs::write("stl/keyway_3d.stl", keyway_3d.to_stl_ascii("keyway_3d"));

    // 2. D-shape
    let d_shape = CSG::circle_with_flat(5.0, 32, 2.0, None);
    let _ = fs::write("stl/d_shape.stl", d_shape.to_stl_ascii("d_shape"));
    let d_3d = d_shape.extrude(1.0);
    let _ = fs::write("stl/d_3d.stl", d_3d.to_stl_ascii("d_3d"));

    // 3. Double-flat circle
    let double_flat = CSG::circle_with_two_flats(8.0, 64, 3.0, None);
    let _ = fs::write("stl/double_flat.stl", double_flat.to_stl_ascii("double_flat"));
    let df_3d = double_flat.extrude(0.5);
    let _ = fs::write("stl/df_3d.stl", df_3d.to_stl_ascii("df_3d"));

    // A 3D teardrop shape
    let teardrop_solid = CSG::teardrop(3.0, 5.0, 32, 32, None);
    let _ = fs::write("stl/teardrop_solid.stl", teardrop_solid.to_stl_ascii("teardrop_solid"));

    // A 3D egg shape
    let egg_solid = CSG::egg(2.0, 4.0, 8, 16, None);
    let _ = fs::write("stl/egg_solid.stl", egg_solid.to_stl_ascii("egg_solid"));

    // An ellipsoid with X radius=2, Y radius=1, Z radius=3
    let ellipsoid = CSG::ellipsoid(2.0, 1.0, 3.0, 16, 8, None);
    let _ = fs::write("stl/ellipsoid.stl", ellipsoid.to_stl_ascii("ellipsoid"));

    // A teardrop 'blank' hole
    let teardrop_cylinder = CSG::teardrop_cylinder(2.0, 4.0, 32.0, 16, None);
    let _ = fs::write("stl/teardrop_cylinder.stl", teardrop_cylinder.to_stl_ascii("teardrop_cylinder"));

    // 1) polygon()
    let polygon_2d = CSG::polygon(
        &[
            [0.0, 0.0],
            [2.0, 0.0],
            [1.5, 1.0],
            [1.0, 2.0],
        ],
        None,
    );
    let _ = fs::write("stl/polygon_2d.stl", polygon_2d.to_stl_ascii("polygon_2d"));

    // 2) rounded_rectangle(width, height, corner_radius, corner_segments)
    let rrect_2d = CSG::rounded_rectangle(4.0, 2.0, 0.3, 8, None);
    let _ = fs::write("stl/rounded_rectangle_2d.stl", rrect_2d.to_stl_ascii("rounded_rectangle_2d"));

    // 3) ellipse(width, height, segments)
    let ellipse = CSG::ellipse(3.0, 1.5, 32, None);
    let _ = fs::write("stl/ellipse.stl", ellipse.to_stl_ascii("ellipse"));

    // 4) regular_ngon(sides, radius)
    let ngon_2d = CSG::regular_ngon(6, 1.0, None); // Hexagon
    let _ = fs::write("stl/ngon_2d.stl", ngon_2d.to_stl_ascii("ngon_2d"));

    // 6) trapezoid(top_width, bottom_width, height)
    let trap_2d = CSG::trapezoid(1.0, 2.0, 2.0, 0.5, None);
    let _ = fs::write("stl/trapezoid_2d.stl", trap_2d.to_stl_ascii("trapezoid_2d"));

    // 7) star(num_points, outer_radius, inner_radius)
    let star_2d = CSG::star(5, 2.0, 0.8, None);
    let _ = fs::write("stl/star_2d.stl", star_2d.to_stl_ascii("star_2d"));

    // 8) teardrop(width, height, segments) [2D shape]
    let teardrop_2d = CSG::teardrop_outline(2.0, 3.0, 16, None);
    let _ = fs::write("stl/teardrop_2d.stl", teardrop_2d.to_stl_ascii("teardrop_2d"));

    // 9) egg_outline(width, length, segments) [2D shape]
    let egg_2d = CSG::egg_outline(2.0, 4.0, 32, None);
    let _ = fs::write("stl/egg_outline_2d.stl", egg_2d.to_stl_ascii("egg_outline_2d"));

    // 10) squircle(width, height, segments)
    let squircle_2d = CSG::squircle(3.0, 3.0, 32, None);
    let _ = fs::write("stl/squircle_2d.stl", squircle_2d.to_stl_ascii("squircle_2d"));

    // 11) keyhole(circle_radius, handle_width, handle_height, segments)
    let keyhole_2d = CSG::keyhole(1.0, 1.0, 2.0, 16, None);
    let _ = fs::write("stl/keyhole_2d.stl", keyhole_2d.to_stl_ascii("keyhole_2d"));

    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux3_2d = CSG::reuleaux_polygon(3, 2.0, 16, None); // Reuleaux triangle
    let _ = fs::write("stl/reuleaux3_2d.stl", reuleaux3_2d.to_stl_ascii("reuleaux_2d"));

    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux4_2d = CSG::reuleaux_polygon(4, 2.0, 16, None); // Reuleaux triangle
    let _ = fs::write("stl/reuleaux4_2d.stl", reuleaux4_2d.to_stl_ascii("reuleaux_2d"));

    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux5_2d = CSG::reuleaux_polygon(5, 2.0, 16, None); // Reuleaux triangle
    let _ = fs::write("stl/reuleaux5_2d.stl", reuleaux5_2d.to_stl_ascii("reuleaux_2d"));
    
    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux6_2d = CSG::reuleaux_polygon(6, 2.0, 16, None); // Reuleaux triangle
    let _ = fs::write("stl/reuleaux6_2d.stl", reuleaux6_2d.to_stl_ascii("reuleaux_2d"));
    
    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux7_2d = CSG::reuleaux_polygon(7, 2.0, 16, None); // Reuleaux triangle
    let _ = fs::write("stl/reuleaux7_2d.stl", reuleaux7_2d.to_stl_ascii("reuleaux_2d"));
    
    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux8_2d = CSG::reuleaux_polygon(8, 2.0, 16, None); // Reuleaux triangle
    let _ = fs::write("stl/reuleaux8_2d.stl", reuleaux8_2d.to_stl_ascii("reuleaux_2d"));
    
    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux9_2d = CSG::reuleaux_polygon(9, 2.0, 16, None); // Reuleaux triangle
    let _ = fs::write("stl/reuleaux9_2d.stl", reuleaux9_2d.to_stl_ascii("reuleaux_2d"));
    
    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux10_2d = CSG::reuleaux_polygon(10, 2.0, 16, None); // Reuleaux triangle
    let _ = fs::write("stl/reuleaux10_2d.stl", reuleaux10_2d.to_stl_ascii("reuleaux_2d"));

    // 13) ring(inner_diam, thickness, segments)
    let ring_2d = CSG::ring(5.0, 1.0, 32, None);
    let _ = fs::write("stl/ring_2d.stl", ring_2d.to_stl_ascii("ring_2d"));

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
                if dx*dx + dy*dy < 15*15 {
                    img.put_pixel(x, y, image::Luma([255u8]));
                }
            }
        }
        let csg_img = CSG::from_image(&img, 128, true, None).center();
        let _ = fs::write("stl/from_image.stl", csg_img.to_stl_ascii("from_image"));
    }

    // 16) gyroid(...) – uses the current CSG volume as a bounding region
    // Let's reuse the `cube` from above:
    #[cfg(feature = "stl-io")]
    {
        let gyroid_inside_cube = cube.gyroid(32, 2.0, 0.0, None);
        let _ = fs::write("stl/gyroid_cube.stl", gyroid_inside_cube.to_stl_binary("gyroid_cube").unwrap());
    }
    
    // Define the start point and the arrow direction vector.
    // The arrow’s length is the norm of the direction vector.
    let start = Point3::new(1.0, 1.0, 1.0);
    let direction = Vector3::new(10.0, 5.0, 20.0);
    
    // Define the resolution (number of segments for the cylindrical shaft and head).
    let segments = 16;
    
    // Create the arrow. We pass `None` for metadata.
    let arrow_csg = CSG::arrow(start, direction, segments, true, None::<()>);
    let _ = fs::write("stl/arrow.stl", arrow_csg.to_stl_ascii("arrow_example"));
    
    let arrow_reversed_csg = CSG::arrow(start, direction, segments, false, None::<()>);
    let _ = fs::write("stl/arrow_reversed.stl", arrow_reversed_csg.to_stl_ascii("arrow_example"));


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
    {
        let tri_2d = CSG::right_triangle(2.0, 1.0, None);
        // A tiny arrow pointing from the right-angle corner outward:
        let arrow = CSG::arrow(
            Point3::new(0.0, 0.0, 0.1), // at corner
            Vector3::new(0.5, 0.0, 0.0), 
            8,
            true,
            None::<()>,
        )
        .scale(0.05, 0.05, 0.05);
        let scene = tri_2d.extrude(0.1).union(&arrow);
        let _ = fs::write("stl/scene_right_triangle.stl", scene.to_stl_ascii("scene_right_triangle"));
    }

    // Scene B: Demonstrate extrude_vector(direction)
    {
        let circle2d = CSG::circle(1.0, 32, None);
        // extrude along an arbitrary vector
        let extruded_along_vec = circle2d.extrude_vector(Vector3::new(0.0, 0.0, 2.0));
        let _ = fs::write("stl/scene_extrude_vector.stl", extruded_along_vec.to_stl_ascii("scene_extrude_vector"));
    }

    // Scene E: Demonstrate center() (moves shape so bounding box is centered on the origin)
    {
        let off_center_circle = CSG::circle(1.0, 32, None).translate(5.0, 2.0, 0.0).extrude(0.1);
        let centered_circle = off_center_circle.center();
        let _ = fs::write("stl/scene_circle_off_center.stl", off_center_circle.to_stl_ascii("scene_circle_off_center"));
        let _ = fs::write("stl/scene_circle_centered.stl", centered_circle.to_stl_ascii("scene_circle_centered"));
    }

    // Scene F: Demonstrate float() (moves shape so bottom is at z=0)
    {
        let sphere_for_float = CSG::sphere(1.0, 16, 8, None).translate(0.0, 0.0, -1.5);
        let floated = sphere_for_float.float();
        let _ = fs::write("stl/scene_sphere_before_float.stl", sphere_for_float.to_stl_ascii("scene_sphere_before_float"));
        let _ = fs::write("stl/scene_sphere_floated.stl", floated.to_stl_ascii("scene_sphere_floated"));
    }

    // Scene G: Demonstrate inverse() (flips inside/outside)
    {
        // Hard to visualize in STL, but let's do it anyway
        let inv_sphere = sphere.inverse();
        #[cfg(feature = "stl-io")]
        let _ = fs::write("stl/scene_inverse_sphere.stl", inv_sphere.to_stl_binary("scene_inverse_sphere").unwrap());
    }

    // Scene H: Demonstrate tessellate() (forces triangulation)
    {
        let tri_sphere = sphere.tessellate();
        #[cfg(feature = "stl-io")]
        let _ = fs::write("stl/scene_tessellate_sphere.stl", tri_sphere.to_stl_binary("scene_tessellate_sphere").unwrap());
    }

    // Scene I: Demonstrate slice(plane) – slice a cube at z=0
    {
        let plane_z = Plane{ normal: Vector3::z(), intercept: 0.5 };
        let sliced_polygons = cube.slice(plane_z);
        let _ = fs::write("stl/scene_sliced_cube.stl", cube.to_stl_ascii("sliced_cube"));
        // Save cross-section as well
        let _ = fs::write("stl/scene_sliced_cube_section.stl", sliced_polygons.to_stl_ascii("sliced_cube_section"));
    }

    // Scene J: Demonstrate re-computing vertices() or printing them
    {
        let circle_extruded = CSG::circle(1.0, 32, None).extrude(0.5);
        let verts = circle_extruded.vertices();
        println!("Scene J circle_extruded has {} vertices", verts.len());
        // We'll still save an STL so there's a visual
        let _ = fs::write("stl/scene_j_circle_extruded.stl", circle_extruded.to_stl_ascii("scene_j_circle_extruded"));
    }

    // Scene K: Demonstrate reuleaux_polygon with a typical triangle shape
    // (already used sides=4 above, so let's do sides=3 here)
    {
        let reuleaux_tri = CSG::reuleaux_polygon(3, 2.0, 16, None).extrude(0.1);
        let _ = fs::write("stl/scene_reuleaux_triangle.stl", reuleaux_tri.to_stl_ascii("scene_reuleaux_triangle"));
    }

    // Scene L: Demonstrate rotate_extrude (360 deg) on a square
    {
        let small_square = CSG::square(1.0, 1.0, None).translate(2.0, 0.0, 0.0);
        let revolve = small_square.rotate_extrude(360.0, 24);
        let _ = fs::write("stl/scene_square_revolve_360.stl", revolve.to_stl_ascii("scene_square_revolve_360"));
    }

    // Scene M: Demonstrate “mirror” across a Y=0 plane
    {
        let plane_y = Plane{ normal: Vector3::y(), intercept: 0.0 };
        let shape = CSG::square(2.0, 1.0, None).translate(1.0, 1.0, 0.0).extrude(0.1);
        let mirrored = shape.mirror(plane_y);
        let _ = fs::write("stl/scene_square_mirrored_y.stl", mirrored.to_stl_ascii("scene_square_mirrored_y"));
    }

    // Scene N: Demonstrate scale() 
    {
        let scaled = sphere.scale(1.0, 2.0, 0.5);
        #[cfg(feature = "stl-io")]
        let _ = fs::write("stl/scene_scaled_sphere.stl", scaled.to_stl_binary("scene_scaled_sphere").unwrap());
    }

    // Scene O: Demonstrate transform() with an arbitrary affine matrix
    {
        use nalgebra::{Matrix4, Translation3};
        let xlate = Translation3::new(2.0, 0.0, 1.0).to_homogeneous();
        // Scale matrix
        let scale_mat = Matrix4::new_scaling(0.5);
        // Combine
        let transform_mat = xlate * scale_mat;
        let shape = CSG::cube(1.0,1.0,1.0, None).transform(&transform_mat);
        let _ = fs::write("stl/scene_transform_cube.stl", shape.to_stl_ascii("scene_transform_cube"));
    }

    // Scene P: Demonstrate offset(distance)
    {
        let poly_2d = CSG::polygon(
            &[
                [0.0,0.0],
                [2.0,0.0],
                [1.0,1.5]
            ],
            None
        );
        let grown = poly_2d.offset(0.2);
        let scene = grown.extrude(0.1);
        let _ = fs::write("stl/scene_offset_grown.stl", scene.to_stl_ascii("scene_offset_grown"));
    }

    // Done!
    println!("All scenes have been created and written to the 'stl' folder (where applicable).");
}
