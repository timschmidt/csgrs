// main.rs
//
// Minimal example of each function of csgrs (which is now generic over the shared-data type S).
// Here, we do not use any shared data, so we'll bind the generic S to ().

use std::fs;
use nalgebra::{Vector3, Point3};

#[cfg(feature = "image")]
use image::{GrayImage, ImageBuffer};

// A type alias for convenience: no shared data, i.e. S = ()
type CSG = csgrs::CSG<()>;

fn main() {
    // Ensure the /stls folder exists
    let _ = fs::create_dir_all("stl");

    // An ellipsoid with X radius=2, Y radius=1, Z radius=3
    let ellipsoid = CSG::ellipsoid(2.0, 1.0, 3.0, 16, 8, None);
    let _ = fs::write("stl/ellipsoid.stl", ellipsoid.to_stl_ascii("ellipsoid"));

    // 1) polygon()
    let polygon_2d = CSG::polygon(&[[0.0, 0.0], [2.0, 0.0], [1.5, 1.0], [1.0, 2.0]], None);
    let _ = fs::write("stl/polygon_2d.stl", polygon_2d.to_stl_ascii("polygon_2d"));

    // 2) rounded_rectangle(width, height, corner_radius, corner_segments)
    let rrect_2d = CSG::rounded_rectangle(4.0, 2.0, 0.3, 8, None);
    let _ = fs::write(
        "stl/rounded_rectangle_2d.stl",
        rrect_2d.to_stl_ascii("rounded_rectangle_2d"),
    );

    // 3) ellipse(width, height, segments)
    let ellipse = CSG::ellipse(3.0, 1.5, 32, None);
    let _ = fs::write("stl/ellipse.stl", ellipse.to_stl_ascii("ellipse"));

    // 4) regular_ngon(sides, radius)
    let ngon_2d = CSG::regular_ngon(6, 1.0, None); // Hexagon
    let _ = fs::write("stl/ngon_2d.stl", ngon_2d.to_stl_ascii("ngon_2d"));

    // 6) trapezoid(top_width, bottom_width, height)
    let trap_2d = CSG::trapezoid(1.0, 2.0, 2.0, 0.5, None);
    let _ = fs::write("stl/trapezoid_2d.stl", trap_2d.to_stl_ascii("trapezoid_2d"));

    // 10) squircle(width, height, segments)
    let squircle_2d = CSG::squircle(3.0, 3.0, 32, None);
    let _ = fs::write("stl/squircle_2d.stl", squircle_2d.to_stl_ascii("squircle_2d"));

    // 11) keyhole(circle_radius, handle_width, handle_height, segments)
    let keyhole_2d = CSG::keyhole(1.0, 1.0, 2.0, 16, None);
    let _ = fs::write("stl/keyhole_2d.stl", keyhole_2d.to_stl_ascii("keyhole_2d"));

    // 12) reuleaux_polygon(sides, side_len, segments)
    let reuleaux3_2d = CSG::reuleaux(3, 2.0, 64, None); // Reuleaux triangle
    let _ = fs::write(
        "stl/reuleaux3_2d.stl",
        reuleaux3_2d.to_stl_ascii("reuleaux_2d"),
    );

    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux4_2d = CSG::reuleaux(4, 2.0, 64, None); // Reuleaux triangle
    let _ = fs::write(
        "stl/reuleaux4_2d.stl",
        reuleaux4_2d.to_stl_ascii("reuleaux_2d"),
    );

    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux5_2d = CSG::reuleaux(5, 2.0, 64, None); // Reuleaux triangle
    let _ = fs::write(
        "stl/reuleaux5_2d.stl",
        reuleaux5_2d.to_stl_ascii("reuleaux_2d"),
    );

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
                if dx * dx + dy * dy < 15 * 15 {
                    img.put_pixel(x, y, image::Luma([255u8]));
                }
            }
        }
        let csg_img = CSG::from_image(&img, 128, true, None).center();
        let _ = fs::write("stl/from_image.stl", csg_img.to_stl_ascii("from_image"));
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
    let _ = fs::write(
        "stl/arrow_reversed.stl",
        arrow_reversed_csg.to_stl_ascii("arrow_example"),
    );

    // 2-D profile for NACA 2412, 1 m chord, 100 pts / surface
    let naca2412 = CSG::airfoil("2412", 1.0, 100, None);
    let _ = fs::write("stl/naca2412.stl", naca2412.to_stl_ascii("2412"));

    // quick solid wing rib 5 mm thick
    let rib = naca2412.extrude(0.005);
    let _ = fs::write("stl/naca2412_extruded.stl", rib.to_stl_ascii("2412_extruded"));

    // symmetric foil for a centerboard
    let naca0015 = CSG::airfoil("0015", 0.3, 80, None)
        .extrude_vector(nalgebra::Vector3::new(0.0, 0.0, 1.2));
    let _ = fs::write("stl/naca0015.stl", naca0015.to_stl_ascii("naca0015"));

    let oct = CSG::octahedron(10.0, None);
    let _ = fs::write("stl/octahedron.stl", oct.to_stl_ascii("octahedron"));

    //let dodec = CSG::dodecahedron(15.0, None);
    //let _ = fs::write("stl/dodecahedron.stl", dodec.to_stl_ascii(""));

    let ico = CSG::icosahedron(12.0, None);
    let _ = fs::write("stl/icosahedron.stl", ico.to_stl_ascii(""));

    let torus = CSG::torus(20.0, 5.0, 48, 24, None);
    let _ = fs::write("stl/torus.stl", torus.to_stl_ascii(""));

    let heart2d = CSG::heart(30.0, 25.0, 128, None);
    let _ = fs::write("stl/heart2d.stl", heart2d.to_stl_ascii(""));

    let crescent2d = CSG::crescent(10.0, 7.0, 4.0, 64, None);
    let _ = fs::write("stl/crescent2d.stl", crescent2d.to_stl_ascii(""));

    let gear_involute_2d = CSG::involute_gear_2d(
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

    let gear_cycloid_2d = CSG::cycloidal_gear_2d(
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

    let rack_involute = CSG::involute_rack_2d(
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

    let rack_cycloid = CSG::cycloidal_rack_2d(
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

    let spur_involute = CSG::spur_gear_involute(
        2.0, 20, 20.0, 0.05, 0.02, 14, 12.0, // face-width (extrusion thickness)
        None,
    );
    let _ = fs::write(
        "stl/spur_involute.stl",
        spur_involute.to_stl_ascii("spur_involute"),
    );

    let spur_cycloid = CSG::spur_gear_cycloid(
        2.0, 17, 18, 0.05, 20, 12.0, // thickness
        None,
    );
    let _ = fs::write(
        "stl/spur_cycloid.stl",
        spur_cycloid.to_stl_ascii("spur_cycloid"),
    );

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

    #[cfg(feature = "bevymesh")]
    println!("{:#?}", bezier_3d.to_bevy_mesh());

    // Done!
    println!(
        "All scenes have been created and written to the 'stl' folder (where applicable)."
    );
}
