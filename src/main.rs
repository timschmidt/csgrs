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

    #[cfg(feature = "bevymesh")]
    println!("{:#?}", bezier_3d.to_bevy_mesh());

    // Done!
    println!(
        "All scenes have been created and written to the 'stl' folder (where applicable)."
    );
}
