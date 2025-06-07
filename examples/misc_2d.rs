//! This example demos the following 2d shapes:
//! - polygon 2d
//! - rounded rectangle
//! - regular ngon
//! - trapezoid
//! - squircle
//! - keyhole
//! - ring
//! - heart
//! - crescent

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/misc_2d";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // 1) polygon
    let polygon_2d = CSG::polygon(&[[0.0, 0.0], [2.0, 0.0], [1.5, 1.0], [1.0, 2.0]], None);
    let _ = fs::write("stl/polygon_2d.stl", polygon_2d.to_stl_ascii("polygon_2d"));
    write_example(&polygon_2d, "polygon_2d");

    // 2) rounded_rectangle(width, height, corner_radius, corner_segments)
    let rrect_2d = CSG::rounded_rectangle(4.0, 2.0, 0.3, 8, None);
    let _ = fs::write(
        "stl/rounded_rectangle_2d.stl",
        rrect_2d.to_stl_ascii("rounded_rectangle_2d"),
    );
    write_example(&rrect_2d, "rounded_rectangle");

    // 3) regular_ngon(sides, radius)
    let ngon_2d = CSG::regular_ngon(6, 1.0, None); // Hexagon
    write_example(&ngon_2d, "ngon");

    // 4) trapezoid(top_width, bottom_width, height)
    let trap_2d = CSG::trapezoid(1.0, 2.0, 2.0, 0.5, None);
    write_example(&trap_2d, "trapezoid");

    // 5) squircle(width, height, segments)
    let squircle_2d = CSG::squircle(3.0, 3.0, 32, None);
    write_example(&squircle_2d, "squircle");

    // 6) keyhole(circle_radius, handle_width, handle_height, segments)
    let keyhole_2d = CSG::keyhole(1.0, 1.0, 2.0, 16, None);
    write_example(&keyhole_2d, "keyhole");

    // 7) ring(inner_diam, thickness, segments)
    let ring_2d = CSG::ring(5.0, 1.0, 32, None);
    write_example(&ring_2d, "ring");

    let heart2d = CSG::heart(30.0, 25.0, 128, None);
    write_example(&heart2d, "heart");

    let crescent2d = CSG::crescent(10.0, 7.0, 4.0, 64, None);
    write_example(&crescent2d, "crescent");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
