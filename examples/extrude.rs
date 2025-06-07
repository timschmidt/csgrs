//! This example shows Extrude & Rotate-Extrude oparations

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/extrude";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    let square_2d = CSG::square(2.0, 2.0, None); // 2x2 square, centered
    let circle_2d = CSG::circle(1.0, 32, None);

    // 8) Extrude & Rotate-Extrude
    let extruded_square = square_2d.extrude(1.0);
    write_example(&extruded_square, "square_extrude");

    let revolve_circle = circle_2d.translate(10.0, 0.0, 0.0).rotate_extrude(360.0, 32);
    write_example(&revolve_circle, "circle_revolve_360");

    let partial_revolve = circle_2d.translate(10.0, 0.0, 0.0).rotate_extrude(180.0, 32);
    write_example(&partial_revolve, "circle_revolve_180");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
