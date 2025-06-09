//! This example demos creating a 3d ellipsoid and a 2d ellipse

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/ellipse_and_ellipsoid";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // An ellipsoid with X radius=2, Y radius=1, Z radius=3
    let ellipsoid = CSG::ellipsoid(2.0, 1.0, 3.0, 16, 8, None);
    write_example(&ellipsoid, "ellipsoid");

    // ellipse(width, height, segments)
    let ellipse = CSG::ellipse(3.0, 1.5, 32, None);
    write_example(&ellipse, "ellipse");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
