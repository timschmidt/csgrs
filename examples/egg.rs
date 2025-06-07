//! This example demos creating a 3d egg and a 2d egg outline

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/egg";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // A 3D egg shape
    let egg_solid = CSG::egg(2.0, 4.0, 8, 16, None);
    write_example(&egg_solid, "egg_solid_2d");

    // 9) egg_outline(width, length, segments) [2D shape]
    let egg_2d = CSG::egg_outline(2.0, 4.0, 32, None);
    write_example(&egg_2d, "egg_outline_2d");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
