//! This example demos creating a circle with keyway shape

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/circle_with_keyway";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // 1. Circle with keyway
    let keyway_shape = CSG::circle_with_keyway(10.0, 64, 2.0, 3.0, None);
    write_example(&keyway_shape, "keyway_shape");

    // Extrude it 2 units:
    let keyway_3d = keyway_shape.extrude(2.0);
    write_example(&keyway_3d, "keyway_3d");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
