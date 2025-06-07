//! This example shows creating a 2d ring shape

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/ring";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // 13) ring(inner_diam, thickness, segments)
    let ring_2d = CSG::ring(5.0, 1.0, 32, None);
    write_example(&ring_2d, "ring_2d");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
