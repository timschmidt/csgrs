//! This example demos creating a 3d torus (donut shape)

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/torus";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // torus(outer radi, inner radi, segments around the donut, segments of the tube cross-section)
    let torus = CSG::torus(20.0, 5.0, 48, 24, None);
    write_example(&torus, "torus_2d");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
