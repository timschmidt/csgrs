//! This example shows creating a `CSG` with 2d pie slice

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/pie_slice";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // Create a pie slice of radius 2, from 0 to 90 degrees
    let wedge = CSG::pie_slice(2.0, 0.0, 90.0, 16, None);

    write_example(&wedge, "wedge");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
