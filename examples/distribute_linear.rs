//! This example shows creating a `CSG` with 2d pie slice

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/distribute_linear";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // Create a pie slice of radius 2, from 0 to 90 degrees
    let wedge = CSG::pie_slice(2.0, 0.0, 90.0, 16, None);

    // Distribute that wedge along a linear axis
    let wedge_line = wedge.distribute_linear(4, nalgebra::Vector3::new(1.0, 0.0, 0.0), 3.0);

    write_example(&wedge_line, "wedge_line");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap()
    );
}
