//! This example shows creating a supershape

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/supershape";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // Create a supershape
    let sshape = CSG::supershape(1.0, 1.0, 6.0, 1.0, 1.0, 1.0, 128, None);
    write_example(&sshape, "supershape");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
