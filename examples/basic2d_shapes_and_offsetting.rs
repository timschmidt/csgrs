//! This example shows basic 2D shapes and 2D offsetting

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/basic2d";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // 7) 2D shapes and 2D offsetting
    let square_2d = CSG::square(2.0, 2.0, None); // 2x2 square, centered
    write_example(&square_2d, "square_2d");

    let circle_2d = CSG::<()>::circle(1.0, 32, None);
    write_example(&circle_2d, "circle_2d");

    let grown_2d = square_2d.offset(0.5);
    write_example(&grown_2d, "square_2d_grow_0_5");

    let shrunk_2d = square_2d.offset(-0.5);
    write_example(&shrunk_2d, "square_2d_shrink_0_5");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
