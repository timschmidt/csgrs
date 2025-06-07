//! This example shows creating a bézier curve

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/bezier_curve";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // ---------------------------------------------------------------------
    // Bézier curve demo ----------------------------------------------------
    let bezier_ctrl = &[
        [0.0, 0.0], // P0
        [1.0, 2.0], // P1
        [3.0, 3.0], // P2
        [4.0, 0.0], // P3
    ];
    let bezier_2d = CSG::bezier(bezier_ctrl, 128, None);
    write_example(&bezier_2d, "bezier_2d");

    // give it a little “body” so we can see it in a solid viewer
    let bezier_3d = bezier_2d.extrude(0.25);
    write_example(&bezier_3d, "bezier_extruded");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
