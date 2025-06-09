//! This example shows computing a uniform B-spline

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/b-spline";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // ---------------------------------------------------------------------
    // B-spline demo --------------------------------------------------------
    let bspline_ctrl = &[[0.0, 0.0], [1.0, 2.5], [3.0, 3.0], [5.0, 0.0], [6.0, -1.5]];
    let bspline_2d = CSG::bspline(
        bspline_ctrl,
        // degree p =
        3,
        // seg/span
        32,
        None,
    );
    write_example(&bspline_2d, "bspline_2d");

    // a quick thickening so we can see it in a solid viewer
    let bspline_3d = bspline_2d.extrude(0.25);
    write_example(&bspline_3d, "bspline_extruded");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
