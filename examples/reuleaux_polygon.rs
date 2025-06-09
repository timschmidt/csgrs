//! This example shows a variety reuleaux polygon usage examples

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/reuleaux_polygon";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // 12) reuleaux_polygon(sides, side_len, segments)
    let reuleaux3_2d = CSG::reuleaux(3, 2.0, 64, None); // Reuleaux triangle
    write_example(&reuleaux3_2d, "reuleaux3_2d");

    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux4_2d = CSG::reuleaux(4, 2.0, 64, None); // Reuleaux triangle
    write_example(&reuleaux4_2d, "reuleaux4_2d");

    // 12) reuleaux_polygon(sides, radius, arc_segments_per_side)
    let reuleaux5_2d = CSG::reuleaux(5, 2.0, 64, None); // Reuleaux triangle
    write_example(&reuleaux5_2d, "reuleaux5_2d");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
