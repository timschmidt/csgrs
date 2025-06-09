//! This example shows distributing a circle along an arc

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/distribute_arc";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // Distribute a circle along an arc
    let circle = CSG::circle(1.0, 32, None);
    write_example(&circle, "circle");

    let arc_array = circle.distribute_arc(5, 5.0, 0.0, 180.0);
    write_example(&arc_array, "arc_array");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
