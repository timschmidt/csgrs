//! This example demos creating a 3d teardrop, a teardrop cylinder, and a 2d teardrop outline

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/teardrop";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // A 3D teardrop shape
    let teardrop_solid = CSG::teardrop(3.0, 5.0, 32, 32, None);
    write_example(&teardrop_solid, "teardrop_solid");

    // A teardrop 'blank' hole
    let teardrop_cylinder = CSG::teardrop_cylinder(2.0, 4.0, 32.0, 16, None);
    write_example(&teardrop_cylinder, "teardrop_cylinder");

    // 8) teardrop(width, height, segments) [2D shape]
    let teardrop_2d = CSG::teardrop_outline(2.0, 3.0, 16, None);
    write_example(&teardrop_2d, "teardrop_2d");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap()
    );
}
