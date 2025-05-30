//! This example demos creating a circle with a flat side (looks like: 'D') and one with two flats

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/circle_with_flat";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // 2. D-shape
    let d_shape = CSG::circle_with_flat(5.0, 32, 2.0, None);
    write_example(&d_shape, "d_shape");
    let d_3d = d_shape.extrude(1.0);
    write_example(&d_3d, "d_shape_extruded");

    // 3. Double-flat circle
    let double_flat = CSG::circle_with_two_flats(8.0, 64, 3.0, None);
    write_example(&double_flat, "double_flat");
    let df_3d = double_flat.extrude(0.5);
    write_example(&df_3d, "double_flat_extruded");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap()
    );
}
