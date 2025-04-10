//! This example shows basic shapes: cube, sphere, cylinder

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/basic_shapes";

fn main() {
    // Ensure the /stls folder exists
    let _ = fs::create_dir_all(PATH);

    // 1) Basic shapes: cube, sphere, cylinder
    let cube = CSG::cube(2.0, 2.0, 2.0, None);
    #[cfg(feature = "stl-io")]
    write_example(&cube, "cube");

    let sphere = CSG::sphere(1.0, 16, 8, None); // center=(0,0,0), radius=1, slices=16, stacks=8, no metadata
    write_example(&sphere, "sphere");

    let cylinder = CSG::cylinder(1.0, 2.0, 32, None); // start=(0,-1,0), end=(0,1,0), radius=1.0, slices=32
    write_example(&cylinder, "cylinder");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap()
    );
}
