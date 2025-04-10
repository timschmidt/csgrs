//! This example shows boolean operations: Union, Subtract, Intersect

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/boolean_operations";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    let cube = CSG::cube(2.0, 2.0, 2.0, None)
        .translate(1.0, 0.0, 0.0)
        .rotate(0.0, 45.0, 0.0)
        .scale(1.0, 0.5, 2.0);
    write_example(&cube, "cube");

    let sphere = CSG::sphere(1.0, 16, 8, None); // center=(0,0,0), radius=1, slices=16, stacks=8, no metadata
    write_example(&sphere, "sphere");

    // 3) Boolean operations: Union, Subtract, Intersect
    let union_shape = cube.union(&sphere);
    write_example(&union_shape, "union_cube_sphere");

    let subtract_shape = cube.difference(&sphere);
    write_example(&subtract_shape, "subtract_cube_sphere");

    let intersect_shape = cube.intersection(&sphere);
    write_example(&intersect_shape, "intersect_cube_sphere");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap()
    );
}
