//! This example shows renormalizing polygons for flat shading

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/renormalize";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    let cube = CSG::cube(2.0, 2.0, 2.0, None)
        .translate(1.0, 0.0, 0.0)
        .rotate(0.0, 45.0, 0.0)
        .scale(1.0, 0.5, 2.0);
    let sphere = CSG::sphere(1.0, 16, 8, None); // center=(0,0,0), radius=1, slices=16, stacks=8

    // polygon to renormalize
    let mut union_shape = cube.union(&sphere);
    write_example(&union_shape, "union_cube_sphere");

    // 10) Renormalize polygons (flat shading):
    union_shape.renormalize();
    write_example(&union_shape, "union_renormalized");    
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap()
    );
}
