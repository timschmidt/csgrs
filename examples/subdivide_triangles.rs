//! This example shows subdividing all polygons in this CSG

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/subdivide_triangles";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    let sphere = CSG::sphere(1.0, 16, 8, None); // center=(0,0,0), radius=1, slices=16, stacks=8, no metadata

    // 9) Subdivide triangles (for smoother sphere or shapes):
    let subdiv_sphere = sphere.subdivided_triangles(2.try_into().expect("not 0")); // 2 subdivision levels
    write_example(&subdiv_sphere, "sphere_subdiv2");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap()
    );
}
