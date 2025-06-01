//! This example shows polyhedron usage with a simple tetrahedron

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/polyhedron";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // 12) Polyhedron example (simple tetrahedron):
    let points = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.5, 1.0, 0.0],
        [0.5, 0.5, 1.0],
    ];
    let faces = &[
        vec![0, 2, 1], // base triangle
        vec![0, 1, 3], // side
        vec![1, 2, 3],
        vec![2, 0, 3],
    ];
    let poly = CSG::polyhedron(&points, faces, None);
    write_example(&poly, "tetrahedron");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_ascii(name),
    );
}
