//! This example shows polyhedron usage with a simple tetrahedron

use csgrs::csg::CSG;
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
    let faces: Vec<&[usize]> = vec![
        &[0, 2, 1], // base triangle
        &[0, 1, 3], // side
        &[1, 2, 3],
        &[2, 0, 3],
    ];
    let poly = CSG::polyhedron(&points, faces.as_slice(), None);
    write_example(&poly.unwrap(), "tetrahedron");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_ascii(name)
    );
}
