//! This example demos creating an arrow and an inverse of the same arrow

use csgrs::CSG;
use std::{fs, path::Path};
use nalgebra::{Vector3, Point3};

const PATH: &str = "stl/arrow";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    // start point
    let start = Point3::new(1.0, 1.0, 1.0);
    // Arrow direction vector, the arrow’s length is the norm of the direction vector.
    let direction = Vector3::new(10.0, 5.0, 20.0);

    // number of segments for the cylindrical shaft and head
    let segments = 16;

    // Create the arrow. We pass `None` for metadata.
    let arrow = CSG::arrow(start, direction, segments, true, None::<()>);
    write_example(&arrow, "arrow");

    let arrow_reversed = CSG::arrow(start, direction, segments, false, None::<()>);
    write_example(&arrow_reversed, "arrow_reversed");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
