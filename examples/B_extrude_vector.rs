//! Scene B: Demonstrate extrude_vector(direction)

use csgrs::CSG;
use nalgebra::Vector3;
use std::fs;

fn main() {
    let circle2d = CSG::<()>::circle(1.0, 32, None);
    // extrude along an arbitrary vector
    let extruded_along_vec = circle2d.extrude_vector(Vector3::new(0.0, 0.0, 2.0));
    let _ = fs::write(
        "stl/scene_extrude_vector.stl",
        extruded_along_vec.to_stl_ascii("scene_extrude_vector"),
    );
}
