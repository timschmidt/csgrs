//! Scene M: Demonstrate “mirror” across a Y=0 plane

use csgrs::{CSG, Plane};
use std::fs;
use nalgebra::Vector3;

fn main() {
    let plane_y = Plane::from_normal(Vector3::y(), 0.0);
    let shape = CSG::<()>::square(2.0, 1.0, None).translate(1.0, 1.0, 0.0).extrude(0.1);
    let mirrored = shape.mirror(plane_y);
    let _ = fs::write("stl/scene_square_mirrored_y.stl", mirrored.to_stl_ascii("scene_square_mirrored_y"));
}
