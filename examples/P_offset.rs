//! Scene P: Demonstrate offset(distance)

use csgrs::CSG;
use std::fs;

fn main() {
    let poly_2d = CSG::<()>::polygon(&[[0.0, 0.0], [2.0, 0.0], [1.0, 1.5]], None);
    let grown = poly_2d.offset(0.2);
    let scene = grown.extrude(0.1);
    let _ = fs::write(
        "stl/scene_offset_grown.stl",
        scene.to_stl_ascii("scene_offset_grown"),
    );
}
