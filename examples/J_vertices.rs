//! Scene J: Demonstrate re-computing vertices() or printing them

use csgrs::CSG;
use std::fs;

fn main() {
    let circle_extruded = CSG::<()>::circle(1.0, 32, None).extrude(0.5);
    let verts = circle_extruded.vertices();
    println!("Scene J circle_extruded has {} vertices", verts.len());
    // We'll still save an STL so there's a visual
    let _ = fs::write(
        "stl/scene_j_circle_extruded.stl",
        circle_extruded
            .to_stl_binary("scene_j_circle_extruded")
            .unwrap(),
    );
}
