//! Scene E: Demonstrate center() (moves shape so bounding box is centered on the origin)

use csgrs::CSG;
use std::fs;

fn main() {
    let off_center_circle = CSG::<()>::circle(1.0, 32, None)
        .translate(5.0, 2.0, 0.0)
        .extrude(0.1);
    let centered_circle = off_center_circle.center();
    let _ = fs::write(
        "stl/scene_circle_off_center.stl",
        off_center_circle.to_stl_ascii("scene_circle_off_center"),
    );
    let _ = fs::write(
        "stl/scene_circle_centered.stl",
        centered_circle.to_stl_ascii("scene_circle_centered"),
    );
}
