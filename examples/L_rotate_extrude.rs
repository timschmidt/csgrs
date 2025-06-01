//! Scene L: Demonstrate rotate_extrude (360 deg) on a square

use csgrs::CSG;
use std::fs;

fn main() {
    let small_square = CSG::<()>::square(1.0, 1.0, None).translate(2.0, 0.0, 0.0);
    let revolve = small_square.rotate_extrude(360.0, 24);
    let _ = fs::write("stl/scene_square_revolve_360.stl", revolve.to_stl_ascii("scene_square_revolve_360"));
}
