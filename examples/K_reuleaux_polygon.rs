//! Scene K: Demonstrate reuleaux_polygon with a typical triangle shape
//! (already used sides=4 in main examples, so let's do sides=3 here)

use csgrs::CSG;
use std::fs;

fn main() {
    let reuleaux_tri = CSG::<()>::reuleaux(3, 2.0, 16, None).extrude(0.1);
    let _ = fs::write("stl/scene_reuleaux_triangle.stl", reuleaux_tri.to_stl_ascii("scene_reuleaux_triangle"));
}
