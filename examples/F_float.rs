//! Scene F: Demonstrate float() (moves shape so bottom is at z=0)

use csgrs::CSG;
use std::fs;

fn main() {
    let sphere_for_float = CSG::<()>::sphere(1.0, 16, 8, None).translate(0.0, 0.0, -1.5);
    let floated = sphere_for_float.float();
    let _ = fs::write("stl/scene_sphere_before_float.stl", sphere_for_float.to_stl_ascii("scene_sphere_before_float"));
    let _ = fs::write("stl/scene_sphere_floated.stl", floated.to_stl_ascii("scene_sphere_floated"));
}
