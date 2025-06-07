//! Scene N: Demonstrate scale()

use csgrs::csg::CSG;
use std::fs;

fn main() {
    let sphere = CSG::<()>::sphere(1.0, 16, 8, None);

    let scaled = sphere.scale(1.0, 2.0, 0.5);
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        "stl/scene_scaled_sphere.stl",
        scaled.to_stl_binary("scene_scaled_sphere").unwrap(),
    );
}
