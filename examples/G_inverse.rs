//! Scene G: Demonstrate inverse() (flips inside/outside)

use csgrs::CSG;
use std::fs;

fn main() {
    let sphere = CSG::<()>::sphere(1.0, 16, 8, None);

    // Hard to visualize in STL, but let's do it anyway
    let inv_sphere = sphere.inverse();
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        "stl/scene_inverse_sphere.stl",
        inv_sphere.to_stl_binary("scene_inverse_sphere").unwrap(),
    );
}
