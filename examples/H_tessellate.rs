//! Scene H: Demonstrate tessellate() (forces triangulation)

use csgrs::CSG;
use std::fs;

fn main() {
    let sphere = CSG::<()>::sphere(1.0, 16, 8, None);

    let tri_sphere = sphere.tessellate();
    #[cfg(feature = "stl-io")]
    let _ = fs::write(
        "stl/scene_tessellate_sphere.stl",
        tri_sphere.to_stl_binary("scene_tessellate_sphere").unwrap(),
    );
}
