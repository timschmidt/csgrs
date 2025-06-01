//! Scene I: Demonstrate slice(plane) – slice a cube at z=0

use csgrs::{CSG, Plane};
use nalgebra::Vector3;
use std::fs;

fn main() {
    let cube = CSG::<()>::cube(2.0, 2.0, 2.0, None);

    let plane_z = Plane::from_normal(Vector3::z(), 0.5);
    let sliced_polygons = cube.slice(plane_z);
    let _ = fs::write("stl/scene_sliced_cube.stl", cube.to_stl_ascii("sliced_cube"));
    // Save cross-section as well
    let _ = fs::write(
        "stl/scene_sliced_cube_section.stl",
        sliced_polygons.to_stl_ascii("sliced_cube_section"),
    );
}
