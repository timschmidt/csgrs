//! Scene O: Demonstrate transform() with an arbitrary affine matrix

use csgrs::csg::CSG;
use std::fs;

fn main() {
    use nalgebra::{Matrix4, Translation3};
    let xlate = Translation3::new(2.0, 0.0, 1.0).to_homogeneous();
    // Scale matrix
    let scale_mat = Matrix4::new_scaling(0.5);
    // Combine
    let transform_mat = xlate * scale_mat;
    let shape = CSG::<()>::cube(1.0, 1.0, 1.0, None).transform(&transform_mat);
    let _ = fs::write(
        "stl/scene_transform_cube.stl",
        shape.to_stl_ascii("scene_transform_cube"),
    );
}
