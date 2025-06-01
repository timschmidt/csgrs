//! Scene A: Demonstrate a right_triangle(width=2, height=1)

use csgrs::CSG;
use std::fs;
use nalgebra::{Point3, Vector3};

fn main() {
    let tri_2d = CSG::right_triangle(2.0, 1.0, None);
    // A tiny arrow pointing from the right-angle corner outward:
    let arrow = CSG::arrow(
        Point3::new(0.0, 0.0, 0.1), // at corner
        Vector3::new(0.5, 0.0, 0.0), 
        8,
        true,
        None::<()>,
    )
    .scale(0.05, 0.05, 0.05);
    let scene = tri_2d.extrude(0.1).union(&arrow);
    let _ = fs::write("stl/scene_right_triangle.stl", scene.to_stl_ascii("scene_right_triangle"));
}
