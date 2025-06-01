//! This example shows ray intersection by printing the results)

use csgrs::csg::CSG;
use nalgebra::{Point3, Vector3};

const PATH: &str = "stl/ray_intersection";

fn main() {
    let cube: CSG<()> = CSG::cube(2.0, 2.0, 2.0, None);

    // 11) Ray intersection demo (just printing the results)
    let ray_origin = Point3::new(0.0, 0.0, -5.0);
    let ray_dir = Vector3::new(0.0, 0.0, 1.0); // pointing along +Z
    let hits = cube.ray_intersections(&ray_origin, &ray_dir);
    println!("Ray hits on the cube: {:?}", hits);
}
