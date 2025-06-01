//! This example shows computing the Minkowski sum: self ⊕ other
//! This uses a Naive approach: Take every vertex in self, add it to every vertex in other,
//! then compute the convex hull of all resulting points.

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/minkowski";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    let cube = CSG::cube(2.0, 2.0, 2.0, None);
    let sphere = CSG::sphere(1.0, 16, 8, None); // center=(0,0,0), radius=1, slices=16, stacks=8

    // 5) Minkowski sum
    #[cfg(feature = "chull-io")]
    let minkowski = cube.minkowski_sum(&sphere);
    #[cfg(feature = "chull-io")]
    write_example(&minkowski, "minkowski_cube_sphere");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
