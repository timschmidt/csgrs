//! This example shows Convex hull usage

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/convex_hull";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    let cube = CSG::cube(2.0, 2.0, 2.0, None)
        .translate(1.0, 0.0, 0.0)
        .rotate(0.0, 45.0, 0.0)
        .scale(1.0, 0.5, 2.0);
    let sphere = CSG::sphere(1.0, 16, 8, None); // center=(0,0,0), radius=1, slices=16, stacks=8, no metadata   

    let union_shape = cube.union(&sphere);
    write_example(&union_shape, "union_cube_sphere");

    // 4) Convex hull
    #[cfg(feature = "chull-io")]
    let hull_of_union = union_shape.convex_hull();
    #[cfg(feature = "chull-io")]
    write_example(&hull_of_union, "hull_union");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap()
    );
}
