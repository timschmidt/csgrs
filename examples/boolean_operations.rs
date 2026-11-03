//! Boolean operations between overlapping 3D primitives.

use csgrs::{
    TriangleMesh,
    io::stl::to_stl_binary,
    solid::{self, SolidExt},
};
use std::{fs, path::Path};

const PATH: &str = "stl/examples/boolean_operations";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = solid::cube(2.0).expect("valid primitive fixture");
    let sphere = solid::sphere(1.35, 32, 16)
        .expect("valid primitive fixture")
        .translated(0.55, 0.25, 0.2)
        .expect("valid transform fixture");

    write_mesh(&cube.try_union(&sphere).unwrap(), "union");
    write_mesh(&cube.try_difference(&sphere).unwrap(), "difference");
    write_mesh(&cube.try_intersection(&sphere).unwrap(), "intersection");
    write_mesh(&cube.try_xor(&sphere).unwrap(), "xor");
}

fn write_mesh(mesh: &TriangleMesh, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        to_stl_binary(mesh, name).unwrap(),
    )
    .unwrap();
}
