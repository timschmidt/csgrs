//! Boolean operations between overlapping 3D primitives.

use csgrs::{
    TriangleMesh,
    io::stl::to_stl_binary,
    solid::{self, SolidExt},
};
use hyperlattice::Real;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/boolean_operations";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = solid::cube(r(2.0));
    let sphere = solid::sphere(r(1.35), 32, 16).translated(r(0.55), r(0.25), r(0.2));

    write_mesh(&cube.try_union(&sphere).unwrap(), "union");
    write_mesh(&cube.try_difference(&sphere).unwrap(), "difference");
    write_mesh(&cube.try_intersection(&sphere).unwrap(), "intersection");
    write_mesh(&cube.try_xor(&sphere).unwrap(), "xor");
}

fn r(value: f64) -> Real {
    Real::try_from(value).expect("example values must be finite")
}

fn write_mesh(mesh: &TriangleMesh, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        to_stl_binary(mesh, name).unwrap(),
    )
    .unwrap();
}
