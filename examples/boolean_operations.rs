//! Boolean operations between overlapping 3D primitives.
//! Requires `stl-io`. Uses a sampled sphere and exact binary values of the float
//! inputs; success describes the represented meshes, not an analytic sphere.
//! Writes four lossy binary STL artifacts beneath the working directory.

use csgrs::{
    TriangleMesh,
    io::stl::to_stl_binary,
    solid::{self, SolidExt},
};
use std::{fs, path::Path};

/// Relative output directory for union, difference, intersection and XOR artifacts.
const PATH: &str = "stl/examples/boolean_operations";

/// Evaluates four separate strict Boolean calls on the same cube/sphere pair.
/// Panics on construction, operation or IO errors; no independent set/receipt
/// assertions are made. Previously written artifacts survive a later failure.
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

/// Serializes binary STL and replaces its named file; export/write failures panic.
/// Does not preserve the exact source or its receipts in the file.
fn write_mesh(mesh: &TriangleMesh, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        to_stl_binary(mesh, name).unwrap(),
    )
    .unwrap();
}
