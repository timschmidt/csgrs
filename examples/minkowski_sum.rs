//! Regularized Minkowski sum of a cube and a sampled sphere mesh.
//! Requires `stl-io`. The rounded appearance is relative to the finite sphere
//! mesh, not an analytic-ball offset. Writes lossy binary STL beneath the working
//! directory and replaces any existing artifact with the same name.

use csgrs::{TriangleMesh, io::stl::to_stl_binary, solid};
use std::{fs, path::Path};

/// Relative output directory for the represented-set sum artifact.
const PATH: &str = "stl/examples/minkowski_sum";

/// Sums a 1.5-unit cube and a radius-0.4 sampled sphere using the strict wrapper.
/// Construction, sum, export and IO failures panic; no independent sum oracle is asserted.
fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = solid::cube(1.5).expect("valid primitive fixture");
    let sphere = solid::sphere(0.4, 16, 8).expect("valid primitive fixture");
    write_mesh(&solid::minkowski_sum(&cube, &sphere).unwrap(), "rounded_cube");
}

/// Replaces one named binary STL artifact, panicking on export or write failure.
fn write_mesh(mesh: &TriangleMesh, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        to_stl_binary(mesh, name).unwrap(),
    )
    .unwrap();
}
