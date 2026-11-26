//! Basic 3D primitives exported as STL files.
//! Requires the manifest's `curve,stl-io` features. Sphere, cylinder, ellipsoid
//! and torus outputs are finite meshes, not analytic surfaces. Finite float
//! arguments enter as their exact binary values; STL is a lossy terminal export.
//! Existing files are replaced, and a later failure leaves earlier files in place.

use csgrs::{TriangleMesh, io::stl::to_stl_binary, solid};
use std::{fs, path::Path};

/// Output directory relative to the process working directory, created by `main`.
const PATH: &str = "stl/examples/basic_shapes";

/// Constructs five primitive meshes with fixed sample counts and exports each.
/// Panics on construction, directory, export or write failure; no shape oracle is asserted.
fn main() {
    fs::create_dir_all(PATH).unwrap();

    write_mesh(&solid::cube(2.0).expect("valid primitive fixture"), "cube");
    write_mesh(
        &solid::sphere(1.0, 32, 16).expect("valid primitive fixture"),
        "sphere",
    );
    write_mesh(
        &solid::cylinder(1.0, 2.0, 32).expect("valid primitive fixture"),
        "cylinder",
    );
    write_mesh(
        &solid::ellipsoid(2.0, 1.0, 3.0, 32, 16).expect("valid primitive fixture"),
        "ellipsoid",
    );
    write_mesh(
        &solid::torus(2.0, 0.35, 48, 16).expect("valid primitive fixture"),
        "torus",
    );
}

/// Replaces one named binary STL file under `PATH`, panicking on export or IO failure.
/// Does not retain export evidence or reimport/validate the written artifact.
fn write_mesh(mesh: &TriangleMesh, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        to_stl_binary(mesh, name).unwrap(),
    )
    .unwrap();
}
