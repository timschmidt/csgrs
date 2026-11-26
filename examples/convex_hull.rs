//! Convex hull over a boolean model.
//! Requires `stl-io`. Unites a translated/rotated cube with a translated sampled
//! sphere, then takes the hull of the union's stored positions. Exports both as
//! lossy binary STL files beneath the working directory, replacing existing files.

use csgrs::{
    TriangleMesh,
    io::stl::to_stl_binary,
    solid::{self, SolidExt},
};
use std::{fs, path::Path};

/// Relative output directory for the union and its convex hull.
const PATH: &str = "stl/examples/convex_hull";

/// Builds the strict Boolean result and native position hull, exporting in that order.
/// Construction/operation/IO failures panic; no independent hull or enclosure oracle
/// is asserted, and the first artifact remains if hull processing later fails.
fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = solid::rotate(
        &solid::cube(2.0)
            .expect("valid primitive fixture")
            .translated(0.8, 0.0, 0.0)
            .expect("valid transform fixture"),
        0.0,
        35.0,
        0.0,
    )
    .expect("valid transform fixture");
    let sphere = solid::sphere(1.1, 32, 16)
        .expect("valid primitive fixture")
        .translated(-0.5, 0.0, 0.0)
        .expect("valid transform fixture");
    let union = cube.try_union(&sphere).unwrap();

    write_mesh(&union, "union");
    write_mesh(&solid::convex_hull(&union).unwrap(), "convex_hull");
}

/// Replaces one named binary STL file, panicking on export or write failure.
/// Does not retain exact-coordinate or certainty evidence in the artifact.
fn write_mesh(mesh: &TriangleMesh, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        to_stl_binary(mesh, name).unwrap(),
    )
    .unwrap();
}
