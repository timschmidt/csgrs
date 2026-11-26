//! Translation, rotation, scale, mirror, and matrix transforms.
//! Requires `stl-io`. Every demonstration starts from the same cube; results
//! are separate files, not a sequential modeling chain. Binary STL is a lossy
//! terminal view, with existing files replaced beneath the working directory.

use csgrs::{
    TriangleMesh,
    io::stl::to_stl_binary,
    solid::{self, SolidExt},
};
use hyperlattice::{Matrix4, Real};
use std::{fs, path::Path};

/// Relative output directory for the five transformed-cube artifacts.
const PATH: &str = "stl/examples/transformations";

/// Applies translation, degree-valued rotation, nonuniform scale, X reflection
/// and scale-then-translation matrix composition to a cube, exporting each result.
/// Operation/export/IO failures panic; transformed geometry is not independently asserted.
fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = solid::cube(1.5).expect("valid primitive fixture");
    write_mesh(
        &cube
            .translated(2.0, 0.0, 0.0)
            .expect("valid transform fixture"),
        "translated",
    );
    write_mesh(
        &solid::rotate(&cube, 30.0, 45.0, 10.0).expect("valid transform fixture"),
        "rotated",
    );
    write_mesh(
        &solid::scale(&cube, 1.0, 0.5, 2.0).expect("valid transform fixture"),
        "scaled",
    );
    write_mesh(
        &solid::scale(&cube, -1.0, 1.0, 1.0).expect("valid transform fixture"),
        "mirrored_x",
    );

    let transform = Matrix4::affine_translation([r(0.0), r(2.0), r(0.0)])
        * Matrix4::affine_nonuniform_scale([r(0.5), r(1.0), r(1.5)]);
    write_mesh(
        &cube.transformed(&transform).expect("valid transform fixture"),
        "matrix_transform",
    );
}

/// Converts a finite float to its exact binary value; NaN/infinity panic here.
fn r(value: f64) -> Real {
    Real::try_from(value).expect("example values must be finite")
}

/// Replaces a named binary STL artifact, panicking on export/write failure.
/// Earlier files remain if a later demonstration fails.
fn write_mesh(mesh: &TriangleMesh, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        to_stl_binary(mesh, name).unwrap(),
    )
    .unwrap();
}
