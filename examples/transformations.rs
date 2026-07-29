//! Translation, rotation, scale, mirror, and matrix transforms.

use csgrs::{
    TriangleMesh,
    io::stl::to_stl_binary,
    solid::{self, SolidExt},
};
use hyperlattice::{Matrix4, Real};
use std::{fs, path::Path};

const PATH: &str = "stl/examples/transformations";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = solid::cube(r(1.5));
    write_mesh(&cube.translated(r(2.0), r(0.0), r(0.0)), "translated");
    write_mesh(&solid::rotate(&cube, r(30.0), r(45.0), r(10.0)), "rotated");
    write_mesh(&solid::scale(&cube, r(1.0), r(0.5), r(2.0)), "scaled");
    write_mesh(&solid::scale(&cube, r(-1.0), r(1.0), r(1.0)), "mirrored_x");

    let transform = Matrix4::affine_translation([r(0.0), r(2.0), r(0.0)])
        * Matrix4::affine_nonuniform_scale([r(0.5), r(1.0), r(1.5)]);
    write_mesh(&cube.transformed(&transform), "matrix_transform");
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
