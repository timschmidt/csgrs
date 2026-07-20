//! Translation, rotation, scale, mirror, and matrix transforms.

use csgrs::{csg::CSG, mesh::Mesh, mesh::plane::Plane};
use hyperlattice::{Matrix4, Real, Vector3};
use std::{fs, path::Path};

const PATH: &str = "stl/examples/transformations";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = Mesh::<()>::cube(r(1.5), ());
    write_mesh(&cube.translate(r(2.0), r(0.0), r(0.0)), "translated");
    write_mesh(&cube.rotate(r(30.0), r(45.0), r(10.0)), "rotated");
    write_mesh(&cube.scale(r(1.0), r(0.5), r(2.0)), "scaled");
    write_mesh(
        &cube.mirror(Plane::from_normal(Vector3::x(), r(0.0))),
        "mirrored_x",
    );

    let transform = Matrix4::affine_translation([r(0.0), r(2.0), r(0.0)])
        * Matrix4::affine_nonuniform_scale([r(0.5), r(1.0), r(1.5)]);
    write_mesh(&cube.transform(&transform), "matrix_transform");
}

fn r(value: f64) -> Real {
    Real::try_from(value).expect("example values must be finite")
}

fn write_mesh(mesh: &Mesh<()>, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        mesh.to_stl_binary(name).unwrap(),
    )
    .unwrap();
}
