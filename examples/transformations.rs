//! Translation, rotation, scale, mirror, and matrix transforms.

use csgrs::{csg::CSG, mesh::Mesh, mesh::plane::Plane};
use nalgebra::{Matrix4, Vector3};
use std::{fs, path::Path};

const PATH: &str = "stl/examples/transformations";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = Mesh::<()>::cube(1.5, None);
    write_mesh(&cube.translate(2.0, 0.0, 0.0), "translated");
    write_mesh(&cube.rotate(30.0, 45.0, 10.0), "rotated");
    write_mesh(&cube.scale(1.0, 0.5, 2.0), "scaled");
    write_mesh(
        &cube.mirror(Plane::from_normal(Vector3::x(), 0.0)),
        "mirrored_x",
    );

    let transform = Matrix4::new_translation(&Vector3::new(0.0, 2.0, 0.0))
        * Matrix4::new_nonuniform_scaling(&Vector3::new(0.5, 1.0, 1.5));
    write_mesh(&cube.transform(&transform), "matrix_transform");
}

fn write_mesh(mesh: &Mesh<()>, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        mesh.to_stl_binary(name).unwrap(),
    )
    .unwrap();
}
