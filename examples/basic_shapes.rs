//! Basic 3D primitives exported as STL files.

use csgrs::mesh::Mesh;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/basic_shapes";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    write_mesh(&Mesh::<()>::cube(2.0, None), "cube");
    write_mesh(&Mesh::<()>::sphere(1.0, 32, 16, None), "sphere");
    write_mesh(&Mesh::<()>::cylinder(1.0, 2.0, 32, None), "cylinder");
    write_mesh(&Mesh::<()>::ellipsoid(2.0, 1.0, 3.0, 32, 16, None), "ellipsoid");
    write_mesh(&Mesh::<()>::torus(2.0, 0.35, 48, 16, None), "torus");
}

fn write_mesh(mesh: &Mesh<()>, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        mesh.to_stl_binary(name).unwrap(),
    )
    .unwrap();
}
