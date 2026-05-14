//! Minkowski sum of a cube and sphere.

use csgrs::mesh::Mesh;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/minkowski_sum";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = Mesh::<()>::cube(1.5, None);
    let sphere = Mesh::<()>::sphere(0.4, 16, 8, None);
    write_mesh(&cube.minkowski_sum(&sphere), "rounded_cube");
}

fn write_mesh(mesh: &Mesh<()>, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        mesh.to_stl_binary(name).unwrap(),
    )
    .unwrap();
}
