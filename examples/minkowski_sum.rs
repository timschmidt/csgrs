//! Minkowski sum of a cube and sphere.

use csgrs::mesh::Mesh;
use hyperlattice::Real;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/minkowski_sum";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = Mesh::<()>::cube(r(1.5), ());
    let sphere = Mesh::<()>::sphere(r(0.4), 16, 8, ());
    write_mesh(&cube.minkowski_sum(&sphere), "rounded_cube");
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
