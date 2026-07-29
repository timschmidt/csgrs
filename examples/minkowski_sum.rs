//! Minkowski sum of a cube and sphere.

use csgrs::{TriangleMesh, io::stl::to_stl_binary, solid};
use hyperlattice::Real;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/minkowski_sum";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = solid::cube(r(1.5));
    let sphere = solid::sphere(r(0.4), 16, 8);
    write_mesh(&solid::minkowski_sum(&cube, &sphere).unwrap(), "rounded_cube");
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
