//! Basic 3D primitives exported as STL files.

use csgrs::{TriangleMesh, io::stl::to_stl_binary, solid};
use std::{fs, path::Path};

const PATH: &str = "stl/examples/basic_shapes";

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

fn write_mesh(mesh: &TriangleMesh, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        to_stl_binary(mesh, name).unwrap(),
    )
    .unwrap();
}
