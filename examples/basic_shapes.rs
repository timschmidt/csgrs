//! Basic 3D primitives exported as STL files.

use csgrs::{TriangleMesh, io::stl::to_stl_binary, solid};
use hyperlattice::Real;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/basic_shapes";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    write_mesh(&solid::cube(r(2.0)), "cube");
    write_mesh(&solid::sphere(r(1.0), 32, 16), "sphere");
    write_mesh(&solid::cylinder(r(1.0), r(2.0), 32), "cylinder");
    write_mesh(&solid::ellipsoid(r(2.0), r(1.0), r(3.0), 32, 16), "ellipsoid");
    write_mesh(&solid::torus(r(2.0), r(0.35), 48, 16), "torus");
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
