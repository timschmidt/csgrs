//! Basic 3D primitives exported as STL files.

use csgrs::mesh::Mesh;
use hyperlattice::Real;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/basic_shapes";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    write_mesh(&Mesh::<()>::cube(r(2.0), ()), "cube");
    write_mesh(&Mesh::<()>::sphere(r(1.0), 32, 16, ()), "sphere");
    write_mesh(&Mesh::<()>::cylinder(r(1.0), r(2.0), 32, ()), "cylinder");
    write_mesh(
        &Mesh::<()>::ellipsoid(r(2.0), r(1.0), r(3.0), 32, 16, ()),
        "ellipsoid",
    );
    write_mesh(&Mesh::<()>::torus(r(2.0), r(0.35), 48, 16, ()), "torus");
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
