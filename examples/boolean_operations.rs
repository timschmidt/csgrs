//! Boolean operations between overlapping 3D primitives.

use csgrs::{csg::CSG, mesh::Mesh};
use hyperlattice::Real;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/boolean_operations";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = Mesh::<()>::cube(r(2.0), ());
    let sphere = Mesh::<()>::sphere(r(1.35), 32, 16, ()).translate(r(0.55), r(0.25), r(0.2));

    write_mesh(&cube.union(&sphere), "union");
    write_mesh(&cube.difference(&sphere), "difference");
    write_mesh(&cube.intersection(&sphere), "intersection");
    write_mesh(&cube.xor(&sphere), "xor");
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
