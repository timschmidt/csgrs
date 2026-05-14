//! Boolean operations between overlapping 3D primitives.

use csgrs::{csg::CSG, mesh::Mesh};
use std::{fs, path::Path};

const PATH: &str = "stl/examples/boolean_operations";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = Mesh::<()>::cube(2.0, ());
    let sphere = Mesh::<()>::sphere(1.35, 32, 16, ()).translate(0.55, 0.25, 0.2);

    write_mesh(&cube.union(&sphere), "union");
    write_mesh(&cube.difference(&sphere), "difference");
    write_mesh(&cube.intersection(&sphere), "intersection");
    write_mesh(&cube.xor(&sphere), "xor");
}

fn write_mesh(mesh: &Mesh<()>, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        mesh.to_stl_binary(name).unwrap(),
    )
    .unwrap();
}
