//! Convex hull over a boolean model.

use csgrs::{csg::CSG, mesh::Mesh};
use std::{fs, path::Path};

const PATH: &str = "stl/examples/convex_hull";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = Mesh::<()>::cube(2.0, ())
        .translate(0.8, 0.0, 0.0)
        .rotate(0.0, 35.0, 0.0);
    let sphere = Mesh::<()>::sphere(1.1, 32, 16, ()).translate(-0.5, 0.0, 0.0);
    let union = cube.union(&sphere);

    write_mesh(&union, "union");
    write_mesh(&union.convex_hull(), "convex_hull");
}

fn write_mesh(mesh: &Mesh<()>, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        mesh.to_stl_binary(name).unwrap(),
    )
    .unwrap();
}
