//! Convex hull over a boolean model.

use csgrs::{
    TriangleMesh,
    io::stl::to_stl_binary,
    solid::{self, SolidExt},
};
use std::{fs, path::Path};

const PATH: &str = "stl/examples/convex_hull";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = solid::rotate(
        &solid::cube(2.0)
            .expect("valid primitive fixture")
            .translated(0.8, 0.0, 0.0)
            .expect("valid transform fixture"),
        0.0,
        35.0,
        0.0,
    )
    .expect("valid transform fixture");
    let sphere = solid::sphere(1.1, 32, 16)
        .expect("valid primitive fixture")
        .translated(-0.5, 0.0, 0.0)
        .expect("valid transform fixture");
    let union = cube.try_union(&sphere).unwrap();

    write_mesh(&union, "union");
    write_mesh(&solid::convex_hull(&union).unwrap(), "convex_hull");
}

fn write_mesh(mesh: &TriangleMesh, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        to_stl_binary(mesh, name).unwrap(),
    )
    .unwrap();
}
