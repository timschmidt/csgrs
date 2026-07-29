//! Convex hull over a boolean model.

use csgrs::{
    TriangleMesh,
    io::stl::to_stl_binary,
    solid::{self, SolidExt},
};
use hyperlattice::Real;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/convex_hull";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = solid::rotate(
        &solid::cube(r(2.0)).translated(r(0.8), r(0.0), r(0.0)),
        r(0.0),
        r(35.0),
        r(0.0),
    );
    let sphere = solid::sphere(r(1.1), 32, 16).translated(r(-0.5), r(0.0), r(0.0));
    let union = cube.try_union(&sphere).unwrap();

    write_mesh(&union, "union");
    write_mesh(&solid::convex_hull(&union).unwrap(), "convex_hull");
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
