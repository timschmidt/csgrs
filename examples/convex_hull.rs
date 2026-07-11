//! Convex hull over a boolean model.

use csgrs::{csg::CSG, mesh::Mesh};
use hyperlattice::Real;
use std::{fs, path::Path};

const PATH: &str = "stl/examples/convex_hull";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let cube = Mesh::<()>::cube(r(2.0), ())
        .translate(r(0.8), r(0.0), r(0.0))
        .rotate(r(0.0), r(35.0), r(0.0));
    let sphere = Mesh::<()>::sphere(r(1.1), 32, 16, ()).translate(r(-0.5), r(0.0), r(0.0));
    let union = cube.union(&sphere);

    write_mesh(&union, "union");
    write_mesh(&union.convex_hull(()), "convex_hull");
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
