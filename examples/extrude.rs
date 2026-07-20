//! Extrude, vector-extrude, revolve, loft, and sweep sketches into meshes.

use csgrs::{
    csg::CSG,
    mesh::{Mesh, Polygon},
    sketch::Profile,
    vertex::Vertex,
};
use hyperlattice::{Point3, Real, Vector3};
use std::{fs, path::Path};

const PATH: &str = "stl/examples/extrude";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let square = Profile::square(r(1.5));
    let circle = Profile::circle(r(0.5), 32).translate(r(1.5), r(0.0), r(0.0));
    let star = Profile::star(5, r(1.2), r(0.45));

    write_mesh(&square.extrude(r(1.0), ()), "square_extrude");
    write_mesh(
        &star.extrude_vector(v3(0.4, 0.25, 1.0), ()),
        "star_vector_extrude",
    );
    write_mesh(&circle.revolve(r(360.0), 48, ()).unwrap(), "circle_revolve");

    let path = (0..40)
        .map(|i| {
            let t = i as f64 * 0.15;
            p3(t.cos() * 0.6, t.sin() * 0.6, i as f64 * 0.04)
        })
        .collect::<Vec<_>>();
    write_mesh(&Profile::circle(r(0.08), 12).sweep(&path, ()), "sweep");

    let bottom = Polygon::new(
        vec![
            Vertex::new(p3(-0.5, -0.5, 0.0), Vector3::z()),
            Vertex::new(p3(0.5, -0.5, 0.0), Vector3::z()),
            Vertex::new(p3(0.5, 0.5, 0.0), Vector3::z()),
            Vertex::new(p3(-0.5, 0.5, 0.0), Vector3::z()),
        ],
        (),
    );
    let top = Polygon::new(
        vec![
            Vertex::new(p3(-0.25, -0.25, 1.0), Vector3::z()),
            Vertex::new(p3(0.25, -0.25, 1.0), Vector3::z()),
            Vertex::new(p3(0.25, 0.25, 1.0), Vector3::z()),
            Vertex::new(p3(-0.25, 0.25, 1.0), Vector3::z()),
        ],
        (),
    );
    write_mesh(&Profile::loft(&[bottom, top]).unwrap(), "loft");
}

fn r(value: f64) -> Real {
    Real::try_from(value).expect("example values must be finite")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn v3(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::from_xyz(r(x), r(y), r(z))
}

fn write_mesh(mesh: &Mesh<()>, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        mesh.to_stl_binary(name).unwrap(),
    )
    .unwrap();
}
