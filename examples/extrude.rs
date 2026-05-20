//! Extrude, vector-extrude, revolve, loft, and sweep sketches into meshes.

use csgrs::{csg::CSG, mesh::Mesh, polygon::Polygon, sketch::Profile, vertex::Vertex};
use nalgebra::{Point3, Vector3};
use std::{fs, path::Path};

const PATH: &str = "stl/examples/extrude";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let square = Profile::<()>::square(1.5, ());
    let circle = Profile::<()>::circle(0.5, 32, ()).translate(1.5, 0.0, 0.0);
    let star = Profile::<()>::star(5, 1.2, 0.45, ());

    write_mesh(&square.extrude(1.0), "square_extrude");
    write_mesh(
        &star.extrude_vector(Vector3::new(0.4, 0.25, 1.0)),
        "star_vector_extrude",
    );
    write_mesh(&circle.revolve(360.0, 48).unwrap(), "circle_revolve");

    let path = (0..40)
        .map(|i| {
            let t = i as f64 * 0.15;
            Point3::new(t.cos() * 0.6, t.sin() * 0.6, i as f64 * 0.04)
        })
        .collect::<Vec<_>>();
    write_mesh(&Profile::<()>::circle(0.08, 12, ()).sweep(&path), "sweep");

    let bottom = Polygon::new(
        vec![
            Vertex::new(Point3::new(-0.5, -0.5, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.5, -0.5, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.5, 0.5, 0.0), Vector3::z()),
            Vertex::new(Point3::new(-0.5, 0.5, 0.0), Vector3::z()),
        ],
        (),
    );
    let top = Polygon::new(
        vec![
            Vertex::new(Point3::new(-0.25, -0.25, 1.0), Vector3::z()),
            Vertex::new(Point3::new(0.25, -0.25, 1.0), Vector3::z()),
            Vertex::new(Point3::new(0.25, 0.25, 1.0), Vector3::z()),
            Vertex::new(Point3::new(-0.25, 0.25, 1.0), Vector3::z()),
        ],
        (),
    );
    write_mesh(&Profile::<()>::loft(&bottom, &top, true).unwrap(), "loft");
}

fn write_mesh(mesh: &Mesh<()>, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        mesh.to_stl_binary(name).unwrap(),
    )
    .unwrap();
}
