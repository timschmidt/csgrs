//! Extrude, vector-extrude, revolve, loft, and sweep curves into meshes.

use csgrs::{
    GeometryContext, TriangleMesh, curve,
    io::stl::to_stl_binary,
    solid::{self, SolidExt},
};
use hyperlattice::{Point3, Real, Vector3};
use std::{fs, path::Path};

const PATH: &str = "stl/examples/extrude";

fn main() {
    fs::create_dir_all(PATH).unwrap();

    let square = curve::square(r(1.5));
    let circle = curve::circle(r(0.5), 32);
    let star = curve::star(5, r(1.2), r(0.45));

    write_mesh(
        &curve::try_extrude(&square, r(1.0), &GeometryContext::STRICT)
            .unwrap()
            .into_value(),
        "square_extrude",
    );
    write_mesh(
        &curve::try_extrude_vector(&star, v3(0.4, 0.25, 1.0), &GeometryContext::STRICT)
            .unwrap()
            .into_value(),
        "star_vector_extrude",
    );
    let revolved = curve::revolve(&circle, r(360.0), 48, &GeometryContext::STRICT)
        .unwrap()
        .into_value()
        .translated(r(1.5), r(0.0), r(0.0));
    write_mesh(&revolved, "circle_revolve");

    let path = (0..40)
        .map(|i| {
            let t = i as f64 * 0.15;
            p3(t.cos() * 0.6, t.sin() * 0.6, i as f64 * 0.04)
        })
        .collect::<Vec<_>>();
    write_mesh(
        &curve::try_sweep(&curve::circle(r(0.08), 12), &path, &GeometryContext::STRICT)
            .unwrap()
            .into_value(),
        "sweep",
    );

    let bottom = vec![
        p3(-0.5, -0.5, 0.0),
        p3(0.5, -0.5, 0.0),
        p3(0.5, 0.5, 0.0),
        p3(-0.5, 0.5, 0.0),
    ];
    let top = vec![
        p3(-0.25, -0.25, 1.0),
        p3(0.25, -0.25, 1.0),
        p3(0.25, 0.25, 1.0),
        p3(-0.25, 0.25, 1.0),
    ];
    write_mesh(&solid::loft(&[bottom, top]).unwrap(), "loft");
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

fn write_mesh(mesh: &TriangleMesh, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        to_stl_binary(mesh, name).unwrap(),
    )
    .unwrap();
}
