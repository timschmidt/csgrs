//! Extrude, vector-extrude, revolve, loft, and sweep curves into meshes.
//! Requires `curve,stl-io`. Circular profiles, angular revolution steps and the
//! helix path are finite samples. Helix coordinates are computed in binary64
//! before exact promotion; native operations are relative to those coordinates,
//! not an analytic helix. Outcome receipts are extracted without reporting them.
//! Writes/replaces lossy binary STL files beneath the working directory.

use csgrs::{GeometryContext, TriangleMesh, curve, io::stl::to_stl_binary, solid};
use hyperlattice::{Point3, Real, Vector3};
use std::{fs, path::Path};

/// Relative output directory for the five modeled mesh artifacts.
const PATH: &str = "stl/examples/extrude";

/// Builds two extrusions, a segmented full revolution, a sampled-path sweep and a loft.
/// Uses strict operation contexts where supplied; construction/operation/IO errors
/// panic and leave any previously written files in place. No shape oracle is asserted.
fn main() {
    fs::create_dir_all(PATH).unwrap();

    let square = curve::square(1.5).expect("valid planar fixture");
    let circle = curve::circle(0.5, 32).expect("valid planar fixture");
    let star = curve::star(5, 1.2, 0.45).expect("valid sampled profile fixture");

    write_mesh(
        &curve::try_extrude(&square, r(1.0), GeometryContext::STRICT)
            .unwrap()
            .into_value(),
        "square_extrude",
    );
    write_mesh(
        &curve::try_extrude_vector(&star, v3(0.4, 0.25, 1.0), GeometryContext::STRICT)
            .unwrap()
            .into_value(),
        "star_vector_extrude",
    );
    // Translate the sampled circle away from the Y axis so this full turn
    // produces a mesh of a torus.
    let profile = curve::translated(&circle, 1.5, 0.0).expect("valid planar transform");
    let revolved = curve::revolve(&profile, r(360.0), 48, GeometryContext::STRICT)
        .unwrap()
        .into_value();
    write_mesh(&revolved, "circle_revolve");

    let path = (0..40)
        .map(|i| {
            let t = i as f64 * 0.15;
            p3(t.cos() * 0.6, t.sin() * 0.6, i as f64 * 0.04)
        })
        .collect::<Vec<_>>();
    write_mesh(
        &curve::try_sweep(
            &curve::circle(0.08, 12).expect("valid planar fixture"),
            &path,
            GeometryContext::STRICT,
        )
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

/// Promotes a finite binary64 value exactly; nonfinite input panics.
fn r(value: f64) -> Real {
    Real::try_from(value).expect("example values must be finite")
}

/// Constructs a native point from three finite binary64 coordinates without decimal reinterpretation.
fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

/// Constructs a native extrusion vector from three exactly promoted finite floats.
fn v3(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::from_xyz(r(x), r(y), r(z))
}

/// Replaces one named binary STL file, panicking on export/write failure.
/// Does not preserve exact source geometry or operation receipts in the artifact.
fn write_mesh(mesh: &TriangleMesh, name: &str) {
    fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        to_stl_binary(mesh, name).unwrap(),
    )
    .unwrap();
}
