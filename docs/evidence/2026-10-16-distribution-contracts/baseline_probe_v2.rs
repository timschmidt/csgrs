use csgrs::solid::SolidExt;
use csgrs::{
    GeometryContext, GeometryOutcome, Real, TriangleMesh, errors::ValidationError, solid,
};
use hypermesh::{Point3, Triangle, Vector3};
use std::{
    error::Error,
    panic::{AssertUnwindSafe, catch_unwind},
    path::Path,
};

fn r(n: i32) -> Real {
    n.into()
}
fn empty(mesh: &TriangleMesh) -> bool {
    mesh.positions.is_empty() && mesh.triangles.is_empty()
}
fn malformed() -> TriangleMesh {
    TriangleMesh::new(vec![Point3::origin()], vec![Triangle::new(0, 1, 0)])
}
fn open() -> TriangleMesh {
    TriangleMesh::new(
        vec![
            Point3::origin(),
            Point3::new(r(1), r(0), r(0)),
            Point3::new(r(0), r(1), r(0)),
        ],
        vec![Triangle::new(0, 1, 2)],
    )
}
fn result(
    name: &str,
    operation: impl FnOnce() -> Result<GeometryOutcome<TriangleMesh>, ValidationError>,
) {
    match catch_unwind(AssertUnwindSafe(operation)) {
        Ok(Ok(outcome)) => println!(
            "{name}: success empty={} triangles={} certainty={:?}",
            empty(&outcome.value),
            outcome.value.triangles.len(),
            outcome.certainty
        ),
        Ok(Err(error)) => println!(
            "{name}: error kind={:?} source={} diagnostic={error:?}",
            error.kind(),
            error.source().is_some()
        ),
        Err(_) => println!("{name}: panic"),
    }
}
fn record(mesh: &TriangleMesh) -> String {
    let points = mesh
        .positions
        .iter()
        .map(|p| format!("[{},{},{}]", p.x.to_json(), p.y.to_json(), p.z.to_json()))
        .collect::<Vec<_>>();
    let triangles = mesh
        .triangles
        .iter()
        .map(|t| format!("[{},{},{}]", t.v0, t.v1, t.v2))
        .collect::<Vec<_>>();
    format!(
        "{{\"positions\":[{}],\"triangles\":[{}]}}\n",
        points.join(","),
        triangles.join(",")
    )
}
fn main() {
    let args = std::env::args().collect::<Vec<_>>();
    let out = Path::new(&args[1]);
    std::fs::create_dir_all(out).unwrap();
    let cube = solid::cube(r(1)).unwrap();
    let strict = GeometryContext::STRICT;
    if let Some(mode) = args.get(2) {
        match mode.as_str() {
            "linear-overflow" => result("linear-byte-overflow", || {
                solid::distribute_linear_with_context(
                    &cube,
                    usize::MAX,
                    Vector3::x(),
                    r(2),
                    &strict,
                )
            }),
            "arc-overflow" => result("arc-byte-overflow", || {
                solid::distribute_arc_with_context(
                    &cube,
                    usize::MAX,
                    r(2),
                    r(0),
                    r(90),
                    &strict,
                )
            }),
            _ => panic!("unknown diagnostic mode"),
        }
        return;
    }
    for (name, mesh) in [
        ("invalid-indices", malformed()),
        ("open", open()),
        ("empty", solid::empty()),
    ] {
        for count in [0, 1, 2] {
            result(&format!("linear-{name}-{count}"), || {
                solid::distribute_linear_with_context(
                    &mesh,
                    count,
                    Vector3::x(),
                    r(3),
                    &strict,
                )
            });
            result(&format!("grid-{name}-{count}"), || {
                solid::distribute_grid_with_context(&mesh, 1, count, r(3), r(3), &strict)
            });
            result(&format!("arc-{name}-{count}"), || {
                solid::distribute_arc_with_context(&mesh, count, r(3), r(0), r(90), &strict)
            });
        }
        println!(
            "strict-wrapper-{name}: linear_empty={} grid_empty={} arc_empty={}",
            empty(&solid::distribute_linear(&mesh, 2, Vector3::x(), r(3))),
            empty(&solid::distribute_grid(&mesh, 1, 2, r(3), r(3))),
            empty(&solid::distribute_arc(&mesh, 2, r(3), r(0), r(90)))
        );
    }
    for count in [0, 1, 2] {
        result(&format!("zero-direction-{count}"), || {
            solid::distribute_linear_with_context(
                &cube,
                count,
                Vector3::zeros(),
                r(1),
                &strict,
            )
        });
    }
    println!(
        "strict-wrapper-zero-direction: empty={}",
        empty(&solid::distribute_linear(&cube, 2, Vector3::zeros(), r(1)))
    );
    result("open-overlapping", || {
        solid::distribute_grid_with_context(&open(), 1, 2, r(0), r(0), &strict)
    });
    let s = Real::e().sin();
    let c = Real::e().cos();
    let z = &s * &s + &c * &c - r(1);
    result("undecided-grid", || {
        solid::distribute_grid_with_context(&cube, 1, 2, r(1) + z.clone(), r(1), &strict)
    });
    println!(
        "strict-wrapper-undecided: empty={}",
        empty(&solid::distribute_grid(&cube, 1, 2, r(1) + z, r(1)))
    );
    result("grid-product-overflow", || {
        solid::distribute_grid_with_context(&cube, usize::MAX, 2, r(2), r(2), &strict)
    });
    result("grid-byte-overflow", || {
        solid::distribute_grid_with_context(&cube, usize::MAX, 1, r(2), r(2), &strict)
    });
    let invalid =
        TriangleMesh::new(vec![Point3::origin()], vec![Triangle::new(usize::MAX, 0, 0)]);
    match catch_unwind(AssertUnwindSafe(|| solid::merge(&[cube.clone(), invalid]))) {
        Ok(mesh) => println!(
            "merge-invalid: success first_index={}",
            mesh.triangles.last().unwrap().v0
        ),
        Err(_) => println!("merge-invalid: panic"),
    }
    for (name, mesh) in [
        (
            "linear-8",
            solid::distribute_linear(&cube, 8, Vector3::x(), r(2)),
        ),
        ("grid-4x4", solid::distribute_grid(&cube, 4, 4, r(2), r(2))),
        (
            "arc-12",
            solid::distribute_arc(&cube, 12, r(10), r(0), r(330)),
        ),
        ("merge-duplicate", solid::merge(&[cube.clone(), cube.clone()])),
        (
            "feature-merge-22",
            solid::merge(
                &(0..22)
                    .map(|index| {
                        cube.translated(r((index % 6) * 3), r((index / 6) * 3), r(0))
                            .unwrap()
                    })
                    .collect::<Vec<_>>(),
            ),
        ),
    ] {
        assert!(!empty(&mesh));
        std::fs::write(out.join(format!("{name}.json")), record(&mesh)).unwrap();
        println!(
            "fixture {name}: positions={} triangles={}",
            mesh.positions.len(),
            mesh.triangles.len()
        );
    }
}
