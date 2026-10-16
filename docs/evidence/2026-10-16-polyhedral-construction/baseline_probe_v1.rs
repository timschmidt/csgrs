use csgrs::{
    GeometryContext, GeometryOutcome, Real, TriangleMesh,
    errors::ValidationError,
    solid::{self, SolidExt},
};
use hypermesh::{Point3, Triangle};
use std::{error::Error, path::Path};

fn r(n: i32) -> Real {
    n.into()
}
fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}
fn ring(xy: &[(i32, i32)], z: i32) -> Vec<Point3> {
    xy.iter().map(|&(x, y)| point(x, y, z)).collect()
}
fn zero() -> Real {
    let s = Real::e().sin();
    let c = Real::e().cos();
    &s * &s + &c * &c - r(1)
}
fn show(
    name: &str,
    context: &GeometryContext,
    value: Result<GeometryOutcome<TriangleMesh>, ValidationError>,
) -> Option<TriangleMesh> {
    match value {
        Err(error) => {
            println!(
                "{name}: error kind={:?} source={} diagnostic={error:?}",
                error.kind(),
                error.source().is_some()
            );
            None
        },
        Ok(outcome) => {
            let mesh = outcome.value;
            let six_volume = mesh.triangles.iter().fold(r(0), |sum, t| {
                let [a, b, c] = t.indices().map(|i| mesh.positions[i].to_vector());
                sum + a.dot(&b.cross(&c))
            });
            let twice_xy_area = mesh.triangles.iter().fold(r(0), |sum, t| {
                let [a, b, c] = t.indices().map(|i| &mesh.positions[i]);
                sum + &(b - a).cross(&(c - a)).0[2]
            });
            println!(
                "{name}: success positions={} triangles={} certainty={:?} six_volume={:?} twice_xy_area={:?} nondegenerate={:?} pwn={:?}",
                mesh.positions.len(),
                mesh.triangles.len(),
                outcome.certainty,
                six_volume.exact_rational_normal_form(),
                twice_xy_area.exact_rational_normal_form(),
                mesh.has_unique_nondegenerate_triangles(&context.mesh_context()),
                mesh.validate_closed_pwn(&context.mesh_context())
            );
            Some(mesh)
        },
    }
}
fn polyhedron(
    points: &[Point3],
    faces: &[Vec<usize>],
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let points = points
        .iter()
        .map(|p| [p.x.clone(), p.y.clone(), p.z.clone()])
        .collect::<Vec<_>>();
    let faces = faces.iter().map(Vec::as_slice).collect::<Vec<_>>();
    solid::polyhedron_with_context(&points, &faces, context)
}
fn prism(
    points: &[(i32, i32)],
    context: &GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let n = points.len();
    let mut vertices = ring(points, 0);
    vertices.extend(ring(points, 2));
    let mut faces = vec![(0..n).rev().collect(), (n..2 * n).collect()];
    for i in 0..n {
        let next = (i + 1) % n;
        faces.push(vec![i, next, n + next, n + i]);
    }
    polyhedron(&vertices, &faces, context)
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
    let out = std::env::args().nth(1).unwrap();
    let out = Path::new(&out);
    std::fs::create_dir_all(out).unwrap();
    let square = [(0, 0), (2, 0), (2, 2), (0, 2)];
    let concave = [(3, 1), (1, 1), (1, 3), (0, 3), (0, 0), (3, 0)];
    let crossed = [(0, 0), (2, 2), (0, 2), (2, 0)];
    let crossed_nonzero = [(0, 0), (3, 3), (0, 2), (2, 0)];
    let repeated = [(0, 0), (2, 0), (2, 2), (0, 0), (0, 2)];
    for (label, context) in [
        ("strict", GeometryContext::STRICT),
        ("approximate", GeometryContext::APPROXIMATE_512),
    ] {
        for (name, xy) in [
            ("square", square.as_slice()),
            ("concave", concave.as_slice()),
            ("crossed-zero-area", crossed.as_slice()),
            ("crossed-nonzero-area", crossed_nonzero.as_slice()),
            ("repeated-point", repeated.as_slice()),
        ] {
            let points = ring(xy, 0);
            show(
                &format!("polyhedron-{name}-{label}"),
                &context,
                polyhedron(&points, &[(0..xy.len()).collect()], &context),
            );
            show(
                &format!("loft-{name}-{label}"),
                &context,
                solid::loft_with_context(&[points, ring(xy, 2)], &context),
            );
        }
        for (name, sections) in [
            (
                "repeated-first",
                vec![ring(&square, 0), ring(&square, 0), ring(&square, 2)],
            ),
            (
                "repeated-middle",
                vec![
                    ring(&square, 0),
                    ring(&square, 1),
                    ring(&square, 1),
                    ring(&square, 2),
                ],
            ),
            (
                "repeated-last",
                vec![ring(&square, 0), ring(&square, 2), ring(&square, 2)],
            ),
        ] {
            show(
                &format!("loft-{name}-{label}"),
                &context,
                solid::loft_with_context(&sections, &context),
            );
        }
        let unclear = vec![
            Point3::origin(),
            point(1, 0, 0),
            Point3::new(r(2), zero(), r(0)),
        ];
        show(
            &format!("polyhedron-unclear-support-{label}"),
            &context,
            polyhedron(&unclear, &[vec![0, 1, 2]], &context),
        );
        let raised = unclear
            .iter()
            .map(|p| Point3::new(p.x.clone(), p.y.clone(), r(2)))
            .collect();
        show(
            &format!("loft-unclear-support-{label}"),
            &context,
            solid::loft_with_context(&[unclear, raised], &context),
        );
        show(
            &format!("polyhedron-empty-{label}"),
            &context,
            polyhedron(&[], &[], &context),
        );
        show(
            &format!("polyhedron-unused-points-{label}"),
            &context,
            polyhedron(&ring(&square, 0), &[], &context),
        );
    }
    for reversed in [false, true] {
        for shift in 0..concave.len() {
            let mut xy = concave.to_vec();
            if reversed {
                xy.reverse();
            }
            xy.rotate_left(shift);
            let context = GeometryContext::STRICT;
            let name = format!("concave-order-{reversed}-{shift}");
            if let Some(loft) = show(
                &format!("loft-{name}"),
                &context,
                solid::loft_with_context(&[ring(&xy, 0), ring(&xy, 2)], &context),
            ) {
                println!(
                    "loft-{name}: arm={:?} notch={:?} union_empty={:?}",
                    solid::contains_point(
                        &loft,
                        &Point3::new((r(1) / r(2)).unwrap(), r(2), r(1))
                    ),
                    solid::contains_point(&loft, &point(2, 2, 1)),
                    loft.try_union(&solid::empty()).map(|m| m.triangles.len())
                );
            }
            show(&format!("polyhedron-{name}"), &context, prism(&xy, &context));
        }
    }
    let cube = solid::cube(r(1)).unwrap();
    let disconnected =
        solid::merge(&[cube.clone(), cube.translated(r(4), r(0), r(0)).unwrap()]).unwrap();
    if let Some(sum) = show(
        "minkowski-disconnected",
        &GeometryContext::STRICT,
        solid::minkowski_sum_with_context(&disconnected, &cube, &GeometryContext::STRICT),
    ) {
        println!(
            "minkowski-disconnected: gap_expected=false gap_actual={:?}",
            solid::contains_point(&sum, &point(3, 1, 1))
        );
    }
    let mut positions = cube.positions.to_vec();
    positions.push(point(20, 0, 0));
    let unused = TriangleMesh::new(positions, cube.triangles.to_vec());
    if let Some(sum) = show(
        "minkowski-unused-point",
        &GeometryContext::STRICT,
        solid::minkowski_sum_with_context(&unused, &cube, &GeometryContext::STRICT),
    ) {
        println!(
            "minkowski-unused-point: outside_expected=false outside_actual={:?}",
            solid::contains_point(
                &sum,
                &Point3::new(r(10), (r(1) / r(2)).unwrap(), (r(1) / r(2)).unwrap())
            )
        );
    }
    let malformed = TriangleMesh::new(vec![], vec![Triangle::new(0, 1, 2)]);
    show(
        "minkowski-empty-malformed",
        &GeometryContext::STRICT,
        solid::minkowski_sum_with_context(
            &solid::empty(),
            &malformed,
            &GeometryContext::STRICT,
        ),
    );
    let point_only = TriangleMesh::new(vec![Point3::origin()], vec![]);
    show(
        "minkowski-degenerate-points",
        &GeometryContext::STRICT,
        solid::minkowski_sum_with_context(&point_only, &point_only, &GeometryContext::STRICT),
    );
    let unclear = TriangleMesh::new(
        vec![
            Point3::origin(),
            point(1, 0, 0),
            point(0, 1, 0),
            Point3::new(zero(), r(0), r(0)),
        ],
        vec![],
    );
    show(
        "minkowski-undecided",
        &GeometryContext::STRICT,
        solid::minkowski_sum_with_context(&unclear, &point_only, &GeometryContext::STRICT),
    );
    let tetra = polyhedron(
        &[
            point(0, 0, 0),
            point(2, 0, 0),
            point(0, 2, 0),
            point(0, 0, 2),
        ],
        &[vec![0, 2, 1], vec![0, 1, 3], vec![0, 3, 2], vec![1, 2, 3]],
        &GeometryContext::STRICT,
    )
    .unwrap()
    .value;
    let low = ring(&[(-2, -2), (2, -2), (2, 2), (-2, 2)], 0);
    let high = ring(&[(-4, -4), (4, -4), (4, 4), (-4, 4)], 8);
    for (name, mesh) in [
        ("tetrahedron", tetra),
        ("loft-square", solid::loft(&[low, high]).unwrap()),
        (
            "cube-minkowski",
            solid::minkowski_sum(&solid::cube(r(2)).unwrap(), &solid::cube(r(3)).unwrap())
                .unwrap(),
        ),
    ] {
        mesh.validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
            .unwrap();
        std::fs::write(out.join(format!("{name}.json")), record(&mesh)).unwrap();
        println!(
            "fixture {name}: positions={} triangles={}",
            mesh.positions.len(),
            mesh.triangles.len()
        );
    }
}
