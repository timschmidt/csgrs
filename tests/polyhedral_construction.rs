#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{
    GeometryCertainty, GeometryContext, Real, TriangleMesh,
    errors::{ValidationError, ValidationErrorKind},
    solid::{self, SolidExt},
};
use hypermesh::{HypermeshError, Point3};
use std::error::Error;

fn r(n: i32) -> Real {
    n.into()
}
fn q(n: i32, d: i32) -> Real {
    (r(n) / r(d)).unwrap()
}
fn exact(left: &Real, right: &Real) {
    assert_eq!(left.certified_eq_until(right, -1024).as_bool(), Some(true));
}
fn section(xy: &[(i32, i32)], z: i32) -> Vec<Point3> {
    xy.iter()
        .map(|&(x, y)| Point3::new(r(x), r(y), r(z)))
        .collect()
}
fn rows(points: &[Point3]) -> Vec<[Real; 3]> {
    points
        .iter()
        .map(|p| [p.x.clone(), p.y.clone(), p.z.clone()])
        .collect()
}
fn volume(mesh: &TriangleMesh, expected: Real) {
    // Independent divergence theorem, not the mesh's volume implementation.
    let six = mesh.triangles.iter().fold(r(0), |sum, t| {
        let [a, b, c] = t.indices().map(|i| &mesh.positions[i]);
        let term = (&a.z + &b.z + &c.z) * &(b - a).cross(&(c - a)).0[2];
        sum + Real::from(term.exact_rational_normal_form().unwrap())
    });
    exact(&six, &(r(6) * expected));
}
fn valid(mesh: &TriangleMesh, context: &GeometryContext) {
    mesh.validate_closed_pwn(&context.mesh_context()).unwrap();
    assert!(
        mesh.is_closed_manifold_geometry(&context.mesh_context())
            .unwrap()
            .value
    );
}
fn empty(mesh: &TriangleMesh) {
    assert!(mesh.positions.is_empty() && mesh.triangles.is_empty());
}
fn l_shape() -> [(i32, i32); 6] {
    [(3, 1), (1, 1), (1, 3), (0, 3), (0, 0), (3, 0)]
}
fn prism_faces(n: usize) -> Vec<Vec<usize>> {
    let mut faces = vec![(0..n).rev().collect(), (n..2 * n).collect()];
    faces.extend((0..n).map(|i| vec![i, (i + 1) % n, n + (i + 1) % n, n + i]));
    faces
}

#[test]
fn concave_faces_keep_authored_winding_and_lofts_compose_in_all_orders() {
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for reverse in [false, true] {
            for shift in 0..6 {
                let mut xy = l_shape();
                if reverse {
                    xy.reverse();
                }
                xy.rotate_left(shift);
                let sections = [section(&xy, 0), section(&xy, 2)];
                let face = solid::polyhedron_with_context(
                    &rows(&sections[0]),
                    &[&[0, 1, 2, 3, 4, 5]],
                    &context,
                )
                .unwrap();
                let twice_area = face.value.triangles.iter().fold(r(0), |sum, t| {
                    let [a, b, c] = t.indices().map(|i| &face.value.positions[i]);
                    sum + &(b - a).cross(&(c - a)).0[2]
                });
                exact(&twice_area, &r(if reverse { -10 } else { 10 }));
                assert_eq!(face.certainty, GeometryCertainty::Certified);

                let authored = rows(&sections.concat());
                let faces = prism_faces(6);
                let face_refs = faces.iter().map(Vec::as_slice).collect::<Vec<_>>();
                let polyhedron =
                    solid::polyhedron_with_context(&authored, &face_refs, &context).unwrap();
                valid(&polyhedron.value, &context);
                volume(&polyhedron.value, r(if reverse { -10 } else { 10 }));

                let loft = solid::loft_with_context(&sections, &context).unwrap();
                assert_eq!(loft.certainty, GeometryCertainty::Certified);
                valid(&loft.value, &context);
                volume(&loft.value, r(10));
                // Area 5 times height 2, with a notch that must remain empty.
                for (point, inside) in [
                    (Point3::new(q(1, 2), r(2), r(1)), true),
                    (Point3::new(r(2), r(2), r(1)), false),
                ] {
                    assert_eq!(solid::contains_point(&loft.value, &point).unwrap(), inside);
                }
                let composed = loft
                    .value
                    .try_union_with_context(&TriangleMesh::new(vec![], vec![]), &context)
                    .unwrap();
                valid(&composed.value, &context);
                volume(&composed.value, r(10));
                let removed = composed
                    .value
                    .try_difference_with_context(&loft.value, &context)
                    .unwrap();
                empty(&removed.value);
                let recovered = removed
                    .value
                    .try_union_with_context(&loft.value, &context)
                    .unwrap();
                volume(&recovered.value, r(10));
            }
        }
    }
}

#[test]
fn concave_loft_orientation_survives_axis_projection_and_negative_travel() {
    for travel in [-2, 2] {
        let sections = [section(&l_shape(), 0), section(&l_shape(), travel)];
        for axis in 0..3 {
            let sections = sections
                .iter()
                .map(|ring| {
                    ring.iter()
                        .map(|p| {
                            let values = [&p.x, &p.y, &p.z];
                            Point3::new(
                                values[axis].clone(),
                                values[(axis + 1) % 3].clone(),
                                values[(axis + 2) % 3].clone(),
                            )
                        })
                        .collect()
                })
                .collect::<Vec<_>>();
            let mesh = solid::loft(&sections).unwrap();
            valid(&mesh, &GeometryContext::STRICT);
            volume(&mesh, r(10));
        }
    }
}

#[test]
fn redundant_sections_coalesce_without_losing_the_result_or_receipt() {
    let square = [(0, 0), (2, 0), (2, 2), (0, 2)];
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for levels in [&[0, 0, 2][..], &[0, 1, 1, 2], &[0, 2, 2]] {
            let sections = levels
                .iter()
                .map(|&z| section(&square, z))
                .collect::<Vec<_>>();
            let result = solid::loft_with_context(&sections, &context).unwrap();
            assert_eq!(result.certainty, GeometryCertainty::Certified);
            valid(&result.value, &context);
            volume(&result.value, r(8));
            empty(
                &result
                    .value
                    .try_difference_with_context(&result.value, &context)
                    .unwrap()
                    .value,
            );
        }
        assert!(
            solid::loft_with_context(&[section(&square, 0), section(&square, 0)], &context)
                .is_err()
        );
        // Collinear boundary vertices are needed for side/cap conformity.
        let collinear = [(0, 0), (1, 0), (2, 0), (2, 2), (0, 2)];
        let mesh = solid::loft_with_context(
            &[section(&collinear, 0), section(&collinear, 2)],
            &context,
        )
        .unwrap();
        valid(&mesh.value, &context);
        volume(&mesh.value, r(8));
        assert!(mesh.value.triangles.iter().any(|t| t.indices().contains(&1)));
    }
}

#[test]
fn invalid_faces_and_sections_fail_before_publishing_geometry() {
    let rings: &[&[(i32, i32)]] = &[
        &[(0, 0), (2, 2), (0, 2), (2, 0)],
        &[(0, 0), (3, 3), (0, 2), (2, 0)],
        &[(0, 0), (2, 0), (2, 2), (0, 0), (0, 2)],
        &[(0, 0), (2, 0), (1, 0), (2, 2), (0, 2)],
    ];
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for ring in rings {
            let lower = section(ring, 0);
            let face = (0..lower.len()).collect::<Vec<_>>();
            for error in [
                solid::polyhedron_with_context(&rows(&lower), &[&face], &context).unwrap_err(),
                solid::loft_with_context(&[lower, section(ring, 2)], &context).unwrap_err(),
            ] {
                assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
                assert!(matches!(
                    error,
                    ValidationError::Triangulation(hypertri::Error::InvalidInput { .. })
                ));
                assert!(error.source().is_some());
            }
        }
        let square = [(0, 0), (2, 0), (2, 2), (0, 2)];
        let lower = section(&square, 0);
        let mut tilted = section(&square, 2);
        tilted[1].z = r(0);
        tilted[2].z = r(0);
        let error = solid::loft_with_context(&[lower, tilted], &context).unwrap_err();
        assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
        assert!(matches!(
            error,
            ValidationError::Mesh(HypermeshError::DegenerateTriangle { .. })
        ));
        assert!(error.source().is_some());
        valid(
            &solid::loft_with_context(&[section(&square, 0), section(&square, 2)], &context)
                .unwrap()
                .value,
            &context,
        );
    }
}

#[test]
fn support_predicate_cause_survives_and_approximate_evidence_stays_qualified() {
    let sin = Real::e().sin();
    let cos = Real::e().cos();
    let zero = uncertainty::terminal_zero(&sin * &sin + &cos * &cos - r(1));
    let points = [
        [r(0), r(0), r(0)],
        [r(1), r(0), r(0)],
        [r(2), zero.clone(), r(0)],
    ];
    let error = solid::polyhedron(&points, &[&[0, 1, 2]]).unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::Undecided);
    assert!(matches!(
        error,
        ValidationError::Mesh(HypermeshError::PredicateUndecided { .. })
    ));
    assert!(error.source().is_some());
    let bottom = vec![
        Point3::new(r(0), r(0), r(0)),
        Point3::new(r(1), r(0), r(0)),
        Point3::new(r(1), r(1), r(0)),
        Point3::new(r(0), r(1), zero),
    ];
    let top = bottom
        .iter()
        .map(|p| Point3::new(p.x.clone(), p.y.clone(), &p.z + r(1)))
        .collect::<Vec<_>>();
    let sections = [bottom, top];
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        let outcome = solid::loft_with_context(&sections, &context);
        if context == GeometryContext::STRICT {
            assert_eq!(outcome.unwrap_err().kind(), ValidationErrorKind::Undecided);
        } else {
            let outcome = outcome.unwrap();
            assert_eq!(outcome.certainty, GeometryCertainty::Approximate512Consumed);
            valid(&outcome.value, &context);
        }
    }
}

#[test]
fn structural_empty_and_open_faces_keep_their_distinct_contracts() {
    empty(&solid::polyhedron(&[], &[]).unwrap());
    let points = [[r(0), r(0), r(0)], [r(1), r(0), r(0)], [r(0), r(1), r(0)]];
    let point_buffer = solid::polyhedron(&points, &[]).unwrap();
    assert_eq!(point_buffer.positions.len(), 3);
    assert!(point_buffer.triangles.is_empty());
    assert!(
        point_buffer
            .try_union(&TriangleMesh::new(vec![], vec![]))
            .is_err()
    );
    let face = solid::polyhedron(&points, &[&[0, 1, 2]]).unwrap();
    assert!(matches!(
        face.try_union(&TriangleMesh::new(vec![], vec![])),
        Err(HypermeshError::OpenInput { .. })
    ));
    assert!(matches!(
        solid::polyhedron(&points, &[&[0, 1, 3]]),
        Err(ValidationError::IndexOutOfRangeWithLen { .. })
    ));
}

#[test]
fn original_tetrahedron_and_square_loft_keep_independent_exact_volumes() {
    let points = [
        [r(0), r(0), r(0)],
        [r(2), r(0), r(0)],
        [r(0), r(2), r(0)],
        [r(0), r(0), r(2)],
    ];
    let tetra =
        solid::polyhedron(&points, &[&[0, 2, 1], &[0, 1, 3], &[0, 3, 2], &[1, 2, 3]]).unwrap();
    valid(&tetra, &GeometryContext::STRICT);
    volume(&tetra, q(4, 3));
    let loft = solid::loft(&[
        section(&[(-2, -2), (2, -2), (2, 2), (-2, 2)], 0),
        section(&[(-4, -4), (4, -4), (4, 4), (-4, 4)], 8),
    ])
    .unwrap();
    valid(&loft, &GeometryContext::STRICT);
    volume(&loft, q(896, 3));
    volume(
        &loft.try_union(&TriangleMesh::new(vec![], vec![])).unwrap(),
        q(896, 3),
    );
}

#[test]
fn original_readme_polyhedron_and_three_section_loft_keep_their_meaning() {
    // These values and resolutions are the original render fixtures. Import
    // each finite f64 exactly; the oracle uses those represented values too.
    let f = |n: f64| Real::try_from(n).unwrap();
    let height = f(1.7);
    let points = [
        [r(-1), r(-1), r(0)],
        [r(1), r(-1), r(0)],
        [r(1), r(1), r(0)],
        [r(-1), r(1), r(0)],
        [r(0), r(0), height.clone()],
    ];
    let pyramid = solid::polyhedron(
        &points,
        &[
            &[3, 2, 1, 0],
            &[0, 1, 4],
            &[1, 2, 4],
            &[2, 3, 4],
            &[3, 0, 4],
        ],
    )
    .unwrap();
    valid(&pyramid, &GeometryContext::STRICT);
    volume(&pyramid, r(4) * height * q(1, 3));

    let parameters = [
        (f(0.8), f(0.8), f(-0.8)),
        (f(1.1), f(0.45), r(0)),
        (f(0.45), f(0.65), r(1)),
    ];
    let sections = parameters
        .iter()
        .map(|(x, y, z)| {
            vec![
                Point3::new(-x, -y, z.clone()),
                Point3::new(x.clone(), -y, z.clone()),
                Point3::new(x.clone(), y.clone(), z.clone()),
                Point3::new(-x, y.clone(), z.clone()),
            ]
        })
        .collect::<Vec<_>>();
    let loft = solid::loft(&sections).unwrap();
    valid(&loft, &GeometryContext::STRICT);
    // Each segment's rectangle area is quadratic in height. Simpson integration
    // is exact, independently of the chosen surface triangulation.
    let expected = parameters.windows(2).fold(r(0), |sum, p| {
        let (a, b, z0) = &p[0];
        let (c, d, z1) = &p[1];
        sum + (z1 - z0) * q(2, 3) * (r(2) * a * b + a * d + c * b + r(2) * c * d)
    });
    volume(&loft, expected.clone());
    volume(&loft.try_union(&solid::empty()).unwrap(), expected);
}

#[test]
fn approximate_section_coalescing_retains_its_qualification() {
    let sin = Real::e().sin();
    let cos = Real::e().cos();
    let zero = uncertainty::terminal_zero(&sin * &sin + &cos * &cos - r(1));
    let square = [(0, 0), (2, 0), (2, 2), (0, 2)];
    let lower = section(&square, 0);
    let repeated = lower
        .iter()
        .map(|p| Point3::new(p.x.clone(), p.y.clone(), zero.clone()))
        .collect();
    let sections = [lower, repeated, section(&square, 2)];
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        let result = solid::loft_with_context(&sections, &context);
        if context == GeometryContext::STRICT {
            assert_eq!(result.unwrap_err().kind(), ValidationErrorKind::Undecided);
        } else {
            let outcome = result.unwrap();
            assert_eq!(outcome.certainty, GeometryCertainty::Approximate512Consumed);
            assert_eq!(outcome.value.positions.len(), 8);
            valid(&outcome.value, &context);
            volume(&outcome.value, r(8));
        }
    }
}
