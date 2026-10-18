#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{
    GeometryCertainty, GeometryContext, Real, TriangleMesh,
    errors::{ValidationError, ValidationErrorKind},
    solid::{self, SolidExt},
};
use hypermesh::{HypermeshError, MinkowskiOptions, Point3, Triangle};
use std::error::Error;
use std::sync::atomic::{AtomicBool, Ordering};

fn r(n: i32) -> Real {
    n.into()
}
fn q(n: i32, d: i32) -> Real {
    (r(n) / r(d)).unwrap()
}
fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}
fn volume(mesh: &TriangleMesh, expected: Real) {
    let six = mesh.triangles.iter().fold(r(0), |sum, t| {
        let [a, b, c] = t.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    assert_eq!(
        six.certified_eq_until(&(r(6) * expected), -1024).as_bool(),
        Some(true)
    );
}
fn complete(
    left: &TriangleMesh,
    right: &TriangleMesh,
    expected: Real,
    context: &GeometryContext,
) -> TriangleMesh {
    let outcome = solid::minkowski_sum_with_context(left, right, context).unwrap();
    assert_eq!(outcome.certainty, GeometryCertainty::Certified);
    let mesh = outcome.value;
    if !mesh.triangles.is_empty() {
        mesh.validate_closed_pwn(&context.mesh_context()).unwrap();
        assert!(
            mesh.is_closed_manifold_geometry(&context.mesh_context())
                .unwrap()
                .value
        );
    } else {
        assert!(mesh.positions.is_empty());
    }
    volume(&mesh, expected);
    mesh
}
fn l_prism() -> TriangleMesh {
    let xy = [(3, 1), (1, 1), (1, 3), (0, 3), (0, 0), (3, 0)];
    let sections = [0, 2].map(|z| xy.iter().map(|&(x, y)| p(x, y, z)).collect());
    solid::loft(&sections).unwrap()
}

#[test]
fn disconnected_sum_keeps_gaps_and_composes() {
    let cube = solid::cube(r(1)).unwrap();
    let a = solid::merge(&[cube.clone(), cube.translated(r(4), r(0), r(0)).unwrap()]).unwrap();
    let b = solid::merge(&[cube.clone(), cube.translated(r(8), r(0), r(0)).unwrap()]).unwrap();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let sum = complete(&a, &cube, r(16), &context);
        assert!(!solid::contains_point(&sum, &p(3, 1, 1)).unwrap());
        assert!(solid::contains_point(&sum, &p(1, 1, 1)).unwrap());
        assert!(solid::contains_point(&sum, &p(5, 1, 1)).unwrap());
        volume(&sum.try_union(&solid::empty()).unwrap(), r(16));
        let removed = sum.try_difference(&sum).unwrap();
        assert!(removed.positions.is_empty() && removed.triangles.is_empty());
        let recovered = removed.try_union(&cube).unwrap();
        complete(&recovered, &cube, r(8), &context);
        let both = complete(&a, &b, r(32), &context);
        for x in [1, 5, 9, 13] {
            assert!(solid::contains_point(&both, &p(x, 1, 1)).unwrap());
        }
        for x in [3, 7, 11] {
            assert!(!solid::contains_point(&both, &p(x, 1, 1)).unwrap());
        }
    }
}

#[test]
fn concave_sum_preserves_notch_and_commutes() {
    let source = l_prism();
    let cube = solid::cube(r(1)).unwrap();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let sum = complete(&source, &cube, r(36), &context);
        let reversed = complete(&cube, &source, r(36), &context);
        for (point, inside) in [(p(3, 3, 1), false), (p(1, 3, 1), true), (p(3, 1, 1), true)] {
            assert_eq!(solid::contains_point(&sum, &point).unwrap(), inside);
            assert_eq!(solid::contains_point(&reversed, &point).unwrap(), inside);
        }
        // Independent box union oracle: ([0,4]×[0,2] ∪ [0,2]×[0,4])×[0,3].
        let expected = solid::cuboid(r(4), r(2), r(3))
            .unwrap()
            .try_union(&solid::cuboid(r(2), r(4), r(3)).unwrap())
            .unwrap();
        assert!(sum.try_difference(&expected).unwrap().triangles.is_empty());
        assert!(expected.try_difference(&sum).unwrap().triangles.is_empty());
        volume(&sum.try_intersection(&reversed).unwrap(), r(36));
    }
}

#[test]
fn hollow_nested_and_multiply_wound_solids_use_nonzero_winding() {
    let cube = solid::cube(r(1)).unwrap();
    let outer = solid::cube(r(4)).unwrap();
    let inner = solid::cube(r(2))
        .unwrap()
        .translated(r(1), r(1), r(1))
        .unwrap();
    let hollow = solid::merge(&[outer.clone(), inner.reversed_winding()]).unwrap();
    let nested = solid::merge(&[outer.clone(), inner]).unwrap();
    let doubled = solid::merge(&[cube.clone(), cube.clone()]).unwrap();
    let cancelled = solid::merge(&[cube.clone(), cube.reversed_winding()]).unwrap();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let sum = complete(&hollow, &cube, r(124), &context);
        let hole = Point3::new(q(5, 2), q(5, 2), q(5, 2));
        assert!(!solid::contains_point(&sum, &hole).unwrap());
        assert!(solid::contains_point(&sum, &Point3::new(q(3, 2), q(5, 2), q(5, 2))).unwrap());
        let full = complete(&nested, &cube, r(125), &context);
        assert!(solid::contains_point(&full, &hole).unwrap());
        complete(&doubled, &cube, r(8), &context);
        complete(&cube.reversed_winding(), &cube, r(8), &context);
        let empty = complete(&cancelled, &cube, r(0), &context);
        complete(&empty.try_union(&cube).unwrap(), &cube, r(8), &context);
    }
}

#[test]
fn source_storage_and_empty_identities_do_not_hide_failures() {
    let cube = solid::cube(r(1)).unwrap();
    let mut positions = cube.positions.to_vec();
    positions.push(p(20, 0, 0));
    let unused = TriangleMesh::new(positions, cube.triangles.to_vec());
    let empty = solid::empty();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let sum = complete(&unused, &cube, r(8), &context);
        assert!(!solid::contains_point(&sum, &Point3::new(r(10), q(1, 2), q(1, 2))).unwrap());
        for (a, b) in [(&cube, &empty), (&empty, &cube), (&empty, &empty)] {
            complete(a, b, r(0), &context);
        }
        for bad in [
            TriangleMesh::new(vec![], vec![Triangle::new(0, 1, 2)]),
            TriangleMesh::new(vec![p(0, 0, 0)], vec![]),
            TriangleMesh::new(
                vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)],
                vec![Triangle::new(0, 1, 2)],
            ),
            TriangleMesh::new(vec![p(0, 0, 0)], vec![Triangle::new(0, 0, 0)]),
        ] {
            for (a, b) in [(&bad, &empty), (&empty, &bad), (&bad, &cube)] {
                let error = solid::minkowski_sum_with_context(a, b, &context).unwrap_err();
                assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
                assert!(matches!(error, ValidationError::Mesh(_)));
                assert!(error.source().is_some());
            }
        }
    }
    // A native proof from the previous query must not validate mutated indices.
    complete(&cube, &cube, r(8), &GeometryContext::STRICT);
    let mut changed = cube.clone();
    std::sync::Arc::make_mut(&mut changed.triangles)[0] = Triangle::new(0, 0, 0);
    assert!(matches!(
        solid::minkowski_sum(&changed, &cube),
        Err(ValidationError::Mesh(
            HypermeshError::DegenerateTriangle { .. }
        ))
    ));
    complete(&cube, &cube, r(8), &GeometryContext::STRICT);
}

#[test]
fn limits_and_cancellation_fail_typed_then_recover() {
    let cube = solid::cube(r(1)).unwrap();
    let a = solid::merge(&[cube.clone(), cube.translated(r(4), r(0), r(0)).unwrap()]).unwrap();
    for options in [
        MinkowskiOptions {
            max_cells: 1,
            ..Default::default()
        },
        MinkowskiOptions {
            max_pair_sums: 1,
            ..Default::default()
        },
        MinkowskiOptions {
            max_pair_points: 1,
            ..Default::default()
        },
    ] {
        let error =
            solid::minkowski_sum_with_options(&a, &cube, &options, &GeometryContext::STRICT)
                .unwrap_err();
        assert_eq!(error.kind(), ValidationErrorKind::ResourceExhausted);
        assert!(matches!(
            error,
            ValidationError::Mesh(HypermeshError::ResourceLimitExceeded { .. })
        ));
        assert!(error.source().is_some());
    }
    let cancelled = AtomicBool::new(true);
    let options = MinkowskiOptions {
        cancellation: Some(&cancelled),
        ..Default::default()
    };
    assert!(matches!(
        solid::minkowski_sum_with_options(&a, &cube, &options, &GeometryContext::STRICT),
        Err(ValidationError::Mesh(
            HypermeshError::OperationCancelled { .. }
        ))
    ));
    cancelled.store(false, Ordering::Relaxed);
    let recovered =
        solid::minkowski_sum_with_options(&a, &cube, &options, &GeometryContext::STRICT)
            .unwrap();
    assert_eq!(recovered.certainty, GeometryCertainty::Certified);
    volume(&recovered.value, r(16));
}

#[test]
fn explicit_hull_preserves_point_set_interpretation_and_typed_failure() {
    let cube = solid::cube(r(1)).unwrap();
    let source =
        solid::merge(&[cube.clone(), cube.translated(r(4), r(0), r(0)).unwrap()]).unwrap();
    let hull = solid::minkowski_hull(&source, &cube).unwrap();
    volume(&hull, r(24));
    assert!(solid::contains_point(&hull, &p(3, 1, 1)).unwrap());
    let point = TriangleMesh::new(vec![Point3::origin()], vec![]);
    assert!(matches!(
        solid::minkowski_hull(&point, &point),
        Err(ValidationError::Mesh(HypermeshError::DegeneratePointSet))
    ));
}

#[test]
fn sums_survive_linear_transforms_repetition_and_exact_small_gaps() {
    use hyperlattice::Matrix4;
    let cube = solid::cube(r(1)).unwrap();
    let source = l_prism();
    let transform = Matrix4::new([
        [r(2), r(1), r(0), r(0)],
        [r(0), r(3), r(1), r(0)],
        [r(0), r(0), r(4), r(0)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let sum = complete(
        &source.transformed(&transform).unwrap(),
        &cube.transformed(&transform).unwrap(),
        r(864),
        &GeometryContext::STRICT,
    );
    assert!(!solid::contains_point(&sum, &p(9, 10, 4)).unwrap());
    assert!(solid::contains_point(&sum, &p(5, 10, 4)).unwrap());
    let first = complete(&source, &cube, r(36), &GeometryContext::STRICT);
    let repeated = complete(&first, &cube, r(84), &GeometryContext::STRICT);
    assert!(!solid::contains_point(&repeated, &p(4, 4, 2)).unwrap());
    let delta = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
    let separated = solid::merge(&[
        cube.clone(),
        cube.translated(r(2) + &delta, r(0), r(0)).unwrap(),
    ])
    .unwrap();
    let sum = complete(&separated, &cube, r(16), &GeometryContext::STRICT);
    assert!(
        !solid::contains_point(&sum, &Point3::new(r(2) + &delta * q(1, 2), r(1), r(1)))
            .unwrap()
    );
}

#[test]
fn referenced_symbolic_coordinates_preserve_policy_and_receipts() {
    let cube = solid::cube(r(1)).unwrap();
    let shifted = cube.translated(Real::pi(), r(0), r(0)).unwrap();
    let sum = complete(&shifted, &cube, r(8), &GeometryContext::STRICT);
    assert!(solid::contains_point(&sum, &Point3::new(Real::pi() + r(1), r(1), r(1))).unwrap());
    let mut positions = cube.positions.to_vec();
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    positions[0].x = uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - r(1));
    let source = TriangleMesh::new(positions, cube.triangles.to_vec());
    let first = solid::minkowski_sum(&source, &cube).unwrap_err();
    assert_eq!(first.kind(), ValidationErrorKind::Undecided);
    let approximate =
        solid::minkowski_sum_with_context(&source, &cube, &GeometryContext::APPROXIMATE_512)
            .unwrap();
    assert_eq!(
        approximate.certainty,
        GeometryCertainty::Approximate512Consumed
    );
    assert_eq!(
        solid::minkowski_sum(&source, &cube).unwrap_err().kind(),
        ValidationErrorKind::Undecided
    );
}

#[test]
fn original_readme_and_full_example_inputs_complete_strictly() {
    let a = Real::try_from(1.1_f64).unwrap();
    let b = Real::try_from(0.45_f64).unwrap();
    // Decompose the L1 dilation into the cube, six face prisms, twelve edge
    // wedges and eight corner tetrahedra; this is independent of the hull code.
    let expected =
        &a * &a * &a + r(6) * &a * &a * &b + r(6) * &a * &b * &b + q(4, 3) * &b * &b * &b;
    complete(
        &solid::cube(a).unwrap(),
        &solid::octahedron(b).unwrap(),
        expected,
        &GeometryContext::STRICT,
    );
    // Keep examples/minkowski_sum.rs at its full original 16×8 sphere sampling.
    let side = Real::try_from(1.5_f64).unwrap();
    let radius = Real::try_from(0.4_f64).unwrap();
    let cube = solid::cube(side.clone()).unwrap();
    let sphere = solid::sphere(radius.clone(), 16, 8).unwrap();
    let result =
        solid::minkowski_sum_with_context(&cube, &sphere, &GeometryContext::STRICT).unwrap();
    assert_eq!(result.certainty, GeometryCertainty::Certified);
    result
        .value
        .validate_closed_pwn(&GeometryContext::STRICT.mesh_context())
        .unwrap();
    let bounds = result
        .value
        .exact_bounds(&GeometryContext::STRICT.mesh_context())
        .unwrap()
        .value
        .unwrap();
    for lower in [&bounds.mins.x, &bounds.mins.y, &bounds.mins.z] {
        assert_eq!(
            lower.certified_eq_until(&(-radius.clone()), -1024).as_bool(),
            Some(true)
        );
    }
    for upper in [&bounds.maxs.x, &bounds.maxs.y, &bounds.maxs.z] {
        assert_eq!(
            upper.certified_eq_until(&(&side + &radius), -1024).as_bool(),
            Some(true)
        );
    }
    // The source contains the radius segment on each coordinate axis. The
    // negative bounding-box corner lies outside every translated source ball,
    // hence outside the sum of the sampled polyhedron and the positive cube.
    assert!(
        solid::contains_point(&result.value, &Point3::new(-&radius * q(1, 2), r(0), r(0)),)
            .unwrap()
    );
    assert!(
        !solid::contains_point(
            &result.value,
            &Point3::new(-radius.clone(), -radius.clone(), -radius),
        )
        .unwrap()
    );
}
