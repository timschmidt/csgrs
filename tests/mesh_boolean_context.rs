use csgrs::{
    GeometryCertainty, GeometryContext, GeometryOutcome, Real, TriangleMesh,
    solid::{self, SolidExt},
};
use hypermesh::{BooleanOp, HypermeshError, HypermeshResult};

fn with_context(
    left: &TriangleMesh,
    right: &TriangleMesh,
    operation: BooleanOp,
    context: &GeometryContext,
) -> HypermeshResult<GeometryOutcome<TriangleMesh>> {
    match operation {
        BooleanOp::Union => left.try_union_with_context(right, context),
        BooleanOp::Difference => left.try_difference_with_context(right, context),
        BooleanOp::Intersection => left.try_intersection_with_context(right, context),
        BooleanOp::SymmetricDifference => left.try_xor_with_context(right, context),
    }
}

fn strict(
    left: &TriangleMesh,
    right: &TriangleMesh,
    operation: BooleanOp,
) -> HypermeshResult<TriangleMesh> {
    match operation {
        BooleanOp::Union => left.try_union(right),
        BooleanOp::Difference => left.try_difference(right),
        BooleanOp::Intersection => left.try_intersection(right),
        BooleanOp::SymmetricDifference => left.try_xor(right),
    }
}

fn signed_volume(mesh: &TriangleMesh) -> f64 {
    mesh.triangles
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle
                .indices()
                .map(|index| mesh.positions[index].clone().to_vector());
            f64::from(a.dot(&b.cross(&c))) / 6.0
        })
        .sum()
}

#[test]
fn rational_booleans_stay_certified_with_either_context() {
    let left = solid::cube(Real::from(2));
    let right = left.translated(Real::one(), Real::zero(), Real::zero());

    for (operation, volume) in [
        (BooleanOp::Union, 12.0),
        (BooleanOp::Difference, 4.0),
        (BooleanOp::Intersection, 4.0),
        (BooleanOp::SymmetricDifference, 8.0),
    ] {
        let legacy = strict(&left, &right, operation).unwrap();
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
            let outcome = with_context(&left, &right, operation, &context).unwrap();
            assert_eq!(outcome.certainty, GeometryCertainty::Certified);
            assert_eq!(outcome.value, legacy);
            assert!((signed_volume(&outcome.value) - volume).abs() < 1e-10);
            assert!(
                outcome
                    .value
                    .is_closed_manifold_geometry(&context.mesh_context())
                    .unwrap()
                    .into_value()
            );
            assert_eq!(
                solid::boolean_with_context(&left, &right, operation, &context).unwrap(),
                outcome
            );
        }
    }
}

#[test]
fn every_boolean_reports_approximation_without_affecting_later_strict_calls() {
    // The two boxes share a face, but the trigonometric identity at that face
    // needs the terminal policy to decide equality.
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    let unresolved_zero = &sine * &sine + &cosine * &cosine - Real::one();
    let left =
        solid::cube(Real::one()).translated(&sine - &Real::one(), Real::zero(), Real::zero());
    let right = solid::cube(Real::one()).translated(
        sine + unresolved_zero,
        Real::zero(),
        Real::zero(),
    );

    for (operation, volume) in [
        (BooleanOp::Union, 2.0),
        (BooleanOp::Difference, 1.0),
        (BooleanOp::Intersection, 0.0),
        (BooleanOp::SymmetricDifference, 2.0),
    ] {
        assert!(matches!(
            strict(&left, &right, operation),
            Err(HypermeshError::PredicateUndecided { .. })
        ));
        let outcome =
            with_context(&left, &right, operation, &GeometryContext::APPROXIMATE_512).unwrap();
        assert_eq!(outcome.certainty, GeometryCertainty::Approximate512Consumed);
        assert!((signed_volume(&outcome.value) - volume).abs() < 1e-10);
        if operation == BooleanOp::Intersection {
            assert!(outcome.value.triangles.is_empty());
        }
        assert!(matches!(
            with_context(&left, &right, operation, &GeometryContext::STRICT),
            Err(HypermeshError::PredicateUndecided { .. })
        ));
        assert!(matches!(
            strict(&left, &right, operation),
            Err(HypermeshError::PredicateUndecided { .. })
        ));
    }
}

#[test]
fn context_booleans_preserve_input_errors() {
    let invalid = TriangleMesh::new(
        vec![hyperlattice::Point3::origin()],
        vec![hypermesh::Triangle::new(0, 1, 2)],
    );
    let cube = solid::cube(Real::one());
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for operation in [
            BooleanOp::Union,
            BooleanOp::Difference,
            BooleanOp::Intersection,
            BooleanOp::SymmetricDifference,
        ] {
            assert_eq!(
                with_context(&invalid, &cube, operation, &context).unwrap_err(),
                HypermeshError::VertexIndexOutOfBounds {
                    index: 1,
                    vertex_count: 1,
                }
            );
        }
    }
}

#[test]
fn drill_cylinder_uses_the_requested_context() {
    let body = solid::cuboid(Real::from(12), Real::from(12), Real::from(4));
    let drill = solid::cylinder(Real::from(2), Real::from(6), 16).translated(
        Real::from(6),
        Real::from(6),
        Real::from(-1),
    );
    assert!(matches!(
        body.try_difference(&drill),
        Err(HypermeshError::PredicateUndecided { .. })
    ));
    let outcome = body
        .try_difference_with_context(&drill, &GeometryContext::APPROXIMATE_512)
        .unwrap();
    assert_eq!(outcome.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(!outcome.value.triangles.is_empty());
    assert!(
        outcome
            .value
            .is_closed_manifold_geometry(&GeometryContext::APPROXIMATE_512.mesh_context())
            .unwrap()
            .into_value()
    );
    let expected_volume = 12.0 * 12.0 * 4.0
        - 4.0 * (16.0 / 2.0) * 2.0_f64.powi(2) * (std::f64::consts::TAU / 16.0).sin();
    assert!((signed_volume(&outcome.value) - expected_volume).abs() < 1e-9);
    assert!(matches!(
        body.try_difference_with_context(&drill, &GeometryContext::STRICT),
        Err(HypermeshError::PredicateUndecided { .. })
    ));
}
