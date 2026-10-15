#![cfg(feature = "sdf")]
//! An unresolved or invalid sampled field must not publish successful geometry.
use csgrs::{
    GeometryContext, Real,
    errors::{ValidationError, ValidationErrorKind},
    solid,
};
use hyperlattice::Point3;
use hypersdf::SdfExpr;

trait FailureKind {
    fn failure_kind(&self) -> Option<ValidationErrorKind>;
}
impl<T> FailureKind for Result<T, ValidationError> {
    fn failure_kind(&self) -> Option<ValidationErrorKind> {
        self.as_ref().err().map(ValidationError::kind)
    }
}

fn point(n: i32) -> Point3 {
    Point3::new(n.into(), n.into(), n.into())
}
fn unknown_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

#[test]
fn closure_uncertainty_is_not_successful_empty_geometry() {
    let zero = unknown_zero();
    let field = |_: &Point3| zero.clone();
    assert_eq!(
        solid::sdf(field, (3, 3, 3), point(-1), point(1), 0.into()).failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
}

#[test]
fn diagnostic_uncertainty_is_not_nonfinite_empty_geometry() {
    let zero = unknown_zero();
    assert_eq!(
        solid::sdf_with_diagnostics(
            |_| zero.clone(),
            (3, 3, 3),
            point(-1),
            point(1),
            0.into()
        )
        .failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
}

#[test]
fn invalid_expression_domains_remain_invalid_input() {
    let expr = SdfExpr::constant((-1).into()).sqrt();
    assert_eq!(
        solid::sdf_expr(expr, (3, 3, 3), point(-1), point(1), 0.into()).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}

#[test]
fn diagnostic_expression_domains_remain_invalid_input() {
    let expr = SdfExpr::sphere(
        hyperlimit::Point3::new(0.into(), 0.into(), 0.into()),
        (-1).into(),
    );
    assert_eq!(
        solid::sdf_expr_with_diagnostics(expr, (3, 3, 3), point(-1), point(1), 0.into())
            .failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}

#[test]
fn expression_branch_uncertainty_retains_its_category() {
    let a = Real::e().sin();
    let expr = SdfExpr::constant(a.clone()).union(SdfExpr::constant(a + unknown_zero()));
    assert_eq!(
        solid::sdf_expr_with_context(
            expr,
            (3, 3, 3),
            point(-1),
            point(1),
            0.into(),
            &GeometryContext::STRICT
        )
        .failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
}

#[test]
fn context_expression_domain_errors_are_distinct_from_uncertainty() {
    let expr = SdfExpr::constant((-1).into()).sqrt();
    assert_eq!(
        solid::sdf_expr_with_context(
            expr,
            (3, 3, 3),
            point(-1),
            point(1),
            0.into(),
            &GeometryContext::STRICT
        )
        .failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}

#[test]
fn overflow_is_resource_exhaustion_before_sampling() {
    let field =
        |_: &Point3| -> Real { panic!("capacity must be checked before invoking the field") };
    assert_eq!(
        solid::sdf(field, (usize::MAX, 2, 2), point(-1), point(1), 0.into()).failure_kind(),
        Some(ValidationErrorKind::ResourceExhausted)
    );
}

#[test]
fn undersized_resolution_is_rejected_instead_of_clamped() {
    assert_eq!(
        solid::sdf_with_context(
            |_| Real::one(),
            (1, 3, 3),
            point(-1),
            point(1),
            0.into(),
            &GeometryContext::STRICT
        )
        .failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}

#[test]
fn partial_sampling_failure_keeps_sample_index_and_native_source() {
    use std::error::Error;
    // x visits -1, 0, 1, so sqrt(-x) fails only at the third sample.
    let error = solid::sdf_expr(
        SdfExpr::x().complement().sqrt(),
        (3, 3, 3),
        point(-1),
        point(1),
        0.into(),
    )
    .unwrap_err();
    assert!(matches!(
        error,
        ValidationError::SdfSample {
            sample: [2, 0, 0],
            ..
        }
    ));
    let cause = error
        .source()
        .unwrap()
        .downcast_ref::<hypersdf::SdfEvaluationError>()
        .unwrap();
    assert_eq!(
        cause.source().unwrap().downcast_ref::<hyperreal::Problem>(),
        Some(&hyperreal::Problem::SqrtNegative)
    );
    // A failure does not poison subsequent sampling or an empty composition.
    use csgrs::solid::SolidExt;
    let empty = solid::sdf_expr(
        SdfExpr::constant(1.into()),
        (3, 3, 3),
        point(-1),
        point(1),
        0.into(),
    )
    .unwrap();
    assert!(empty.positions.is_empty() && empty.triangles.is_empty());
    let cube = solid::cube(1.into()).unwrap();
    let restored = empty.try_union(&cube).unwrap();
    assert_eq!(
        solid::bounding_box(&restored).unwrap(),
        solid::bounding_box(&cube).unwrap()
    );
    let removed = restored
        .try_difference(&cube)
        .unwrap()
        .try_union(&empty)
        .unwrap();
    assert!(removed.positions.is_empty() && removed.triangles.is_empty());
}

#[test]
fn diagnostic_plane_has_independently_known_coordinates_and_counts() {
    // Samples x=-1,0,1 at iso=1/3: 18 negative and 9 positive.
    let third = (Real::one() / Real::from(3)).unwrap();
    let result = solid::sdf_expr_with_diagnostics_and_context(
        SdfExpr::x(),
        (3, 3, 3),
        point(-1),
        point(1),
        third.clone(),
        &GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(result.certainty, csgrs::GeometryCertainty::Certified);
    let (mesh, diagnostics) = result.value;
    assert!(!mesh.triangles.is_empty());
    assert!(mesh.positions.iter().all(|p| p.x == third));
    assert_eq!(
        (
            diagnostics.negative_sample_count,
            diagnostics.zero_sample_count,
            diagnostics.positive_sample_count
        ),
        (18, 0, 9)
    );
    assert_eq!(diagnostics.min_finite_value, Some((-1).into()));
    assert_eq!(diagnostics.max_finite_value, Some(1.into()));
    assert!(diagnostics.hypersdf_preview.unwrap().is_self_consistent());
}

#[test]
fn approximate_samples_do_not_certify_expression_branches_or_poison_strict_queries() {
    let expr = SdfExpr::x().add_expr(SdfExpr::constant(unknown_zero()));
    let sample = |context| {
        solid::sdf_expr_with_diagnostics_and_context(
            expr.clone(),
            (3, 3, 3),
            point(-1),
            point(1),
            0.into(),
            context,
        )
    };
    assert_eq!(
        sample(&GeometryContext::STRICT).failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
    let approximate = sample(&GeometryContext::APPROXIMATE_512).unwrap();
    assert_eq!(
        approximate.certainty,
        csgrs::GeometryCertainty::Approximate512Consumed
    );
    let preview = approximate.value.1.hypersdf_preview.unwrap();
    assert!(preview.is_self_consistent());
    assert_eq!(preview.grid_samples.samples.unknown_sign_count, 9);
    assert_eq!(
        sample(&GeometryContext::STRICT).failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
    // Choosing an approximate meshing policy does not authorize an approximate min branch.
    let branch = SdfExpr::constant(unknown_zero()).union(SdfExpr::constant(0.into()));
    assert_eq!(
        solid::sdf_expr_with_context(
            branch,
            (3, 3, 3),
            point(-1),
            point(1),
            0.into(),
            &GeometryContext::APPROXIMATE_512
        )
        .failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
    let certified = solid::sdf_expr_with_context(
        SdfExpr::constant(1.into()),
        (3, 3, 3),
        point(-1),
        point(1),
        0.into(),
        &GeometryContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(certified.certainty, csgrs::GeometryCertainty::Certified);
}
