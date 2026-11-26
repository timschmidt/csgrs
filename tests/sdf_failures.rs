#![cfg(feature = "sdf")]
//! An unresolved or invalid sampled field must not publish successful geometry.
//! Field expression evaluation, meshing policy, diagnostics and sampled geometry have separate contracts.
#[path = "support/uncertainty.rs"]
/// Shared proved-zero control and independently unresolved sample/branch witness.
mod uncertainty;

use csgrs::{
    GeometryContext, Real,
    errors::{ValidationError, ValidationErrorKind},
    solid,
};
use hyperlattice::Point3;
use hypersdf::SdfExpr;

/// Observes typed modeling failure without reinterpreting successful geometry.
trait FailureKind {
    /// Returns the existing failure category or `None` on success.
    fn failure_kind(&self) -> Option<ValidationErrorKind>;
}
impl<T> FailureKind for Result<T, ValidationError> {
    /// Borrows the result and classifies its error without inspecting or modifying success values.
    fn failure_kind(&self) -> Option<ValidationErrorKind> {
        self.as_ref().err().map(ValidationError::kind)
    }
}

/// Constructs an exact diagonal point `(n,n,n)` for sampling bounds.
fn point(n: i32) -> Point3 {
    Point3::new(n.into(), n.into(), n.into())
}
/// Checks the retained Pythagorean identity and returns the shared strictly unresolved exact zero.
fn unknown_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

#[test]
/// Requires an unresolved constant closure to report undecided rather than an empty extracted mesh.
fn closure_uncertainty_is_not_successful_empty_geometry() {
    let zero = unknown_zero();
    let field = |_: &Point3| zero.clone();
    assert_eq!(
        solid::sdf(field, (3, 3, 3), point(-1), point(1), 0).failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
}

#[test]
/// Requires the same undecided sample category when strict diagnostics are explicitly requested.
fn diagnostic_uncertainty_is_not_nonfinite_empty_geometry() {
    let zero = unknown_zero();
    assert_eq!(
        solid::sdf_with_context(
            |_| zero.clone(),
            (3, 3, 3),
            point(-1),
            point(1),
            0.into(),
            GeometryContext::STRICT,
            csgrs::solid::SampleOptions::with_diagnostics(),
        )
        .failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
}

#[test]
/// Requires square root of a negative constant expression to remain invalid input.
fn invalid_expression_domains_remain_invalid_input() {
    let expr = SdfExpr::constant((-1).into()).sqrt();
    assert_eq!(
        solid::sdf_expr(expr, (3, 3, 3), point(-1), point(1), 0).failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}

#[test]
/// Requires a negative-radius sphere expression to fail through the diagnostic contextual API.
fn diagnostic_expression_domains_remain_invalid_input() {
    let expr = SdfExpr::sphere(
        hyperlimit::Point3::new(0.into(), 0.into(), 0.into()),
        (-1).into(),
    );
    assert_eq!(
        solid::sdf_expr_with_context(
            expr,
            (3, 3, 3),
            point(-1),
            point(1),
            0.into(),
            GeometryContext::STRICT,
            csgrs::solid::SampleOptions::with_diagnostics()
        )
        .failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}

#[test]
/// Requires an unresolved minimum/union branch between equal-valued constants to remain undecided.
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
            GeometryContext::STRICT,
            csgrs::solid::SampleOptions::default()
        )
        .failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
}

#[test]
/// Checks the contextual nondiagnostic expression path keeps square-root domain failure distinct from uncertainty.
fn context_expression_domain_errors_are_distinct_from_uncertainty() {
    let expr = SdfExpr::constant((-1).into()).sqrt();
    assert_eq!(
        solid::sdf_expr_with_context(
            expr,
            (3, 3, 3),
            point(-1),
            point(1),
            0.into(),
            GeometryContext::STRICT,
            csgrs::solid::SampleOptions::default()
        )
        .failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}

#[test]
/// Uses a panicking field callback to require grid-capacity rejection before any sample evaluation.
/// The resource-exhausted category reflects an unaddressable grid, not measured allocator exhaustion.
fn overflow_is_resource_exhaustion_before_sampling() {
    let field =
        |_: &Point3| -> Real { panic!("capacity must be checked before invoking the field") };
    assert_eq!(
        solid::sdf(field, (usize::MAX, 2, 2), point(-1), point(1), 0).failure_kind(),
        Some(ValidationErrorKind::ResourceExhausted)
    );
}

#[test]
/// Requires a one-sample axis to be invalid even for a trivially positive field.
fn undersized_resolution_is_rejected_instead_of_clamped() {
    assert_eq!(
        solid::sdf_with_context(
            |_| Real::one(),
            (1, 3, 3),
            point(-1),
            point(1),
            0.into(),
            GeometryContext::STRICT,
            csgrs::solid::SampleOptions::default()
        )
        .failure_kind(),
        Some(ValidationErrorKind::InvalidInput)
    );
}

#[test]
/// Requires `sqrt(-x)` to fail at grid index `[2,0,0]` with a downcastable `SqrtNegative` cause.
/// A later positive field yields canonical empty, which is consumed by box union/removal; the restored
/// box is checked by bounds, not a full independent set oracle.
fn partial_sampling_failure_keeps_sample_index_and_native_source() {
    use std::error::Error;
    // x visits -1, 0, 1, so sqrt(-x) fails only at the third sample.
    let error = solid::sdf_expr(
        SdfExpr::x().complement().sqrt(),
        (3, 3, 3),
        point(-1),
        point(1),
        0,
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
    let empty =
        solid::sdf_expr(SdfExpr::constant(1.into()), (3, 3, 3), point(-1), point(1), 0)
            .unwrap();
    assert!(empty.positions.is_empty() && empty.triangles.is_empty());
    let cube = solid::cube(1).unwrap();
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
/// Checks the sampled plane `x=1/3`, certified local receipt, 18/0/9 sign counts and raw field extrema -1/1.
/// Preview self-consistency is checked separately; the open plane mesh is not validated as a closed solid.
fn diagnostic_plane_has_independently_known_coordinates_and_counts() {
    // Samples x=-1,0,1 at iso=1/3: 18 negative and 9 positive.
    let third = (Real::one() / Real::from(3)).unwrap();
    let result = solid::sdf_expr_with_context(
        SdfExpr::x(),
        (3, 3, 3),
        point(-1),
        point(1),
        third.clone(),
        GeometryContext::STRICT,
        csgrs::solid::SampleOptions::with_diagnostics(),
    )
    .unwrap();
    assert_eq!(result.certainty, csgrs::GeometryCertainty::Certified);
    let solid::SampledMesh {
        mesh,
        diagnostics: Some(diagnostics),
    } = result.value
    else {
        panic!("requested diagnostics must be returned");
    };
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
/// Runs an unresolved plane through strict failure, qualified extraction and strict failure again.
/// Preview retains nine unknown sample signs; an approximate meshing policy still cannot decide an
/// unresolved expression branch. A positive constant provides a certified local control.
fn approximate_samples_do_not_certify_expression_branches_or_poison_strict_queries() {
    let expr = SdfExpr::x().add_expr(SdfExpr::constant(unknown_zero()));
    let sample = |context| {
        solid::sdf_expr_with_context(
            expr.clone(),
            (3, 3, 3),
            point(-1),
            point(1),
            0.into(),
            context,
            csgrs::solid::SampleOptions::with_diagnostics(),
        )
    };
    assert_eq!(
        sample(GeometryContext::STRICT).failure_kind(),
        Some(ValidationErrorKind::Undecided)
    );
    let approximate = sample(GeometryContext::APPROXIMATE_512).unwrap();
    assert_eq!(
        approximate.certainty,
        csgrs::GeometryCertainty::Approximate512Consumed
    );
    let preview = approximate
        .value
        .diagnostics
        .unwrap()
        .hypersdf_preview
        .unwrap();
    assert!(preview.is_self_consistent());
    assert_eq!(preview.grid_samples.samples.unknown_sign_count, 9);
    assert_eq!(
        sample(GeometryContext::STRICT).failure_kind(),
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
            GeometryContext::APPROXIMATE_512,
            csgrs::solid::SampleOptions::default()
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
        GeometryContext::APPROXIMATE_512,
        csgrs::solid::SampleOptions::default(),
    )
    .unwrap();
    assert_eq!(certified.certainty, csgrs::GeometryCertainty::Certified);
}

#[test]
/// Compares diagnostic-on/off closure and expression sampling of the rational plane under both policies.
/// Requires matching geometry/receipts and selected diagnostic counts, plus ordinary-wrapper agreement;
/// it does not compare every construction-evidence field or diagnostic statistic.
fn diagnostics_are_returned_exactly_when_requested_and_change_nothing_else() {
    use csgrs::solid::SampleOptions;
    let third = (Real::one() / Real::from(3)).unwrap();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        let field = |options| {
            solid::sdf_with_context(
                |p: &Point3| p.x.clone(),
                (3, 3, 3),
                point(-1),
                point(1),
                third.clone(),
                context,
                options,
            )
            .unwrap()
        };
        let plain = field(SampleOptions::default());
        let requested = field(SampleOptions::with_diagnostics());
        assert!(plain.value.diagnostics.is_none());
        assert_eq!(requested.value.diagnostics.unwrap().negative_sample_count, 18);
        assert_eq!(plain.value.mesh, requested.value.mesh);
        assert_eq!(plain.certainty, requested.certainty);
        let strict =
            solid::sdf(|p| p.x.clone(), (3, 3, 3), point(-1), point(1), third.clone());
        assert_eq!(strict.unwrap(), plain.value.mesh);

        let expression = |options| {
            solid::sdf_expr_with_context(
                SdfExpr::x(),
                (3, 3, 3),
                point(-1),
                point(1),
                third.clone(),
                context,
                options,
            )
            .unwrap()
        };
        let plain = expression(SampleOptions::default());
        let requested = expression(SampleOptions::with_diagnostics());
        assert!(plain.value.diagnostics.is_none());
        assert_eq!(requested.value.diagnostics.unwrap().positive_sample_count, 9);
        assert_eq!(plain.value.mesh, requested.value.mesh);
        assert_eq!(plain.certainty, requested.certainty);
    }
}
