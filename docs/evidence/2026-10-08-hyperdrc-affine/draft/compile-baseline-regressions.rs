use super::types::transform_exact_region;
use super::{circle_polygon, polygons_to_profile, rect_polygon, transform_polygon};
use crate::{PREDICATE_POLICY, Scalar};

fn deny_refinement() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-36, 1368, -11034, 31728, -38280, 16620]
            .into_iter()
            .map(Scalar::from)
            .collect(),
    )
    .unwrap();
    assert!(matches!(
        polynomial.isolate_unit_interval_roots(),
        Err(hypercurve::ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted
    ));
}

fn denied_transform(origin: [f64; 2], angle: f64) {
    for warm in [false, true] {
        let polygon = rect_polygon([0.0, 0.0], [2.0, 4.0], 0.0);
        if warm {
            assert_eq!(polygon.try_area().unwrap(), Scalar::from(8));
            assert!(polygon.try_bounds().unwrap().is_some());
            let previous = transform_polygon(&polygon, origin, angle);
            assert_eq!(previous.try_area().unwrap(), Scalar::from(8));
        }
        let mut escaped = None;
        let denied = hypercurve::evaluate_with_budget(
            PREDICATE_POLICY,
            hypercurve::WorkBudget::ZERO,
            || {
                deny_refinement();
                escaped = Some(transform_polygon(&polygon, origin, angle));
            },
        );
        assert!(denied.result.is_err());
        let escaped = escaped.unwrap();
        assert!(
            escaped.try_exact_region().is_err(),
            "denied transform escaped"
        );
        assert!(escaped.try_area().is_err());
        assert!(escaped.try_bounds().is_err());
        assert!(
            polygons_to_profile(vec![escaped], None)
                .try_into_native()
                .is_err()
        );
        let retry = transform_polygon(&polygon, origin, angle);
        assert_eq!(retry.try_area().unwrap(), Scalar::from(8));
    }
}

#[test]
fn exhausted_identity_transform_cannot_escape_as_success() {
    denied_transform([0.0, 0.0], 0.0);
}

#[test]
fn exhausted_rotated_transform_cannot_escape_as_success() {
    denied_transform([10.0, 20.0], 90.0);
}

#[test]
fn native_affine_helper_rejects_retained_source_failure() {
    let polygon = rect_polygon([0.0, 0.0], [2.0, 4.0], 0.0)
        .with_exact_construction_error("retained affine input failure");
    assert!(
        transform_exact_region(
            &polygon,
            Scalar::one(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
        )
        .is_err()
    );
}

#[test]
fn native_affine_policy_is_independent_of_enclosing_preview() {
    let polygon = rect_polygon([0.0, 0.0], [2.0, 4.0], 0.0);
    let sine = Scalar::e().sin();
    let cosine = Scalar::e().cos();
    let zero = &sine * &sine + &cosine * &cosine - Scalar::one();
    let transform = || {
        transform_exact_region(
            &polygon,
            zero.clone(),
            Scalar::zero(),
            Scalar::zero(),
            Scalar::zero(),
        )
        .unwrap_err()
        .to_string()
    };
    let strict = transform();
    let preview = hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
        .unwrap()
        .evaluate(transform);
    assert!(
        preview.is_certified(),
        "affine transform consumed a preview decision"
    );
    assert_eq!(preview.certified().unwrap(), strict);
}

#[test]
fn public_affine_wrapper_retains_source_error() {
    let polygon = rect_polygon([0.0, 0.0], [2.0, 4.0], 0.0)
        .with_exact_construction_error("retained affine input failure");
    let transformed = transform_polygon(&polygon, [10.0, 20.0], 90.0);
    assert!(transformed.try_area().is_err());
    assert!(
        transformed
            .exact_construction_error()
            .unwrap()
            .contains("retained affine input failure")
    );
}

#[test]
fn native_affine_circle_geometry_does_not_depend_on_report_samples() {
    for samples in [17, 128] {
        let polygon = circle_polygon([0.0, 0.0], 2.0, samples);
        let transformed = transform_polygon(&polygon, [3.0, -4.0], 90.0);
        assert_eq!(transformed.exterior().0.len(), samples + 1);
        assert_eq!(
            crate::scalar::try_compare(
                &transformed.try_area().unwrap(),
                &(Scalar::from(4) * Scalar::pi()),
            )
            .unwrap(),
            std::cmp::Ordering::Equal
        );
        assert_eq!(
            transformed.try_bounds().unwrap(),
            Some([1.into(), (-6).into(), 5.into(), (-2).into()])
        );
    }
}
