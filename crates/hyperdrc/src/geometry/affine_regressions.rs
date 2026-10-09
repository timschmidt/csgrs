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
        .evaluate(|| hypercurve::provisional(transform));
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

#[test]
fn fresh_zero_budget_allows_transforms_that_need_no_refinement() {
    let polygon = circle_polygon([0.0, 0.0], 2.0, 17);
    let transformed =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            transform_polygon(&polygon, [3.0, -4.0], 30.0)
        });
    assert_eq!(transformed.usage, hypercurve::WorkUsage::default());
    let transformed = transformed.result.unwrap().certified().unwrap();
    assert_eq!(
        crate::scalar::try_compare(
            &transformed.try_area().unwrap(),
            &(Scalar::from(4) * Scalar::pi()),
        )
        .unwrap(),
        std::cmp::Ordering::Equal
    );
}

#[test]
fn empty_transform_distinguishes_valid_empty_geometry_from_failed_source() {
    let empty = circle_polygon([0.0, 0.0], 0.0, 17);
    let transformed = transform_polygon(&empty, [3.0, -4.0], 90.0);
    assert!(transformed.try_exact_region().unwrap().is_empty());
    assert_eq!(transformed.try_area().unwrap(), Scalar::zero());
    assert_eq!(transformed.try_bounds().unwrap(), None);
    let failed = empty.with_exact_construction_error("failed empty affine input");
    let transformed = transform_polygon(&failed, [3.0, -4.0], 90.0);
    assert!(transformed.try_exact_region().is_err());
    assert!(transformed.try_area().is_err());
    assert!(transformed.try_bounds().is_err());
}

#[test]
fn affine_rotation_preserves_material_hole_and_boundary_membership() {
    let outer = rect_polygon([2.0, 2.0], [4.0, 4.0], 0.0);
    let hole = rect_polygon([1.5, 1.5], [1.0, 1.0], 0.0);
    let polygon = super::Polygon::new(outer.exterior().clone(), vec![hole.exterior().clone()]);
    let transformed = transform_polygon(&polygon, [10.0, 20.0], 90.0);
    assert_eq!(transformed.interiors().len(), 1);
    assert_eq!(transformed.try_area().unwrap(), Scalar::from(15));
    assert_eq!(
        transformed.try_bounds().unwrap(),
        Some([6.into(), 20.into(), 10.into(), 24.into()])
    );
    let profile = polygons_to_profile(vec![transformed], None);
    use hypercurve::RegionPointLocation::{Boundary, Inside, Outside};
    for (x, y, expected) in [
        ("7", "23", Inside),
        ("8.5", "21.5", Outside),
        ("9", "21.5", Boundary),
    ] {
        assert_eq!(
            profile
                .classify_xy(crate::scalar::scalar(x), crate::scalar::scalar(y))
                .unwrap(),
            expected
        );
    }
}

#[test]
fn full_degree_five_native_transform_does_not_substitute_report_chords() {
    use hypercurve::{
        Curve2, CurvePath2, CurveRegion2, CurveRegionLoopRole, FillRule, LineSeg2, Point2,
        RationalBezier2,
    };
    // The complete cap used by the native area regression: W(t) = 1 + 5t + t^5.
    // Its integral is unavailable. A report triangle must never replace it.
    let curve = RationalBezier2::try_new(
        [(0, 0), (1, 1), (2, 2), (3, 2), (4, 1), (5, 0)]
            .into_iter()
            .map(|(x, y)| Point2::from_values(x, y))
            .collect(),
        [1, 2, 3, 4, 5, 7].into_iter().map(Scalar::from).collect(),
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![
        Curve2::from(curve),
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(5, 0), Point2::from_values(0, 0)).unwrap(),
        ),
    ])
    .unwrap();
    let native = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();
    let view =
        super::polygon_from_points(vec![[0.0, 0.0], [5.0, 0.0], [325.0 / 113.0, 175.0 / 113.0]]);
    let polygon = super::Polygon::from_exact_region(native, view.exterior().clone(), vec![]);
    assert!(polygon.try_area().is_err());
    let transformed = transform_polygon(&polygon, [10.0, 20.0], 90.0);
    assert!(transformed.try_exact_region().is_ok());
    assert!(transformed.unsigned_area() > 0.0);
    assert!(transformed.try_area().is_err());
    assert!(
        polygons_to_profile(vec![transformed], None)
            .try_area()
            .is_err()
    );
}

#[test]
fn shared_affine_sources_keep_thread_budget_and_preview_scopes_separate() {
    let polygon = circle_polygon([0.0, 0.0], 2.0, 17);
    polygon.try_area().unwrap();
    polygon.try_bounds().unwrap();
    std::thread::scope(|scope| {
        for denied in [true, false, true, false] {
            let polygon = &polygon;
            scope.spawn(move || {
                if denied {
                    let mut escaped = None;
                    let result = hypercurve::evaluate_with_budget(
                        PREDICATE_POLICY,
                        hypercurve::WorkBudget::ZERO,
                        || {
                            deny_refinement();
                            escaped = Some(transform_polygon(polygon, [3.0, -4.0], 90.0));
                        },
                    );
                    assert!(result.result.is_err());
                    assert!(escaped.unwrap().try_exact_region().is_err());
                } else {
                    let transformed =
                        hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
                            .unwrap()
                            .evaluate(|| transform_polygon(polygon, [3.0, -4.0], 90.0));
                    assert_eq!(
                        transformed.try_bounds().unwrap(),
                        Some([1.into(), (-6).into(), 5.into(), (-2).into()])
                    );
                }
            });
        }
    });
    assert!(polygon.try_area().is_ok());
    assert!(polygon.try_bounds().is_ok());
}

#[test]
fn denied_affine_geometry_reaches_a_blocking_readiness_receipt() {
    let polygon = rect_polygon([0.0, 0.0], [2.0, 4.0], 0.0);
    let mut escaped = None;
    let denied =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            deny_refinement();
            escaped = Some(transform_polygon(&polygon, [10.0, 20.0], 90.0));
        });
    assert!(denied.result.is_err());
    let profile = polygons_to_profile(vec![escaped.unwrap()], None);
    let mut findings = Vec::new();
    let coverage = crate::ReadinessRunner::new([crate::Check::LayerSanity])
        .run(&mut findings, |_, sink| {
            sink.extend(crate::checks::skinny_layer_feature_readiness(
                "F.Cu",
                &profile,
                &2.into(),
                &Scalar::zero(),
            ));
            Ok::<_, std::convert::Infallible>(crate::CheckRunDisposition::Executed)
        })
        .unwrap();
    assert_eq!(
        coverage.checks[0].status,
        crate::CheckExecutionStatus::Uncertain
    );
    assert_eq!(coverage.blocking_count(), 1);
    assert_eq!(findings[0].check, "geometry-uncertainty");
    assert_eq!(findings[0].layers, ["F.Cu"]);
}
