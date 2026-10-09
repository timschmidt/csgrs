use super::*;
use crate::geometry::polygon_area_scalar;

fn square() -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 1.0, y: 0.0 },
            Coord { x: 1.0, y: 1.0 },
            Coord { x: 0.0, y: 1.0 },
            Coord { x: 0.0, y: 0.0 },
        ]),
        Vec::new(),
    )
}

fn deny_refinement() {
    let polynomial = hypercurve::BezierParameterPolynomial::try_new_power_basis(
        [-36, 1368, -11034, 31728, -38280, 16620]
            .into_iter()
            .map(Real::from)
            .collect(),
    )
    .unwrap();
    assert!(matches!(
        polynomial.isolate_unit_interval_roots(),
        Err(hypercurve::ExactCurveError::Blocked(blocker))
            if blocker.reason() == hypercurve::UncertaintyReason::BudgetExhausted
    ));
}

#[test]
fn failed_source_construction_does_not_publish_placeholder_area_or_bounds() {
    let invalid = Polygon::new(LineString(Vec::new()), Vec::new());
    assert!(invalid.exact_construction_error().is_some());
    assert!(invalid.exact_area().is_err());
    assert!(polygon_area_scalar(&invalid).is_err());
    assert!(invalid.exact_bounds().is_err());

    let warm = square();
    assert_eq!(warm.exact_area(), Ok(&Real::one()));
    assert!(warm.exact_bounds().unwrap().is_some());
    let valid_clone = warm.clone();
    let invalid = warm.with_exact_construction_error("retained source failure");
    assert!(invalid.exact_area().is_err());
    assert!(polygon_area_scalar(&invalid).is_err());
    assert!(invalid.exact_bounds().is_err());
    assert_eq!(valid_clone.exact_area(), Ok(&Real::one()));
    assert!(valid_clone.exact_bounds().unwrap().is_some());
}

#[test]
fn denied_area_query_remains_retryable_and_cannot_use_report_area() {
    for warm in [false, true] {
        let polygon = square();
        if warm {
            assert_eq!(polygon.exact_area(), Ok(&Real::one()));
        }
        let mut during = None;
        let denied = hypercurve::evaluate_with_budget(
            PREDICATE_POLICY,
            hypercurve::WorkBudget::ZERO,
            || {
                deny_refinement();
                during = Some((polygon.exact_area().cloned(), polygon_area_scalar(&polygon)));
            },
        );
        assert!(denied.result.is_err());
        let (cached, measured) = during.unwrap();
        assert!(cached.is_err());
        assert!(measured.is_err());
        assert_eq!(polygon.exact_area(), Ok(&Real::one()));
        assert_eq!(polygon_area_scalar(&polygon), Ok(Real::one()));
    }
}

#[test]
fn denied_bounds_query_remains_retryable() {
    for warm in [false, true] {
        let polygon = square();
        if warm {
            assert!(polygon.exact_bounds().unwrap().is_some());
        }
        let mut during = None;
        let denied = hypercurve::evaluate_with_budget(
            PREDICATE_POLICY,
            hypercurve::WorkBudget::ZERO,
            || {
                deny_refinement();
                during = Some(polygon.try_bounds());
            },
        );
        assert!(denied.result.is_err());
        assert!(during.unwrap().is_err());
        assert_eq!(
            polygon.exact_bounds().unwrap(),
            Some(&[0.into(), 0.into(), 1.into(), 1.into()])
        );
    }
}

#[test]
fn shared_measurement_caches_keep_thread_budgets_and_preview_scopes_separate() {
    let polygon = Polygon::from_exact_region(
        crate::geometry::circle_region(&[Real::zero(), Real::zero()], &(Real::one())).unwrap(),
        LineString::from(vec![
            Coord { x: 1.0, y: 0.0 },
            Coord { x: 0.0, y: 1.0 },
            Coord { x: -1.0, y: 0.0 },
            Coord { x: 0.0, y: -1.0 },
            Coord { x: 1.0, y: 0.0 },
        ]),
        Vec::new(),
    );
    assert_eq!(polygon.unsigned_area(), 2.0);
    std::thread::scope(|scope| {
        for denied in [true, false, true, false] {
            let polygon = &polygon;
            scope.spawn(move || {
                if denied {
                    let result = hypercurve::evaluate_with_budget(
                        PREDICATE_POLICY,
                        hypercurve::WorkBudget::ZERO,
                        || {
                            deny_refinement();
                            assert!(polygon.exact_area().is_err());
                            assert!(polygon.exact_bounds().is_err());
                        },
                    );
                    assert!(result.result.is_err());
                } else {
                    hypercurve::CurvePreviewOptions::try_approximate_512(1.0, 1.0)
                        .unwrap()
                        .evaluate(|| {
                            assert_eq!(
                                crate::scalar::compare(polygon.exact_area().unwrap(), &Real::pi()),
                                Some(std::cmp::Ordering::Equal)
                            );
                            assert_eq!(
                                polygon.exact_bounds().unwrap(),
                                Some(&[(-1).into(), (-1).into(), 1.into(), 1.into()])
                            );
                        });
                }
            });
        }
    });
    let cached =
        hypercurve::evaluate_with_budget(PREDICATE_POLICY, hypercurve::WorkBudget::ZERO, || {
            (polygon.exact_area().cloned(), polygon.try_bounds())
        });
    assert_eq!(cached.usage, hypercurve::WorkUsage::default());
    let (area, bounds) = cached.result.unwrap().certified().unwrap();
    assert_eq!(
        crate::scalar::compare(&area.unwrap(), &Real::pi()),
        Some(std::cmp::Ordering::Equal)
    );
    assert_eq!(
        bounds.unwrap(),
        Some([(-1).into(), (-1).into(), 1.into(), 1.into()])
    );
}

#[test]
fn unavailable_native_area_cannot_be_replaced_with_report_chords() {
    use hypercurve::{
        Curve2, CurvePath2, CurveRegionLoopRole, FillRule, LineSeg2, Point2, RationalBezier2,
    };

    // This full degree-five cap is also retained by Hypercurve's exact-closure
    // suite. W(t) = 1 + 5t + t^5 is positive, but its native area currently has
    // no scalar representation. The finite triangle samples t = 0, 1/2, 1.
    let curve = RationalBezier2::try_new(
        [(0, 0), (1, 1), (2, 2), (3, 2), (4, 1), (5, 0)]
            .into_iter()
            .map(|(x, y)| Point2::from_values(x, y))
            .collect(),
        [1, 2, 3, 4, 5, 7].into_iter().map(Real::from).collect(),
    )
    .unwrap();
    let path = CurvePath2::try_new(vec![
        Curve2::from(curve),
        Curve2::from(
            LineSeg2::try_new(Point2::from_values(5, 0), Point2::from_values(0, 0)).unwrap(),
        ),
    ])
    .unwrap();
    let region = CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path],
        &[CurveRegionLoopRole::Material],
        &[FillRule::NonZero],
    )
    .unwrap();
    assert_eq!(region.filled_area().unwrap(), None);
    let pcb = crate::PcbRegion::new(region.clone(), None);
    let error = pcb.try_area().unwrap_err();
    assert_eq!(error.operation, "region-area");
    assert!(error.detail.contains("area integral unavailable"));
    let polygon = Polygon::from_exact_region(
        region,
        LineString::from(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 5.0, y: 0.0 },
            Coord {
                x: 325.0 / 113.0,
                y: 175.0 / 113.0,
            },
            Coord { x: 0.0, y: 0.0 },
        ]),
        Vec::new(),
    );
    assert!(polygon.unsigned_area() > 0.0);
    assert!(polygon.exact_area().is_err());
    assert!(polygon_area_scalar(&polygon).is_err());
}
