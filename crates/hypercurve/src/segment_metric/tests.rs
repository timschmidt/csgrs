use super::*;
use crate::{CurveCertainty, PredicatePolicy, WorkBudget, evaluate_under, evaluate_with_budget};

fn p(x: i32, y: i32) -> Point2 {
    Point2::from_values(x, y)
}

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn line(start: Point2, end: Point2) -> Segment2 {
    Segment2::Line(LineSeg2::new_unchecked(start, end))
}

fn arc(start: Point2, end: Point2, center: Point2) -> Segment2 {
    Segment2::Arc(CircularArc2::try_from_center(start, end, center, false).unwrap())
}

fn upper_unit_arc() -> Segment2 {
    arc(p(1, 0), p(-1, 0), p(0, 0))
}

fn assert_distance(left: &Segment2, right: &Segment2, expected: &Real) {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (left, right) in [(left, right), (right, left)] {
            for left in [left.clone(), left.reversed()] {
                for right in [right.clone(), right.reversed()] {
                    let measured = evaluate_under(policy, || left.distance_squared(&right));
                    assert_eq!(measured.certainty(), CurveCertainty::Certified);
                    let measured = measured.certified().unwrap().unwrap();
                    assert_eq!(
                        hyperlimit::compare_reals(&measured, expected, PredicatePolicy::STRICT)
                            .value(),
                        Some(Ordering::Equal),
                        "distance {measured:?}, expected {expected:?}; {left:?} / {right:?}"
                    );
                }
            }
        }
    }
}

fn assert_point_distance(segment: &Segment2, point: &Point2, expected: &Real) {
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for segment in [segment.clone(), segment.reversed()] {
            let measured = evaluate_under(policy, || segment.point_distance_squared(point));
            assert_eq!(measured.certainty(), CurveCertainty::Certified);
            assert_eq!(
                hyperlimit::compare_reals(
                    &measured.certified().unwrap().unwrap(),
                    expected,
                    PredicatePolicy::STRICT,
                )
                .value(),
                Some(Ordering::Equal)
            );
        }
    }
}

#[test]
fn line_metrics_cover_interior_endpoints_crossings_and_singletons() {
    let horizontal = line(p(0, 0), p(2, 0));
    for (other, expected) in [
        (line(p(0, 3), p(2, 3)), 9),
        (line(p(3, 1), p(3, 3)), 2),
        (line(p(1, -1), p(1, 1)), 0),
        (line(p(1, 0), p(3, 0)), 0),
        (line(p(2, 0), p(3, 1)), 0),
        (line(p(1, 2), p(1, 2)), 4),
    ] {
        assert_distance(&horizontal, &other, &Real::from(expected));
    }
    assert_distance(&line(p(1, 2), p(1, 2)), &line(p(4, 6), p(4, 6)), &25.into());
    // The supporting line is y = 3x/4; (0, 5) projects into its finite span.
    assert_point_distance(&line(p(0, 0), p(4, 3)), &p(0, 5), &16.into());
    for (point, expected) in [(p(-1, 1), 2), (p(1, 1), 1), (p(3, 1), 2)] {
        assert_point_distance(&horizontal, &point, &expected.into());
    }
}

#[test]
fn point_arc_metrics_retain_radial_and_endpoint_domains() {
    let upper = upper_unit_arc();
    for (point, expected) in [
        (p(0, 2), Real::one()),
        (p(0, -2), 5.into()),
        (p(0, 0), Real::one()),
        (Point2::new(q(1, 2), Real::zero()), q(1, 4)),
        (Point2::new(q(3, 5), q(4, 5)), Real::zero()),
        (
            p(1, 1),
            Real::from(3) - Real::from(2) * Real::from(2).sqrt().unwrap(),
        ),
    ] {
        assert_point_distance(&upper, &point, &expected);
    }
    let major = arc(p(1, 0), p(0, -1), p(0, 0));
    assert_point_distance(&major, &p(0, 2), &Real::one());
    assert_point_distance(&major, &Point2::new(q(3, 5), -q(4, 5)), &q(2, 5));
}

#[test]
fn line_arc_metrics_include_stationary_feet_and_reject_outside_feet() {
    let upper = upper_unit_arc();
    for (other, expected) in [
        (line(p(-2, 3), p(2, 3)), Real::from(4)),
        (line(p(-2, 1), p(2, 1)), Real::zero()),
        (line(p(0, -2), p(0, 2)), Real::zero()),
        (line(p(-2, -2), p(2, -2)), Real::from(4)),
        (
            line(p(2, 2), p(3, 2)),
            Real::from(9) - Real::from(4) * Real::from(2).sqrt().unwrap(),
        ),
        (
            line(
                Point2::new(-q(1, 10), Real::zero()),
                Point2::new(q(1, 10), Real::zero()),
            ),
            q(81, 100),
        ),
        (line(p(0, 0), p(0, 0)), Real::one()),
    ] {
        assert_distance(&upper, &other, &expected);
    }
    // The full circle approaches the line at y=1, outside this lower arc.
    let lower = arc(p(-1, 0), p(1, 0), p(0, 0));
    assert_distance(&lower, &line(p(-2, 3), p(2, 3)), &9.into());
}

#[test]
fn arc_arc_metrics_cover_stationary_concentric_nested_and_contact_pairs() {
    let facing_right = arc(p(0, -1), p(0, 1), p(0, 0));
    let facing_left = arc(p(5, 2), p(5, -2), p(5, 0));
    assert_distance(&facing_right, &facing_left, &4.into());
    let outer_left = arc(p(0, 5), p(0, -5), p(0, 0));
    let inner_left = arc(p(2, 1), p(2, -1), p(2, 0));
    // The inner circle is nested; closest pair on these left semicircles is
    // attained at endpoints rather than the inadmissible right-hand normals.
    assert_distance(
        &outer_left,
        &inner_left,
        &(Real::from(30) - Real::from(2) * Real::from(29).sqrt().unwrap()),
    );
    assert_distance(
        &arc(p(0, -5), p(0, 5), p(0, 0)),
        &arc(p(2, -1), p(2, 1), p(2, 0)),
        &4.into(),
    );
    let upper = upper_unit_arc();
    assert_distance(&upper, &upper, &Real::zero());
    assert_distance(&upper, &arc(p(-1, 0), p(1, 0), p(0, 0)), &Real::zero());
    assert_distance(&upper, &arc(p(2, 0), p(-2, 0), p(0, 0)), &Real::one());
    assert_distance(&upper, &arc(p(3, 0), p(1, 0), p(2, 0)), &Real::zero());
    assert_distance(&upper, &arc(p(2, 0), p(0, 0), p(1, 0)), &Real::zero());
    let first_quadrant = arc(p(1, 0), p(0, 1), p(0, 0));
    let third_quadrant = arc(p(-1, 0), p(0, -1), p(0, 0));
    assert_distance(&first_quadrant, &third_quadrant, &2.into());
    assert_distance(&arc(p(2, 0), p(0, 2), p(0, 0)), &third_quadrant, &5.into());
}

fn transformed_point(point: &Point2, scale: &Real, translation: &Real) -> Point2 {
    let cosine = q(3, 5);
    let sine = q(4, 5);
    Point2::new(
        scale * (point.x() * &cosine - point.y() * &sine) + translation,
        scale * (point.x() * &sine + point.y() * &cosine) - translation,
    )
}

fn transformed(segment: &Segment2, scale: &Real, translation: &Real) -> Segment2 {
    let point = |point| transformed_point(point, scale, translation);
    match segment {
        Segment2::Line(value) => line(point(value.start()), point(value.end())),
        Segment2::Arc(value) => Segment2::Arc(
            CircularArc2::try_from_center(
                point(value.start()),
                point(value.end()),
                point(value.center()),
                value.is_clockwise(),
            )
            .unwrap(),
        ),
    }
}

#[test]
fn exact_rotation_large_translation_and_tiny_separation_preserve_distances() {
    let translation = Real::from(10_i64).powi_i64(20).unwrap();
    for scale in [
        q(1, 1_000_000_000_000),
        Real::one(),
        Real::from(13).sqrt().unwrap(),
    ] {
        let left = transformed(&upper_unit_arc(), &scale, &translation);
        let right = transformed(&line(p(-2, 3), p(2, 3)), &scale, &translation);
        assert_distance(&left, &right, &(Real::from(4) * &scale * &scale));
        let first = transformed(&arc(p(0, -1), p(0, 1), p(0, 0)), &scale, &translation);
        let second = transformed(&arc(p(5, 2), p(5, -2), p(5, 0)), &scale, &translation);
        assert_distance(&first, &second, &(Real::from(4) * &scale * &scale));
    }
    let tiny = q(1, 1_000_000_000_000);
    assert_distance(
        &line(p(0, 0), p(1, 0)),
        &line(
            Point2::new(Real::zero(), tiny.clone()),
            Point2::new(Real::one(), tiny.clone()),
        ),
        &(&tiny * &tiny),
    );
}

#[test]
fn uncertain_candidate_order_is_not_replaced_with_an_endpoint_or_zero() {
    let unresolved = crate::test_support::terminally_unresolved_zero();
    let point = Point2::new(unresolved, -Real::one());
    for segment in [upper_unit_arc(), line(p(0, 0), p(1, 0))] {
        for policy in [
            PredicatePolicy::STRICT,
            PredicatePolicy::APPROXIMATE_512,
            PredicatePolicy::STRICT,
        ] {
            let result = evaluate_under(policy, || segment.point_distance_squared(&point));
            if policy == PredicatePolicy::STRICT {
                assert!(matches!(
                    result.certified().unwrap(),
                    Err(ExactCurveError::Blocked(_))
                ));
            } else {
                assert_eq!(result.certainty(), CurveCertainty::Approximate512Consumed);
                assert!(result.into_unverified().is_ok());
            }
        }
    }
}

fn circle_halves(x: i32, y: i32, radius: i32) -> [Segment2; 2] {
    // An oblique rational seam keeps axis extrema strictly inside the arcs.
    let dx = Real::from(radius) * q(3, 5);
    let dy = Real::from(radius) * q(4, 5);
    let first = Point2::new(Real::from(x) + &dx, Real::from(y) + &dy);
    let second = Point2::new(Real::from(x) - &dx, Real::from(y) - &dy);
    [
        arc(first.clone(), second.clone(), p(x, y)),
        arc(second, first, p(x, y)),
    ]
}

fn minimum(values: impl IntoIterator<Item = Real>) -> Real {
    values
        .into_iter()
        .reduce(|left, right| {
            if hyperlimit::compare_reals(&left, &right, PredicatePolicy::STRICT)
                .value()
                .unwrap()
                == Ordering::Greater
            {
                right
            } else {
                left
            }
        })
        .unwrap()
}

#[test]
fn split_circle_pairs_match_independent_full_circle_distance_oracle() {
    for (x, y) in [
        (0, 0),
        (1, 0),
        (1, 1),
        (2, 0),
        (2, 1),
        (2, 2),
        (3, 0),
        (3, 1),
        (3, 2),
        (4, 0),
        (3, 4),
    ] {
        let squared = x * x + y * y;
        let distance = Real::from(squared).sqrt().unwrap();
        let gap = if squared > 9 {
            distance - Real::from(3)
        } else if squared < 1 {
            Real::one() - distance
        } else {
            Real::zero()
        };
        let left = circle_halves(0, 0, 1);
        let right = circle_halves(x, y, 2);
        let actual = minimum(left.iter().flat_map(|left| {
            right
                .iter()
                .map(move |right| left.distance_squared(right).unwrap())
        }));
        assert_eq!(
            hyperlimit::compare_reals(&actual, &(&gap * &gap), PredicatePolicy::STRICT).value(),
            Some(Ordering::Equal),
            "centers at ({x}, {y})"
        );
    }
}

#[test]
fn split_circles_match_independent_horizontal_segment_distance_oracle() {
    let segment = line(p(-2, 0), p(2, 0));
    for radius in [1, 2, 5] {
        for x in -3_i32..=3 {
            for y in -3_i32..=3 {
                let min_squared = (x.abs() - 2).max(0).pow(2) + y * y;
                let max_squared = (x.abs() + 2).pow(2) + y * y;
                let gap = if min_squared > radius * radius {
                    Real::from(min_squared).sqrt().unwrap() - Real::from(radius)
                } else if max_squared < radius * radius {
                    Real::from(radius) - Real::from(max_squared).sqrt().unwrap()
                } else {
                    Real::zero()
                };
                let actual = minimum(
                    circle_halves(x, y, radius)
                        .iter()
                        .map(|arc| segment.distance_squared(arc).unwrap()),
                );
                assert_eq!(
                    hyperlimit::compare_reals(&actual, &(&gap * &gap), PredicatePolicy::STRICT)
                        .value(),
                    Some(Ordering::Equal),
                    "circle ({x}, {y}), radius {radius}"
                );
            }
        }
    }
}

#[test]
fn exhausted_scope_blocks_queries_without_poisoning_retry() {
    let left = upper_unit_arc();
    let right = line(p(-2, 3), p(2, 3));
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let exhausted = evaluate_with_budget(policy, WorkBudget::ZERO, || {
            assert!(
                hypersolve::work_budget::consume(crate::WorkKind::AlgebraicPromotion, 1).is_err()
            );
            for result in [
                left.distance_squared(&right),
                left.point_distance_squared(&p(0, 2)),
            ] {
                assert!(
                    matches!(result, Err(ExactCurveError::Blocked(blocker)) if blocker.reason() == UncertaintyReason::BudgetExhausted)
                );
            }
        });
        assert!(exhausted.result.is_err());
        let retry =
            evaluate_with_budget(policy, WorkBudget::ZERO, || left.distance_squared(&right));
        assert_eq!(
            retry.result.unwrap().certified().unwrap().unwrap(),
            Real::from(4)
        );
        assert_eq!(retry.usage, crate::WorkUsage::default());
    }
}

#[test]
fn nested_exhaustion_survives_success_and_numeric_error_adapters() {
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for numeric_error in [false, true] {
            let attempt = hypersolve::work_budget::with_budget(WorkBudget::ZERO, || {
                let result = MetricContext(policy).evaluate(|| {
                    let _ = hypersolve::work_budget::consume(crate::WorkKind::RootRefinement, 1);
                    if numeric_error {
                        Err(MetricContext::invalid(CurveError::Real(
                            "nested adapter could not construct a scalar".into(),
                        )))
                    } else {
                        Ok(Real::zero())
                    }
                });
                assert!(
                    matches!(result, Err(ExactCurveError::Blocked(blocker)) if blocker.reason() == UncertaintyReason::BudgetExhausted)
                );
            });
            assert_eq!(
                attempt.result.unwrap_err().kind,
                crate::WorkKind::RootRefinement
            );
            assert_eq!(attempt.usage, crate::WorkUsage::default());
        }
    }
}

#[test]
fn explicit_metric_policy_suspends_enclosing_display_preview() {
    let gap = q(1, 1_000_000_000_000);
    let line = line(
        Point2::new((-2).into(), Real::one() + &gap),
        Point2::new(2.into(), Real::one() + &gap),
    );
    let arc = upper_unit_arc();
    for preview in [
        crate::CurvePreviewOptions::try_strict(0.1, 0.1).unwrap(),
        crate::CurvePreviewOptions::try_approximate_512(0.1, 0.1).unwrap(),
    ] {
        preview.evaluate(|| {
            let before = crate::policy::principal_context();
            assert!(before.is_edge_preview());
            assert_distance(&line, &arc, &(&gap * &gap));
            assert_point_distance(
                &arc,
                &Point2::new(Real::zero(), Real::one() + &gap),
                &(&gap * &gap),
            );
            assert_eq!(crate::policy::principal_context(), before);
        });
    }
}
