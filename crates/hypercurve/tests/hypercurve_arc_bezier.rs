mod support;
use hypercurve::{
    CircularArc2, Classification, Curve2, CurveContext, CurveGeometry2, CurvePath2, LineSeg2,
    Point2, Real, UncertaintyReason,
};
use hypercurve::{CurveCertainty, CurveFamily2, CurveOperation2, ExactCurveError};
use hyperreal::RealSign;
use std::cmp::Ordering;

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(r(x), r(y))
}

fn half() -> Real {
    (r(1) / r(2)).unwrap()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (r(numerator) / r(denominator)).unwrap()
}

fn assert_replayed_containment(classification: Classification<bool>) {
    assert_eq!(classification, Classification::Decided(true));
}

#[test]
fn quarter_arc_decomposes_to_one_exact_conic() {
    let arc = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), false).unwrap();
    let decomposition = arc.rational_bezier_decomposition().unwrap();

    assert_eq!(decomposition.spans().len(), 1);
    let span = &decomposition.spans()[0];
    assert_eq!(
        span.curve().control_weight().structural_facts().sign,
        Some(RealSign::Positive)
    );
    assert_eq!(span.parameter_range(), (&r(0), &r(1)));
    assert_eq!(span.curve().control(), &p(1, 1));
    assert_eq!(span.curve().weights(), [&r(1), &r(1), &r(2)]);
    let third = (r(1) / r(3)).unwrap();
    let point = decomposition
        .point_at(&third, &CurveContext::STRICT)
        .unwrap()
        .into_value();
    assert_eq!(point.x().partial_cmp(point.y()), Some(Ordering::Greater));
    assert_eq!(point.x().partial_cmp(&r(1)), Some(Ordering::Less));
    assert_eq!(point.y().partial_cmp(&r(0)), Some(Ordering::Greater));
}

#[test]
fn semicircle_uses_two_quarter_spans_with_exact_join() {
    let arc = CircularArc2::try_from_center(p(1, 0), p(-1, 0), p(0, 0), false).unwrap();
    let decomposition = arc.rational_bezier_decomposition().unwrap();

    assert_eq!(decomposition.spans().len(), 2);
    assert_eq!(decomposition.spans()[0].parameter_range(), (&r(0), &half()));
    assert_eq!(decomposition.spans()[1].parameter_range(), (&half(), &r(1)));
    assert_eq!(
        decomposition
            .point_at(&half(), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(0, 1)
    );
    assert_eq!(
        crate::support::under_classified_result(&CurveContext::STRICT, || arc
            .representative_point())
        .unwrap(),
        Classification::Decided(p(0, 1))
    );
}

#[test]
fn rationally_trimmed_semicircle_redecomposes_exactly() {
    let source = Curve2::from(CircularArc2::from_bulge(p(0, 0), p(2, 0), r(1)).unwrap());
    let quarter = (r(1) / r(4)).unwrap();
    let three_quarters = (r(3) / r(4)).unwrap();
    let trimmed = source
        .subcurve(quarter.into(), three_quarters.into())
        .unwrap();
    let Some(CurveGeometry2::CircularArc(arc)) = trimmed.geometry() else {
        panic!("trimmed arc changed family");
    };

    let decomposition = arc.rational_bezier_decomposition().unwrap();
    assert!(!decomposition.spans().is_empty());
    assert_eq!(
        decomposition
            .point_at(&r(0), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        arc.start().clone()
    );
    assert_eq!(
        decomposition
            .point_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        arc.end().clone()
    );
}

#[test]
fn major_arc_preserves_rational_charts_and_requested_orientation() {
    let arc = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), true).unwrap();
    let decomposition = arc.rational_bezier_decomposition().unwrap();
    let expected_midpoint = Point2::new(-q(4, 5), -q(3, 5));

    assert_eq!(decomposition.spans().len(), 3);
    for (span, (start, end)) in decomposition.spans().iter().zip([
        (p(1, 0), p(0, -1)),
        (p(0, -1), p(-1, 0)),
        (p(-1, 0), p(0, 1)),
    ]) {
        assert_eq!(span.curve().start(), &start);
        assert_eq!(span.curve().end(), &end);
        for point in span.curve().control_points() {
            assert!(point.x().exact_rational_ref().is_some());
            assert!(point.y().exact_rational_ref().is_some());
        }
        assert!(
            span.curve()
                .weights()
                .iter()
                .all(|weight| weight.exact_rational_ref().is_some())
        );
    }
    assert_eq!(
        decomposition
            .point_at(&half(), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        expected_midpoint
    );
    assert_eq!(
        crate::support::under_classified_result(&CurveContext::STRICT, || arc
            .representative_point())
        .unwrap(),
        Classification::Decided(expected_midpoint.clone())
    );
    assert_eq!(
        crate::support::under_classified(&CurveContext::STRICT, || arc
            .contains_point(&expected_midpoint)),
        Classification::Decided(true)
    );
    assert_eq!(
        crate::support::under_classified(&CurveContext::STRICT, || arc
            .contains_sweep_point(&Point2::new(half().sqrt().unwrap(), half().sqrt().unwrap()))),
        Classification::Decided(false)
    );
    assert_eq!(
        crate::support::under_classified(&CurveContext::STRICT, || arc
            .contains_sweep_point(&p(-1, 0))),
        Classification::Decided(true)
    );
}

#[test]
fn angular_inverse_selects_unequal_major_arc_charts() {
    for clockwise in [false, true] {
        let direction = if clockwise { r(-1) } else { r(1) };
        let arc = CircularArc2::try_from_center(
            p(1, 0),
            Point2::new(q(3, 5), -&direction * q(4, 5)),
            p(0, 0),
            clockwise,
        )
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (point, expected) in [
                (Point2::new(r(0), direction.clone()), q(1, 3)),
                (p(-1, 0), q(2, 3)),
                (Point2::new(r(0), -direction.clone()), q(5, 6)),
            ] {
                let Classification::Decided(fraction) =
                    crate::support::under_classified_result(&policy, || arc.sweep_fraction(&point))
                        .unwrap()
                else {
                    panic!("the exact point has an angular parameter");
                };
                assert_eq!(
                    crate::support::under_classified_result(&policy, || arc
                        .parameter_at_sweep_fraction(&fraction))
                    .unwrap(),
                    Classification::Decided(expected)
                );
            }
        }
    }
}

#[test]
fn sweep_fraction_orders_major_arc_cardinal_points_exactly() {
    let arc = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), true).unwrap();
    let policy = CurveContext::STRICT;

    assert_eq!(
        crate::support::under_classified_result(&policy, || arc.sweep_fraction(&p(1, 0))).unwrap(),
        Classification::Decided(r(0))
    );
    assert_eq!(
        crate::support::under_classified_result(&policy, || arc.sweep_fraction(&p(0, -1))).unwrap(),
        Classification::Decided(q(1, 3))
    );
    assert_eq!(
        crate::support::under_classified_result(&policy, || arc.sweep_fraction(&p(-1, 0))).unwrap(),
        Classification::Decided(q(2, 3))
    );
    assert_eq!(
        crate::support::under_classified_result(&policy, || arc.sweep_fraction(&p(0, 1))).unwrap(),
        Classification::Decided(r(1))
    );
}

#[test]
fn directed_sweep_evaluation_round_trips_minor_major_and_full_arcs() {
    let policy = CurveContext::APPROXIMATE_512;
    let minor = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), false).unwrap();
    let root_half = (r(2).sqrt().unwrap() / r(2)).unwrap();
    let minor_midpoint = Point2::new(root_half.clone(), root_half);
    assert_eq!(
        crate::support::under_classified_result(&policy, || minor.point_at_sweep_fraction(&half()))
            .unwrap(),
        Classification::Decided(minor_midpoint.clone())
    );
    assert_eq!(
        crate::support::under_classified_result(&policy, || minor.sweep_fraction(&minor_midpoint))
            .unwrap(),
        Classification::Decided(half())
    );
    let Classification::Decided(minor_parameter) =
        crate::support::under_classified_result(&policy, || {
            minor.parameter_at_sweep_fraction(&half())
        })
        .unwrap()
    else {
        panic!("minor-arc rational parameter was not certified");
    };
    let minor_replayed = crate::support::under(&policy, || {
        Curve2::from(minor.clone()).point_at(&minor_parameter.clone().into())
    })
    .unwrap()
    .into_value();
    assert_replayed_containment(crate::support::under_classified(&policy, || {
        minor.contains_point(
            minor_replayed
                .coordinates()
                .expect("native curve evaluation coordinates"),
        )
    }));

    let major = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), true).unwrap();
    let Classification::Decided(strict_major_parameter) =
        crate::support::under_classified_result(&CurveContext::STRICT, || {
            major.parameter_at_sweep_fraction(&q(1, 3))
        })
        .unwrap()
    else {
        panic!("strict major-arc rational parameter was not certified");
    };
    assert_eq!(
        crate::support::under(&policy, || Curve2::from(major.clone())
            .point_at(&strict_major_parameter.clone().into()))
        .unwrap()
        .into_value(),
        p(0, -1).into()
    );
    for (fraction, expected) in [(q(1, 3), p(0, -1)), (q(2, 3), p(-1, 0))] {
        assert_eq!(
            crate::support::under_classified_result(&policy, || major
                .point_at_sweep_fraction(&fraction))
            .unwrap(),
            Classification::Decided(expected.clone())
        );
        assert_eq!(
            crate::support::under_classified_result(&policy, || major.sweep_fraction(&expected))
                .unwrap(),
            Classification::Decided(fraction.clone())
        );
        let Classification::Decided(parameter) =
            crate::support::under_classified_result(&policy, || {
                major.parameter_at_sweep_fraction(&fraction)
            })
            .unwrap()
        else {
            panic!("major-arc rational parameter was not certified");
        };
        let replayed = crate::support::under(&policy, || {
            Curve2::from(major.clone()).point_at(&parameter.clone().into())
        })
        .unwrap()
        .into_value();
        assert_replayed_containment(crate::support::under_classified(&policy, || {
            major.contains_point(
                replayed
                    .coordinates()
                    .expect("native curve evaluation coordinates"),
            )
        }));
    }

    let full = CircularArc2::try_from_center(p(1, 0), p(1, 0), p(0, 0), false).unwrap();
    for (fraction, expected) in [(q(1, 4), p(0, 1)), (q(1, 2), p(-1, 0)), (q(3, 4), p(0, -1))] {
        assert_eq!(
            crate::support::under_classified_result(&policy, || full
                .point_at_sweep_fraction(&fraction))
            .unwrap(),
            Classification::Decided(expected.clone())
        );
        assert_eq!(
            crate::support::under_classified_result(&policy, || full.sweep_fraction(&expected))
                .unwrap(),
            Classification::Decided(fraction.clone())
        );
        assert_eq!(
            crate::support::under_classified_result(&policy, || full
                .parameter_at_sweep_fraction(&fraction))
            .unwrap(),
            Classification::Decided(fraction)
        );
    }
    assert_eq!(
        crate::support::under_classified_result(&policy, || full.point_at_sweep_fraction(&r(0)))
            .unwrap(),
        Classification::Decided(full.start().clone())
    );
    assert_eq!(
        crate::support::under_classified_result(&policy, || full.point_at_sweep_fraction(&r(1)))
            .unwrap(),
        Classification::Decided(full.end().clone())
    );
}

#[test]
fn inverse_sweep_witness_replays_exact_point_across_existing_clone() {
    let center = Point2::new(r(3), q(13, 6));
    let arc =
        CircularArc2::try_from_center(Point2::new(r(3), q(13, 3)), p(5, 3), center, false).unwrap();
    let retained_clone = arc.clone();
    let witness = p(3, 0);
    let policy = CurveContext::STRICT;

    let Classification::Decided(parameter) =
        crate::support::under_classified_result(&policy, || arc.sweep_fraction(&witness)).unwrap()
    else {
        panic!("non-cardinal incident point should have an exact directed-sweep parameter");
    };

    assert_ne!(parameter, r(0));
    assert_ne!(parameter, r(1));
    assert_eq!(
        crate::support::under_classified_result(&policy, || retained_clone
            .point_at_sweep_fraction(&parameter))
        .unwrap(),
        Classification::Decided(witness)
    );
}

#[test]
fn full_circle_uses_four_quarter_spans() {
    let arc = CircularArc2::try_from_center(p(1, 0), p(1, 0), p(0, 0), false).unwrap();
    let decomposition = arc.rational_bezier_decomposition().unwrap();
    let quarter = (r(1) / r(4)).unwrap();
    let three_quarters = (r(3) / r(4)).unwrap();

    assert_eq!(decomposition.spans().len(), 4);
    assert_eq!(
        decomposition
            .point_at(&r(0), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(1, 0)
    );
    assert_eq!(
        decomposition
            .point_at(&quarter, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(0, 1)
    );
    assert_eq!(
        decomposition
            .point_at(&half(), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(-1, 0)
    );
    assert_eq!(
        decomposition
            .point_at(&three_quarters, &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(0, -1)
    );
    assert_eq!(
        decomposition
            .point_at(&r(1), &CurveContext::STRICT)
            .unwrap()
            .into_value(),
        p(1, 0)
    );
    assert_eq!(
        crate::support::under_classified(&CurveContext::STRICT, || arc
            .contains_sweep_point(&p(0, 1))),
        Classification::Decided(true)
    );
    assert_eq!(
        crate::support::under_classified(&CurveContext::STRICT, || arc
            .contains_sweep_point(&p(7, -3))),
        Classification::Decided(true)
    );
}

#[test]
fn sweep_fraction_orders_full_circle_cardinal_points_exactly() {
    let arc = CircularArc2::try_from_center(p(1, 0), p(1, 0), p(0, 0), false).unwrap();
    let policy = CurveContext::STRICT;

    assert_eq!(
        crate::support::under_classified_result(&policy, || arc.sweep_fraction(&p(0, 1))).unwrap(),
        Classification::Decided(q(1, 4))
    );
    assert_eq!(
        crate::support::under_classified_result(&policy, || arc.sweep_fraction(&p(-1, 0))).unwrap(),
        Classification::Decided(q(1, 2))
    );
    assert_eq!(
        crate::support::under_classified_result(&policy, || arc.sweep_fraction(&p(0, -1))).unwrap(),
        Classification::Decided(q(3, 4))
    );
}

#[test]
fn top_level_arc_reuses_promotion_and_builds_mixed_boundary() {
    let arc = Curve2::new(CurveGeometry2::CircularArc(
        CircularArc2::try_from_center(p(-1, 0), p(1, 0), p(0, 0), false).unwrap(),
    ));
    let clone = arc.clone();
    let fragments = arc.native_bezier_fragments().unwrap();

    assert_eq!(fragments.len(), 2);
    assert!(std::ptr::eq(
        fragments,
        crate::support::under(&CurveContext::STRICT, || clone.native_bezier_fragments())
            .unwrap()
            .into_value()
    ));
    assert!(
        fragments
            .iter()
            .all(|fragment| matches!(fragment.curve(), CurveGeometry2::RationalQuadraticBezier(_)))
    );
    assert_eq!(
        crate::support::under(&CurveContext::STRICT, || arc.point_at(&half().into()))
            .unwrap()
            .into_value(),
        p(0, -1).into()
    );

    let closing = Curve2::from(LineSeg2::try_new(p(1, 0), p(-1, 0)).unwrap());
    let path = CurvePath2::try_new(vec![arc, closing]).unwrap();
    let boundary = path.boundary_loop().unwrap();
    assert_eq!(boundary.len(), 3);
    assert_eq!(boundary.len(), 3);
}

#[test]
fn public_arc_native_topology_obeys_terminal_policy_once() {
    let undecidable_zero = support::terminally_unresolved_zero();
    let center = Point2::new(Real::from(3) + &undecidable_zero, Real::one());
    let arc = CircularArc2::try_from_center(p(3, 0), p(3, 2), center, false).unwrap();

    let approximate_sweep = crate::support::under(&CurveContext::APPROXIMATE_512, || {
        arc.directed_sweep_angle()
    })
    .unwrap();
    assert_eq!(
        approximate_sweep.certainty,
        CurveCertainty::Approximate512Consumed
    );
    let strict_sweep = arc.directed_sweep_angle().unwrap_err();
    assert!(matches!(
        strict_sweep,
        ExactCurveError::Blocked(blocker)
            if blocker.operation() == CurveOperation2::Evaluation
                && blocker.family() == Some(CurveFamily2::CircularArc)
                && blocker.reason() == UncertaintyReason::RealSign
    ));

    let approximate_decomposition = crate::support::under(&CurveContext::APPROXIMATE_512, || {
        arc.rational_bezier_decomposition()
    })
    .unwrap();
    assert_eq!(
        approximate_decomposition.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(approximate_decomposition.value.spans().len(), 2);
    assert!(matches!(
        crate::support::under(&CurveContext::STRICT, || arc.rational_bezier_decomposition()),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::BezierDecomposition
                && blocker.reason() == UncertaintyReason::RealSign
    ));

    let curve = Curve2::from(arc.clone());
    let approximate_curve_fragments = crate::support::under(&CurveContext::APPROXIMATE_512, || {
        curve.native_bezier_fragments()
    })
    .unwrap();
    assert_eq!(
        approximate_curve_fragments.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(approximate_curve_fragments.value.len(), 2);
    assert!(approximate_curve_fragments.value.iter().all(|fragment| {
        matches!(fragment.curve(), CurveGeometry2::RationalQuadraticBezier(_))
    }));
    assert!(matches!(
        crate::support::under(&CurveContext::STRICT, || curve.native_bezier_fragments()),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::NativeTopology
                && blocker.reason() == UncertaintyReason::RealSign
    ));

    let path = CurvePath2::try_new(vec![curve]).unwrap();
    let approximate_path_fragments = crate::support::under(&CurveContext::APPROXIMATE_512, || {
        path.native_bezier_fragments()
    })
    .unwrap();
    assert_eq!(
        approximate_path_fragments.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(approximate_path_fragments.value.len(), 2);
    assert!(matches!(
        crate::support::under(&CurveContext::STRICT, || path.native_bezier_fragments()),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::NativeTopology
                && blocker.reason() == UncertaintyReason::RealSign
    ));
    let repeated = crate::support::under(&CurveContext::APPROXIMATE_512, || {
        path.native_bezier_fragments()
    })
    .unwrap();
    assert_eq!(repeated.certainty, CurveCertainty::Approximate512Consumed);

    let exact_semicircle =
        CircularArc2::try_from_center(p(1, 0), p(-1, 0), p(0, 0), false).unwrap();
    let decomposition = exact_semicircle.rational_bezier_decomposition().unwrap();
    let ambiguous_join_parameter = half() + undecidable_zero;
    let approximate_point = decomposition
        .point_at(&ambiguous_join_parameter, &CurveContext::APPROXIMATE_512)
        .unwrap();
    assert_eq!(
        approximate_point.certainty,
        CurveCertainty::Approximate512Consumed
    );
    assert_eq!(approximate_point.value, p(0, 1));
    assert!(matches!(
        decomposition.point_at(&ambiguous_join_parameter, &CurveContext::STRICT),
        Err(ExactCurveError::Blocked(blocker))
            if blocker.operation() == CurveOperation2::Evaluation
                && blocker.reason() == UncertaintyReason::Ordering
    ));
}
