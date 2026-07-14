use hypercurve::{
    BezierSubcurve2, CircularArc2, Classification, Curve2, CurveGeometry2, CurvePath2, CurvePolicy,
    CurveSource2, LineSeg2, Point2, Real,
};
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
    assert_eq!(
        span.curve().control_weight() * span.curve().control_weight(),
        half()
    );
    let third = (r(1) / r(3)).unwrap();
    let point = decomposition.point_at(&third).unwrap();
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
    assert_eq!(decomposition.point_at(&half()).unwrap(), p(0, 1));
    assert_eq!(
        arc.representative_point(&CurvePolicy::certified()).unwrap(),
        Classification::Decided(p(0, 1))
    );
}

#[test]
fn rationally_trimmed_semicircle_redecomposes_exactly() {
    let source = Curve2::from(CircularArc2::from_bulge(p(0, 0), p(2, 0), r(1)).unwrap());
    let quarter = (r(1) / r(4)).unwrap();
    let three_quarters = (r(3) / r(4)).unwrap();
    let trimmed = source.subcurve(quarter, three_quarters).unwrap();
    let CurveGeometry2::CircularArc(arc) = trimmed.geometry() else {
        panic!("trimmed arc changed family");
    };

    let decomposition = arc.rational_bezier_decomposition().unwrap();
    assert!(!decomposition.spans().is_empty());
    assert_eq!(decomposition.point_at(&r(0)).unwrap(), arc.start().clone());
    assert_eq!(decomposition.point_at(&r(1)).unwrap(), arc.end().clone());
}

#[test]
fn major_arc_uses_requested_orientation_and_major_midpoint() {
    let arc = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), true).unwrap();
    let decomposition = arc.rational_bezier_decomposition().unwrap();
    let root_half = half().sqrt().unwrap();
    let expected_midpoint = Point2::new(-root_half.clone(), -root_half);

    assert_eq!(decomposition.spans().len(), 2);
    assert_eq!(decomposition.point_at(&half()).unwrap(), expected_midpoint);
    assert_eq!(
        arc.representative_point(&CurvePolicy::certified()).unwrap(),
        Classification::Decided(expected_midpoint.clone())
    );
    assert_eq!(
        arc.contains_point(&expected_midpoint, &CurvePolicy::certified()),
        Classification::Decided(true)
    );
    assert_eq!(
        arc.contains_sweep_point(
            &Point2::new(half().sqrt().unwrap(), half().sqrt().unwrap()),
            &CurvePolicy::certified(),
        ),
        Classification::Decided(false)
    );
    assert_eq!(
        arc.contains_sweep_point(&p(-1, 0), &CurvePolicy::certified()),
        Classification::Decided(true)
    );
}

#[test]
fn sweep_fraction_orders_major_arc_cardinal_points_exactly() {
    let arc = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), true).unwrap();
    let policy = CurvePolicy::certified();

    assert_eq!(
        arc.sweep_fraction(&p(1, 0), &policy).unwrap(),
        Classification::Decided(r(0))
    );
    assert_eq!(
        arc.sweep_fraction(&p(0, -1), &policy).unwrap(),
        Classification::Decided(q(1, 3))
    );
    assert_eq!(
        arc.sweep_fraction(&p(-1, 0), &policy).unwrap(),
        Classification::Decided(q(2, 3))
    );
    assert_eq!(
        arc.sweep_fraction(&p(0, 1), &policy).unwrap(),
        Classification::Decided(r(1))
    );
}

#[test]
fn directed_sweep_evaluation_round_trips_minor_major_and_full_arcs() {
    let policy = CurvePolicy::certified();
    let minor = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), false).unwrap();
    let root_half = (r(2).sqrt().unwrap() / r(2)).unwrap();
    let minor_midpoint = Point2::new(root_half.clone(), root_half);
    assert_eq!(
        minor.point_at_sweep_fraction(&half(), &policy).unwrap(),
        Classification::Decided(minor_midpoint.clone())
    );
    assert_eq!(
        minor.sweep_fraction(&minor_midpoint, &policy).unwrap(),
        Classification::Decided(half())
    );

    let major = CircularArc2::try_from_center(p(1, 0), p(0, 1), p(0, 0), true).unwrap();
    for (fraction, expected) in [(q(1, 3), p(0, -1)), (q(2, 3), p(-1, 0))] {
        assert_eq!(
            major.point_at_sweep_fraction(&fraction, &policy).unwrap(),
            Classification::Decided(expected.clone())
        );
        assert_eq!(
            major.sweep_fraction(&expected, &policy).unwrap(),
            Classification::Decided(fraction)
        );
    }

    let full = CircularArc2::try_from_center(p(1, 0), p(1, 0), p(0, 0), false).unwrap();
    for (fraction, expected) in [(q(1, 4), p(0, 1)), (q(1, 2), p(-1, 0)), (q(3, 4), p(0, -1))] {
        assert_eq!(
            full.point_at_sweep_fraction(&fraction, &policy).unwrap(),
            Classification::Decided(expected.clone())
        );
        assert_eq!(
            full.sweep_fraction(&expected, &policy).unwrap(),
            Classification::Decided(fraction)
        );
    }
    assert_eq!(
        full.point_at_sweep_fraction(&r(0), &policy).unwrap(),
        Classification::Decided(full.start().clone())
    );
    assert_eq!(
        full.point_at_sweep_fraction(&r(1), &policy).unwrap(),
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
    let policy = CurvePolicy::certified();

    let Classification::Decided(parameter) = arc.sweep_fraction(&witness, &policy).unwrap() else {
        panic!("non-cardinal incident point should have an exact directed-sweep parameter");
    };

    assert_ne!(parameter, r(0));
    assert_ne!(parameter, r(1));
    assert_eq!(
        retained_clone
            .point_at_sweep_fraction(&parameter, &policy)
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
    assert_eq!(decomposition.point_at(&r(0)).unwrap(), p(1, 0));
    assert_eq!(decomposition.point_at(&quarter).unwrap(), p(0, 1));
    assert_eq!(decomposition.point_at(&half()).unwrap(), p(-1, 0));
    assert_eq!(decomposition.point_at(&three_quarters).unwrap(), p(0, -1));
    assert_eq!(decomposition.point_at(&r(1)).unwrap(), p(1, 0));
    assert_eq!(
        arc.contains_sweep_point(&p(0, 1), &CurvePolicy::certified()),
        Classification::Decided(true)
    );
    assert_eq!(
        arc.contains_sweep_point(&p(7, -3), &CurvePolicy::certified()),
        Classification::Decided(true)
    );
}

#[test]
fn sweep_fraction_orders_full_circle_cardinal_points_exactly() {
    let arc = CircularArc2::try_from_center(p(1, 0), p(1, 0), p(0, 0), false).unwrap();
    let policy = CurvePolicy::certified();

    assert_eq!(
        arc.sweep_fraction(&p(0, 1), &policy).unwrap(),
        Classification::Decided(q(1, 4))
    );
    assert_eq!(
        arc.sweep_fraction(&p(-1, 0), &policy).unwrap(),
        Classification::Decided(q(1, 2))
    );
    assert_eq!(
        arc.sweep_fraction(&p(0, -1), &policy).unwrap(),
        Classification::Decided(q(3, 4))
    );
}

#[test]
fn top_level_arc_reuses_promotion_and_builds_mixed_boundary() {
    let source = CurveSource2::with_version(70, 4);
    let arc = Curve2::with_source(
        CurveGeometry2::CircularArc(
            CircularArc2::try_from_center(p(-1, 0), p(1, 0), p(0, 0), false).unwrap(),
        ),
        source,
    )
    .unwrap();
    let clone = arc.clone();
    let fragments = arc.native_bezier_fragments().unwrap();

    assert_eq!(fragments.len(), 2);
    assert!(std::ptr::eq(
        fragments,
        clone.native_bezier_fragments().unwrap()
    ));
    assert!(
        fragments
            .iter()
            .all(|fragment| matches!(fragment.curve(), BezierSubcurve2::RationalQuadratic(_)))
    );
    assert_eq!(fragments[0].provenance().source(), Some(source));
    assert_eq!(fragments[0].provenance().source_span_index(), Some(0));
    assert_eq!(fragments[1].provenance().source_span_index(), Some(1));
    assert_eq!(arc.point_at(&half()).unwrap(), p(0, -1));

    let closing = Curve2::from(LineSeg2::try_new(p(1, 0), p(-1, 0)).unwrap());
    let path = CurvePath2::try_new(vec![arc, closing]).unwrap();
    let boundary = path.bezier_boundary_loop().unwrap();
    assert_eq!(boundary.fragments().len(), 3);
    assert_eq!(boundary.boundary_loop().fragments().len(), 3);
}
