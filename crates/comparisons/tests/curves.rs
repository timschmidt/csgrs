use exactcore_hyper_comparison::{
    circle2_distance, circle2_relation, line2_intersection, line2_relation, point2_distance,
    segment2_relation,
};
use hypercurve::{
    CircleCircleRelation, CircularArc2, IntersectionKind, LineCircleRelation, LineLineIntersection,
    LineSeg2, LineSide, Point2, Real, Segment2, SegmentIntersection,
};

fn r(value: i64) -> Real {
    value.into()
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn as_f64(value: &Real) -> f64 {
    value
        .to_f64_lossy()
        .expect("integer test geometry is finite")
}

fn assert_close(left: f64, right: f64, context: &str) {
    let scale = 1.0_f64.max(left.abs()).max(right.abs());
    assert!(
        left.is_finite() && right.is_finite() && (left - right).abs() <= 2.0e-12 * scale,
        "{context}: exactCore={left:.17e}, hypercurve={right:.17e}"
    );
}

fn radius_five_arc(center_x: i64) -> CircularArc2 {
    CircularArc2::try_from_center(
        p(center_x + 5, 0),
        p(center_x - 5, 0),
        p(center_x, 0),
        false,
    )
    .unwrap()
}

#[test]
fn line_segment_geometry_and_point_metric_match() {
    let segment = LineSeg2::try_new(p(-2, 3), p(6, -3)).unwrap();
    let (dx, dy) = segment.delta();
    assert_eq!(dx, r(8));
    assert_eq!(dy, r(-6));
    assert_eq!(segment.length_squared(), r(100));
    assert_close(
        point2_distance([-2, 3, 6, -3]),
        as_f64(&segment.length_squared().sqrt().unwrap()),
        "segment length",
    );

    let quarter = segment.point_at((r(1) / r(4)).unwrap());
    assert_eq!(quarter, Point2::new(r(0), (r(3) / r(2)).unwrap()));
    let reversed = segment.reversed();
    assert_eq!(reversed.start(), segment.end());
    assert_eq!(reversed.end(), segment.start());
    assert_eq!(reversed.point_at((r(3) / r(4)).unwrap()), quarter);
}

#[test]
fn line_side_and_finite_point_containment_match() {
    let segment = LineSeg2::try_new(p(0, 0), p(6, 0)).unwrap();
    let cases = [
        (p(2, 3), LineSide::Left, false),
        (p(2, -3), LineSide::Right, false),
        (p(2, 0), LineSide::On, true),
        (p(9, 0), LineSide::On, false),
    ];

    for (query, expected_side, expected_contains) in cases {
        assert_eq!(segment.classify_point(&query).unwrap(), expected_side);
        assert_eq!(segment.contains_point(&query).unwrap(), expected_contains);

        let coordinates = [
            0,
            0,
            6,
            0,
            as_f64(query.x()) as i64,
            as_f64(query.y()) as i64,
            0,
            0,
        ];
        let expected_core_side = match expected_side {
            // CORE's Line2d member convention is reversed relative to the
            // ordinary oriented-line determinant used by hypercurve.
            LineSide::Left => -1,
            LineSide::Right => 1,
            LineSide::On => 0,
        };
        assert_eq!(line2_relation(0, &coordinates), expected_core_side);
        assert_eq!(segment2_relation(0, &coordinates) == 1, expected_contains);
    }
}

#[test]
fn finite_line_intersection_topology_and_witnesses_match() {
    let cases = [
        [0, 0, 6, 6, 0, 6, 6, 0],
        [0, 0, 6, 0, 6, 0, 8, 2],
        [0, 0, 6, 0, 2, 0, 8, 0],
        [0, 0, 2, 0, 3, 0, 7, 0],
    ];

    for coordinates in cases {
        let first = LineSeg2::try_new(
            p(coordinates[0], coordinates[1]),
            p(coordinates[2], coordinates[3]),
        )
        .unwrap();
        let second = LineSeg2::try_new(
            p(coordinates[4], coordinates[5]),
            p(coordinates[6], coordinates[7]),
        )
        .unwrap();
        let relation = first.intersect_line(&second).unwrap();
        let hyper_dimension = match &relation {
            LineLineIntersection::None => -1,
            LineLineIntersection::Point { .. } => 0,
            LineLineIntersection::Overlap { .. } => 1,
            LineLineIntersection::Uncertain { reason } => {
                panic!("unexpected uncertain result: {reason:?}")
            }
        };
        assert_eq!(segment2_relation(1, &coordinates), hyper_dimension);

        if let LineLineIntersection::Point {
            point,
            a_param,
            b_param,
            kind,
        } = relation
        {
            assert!(matches!(
                kind,
                IntersectionKind::Crossing | IntersectionKind::Endpoint
            ));
            assert!(a_param >= Real::zero() && a_param <= Real::one());
            assert!(b_param >= Real::zero() && b_param <= Real::one());
            let core = line2_intersection(coordinates).unwrap();
            assert_close(core[0], as_f64(point.x()), "intersection x");
            assert_close(core[1], as_f64(point.y()), "intersection y");
        }
    }
}

#[test]
fn segment_enum_line_dispatch_matches_direct_and_core_paths() {
    let first = LineSeg2::try_new(p(0, 0), p(4, 4)).unwrap();
    let second = LineSeg2::try_new(p(0, 4), p(4, 0)).unwrap();
    let direct = first.intersect_line(&second).unwrap();
    let dispatched = Segment2::Line(first)
        .intersect_segment(&Segment2::Line(second))
        .unwrap();
    assert_eq!(dispatched, SegmentIntersection::LineLine(direct.clone()));
    assert!(matches!(direct, LineLineIntersection::Point { .. }));
    assert_eq!(segment2_relation(1, &[0, 0, 4, 4, 0, 4, 4, 0]), 0);
}

#[test]
fn supporting_line_circle_relation_matches_core_coarse_distance_contract() {
    let circle = radius_five_arc(0);
    let cases = [(-10, 7, 10, 7, 1), (-10, 5, 10, 5, 0), (-10, 0, 10, 0, 0)];
    for (x0, y0, x1, y1, expected_core_sign) in cases {
        let line = LineSeg2::try_new(p(x0, y0), p(x1, y1)).unwrap();
        let relation = line.supporting_line_circle_relation(&circle).unwrap();
        assert_eq!(
            relation.is_disjoint(),
            expected_core_sign > 0,
            "relation was {relation:?}"
        );
        assert_eq!(
            circle2_relation(0, 5, [0, 0, x0, y0, x1, y1]),
            expected_core_sign
        );
        assert!(matches!(
            relation,
            LineCircleRelation::Disjoint
                | LineCircleRelation::Tangent { .. }
                | LineCircleRelation::Secant { .. }
        ));
    }
}

#[test]
fn circle_point_signed_distance_matches_supporting_circle_geometry() {
    let circle = radius_five_arc(0);
    for (x, y) in [(0, 0), (3, 4), (0, 13), (-12, 5)] {
        let query = p(x, y);
        let radial_distance = query
            .distance_squared(circle.center())
            .sqrt()
            .unwrap()
            .to_f64_lossy()
            .unwrap();
        assert_close(
            circle2_distance(0, 5, 0, [0, 0, x, y]),
            radial_distance - 5.0,
            "signed circle-point distance",
        );
    }
}

#[test]
fn circle_pair_relation_and_clamped_separation_match() {
    let first = radius_five_arc(0);
    let cases = [
        (0, CircleCircleRelation::Coincident, 0.0),
        (
            8,
            CircleCircleRelation::Secant {
                first_point: p(4, 3),
                second_point: p(4, -3),
            },
            0.0,
        ),
        (10, CircleCircleRelation::Tangent { point: p(5, 0) }, 0.0),
        (12, CircleCircleRelation::Disjoint, 2.0),
    ];

    for (center_x, expected_kind, expected_separation) in cases {
        let second = radius_five_arc(center_x);
        let actual = first.circle_relation(&second).unwrap();
        assert_eq!(
            std::mem::discriminant(&actual),
            std::mem::discriminant(&expected_kind),
            "circle relation was {actual:?}"
        );
        assert_close(
            circle2_distance(1, 5, 5, [0, 0, center_x, 0]),
            expected_separation,
            "clamped circle separation",
        );
    }
}

#[test]
#[ignore = "known exactCorelib Circle2d circle distance clamps all overlaps to zero"]
fn core_circle_distance_cannot_distinguish_secant_from_tangent() {
    let secant = radius_five_arc(8)
        .circle_relation(&radius_five_arc(0))
        .unwrap();
    assert!(matches!(secant, CircleCircleRelation::Secant { .. }));
    assert!(circle2_distance(1, 5, 5, [0, 0, 8, 0]) < 0.0);
}

#[test]
#[ignore = "exactCorelib accepts zero-length Segment2d values while hypercurve rejects them"]
fn degenerate_line_segment_constructor_contracts_match() {
    assert!(LineSeg2::try_new(p(1, 2), p(1, 2)).is_ok());
}
