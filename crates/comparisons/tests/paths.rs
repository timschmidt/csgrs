use exactcore_hyper_comparison::{
    between2, circle2_distance, circle2_relation, expr_constant, line2_relation, point2_distance,
    segment2_relation,
};
use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    ArcDirection, CardinalPoint, CircularArc, ExplicitArcPointClassification,
    ExplicitCircleRelationClass, ExplicitCircularArc, LineExplicitArcIntersectionClass,
    LinePathSegment, SegmentParameterOrder,
};
use hyperreal::Real;

const POLICY: PredicatePolicy = PredicatePolicy::STRICT;

fn r(value: i64) -> Real {
    value.into()
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn line(x0: i64, y0: i64, x1: i64, y1: i64) -> LinePathSegment {
    LinePathSegment::new(p(x0, y0), p(x1, y1), POLICY).unwrap()
}

fn full_circle(center_x: i64, radius: i64) -> ExplicitCircularArc {
    ExplicitCircularArc::new(
        p(center_x, 0),
        r(radius),
        p(center_x + radius, 0),
        p(center_x + radius, 0),
        ArcDirection::Ccw,
        POLICY,
    )
    .unwrap()
}

fn assert_close(left: f64, right: f64, context: &str) {
    let scale = 1.0_f64.max(left.abs()).max(right.abs());
    assert!(
        left.is_finite() && right.is_finite() && (left - right).abs() <= 2.0e-12 * scale,
        "{context}: exactCore={left:.17e}, hyperpath={right:.17e}"
    );
}

#[test]
fn line_path_segment_metric_direction_and_axis_facts_match() {
    let diagonal = line(-2, 3, 6, -3);
    assert_eq!(diagonal.length_squared(), r(100));
    assert_close(
        point2_distance([-2, 3, 6, -3]),
        diagonal.euclidean_length().unwrap().to_f64_lossy().unwrap(),
        "line path length",
    );
    assert_eq!(diagonal.direction_vector(), p(8, -6));
    assert_eq!(diagonal.start_tangent(), p(8, -6));
    assert_eq!(diagonal.end_tangent(), p(8, -6));
    assert_eq!(diagonal.axis_length(POLICY), None);
    assert_eq!(line2_relation(5, &[-2, 3, 6, -3, 0, 0, 0, 0]), 0);
    assert_eq!(line2_relation(6, &[-2, 3, 6, -3, 0, 0, 0, 0]), 0);

    let horizontal = line(9, 4, -3, 4);
    assert_eq!(horizontal.axis_length(POLICY), Some(r(12)));
    assert_eq!(line2_relation(6, &[9, 4, -3, 4, 0, 0, 0, 0]), 1);
    assert_eq!(horizontal.bounds_min(), &p(-3, 4));
    assert_eq!(horizontal.bounds_max(), &p(9, 4));
}

#[test]
fn endpoint_equality_matches_core_segment_coincidence() {
    let first = line(0, 0, 7, 3);
    let cases = [
        (line(0, 0, 7, 3), true),
        (line(7, 3, 0, 0), true),
        (line(0, 0, 8, 3), false),
    ];
    for (second, expected) in cases {
        assert_eq!(
            first.exact_endpoint_equal(&second, POLICY).value(),
            Some(expected)
        );
        let coordinates = [
            0,
            0,
            7,
            3,
            second.start().x.to_f64_lossy().unwrap() as i64,
            second.start().y.to_f64_lossy().unwrap() as i64,
            second.end().x.to_f64_lossy().unwrap() as i64,
            second.end().y.to_f64_lossy().unwrap() as i64,
        ];
        assert_eq!(segment2_relation(2, &coordinates) == 1, expected);
    }
}

#[test]
fn certified_axis_parameter_order_matches_core_between_predicate() {
    let forward = line(0, 0, 10, 0);
    let reverse = line(10, 0, 0, 0);
    assert_eq!(
        forward.compare_points_along(&p(2, 0), &p(8, 0), POLICY),
        SegmentParameterOrder::Before
    );
    assert!(between2([0, 0, 2, 0, 8, 0]));
    assert_eq!(
        reverse.compare_points_along(&p(8, 0), &p(2, 0), POLICY),
        SegmentParameterOrder::Before
    );
    assert!(between2([10, 0, 8, 0, 2, 0]));
    assert_eq!(
        forward.compare_points_along(&p(8, 0), &p(2, 0), POLICY),
        SegmentParameterOrder::After
    );
    assert_eq!(
        forward.compare_points_along(&p(4, 0), &p(4, 0), POLICY),
        SegmentParameterOrder::Equal
    );
}

#[test]
fn cardinal_arc_circle_geometry_and_symbolic_length_match() {
    let arc = CircularArc::cardinal(
        p(2, -3),
        r(5),
        CardinalPoint::East,
        CardinalPoint::North,
        ArcDirection::Ccw,
        POLICY,
    )
    .unwrap();
    assert_eq!(arc.start(), p(7, -3));
    assert_eq!(arc.end(), p(2, 2));
    assert_eq!(arc.chord_length_squared(), r(50));
    assert_close(
        point2_distance([7, -3, 2, 2]),
        arc.chord_length_squared()
            .sqrt()
            .unwrap()
            .to_f64_lossy()
            .unwrap(),
        "arc chord",
    );
    assert_eq!(arc.start_tangent(), p(0, 5));
    assert_eq!(arc.end_tangent(), p(-5, 0));
    assert_close(
        2.5 * expr_constant(0),
        arc.exact_length().to_f64_lossy().unwrap(),
        "quarter-circle length",
    );
    assert_eq!(circle2_distance(0, 5, 0, [2, -3, 7, -3]), 0.0);
    assert_eq!(circle2_distance(0, 5, 0, [2, -3, 2, 2]), 0.0);
}

#[test]
fn full_circle_point_membership_matches_core_signed_distance_zero_set() {
    let circle = full_circle(0, 5);
    for (query, expected) in [
        (p(3, 4), ExplicitArcPointClassification::OnArc),
        (p(0, 0), ExplicitArcPointClassification::OffCircle),
        (p(0, 6), ExplicitArcPointClassification::OffCircle),
    ] {
        let hyper = circle.classify_point(&query, POLICY);
        assert_eq!(hyper, expected);
        let x = query.x.to_f64_lossy().unwrap() as i64;
        let y = query.y.to_f64_lossy().unwrap() as i64;
        assert_eq!(
            circle2_distance(0, 5, 0, [0, 0, x, y]) == 0.0,
            hyper == ExplicitArcPointClassification::OnArc
        );
    }
}

#[test]
fn full_circle_segment_boundary_intersections_match() {
    let circle = full_circle(0, 5);
    let cases = [
        (-10, 0, 10, 0, LineExplicitArcIntersectionClass::Secant, -1),
        (-10, 5, 10, 5, LineExplicitArcIntersectionClass::Tangent, 0),
        (-10, 7, 10, 7, LineExplicitArcIntersectionClass::Disjoint, 1),
    ];
    for (x0, y0, x1, y1, expected, core_sign) in cases {
        let segment = line(x0, y0, x1, y1);
        let general = circle.intersect_segment(&segment, POLICY);
        let axis = circle.intersect_axis_aligned_segment(&segment, POLICY);
        assert_eq!(general.class, expected);
        assert_eq!(axis.class, expected);
        assert_eq!(circle2_relation(1, 5, [0, 0, x0, y0, x1, y1]), core_sign);
    }
}

#[test]
fn retained_circle_relation_matches_core_clamped_separation() {
    let base = full_circle(0, 5);
    let cases = [
        (
            full_circle(0, 5),
            ExplicitCircleRelationClass::SameCircle,
            0.0,
        ),
        (
            full_circle(20, 5),
            ExplicitCircleRelationClass::Separate,
            10.0,
        ),
        (
            full_circle(10, 5),
            ExplicitCircleRelationClass::ExternallyTangent,
            0.0,
        ),
        (full_circle(6, 5), ExplicitCircleRelationClass::Secant, 0.0),
        (
            full_circle(3, 2),
            ExplicitCircleRelationClass::InternallyTangent,
            0.0,
        ),
        (
            full_circle(1, 2),
            ExplicitCircleRelationClass::Contained,
            0.0,
        ),
    ];
    for (other, expected, separation) in cases {
        let report = base.classify_circle_relation(&other, POLICY);
        assert_eq!(report.class, expected);
        let center_x = other.center().x.to_f64_lossy().unwrap() as i64;
        let radius = other.radius().to_f64_lossy().unwrap() as i64;
        assert_close(
            circle2_distance(1, 5, radius, [0, 0, center_x, 0]),
            separation,
            "circle separation",
        );
    }
}

#[test]
#[ignore = "CORE's circle-to-segment boundary distance is negative for a segment contained inside the disk, while hyperpath reports no boundary intersection"]
fn contained_segment_boundary_intersection_contracts_match() {
    let circle = full_circle(0, 5);
    let segment = line(-1, 0, 1, 0);
    assert_eq!(
        circle.intersect_segment(&segment, POLICY).class,
        LineExplicitArcIntersectionClass::Secant
    );
    assert_eq!(circle2_relation(1, 5, [0, 0, -1, 0, 1, 0]), -1);
}
