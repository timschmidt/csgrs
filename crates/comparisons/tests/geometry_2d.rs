use std::cmp::Ordering;

use exactcore_hyper_comparison::{
    area2, between2, circle2_relation, incircle2, line2_intersection, line2_point_distance,
    line2_relation, orientation2, point2_distance, segment2_point_distance, segment2_relation,
};
use hyperlimit::{
    CircleLineRelation, CircleSegmentRelation, LineSide, Point2, PointSegmentLocation,
    PredicatePolicy, SegmentIntersection, Sign, classify_circle_line2, classify_circle_segment2,
    classify_point_line, classify_point_segment, classify_segment_intersection,
    construct_line_intersection_point, incircle2 as hyper_incircle2, orient2, orient2d_value,
    point_on_segment,
};
use hyperreal::Real;
use proptest::prelude::*;

const POLICY: PredicatePolicy = PredicatePolicy::STRICT;

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(x.into(), y.into())
}

fn points(coordinates: [i64; 8]) -> [Point2; 4] {
    [
        p(coordinates[0], coordinates[1]),
        p(coordinates[2], coordinates[3]),
        p(coordinates[4], coordinates[5]),
        p(coordinates[6], coordinates[7]),
    ]
}

fn sign_number(sign: Sign) -> i32 {
    match sign {
        Sign::Negative => -1,
        Sign::Zero => 0,
        Sign::Positive => 1,
    }
}

fn exactcore_line_side_number(side: LineSide) -> i32 {
    // CORE's Line2d::orientation convention is the reverse of its free
    // orientation2d function and Hyperlimit's directed-line convention.
    match side {
        LineSide::Right => 1,
        LineSide::On => 0,
        LineSide::Left => -1,
    }
}

fn assert_close(left: f64, right: f64, context: &str) {
    let scale = 1.0_f64.max(left.abs()).max(right.abs());
    assert!(
        left.is_finite() && right.is_finite() && (left - right).abs() <= 2.0e-12 * scale,
        "{context}: exactCore={left:.17e}, Hyper={right:.17e}"
    );
}

fn point_distance_hyper(a: &Point2, b: &Point2) -> f64 {
    (a - b).norm().to_f64_lossy().unwrap()
}

fn line_distance_hyper(a: &Point2, b: &Point2, query: &Point2) -> f64 {
    let direction = b - a;
    let offset = query - a;
    let numerator = direction.wedge(&offset).abs();
    (numerator / direction.norm())
        .unwrap()
        .to_f64_lossy()
        .unwrap()
}

fn segment_distance_hyper(a: &Point2, b: &Point2, query: &Point2) -> f64 {
    let direction = b - a;
    let offset = query - a;
    let projection = direction.dot(&offset);
    let length_squared = direction.norm_squared();
    if projection.partial_cmp(&Real::zero()) == Some(Ordering::Less) || projection == Real::zero() {
        return point_distance_hyper(a, query);
    }
    if projection.partial_cmp(&length_squared) != Some(Ordering::Less) {
        return point_distance_hyper(b, query);
    }
    line_distance_hyper(a, b, query)
}

#[test]
fn orientation_area_line_side_and_collinearity_match() {
    let cases = [
        [0, 0, 5, 0, 1, 3],
        [0, 0, 1, 3, 5, 0],
        [-7, 11, 5, -3, 17, -17],
        [1, 1, 3, 3, 9, 9],
    ];
    for coordinates in cases {
        let [a, b, query] = [
            p(coordinates[0], coordinates[1]),
            p(coordinates[2], coordinates[3]),
            p(coordinates[4], coordinates[5]),
        ];
        let hyper_sign = orient2(&a, &b, &query, POLICY).value().unwrap();
        assert_eq!(orientation2(coordinates), sign_number(hyper_sign));
        assert_eq!(
            line2_relation(0, &[coordinates.as_slice(), &[0, 0]].concat()),
            exactcore_line_side_number(
                classify_point_line(&a, &b, &query, POLICY).value().unwrap()
            ),
        );

        let hyper_area = orient2d_value(&a, &b, &query).to_f64_lossy().unwrap();
        assert_close(area2(coordinates), hyper_area, "twice signed area");
        assert_eq!(
            line2_relation(1, &[coordinates.as_slice(), &[0, 0]].concat()) == 1,
            hyper_sign == Sign::Zero,
        );
    }
}

#[test]
fn strict_between_and_closed_segment_point_classification_match() {
    let cases = [
        ([0, 0, 2, 2, 5, 5], PointSegmentLocation::OnSegment),
        ([0, 0, 0, 0, 5, 5], PointSegmentLocation::OnEndpoint),
        ([0, 0, 5, 5, 5, 5], PointSegmentLocation::OnEndpoint),
        ([0, 0, 7, 7, 5, 5], PointSegmentLocation::CollinearOutside),
        ([0, 0, 2, 3, 5, 5], PointSegmentLocation::OffLine),
    ];
    for (coordinates, expected) in cases {
        let a = p(coordinates[0], coordinates[1]);
        let query = p(coordinates[2], coordinates[3]);
        let b = p(coordinates[4], coordinates[5]);
        let hyper = classify_point_segment(&a, &b, &query, POLICY)
            .value()
            .unwrap();
        assert_eq!(hyper, expected);
        assert_eq!(
            between2(coordinates),
            hyper == PointSegmentLocation::OnSegment
        );

        let padded = [
            coordinates[0],
            coordinates[1],
            coordinates[4],
            coordinates[5],
            coordinates[2],
            coordinates[3],
            0,
            0,
        ];
        assert_eq!(
            segment2_relation(0, &padded) == 1,
            point_on_segment(&a, &b, &query, POLICY).value().unwrap(),
        );
    }
}

#[test]
fn line_relations_and_exact_intersection_construction_match() {
    let cases = [
        [0, 0, 4, 4, 0, 4, 4, 0],
        [0, 0, 4, 0, 0, 3, 4, 3],
        [0, 0, 4, 4, 2, 2, 8, 8],
        [3, -2, 3, 9, -4, 5, 7, 5],
    ];
    for coordinates in cases {
        let [a, b, c, d] = points(coordinates);
        let direction_origin = p(0, 0);
        let first_direction = p(
            coordinates[2] - coordinates[0],
            coordinates[3] - coordinates[1],
        );
        let second_direction = p(
            coordinates[6] - coordinates[4],
            coordinates[7] - coordinates[5],
        );
        let parallel = orient2(
            &direction_origin,
            &first_direction,
            &second_direction,
            POLICY,
        )
        .value()
            == Some(Sign::Zero);
        assert_eq!(line2_relation(2, &coordinates) == 1, parallel);

        let coincident = classify_point_line(&a, &b, &c, POLICY).value() == Some(LineSide::On)
            && classify_point_line(&a, &b, &d, POLICY).value() == Some(LineSide::On);
        assert_eq!(line2_relation(4, &coordinates) == 1, coincident);

        let core_point = line2_intersection(coordinates);
        let hyper_point = construct_line_intersection_point(&a, &b, &c, &d);
        assert_eq!(core_point.is_some(), hyper_point.is_some());
        if let (Some(core), Some(hyper)) = (core_point, hyper_point) {
            assert_close(
                core[0],
                hyper.x.to_f64_lossy().unwrap(),
                "line intersection x",
            );
            assert_close(
                core[1],
                hyper.y.to_f64_lossy().unwrap(),
                "line intersection y",
            );
            assert_eq!(line2_relation(3, &coordinates), 0);
        }

        assert_eq!(
            line2_relation(5, &coordinates) == 1,
            coordinates[0] == coordinates[2],
        );
        assert_eq!(
            line2_relation(6, &coordinates) == 1,
            coordinates[1] == coordinates[3],
        );
    }
}

#[test]
fn segment_intersection_topology_matches() {
    let cases = [
        ([0, 0, 6, 6, 0, 6, 6, 0], SegmentIntersection::Proper),
        ([0, 0, 6, 0, 6, 0, 8, 2], SegmentIntersection::EndpointTouch),
        (
            [0, 0, 6, 0, 2, 0, 8, 0],
            SegmentIntersection::CollinearOverlap,
        ),
        ([0, 0, 6, 0, 6, 0, 0, 0], SegmentIntersection::Identical),
        ([0, 0, 2, 0, 3, 0, 7, 0], SegmentIntersection::Disjoint),
        ([0, 0, 2, 0, 0, 2, 2, 2], SegmentIntersection::Disjoint),
    ];
    for (coordinates, expected) in cases {
        let [a, b, c, d] = points(coordinates);
        let hyper = classify_segment_intersection(&a, &b, &c, &d, POLICY)
            .value()
            .unwrap();
        assert_eq!(hyper, expected);

        let core_dimension = segment2_relation(1, &coordinates);
        assert_eq!(core_dimension != -1, hyper.intersects());
        assert_eq!(core_dimension == 1, hyper.has_positive_length_overlap());
        assert_eq!(
            segment2_relation(2, &coordinates) == 1,
            hyper == SegmentIntersection::Identical
        );

        let parallel = orient2(
            &p(0, 0),
            &p(
                coordinates[2] - coordinates[0],
                coordinates[3] - coordinates[1],
            ),
            &p(
                coordinates[6] - coordinates[4],
                coordinates[7] - coordinates[5],
            ),
            POLICY,
        )
        .value()
            == Some(Sign::Zero);
        assert_eq!(segment2_relation(3, &coordinates) == 1, parallel);
    }
}

#[test]
fn incircle_sign_matches_for_both_windings_and_boundary() {
    let cases = [
        [0, 0, 4, 0, 0, 4, 1, 1],
        [0, 0, 0, 4, 4, 0, 1, 1],
        [0, 0, 4, 0, 0, 4, 4, 4],
        [0, 0, 4, 0, 0, 4, 5, 5],
    ];
    for coordinates in cases {
        let [a, b, c, query] = points(coordinates);
        let hyper = hyper_incircle2(&a, &b, &c, &query, POLICY).value().unwrap();
        assert_eq!(incircle2(coordinates), sign_number(hyper));
    }
}

#[test]
fn point_line_and_segment_distances_match() {
    let point_cases = [[0, 0, 3, 4], [-7, 11, 5, 16], [4, -2, 4, -2]];
    for coordinates in point_cases {
        let a = p(coordinates[0], coordinates[1]);
        let b = p(coordinates[2], coordinates[3]);
        assert_close(
            point2_distance(coordinates),
            point_distance_hyper(&a, &b),
            "point distance",
        );
    }

    let cases = [
        [0, 0, 0, 7, 3, 4],
        [0, 0, 6, 8, 3, 4],
        [-2, 1, 4, 1, 1, 5],
        [0, 0, 4, 0, -3, 4],
        [0, 0, 4, 0, 7, 4],
    ];
    for coordinates in cases {
        let a = p(coordinates[0], coordinates[1]);
        let b = p(coordinates[2], coordinates[3]);
        let query = p(coordinates[4], coordinates[5]);
        assert_close(
            line2_point_distance(coordinates),
            line_distance_hyper(&a, &b, &query),
            "line-point distance",
        );
        assert_close(
            segment2_point_distance(coordinates),
            segment_distance_hyper(&a, &b, &query),
            "segment-point distance",
        );
    }
}

#[test]
fn circle_line_and_segment_boundary_relations_match() {
    let center = p(0, 0);
    let radius_squared: Real = 25.into();
    let line_cases = [
        ([0, 0, -10, 0, 10, 0], CircleLineRelation::Secant),
        ([0, 0, -10, 5, 10, 5], CircleLineRelation::Tangent),
        ([0, 0, -10, 6, 10, 6], CircleLineRelation::Disjoint),
    ];
    for (coordinates, expected) in line_cases {
        let a = p(coordinates[2], coordinates[3]);
        let b = p(coordinates[4], coordinates[5]);
        let hyper = classify_circle_line2(&center, &radius_squared, &a, &b, POLICY)
            .value()
            .unwrap();
        assert_eq!(hyper, expected);
        let core_sign = circle2_relation(0, 5, coordinates);
        let hyper_sign = match hyper {
            // CORE's Circle2d::distance(Line2d) clamps negative distances to
            // zero, so secants and tangencies intentionally collapse here.
            CircleLineRelation::Secant | CircleLineRelation::Tangent => 0,
            CircleLineRelation::Disjoint => 1,
            CircleLineRelation::DegenerateLine => unreachable!(),
        };
        assert_eq!(core_sign, hyper_sign);
    }

    let segment_cases = [
        ([0, 0, -10, 0, 10, 0], CircleSegmentRelation::Secant, -1),
        (
            [0, 0, -1, 0, 1, 0],
            CircleSegmentRelation::ContainedInside,
            -1,
        ),
        // Hyperlimit calls a transverse endpoint boundary hit a secant; CORE's
        // minimum boundary distance is still exactly zero.
        ([0, 0, 5, 0, 5, 3], CircleSegmentRelation::Secant, 0),
        ([0, 0, -10, 5, 10, 5], CircleSegmentRelation::Tangent, 0),
        ([0, 0, 6, 0, 9, 0], CircleSegmentRelation::Disjoint, 1),
    ];
    for (coordinates, expected, expected_core_sign) in segment_cases {
        let a = p(coordinates[2], coordinates[3]);
        let b = p(coordinates[4], coordinates[5]);
        let hyper = classify_circle_segment2(&center, &radius_squared, &a, &b, POLICY)
            .value()
            .unwrap();
        assert_eq!(hyper, expected);
        assert_eq!(circle2_relation(1, 5, coordinates), expected_core_sign);
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn integer_orientation_properties_match(coordinates in prop::array::uniform6(-10_000_i64..=10_000)) {
        let a = p(coordinates[0], coordinates[1]);
        let b = p(coordinates[2], coordinates[3]);
        let c = p(coordinates[4], coordinates[5]);
        let hyper = orient2(&a, &b, &c, POLICY).value().unwrap();
        prop_assert_eq!(orientation2(coordinates), sign_number(hyper));
    }

    #[test]
    fn nondegenerate_segment_intersection_properties_match(coordinates in prop::array::uniform8(-100_i64..=100)) {
        prop_assume!(coordinates[0] != coordinates[2] || coordinates[1] != coordinates[3]);
        prop_assume!(coordinates[4] != coordinates[6] || coordinates[5] != coordinates[7]);
        let [a, b, c, d] = points(coordinates);
        let hyper = classify_segment_intersection(&a, &b, &c, &d, POLICY).value().unwrap();
        let core = segment2_relation(1, &coordinates);
        prop_assert_eq!(core != -1, hyper.intersects());
        prop_assert_eq!(core == 1, hyper.has_positive_length_overlap());
    }
}

#[test]
#[ignore = "known exactCorelib Line2d::intersects checks parallel before coincident"]
fn exactcore_coincident_lines_report_disjoint_dimension() {
    let coordinates = [0, 0, 4, 4, 2, 2, 8, 8];
    assert_eq!(line2_relation(3, &coordinates), 1);
}

#[test]
#[ignore = "known exactCorelib Line2d orientation convention is reversed"]
fn exactcore_line_member_orientation_does_not_match_free_orientation2d() {
    let coordinates = [0, 0, 5, 0, 1, 3];
    let padded = [0, 0, 5, 0, 1, 3, 0, 0];
    assert_eq!(line2_relation(0, &padded), orientation2(coordinates));
}

#[test]
#[ignore = "known exactCorelib Circle2d line distance clamps secants to zero"]
fn exactcore_circle_line_distance_is_not_a_signed_boundary_distance() {
    assert_eq!(circle2_relation(0, 5, [0, 0, -10, 0, 10, 0]), -1);
}
