use std::cmp::Ordering;

use exactcore_hyper_comparison::{
    line3_point_distance, line3_relation, orientation3, plane3_point_distance, plane3_relation,
    point3_distance, segment3_point_distance, segment3_relation, triangle3_relation, volume3,
};
use hyperlattice::{Matrix3, Point3, Real};
use hyperlimit::{
    Plane3, PlaneSegmentRelation, PlaneSide, PointSegmentLocation, PredicatePolicy,
    Segment3Intersection, SegmentTriangleIntersection, Sign, Triangle3Location,
    classify_plane_segment, classify_point_oriented_plane, classify_point_plane,
    classify_point_segment3, classify_point_triangle3, classify_segment_triangle3_intersection,
    classify_segment3_intersection, classify_triangle_triangle3,
    compare_point_line3_distance_squared, compare_point_plane_distance_squared,
    compare_point_segment3_distance_squared, orient3, point_on_segment3, point_plane_value,
};
use proptest::prelude::*;

const POLICY: PredicatePolicy = PredicatePolicy::STRICT;

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}

fn points4(coordinates: [i64; 12]) -> [Point3; 4] {
    [
        p(coordinates[0], coordinates[1], coordinates[2]),
        p(coordinates[3], coordinates[4], coordinates[5]),
        p(coordinates[6], coordinates[7], coordinates[8]),
        p(coordinates[9], coordinates[10], coordinates[11]),
    ]
}

fn sign_number(sign: Sign) -> i32 {
    match sign {
        Sign::Negative => -1,
        Sign::Zero => 0,
        Sign::Positive => 1,
    }
}

fn plane_side_number(side: PlaneSide) -> i32 {
    match side {
        PlaneSide::Below => -1,
        PlaneSide::On => 0,
        PlaneSide::Above => 1,
    }
}

fn assert_close(left: f64, right: f64, context: &str) {
    let scale = 1.0_f64.max(left.abs()).max(right.abs());
    assert!(
        left.is_finite() && right.is_finite() && (left - right).abs() <= 3.0e-12 * scale,
        "{context}: exactCore={left:.17e}, Hyper={right:.17e}"
    );
}

fn plane_from_points(a: &Point3, b: &Point3, c: &Point3) -> Plane3 {
    let normal = (b - a).cross(&(c - a));
    let offset = -normal.dot(&a.to_vector());
    Plane3 {
        normal: Point3::new(
            normal.0[0].clone(),
            normal.0[1].clone(),
            normal.0[2].clone(),
        ),
        offset,
    }
}

fn point_distance_hyper(a: &Point3, b: &Point3) -> f64 {
    (a - b).norm().to_f64_lossy().unwrap()
}

fn line_distance_hyper(a: &Point3, b: &Point3, query: &Point3) -> f64 {
    let direction = b - a;
    let offset = query - a;
    let numerator = direction.cross(&offset).norm();
    (numerator / direction.norm())
        .unwrap()
        .to_f64_lossy()
        .unwrap()
}

fn segment_distance_hyper(a: &Point3, b: &Point3, query: &Point3) -> f64 {
    let direction = b - a;
    let offset = query - a;
    let projection = direction.dot(&offset);
    let length_squared = direction.norm_squared();
    if projection.partial_cmp(&Real::zero()) != Some(Ordering::Greater) {
        return point_distance_hyper(a, query);
    }
    if projection.partial_cmp(&length_squared) != Some(Ordering::Less) {
        return point_distance_hyper(b, query);
    }
    line_distance_hyper(a, b, query)
}

fn plane_distance_hyper(plane: &Plane3, query: &Point3) -> f64 {
    let numerator = point_plane_value(plane, query).abs();
    let denominator = plane.normal.to_vector().norm();
    (numerator / denominator).unwrap().to_f64_lossy().unwrap()
}

#[test]
fn orientation_and_signed_tetrahedral_volume_match() {
    let cases = [
        [0, 0, 0, 4, 0, 0, 0, 5, 0, 0, 0, 6],
        [0, 0, 0, 0, 5, 0, 4, 0, 0, 0, 0, 6],
        [1, 2, 3, 4, -2, 5, -3, 1, 7, 9, -4, 2],
        [0, 0, 0, 4, 0, 0, 0, 5, 0, 2, 2, 0],
    ];
    for coordinates in cases {
        let [a, b, c, d] = points4(coordinates);
        let hyper_sign = orient3(&a, &b, &c, &d, POLICY).value().unwrap();
        assert_eq!(orientation3(coordinates), sign_number(hyper_sign));

        let rows = [
            [&a.x - &d.x, &a.y - &d.y, &a.z - &d.z],
            [&b.x - &d.x, &b.y - &d.y, &b.z - &d.z],
            [&c.x - &d.x, &c.y - &d.y, &c.z - &d.z],
        ];
        let hyper_volume = Matrix3::new(rows).determinant().to_f64_lossy().unwrap();
        assert_close(
            volume3(coordinates),
            hyper_volume,
            "signed tetrahedral determinant",
        );
    }
}

#[test]
fn line3_incidence_parallel_skew_and_intersection_dimension_match() {
    let cases = [
        ([0, 0, 0, 4, 0, 0, 0, -2, 0, 0, 2, 0], 0),
        ([0, 0, 0, 4, 0, 0, 0, 1, 0, 4, 1, 0], -1),
        ([0, 0, 0, 4, 0, 0, 2, 0, 0, 8, 0, 0], 1),
        ([0, 0, 0, 4, 0, 0, 0, 0, 1, 0, 2, 1], -1),
    ];
    for (coordinates, expected_dimension) in cases {
        let [a, b, c, d] = points4(coordinates);
        let first_direction = b.clone() - &a;
        let second_direction = d.clone() - &c;
        let parallel = first_direction.cross(&second_direction).norm_squared() == Real::zero();
        let c_on_first = classify_point_segment3(&a, &b, &c, POLICY).value().unwrap()
            != PointSegmentLocation::OffLine;
        let d_on_first = classify_point_segment3(&a, &b, &d, POLICY).value().unwrap()
            != PointSegmentLocation::OffLine;
        let coincident = c_on_first && d_on_first;
        let skew = orient3(&a, &b, &c, &d, POLICY).value() != Some(Sign::Zero);

        assert_eq!(line3_relation(1, &coordinates) == 1, parallel);
        assert_eq!(line3_relation(2, &coordinates) == 1, skew);
        assert_eq!(line3_relation(4, &coordinates) == 1, coincident);
        assert_eq!(line3_relation(3, &coordinates), expected_dimension);

        assert_eq!(
            expected_dimension,
            if coincident {
                1
            } else if skew || parallel {
                -1
            } else {
                0
            }
        );
    }

    let point_case = [0, 0, 0, 4, 0, 0, 9, 0, 0, 0, 0, 0];
    assert_eq!(line3_relation(0, &point_case), 1);
}

#[test]
fn segment3_point_and_intersection_topology_match() {
    let point_cases = [
        ([0, 0, 0, 6, 6, 6, 3, 3, 3], PointSegmentLocation::OnSegment),
        (
            [0, 0, 0, 6, 6, 6, 0, 0, 0],
            PointSegmentLocation::OnEndpoint,
        ),
        (
            [0, 0, 0, 6, 6, 6, 9, 9, 9],
            PointSegmentLocation::CollinearOutside,
        ),
        ([0, 0, 0, 6, 6, 6, 3, 3, 4], PointSegmentLocation::OffLine),
    ];
    for (coordinates, expected) in point_cases {
        let a = p(coordinates[0], coordinates[1], coordinates[2]);
        let b = p(coordinates[3], coordinates[4], coordinates[5]);
        let query = p(coordinates[6], coordinates[7], coordinates[8]);
        let hyper = classify_point_segment3(&a, &b, &query, POLICY)
            .value()
            .unwrap();
        assert_eq!(hyper, expected);
        let padded = [
            coordinates[0],
            coordinates[1],
            coordinates[2],
            coordinates[3],
            coordinates[4],
            coordinates[5],
            coordinates[6],
            coordinates[7],
            coordinates[8],
            0,
            0,
            0,
        ];
        assert_eq!(
            segment3_relation(0, &padded) == 1,
            point_on_segment3(&a, &b, &query, POLICY).value().unwrap(),
        );
    }

    let cases = [
        (
            [0, 0, 0, 6, 0, 0, 3, -3, 0, 3, 3, 0],
            Segment3Intersection::Proper,
        ),
        (
            [0, 0, 0, 6, 0, 0, 6, 0, 0, 6, 3, 0],
            Segment3Intersection::EndpointTouch,
        ),
        (
            [0, 0, 0, 6, 0, 0, 2, 0, 0, 8, 0, 0],
            Segment3Intersection::CollinearOverlap,
        ),
        (
            [0, 0, 0, 6, 0, 0, 6, 0, 0, 0, 0, 0],
            Segment3Intersection::Identical,
        ),
        (
            [0, 0, 0, 2, 0, 0, 3, 1, 0, 7, 1, 0],
            Segment3Intersection::CoplanarDisjoint,
        ),
        (
            [0, 0, 0, 6, 0, 0, 3, -3, 1, 3, 3, 1],
            Segment3Intersection::SkewDisjoint,
        ),
    ];
    for (coordinates, expected) in cases {
        let [a, b, c, d] = points4(coordinates);
        let hyper = classify_segment3_intersection(&a, &b, &c, &d, POLICY)
            .value()
            .unwrap();
        assert_eq!(hyper, expected);
        let core = segment3_relation(1, &coordinates);
        assert_eq!(
            core != -1,
            !matches!(
                hyper,
                Segment3Intersection::SkewDisjoint | Segment3Intersection::CoplanarDisjoint
            )
        );
        assert_eq!(
            core == 1,
            matches!(
                hyper,
                Segment3Intersection::CollinearOverlap | Segment3Intersection::Identical
            )
        );
        assert_eq!(
            segment3_relation(2, &coordinates) == 1,
            hyper == Segment3Intersection::Identical
        );
        assert_eq!(
            segment3_relation(3, &coordinates) == 1,
            hyper != Segment3Intersection::SkewDisjoint
        );
    }
}

#[test]
fn plane_point_line_segment_and_plane_relations_match() {
    let plane_points = [p(0, 0, 0), p(4, 0, 0), p(0, 4, 0)];
    let plane = plane_from_points(&plane_points[0], &plane_points[1], &plane_points[2]);
    for query in [p(1, 1, -3), p(2, 2, 0), p(1, 1, 5)] {
        let coordinates = [
            0,
            0,
            0,
            4,
            0,
            0,
            0,
            4,
            0,
            query.x.to_f64_lossy().unwrap() as i64,
            query.y.to_f64_lossy().unwrap() as i64,
            query.z.to_f64_lossy().unwrap() as i64,
            0,
            0,
            0,
            0,
            0,
            0,
        ];
        let hyper = classify_point_plane(&query, &plane, POLICY)
            .value()
            .unwrap();
        assert_eq!(
            plane3_relation(0, &coordinates) == 1,
            hyper == PlaneSide::On
        );
        assert_eq!(plane3_relation(1, &coordinates), plane_side_number(hyper));

        let oriented = classify_point_oriented_plane(
            &plane_points[0],
            &plane_points[1],
            &plane_points[2],
            &query,
            POLICY,
        )
        .value()
        .unwrap();
        assert_eq!(
            orientation3(coordinates[..12].try_into().unwrap()),
            plane_side_number(oriented)
        );
    }

    let line_cases = [
        ([1, 1, 0], [3, 1, 0], PlaneSegmentRelation::Coplanar, 1),
        ([1, 1, -2], [1, 1, 3], PlaneSegmentRelation::Crossing, 0),
        ([1, 1, 2], [3, 1, 2], PlaneSegmentRelation::Above, -1),
    ];
    for (start, end, expected, core_dimension) in line_cases {
        let start_point = p(start[0], start[1], start[2]);
        let end_point = p(end[0], end[1], end[2]);
        let coordinates = [
            0, 0, 0, 4, 0, 0, 0, 4, 0, start[0], start[1], start[2], end[0], end[1], end[2], 0, 0,
            0,
        ];
        let hyper = classify_plane_segment(&plane, &start_point, &end_point, POLICY)
            .value()
            .unwrap();
        assert_eq!(hyper, expected);
        assert_eq!(
            plane3_relation(2, &coordinates) == 1,
            hyper == PlaneSegmentRelation::Coplanar,
        );
        assert_eq!(plane3_relation(3, &coordinates), core_dimension);
    }

    let segment_cases = [
        ([1, 1, -2], [1, 1, 3], PlaneSegmentRelation::Crossing, 0),
        ([1, 1, 0], [3, 1, 0], PlaneSegmentRelation::Coplanar, 1),
        ([1, 1, 0], [1, 1, 3], PlaneSegmentRelation::EndpointTouch, 0),
        ([1, 1, 2], [1, 1, 3], PlaneSegmentRelation::Above, -1),
    ];
    for (start, end, expected, core_dimension) in segment_cases {
        let start_point = p(start[0], start[1], start[2]);
        let end_point = p(end[0], end[1], end[2]);
        let coordinates = [
            0, 0, 0, 4, 0, 0, 0, 4, 0, start[0], start[1], start[2], end[0], end[1], end[2], 0, 0,
            0,
        ];
        let hyper = classify_plane_segment(&plane, &start_point, &end_point, POLICY)
            .value()
            .unwrap();
        assert_eq!(hyper, expected);
        assert_eq!(
            plane3_relation(4, &coordinates) == 1,
            hyper == PlaneSegmentRelation::Coplanar
        );
        assert_eq!(plane3_relation(5, &coordinates), core_dimension);
    }

    let parallel_distinct = [0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 1, 4, 0, 1, 0, 4, 1];
    assert_eq!(plane3_relation(6, &parallel_distinct), 1);
    assert_eq!(plane3_relation(7, &parallel_distinct), -1);

    let transverse = [0, 0, 0, 4, 0, 0, 0, 4, 0, 1, 0, 0, 1, 4, 0, 1, 0, 4];
    assert_eq!(plane3_relation(6, &transverse), 0);
    assert_eq!(plane3_relation(7, &transverse), 1);
}

#[test]
fn point_line_segment_and_plane_distances_match_with_predicates() {
    let point_coordinates = [0, 0, 0, 3, 4, 12];
    assert_close(
        point3_distance(point_coordinates),
        point_distance_hyper(&p(0, 0, 0), &p(3, 4, 12)),
        "point3 distance",
    );

    let line_coordinates = [0, 0, 0, 0, 0, 5, 3, 4, 2];
    let line_start = p(0, 0, 0);
    let line_end = p(0, 0, 5);
    let line_query = p(3, 4, 2);
    assert_close(
        line3_point_distance(line_coordinates),
        line_distance_hyper(&line_start, &line_end, &line_query),
        "line3-point distance",
    );
    assert_eq!(
        compare_point_line3_distance_squared(
            &line_query,
            &line_start,
            &line_end,
            &25.into(),
            POLICY
        )
        .value(),
        Some(Ordering::Equal),
    );

    let segment_coordinates = [0, 0, 0, 0, 0, 5, 3, 4, 5];
    let segment_query = p(3, 4, 5);
    assert_close(
        segment3_point_distance(segment_coordinates),
        segment_distance_hyper(&line_start, &line_end, &segment_query),
        "segment3-point distance",
    );
    assert_eq!(
        compare_point_segment3_distance_squared(
            &segment_query,
            &line_start,
            &line_end,
            &25.into(),
            POLICY,
        )
        .value(),
        Some(Ordering::Equal),
    );

    let plane_coordinates = [0, 0, 0, 4, 0, 0, 0, 4, 0, 3, 4, 7];
    let plane = plane_from_points(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0));
    let plane_query = p(3, 4, 7);
    assert_close(
        plane3_point_distance(plane_coordinates),
        plane_distance_hyper(&plane, &plane_query),
        "plane3-point distance",
    );
    assert_eq!(
        compare_point_plane_distance_squared(&plane_query, &plane, &49.into(), POLICY).value(),
        Some(Ordering::Equal),
    );
}

#[test]
fn triangle3_point_and_noncoplanar_intersections_match() {
    let base = [0, 0, 0, 6, 0, 0, 0, 6, 0];
    let point_cases = [
        ([1, 1, 0], Triangle3Location::Inside),
        ([3, 0, 0], Triangle3Location::OnEdge),
        ([0, 0, 0], Triangle3Location::OnVertex),
        ([5, 5, 0], Triangle3Location::Outside),
        ([1, 1, 2], Triangle3Location::OffPlane),
    ];
    for (query, expected) in point_cases {
        let a = p(0, 0, 0);
        let b = p(6, 0, 0);
        let c = p(0, 6, 0);
        let q = p(query[0], query[1], query[2]);
        let hyper = classify_point_triangle3(&a, &b, &c, &q, POLICY)
            .value()
            .unwrap();
        assert_eq!(hyper, expected);
        let coordinates = [
            base[0], base[1], base[2], base[3], base[4], base[5], base[6], base[7], base[8],
            query[0], query[1], query[2], 0, 0, 0, 0, 0, 0,
        ];
        assert_eq!(
            triangle3_relation(0, &coordinates) == 1,
            hyper != Triangle3Location::OffPlane
        );
        if hyper != Triangle3Location::OffPlane {
            assert_eq!(
                triangle3_relation(1, &coordinates) == 1,
                !matches!(
                    hyper,
                    Triangle3Location::Outside | Triangle3Location::Degenerate
                ),
            );
            assert_eq!(
                triangle3_relation(2, &coordinates) == 1,
                matches!(
                    hyper,
                    Triangle3Location::OnEdge | Triangle3Location::OnVertex
                ),
            );
            assert_eq!(
                triangle3_relation(3, &coordinates) == 1,
                hyper == Triangle3Location::Inside
            );
        }
    }

    let segment_cases = [
        (
            [1, 1, -2, 1, 1, 2],
            SegmentTriangleIntersection::Proper,
            true,
        ),
        (
            [0, 0, -2, 0, 0, 2],
            SegmentTriangleIntersection::BoundaryTouch,
            true,
        ),
        (
            [8, 8, -2, 8, 8, 2],
            SegmentTriangleIntersection::Disjoint,
            false,
        ),
    ];
    for (segment, expected, intersects) in segment_cases {
        let (a, b, c) = (p(0, 0, 0), p(6, 0, 0), p(0, 6, 0));
        let start = p(segment[0], segment[1], segment[2]);
        let end = p(segment[3], segment[4], segment[5]);
        let hyper = classify_segment_triangle3_intersection(&start, &end, &a, &b, &c, POLICY)
            .value()
            .unwrap();
        assert_eq!(hyper, expected);
        let coordinates = [
            0, 0, 0, 6, 0, 0, 0, 6, 0, segment[0], segment[1], segment[2], segment[3], segment[4],
            segment[5], 0, 0, 0,
        ];
        assert_eq!(triangle3_relation(4, &coordinates) == 1, intersects);
        assert_eq!(
            triangle3_relation(5, &coordinates) == 1,
            intersects,
            "for these noncoplanar cases the spanning segment contains the line/triangle event",
        );
    }

    let first = [0, 0, 0, 6, 0, 0, 0, 6, 0];
    let second = [1, 1, -2, 1, 1, 2, 4, 1, 0];
    let coordinates: [i64; 18] = [first, second].concat().try_into().unwrap();
    let a = p(0, 0, 0);
    let b = p(6, 0, 0);
    let c = p(0, 6, 0);
    let d = p(1, 1, -2);
    let e = p(1, 1, 2);
    let f = p(4, 1, 0);
    let hyper = classify_triangle_triangle3(&a, &b, &c, &d, &e, &f, POLICY)
        .value()
        .unwrap();
    assert!(hyper.relation.intersects());
    assert_eq!(triangle3_relation(6, &coordinates), 1);
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(256))]

    #[test]
    fn integer_orientation3_properties_match(coordinates in prop::array::uniform12(-1_000_i64..=1_000)) {
        let [a, b, c, d] = points4(coordinates);
        let hyper = orient3(&a, &b, &c, &d, POLICY).value().unwrap();
        prop_assert_eq!(orientation3(coordinates), sign_number(hyper));
    }
}

#[test]
#[ignore = "known exactCorelib Plane3d::isCoincident predicate is inverted"]
fn exactcore_identical_planes_do_not_report_coincidence() {
    let coordinates = [0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 0, 4, 0, 0, 0, 4, 0];
    assert_eq!(plane3_relation(7, &coordinates), 2);
}

#[test]
#[ignore = "known exactCorelib Plane3d::isCoincident can label transverse planes coincident"]
fn exactcore_transverse_origin_planes_report_wrong_dimension() {
    let coordinates = [0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 0, 0, 4, 0, 0, 0, 4];
    assert_eq!(plane3_relation(7, &coordinates), 1);
}

#[test]
#[ignore = "known exactCorelib coplanar Triangle3d intersection treats -1 as true"]
fn exactcore_disjoint_coplanar_triangle_and_segment_report_intersection() {
    let coordinates = [0, 0, 0, 6, 0, 0, 0, 6, 0, 8, 8, 0, 9, 8, 0, 0, 0, 0];
    assert_eq!(triangle3_relation(4, &coordinates), 0);
}

#[test]
#[ignore = "known exactCorelib Triangle3d::do_intersect uses dimensions as booleans"]
fn exactcore_disjoint_triangles_report_intersection() {
    let coordinates = [0, 0, 0, 6, 0, 0, 0, 6, 0, 10, 10, 3, 16, 10, 3, 10, 16, 3];
    assert_eq!(triangle3_relation(6, &coordinates), 0);
}
