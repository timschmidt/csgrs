use hyperlimit::{
    CoplanarProjection, PlaneSide, Point2, Point3, SupportDopAxis3, SupportDopValidationError,
    TriangleLocation, TrianglePlaneRelation, WitnessedSupportDop3, ccw_projected_turn_less,
    classify_point_projected_triangle3, classify_triangle_against_oriented_plane,
    intersect_segment_with_projected_line3, midpoint3, projected_polygon_area2_abs_value,
    projected_vector3,
};
use hyperreal::{Rational, Real};

fn r(value: i64) -> Real {
    Real::from(value)
}

fn q(num: i64, den: u64) -> Real {
    Real::from(Rational::fraction(num, den).expect("test rational denominator is nonzero"))
}

fn p3(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn p2(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

#[test]
fn witnessed_support_dop_builds_replays_and_refreshes() {
    let mut points = vec![p3(0, 0, 0), p3(4, 1, 0), p3(-2, 3, 0)];
    let axes = SupportDopAxis3::orthogonal_axes();

    let mut dop = WitnessedSupportDop3::from_points(&points, &axes).unwrap();

    assert_eq!(dop.vertex_count, points.len());
    assert_eq!(dop.slabs[0].min.vertex, 2);
    assert_eq!(dop.slabs[0].max.vertex, 1);
    assert_eq!(dop.slabs[1].min.vertex, 0);
    assert_eq!(dop.slabs[1].max.vertex, 2);
    dop.validate_against_points(&points).unwrap();

    let classifier = dop.to_support_dop3();
    assert_eq!(classifier.slabs()[0].min_witness, Some(2));
    assert_eq!(classifier.slabs()[0].max_witness, Some(1));

    points[1] = p3(6, 1, 0);
    let report = dop.refresh_for_changed_vertices(&points, &[1]).unwrap();

    assert_eq!(report.changed_vertices, 1);
    assert_eq!(report.axis_count, axes.len());
    assert_eq!(dop.slabs[0].max.vertex, 1);
    assert_eq!(dop.slabs[0].max.distance, r(6));
    dop.validate_against_points(&points).unwrap();

    assert_eq!(
        WitnessedSupportDop3::from_points(&points, &[SupportDopAxis3::new([0, 0, 0])]).unwrap_err(),
        SupportDopValidationError::ZeroAxis
    );
}

#[test]
fn triangle_plane_classifier_retains_sides_and_replays_sources() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(0, 0, 2),
        p3(1, 0, 2),
        p3(0, 1, 2),
        p3(0, 0, -1),
    ];

    let above_query = classify_triangle_against_oriented_plane(
        [&points[0], &points[1], &points[2]],
        [&points[3], &points[4], &points[5]],
    );
    assert_eq!(above_query.relation, TrianglePlaneRelation::StrictlyBelow);
    assert_eq!(above_query.vertex_sides, [Some(PlaneSide::Below); 3]);
    above_query.validate().unwrap();
    above_query
        .validate_against_sources(&points, [0, 1, 2], [3, 4, 5])
        .unwrap();

    let straddling = classify_triangle_against_oriented_plane(
        [&points[0], &points[1], &points[2]],
        [&points[0], &points[3], &points[6]],
    );
    assert_eq!(straddling.relation, TrianglePlaneRelation::Straddling);
    assert_eq!(
        straddling.vertex_sides,
        [
            Some(PlaneSide::On),
            Some(PlaneSide::Below),
            Some(PlaneSide::Above)
        ]
    );
    straddling.validate().unwrap();
}

#[test]
fn projected_planar_helpers_cover_area_turns_and_intersections() {
    let square = vec![p3(0, 0, 0), p3(4, 0, 0), p3(4, 3, 0), p3(0, 3, 0)];
    assert_eq!(
        projected_polygon_area2_abs_value(&square, CoplanarProjection::Xy),
        Some(r(24))
    );

    let midpoint = midpoint3(&square[0], &square[2]);
    assert_eq!(midpoint, Point3::new(r(2), q(3, 2), r(0)));
    assert_eq!(
        projected_vector3(&square[0], &square[2], CoplanarProjection::Xy),
        p2(4, 3)
    );
    assert_eq!(
        classify_point_projected_triangle3(
            &Point3::new(r(2), r(1), r(0)),
            [&square[0], &square[1], &square[2]],
            CoplanarProjection::Xy,
        )
        .value(),
        Some(TriangleLocation::Inside)
    );

    let intersection = intersect_segment_with_projected_line3(
        &p3(0, 0, 0),
        &p3(4, 0, 0),
        &p3(2, -1, 0),
        &p3(2, 1, 0),
        CoplanarProjection::Xy,
    )
    .unwrap();
    assert_eq!(intersection, p3(2, 0, 0));

    assert_eq!(
        ccw_projected_turn_less(&p2(1, 0), &p2(0, 1), &p2(0, -1)),
        Some(true)
    );
}
