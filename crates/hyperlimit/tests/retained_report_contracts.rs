use hyperlimit::{
    CoplanarProjection, CoplanarTriangleClassification, CoplanarTriangleRelation,
    CoplanarTriangleValidationError, PlaneSide, Point2, Point3, PointSegmentLocation,
    PredicateOutcome, PredicatePolicy, RingEvenOddEdgeReport, RingEvenOddReport,
    RingEvenOddValidationError, RingPointLocation, SegmentIntersection, Sign, TriangleDegeneracy,
    TriangleLocation, TrianglePlaneRelation, TrianglePlaneReportValidationError,
    TriangleTriangleIntersection, TriangleTriangleValidationError,
    classify_point_ring_even_odd_report, classify_triangle_against_oriented_plane,
    classify_triangle_triangle3, triangle_plane_relation_from_sides,
};
use hyperreal::Real;

const POLICY: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;

fn p2(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn p3(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn decided<T>(outcome: PredicateOutcome<T>) -> T {
    outcome.value().expect("integer predicate should decide")
}

fn nonstraddling_edge(edge_index: usize) -> RingEvenOddEdgeReport {
    RingEvenOddEdgeReport {
        edge_index,
        segment_location: PointSegmentLocation::OffLine,
        a_above: Some(false),
        b_above: Some(false),
        upward: None,
        orientation: None,
        crosses_right: false,
    }
}

#[test]
fn ring_reports_validate_boundary_parity_and_each_edge_shape() {
    let short = RingEvenOddReport {
        location: RingPointLocation::Outside,
        edge_count: 2,
        crossing_count: 0,
        boundary_edge: None,
        edges: Vec::new(),
    };
    assert_eq!(short.validate(), Ok(()));

    let mut forged = short.clone();
    forged.edges.push(nonstraddling_edge(0));
    assert_eq!(
        forged.validate(),
        Err(RingEvenOddValidationError::EdgeCountMismatch)
    );
    let mut forged = short;
    forged.location = RingPointLocation::Inside;
    assert_eq!(
        forged.validate(),
        Err(RingEvenOddValidationError::LocationMismatch)
    );

    let outside = RingEvenOddReport {
        location: RingPointLocation::Outside,
        edge_count: 3,
        crossing_count: 0,
        boundary_edge: None,
        edges: (0..3).map(nonstraddling_edge).collect(),
    };
    assert_eq!(outside.validate(), Ok(()));

    let boundary_edge = RingEvenOddEdgeReport {
        edge_index: 0,
        segment_location: PointSegmentLocation::OnEndpoint,
        a_above: None,
        b_above: None,
        upward: None,
        orientation: None,
        crosses_right: false,
    };
    let boundary = RingEvenOddReport {
        location: RingPointLocation::Boundary,
        edge_count: 3,
        crossing_count: 0,
        boundary_edge: Some(0),
        edges: vec![boundary_edge.clone()],
    };
    assert_eq!(boundary.validate(), Ok(()));

    let mut forged = boundary.clone();
    forged.edges[0].upward = Some(true);
    assert_eq!(
        forged.validate(),
        Err(RingEvenOddValidationError::BoundaryMismatch)
    );

    let mut forged = outside.clone();
    forged.edges[0].a_above = None;
    assert_eq!(
        forged.validate(),
        Err(RingEvenOddValidationError::MissingStraddleFacts)
    );

    let mut forged = outside.clone();
    forged.edges[0].orientation = Some(Sign::Positive);
    assert_eq!(
        forged.validate(),
        Err(RingEvenOddValidationError::UnexpectedStraddleFacts)
    );

    let mut straddling = nonstraddling_edge(0);
    straddling.b_above = Some(true);
    let mut forged = outside.clone();
    forged.edges[0] = straddling.clone();
    assert_eq!(
        forged.validate(),
        Err(RingEvenOddValidationError::MissingStraddleFacts)
    );

    straddling.upward = Some(true);
    straddling.orientation = Some(Sign::Positive);
    straddling.crosses_right = false;
    let mut forged = outside.clone();
    forged.edges[0] = straddling;
    assert_eq!(
        forged.validate(),
        Err(RingEvenOddValidationError::CrossingMismatch)
    );

    let mut forged = outside.clone();
    forged.edges[0].edge_index = 3;
    assert_eq!(
        forged.validate(),
        Err(RingEvenOddValidationError::EdgeCountMismatch)
    );
    let mut forged = outside.clone();
    forged.boundary_edge = Some(0);
    assert_eq!(
        forged.validate(),
        Err(RingEvenOddValidationError::BoundaryMismatch)
    );
    let mut forged = outside.clone();
    forged.crossing_count = 1;
    assert_eq!(
        forged.validate(),
        Err(RingEvenOddValidationError::CrossingMismatch)
    );
    let mut forged = outside;
    forged.location = RingPointLocation::Inside;
    assert_eq!(
        forged.validate(),
        Err(RingEvenOddValidationError::LocationMismatch)
    );

    let square = [p2(0, 0), p2(4, 0), p2(4, 4), p2(0, 4)];
    let actual = decided(classify_point_ring_even_odd_report(
        &square,
        &p2(2, 2),
        POLICY,
    ));
    assert_eq!(
        actual.validate_against_sources(&square, &p2(5, 2), POLICY),
        Err(RingEvenOddValidationError::SourceReplayMismatch)
    );
}

#[test]
fn coplanar_reports_validate_projection_completeness_and_derived_relation() {
    let projectionless = CoplanarTriangleClassification {
        projection: None,
        relation: CoplanarTriangleRelation::Unknown,
        edge_intersections: Vec::new(),
        right_vertices_in_left: [None; 3],
        left_vertices_in_right: [None; 3],
    };
    assert_eq!(projectionless.validate(), Ok(()));

    let mut forged = projectionless.clone();
    forged
        .edge_intersections
        .push(SegmentIntersection::Disjoint);
    assert_eq!(
        forged.validate(),
        Err(CoplanarTriangleValidationError::ProjectionlessUnknownHasFacts)
    );
    let mut forged = projectionless.clone();
    forged.relation = CoplanarTriangleRelation::Disjoint;
    assert_eq!(
        forged.validate(),
        Err(CoplanarTriangleValidationError::DecidedRelationWithoutProjection)
    );

    let partial_unknown = CoplanarTriangleClassification {
        projection: Some(CoplanarProjection::Xy),
        edge_intersections: vec![SegmentIntersection::Disjoint],
        ..projectionless.clone()
    };
    assert_eq!(partial_unknown.validate(), Ok(()));

    let complete = CoplanarTriangleClassification {
        projection: Some(CoplanarProjection::Xy),
        relation: CoplanarTriangleRelation::Disjoint,
        edge_intersections: vec![SegmentIntersection::Disjoint; 9],
        right_vertices_in_left: [Some(TriangleLocation::Outside); 3],
        left_vertices_in_right: [Some(TriangleLocation::Outside); 3],
    };
    assert_eq!(complete.validate(), Ok(()));

    let mut forged = complete.clone();
    forged.edge_intersections.pop();
    assert_eq!(
        forged.validate(),
        Err(CoplanarTriangleValidationError::MissingEdgeIntersections)
    );
    let mut forged = complete.clone();
    forged.right_vertices_in_left[0] = None;
    assert_eq!(
        forged.validate(),
        Err(CoplanarTriangleValidationError::MissingVertexLocations)
    );
    let mut forged = complete.clone();
    forged.relation = CoplanarTriangleRelation::Touching;
    assert_eq!(
        forged.validate(),
        Err(CoplanarTriangleValidationError::RelationMismatch)
    );

    let points = [p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0)];
    assert_eq!(
        projectionless.validate_against_sources(&points, [0, 1, 2], [0, 1, 3], POLICY),
        Err(CoplanarTriangleValidationError::SourceReplayMismatch)
    );
}

#[test]
fn triangle_plane_reports_replay_borrowed_and_indexed_sources() {
    assert_eq!(
        triangle_plane_relation_from_sides([Some(PlaneSide::Above); 3]),
        TrianglePlaneRelation::StrictlyAbove
    );
    assert_eq!(
        triangle_plane_relation_from_sides([Some(PlaneSide::Below); 3]),
        TrianglePlaneRelation::StrictlyBelow
    );
    assert_eq!(
        triangle_plane_relation_from_sides([Some(PlaneSide::On); 3]),
        TrianglePlaneRelation::Coplanar
    );
    assert_eq!(
        triangle_plane_relation_from_sides([
            Some(PlaneSide::Above),
            Some(PlaneSide::Below),
            Some(PlaneSide::On),
        ]),
        TrianglePlaneRelation::Straddling
    );
    assert_eq!(
        triangle_plane_relation_from_sides([None, Some(PlaneSide::On), Some(PlaneSide::On)]),
        TrianglePlaneRelation::Unknown
    );

    let plane = [p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0)];
    let query = [p3(0, 0, 2), p3(1, 0, 2), p3(0, 1, 2)];
    let report = classify_triangle_against_oriented_plane(
        [&plane[0], &plane[1], &plane[2]],
        [&query[0], &query[1], &query[2]],
        POLICY,
    );
    assert_eq!(
        report.validate_against_triangles(
            [&plane[0], &plane[1], &plane[2]],
            [&query[0], &query[1], &query[2]],
            POLICY,
        ),
        Ok(())
    );

    let mut forged = report.clone();
    forged.relation = TrianglePlaneRelation::Coplanar;
    assert_eq!(
        forged.validate(),
        Err(TrianglePlaneReportValidationError::RelationMismatch)
    );

    let mut points = plane.to_vec();
    points.extend(query.clone());
    assert_eq!(
        report.validate_against_sources(&points, [0, 1, 2], [3, 4, 6], POLICY),
        Err(TrianglePlaneReportValidationError::SourceReplayMismatch)
    );
    points.push(p3(0, 0, -2));
    assert_eq!(
        report.validate_against_sources(&points, [0, 1, 2], [3, 4, 6], POLICY),
        Err(TrianglePlaneReportValidationError::SourceReplayMismatch)
    );
}

#[test]
fn triangle_triangle_reports_reject_each_publicly_forgeable_shape() {
    let left = [p3(0, 0, 0), p3(4, 0, 0), p3(0, 4, 0)];
    let crossing = [p3(1, 1, -1), p3(1, 1, 1), p3(3, 1, 0)];
    let coplanar = [p3(1, 1, 0), p3(5, 1, 0), p3(1, 5, 0)];
    let degenerate = [p3(0, 0, 0), p3(1, 1, 1), p3(2, 2, 2)];

    let crossing_report = decided(classify_triangle_triangle3(
        &left[0],
        &left[1],
        &left[2],
        &crossing[0],
        &crossing[1],
        &crossing[2],
        POLICY,
    ));
    let coplanar_report = decided(classify_triangle_triangle3(
        &left[0],
        &left[1],
        &left[2],
        &coplanar[0],
        &coplanar[1],
        &coplanar[2],
        POLICY,
    ));
    let degenerate_report = decided(classify_triangle_triangle3(
        &left[0],
        &left[1],
        &left[2],
        &degenerate[0],
        &degenerate[1],
        &degenerate[2],
        POLICY,
    ));

    assert_eq!(crossing_report.edge_report_count(), 6);
    assert_eq!(crossing_report.validate(), Ok(()));
    assert_eq!(coplanar_report.validate(), Ok(()));
    assert_eq!(degenerate_report.validate(), Ok(()));
    assert_eq!(
        degenerate_report.left_degeneracy,
        TriangleDegeneracy::NonDegenerate
    );
    assert_eq!(
        degenerate_report.right_degeneracy,
        TriangleDegeneracy::Degenerate
    );

    let mut forged = degenerate_report.clone();
    forged.relation = TriangleTriangleIntersection::Disjoint;
    assert_eq!(
        forged.validate(),
        Err(TriangleTriangleValidationError::DegeneracyMismatch)
    );

    let mut forged = crossing_report.clone();
    forged.left_edges_against_right[0] = None;
    assert_eq!(
        forged.validate(),
        Err(TriangleTriangleValidationError::MissingEdgeReports)
    );

    let mut forged = crossing_report.clone();
    forged.relation = TriangleTriangleIntersection::Disjoint;
    assert_eq!(
        forged.validate(),
        Err(TriangleTriangleValidationError::RelationMismatch)
    );

    let mut forged = crossing_report.clone();
    forged.coplanar = coplanar_report.coplanar.clone();
    assert_eq!(
        forged.validate(),
        Err(TriangleTriangleValidationError::UnexpectedCoplanarClassification)
    );

    let mut forged = coplanar_report.clone();
    forged.coplanar = None;
    assert_eq!(
        forged.validate(),
        Err(TriangleTriangleValidationError::MissingCoplanarClassification)
    );

    let mut forged = coplanar_report;
    forged.relation = TriangleTriangleIntersection::CoplanarTouching;
    assert_eq!(
        forged.validate(),
        Err(TriangleTriangleValidationError::CoplanarRelationMismatch)
    );

    let separated = [p3(0, 0, 2), p3(4, 0, 2), p3(0, 4, 2)];
    assert_eq!(
        crossing_report.validate_against_triangles(
            [&left[0], &left[1], &left[2]],
            [&separated[0], &separated[1], &separated[2]],
            POLICY,
        ),
        Err(TriangleTriangleValidationError::SourceReplayMismatch)
    );
}
