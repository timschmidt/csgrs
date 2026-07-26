use hypercurve::{
    BulgeVertex2, Classification, Contour2, ContourPointLocation, CurveError, CurvePolicy,
    FillRule, Real, Segment2, SegmentKindCounts, UncertaintyReason,
};

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> hypercurve::Point2 {
    hypercurve::Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn assert_line(segment: &Segment2, start: hypercurve::Point2, end: hypercurve::Point2) {
    let Segment2::Line(line) = segment else {
        panic!("expected line segment");
    };
    assert_eq!(line.start(), &start);
    assert_eq!(line.end(), &end);
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn rectangle() -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(4, 0, 0),
        vertex(4, 4, 0),
        vertex(0, 4, 0),
    ])
    .unwrap()
}

fn rotated_rectangle() -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(4, 4, 0),
        vertex(0, 4, 0),
        vertex(0, 0, 0),
        vertex(4, 0, 0),
    ])
    .unwrap()
}

fn reversed_rectangle() -> Contour2 {
    Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(0, 4, 0),
        vertex(4, 4, 0),
        vertex(4, 0, 0),
    ])
    .unwrap()
}

#[test]
fn contour_builds_closed_bulge_loop() {
    let contour = rectangle();

    assert_eq!(contour.len(), 4);
    assert_eq!(contour.fill_rule(), FillRule::NonZero);
    assert!(
        contour
            .segments()
            .iter()
            .all(|segment| matches!(segment, Segment2::Line(_)))
    );
}

#[test]
fn contour_rejects_open_segment_chain() {
    let segments = vec![
        vertex(0, 0, 0).segment_to(&vertex(1, 0, 0)).unwrap(),
        vertex(1, 0, 0).segment_to(&vertex(2, 0, 0)).unwrap(),
    ];

    let err = Contour2::try_new(segments).expect_err("open chain is not a contour");
    assert_eq!(err, CurveError::DisconnectedCurveString);
}
#[test]
fn contour_merge_adjacent_collinear_lines() {
    let contour = Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(2, 0, 0),
        vertex(4, 0, 0),
        vertex(4, 4, 0),
        vertex(0, 4, 0),
    ])
    .unwrap();

    let merged = match contour.merge_adjacent_collinear_lines(&policy()).unwrap() {
        Classification::Decided(contour) => contour,
        other => panic!("expected a merged contour, got {other:?}"),
    };

    assert_eq!(merged.len(), 4);
    assert_eq!(merged.fill_rule(), FillRule::NonZero);
    assert_line(&merged.segments()[0], p(0, 0), p(4, 0));
    assert_line(&merged.segments()[1], p(4, 0), p(4, 4));
}

#[test]
fn contour_line_merge_preserves_mixed_segment_kinds() {
    let contour = Contour2::try_new(vec![
        Segment2::Line(hypercurve::LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap()),
        Segment2::Arc(hypercurve::CircularArc2::from_bulge(p(1, 0), p(3, 0), s(1)).unwrap()),
        Segment2::Line(hypercurve::LineSeg2::try_new(p(3, 0), p(0, 0)).unwrap()),
    ])
    .unwrap();

    let merged = match contour.merge_adjacent_collinear_lines(&policy()).unwrap() {
        Classification::Decided(contour) => contour,
        other => panic!("expected an unchanged mixed contour, got {other:?}"),
    };

    assert_eq!(merged.len(), 3);
    assert!(matches!(merged.segments()[0], Segment2::Line(_)));
    assert!(matches!(merged.segments()[1], Segment2::Arc(_)));
    assert!(matches!(merged.segments()[2], Segment2::Line(_)));
}

#[test]
fn contour_merge_adjacent_collinear_lines_merges_wraparound_run() {
    let contour = Contour2::from_bulge_vertices(&[
        vertex(2, 0, 0),
        vertex(4, 0, 0),
        vertex(4, 4, 0),
        vertex(0, 4, 0),
        vertex(0, 0, 0),
    ])
    .unwrap();

    let merged = match contour.merge_adjacent_collinear_lines(&policy()).unwrap() {
        Classification::Decided(contour) => contour,
        other => panic!("expected a wraparound merge, got {other:?}"),
    };

    assert_eq!(merged.len(), 4);
    assert_line(&merged.segments()[0], p(0, 0), p(4, 0));
    assert_line(&merged.segments()[1], p(4, 0), p(4, 4));
    assert_line(&merged.segments()[3], p(0, 4), p(0, 0));
}
#[test]
fn contour_chamfer_preserves_fill_rule() {
    let contour = Contour2::from_bulge_vertices_with_fill_rule(
        &[
            vertex(0, 0, 0),
            vertex(4, 0, 0),
            vertex(4, 4, 0),
            vertex(0, 4, 0),
        ],
        FillRule::EvenOdd,
    )
    .unwrap();

    let Classification::Decided(chamfer) = contour
        .chamfer_vertex_by_parameters(1, q(3, 4), q(1, 4), &policy())
        .unwrap()
    else {
        panic!("chamfer should materialize");
    };
    assert_eq!(chamfer.fill_rule(), FillRule::EvenOdd);
}

#[test]
fn contour_chamfer_evidence_boundary_parameters() {
    assert_eq!(
        rectangle()
            .chamfer_vertex_by_parameters(1, s(1), q(1, 4), &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}
#[test]
fn contour_chamfer_vertex_by_points_evidence_off_segment_boundary() {
    assert_eq!(
        rectangle()
            .chamfer_vertex_by_points(1, &p(5, 0), &p(4, 1), &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn contour_chamfer_line_line_wraparound_vertex_materializes_closed_contour() {
    let Classification::Decided(chamfer) = rectangle()
        .chamfer_vertex_by_parameters(0, q(3, 4), q(1, 4), &policy())
        .unwrap()
    else {
        panic!("wraparound contour chamfer should materialize");
    };

    assert_eq!(chamfer.len(), 5);
    assert_eq!(chamfer.segments()[0].start(), &p(0, 4));
    assert_eq!(chamfer.segments()[0].end(), &p(0, 1));
    assert_eq!(chamfer.segments()[1].start(), &p(0, 1));
    assert_eq!(chamfer.segments()[1].end(), &p(1, 0));
    assert_eq!(chamfer.segments()[2].start(), &p(1, 0));
    assert_eq!(chamfer.segments()[4].end(), &p(0, 4));
}

#[test]
fn contour_chamfer_line_line_wraparound_vertex_by_points_materializes_closed_contour() {
    let Classification::Decided(chamfer) = rectangle()
        .chamfer_vertex_by_points(0, &p(0, 1), &p(1, 0), &policy())
        .unwrap()
    else {
        panic!("point-bearing wraparound chamfer should materialize");
    };

    assert_eq!(chamfer.len(), 5);
    assert_eq!(chamfer.segments()[0].end(), &p(0, 1));
    assert_eq!(chamfer.segments()[1].start(), &p(0, 1));
    assert_eq!(chamfer.segments()[1].end(), &p(1, 0));
    assert_eq!(chamfer.segments()[2].start(), &p(1, 0));
}

#[test]
fn contour_chamfer_rejects_out_of_range_vertex() {
    let contour = rectangle();

    assert_eq!(
        contour
            .chamfer_vertex_by_parameters(4, q(3, 4), q(1, 4), &policy())
            .unwrap_err(),
        CurveError::InvalidCurveRange
    );
}
#[test]
fn contour_fillet_preserves_fill_rule() {
    let contour = Contour2::from_bulge_vertices_with_fill_rule(
        &[
            vertex(0, 0, 0),
            vertex(4, 0, 0),
            vertex(4, 4, 0),
            vertex(0, 4, 0),
        ],
        FillRule::EvenOdd,
    )
    .unwrap();

    let Classification::Decided(fillet) = contour
        .fillet_vertex_by_points(1, &p(3, 0), &p(4, 1), &p(3, 1), false, &policy())
        .unwrap()
    else {
        panic!("fillet should materialize");
    };
    assert_eq!(fillet.fill_rule(), FillRule::EvenOdd);
}

#[test]
fn contour_fillet_evidence_wrong_orientation_boundary() {
    assert_eq!(
        rectangle()
            .fillet_vertex_by_points(1, &p(3, 0), &p(4, 1), &p(3, 1), true, &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn contour_fillet_line_line_wraparound_vertex_materializes_closed_contour() {
    let Classification::Decided(fillet) = rectangle()
        .fillet_vertex_by_points(0, &p(0, 1), &p(1, 0), &p(1, 1), false, &policy())
        .unwrap()
    else {
        panic!("wraparound contour fillet should materialize");
    };

    assert_eq!(fillet.len(), 5);
    assert_eq!(fillet.segments()[0].start(), &p(0, 4));
    assert_eq!(fillet.segments()[0].end(), &p(0, 1));
    let Segment2::Arc(arc) = &fillet.segments()[1] else {
        panic!("wraparound fillet segment should be an arc");
    };
    assert_eq!(arc.start(), &p(0, 1));
    assert_eq!(arc.end(), &p(1, 0));
    assert_eq!(arc.center(), &p(1, 1));
    assert!(!arc.is_clockwise());
    assert_eq!(fillet.segments()[2].start(), &p(1, 0));
    assert_eq!(fillet.segments()[4].end(), &p(0, 4));
}

#[test]
fn contour_fillet_line_line_wraparound_vertex_by_parameters_materializes_closed_contour() {
    let Classification::Decided(fillet) = rectangle()
        .fillet_vertex_by_parameters(0, q(3, 4), q(1, 4), &p(1, 1), false, &policy())
        .unwrap()
    else {
        panic!("parameter wraparound fillet should materialize");
    };

    assert_eq!(fillet.len(), 5);
    assert_eq!(fillet.segments()[0].end(), &p(0, 1));
    let Segment2::Arc(arc) = &fillet.segments()[1] else {
        panic!("wraparound fillet segment should be an arc");
    };
    assert_eq!(arc.start(), &p(0, 1));
    assert_eq!(arc.end(), &p(1, 0));
    assert_eq!(arc.center(), &p(1, 1));
    assert_eq!(fillet.segments()[2].start(), &p(1, 0));
}

#[test]
fn contour_fillet_arc_arc_wraparound_preserves_sources() {
    let previous_center = hypercurve::Point2::new(s(3), q(13, 6));
    let previous_start = hypercurve::Point2::new(s(3), q(13, 3));
    let shared_vertex = p(5, 3);
    let next_center = hypercurve::Point2::new(q(13, 2), s(1));
    let next_end = hypercurve::Point2::new(q(9, 2), q(5, 2));
    let previous_arc = hypercurve::CircularArc2::try_from_center(
        previous_start.clone(),
        shared_vertex.clone(),
        previous_center.clone(),
        false,
    )
    .unwrap();
    let next_arc = hypercurve::CircularArc2::try_from_center(
        shared_vertex,
        next_end.clone(),
        next_center.clone(),
        true,
    )
    .unwrap();
    let contour = Contour2::try_new(vec![
        Segment2::Arc(next_arc),
        Segment2::Line(
            hypercurve::LineSeg2::try_new(next_end.clone(), previous_start.clone()).unwrap(),
        ),
        Segment2::Arc(previous_arc),
    ])
    .unwrap();

    let Classification::Decided(output) = contour
        .fillet_vertex_by_points(0, &p(3, 0), &p(4, 1), &p(3, 1), false, &policy())
        .unwrap()
    else {
        panic!("wraparound arc-arc contour fillet should materialize");
    };
    let [
        Segment2::Arc(previous),
        Segment2::Arc(inserted),
        Segment2::Arc(next),
        Segment2::Line(closing),
    ] = output.segments()
    else {
        panic!("wraparound arc-arc fillet should preserve both source arcs");
    };
    assert_eq!(previous.start(), &previous_start);
    assert_eq!(previous.end(), &p(3, 0));
    assert_eq!(previous.center(), &previous_center);
    assert_eq!(inserted.start(), &p(3, 0));
    assert_eq!(inserted.end(), &p(4, 1));
    assert_eq!(inserted.center(), &p(3, 1));
    assert_eq!(next.start(), &p(4, 1));
    assert_eq!(next.end(), &next_end);
    assert_eq!(next.center(), &next_center);
    assert_eq!(closing.start(), &next_end);
    assert_eq!(closing.end(), &previous_start);
}

#[test]
fn contour_fillet_rejects_out_of_range_vertex() {
    let contour = rectangle();

    assert_eq!(
        contour
            .fillet_vertex_by_points(4, &p(3, 0), &p(4, 1), &p(3, 1), false, &policy())
            .unwrap_err(),
        CurveError::InvalidCurveRange
    );
}

#[test]
fn rectangle_classifies_inside_outside_and_boundary() {
    let contour = rectangle();

    assert_eq!(
        contour.classify_point(&p(1, 1), &policy()),
        Classification::Decided(ContourPointLocation::Inside)
    );
    assert_eq!(
        contour.classify_point(&p(-1, 1), &policy()),
        Classification::Decided(ContourPointLocation::Outside)
    );
    assert_eq!(
        contour.classify_point(&p(4, 2), &policy()),
        Classification::Decided(ContourPointLocation::Boundary)
    );
    assert_eq!(
        contour.classify_point(&p(0, 0), &policy()),
        Classification::Decided(ContourPointLocation::Boundary)
    );
}

#[test]
fn batched_contour_classification_matches_scalar_classification() {
    let contour = rectangle();
    let policy = policy();
    let points = [p(1, 1), p(-1, 1), p(4, 2), p(0, 0), p(9, 2)];
    let facts = hypercurve::Contour2::structural_facts(&contour, &policy);
    assert_eq!(facts.segment_kinds, SegmentKindCounts { lines: 4, arcs: 0 });
    let batched = hypercurve::Contour2::classify_points(&contour, &points, &policy);
    assert_eq!(
        batched,
        points
            .iter()
            .map(|point| contour.classify_point(point, &policy))
            .collect::<Vec<_>>()
    );
}

#[test]
fn batched_line_contour_classification_preserves_half_open_vertex_cases() {
    let contour = Contour2::from_bulge_vertices(&[
        vertex(0, 2, 0),
        vertex(1, 1, 0),
        vertex(2, 0, 0),
        vertex(1, -1, 0),
        vertex(0, -2, 0),
        vertex(-1, -1, 0),
        vertex(-2, 0, 0),
        vertex(-1, 1, 0),
    ])
    .unwrap();
    let policy = policy();
    let points = [
        p(0, 0),
        p(0, 1),
        p(0, 2),
        p(1, 1),
        p(2, 0),
        p(3, 0),
        p(3, 1),
        p(3, 2),
        p(0, -2),
    ];
    let batched = hypercurve::Contour2::classify_points(&contour, &points, &policy);
    assert_eq!(
        batched,
        points
            .iter()
            .map(|point| contour.classify_point(point, &policy))
            .collect::<Vec<_>>()
    );
}

#[test]
fn contour_aabb_miss_classifies_outside_and_zero_winding() {
    let contour = rectangle();

    assert_eq!(
        contour.point_on_boundary(&p(9, 2), &policy()),
        Classification::Decided(false)
    );
    assert_eq!(
        contour.winding_number(&p(9, 2), &policy()),
        Classification::Decided(0)
    );
    assert_eq!(
        contour.classify_point(&p(9, 2), &policy()),
        Classification::Decided(ContourPointLocation::Outside)
    );
}

#[test]
fn contour_aabb_edge_hit_still_checks_boundary() {
    let contour = rectangle();

    assert_eq!(
        contour.point_on_boundary(&p(4, 2), &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        contour.classify_point(&p(4, 2), &policy()),
        Classification::Decided(ContourPointLocation::Boundary)
    );
}

#[test]
fn rectangle_winding_is_positive_inside_and_boundary_is_explicit() {
    let contour = rectangle();

    assert_eq!(
        contour.winding_number(&p(2, 2), &policy()),
        Classification::Decided(1)
    );
    assert_eq!(
        contour.winding_number(&p(4, 2), &policy()),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn exact_boundary_equality_ignores_closed_start_and_direction() {
    let contour = rectangle();
    let rotated = rotated_rectangle();
    let reversed = reversed_rectangle();
    let even_odd = Contour2::from_bulge_vertices_with_fill_rule(
        &[
            vertex(0, 0, 0),
            vertex(4, 0, 0),
            vertex(4, 4, 0),
            vertex(0, 4, 0),
        ],
        FillRule::EvenOdd,
    )
    .unwrap();
    let different = Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(5, 0, 0),
        vertex(5, 4, 0),
        vertex(0, 4, 0),
    ])
    .unwrap();

    assert!(contour.has_same_exact_boundary(&rotated));
    assert!(contour.has_same_exact_boundary(&reversed));
    assert!(!contour.has_same_exact_boundary(&even_odd));
    assert!(!contour.has_same_exact_boundary(&different));
}

#[test]
fn even_odd_fill_uses_winding_parity() {
    let twice = Contour2::from_bulge_vertices_with_fill_rule(
        &[
            vertex(0, 0, 1),
            vertex(2, 0, 1),
            vertex(0, 0, 1),
            vertex(2, 0, 1),
        ],
        FillRule::EvenOdd,
    )
    .unwrap();

    assert_eq!(
        twice.winding_number(&p(1, 0), &policy()),
        Classification::Decided(2)
    );
    assert_eq!(
        twice.classify_point(&p(1, 0), &policy()),
        Classification::Decided(ContourPointLocation::Outside)
    );
}

#[test]
fn circular_contour_winds_positive_semicircle_counter_clockwise() {
    let contour = Contour2::from_bulge_vertices(&[vertex(0, 0, 1), vertex(2, 0, 1)]).unwrap();

    assert_eq!(
        contour.winding_number(&p(1, 0), &policy()),
        Classification::Decided(1)
    );
    assert_eq!(
        contour.classify_point(&p(3, 0), &policy()),
        Classification::Decided(ContourPointLocation::Outside)
    );

    let reversed = Contour2::from_bulge_vertices(&[vertex(2, 0, -1), vertex(0, 0, -1)]).unwrap();
    assert_eq!(
        reversed.winding_number(&p(1, 0), &policy()),
        Classification::Decided(-1)
    );
}
