use hypercurve::{
    BezierSplitFragment2, BulgeVertex2, CircularArc2, Classification, Contour2, CurveContext,
    CurveError, CurvePathRegionTrim2, CurveRegion2, CurveString2, CurveStringEndpoint2,
    CurveStringTrimPoint2, ExactCurveError, LineSeg2, Point2, Real, Segment2, SegmentKindCounts,
    UncertaintyReason,
};

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn line_segment(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap())
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
    Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(xmin, ymin), s(0)),
        BulgeVertex2::new(p(xmax, ymin), s(0)),
        BulgeVertex2::new(p(xmax, ymax), s(0)),
        BulgeVertex2::new(p(xmin, ymax), s(0)),
    ])
    .unwrap()
}

fn rectangle_region(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> CurveRegion2 {
    CurveRegion2::try_from_native_material_contours(
        vec![rectangle(xmin, ymin, xmax, ymax)],
        &policy(),
    )
    .unwrap()
    .into_value()
}

fn assert_line(segment: &Segment2, start: Point2, end: Point2) {
    let Segment2::Line(line) = segment else {
        panic!("expected line segment");
    };
    assert_eq!(line.start(), &start);
    assert_eq!(line.end(), &end);
}

fn assert_trim_path_line(path: &CurvePathRegionTrim2, start: Point2, end: Point2) {
    let [fragment] = path.fragments() else {
        panic!("expected one retained fragment");
    };
    let BezierSplitFragment2::Materialized { curve, .. } = fragment.trim_fragment().fragment()
    else {
        panic!("expected a materialized retained line");
    };
    assert_eq!(curve.start(), &start);
    assert_eq!(curve.end(), &end);
}

fn policy() -> CurveContext {
    CurveContext::STRICT
}

#[test]
fn curve_string_and_contour_reject_forged_zero_length_segments() {
    let zero = Segment2::Line(LineSeg2::new_unchecked(p(0, 0), p(0, 0)));

    assert_eq!(
        CurveString2::try_new(vec![zero.clone()]).unwrap_err(),
        CurveError::ZeroLengthLine
    );
    assert_eq!(
        Contour2::try_new(vec![
            line_segment(0, 0, 1, 0),
            line_segment(1, 0, 0, 1),
            zero,
        ])
        .unwrap_err(),
        CurveError::ZeroLengthLine
    );
}

#[test]
fn curve_string_endpoint_connection_classifies_exactly() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(1, 0, 2, 0)]).unwrap();

    assert_eq!(
        first
            .endpoint_connection(
                &second,
                CurveStringEndpoint2::End,
                CurveStringEndpoint2::Start,
                &policy(),
            )
            .unwrap(),
        Classification::Decided(true)
    );
}
#[test]
fn curve_string_structural_facts_include_segment_box_counts() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 2, 0),
        line_segment(2, 0, 2, 3),
        line_segment(2, 3, 5, 3),
    ])
    .unwrap();
    let facts = hypercurve::CurveString2::structural_facts(&curve, &policy());

    assert_eq!(facts.segment_kinds, SegmentKindCounts { lines: 3, arcs: 0 });
    assert_eq!(facts.decided_segment_box_count, 3);
    assert!(facts.has_decided_curve_box);
}

#[test]
fn contour_structural_facts_include_segment_box_counts() {
    let contour = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(0, 0), s(0)),
        BulgeVertex2::new(p(4, 0), s(0)),
        BulgeVertex2::new(p(4, 3), s(0)),
        BulgeVertex2::new(p(0, 3), s(0)),
    ])
    .unwrap();
    let facts = hypercurve::Contour2::structural_facts(&contour, &policy());

    assert_eq!(facts.segment_kinds, SegmentKindCounts { lines: 4, arcs: 0 });
    assert_eq!(facts.decided_segment_box_count, 4);
    assert!(facts.has_decided_curve_box);
}

#[test]
fn curve_string_merge_adjacent_collinear_lines() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 2, 0),
        line_segment(2, 0, 5, 0),
        line_segment(5, 0, 5, 2),
        line_segment(5, 2, 5, 6),
    ])
    .unwrap();

    let merged = match curve.merge_adjacent_collinear_lines(&policy()).unwrap() {
        Classification::Decided(curve) => curve,
        other => panic!("expected merged line runs, got {other:?}"),
    };

    assert_eq!(merged.len(), 2);
    assert_line(&merged.segments()[0], p(0, 0), p(5, 0));
    assert_line(&merged.segments()[1], p(5, 0), p(5, 6));
}

#[test]
fn curve_string_merge_adjacent_collinear_lines_preserves_corners() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 2, 0), line_segment(2, 0, 2, 3)]).unwrap();

    let merged = match curve.merge_adjacent_collinear_lines(&policy()).unwrap() {
        Classification::Decided(curve) => curve,
        other => panic!("expected preserved corner, got {other:?}"),
    };

    assert_eq!(merged.len(), 2);
    assert_line(&merged.segments()[0], p(0, 0), p(2, 0));
    assert_line(&merged.segments()[1], p(2, 0), p(2, 3));
}

#[test]
fn curve_string_line_merge_preserves_mixed_segment_kinds() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 1, 0),
        Segment2::Arc(CircularArc2::from_bulge(p(1, 0), p(3, 0), s(1)).unwrap()),
    ])
    .unwrap();

    let merged = match curve.merge_adjacent_collinear_lines(&policy()).unwrap() {
        Classification::Decided(curve) => curve,
        other => panic!("expected preserved mixed segments, got {other:?}"),
    };

    assert_eq!(merged.len(), 2);
    assert!(matches!(merged.segments()[0], Segment2::Line(_)));
    assert!(matches!(merged.segments()[1], Segment2::Arc(_)));
}

#[test]
fn curve_string_merge_adjacent_collinear_lines_preserves_reversal() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 2, 0), line_segment(2, 0, 1, 0)]).unwrap();

    let merged = match curve.merge_adjacent_collinear_lines(&policy()).unwrap() {
        Classification::Decided(curve) => curve,
        other => panic!("expected preserved reversal, got {other:?}"),
    };

    assert_eq!(merged.len(), 2);
    assert_line(&merged.segments()[0], p(0, 0), p(2, 0));
    assert_line(&merged.segments()[1], p(2, 0), p(1, 0));
}

#[test]
fn curve_string_remove_adjacent_reversed_duplicates() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 1, 0),
        line_segment(1, 0, 2, 0),
        line_segment(2, 0, 1, 0),
        line_segment(1, 0, 3, 0),
    ])
    .unwrap();

    let deduped = match curve.remove_adjacent_reversed_duplicates().unwrap() {
        Classification::Decided(curve) => curve,
        other => panic!("expected partial duplicate removal, got {other:?}"),
    };

    assert_eq!(deduped.len(), 2);
    assert_line(&deduped.segments()[0], p(0, 0), p(1, 0));
    assert_line(&deduped.segments()[1], p(1, 0), p(3, 0));
}

#[test]
fn curve_string_remove_adjacent_reversed_duplicates_handles_mixed_segment_kinds() {
    let arc = Segment2::Arc(CircularArc2::from_bulge(p(1, 0), p(3, 0), s(1)).unwrap());
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 1, 0),
        arc.clone(),
        arc.reversed(),
        line_segment(1, 0, 0, 0),
        line_segment(0, 0, -1, 0),
    ])
    .unwrap();

    let deduped = match curve.remove_adjacent_reversed_duplicates().unwrap() {
        Classification::Decided(curve) => curve,
        other => panic!("expected mixed duplicate removal, got {other:?}"),
    };

    assert_eq!(deduped.len(), 1);
    assert_line(&deduped.segments()[0], p(0, 0), p(-1, 0));
}

#[test]
fn curve_string_remove_adjacent_reversed_duplicates_evidence_empty_output_as_boundary() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 1, 0),
        line_segment(1, 0, 2, 0),
        line_segment(2, 0, 1, 0),
        line_segment(1, 0, 0, 0),
    ])
    .unwrap();

    assert_eq!(
        curve.remove_adjacent_reversed_duplicates().unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_remove_adjacent_reversed_duplicates_keeps_partial_backtrack() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 3, 0), line_segment(3, 0, 1, 0)]).unwrap();

    let deduped = match curve.remove_adjacent_reversed_duplicates().unwrap() {
        Classification::Decided(curve) => curve,
        other => panic!("expected preserved partial backtrack, got {other:?}"),
    };

    assert_eq!(deduped.len(), 2);
    assert_line(&deduped.segments()[0], p(0, 0), p(3, 0));
    assert_line(&deduped.segments()[1], p(3, 0), p(1, 0));
}
#[test]
fn curve_string_link_preserves_mixed_segment_kinds() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(1, 0), p(3, 0), s(1)).unwrap(),
    )])
    .unwrap();

    let Classification::Decided(Some(linked)) =
        first.link_connected_endpoints(&second, &policy()).unwrap()
    else {
        panic!("exact endpoint link should materialize");
    };
    assert!(matches!(
        linked.segments(),
        [Segment2::Line(_), Segment2::Arc(_)]
    ));
}

#[test]
fn curve_string_link_reverses_second_curve_for_end_to_end_match() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 2, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(4, 0, 2, 0)]).unwrap();

    let Classification::Decided(Some(linked)) =
        first.link_connected_endpoints(&second, &policy()).unwrap()
    else {
        panic!("end-to-end link should materialize");
    };
    assert_line(&linked.segments()[0], p(0, 0), p(2, 0));
    assert_line(&linked.segments()[1], p(2, 0), p(4, 0));
}

#[test]
fn curve_string_ordered_link_materializes_multistep_chain() {
    let curves = vec![
        CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap(),
        CurveString2::try_new(vec![line_segment(2, 0, 1, 0)]).unwrap(),
        CurveString2::try_new(vec![line_segment(2, 0, 3, 0)]).unwrap(),
    ];
    let Classification::Decided(linked) =
        CurveString2::link_ordered_connected_endpoints(curves.clone(), &policy()).unwrap()
    else {
        panic!("ordered link should materialize");
    };
    assert_eq!(linked.len(), 3);
    assert_eq!(linked.start(), Some(&p(0, 0)));
    assert_eq!(linked.end(), Some(&p(3, 0)));

    let Classification::Decided(borrowed) =
        CurveString2::link_ordered_connected_endpoints_borrowed(&curves, &policy()).unwrap()
    else {
        panic!("borrowed ordered link should materialize");
    };
    assert_eq!(borrowed, linked);
}

#[test]
fn curve_string_ordered_link_evidence_disconnected_step() {
    let curves = vec![
        CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap(),
        CurveString2::try_new(vec![line_segment(2, 0, 3, 0)]).unwrap(),
    ];
    assert_eq!(
        CurveString2::link_ordered_connected_endpoints(curves, &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_connect_materializes_connector_and_preserves_geometry() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(3, 0), p(5, 0), s(1)).unwrap(),
    )])
    .unwrap();
    let Classification::Decided(connected) = first
        .connect_end_to_start_with_line(&second, &policy())
        .unwrap()
    else {
        panic!("connector should materialize");
    };
    assert_eq!(connected.len(), 3);
    assert_line(&connected.segments()[1], p(1, 0), p(3, 0));
    assert!(matches!(connected.segments()[2], Segment2::Arc(_)));
}

#[test]
fn curve_string_connect_nearest_endpoints_evidence_tie_boundary() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 2, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(1, 3, 1, 5)]).unwrap();
    assert_eq!(
        first
            .connect_nearest_endpoints_with_line(&second, &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_connect_end_to_start_blocks_already_connected_endpoints() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(1, 0, 2, 0)]).unwrap();
    assert_eq!(
        first
            .connect_end_to_start_with_line(&second, &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_connect_and_endpoint_classification_reject_empty_input() {
    let empty = CurveString2::new_unchecked(Vec::new());
    let nonempty = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    assert_eq!(
        empty
            .connect_end_to_start_with_line(&nonempty, &policy())
            .unwrap_err(),
        CurveError::EmptyCurveString
    );
    assert_eq!(
        empty
            .endpoint_connection(
                &nonempty,
                CurveStringEndpoint2::Start,
                CurveStringEndpoint2::Start,
                &policy(),
            )
            .unwrap_err(),
        CurveError::EmptyCurveString
    );
}

#[test]

fn curve_string_extend_line_start_to_exact_target() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 2, 0), line_segment(2, 0, 2, 2)]).unwrap();

    let Classification::Decided(extended) = curve
        .extend_line_endpoint_to_point(CurveStringEndpoint2::Start, p(-3, 0), &policy())
        .unwrap()
    else {
        panic!("start line extension should materialize");
    };

    assert_eq!(extended.len(), 2);
    assert_eq!(extended.start(), Some(&p(-3, 0)));
    assert_eq!(extended.end(), Some(&p(2, 2)));
}

#[test]
fn curve_string_extend_line_evidence_interior_target_boundary() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();
    assert_eq!(
        curve
            .extend_line_endpoint_to_point(CurveStringEndpoint2::End, p(1, 0), &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_extend_line_evidence_off_support_boundary() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();
    assert_eq!(
        curve
            .extend_line_endpoint_to_point(CurveStringEndpoint2::End, p(5, 1), &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}
#[test]
fn curve_string_extend_arc_endpoint_evidence_off_circle_boundary() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();
    assert_eq!(
        curve
            .extend_line_endpoint_to_point(CurveStringEndpoint2::End, p(3, 0), &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_extend_arc_endpoint_blocks_existing_arc_point() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();
    assert_eq!(
        curve
            .extend_endpoint_to_point(CurveStringEndpoint2::End, p(1, -1), &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}
#[test]
fn curve_string_chamfer_vertex_by_points_evidence_off_segment_boundary() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    assert_eq!(
        curve
            .chamfer_vertex_by_points(1, &p(5, 0), &p(4, 1), &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_chamfer_vertex_evidence_boundary_parameters() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    assert_eq!(
        curve
            .chamfer_vertex_by_parameters(1, s(1), q(1, 4), &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_chamfer_arc_line_vertex_materializes_exact_segments() {
    let curve = CurveString2::try_new(vec![
        Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap()),
        line_segment(2, 0, 2, 2),
    ])
    .unwrap();

    let Classification::Decided(chamfer) = curve
        .chamfer_vertex_by_parameters(1, q(1, 2), q(1, 2), &policy())
        .unwrap()
    else {
        panic!("arc-line chamfer should materialize");
    };
    let [
        Segment2::Arc(previous),
        Segment2::Line(bevel),
        Segment2::Line(next),
    ] = chamfer.segments()
    else {
        panic!("arc-line chamfer should preserve both source families");
    };
    assert_eq!(previous.start(), &p(0, 0));
    assert_eq!(previous.end(), &p(1, -1));
    assert_eq!(bevel.start(), &p(1, -1));
    assert_eq!(bevel.end(), &p(2, 1));
    assert_eq!(next.start(), &p(2, 1));
    assert_eq!(next.end(), &p(2, 2));
}
#[test]
fn curve_string_chamfer_arc_arc_vertex_materializes_exact_segments() {
    let curve = CurveString2::try_new(vec![
        Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap()),
        Segment2::Arc(CircularArc2::from_bulge(p(2, 0), p(4, 0), s(1)).unwrap()),
    ])
    .unwrap();

    let Classification::Decided(chamfer) = curve
        .chamfer_vertex_by_parameters(1, q(1, 2), q(1, 2), &policy())
        .unwrap()
    else {
        panic!("arc-arc chamfer should materialize");
    };
    let [
        Segment2::Arc(previous),
        Segment2::Line(bevel),
        Segment2::Arc(next),
    ] = chamfer.segments()
    else {
        panic!("arc-arc chamfer should preserve both circular arcs");
    };
    assert_eq!(previous.start(), &p(0, 0));
    assert_eq!(previous.end(), &p(1, -1));
    assert_eq!(bevel.start(), &p(1, -1));
    assert_eq!(bevel.end(), &p(3, -1));
    assert_eq!(next.start(), &p(3, -1));
    assert_eq!(next.end(), &p(4, 0));
}
#[test]
fn curve_string_fillet_arc_line_vertex_materializes_exact_native_segments() {
    let source_arc = CircularArc2::try_from_center(p(5, 2), p(5, 0), p(5, 1), false).unwrap();
    let curve =
        CurveString2::try_new(vec![Segment2::Arc(source_arc), line_segment(5, 0, 0, 0)]).unwrap();

    let Classification::Decided(fillet) = curve
        .fillet_vertex_by_points(1, &p(4, 1), &p(3, 0), &p(3, 1), true, &policy())
        .unwrap()
    else {
        panic!("arc-line fillet should materialize");
    };
    let [
        Segment2::Arc(previous),
        Segment2::Arc(inserted),
        Segment2::Line(next),
    ] = fillet.segments()
    else {
        panic!("arc-line fillet should preserve the source arc");
    };
    assert_eq!(previous.start(), &p(5, 2));
    assert_eq!(previous.end(), &p(4, 1));
    assert_eq!(inserted.start(), &p(4, 1));
    assert_eq!(inserted.end(), &p(3, 0));
    assert!(inserted.is_clockwise());
    assert_eq!(next.start(), &p(3, 0));
    assert_eq!(next.end(), &p(0, 0));
}

#[test]
fn curve_string_fillet_arc_arc_vertex_certifies_distinct_circle_tangents() {
    let previous_center = Point2::new(s(3), q(13, 6));
    let previous_start = Point2::new(s(3), q(13, 3));
    let shared_vertex = p(5, 3);
    let next_center = Point2::new(q(13, 2), s(1));
    let next_end = Point2::new(q(9, 2), q(5, 2));
    let previous_arc = CircularArc2::try_from_center(
        previous_start.clone(),
        shared_vertex.clone(),
        previous_center.clone(),
        false,
    )
    .unwrap();
    let next_arc =
        CircularArc2::try_from_center(shared_vertex, next_end.clone(), next_center.clone(), true)
            .unwrap();
    let Classification::Decided(previous_param) =
        previous_arc.sweep_fraction(&p(3, 0), &policy()).unwrap()
    else {
        panic!("previous tangent parameter should be exact");
    };
    let Classification::Decided(next_param) = next_arc.sweep_fraction(&p(4, 1), &policy()).unwrap()
    else {
        panic!("next tangent parameter should be exact");
    };
    let curve =
        CurveString2::try_new(vec![Segment2::Arc(previous_arc), Segment2::Arc(next_arc)]).unwrap();

    let Classification::Decided(fillet) = curve
        .fillet_vertex_by_parameters(1, previous_param, next_param, &p(3, 1), false, &policy())
        .unwrap()
    else {
        panic!("arc-arc fillet should materialize");
    };
    let [
        Segment2::Arc(previous),
        Segment2::Arc(inserted),
        Segment2::Arc(next),
    ] = fillet.segments()
    else {
        panic!("arc-arc fillet should retain both source circles");
    };
    assert_eq!(previous.start(), &previous_start);
    assert_eq!(previous.end(), &p(3, 0));
    assert_eq!(previous.center(), &previous_center);
    assert_eq!(inserted.center(), &p(3, 1));
    assert_eq!(next.start(), &p(4, 1));
    assert_eq!(next.end(), &next_end);
    assert_eq!(next.center(), &next_center);
}

#[test]
fn curve_string_fillet_evidence_radius_mismatch_boundary() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    assert_eq!(
        curve
            .fillet_vertex_by_points(1, &p(3, 0), &p(4, 1), &p(3, 2), false, &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_fillet_evidence_wrong_orientation_boundary() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    assert_eq!(
        curve
            .fillet_vertex_by_points(1, &p(3, 0), &p(4, 1), &p(3, 1), true, &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_fillet_evidence_boundary_parameters() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 4)]).unwrap();

    assert_eq!(
        curve
            .fillet_vertex_by_points(1, &p(4, 0), &p(4, 1), &p(3, 1), false, &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}
#[test]
fn curve_string_trim_materializes_across_line_segments_with_source_ranges() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 4),
        line_segment(4, 4, 8, 4),
    ])
    .unwrap();

    let Classification::Decided(trimmed) = curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, q(1, 2)),
            CurveStringTrimPoint2::new(2, q(1, 2)),
            &policy(),
        )
        .unwrap()
    else {
        panic!("line-chain trim should materialize");
    };
    assert_eq!(trimmed.len(), 3);
    assert_eq!(trimmed.start(), Some(&p(2, 0)));
    assert_eq!(trimmed.end(), Some(&p(6, 4)));
}

#[test]
fn curve_string_trim_preserves_whole_arc_segment() {
    let arc = Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap());
    let curve = CurveString2::try_new(vec![arc.clone()]).unwrap();

    let Classification::Decided(trimmed) = curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, s(0)),
            CurveStringTrimPoint2::new(0, s(1)),
            &policy(),
        )
        .unwrap()
    else {
        panic!("whole-arc trim should materialize");
    };

    assert_eq!(trimmed.segments(), &[arc]);
}

#[test]
fn curve_string_trim_materializes_exact_partial_arc_from_sweep_parameters() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();
    let half_sqrt_two = (Real::from(2_u8).sqrt().unwrap() / Real::from(2_u8)).unwrap();
    let expected_start = Point2::new(Real::one() - &half_sqrt_two, -half_sqrt_two.clone());
    let expected_end = Point2::new(Real::one() + &half_sqrt_two, -half_sqrt_two);

    let Classification::Decided(trimmed) = curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, q(1, 4)),
            CurveStringTrimPoint2::new(0, q(3, 4)),
            &policy(),
        )
        .unwrap()
    else {
        panic!("arc trim should materialize");
    };
    assert_eq!(trimmed.start(), Some(&expected_start));
    assert_eq!(trimmed.end(), Some(&expected_end));
    let [Segment2::Arc(trimmed_arc)] = trimmed.segments() else {
        panic!("arc trim should preserve the circular-arc family");
    };
    assert_eq!(trimmed_arc.center(), &p(1, 0));
    assert_eq!(trimmed_arc.radius_squared(), Real::one());
    assert!(!trimmed_arc.is_clockwise());
}

#[test]
fn curve_string_trim_retains_non_cardinal_arc_parameter_lineage() {
    let source_arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap();
    let curve = CurveString2::try_new(vec![Segment2::Arc(source_arc.clone())]).unwrap();
    let policy = policy();
    let start_fraction = q(1, 7);
    let end_fraction = q(5, 7);
    let midpoint_fraction = q(3, 7);
    let Classification::Decided(expected_start) = source_arc
        .point_at_sweep_fraction(&start_fraction, &policy)
        .unwrap()
    else {
        panic!("source start fraction should evaluate exactly");
    };
    let Classification::Decided(expected_end) = source_arc
        .point_at_sweep_fraction(&end_fraction, &policy)
        .unwrap()
    else {
        panic!("source end fraction should evaluate exactly");
    };
    let Classification::Decided(expected_midpoint) = source_arc
        .point_at_sweep_fraction(&midpoint_fraction, &policy)
        .unwrap()
    else {
        panic!("source midpoint fraction should evaluate exactly");
    };

    let Classification::Decided(trimmed) = curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, start_fraction),
            CurveStringTrimPoint2::new(0, end_fraction),
            &policy,
        )
        .unwrap()
    else {
        panic!("exact arc trim should materialize");
    };
    let [Segment2::Arc(trimmed_arc)] = trimmed.segments() else {
        panic!("arc trim should preserve the circular-arc family");
    };

    assert_eq!(trimmed_arc.start(), &expected_start);
    assert_eq!(trimmed_arc.end(), &expected_end);
    assert_eq!(
        trimmed_arc.contains_point(trimmed_arc.start(), &policy),
        Classification::Decided(true)
    );
    assert_eq!(
        trimmed_arc.contains_point(trimmed_arc.end(), &policy),
        Classification::Decided(true)
    );
    assert_eq!(
        trimmed_arc
            .point_at_sweep_fraction(&q(1, 2), &policy)
            .unwrap(),
        Classification::Decided(expected_midpoint.clone())
    );
    assert_eq!(
        trimmed_arc.representative_point(&policy).unwrap(),
        Classification::Decided(expected_midpoint.clone())
    );

    let Classification::Decided(repeated_trim) = curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, q(1, 7)),
            CurveStringTrimPoint2::new(0, q(5, 7)),
            &policy,
        )
        .unwrap()
    else {
        panic!("repeated exact arc trim should materialize");
    };
    let [Segment2::Arc(repeated_arc)] = repeated_trim.segments() else {
        panic!("repeated arc trim should preserve the circular-arc family");
    };
    assert!(std::ptr::eq(
        trimmed_arc
            .rational_bezier_decomposition(&policy)
            .unwrap()
            .into_value(),
        repeated_arc
            .rational_bezier_decomposition(&policy)
            .unwrap()
            .into_value()
    ));

    let nested_curve = CurveString2::try_new(vec![Segment2::Arc(trimmed_arc.clone())]).unwrap();
    let Classification::Decided(nested_trim) = nested_curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, q(1, 4)),
            CurveStringTrimPoint2::new(0, q(3, 4)),
            &policy,
        )
        .unwrap()
    else {
        panic!("nested exact arc trim should materialize");
    };
    let [Segment2::Arc(nested_arc)] = nested_trim.segments() else {
        panic!("nested arc trim should preserve the circular-arc family");
    };
    assert_eq!(
        nested_arc
            .point_at_sweep_fraction(&q(1, 2), &policy)
            .unwrap(),
        Classification::Decided(expected_midpoint)
    );
}

#[test]
fn curve_string_trim_rejects_reversed_and_out_of_domain_ranges() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    assert_eq!(
        curve
            .trim_between_parameters(
                CurveStringTrimPoint2::new(0, q(3, 4)),
                CurveStringTrimPoint2::new(0, q(1, 4)),
                &policy(),
            )
            .unwrap_err(),
        CurveError::InvalidCurveRange
    );
    assert_eq!(
        curve
            .trim_between_parameters(
                CurveStringTrimPoint2::new(0, s(-1)),
                CurveStringTrimPoint2::new(0, q(1, 4)),
                &policy(),
            )
            .unwrap_err(),
        CurveError::InvalidCurveParameter
    );
}
#[test]
fn curve_string_trim_between_points_materializes_partial_arc() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();

    let Classification::Decided(trimmed) = curve
        .trim_between_points(&p(0, 0), &p(1, -1), &policy())
        .unwrap()
    else {
        panic!("point-bearing arc trim should materialize");
    };
    assert_eq!(trimmed.start(), Some(&p(0, 0)));
    assert_eq!(trimmed.end(), Some(&p(1, -1)));
    let Segment2::Arc(arc) = &trimmed.segments()[0] else {
        panic!("partial point trim should preserve arc topology");
    };
    assert_eq!(arc.center(), &p(1, 0));
    assert_eq!(arc.radius_squared(), s(1));
}

#[test]
fn curve_string_trim_between_points_accepts_shared_vertex_once() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 2, 0), line_segment(2, 0, 2, 2)]).unwrap();

    let Classification::Decided(trimmed) = curve
        .trim_between_points(&p(2, 0), &p(2, 2), &policy())
        .unwrap()
    else {
        panic!("shared vertex trim should materialize");
    };
    assert_eq!(trimmed.len(), 1);
    assert_eq!(trimmed.start(), Some(&p(2, 0)));
    assert_eq!(trimmed.end(), Some(&p(2, 2)));
}

#[test]
fn curve_string_trim_between_points_evidence_repeated_nonadjacent_point_boundary() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 1, 0),
        line_segment(1, 0, 0, 0),
        line_segment(0, 0, 0, 1),
    ])
    .unwrap();

    assert_eq!(
        curve
            .trim_between_points(&p(0, 0), &p(0, 1), &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}
#[test]
fn curve_string_trim_between_curve_intersections_evidence_ambiguous_cutter_hits() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)]).unwrap();
    let ambiguous_cutter = CurveString2::try_new(vec![
        line_segment(2, -1, 2, 1),
        line_segment(2, 1, 8, 1),
        line_segment(8, 1, 8, -1),
    ])
    .unwrap();
    let end_cutter = CurveString2::try_new(vec![line_segment(9, -1, 9, 1)]).unwrap();

    assert_eq!(
        curve
            .trim_between_curve_intersections(&ambiguous_cutter, &end_cutter, &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_trim_between_curve_intersections_evidence_overlap_blocker() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)]).unwrap();
    let overlapping_cutter = CurveString2::try_new(vec![line_segment(2, 0, 4, 0)]).unwrap();
    let end_cutter = CurveString2::try_new(vec![line_segment(8, -1, 8, 1)]).unwrap();

    assert_eq!(
        curve
            .trim_between_curve_intersections(&overlapping_cutter, &end_cutter, &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}
#[test]
fn curve_string_trim_inside_region_splits_disconnected_inside_windows() {
    let curve = CurveString2::try_new(vec![line_segment(-2, 1, 8, 1)]).unwrap();
    let region = CurveRegion2::try_from_native_material_contours(
        vec![rectangle(0, 0, 2, 2), rectangle(4, 0, 6, 2)],
        &policy(),
    )
    .unwrap()
    .into_value();

    let trimmed = curve
        .trim_inside_region(&region, &policy())
        .unwrap()
        .into_value();

    assert_eq!(trimmed.len(), 2);
    assert_trim_path_line(&trimmed[0], p(0, 1), p(2, 1));
    assert_trim_path_line(&trimmed[1], p(4, 1), p(6, 1));
}

#[test]
fn curve_string_trim_inside_region_respects_holes() {
    let region = CurveRegion2::try_from_native_contours(
        vec![rectangle(0, 0, 10, 4)],
        vec![rectangle(4, 0, 6, 4)],
        &policy(),
    )
    .unwrap()
    .into_value();
    let curve = CurveString2::try_new(vec![line_segment(1, 2, 9, 2)]).unwrap();

    let trimmed = curve
        .trim_inside_region(&region, &policy())
        .unwrap()
        .into_value();

    assert_trim_path_line(&trimmed[0], p(1, 2), p(4, 2));
    assert_trim_path_line(&trimmed[1], p(6, 2), p(9, 2));
}

#[test]
fn curve_string_trim_inside_region_evidence_boundary_overlap_blocker() {
    let region = rectangle_region(0, 0, 4, 4);
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    let error = curve.trim_inside_region(&region, &policy()).unwrap_err();
    assert!(matches!(
        error,
        ExactCurveError::Blocked(blocker)
            if blocker.reason() == UncertaintyReason::Boundary
    ));
}
