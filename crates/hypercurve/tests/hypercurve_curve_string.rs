use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, CurveError, CurvePolicy, CurveString2,
    CurveStringEndpoint2, CurveStringEndpointConnectionStatus2, CurveStringLinkKind2,
    CurveStringTrimPoint2, IntersectionKind, LineArcIntersection, LineArcOrder, LineSeg2, Point2,
    Real, Segment2, SegmentIntersection, UncertaintyReason,
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

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn sparse_zigzag(segment_count: i32) -> CurveString2 {
    let mut segments = Vec::with_capacity(segment_count as usize);
    let mut previous = p(0, 0);
    for index in 1..=segment_count {
        let next_y = if index % 2 == 0 { 0 } else { 1 };
        let next = p(index * 3, next_y);
        segments.push(Segment2::Line(
            LineSeg2::try_new(previous, next.clone()).unwrap(),
        ));
        previous = next;
    }
    CurveString2::try_new(segments).unwrap()
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
fn curve_string_endpoint_report_certifies_exact_connection() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(1, 0, 2, 0)]).unwrap();

    let report = first
        .endpoint_connection_report(
            &second,
            CurveStringEndpoint2::End,
            CurveStringEndpoint2::Start,
            &policy(),
        )
        .unwrap();

    assert_eq!(report.first_endpoint(), CurveStringEndpoint2::End);
    assert_eq!(report.second_endpoint(), CurveStringEndpoint2::Start);
    assert_eq!(
        report.status(),
        CurveStringEndpointConnectionStatus2::NativeExact
    );
    assert!(report.topology_status().is_native_exact());
    assert_eq!(report.distance_squared(), &s(0));
}

#[test]
fn curve_string_link_materializes_unique_end_start_connection() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(1, 0, 2, 0)]).unwrap();

    let linked = match first.link_connected_endpoints(&second, &policy()).unwrap() {
        Classification::Decided(Some(linked)) => linked,
        other => panic!("expected decided linked curve string, got {other:?}"),
    };

    assert_eq!(
        linked.report().kind(),
        CurveStringLinkKind2::FirstEndToSecondStart
    );
    assert_eq!(linked.report().first_segment_count(), 1);
    assert_eq!(linked.report().second_segment_count(), 1);
    assert!(linked.report().status().is_native_exact());
    assert_eq!(linked.curve_string().len(), 2);
    assert_eq!(linked.curve_string().start(), Some(&p(0, 0)));
    assert_eq!(linked.curve_string().end(), Some(&p(2, 0)));
}

#[test]
fn curve_string_link_reverses_second_curve_when_endpoints_match_end_to_end() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(2, 0, 1, 0)]).unwrap();

    let linked = match first.link_connected_endpoints(&second, &policy()).unwrap() {
        Classification::Decided(Some(linked)) => linked,
        other => panic!("expected decided linked curve string, got {other:?}"),
    };

    assert_eq!(
        linked.report().kind(),
        CurveStringLinkKind2::FirstEndToSecondEnd
    );
    assert_eq!(linked.curve_string().start(), Some(&p(0, 0)));
    assert_eq!(linked.curve_string().end(), Some(&p(2, 0)));
    assert_eq!(linked.curve_string().segments()[1].start(), &p(1, 0));
    assert_eq!(linked.curve_string().segments()[1].end(), &p(2, 0));
}

#[test]
fn curve_string_link_returns_none_for_certified_disconnected_inputs() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(3, 0, 4, 0)]).unwrap();

    let disconnected = first
        .endpoint_connection_report(
            &second,
            CurveStringEndpoint2::End,
            CurveStringEndpoint2::Start,
            &policy(),
        )
        .unwrap();

    assert_eq!(
        disconnected.status(),
        CurveStringEndpointConnectionStatus2::Disconnected
    );
    assert_eq!(
        first.link_connected_endpoints(&second, &policy()).unwrap(),
        Classification::Decided(None)
    );
}

#[test]
fn curve_string_link_rejects_multiple_exact_endpoint_pairings() {
    let first = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();
    let second = CurveString2::try_new(vec![line_segment(1, 0, 0, 0)]).unwrap();

    assert_eq!(
        first.link_connected_endpoints(&second, &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Boundary)
    );
}

#[test]
fn curve_string_endpoint_report_rejects_empty_unchecked_input() {
    let empty = CurveString2::new_unchecked(Vec::new());
    let nonempty = CurveString2::try_new(vec![line_segment(0, 0, 1, 0)]).unwrap();

    assert_eq!(
        empty
            .endpoint_connection_report(
                &nonempty,
                CurveStringEndpoint2::End,
                CurveStringEndpoint2::Start,
                &policy(),
            )
            .unwrap_err(),
        CurveError::EmptyCurveString
    );
}

#[test]
fn curve_string_trim_materializes_exact_line_subsegment_with_report() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    let trim = curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, q(1, 4)),
            CurveStringTrimPoint2::new(0, q(3, 4)),
            &policy(),
        )
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert_eq!(trim.report().source_segment_count(), 1);
    assert_eq!(trim.report().segment_reports().len(), 1);
    assert_eq!(trim.report().segment_reports()[0].source_segment_index(), 0);
    assert_eq!(
        trim.report().segment_reports()[0].source_range().start(),
        &q(1, 4)
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_range().end(),
        &q(3, 4)
    );
    let trimmed = trim.curve_string().expect("line trim should materialize");
    assert_eq!(trimmed.start(), Some(&p(1, 0)));
    assert_eq!(trimmed.end(), Some(&p(3, 0)));
}

#[test]
fn curve_string_trim_materializes_across_line_segments_with_source_ranges() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 4),
        line_segment(4, 4, 8, 4),
    ])
    .unwrap();

    let trim = curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, q(1, 2)),
            CurveStringTrimPoint2::new(2, q(1, 2)),
            &policy(),
        )
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    let reports = trim.report().segment_reports();
    assert_eq!(reports.len(), 3);
    assert_eq!(reports[0].source_range().start(), &q(1, 2));
    assert_eq!(reports[0].source_range().end(), &s(1));
    assert_eq!(reports[1].source_range().start(), &s(0));
    assert_eq!(reports[1].source_range().end(), &s(1));
    assert_eq!(reports[2].source_range().start(), &s(0));
    assert_eq!(reports[2].source_range().end(), &q(1, 2));

    let trimmed = trim
        .curve_string()
        .expect("line-chain trim should materialize");
    assert_eq!(trimmed.len(), 3);
    assert_eq!(trimmed.start(), Some(&p(2, 0)));
    assert_eq!(trimmed.end(), Some(&p(6, 4)));
}

#[test]
fn curve_string_trim_preserves_whole_arc_segment() {
    let arc = Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap());
    let curve = CurveString2::try_new(vec![arc.clone()]).unwrap();

    let trim = curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, s(0)),
            CurveStringTrimPoint2::new(0, s(1)),
            &policy(),
        )
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert_eq!(trim.curve_string().unwrap().segments(), &[arc]);
}

#[test]
fn curve_string_trim_reports_partial_arc_as_unsupported_without_materializing() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();

    let trim = curve
        .trim_between_parameters(
            CurveStringTrimPoint2::new(0, q(1, 4)),
            CurveStringTrimPoint2::new(0, q(3, 4)),
            &policy(),
        )
        .unwrap();

    assert!(trim.curve_string().is_none());
    assert!(trim.report().status().is_retained_evidence());
    assert_eq!(
        trim.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
    assert_eq!(trim.report().segment_reports().len(), 1);
    assert!(
        trim.report().segment_reports()[0]
            .status()
            .is_retained_evidence()
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_range().start(),
        &q(1, 4)
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_range().end(),
        &q(3, 4)
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
fn curve_string_trim_between_points_materializes_line_subsegment() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    let trim = curve
        .trim_between_points(&p(1, 0), &p(3, 0), &policy())
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert_eq!(trim.report().start().segment_index(), 0);
    assert_eq!(trim.report().start().param(), &q(1, 4));
    assert_eq!(trim.report().end().segment_index(), 0);
    assert_eq!(trim.report().end().param(), &q(3, 4));
    let trimmed = trim.curve_string().expect("point trim should materialize");
    assert_eq!(trimmed.start(), Some(&p(1, 0)));
    assert_eq!(trimmed.end(), Some(&p(3, 0)));
}

#[test]
fn curve_string_trim_between_points_materializes_partial_arc() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();

    let trim = curve
        .trim_between_points(&p(0, 0), &p(1, -1), &policy())
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert_eq!(trim.report().segment_reports().len(), 1);
    assert_eq!(
        trim.report().segment_reports()[0].source_range().start(),
        &s(0)
    );
    assert_eq!(
        trim.report().segment_reports()[0].source_range().end(),
        &q(1, 2)
    );
    let trimmed = trim
        .curve_string()
        .expect("point-bearing arc trim should materialize");
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

    let trim = curve
        .trim_between_points(&p(2, 0), &p(2, 2), &policy())
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert_eq!(trim.report().start().segment_index(), 1);
    assert_eq!(trim.report().start().param(), &s(0));
    let trimmed = trim
        .curve_string()
        .expect("shared vertex trim should materialize");
    assert_eq!(trimmed.len(), 1);
    assert_eq!(trimmed.start(), Some(&p(2, 0)));
    assert_eq!(trimmed.end(), Some(&p(2, 2)));
}

#[test]
fn curve_string_trim_between_points_reports_repeated_nonadjacent_point_boundary() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 1, 0),
        line_segment(1, 0, 0, 0),
        line_segment(0, 0, 0, 1),
    ])
    .unwrap();

    let trim = curve
        .trim_between_points(&p(0, 0), &p(0, 1), &policy())
        .unwrap();

    assert!(trim.curve_string().is_none());
    assert!(trim.report().status().is_retained_evidence());
    assert_eq!(trim.report().blocker(), Some(UncertaintyReason::Boundary));
}

#[test]
fn curve_string_trim_between_curve_intersections_materializes_line_window() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)]).unwrap();
    let start_cutter = CurveString2::try_new(vec![line_segment(2, -1, 2, 1)]).unwrap();
    let end_cutter = CurveString2::try_new(vec![line_segment(8, -1, 8, 1)]).unwrap();

    let trim = curve
        .trim_between_curve_intersections(&start_cutter, &end_cutter, &policy())
        .unwrap();

    assert!(trim.report().status().is_native_exact());
    assert!(trim.report().blocker().is_none());
    assert_eq!(trim.report().start_hits().len(), 1);
    assert_eq!(trim.report().end_hits().len(), 1);
    assert_eq!(trim.report().start_hits()[0].source_segment_index(), 0);
    assert_eq!(trim.report().start_hits()[0].cutter_segment_index(), 0);
    assert_eq!(trim.report().start_hits()[0].point(), &p(2, 0));
    assert_eq!(
        trim.report().start_hits()[0].kind(),
        IntersectionKind::Crossing
    );
    assert_eq!(trim.report().end_hits()[0].point(), &p(8, 0));

    let trim_report = trim
        .report()
        .trim_report()
        .expect("curve trim should retain point trim report");
    assert_eq!(trim_report.start().param(), &q(1, 5));
    assert_eq!(trim_report.end().param(), &q(4, 5));
    let trimmed = trim
        .curve_string()
        .expect("curve-intersection trim should materialize");
    assert_eq!(trimmed.start(), Some(&p(2, 0)));
    assert_eq!(trimmed.end(), Some(&p(8, 0)));
}

#[test]
fn curve_string_trim_between_curve_intersections_reports_ambiguous_cutter_hits() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)]).unwrap();
    let ambiguous_cutter = CurveString2::try_new(vec![
        line_segment(2, -1, 2, 1),
        line_segment(2, 1, 8, 1),
        line_segment(8, 1, 8, -1),
    ])
    .unwrap();
    let end_cutter = CurveString2::try_new(vec![line_segment(9, -1, 9, 1)]).unwrap();

    let trim = curve
        .trim_between_curve_intersections(&ambiguous_cutter, &end_cutter, &policy())
        .unwrap();

    assert!(trim.curve_string().is_none());
    assert!(trim.report().status().is_retained_evidence());
    assert_eq!(trim.report().blocker(), Some(UncertaintyReason::Boundary));
    assert_eq!(trim.report().start_hits().len(), 2);
    assert_eq!(trim.report().end_hits().len(), 1);
    assert!(trim.report().trim_report().is_none());
}

#[test]
fn curve_string_trim_between_curve_intersections_reports_overlap_blocker() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 10, 0)]).unwrap();
    let overlapping_cutter = CurveString2::try_new(vec![line_segment(2, 0, 4, 0)]).unwrap();
    let end_cutter = CurveString2::try_new(vec![line_segment(8, -1, 8, 1)]).unwrap();

    let trim = curve
        .trim_between_curve_intersections(&overlapping_cutter, &end_cutter, &policy())
        .unwrap();

    assert!(trim.curve_string().is_none());
    assert!(trim.report().status().is_retained_evidence());
    assert_eq!(
        trim.report().blocker(),
        Some(UncertaintyReason::Unsupported)
    );
    assert!(trim.report().start_hits().is_empty());
    assert_eq!(trim.report().end_hits().len(), 1);
    assert!(trim.report().trim_report().is_none());
}

#[test]
fn prepared_curve_string_intersections_match_plain_sparse_scan() {
    let curve = sparse_zigzag(80);
    let cutter = CurveString2::try_new(vec![line_segment(121, -2, 121, 3)]).unwrap();
    let policy = policy();
    let prepared_curve = curve.prepare_topology_queries(&policy);
    let prepared_cutter = cutter.prepare_topology_queries(&policy);

    assert_eq!(prepared_curve.curve_string(), &curve);
    assert!(prepared_curve.curve_box().is_some());
    assert_eq!(prepared_curve.segment_boxes().len(), curve.segments().len());

    let plain_events = curve.intersect_curve_string(&cutter, &policy).unwrap();
    let prepared_events = prepared_curve
        .intersect_prepared_curve_string(&prepared_cutter, &policy)
        .unwrap();
    let mixed_events = prepared_curve
        .intersect_curve_string(&cutter, &policy)
        .unwrap();

    assert_eq!(prepared_events, plain_events);
    assert_eq!(mixed_events, plain_events);
    assert_eq!(prepared_events.len(), 1);
}

#[test]
fn prepared_curve_string_intersections_preserve_line_arc_hits() {
    let line_curve = CurveString2::try_new(vec![line_segment(1, -2, 1, 2)]).unwrap();
    let arc_curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();
    let policy = policy();
    let prepared_line = line_curve.prepare_topology_queries(&policy);
    let prepared_arc = arc_curve.prepare_topology_queries(&policy);

    let plain_events = line_curve
        .intersect_curve_string(&arc_curve, &policy)
        .unwrap();
    let prepared_events = prepared_line
        .intersect_prepared_curve_string(&prepared_arc, &policy)
        .unwrap();

    assert_eq!(prepared_events, plain_events);
    assert_eq!(prepared_events.len(), 1);
    let SegmentIntersection::LineArc {
        order,
        result: LineArcIntersection::Point(hit),
    } = &prepared_events[0].relation
    else {
        panic!("expected prepared line-arc point event");
    };
    assert_eq!(*order, LineArcOrder::LineThenArc);
    assert_eq!(hit.point, p(1, -1));
}

#[test]
fn prepared_segment_pair_intersection_matches_plain_segment_relation() {
    let line = Segment2::Line(LineSeg2::try_new(p(1, -2), p(1, 2)).unwrap());
    let arc = Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap());
    let prepared_line = hypercurve::PreparedSegment2::from_segment(&line);
    let prepared_arc = hypercurve::PreparedSegment2::from_segment(&arc);
    let policy = policy();

    let plain = line.intersect_segment(&arc, &policy).unwrap();
    let prepared = prepared_line
        .intersect_prepared_segment(&prepared_arc, &policy)
        .unwrap();

    assert_eq!(prepared, plain);
    let SegmentIntersection::LineArc {
        order: LineArcOrder::LineThenArc,
        result: LineArcIntersection::Point(hit),
    } = prepared
    else {
        panic!("expected prepared line-arc pair to preserve point relation");
    };
    assert_eq!(hit.point, p(1, -1));
}

#[test]
fn prepared_curve_string_intersections_skip_decided_disjoint_boxes() {
    let first = CurveString2::from_bulge_vertices(&[
        BulgeVertex2::new(p(0, 0), s(0)),
        BulgeVertex2::new(p(2, 0), s(0)),
    ])
    .unwrap();
    let second = CurveString2::from_bulge_vertices(&[
        BulgeVertex2::new(p(10, 10), s(0)),
        BulgeVertex2::new(p(12, 10), s(0)),
    ])
    .unwrap();
    let policy = policy();
    let prepared_first = first.prepare_topology_queries(&policy);
    let prepared_second = second.prepare_topology_queries(&policy);

    assert!(
        prepared_first
            .intersect_prepared_curve_string(&prepared_second, &policy)
            .unwrap()
            .is_empty()
    );
}
