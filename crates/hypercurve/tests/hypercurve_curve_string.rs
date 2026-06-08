use hypercurve::{
    BulgeVertex2, CircularArc2, Contour2, CurveError, CurvePolicy, CurveString2,
    LineArcIntersection, LineArcOrder, LineSeg2, Point2, Real, Segment2, SegmentIntersection,
};

fn s(value: i32) -> Real {
    value.into()
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
