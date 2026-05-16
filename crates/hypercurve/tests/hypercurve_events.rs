use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, ContourIntersection, ContourOperand,
    CurvePolicy, DefaultBackend, IntersectionKind, LineSeg2, Scalar, Segment2,
};

fn s(value: i32) -> Scalar<DefaultBackend> {
    value.into()
}

fn p(x: i32, y: i32) -> hypercurve::Point2<DefaultBackend> {
    hypercurve::Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2<DefaultBackend> {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn contour(vertices: &[BulgeVertex2<DefaultBackend>]) -> Contour2<DefaultBackend> {
    Contour2::from_bulge_vertices(vertices).unwrap()
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2<DefaultBackend> {
    contour(&[
        vertex(xmin, ymin, 0),
        vertex(xmax, ymin, 0),
        vertex(xmax, ymax, 0),
        vertex(xmin, ymax, 0),
    ])
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn point_event_point(
    event: &ContourIntersection<DefaultBackend>,
) -> hypercurve::Point2<DefaultBackend> {
    let ContourIntersection::Point(point) = event else {
        panic!("expected point event");
    };
    point.point.clone()
}

#[test]
fn contour_events_sort_points_by_first_segment_parameter() {
    let a = rectangle(0, 0, 4, 4);
    let b = contour(&[
        vertex(3, -1, 0),
        vertex(3, 1, 0),
        vertex(1, 1, 0),
        vertex(1, -1, 0),
    ]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    assert_eq!(events.len(), 2);

    let sorted = events.sorted_events_for_segment(ContourOperand::First, 0, &policy());
    let Classification::Decided(sorted) = sorted else {
        panic!("expected ordered events");
    };

    assert_eq!(sorted.len(), 2);
    assert_eq!(point_event_point(sorted[0]), p(1, 0));
    assert_eq!(point_event_point(sorted[1]), p(3, 0));
}

#[test]
fn contour_events_sort_line_arc_hits_by_second_segment_parameter() {
    let circle = contour(&[vertex(0, 0, 1), vertex(2, 0, 1)]);
    let cutter = contour(&[
        vertex(1, -2, 0),
        vertex(1, 2, 0),
        vertex(3, 2, 0),
        vertex(3, -2, 0),
    ]);

    let events = circle.intersect_contour(&cutter, &policy()).unwrap();
    assert_eq!(events.len(), 2);

    let sorted = events.sorted_events_for_segment(ContourOperand::Second, 0, &policy());
    let Classification::Decided(sorted) = sorted else {
        panic!("expected ordered line-arc events");
    };

    assert_eq!(sorted.len(), 2);
    assert_eq!(point_event_point(sorted[0]), p(1, -1));
    assert_eq!(point_event_point(sorted[1]), p(1, 1));
}

#[test]
fn contour_events_preserve_line_overlap() {
    let a = rectangle(0, 0, 4, 4);
    let b = contour(&[
        vertex(2, 0, 0),
        vertex(6, 0, 0),
        vertex(6, -2, 0),
        vertex(2, -2, 0),
    ]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    let overlap = events.events().iter().find_map(|event| match event {
        ContourIntersection::Overlap(overlap) => Some(overlap),
        _ => None,
    });
    let overlap = overlap.expect("expected shared line overlap");

    assert_eq!(overlap.a_segment_index, 0);
    assert_eq!(overlap.b_segment_index, 0);
    assert!(matches!(overlap.segment, Segment2::Line(_)));
}

fn arc_overlap_cutter() -> Contour2<DefaultBackend> {
    Contour2::try_new(vec![
        Segment2::Arc(CircularArc2::try_from_center(p(1, -1), p(2, 0), p(1, 0), false).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(2, 0), p(2, -2)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(2, -2), p(1, -1)).unwrap()),
    ])
    .unwrap()
}

#[test]
fn contour_events_preserve_arc_overlap() {
    let a = contour(&[vertex(0, 0, 1), vertex(2, 0, 1)]);
    let b = arc_overlap_cutter();

    let events = a.intersect_contour(&b, &policy()).unwrap();
    let overlap = events.events().iter().find_map(|event| match event {
        ContourIntersection::Overlap(overlap) if matches!(overlap.segment, Segment2::Arc(_)) => {
            Some(overlap)
        }
        _ => None,
    });
    let overlap = overlap.expect("expected shared arc overlap");

    assert_eq!(overlap.a_segment_index, 0);
    assert_eq!(overlap.b_segment_index, 0);
}

#[test]
fn contour_event_kinds_are_carried_to_point_events() {
    let a = rectangle(0, 0, 4, 4);
    let b = contour(&[
        vertex(4, -1, 0),
        vertex(4, 1, 0),
        vertex(6, 1, 0),
        vertex(6, -1, 0),
    ]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    assert!(events.events().iter().any(|event| {
        matches!(
            event,
            ContourIntersection::Point(point)
                if point.point == p(4, 0) && point.kind == IntersectionKind::Endpoint
        )
    }));
}

#[test]
fn contour_event_broad_phase_skips_decided_disjoint_boxes() {
    let a = rectangle(0, 0, 4, 4);
    let b = rectangle(10, 10, 14, 14);

    let events = a.intersect_contour(&b, &policy()).unwrap();

    assert!(events.is_empty());
}

#[test]
fn contour_event_broad_phase_keeps_edge_touching_boxes() {
    let a = rectangle(0, 0, 4, 4);
    let b = rectangle(4, 1, 6, 3);

    let events = a.intersect_contour(&b, &policy()).unwrap();

    assert!(events.events().iter().any(|event| {
        matches!(
            event,
            ContourIntersection::Overlap(overlap)
                if overlap.a_segment_index == 1 && overlap.b_segment_index == 3
        )
    }));
}

#[test]
fn prepared_contour_events_match_plain_events_for_shared_edges() {
    let a = rectangle(0, 0, 4, 4);
    let b = rectangle(4, 1, 6, 3);
    let policy = policy();
    let prepared_a = a.prepare_topology_queries(&policy);
    let prepared_b = b.prepare_topology_queries(&policy);

    assert_eq!(prepared_a.contour(), &a);
    assert!(prepared_a.contour_box().is_some());
    assert_eq!(prepared_a.segment_boxes().len(), a.segments().len());

    let plain_events = a.intersect_contour(&b, &policy).unwrap();
    let prepared_events = prepared_a
        .intersect_prepared_contour(&prepared_b, &policy)
        .unwrap();
    let mixed_events = prepared_a.intersect_contour(&b, &policy).unwrap();

    assert_eq!(prepared_events, plain_events);
    assert_eq!(mixed_events, plain_events);
    assert!(prepared_events.events().iter().any(|event| {
        matches!(
            event,
            ContourIntersection::Overlap(overlap)
                if overlap.a_segment_index == 1 && overlap.b_segment_index == 3
        )
    }));
}

#[test]
fn prepared_contour_events_match_plain_events_for_arc_overlap() {
    let a = contour(&[vertex(0, 0, 1), vertex(2, 0, 1)]);
    let b = arc_overlap_cutter();
    let policy = policy();
    let prepared_a = a.prepare_topology_queries(&policy);
    let prepared_b = b.prepare_topology_queries(&policy);

    let plain_events = a.intersect_contour(&b, &policy).unwrap();
    let prepared_events = prepared_a
        .intersect_prepared_contour(&prepared_b, &policy)
        .unwrap();

    assert_eq!(prepared_events, plain_events);
    assert!(prepared_events.events().iter().any(|event| {
        matches!(
            event,
            ContourIntersection::Overlap(overlap) if matches!(overlap.segment, Segment2::Arc(_))
        )
    }));
}
