use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, ContourIntersection,
    ContourIntersectionSet, ContourOperand, ContourOverlapIntersection, ContourPointIntersection,
    CurveError, CurvePolicy, IntersectionKind, LineSeg2, ParamRange, Real, Segment2,
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

fn contour(vertices: &[BulgeVertex2]) -> Contour2 {
    Contour2::from_bulge_vertices(vertices).unwrap()
}

fn rectangle(xmin: i32, ymin: i32, xmax: i32, ymax: i32) -> Contour2 {
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

fn point_event_point(event: &ContourIntersection) -> hypercurve::Point2 {
    let ContourIntersection::Point(point) = event else {
        panic!("expected point event");
    };
    point.point.clone()
}

fn assert_topology_error<T>(result: hypercurve::CurveResult<T>) {
    match result {
        Err(CurveError::Topology(_)) => {}
        Ok(_) => panic!("expected topology error"),
        Err(error) => panic!("expected topology error, got {error:?}"),
    }
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
fn contour_intersection_set_constructor_rejects_duplicate_events() {
    ContourIntersectionSet::new(Vec::new()).unwrap();

    let a = rectangle(0, 0, 4, 4);
    let b = contour(&[
        vertex(3, -1, 0),
        vertex(3, 1, 0),
        vertex(1, 1, 0),
        vertex(1, -1, 0),
    ]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    assert_eq!(events.len(), 2);
    ContourIntersectionSet::new(events.events().to_vec()).unwrap();

    let duplicate = events.events()[0].clone();
    assert_topology_error(ContourIntersectionSet::new(vec![
        duplicate.clone(),
        duplicate,
    ]));
}

#[test]
fn contour_intersection_set_constructor_validates_event_parameters() {
    let point = ContourIntersection::Point(ContourPointIntersection {
        a_segment_index: 0,
        b_segment_index: 1,
        point: p(1, 0),
        a_param: q(1, 2),
        b_param: q(1, 2),
        kind: IntersectionKind::Crossing,
    });
    ContourIntersectionSet::new(vec![point.clone()]).unwrap();

    let mut outside_point = point.clone();
    let ContourIntersection::Point(outside) = &mut outside_point else {
        panic!("expected point event");
    };
    outside.a_param = s(-1);
    assert_topology_error(ContourIntersectionSet::new(vec![outside_point]));

    let mut endpoint_crossing = point.clone();
    let ContourIntersection::Point(endpoint) = &mut endpoint_crossing else {
        panic!("expected point event");
    };
    endpoint.a_param = s(0);
    assert_topology_error(ContourIntersectionSet::new(vec![endpoint_crossing]));

    let mut interior_endpoint = point.clone();
    let ContourIntersection::Point(interior) = &mut interior_endpoint else {
        panic!("expected point event");
    };
    interior.kind = IntersectionKind::Endpoint;
    assert_topology_error(ContourIntersectionSet::new(vec![interior_endpoint]));

    let mut overlap_kind = point;
    let ContourIntersection::Point(overlap) = &mut overlap_kind else {
        panic!("expected point event");
    };
    overlap.kind = IntersectionKind::Overlap;
    assert_topology_error(ContourIntersectionSet::new(vec![overlap_kind]));

    let overlap_segment = Segment2::Line(LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap());
    let overlap = ContourIntersection::Overlap(ContourOverlapIntersection {
        a_segment_index: 0,
        b_segment_index: 1,
        segment: overlap_segment,
        a_range: ParamRange::new(s(0), s(1)),
        b_range: ParamRange::new(s(1), s(0)),
    });
    ContourIntersectionSet::new(vec![overlap.clone()]).unwrap();

    let mut zero_overlap = overlap.clone();
    let ContourIntersection::Overlap(zero) = &mut zero_overlap else {
        panic!("expected overlap event");
    };
    zero.a_range = ParamRange::new(s(1), s(1));
    assert_topology_error(ContourIntersectionSet::new(vec![zero_overlap]));

    let mut degenerate_geometry = overlap.clone();
    let ContourIntersection::Overlap(degenerate) = &mut degenerate_geometry else {
        panic!("expected overlap event");
    };
    degenerate.segment = Segment2::Line(LineSeg2::new_unchecked(p(0, 0), p(0, 0)));
    assert_topology_error(ContourIntersectionSet::new(vec![degenerate_geometry]));

    let mut outside_overlap = overlap;
    let ContourIntersection::Overlap(outside) = &mut outside_overlap else {
        panic!("expected overlap event");
    };
    outside.b_range = ParamRange::new(s(0), s(2));
    assert_topology_error(ContourIntersectionSet::new(vec![outside_overlap]));
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

fn arc_overlap_cutter() -> Contour2 {
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
fn contour_self_events_ignore_ordinary_connectivity() {
    let rectangle = rectangle(0, 0, 4, 4);

    let events = rectangle.intersect_self(&policy()).unwrap();

    assert!(events.is_empty());
}

#[test]
fn contour_self_events_keep_nonadjacent_line_arc_crossing() {
    let contour = contour(&[
        vertex(0, 0, 1),
        vertex(2, 0, 0),
        vertex(3, 2, 0),
        vertex(1, 2, 0),
        vertex(1, -2, 0),
        vertex(3, -3, 0),
        vertex(-1, -3, 0),
    ]);

    let events = contour.intersect_self(&policy()).unwrap();

    assert!(events.events().iter().any(|event| {
        matches!(
            event,
            ContourIntersection::Point(point)
                if point.a_segment_index == 0
                    && point.b_segment_index == 3
                    && point.point == p(1, -1)
                    && point.kind == IntersectionKind::Crossing
        )
    }));
}

#[test]
fn contour_self_events_keep_adjacent_line_arc_crossing_but_drop_shared_endpoint() {
    let contour = contour(&[
        vertex(0, 0, 1),
        vertex(2, 0, 0),
        vertex(0, -2, 0),
        vertex(-1, 0, 0),
    ]);

    let events = contour.intersect_self(&policy()).unwrap();

    assert!(events.events().iter().any(|event| {
        matches!(
            event,
            ContourIntersection::Point(point)
                if point.a_segment_index == 0
                    && point.b_segment_index == 1
                    && point.point == p(1, -1)
        )
    }));
    assert!(!events.events().iter().any(|event| {
        matches!(
            event,
            ContourIntersection::Point(point)
                if point.a_segment_index == 0
                    && point.b_segment_index == 1
                    && point.point == p(2, 0)
        )
    }));
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

#[test]
fn prepared_contour_self_events_match_plain_events() {
    let contour = contour(&[
        vertex(0, 0, 1),
        vertex(2, 0, 0),
        vertex(3, 2, 0),
        vertex(1, 2, 0),
        vertex(1, -2, 0),
        vertex(3, -3, 0),
        vertex(-1, -3, 0),
    ]);
    let policy = policy();
    let prepared = contour.prepare_topology_queries(&policy);

    assert_eq!(
        prepared.intersect_self(&policy).unwrap(),
        contour.intersect_self(&policy).unwrap()
    );
}
