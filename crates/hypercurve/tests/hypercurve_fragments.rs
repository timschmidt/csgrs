use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, ContourOperand, CurvePolicy,
    DefaultBackend, LineSeg2, Scalar, Segment2,
};

fn s(value: i32) -> Scalar<DefaultBackend> {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Scalar<DefaultBackend> {
    (s(numerator) / s(denominator)).unwrap()
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

fn arc_overlap_cutter() -> Contour2<DefaultBackend> {
    Contour2::try_new(vec![
        Segment2::Arc(CircularArc2::try_from_center(p(1, -1), p(2, 0), p(1, 0), false).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(2, 0), p(2, -2)).unwrap()),
        Segment2::Line(LineSeg2::try_new(p(2, -2), p(1, -1)).unwrap()),
    ])
    .unwrap()
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

#[test]
fn contour_fragments_split_line_segments_at_point_events() {
    let a = rectangle(0, 0, 4, 4);
    let b = contour(&[
        vertex(3, -1, 0),
        vertex(3, 1, 0),
        vertex(1, 1, 0),
        vertex(1, -1, 0),
    ]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    let fragments = a
        .split_at_intersections(&events, ContourOperand::First, &policy())
        .unwrap();
    let Classification::Decided(fragments) = fragments else {
        panic!("expected decided fragments");
    };

    assert_eq!(fragments.len(), 6);
    assert_eq!(fragments.fragments()[0].source_segment_index, 0);
    assert_eq!(fragments.fragments()[0].source_range.start(), &s(0));
    assert_eq!(fragments.fragments()[0].source_range.end(), &q(1, 4));
    assert_line(&fragments.fragments()[0].segment, p(0, 0), p(1, 0));
    assert_line(&fragments.fragments()[1].segment, p(1, 0), p(3, 0));
    assert_line(&fragments.fragments()[2].segment, p(3, 0), p(4, 0));
}

#[test]
fn contour_fragments_split_line_segments_at_overlap_endpoints() {
    let a = rectangle(0, 0, 4, 4);
    let b = contour(&[
        vertex(2, 0, 0),
        vertex(6, 0, 0),
        vertex(6, -2, 0),
        vertex(2, -2, 0),
    ]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    let Classification::Decided(fragments) = a
        .split_at_intersections(&events, ContourOperand::First, &policy())
        .unwrap()
    else {
        panic!("expected decided fragments");
    };

    assert_eq!(fragments.len(), 5);
    assert_line(&fragments.fragments()[0].segment, p(0, 0), p(2, 0));
    assert_line(&fragments.fragments()[1].segment, p(2, 0), p(4, 0));
}

#[test]
fn contour_fragments_split_arc_segments_at_event_points() {
    let circle = contour(&[vertex(0, 0, 1), vertex(2, 0, 1)]);
    let cutter = contour(&[
        vertex(1, -2, 0),
        vertex(1, 2, 0),
        vertex(3, 2, 0),
        vertex(3, -2, 0),
    ]);

    let events = circle.intersect_contour(&cutter, &policy()).unwrap();
    let Classification::Decided(fragments) = circle
        .split_at_intersections(&events, ContourOperand::First, &policy())
        .unwrap()
    else {
        panic!("expected decided fragments");
    };

    assert_eq!(fragments.len(), 4);
    assert_arc(&fragments.fragments()[0].segment, p(0, 0), p(1, -1));
    assert_arc(&fragments.fragments()[1].segment, p(1, -1), p(2, 0));
    assert_arc(&fragments.fragments()[2].segment, p(2, 0), p(1, 1));
    assert_arc(&fragments.fragments()[3].segment, p(1, 1), p(0, 0));
}

#[test]
fn contour_fragments_split_arc_segments_at_overlap_endpoints() {
    let circle = contour(&[vertex(0, 0, 1), vertex(2, 0, 1)]);
    let cutter = arc_overlap_cutter();

    let events = circle.intersect_contour(&cutter, &policy()).unwrap();
    let Classification::Decided(fragments) = circle
        .split_at_intersections(&events, ContourOperand::First, &policy())
        .unwrap()
    else {
        panic!("expected decided arc overlap fragments");
    };

    assert_eq!(fragments.len(), 3);
    assert_arc(&fragments.fragments()[0].segment, p(0, 0), p(1, -1));
    assert_arc(&fragments.fragments()[1].segment, p(1, -1), p(2, 0));
    assert_arc(&fragments.fragments()[2].segment, p(2, 0), p(0, 0));
}

fn assert_line(
    segment: &Segment2<DefaultBackend>,
    start: hypercurve::Point2<DefaultBackend>,
    end: hypercurve::Point2<DefaultBackend>,
) {
    let Segment2::Line(line) = segment else {
        panic!("expected line fragment");
    };
    assert_eq!(line.start(), &start);
    assert_eq!(line.end(), &end);
}

fn assert_arc(
    segment: &Segment2<DefaultBackend>,
    start: hypercurve::Point2<DefaultBackend>,
    end: hypercurve::Point2<DefaultBackend>,
) {
    let Segment2::Arc(arc) = segment else {
        panic!("expected arc fragment");
    };
    assert_eq!(arc.start(), &start);
    assert_eq!(arc.end(), &end);
    assert_eq!(arc.center(), &p(1, 0));
}
