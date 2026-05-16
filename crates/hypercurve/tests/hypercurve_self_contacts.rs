use hypercurve::{
    BulgeVertex2, Classification, Contour2, CurvePolicy, CurveString2, LineSeg2, Point2, Real,
    Segment2,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn contour(vertices: &[BulgeVertex2]) -> Contour2 {
    Contour2::from_bulge_vertices(vertices).unwrap()
}

fn line_segment(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap())
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

#[test]
fn curve_string_self_contact_detector_ignores_adjacent_corner() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 2, 0), line_segment(2, 0, 2, 2)]).unwrap();

    assert_eq!(
        curve.has_self_contacts(&policy()).unwrap(),
        Classification::Decided(false)
    );
}

#[test]
fn curve_string_self_contact_detector_finds_nonadjacent_crossing() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 4),
        line_segment(4, 4, 0, 4),
        line_segment(0, 4, 4, 0),
    ])
    .unwrap();

    assert_eq!(
        curve.has_self_contacts(&policy()).unwrap(),
        Classification::Decided(true)
    );
}

#[test]
fn curve_string_self_contact_detector_does_not_ignore_closing_endpoint() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 2, 0),
        line_segment(2, 0, 2, 2),
        line_segment(2, 2, 0, 0),
    ])
    .unwrap();

    assert_eq!(
        curve.has_self_contacts(&policy()).unwrap(),
        Classification::Decided(true)
    );
}

#[test]
fn prepared_curve_string_self_contact_detector_matches_plain_detector() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 2, 0),
        line_segment(2, 0, 2, 2),
        line_segment(2, 2, 0, 0),
    ])
    .unwrap();
    let policy = policy();
    let prepared = curve.prepare_topology_queries(&policy);

    assert_eq!(prepared.curve_string(), &curve);
    assert!(prepared.curve_box().is_some());
    assert_eq!(prepared.segment_boxes().len(), curve.segments().len());
    assert_eq!(
        prepared.has_self_contacts(&policy).unwrap(),
        curve.has_self_contacts(&policy).unwrap()
    );
    assert_eq!(
        prepared.has_self_contacts(&policy).unwrap(),
        Classification::Decided(true)
    );
}

#[test]
fn self_contact_detector_ignores_adjacent_rectangle_vertices() {
    let rectangle = contour(&[
        vertex(0, 0, 0),
        vertex(4, 0, 0),
        vertex(4, 4, 0),
        vertex(0, 4, 0),
    ]);

    assert_eq!(
        rectangle.has_self_contacts(&policy()).unwrap(),
        Classification::Decided(false)
    );
}

#[test]
fn self_contact_detector_finds_nonadjacent_crossing() {
    let bowtie = contour(&[
        vertex(0, 0, 0),
        vertex(4, 4, 0),
        vertex(0, 4, 0),
        vertex(4, 0, 0),
    ]);

    assert_eq!(
        bowtie.has_self_contacts(&policy()).unwrap(),
        Classification::Decided(true)
    );
}

#[test]
fn self_contact_detector_finds_nonadjacent_line_arc_crossing() {
    let contour = contour(&[
        vertex(0, 0, 1),
        vertex(2, 0, 0),
        vertex(3, 2, 0),
        vertex(1, 2, 0),
        vertex(1, -2, 0),
        vertex(3, -3, 0),
        vertex(-1, -3, 0),
    ]);

    assert_eq!(
        contour.has_self_contacts(&policy()).unwrap(),
        Classification::Decided(true)
    );
}

#[test]
fn self_contact_detector_finds_adjacent_line_arc_crossing_beyond_shared_endpoint() {
    let contour = contour(&[
        vertex(0, 0, 1),
        vertex(2, 0, 0),
        vertex(0, -2, 0),
        vertex(-1, 0, 0),
    ]);

    assert_eq!(
        contour.has_self_contacts(&policy()).unwrap(),
        Classification::Decided(true)
    );
}

#[test]
fn prepared_contour_self_contact_detector_matches_plain_detector() {
    let pinched = contour(&[
        vertex(0, 0, 0),
        vertex(2, 0, 0),
        vertex(1, 1, 0),
        vertex(2, 2, 0),
        vertex(0, 2, 0),
        vertex(1, 1, 0),
    ]);
    let policy = policy();
    let prepared = pinched.prepare_topology_queries(&policy);

    assert_eq!(prepared.contour(), &pinched);
    assert!(prepared.contour_box().is_some());
    assert_eq!(prepared.segment_boxes().len(), pinched.segments().len());
    assert_eq!(
        prepared.has_self_contacts(&policy).unwrap(),
        pinched.has_self_contacts(&policy).unwrap()
    );
    assert_eq!(
        prepared.has_self_contacts(&policy).unwrap(),
        Classification::Decided(true)
    );
}

#[test]
fn self_contact_detector_finds_repeated_nonadjacent_endpoint() {
    let pinched = contour(&[
        vertex(0, 0, 0),
        vertex(2, 0, 0),
        vertex(1, 1, 0),
        vertex(2, 2, 0),
        vertex(0, 2, 0),
        vertex(1, 1, 0),
    ]);

    assert_eq!(
        pinched.has_self_contacts(&policy()).unwrap(),
        Classification::Decided(true)
    );
}
