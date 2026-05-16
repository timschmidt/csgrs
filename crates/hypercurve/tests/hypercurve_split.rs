use hypercurve::{
    BulgeVertex2, Classification, Contour2, ContourOperand, ContourSplitMap, CurvePolicy, Real,
};

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (s(numerator) / s(denominator)).unwrap()
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

#[test]
fn split_map_includes_endpoints_and_sorted_point_events() {
    let a = rectangle(0, 0, 4, 4);
    let b = contour(&[
        vertex(3, -1, 0),
        vertex(3, 1, 0),
        vertex(1, 1, 0),
        vertex(1, -1, 0),
    ]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    let split_map =
        ContourSplitMap::from_intersections(a.len(), &events, ContourOperand::First, &policy());
    let Classification::Decided(split_map) = split_map else {
        panic!("expected decided split map");
    };

    assert_eq!(split_map.segment_count(), 4);
    assert_eq!(
        split_map.params_for_segment(0).unwrap(),
        [s(0), q(1, 4), q(3, 4), s(1)].as_slice()
    );
    assert_eq!(
        split_map.params_for_segment(1).unwrap(),
        [s(0), s(1)].as_slice()
    );
}

#[test]
fn split_map_deduplicates_overlap_endpoints() {
    let a = rectangle(0, 0, 4, 4);
    let b = contour(&[
        vertex(2, 0, 0),
        vertex(6, 0, 0),
        vertex(6, -2, 0),
        vertex(2, -2, 0),
    ]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    let split_map =
        ContourSplitMap::from_intersections(a.len(), &events, ContourOperand::First, &policy());
    let Classification::Decided(split_map) = split_map else {
        panic!("expected decided split map");
    };

    assert_eq!(
        split_map.params_for_segment(0).unwrap(),
        [s(0), q(1, 2), s(1)].as_slice()
    );
}

#[test]
fn split_map_sorts_reversed_overlap_parameters_for_second_operand() {
    let a = rectangle(0, 0, 4, 4);
    let b = contour(&[
        vertex(5, 0, 0),
        vertex(-1, 0, 0),
        vertex(-1, -1, 0),
        vertex(5, -1, 0),
    ]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    let split_map =
        ContourSplitMap::from_intersections(b.len(), &events, ContourOperand::Second, &policy());
    let Classification::Decided(split_map) = split_map else {
        panic!("expected decided split map");
    };

    assert_eq!(
        split_map.params_for_segment(0).unwrap(),
        [s(0), q(1, 6), q(5, 6), s(1)].as_slice()
    );
}

#[test]
fn split_map_preserves_same_circle_arc_overlap_endpoints() {
    let a = contour(&[vertex(0, 0, 1), vertex(2, 0, 1)]);
    let b = contour(&[vertex(0, 0, 1), vertex(2, 0, 1)]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    let Classification::Decided(split_map) =
        ContourSplitMap::from_intersections(a.len(), &events, ContourOperand::First, &policy())
    else {
        panic!("expected decided split map");
    };

    assert_eq!(
        split_map.params_for_segment(0).unwrap(),
        [s(0), s(1)].as_slice()
    );
    assert_eq!(
        split_map.params_for_segment(1).unwrap(),
        [s(0), s(1)].as_slice()
    );
}

#[test]
fn split_points_flatten_in_segment_order() {
    let a = rectangle(0, 0, 4, 4);
    let b = contour(&[
        vertex(3, -1, 0),
        vertex(3, 1, 0),
        vertex(1, 1, 0),
        vertex(1, -1, 0),
    ]);

    let events = a.intersect_contour(&b, &policy()).unwrap();
    let Classification::Decided(split_map) =
        ContourSplitMap::from_intersections(a.len(), &events, ContourOperand::First, &policy())
    else {
        panic!("expected decided split map");
    };

    let split_points = split_map.split_points();
    assert_eq!(split_points.len(), 10);
    assert_eq!(split_points[0].segment_index, 0);
    assert_eq!(split_points[0].param, s(0));
    assert_eq!(split_points[3].segment_index, 0);
    assert_eq!(split_points[3].param, s(1));
    assert_eq!(split_points[4].segment_index, 1);
}
