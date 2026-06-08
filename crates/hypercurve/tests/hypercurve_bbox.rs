use hypercurve::{
    Aabb2, BulgeVertex2, CircularArc2, Classification, Contour2, CurvePolicy, CurveString2,
    LineSeg2, Point2, Real, Region2, Segment2, UncertaintyReason,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn line(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> LineSeg2 {
    LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap()
}

fn line_segment(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> Segment2 {
    Segment2::Line(line(start_x, start_y, end_x, end_y))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

fn assert_bbox(bbox: &Aabb2, min: Point2, max: Point2) {
    assert_eq!(bbox.min(), &min);
    assert_eq!(bbox.max(), &max);
}

#[test]
fn aabb_ordering_classifier_rejects_reversed_unchecked_box() {
    let valid = Aabb2::new_unchecked(p(0, -1), p(2, 3));
    let reversed_x = Aabb2::new_unchecked(p(2, -1), p(0, 3));
    let reversed_y = Aabb2::new_unchecked(p(0, 3), p(2, -1));

    assert_eq!(
        valid.has_valid_ordering(&policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        reversed_x.has_valid_ordering(&policy()),
        Classification::Decided(false)
    );
    assert_eq!(
        reversed_y.has_valid_ordering(&policy()),
        Classification::Decided(false)
    );
}

#[test]
fn line_aabb_sorts_reversed_endpoint_coordinates() {
    let Classification::Decided(bbox) = Aabb2::from_line(&line(5, -2, -1, 3), &policy()) else {
        panic!("line bbox should be decided");
    };

    assert_bbox(&bbox, p(-1, -2), p(5, 3));
}

#[test]
fn semicircle_aabb_includes_only_swept_cardinal_extreme() {
    let ccw_top = CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), false).unwrap();
    let Classification::Decided(top_box) = Aabb2::from_arc(&ccw_top, &policy()).unwrap() else {
        panic!("ccw semicircle bbox should be decided");
    };
    assert_bbox(&top_box, p(-5, 0), p(5, 5));

    let clockwise_bottom = CircularArc2::try_from_center(p(5, 0), p(-5, 0), p(0, 0), true).unwrap();
    let Classification::Decided(bottom_box) =
        Aabb2::from_arc(&clockwise_bottom, &policy()).unwrap()
    else {
        panic!("clockwise semicircle bbox should be decided");
    };
    assert_bbox(&bottom_box, p(-5, -5), p(5, 0));
}

#[test]
fn quarter_arc_aabb_uses_endpoint_extrema_when_no_cardinal_point_is_internal() {
    let arc = CircularArc2::try_from_center(p(5, 0), p(0, 5), p(0, 0), false).unwrap();
    let Classification::Decided(bbox) = Aabb2::from_arc(&arc, &policy()).unwrap() else {
        panic!("quarter arc bbox should be decided");
    };

    assert_bbox(&bbox, p(0, 0), p(5, 5));
}

#[test]
fn aabb_overlap_is_inclusive_at_edge_and_corner_contacts() {
    let Classification::Decided(first) = Aabb2::from_line(&line(0, 0, 2, 2), &policy()) else {
        panic!("first line bbox should be decided");
    };
    let Classification::Decided(edge_touching) = Aabb2::from_line(&line(2, -1, 4, 1), &policy())
    else {
        panic!("edge-touching line bbox should be decided");
    };
    let Classification::Decided(corner_touching) = Aabb2::from_line(&line(2, 2, 4, 4), &policy())
    else {
        panic!("corner-touching line bbox should be decided");
    };
    let Classification::Decided(disjoint) = Aabb2::from_line(&line(3, 3, 4, 4), &policy()) else {
        panic!("disjoint line bbox should be decided");
    };

    assert_eq!(
        first.overlaps(&edge_touching, &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        first.overlaps(&corner_touching, &policy()),
        Classification::Decided(true)
    );
    assert_eq!(
        first.overlaps(&disjoint, &policy()),
        Classification::Decided(false)
    );
}

#[test]
fn curve_string_aabb_unions_segment_boxes() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 0),
        line_segment(4, 0, 4, 3),
        line_segment(4, 3, -2, 3),
    ])
    .unwrap();
    let Classification::Decided(bbox) = Aabb2::from_curve_string(&curve, &policy()).unwrap() else {
        panic!("curve string bbox should be decided");
    };

    assert_bbox(&bbox, p(-2, 0), p(4, 3));
}

#[test]
fn region_aabb_unions_material_and_hole_boundaries() {
    let material = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(0, 0), s(0)),
        BulgeVertex2::new(p(10, 0), s(0)),
        BulgeVertex2::new(p(10, 10), s(0)),
        BulgeVertex2::new(p(0, 10), s(0)),
    ])
    .unwrap();
    let hole = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(20, 2), s(0)),
        BulgeVertex2::new(p(24, 2), s(0)),
        BulgeVertex2::new(p(24, 6), s(0)),
        BulgeVertex2::new(p(20, 6), s(0)),
    ])
    .unwrap();
    let region = Region2::new(vec![material], vec![hole]);

    let Classification::Decided(bbox) = Aabb2::from_region(&region, &policy()).unwrap() else {
        panic!("region bbox should be decided");
    };
    assert_bbox(&bbox, p(0, 0), p(24, 10));
}

#[test]
fn empty_region_aabb_is_explicitly_unsupported() {
    assert_eq!(
        Aabb2::from_region(&Region2::empty(), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn curve_string_intersection_broad_phase_keeps_real_hits() {
    let first = CurveString2::try_new(vec![
        line_segment(0, 0, 2, 0),
        line_segment(2, 0, 4, 0),
        line_segment(4, 0, 6, 0),
    ])
    .unwrap();
    let second = CurveString2::try_new(vec![
        line_segment(1, 2, 3, 2),
        line_segment(3, 2, 3, -2),
        line_segment(3, -2, 5, -2),
    ])
    .unwrap();

    let intersections = first.intersect_curve_string(&second, &policy()).unwrap();

    assert_eq!(intersections.len(), 1);
    assert_eq!(intersections[0].a_segment_index, 1);
    assert_eq!(intersections[0].b_segment_index, 1);
}
