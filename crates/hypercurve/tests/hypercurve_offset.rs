use hypercurve::{
    BulgeVertex2, CircularArc2, Classification, Contour2, CurvePolicy, CurveString2, LineSeg2,
    OffsetCap, Point2, Real, Segment2, UncertaintyReason,
};

fn s(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (s(numerator) / s(denominator)).unwrap()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn vertex(x: i32, y: i32, bulge: i32) -> BulgeVertex2 {
    BulgeVertex2::new(p(x, y), s(bulge))
}

fn line_segment(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap())
}

fn assert_line(segment: &Segment2, start: Point2, end: Point2) {
    let Segment2::Line(line) = segment else {
        panic!("expected line segment");
    };
    assert_eq!(line.start(), &start);
    assert_eq!(line.end(), &end);
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

#[test]
fn line_offset_left_moves_by_unit_normal() {
    let line = LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap();
    let offset = line.offset_left(s(2)).unwrap();

    assert_eq!(offset.start(), &p(0, 2));
    assert_eq!(offset.end(), &p(4, 2));
}

#[test]
fn diagonal_line_offset_left_uses_normalized_direction() {
    let line = LineSeg2::try_new(p(0, 0), p(3, 4)).unwrap();
    let offset = line.offset_left(s(5)).unwrap();

    assert_eq!(offset.start(), &p(-4, 3));
    assert_eq!(offset.end(), &p(-1, 7));
}

#[test]
fn counter_clockwise_arc_offset_left_moves_inward() {
    let arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap();
    let Classification::Decided(offset) = arc.offset_left(q(1, 2), &policy()).unwrap() else {
        panic!("positive inward offset should keep a valid arc");
    };

    assert_eq!(offset.start(), &Point2::new(q(1, 2), s(0)));
    assert_eq!(offset.end(), &Point2::new(q(3, 2), s(0)));
    assert_eq!(offset.center(), &p(1, 0));
    assert_eq!(offset.radius_squared(), q(1, 4));
    assert!(!offset.is_clockwise());
    assert_eq!(offset.bulge(), Some(&s(1)));
}

#[test]
fn clockwise_arc_offset_left_moves_outward() {
    let arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1)).unwrap();
    let Classification::Decided(offset) = arc.offset_left(s(1), &policy()).unwrap() else {
        panic!("positive outward offset should keep a valid arc");
    };

    assert_eq!(offset.start(), &p(-1, 0));
    assert_eq!(offset.end(), &p(3, 0));
    assert_eq!(offset.center(), &p(1, 0));
    assert_eq!(offset.radius_squared(), s(4));
    assert!(offset.is_clockwise());
    assert_eq!(offset.bulge(), Some(&s(-1)));
}

#[test]
fn arc_offset_left_rejects_radius_collapse_and_reversal() {
    let arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap();

    assert_eq!(
        arc.offset_left(s(1), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        arc.offset_left(s(2), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn negative_arc_offset_can_cross_the_opposite_side() {
    let arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap();
    let Classification::Decided(offset) = arc.offset_left(s(-1), &policy()).unwrap() else {
        panic!("negative left offset should produce the right-side arc");
    };

    assert_eq!(offset.start(), &p(-1, 0));
    assert_eq!(offset.end(), &p(3, 0));
    assert_eq!(offset.radius_squared(), s(4));
    assert!(!offset.is_clockwise());
}

#[test]
fn segment_offset_dispatches_line_and_arc() {
    let line = Segment2::Line(LineSeg2::try_new(p(0, 0), p(2, 0)).unwrap());
    let Classification::Decided(Segment2::Line(offset_line)) =
        line.offset_left(s(1), &policy()).unwrap()
    else {
        panic!("line segment offset should dispatch to line primitive");
    };
    assert_eq!(offset_line.start(), &p(0, 1));
    assert_eq!(offset_line.end(), &p(2, 1));

    let arc = Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1)).unwrap());
    let Classification::Decided(Segment2::Arc(offset_arc)) =
        arc.offset_left(s(1), &policy()).unwrap()
    else {
        panic!("arc segment offset should dispatch to arc primitive");
    };
    assert_eq!(offset_arc.radius_squared(), s(4));
}

#[test]
fn curve_string_offset_miters_line_line_corner() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 3)]).unwrap();
    let Classification::Decided(offset) =
        curve.offset_left_with_line_joins(s(1), &policy()).unwrap()
    else {
        panic!("line-line offset should be decided");
    };

    assert_eq!(offset.len(), 2);
    assert_line(&offset.segments()[0], p(0, 1), p(3, 1));
    assert_line(&offset.segments()[1], p(3, 1), p(3, 3));
}

#[test]
fn curve_string_offset_skips_collinear_zero_length_join() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 2, 0), line_segment(2, 0, 5, 0)]).unwrap();
    let Classification::Decided(offset) =
        curve.offset_left_with_line_joins(s(1), &policy()).unwrap()
    else {
        panic!("collinear offset should be decided");
    };

    assert_eq!(offset.len(), 2);
    assert_line(&offset.segments()[0], p(0, 1), p(2, 1));
    assert_line(&offset.segments()[1], p(2, 1), p(5, 1));
}

#[test]
fn curve_string_offset_round_joins_mixed_arc_line_corner() {
    let curve = CurveString2::try_new(vec![
        Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1)).unwrap()),
        line_segment(2, 0, 4, 0),
    ])
    .unwrap();
    let Classification::Decided(offset) =
        curve.offset_left_with_line_joins(s(1), &policy()).unwrap()
    else {
        panic!("mixed arc-line offset should be decided");
    };

    assert_eq!(offset.len(), 3);
    let Segment2::Arc(arc) = &offset.segments()[0] else {
        panic!("first offset segment should remain an arc");
    };
    assert_eq!(arc.end(), &p(3, 0));
    let Segment2::Arc(join) = &offset.segments()[1] else {
        panic!("mixed join should be a round arc");
    };
    assert_eq!(join.start(), &p(3, 0));
    assert_eq!(join.end(), &p(2, 1));
    assert_eq!(join.center(), &p(2, 0));
    assert_eq!(join.radius_squared(), s(1));
    assert!(!join.is_clockwise());
    assert_line(&offset.segments()[2], p(2, 1), p(4, 1));
}

#[test]
fn curve_string_offset_round_joins_parallel_reversal() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 2, 0), line_segment(2, 0, 0, 0)]).unwrap();
    let Classification::Decided(offset) =
        curve.offset_left_with_line_joins(s(1), &policy()).unwrap()
    else {
        panic!("parallel reversal offset should be decided");
    };

    assert_eq!(offset.len(), 3);
    assert_line(&offset.segments()[0], p(0, 1), p(2, 1));
    let Segment2::Arc(join) = &offset.segments()[1] else {
        panic!("parallel reversal join should be a round arc");
    };
    assert_eq!(join.start(), &p(2, 1));
    assert_eq!(join.end(), &p(2, -1));
    assert_eq!(join.center(), &p(2, 0));
    assert_eq!(join.radius_squared(), s(1));
    assert!(join.is_clockwise());
    assert_line(&offset.segments()[2], p(2, -1), p(0, -1));
}

#[test]
fn curve_string_joined_offset_propagates_arc_radius_uncertainty() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap(),
    )])
    .unwrap();

    assert_eq!(
        curve.offset_left_with_line_joins(s(1), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn curve_string_zero_offset_preserves_exact_segments() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 3)]).unwrap();
    let Classification::Decided(offset) =
        curve.offset_left_with_line_joins(s(0), &policy()).unwrap()
    else {
        panic!("zero offset should be decided");
    };

    assert_eq!(offset, curve);
}

#[test]
fn curve_string_checked_offset_accepts_simple_open_path() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 3)]).unwrap();
    let Classification::Decided(offset) = curve.offset_left_checked(s(1), &policy()).unwrap()
    else {
        panic!("simple open checked offset should be decided");
    };

    assert_eq!(offset.len(), 2);
}

#[test]
fn curve_string_checked_offset_rejects_self_contacting_result() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 4),
        line_segment(4, 4, 0, 4),
        line_segment(0, 4, 4, 0),
    ])
    .unwrap();

    assert_eq!(
        curve.offset_left_checked(s(0), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn curve_string_offset_outline_dispatches_cap_styles() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 3)]).unwrap();

    assert_eq!(
        curve
            .offset_outline(s(1), OffsetCap::Round, &policy())
            .unwrap(),
        curve.offset_outline_round_caps(s(1), &policy()).unwrap()
    );
    assert_eq!(
        curve
            .offset_outline(s(1), OffsetCap::Butt, &policy())
            .unwrap(),
        curve.offset_outline_butt_caps(s(1), &policy()).unwrap()
    );
    assert_eq!(
        curve
            .offset_outline(s(1), OffsetCap::Square, &policy())
            .unwrap(),
        curve.offset_outline_square_caps(s(1), &policy()).unwrap()
    );
}

#[test]
fn curve_string_round_cap_outline_wraps_single_line() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();
    let Classification::Decided(outline) =
        curve.offset_outline_round_caps(s(1), &policy()).unwrap()
    else {
        panic!("simple line outline should be decided");
    };

    assert_eq!(outline.len(), 4);
    assert_line(&outline.segments()[0], p(0, 1), p(4, 1));
    let Segment2::Arc(end_cap) = &outline.segments()[1] else {
        panic!("end cap should be an arc");
    };
    assert_eq!(end_cap.start(), &p(4, 1));
    assert_eq!(end_cap.end(), &p(4, -1));
    assert_eq!(end_cap.center(), &p(4, 0));
    assert!(end_cap.is_clockwise());
    assert_line(&outline.segments()[2], p(4, -1), p(0, -1));
    let Segment2::Arc(start_cap) = &outline.segments()[3] else {
        panic!("start cap should be an arc");
    };
    assert_eq!(start_cap.start(), &p(0, -1));
    assert_eq!(start_cap.end(), &p(0, 1));
    assert_eq!(start_cap.center(), &p(0, 0));
    assert!(start_cap.is_clockwise());
}

#[test]
fn curve_string_round_cap_outline_keeps_mitered_corners() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 3)]).unwrap();
    let Classification::Decided(outline) =
        curve.offset_outline_round_caps(s(1), &policy()).unwrap()
    else {
        panic!("mitered open outline should be decided");
    };

    assert_eq!(outline.len(), 6);
    assert_line(&outline.segments()[0], p(0, 1), p(3, 1));
    assert_line(&outline.segments()[1], p(3, 1), p(3, 3));
    let Segment2::Arc(end_cap) = &outline.segments()[2] else {
        panic!("end cap should be an arc");
    };
    assert_eq!(end_cap.center(), &p(4, 3));
    assert_line(&outline.segments()[3], p(5, 3), p(5, -1));
    assert_line(&outline.segments()[4], p(5, -1), p(0, -1));
    let Segment2::Arc(start_cap) = &outline.segments()[5] else {
        panic!("start cap should be an arc");
    };
    assert_eq!(start_cap.center(), &p(0, 0));
}

#[test]
fn curve_string_round_cap_outline_rejects_nonpositive_distance() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    assert_eq!(
        curve.offset_outline_round_caps(s(0), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        curve.offset_outline_round_caps(s(-1), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn curve_string_round_cap_outline_rejects_self_contacting_input() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 4),
        line_segment(4, 4, 0, 4),
        line_segment(0, 4, 4, 0),
    ])
    .unwrap();

    assert_eq!(
        curve.offset_outline_round_caps(s(1), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn curve_string_butt_cap_outline_wraps_single_line() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();
    let Classification::Decided(outline) = curve.offset_outline_butt_caps(s(1), &policy()).unwrap()
    else {
        panic!("simple butt-cap line outline should be decided");
    };

    assert_eq!(outline.len(), 4);
    assert_line(&outline.segments()[0], p(0, 1), p(4, 1));
    assert_line(&outline.segments()[1], p(4, 1), p(4, -1));
    assert_line(&outline.segments()[2], p(4, -1), p(0, -1));
    assert_line(&outline.segments()[3], p(0, -1), p(0, 1));
}

#[test]
fn curve_string_butt_cap_outline_connects_arc_endpoint_offsets() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1)).unwrap(),
    )])
    .unwrap();
    let Classification::Decided(outline) =
        curve.offset_outline_butt_caps(q(1, 2), &policy()).unwrap()
    else {
        panic!("arc butt-cap outline should be decided");
    };

    assert_eq!(outline.len(), 4);
    let Segment2::Arc(left) = &outline.segments()[0] else {
        panic!("left offset should remain an arc");
    };
    assert_eq!(left.start(), &Point2::new(q(-1, 2), s(0)));
    assert_eq!(left.end(), &Point2::new(q(5, 2), s(0)));
    assert_eq!(left.center(), &p(1, 0));
    assert_line(
        &outline.segments()[1],
        Point2::new(q(5, 2), s(0)),
        Point2::new(q(3, 2), s(0)),
    );
    let Segment2::Arc(right) = &outline.segments()[2] else {
        panic!("right offset should remain an arc");
    };
    assert_eq!(right.start(), &Point2::new(q(3, 2), s(0)));
    assert_eq!(right.end(), &Point2::new(q(1, 2), s(0)));
    assert_eq!(right.center(), &p(1, 0));
    assert_line(
        &outline.segments()[3],
        Point2::new(q(1, 2), s(0)),
        Point2::new(q(-1, 2), s(0)),
    );
}

#[test]
fn curve_string_butt_cap_outline_keeps_mitered_corners() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 3)]).unwrap();
    let Classification::Decided(outline) = curve.offset_outline_butt_caps(s(1), &policy()).unwrap()
    else {
        panic!("mitered butt-cap outline should be decided");
    };

    assert_eq!(outline.len(), 6);
    assert_line(&outline.segments()[0], p(0, 1), p(3, 1));
    assert_line(&outline.segments()[1], p(3, 1), p(3, 3));
    assert_line(&outline.segments()[2], p(3, 3), p(5, 3));
    assert_line(&outline.segments()[3], p(5, 3), p(5, -1));
    assert_line(&outline.segments()[4], p(5, -1), p(0, -1));
    assert_line(&outline.segments()[5], p(0, -1), p(0, 1));
}

#[test]
fn curve_string_butt_cap_outline_rejects_nonpositive_distance() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    assert_eq!(
        curve.offset_outline_butt_caps(s(0), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        curve.offset_outline_butt_caps(s(-1), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn curve_string_butt_cap_outline_rejects_self_contacting_input() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 4),
        line_segment(4, 4, 0, 4),
        line_segment(0, 4, 4, 0),
    ])
    .unwrap();

    assert_eq!(
        curve.offset_outline_butt_caps(s(1), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn curve_string_square_cap_outline_extends_single_line() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();
    let Classification::Decided(outline) =
        curve.offset_outline_square_caps(s(1), &policy()).unwrap()
    else {
        panic!("simple square-cap line outline should be decided");
    };

    assert_eq!(outline.len(), 4);
    assert_line(&outline.segments()[0], p(-1, 1), p(5, 1));
    assert_line(&outline.segments()[1], p(5, 1), p(5, -1));
    assert_line(&outline.segments()[2], p(5, -1), p(-1, -1));
    assert_line(&outline.segments()[3], p(-1, -1), p(-1, 1));
}

#[test]
fn curve_string_square_cap_outline_extends_mitered_path_end_tangents() {
    let curve =
        CurveString2::try_new(vec![line_segment(0, 0, 4, 0), line_segment(4, 0, 4, 3)]).unwrap();
    let Classification::Decided(outline) =
        curve.offset_outline_square_caps(s(1), &policy()).unwrap()
    else {
        panic!("mitered square-cap outline should be decided");
    };

    assert_eq!(outline.len(), 6);
    assert_line(&outline.segments()[0], p(-1, 1), p(3, 1));
    assert_line(&outline.segments()[1], p(3, 1), p(3, 4));
    assert_line(&outline.segments()[2], p(3, 4), p(5, 4));
    assert_line(&outline.segments()[3], p(5, 4), p(5, -1));
    assert_line(&outline.segments()[4], p(5, -1), p(-1, -1));
    assert_line(&outline.segments()[5], p(-1, -1), p(-1, 1));
}

#[test]
fn curve_string_square_cap_outline_extends_arc_endpoint_tangents() {
    let curve = CurveString2::try_new(vec![Segment2::Arc(
        CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1)).unwrap(),
    )])
    .unwrap();
    let Classification::Decided(outline) = curve
        .offset_outline_square_caps(q(1, 2), &policy())
        .unwrap()
    else {
        panic!("arc square-cap outline should be decided");
    };

    assert_eq!(outline.len(), 8);
    assert_line(
        &outline.segments()[0],
        Point2::new(q(-1, 2), q(-1, 2)),
        Point2::new(q(-1, 2), s(0)),
    );
    let Segment2::Arc(left) = &outline.segments()[1] else {
        panic!("left offset should remain an arc");
    };
    assert_eq!(left.start(), &Point2::new(q(-1, 2), s(0)));
    assert_eq!(left.end(), &Point2::new(q(5, 2), s(0)));
    assert_line(
        &outline.segments()[2],
        Point2::new(q(5, 2), s(0)),
        Point2::new(q(5, 2), q(-1, 2)),
    );
    assert_line(
        &outline.segments()[3],
        Point2::new(q(5, 2), q(-1, 2)),
        Point2::new(q(3, 2), q(-1, 2)),
    );
    assert_line(
        &outline.segments()[4],
        Point2::new(q(3, 2), q(-1, 2)),
        Point2::new(q(3, 2), s(0)),
    );
    let Segment2::Arc(right) = &outline.segments()[5] else {
        panic!("right offset should remain an arc");
    };
    assert_eq!(right.start(), &Point2::new(q(3, 2), s(0)));
    assert_eq!(right.end(), &Point2::new(q(1, 2), s(0)));
    assert_line(
        &outline.segments()[6],
        Point2::new(q(1, 2), s(0)),
        Point2::new(q(1, 2), q(-1, 2)),
    );
    assert_line(
        &outline.segments()[7],
        Point2::new(q(1, 2), q(-1, 2)),
        Point2::new(q(-1, 2), q(-1, 2)),
    );
}

#[test]
fn curve_string_square_cap_outline_rejects_nonpositive_distance() {
    let curve = CurveString2::try_new(vec![line_segment(0, 0, 4, 0)]).unwrap();

    assert_eq!(
        curve.offset_outline_square_caps(s(0), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
    assert_eq!(
        curve.offset_outline_square_caps(s(-1), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn curve_string_square_cap_outline_rejects_self_contacting_input() {
    let curve = CurveString2::try_new(vec![
        line_segment(0, 0, 4, 4),
        line_segment(4, 4, 0, 4),
        line_segment(0, 4, 4, 0),
    ])
    .unwrap();

    assert_eq!(
        curve.offset_outline_square_caps(s(1), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn contour_offset_miters_closed_rectangle_inward() {
    let rectangle = Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(4, 0, 0),
        vertex(4, 4, 0),
        vertex(0, 4, 0),
    ])
    .unwrap();
    let Classification::Decided(offset) = rectangle
        .offset_left_with_line_joins(s(1), &policy())
        .unwrap()
    else {
        panic!("rectangle offset should be decided");
    };

    assert_eq!(offset.len(), 4);
    assert_line(&offset.segments()[0], p(1, 1), p(3, 1));
    assert_line(&offset.segments()[1], p(3, 1), p(3, 3));
    assert_line(&offset.segments()[2], p(3, 3), p(1, 3));
    assert_line(&offset.segments()[3], p(1, 3), p(1, 1));
}

#[test]
fn contour_joined_offset_rejects_collapsed_line_miter() {
    let rectangle = Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(2, 0, 0),
        vertex(2, 2, 0),
        vertex(0, 2, 0),
    ])
    .unwrap();

    assert_eq!(
        rectangle
            .offset_left_with_line_joins(s(1), &policy())
            .unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}

#[test]
fn contour_checked_offset_accepts_simple_rectangle() {
    let rectangle = Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(6, 0, 0),
        vertex(6, 6, 0),
        vertex(0, 6, 0),
    ])
    .unwrap();

    let Classification::Decided(offset) = rectangle.offset_left_checked(s(1), &policy()).unwrap()
    else {
        panic!("simple rectangle checked offset should be decided");
    };
    assert_eq!(offset.len(), 4);
}

#[test]
fn contour_checked_offset_rejects_self_contacting_result() {
    let bowtie = Contour2::from_bulge_vertices(&[
        vertex(0, 0, 0),
        vertex(4, 4, 0),
        vertex(0, 4, 0),
        vertex(4, 0, 0),
    ])
    .unwrap();

    assert_eq!(
        bowtie.offset_left_checked(s(0), &policy()).unwrap(),
        Classification::Uncertain(UncertaintyReason::Unsupported)
    );
}
