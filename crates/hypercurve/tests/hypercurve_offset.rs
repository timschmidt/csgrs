use hypercurve::{
    CircularArc2, Classification, CurveContext, CurveString2, LineSeg2, Point2, Real, Segment2,
    UncertaintyReason,
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

fn line(start_x: i32, start_y: i32, end_x: i32, end_y: i32) -> Segment2 {
    Segment2::Line(LineSeg2::try_new(p(start_x, start_y), p(end_x, end_y)).unwrap())
}

fn assert_line(segment: &Segment2, start: Point2, end: Point2) {
    let Segment2::Line(line) = segment else {
        panic!("expected a line segment");
    };
    assert_eq!(line.start(), &start);
    assert_eq!(line.end(), &end);
}

#[test]
fn native_line_and_arc_parallel_primitives_are_exact() {
    let policy = CurveContext::STRICT;
    let horizontal = LineSeg2::try_new(p(0, 0), p(4, 0)).unwrap();
    let diagonal = LineSeg2::try_new(p(0, 0), p(3, 4)).unwrap();
    assert_line(
        &Segment2::Line(horizontal.offset_left(s(2)).unwrap()),
        p(0, 2),
        p(4, 2),
    );
    assert_line(
        &Segment2::Line(diagonal.offset_left(s(5)).unwrap()),
        p(-4, 3),
        p(-1, 7),
    );

    let counter_clockwise = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap();
    let Classification::Decided(inward) = counter_clockwise.offset_left(q(1, 2), &policy).unwrap()
    else {
        panic!("the inward arc remains nondegenerate");
    };
    assert_eq!(inward.start(), &Point2::new(q(1, 2), s(0)));
    assert_eq!(inward.end(), &Point2::new(q(3, 2), s(0)));
    assert_eq!(inward.radius_squared(), q(1, 4));

    let clockwise = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1)).unwrap();
    let Classification::Decided(outward) = clockwise.offset_left(s(1), &policy).unwrap() else {
        panic!("the clockwise left offset expands");
    };
    assert_eq!(outward.start(), &p(-1, 0));
    assert_eq!(outward.end(), &p(3, 0));
    assert_eq!(outward.radius_squared(), s(4));
}

#[test]
fn primitive_arc_reports_radius_collapse_boundary() {
    let arc = CircularArc2::from_bulge(p(0, 0), p(2, 0), s(1)).unwrap();
    for distance in [s(1), s(2)] {
        assert_eq!(
            arc.offset_left(distance, &CurveContext::STRICT).unwrap(),
            Classification::Uncertain(UncertaintyReason::Unsupported)
        );
    }
}

#[test]
fn native_curve_string_parallel_miters_line_corners() {
    let curve = CurveString2::try_new(vec![line(0, 0, 4, 0), line(4, 0, 4, 3)]).unwrap();
    let Classification::Decided(offset) = curve
        .offset_left_with_line_joins(s(1), &CurveContext::STRICT)
        .unwrap()
    else {
        panic!("line-line parallel is decided");
    };
    assert_eq!(offset.len(), 2);
    assert_line(&offset.segments()[0], p(0, 1), p(3, 1));
    assert_line(&offset.segments()[1], p(3, 1), p(3, 3));
}

#[test]
fn native_curve_string_parallel_rounds_mixed_and_reversal_joins() {
    let mixed = CurveString2::try_new(vec![
        Segment2::Arc(CircularArc2::from_bulge(p(0, 0), p(2, 0), s(-1)).unwrap()),
        line(2, 0, 4, 0),
    ])
    .unwrap();
    let Classification::Decided(mixed) = mixed
        .offset_left_with_line_joins(s(1), &CurveContext::STRICT)
        .unwrap()
    else {
        panic!("mixed native parallel is decided");
    };
    assert_eq!(mixed.len(), 3);
    let Segment2::Arc(join) = &mixed.segments()[1] else {
        panic!("mixed join is circular");
    };
    assert_eq!(
        (join.start(), join.end(), join.center()),
        (&p(3, 0), &p(2, 1), &p(2, 0))
    );

    let reversal = CurveString2::try_new(vec![line(0, 0, 2, 0), line(2, 0, 0, 0)]).unwrap();
    let Classification::Decided(reversal) = reversal
        .offset_left_with_line_joins(s(1), &CurveContext::STRICT)
        .unwrap()
    else {
        panic!("parallel reversal is decided");
    };
    assert_eq!(reversal.len(), 3);
    let Segment2::Arc(join) = &reversal.segments()[1] else {
        panic!("reversal join is circular");
    };
    assert_eq!((join.start(), join.end()), (&p(2, 1), &p(2, -1)));
}

#[test]
fn native_curve_string_zero_parallel_is_identity() {
    let curve = CurveString2::try_new(vec![line(0, 0, 4, 0), line(4, 0, 4, 3)]).unwrap();
    assert_eq!(
        curve
            .offset_left_with_line_joins(s(0), &CurveContext::STRICT)
            .unwrap(),
        Classification::Decided(curve)
    );
}
