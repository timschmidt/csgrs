use hypercurve::{
    CircularArc2, Classification, CurveContext, LineSeg2, Point2, Real, Segment2, UncertaintyReason,
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
