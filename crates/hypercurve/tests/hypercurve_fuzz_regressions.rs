use hypercurve::{
    CurvePolicy, LineArcIntersection, LineSeg2, Point2, Real, Segment2, SegmentIntersection,
};

fn s(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(s(x), s(y))
}

fn policy() -> CurvePolicy {
    CurvePolicy::certified()
}

#[test]
fn fuzz_line_arc_candidate_outside_finite_line_is_rejected() {
    let arc = Segment2::from_bulge(p(-29, 16), p(13, 16), s(1)).unwrap();
    let line = Segment2::Line(LineSeg2::try_new(p(9, 41), p(-15, 17)).unwrap());

    let intersection = arc.intersect_segment(&line, &policy()).unwrap();
    match &intersection {
        SegmentIntersection::LineArc {
            result: LineArcIntersection::None,
            ..
        } => {}
        SegmentIntersection::LineArc {
            result: LineArcIntersection::Point(hit),
            ..
        } => panic!(
            "outside hit retained at line_param={:?}, arc_param={:?}, point={:?}",
            hit.line_param.to_f64_lossy(),
            hit.arc_param.to_f64_lossy(),
            [hit.point.x().to_f64_lossy(), hit.point.y().to_f64_lossy()]
        ),
        _ => panic!("unexpected line/arc relation"),
    }
}
