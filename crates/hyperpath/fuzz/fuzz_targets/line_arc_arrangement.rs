#![no_main]

use hyperlimit::PredicatePolicy;
use hyperpath::{
    ArcDirection, ExplicitCircularArc, LineArcArrangementEventClass, LineArrangementError,
    LinePathSegment, arrange_line_segments_with_explicit_arcs,
};
use hyperreal::{Rational, Real};
use libfuzzer_sys::fuzz_target;

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn p(x: i64, y: i64) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(r(x), r(y))
}

fn signed(byte: u8) -> i64 {
    i64::from(i8::from_ne_bytes([byte]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 12 {
        return;
    }

    let cx = signed(data[0]);
    let cy = signed(data[1]);
    let radius = i64::from(data[2] % 64) + 1;
    let arc = ExplicitCircularArc::new(
        p(cx, cy),
        r(radius),
        p(cx + radius, cy),
        p(cx + radius, cy),
        ArcDirection::Ccw,
    )
    .unwrap();

    let arbitrary = LinePathSegment::new(
        p(signed(data[3]), signed(data[4])),
        p(signed(data[5]), signed(data[6])),
    );
    match arrange_line_segments_with_explicit_arcs(
        &[arbitrary],
        &[arc.clone()],
        PredicatePolicy::default(),
    ) {
        Ok(report) => {
            assert_eq!(report.events.len(), 1);
            assert_eq!(report.line_breakpoints.len(), 1);
            assert!(!report.line_breakpoints[0].is_empty());
            for point in &report.events[0].points {
                assert!(
                    report.line_breakpoints[0]
                        .iter()
                        .any(|breakpoint| &breakpoint.point == point)
                );
            }
        }
        Err(LineArrangementError::DegenerateSegment { .. }) => {}
        Err(error) => panic!("unexpected exact line/arc arrangement error: {error:?}"),
    }

    let pad = i64::from(data[7] % 64) + 1;
    let horizontal = LinePathSegment::new(p(cx - radius - pad, cy), p(cx + radius + pad, cy));
    let vertical = LinePathSegment::new(p(cx, cy - radius - pad), p(cx, cy + radius + pad));
    let report = arrange_line_segments_with_explicit_arcs(
        &[horizontal, vertical],
        &[arc],
        PredicatePolicy::default(),
    )
    .unwrap();

    assert_eq!(report.events.len(), 2);
    assert_eq!(report.events[0].class, LineArcArrangementEventClass::Secant);
    assert_eq!(report.events[1].class, LineArcArrangementEventClass::Secant);
    assert_eq!(report.line_breakpoints[0].len(), 4);
    assert_eq!(report.line_breakpoints[1].len(), 4);
    assert_eq!(report.line_fragments.len(), 6);
});
