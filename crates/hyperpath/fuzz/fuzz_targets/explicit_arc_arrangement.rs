#![no_main]

use hyperlimit::PredicatePolicy;
use hyperpath::{
    ArcDirection, ExplicitArcArrangementClass, ExplicitCircularArc, arrange_explicit_arcs,
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
    if data.len() < 8 {
        return;
    }

    let scale = i64::from(data[0] % 32) + 1;
    let cx = signed(data[1]);
    let cy = signed(data[2]);
    let top_half = ExplicitCircularArc::new(
        p(cx, cy),
        r(5 * scale),
        p(cx + 5 * scale, cy),
        p(cx - 5 * scale, cy),
        ArcDirection::Ccw,
    )
    .unwrap();
    let top_left = ExplicitCircularArc::new(
        p(cx, cy),
        r(5 * scale),
        p(cx, cy + 5 * scale),
        p(cx - 5 * scale, cy),
        ArcDirection::Ccw,
    )
    .unwrap();
    let report = arrange_explicit_arcs(&[top_half, top_left], PredicatePolicy::default()).unwrap();

    assert_eq!(
        report.events[0].class,
        ExplicitArcArrangementClass::SameCircleFirstCoversSecond
    );
    assert_eq!(report.breakpoints.len(), 2);
    assert_eq!(report.fragments.len(), 3);

    let left_center_x = signed(data[3]) - 3 * scale;
    let right_center_x = signed(data[3]) + 3 * scale;
    let center_y = signed(data[4]);
    let left = ExplicitCircularArc::new(
        p(left_center_x, center_y),
        r(5 * scale),
        p(left_center_x, center_y - 5 * scale),
        p(left_center_x, center_y + 5 * scale),
        ArcDirection::Ccw,
    )
    .unwrap();
    let right = ExplicitCircularArc::new(
        p(right_center_x, center_y),
        r(5 * scale),
        p(right_center_x, center_y + 5 * scale),
        p(right_center_x, center_y - 5 * scale),
        ArcDirection::Ccw,
    )
    .unwrap();
    let report = arrange_explicit_arcs(&[left, right], PredicatePolicy::default()).unwrap();

    assert_eq!(
        report.events[0].class,
        ExplicitArcArrangementClass::DifferentCircleTwoPoints
    );
    assert_eq!(report.events[0].points.len(), 2);
    assert_eq!(report.breakpoints[0].len(), 4);
    assert_eq!(report.breakpoints[1].len(), 4);

    let full = ExplicitCircularArc::new(
        p(signed(data[5]), signed(data[6])),
        r(i64::from(data[7] % 32) + 1),
        p(
            signed(data[5]) + i64::from(data[7] % 32) + 1,
            signed(data[6]),
        ),
        p(
            signed(data[5]) + i64::from(data[7] % 32) + 1,
            signed(data[6]),
        ),
        ArcDirection::Ccw,
    )
    .unwrap();
    let report = arrange_explicit_arcs(&[full], PredicatePolicy::default()).unwrap();
    assert_eq!(report.breakpoints[0].len(), 1);
    assert_eq!(report.fragments.len(), 1);
    assert!(report.fragments[0].arc.facts().known_full_circle);
});
