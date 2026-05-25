#![no_main]

use hyperlimit::PredicatePolicy;
use hyperpath::{
    LineArrangementError, LineArrangementEventClass, LinePathSegment, arrange_line_segments,
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
    if data.len() < 16 {
        return;
    }

    let first = LinePathSegment::new(
        p(signed(data[0]), signed(data[1])),
        p(signed(data[2]), signed(data[3])),
    );
    let second = LinePathSegment::new(
        p(signed(data[4]), signed(data[5])),
        p(signed(data[6]), signed(data[7])),
    );
    let third = LinePathSegment::new(
        p(signed(data[8]), signed(data[9])),
        p(signed(data[10]), signed(data[11])),
    );

    match arrange_line_segments(&[first, second, third], PredicatePolicy::default()) {
        Ok(report) => {
            assert_eq!(report.breakpoints.len(), 3);
            assert_eq!(report.events.len(), 3);
            for (index, points) in report.breakpoints.iter().enumerate() {
                assert!(points.len() >= 2);
                for point in points {
                    assert_eq!(point.segment, index);
                }
            }
            for fragment in &report.fragments {
                assert_ne!(fragment.segment.start(), fragment.segment.end());
                assert_eq!(fragment.source_segment, fragment.start.segment);
                assert_eq!(fragment.source_segment, fragment.end.segment);
            }
        }
        Err(LineArrangementError::DegenerateSegment { .. }) => {}
        Err(error) => panic!("unexpected exact arrangement error: {error:?}"),
    }

    let x = signed(data[12]);
    let y = signed(data[13]);
    let dx = i64::from(data[14] % 64) + 1;
    let dy = i64::from(data[15] % 64) + 1;
    let horizontal = LinePathSegment::new(p(x - dx, y), p(x + dx, y));
    let vertical = LinePathSegment::new(p(x, y - dy), p(x, y + dy));
    let report =
        arrange_line_segments(&[horizontal, vertical], PredicatePolicy::default()).unwrap();

    assert_eq!(
        report.events[0].class,
        LineArrangementEventClass::ProperCrossing
    );
    assert_eq!(report.events[0].point.as_ref().unwrap(), &p(x, y));
    assert_eq!(report.fragments.len(), 4);
});
