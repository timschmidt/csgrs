#![no_main]

use hypercurve::{
    Classification, Curve2, CurveContext, CurvePoint2, Point2, QuadraticBezier2, Real,
};
use libfuzzer_sys::fuzz_target;

fn real_from_byte(byte: u8) -> Real {
    Real::from(byte as i32 - 128)
}

fn point(x: u8, y: u8) -> Point2 {
    Point2::new(real_from_byte(x), real_from_byte(y))
}

fn coincide(first: &CurvePoint2, second: &CurvePoint2, policy: &CurveContext) {
    assert_eq!(
        first.coincides_with(second, policy).value,
        Classification::Decided(true)
    );
}

/// Exact pieces of a completed pair topology must reassemble their source.
fn assert_reassembles(source: &Curve2, pieces: &[Curve2], policy: &CurveContext) {
    let (Some(first), Some(last)) = (pieces.first(), pieces.last()) else {
        panic!("a completed topology keeps at least one piece per curve");
    };
    coincide(&first.start(), &source.start(), policy);
    for pair in pieces.windows(2) {
        coincide(&pair[0].end(), &pair[1].start(), policy);
    }
    coincide(&last.end(), &source.end(), policy);
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 12 {
        return;
    }

    let policy = CurveContext::STRICT;
    let curve = |chunk: &[u8]| {
        Curve2::from(QuadraticBezier2::new(
            point(chunk[0], chunk[1]),
            point(chunk[2], chunk[3]),
            point(chunk[4], chunk[5]),
        ))
    };
    let first = curve(&data[0..6]);
    let second = curve(&data[6..12]);

    if let Ok(outcome) = first.intersection_topology(&second, &policy) {
        let topology = outcome.into_value();
        if topology.result().is_complete() {
            assert_reassembles(&first, topology.first(), &policy);
            assert_reassembles(&second, topology.second(), &policy);
        }
    }
});
