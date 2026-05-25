#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    ClearanceStatus, LinePathSegment, NetId, PcbCircularBoardOutline, PcbCircularPad, PcbTrace,
    SweptLineSegment, TraceLayer, check_circular_pad_circular_board_clearance,
    check_trace_circular_board_clearance,
};
use hyperreal::{Rational, Real};
use libfuzzer_sys::fuzz_target;

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn signed(byte: u8) -> i64 {
    i64::from(i8::from_ne_bytes([byte]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 15 {
        return;
    }

    let radius = i64::from(data[0] % 96);
    let board =
        PcbCircularBoardOutline::new(p(signed(data[1]), signed(data[2])), r(radius)).unwrap();
    assert_eq!(
        PcbCircularBoardOutline::new(p(0, 0), r(-1)).unwrap_err(),
        "circular board radius must be nonnegative"
    );

    let trace = PcbTrace::new(
        NetId(1),
        TraceLayer(0),
        SweptLineSegment::new(
            LinePathSegment::new(
                p(signed(data[3]), signed(data[4])),
                p(signed(data[5]), signed(data[6])),
            ),
            r(i64::from(data[7] % 64)),
        )
        .unwrap(),
    );
    let clearance = r(i64::from(data[8] % 64));
    let trace_report = check_trace_circular_board_clearance(
        &trace,
        &board,
        &clearance,
        PredicatePolicy::default(),
    );
    assert_ne!(trace_report.status, ClearanceStatus::Unknown);

    let pad = PcbCircularPad::new(
        NetId(2),
        TraceLayer(0),
        p(signed(data[9]), signed(data[10])),
        r(i64::from(data[11] % 64)),
    )
    .unwrap();
    let pad_report = check_circular_pad_circular_board_clearance(
        &pad,
        &board,
        &clearance,
        PredicatePolicy::default(),
    );
    assert_ne!(pad_report.status, ClearanceStatus::Unknown);

    let roomy_radius = i64::from(data[12] % 64) + 64;
    let roomy = PcbCircularBoardOutline::new(p(0, 0), r(roomy_radius)).unwrap();
    let centered_trace = PcbTrace::new(
        NetId(3),
        TraceLayer(0),
        SweptLineSegment::new(
            LinePathSegment::new(
                p(-i64::from(data[13] % 16), 0),
                p(i64::from(data[13] % 16), 0),
            ),
            r(i64::from(data[14] % 16)),
        )
        .unwrap(),
    );
    assert_eq!(
        check_trace_circular_board_clearance(
            &centered_trace,
            &roomy,
            &r(0),
            PredicatePolicy::default(),
        )
        .status,
        ClearanceStatus::CertifiedClear,
    );
});
