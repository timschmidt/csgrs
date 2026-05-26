#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    ClearanceStatus, LinePathSegment, NetId, PcbCircularPad, PcbObroundBoardOutline, PcbTrace,
    SweptLineSegment, TraceLayer, check_circular_pad_obround_board_clearance,
    check_trace_obround_board_clearance,
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
    if data.len() < 16 {
        return;
    }

    let board = PcbObroundBoardOutline::new(
        LinePathSegment::new(
            p(signed(data[0]), signed(data[1])),
            p(signed(data[2]), signed(data[3])),
        ),
        r(i64::from(data[4] % 96)),
    )
    .unwrap();
    assert_eq!(
        PcbObroundBoardOutline::new(LinePathSegment::new(p(0, 0), p(1, 0)), r(-1)).unwrap_err(),
        "obround board diameter must be nonnegative"
    );

    let trace = PcbTrace::new(
        NetId(1),
        TraceLayer(0),
        SweptLineSegment::new(
            LinePathSegment::new(
                p(signed(data[5]), signed(data[6])),
                p(signed(data[7]), signed(data[8])),
            ),
            r(i64::from(data[9] % 64)),
        )
        .unwrap(),
    );
    let clearance = r(i64::from(data[10] % 64));
    let trace_report =
        check_trace_obround_board_clearance(&trace, &board, &clearance, PredicatePolicy::default());
    assert_ne!(trace_report.status, ClearanceStatus::Unknown);

    let pad = PcbCircularPad::new(
        NetId(2),
        TraceLayer(0),
        p(signed(data[11]), signed(data[12])),
        r(i64::from(data[13] % 64)),
    )
    .unwrap();
    let pad_report = check_circular_pad_obround_board_clearance(
        &pad,
        &board,
        &clearance,
        PredicatePolicy::default(),
    );
    assert_ne!(pad_report.status, ClearanceStatus::Unknown);

    let roomy = PcbObroundBoardOutline::new(LinePathSegment::new(p(0, 0), p(100, 0)), r(64))
        .unwrap();
    let centered_trace = PcbTrace::new(
        NetId(3),
        TraceLayer(0),
        SweptLineSegment::new(
            LinePathSegment::new(
                p(i64::from(data[14] % 50), 0),
                p(50 + i64::from(data[15] % 50), 0),
            ),
            r(8),
        )
        .unwrap(),
    );
    assert_eq!(
        check_trace_obround_board_clearance(
            &centered_trace,
            &roomy,
            &r(0),
            PredicatePolicy::default(),
        )
        .status,
        ClearanceStatus::CertifiedClear,
    );
});
