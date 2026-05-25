#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    ClearanceStatus, LinePathSegment, NetId, PcbBoardOutline, PcbCircularPad, PcbObroundPad,
    PcbTrace, SweptLineSegment, TraceLayer, check_obround_pad_board_clearance,
    check_trace_obround_pad_clearance, check_trace_pad_clearance,
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
    if data.len() < 18 {
        return;
    }

    let pad = PcbObroundPad::new(
        NetId(2),
        TraceLayer(u16::from(data[0] % 8)),
        LinePathSegment::new(
            p(signed(data[1]), signed(data[2])),
            p(signed(data[3]), signed(data[4])),
        ),
        r(i64::from(data[5] % 64)),
    )
    .unwrap();
    assert!(
        PcbObroundPad::new(
            NetId(2),
            TraceLayer(0),
            LinePathSegment::new(p(0, 0), p(1, 0)),
            r(-1),
        )
        .is_err()
    );

    let trace_layer = if data[6] & 1 == 0 {
        pad.layer()
    } else {
        TraceLayer(u16::from((data[0] + 1) % 8))
    };
    let trace = PcbTrace::new(
        NetId(u32::from(data[7] % 4)),
        trace_layer,
        SweptLineSegment::new(
            LinePathSegment::new(
                p(signed(data[8]), signed(data[9])),
                p(signed(data[10]), signed(data[11])),
            ),
            r(i64::from(data[12] % 48)),
        )
        .unwrap(),
    );
    let clearance = r(i64::from(data[13] % 48));
    let trace_report =
        check_trace_obround_pad_clearance(&trace, &pad, &clearance, PredicatePolicy::default());
    assert_ne!(trace_report.status, ClearanceStatus::Unknown);

    let board = PcbBoardOutline::new(p(-180, -180), p(180, 180)).unwrap();
    let board_report =
        check_obround_pad_board_clearance(&pad, &board, &clearance, PredicatePolicy::default());
    assert_ne!(board_report.status, ClearanceStatus::Unknown);

    let center = p(signed(data[14]), signed(data[15]));
    let diameter = r(i64::from(data[16] % 64));
    let degenerate = PcbObroundPad::new(
        NetId(3),
        TraceLayer(0),
        LinePathSegment::new(center.clone(), center.clone()),
        diameter.clone(),
    )
    .unwrap();
    let circular = PcbCircularPad::new(NetId(3), TraceLayer(0), center, diameter).unwrap();
    let axis_trace = PcbTrace::new(
        NetId(u32::from(data[17] % 3)),
        TraceLayer(0),
        SweptLineSegment::new(LinePathSegment::new(p(-160, 0), p(160, 0)), r(2)).unwrap(),
    );
    assert_eq!(
        check_trace_obround_pad_clearance(
            &axis_trace,
            &degenerate,
            &clearance,
            PredicatePolicy::default(),
        )
        .status,
        check_trace_pad_clearance(
            &axis_trace,
            &circular,
            &clearance,
            PredicatePolicy::default()
        )
        .status,
    );
});
