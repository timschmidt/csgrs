#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    ClearanceStatus, LinePathSegment, NetId, PcbBoardOutline, PcbOrientedRectPad, PcbRectPad,
    PcbTrace, SweptLineSegment, TraceLayer, check_oriented_rect_pad_board_clearance,
    check_rect_pad_board_clearance, check_trace_oriented_rect_pad_clearance,
    check_trace_rect_pad_clearance,
};
use hyperreal::{Rational, Real};
use libfuzzer_sys::fuzz_target;

fn r(value: i64) -> Real {
    Real::new(Rational::new(value))
}

fn rq(numerator: i64, denominator: i64) -> Real {
    Real::new(Rational::new(numerator) / Rational::new(denominator))
}

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn signed(byte: u8) -> i64 {
    i64::from(i8::from_ne_bytes([byte]))
}

fn unit_axis(selector: u8) -> Point2 {
    match selector % 8 {
        0 => Point2::new(r(1), r(0)),
        1 => Point2::new(r(0), r(1)),
        2 => Point2::new(r(-1), r(0)),
        3 => Point2::new(r(0), r(-1)),
        4 => Point2::new(rq(3, 5), rq(4, 5)),
        5 => Point2::new(rq(-3, 5), rq(4, 5)),
        6 => Point2::new(rq(5, 13), rq(12, 13)),
        _ => Point2::new(rq(8, 17), rq(-15, 17)),
    }
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 18 {
        return;
    }

    let width = i64::from(data[0] % 48);
    let height = i64::from(data[1] % 48);
    let center = p(signed(data[2]), signed(data[3]));
    let local_x = unit_axis(data[4]);
    let pad = PcbOrientedRectPad::new(
        NetId(2),
        TraceLayer(u16::from(data[5] % 8)),
        center.clone(),
        r(width),
        r(height),
        local_x.clone(),
        PredicatePolicy::default(),
    )
    .unwrap();
    assert_eq!(pad.facts().local_x_length_squared, r(1));

    let trace_layer = if data[6] & 1 == 0 {
        pad.layer()
    } else {
        TraceLayer(u16::from((data[5] + 1) % 8))
    };
    let trace = PcbTrace::new(
        NetId(u32::from(data[7] % 4)),
        trace_layer,
        SweptLineSegment::new(
            LinePathSegment::new(
                p(signed(data[8]), signed(data[9])),
                p(signed(data[10]), signed(data[11])),
            ),
            r(i64::from(data[12] % 24)),
        )
        .unwrap(),
    );
    let clearance = r(i64::from(data[13] % 24));
    let trace_report = check_trace_oriented_rect_pad_clearance(
        &trace,
        &pad,
        &clearance,
        PredicatePolicy::default(),
    );
    assert_ne!(trace_report.status, ClearanceStatus::Unknown);

    let board = PcbBoardOutline::new(p(-200, -200), p(200, 200)).unwrap();
    let board_report = check_oriented_rect_pad_board_clearance(
        &pad,
        &board,
        &clearance,
        PredicatePolicy::default(),
    );
    assert_ne!(board_report.status, ClearanceStatus::Unknown);

    let axis_pad = PcbOrientedRectPad::new(
        NetId(3),
        TraceLayer(0),
        center.clone(),
        r(width),
        r(height),
        Point2::new(r(1), r(0)),
        PredicatePolicy::default(),
    )
    .unwrap();
    let rect = PcbRectPad::new(NetId(3), TraceLayer(0), center, r(width), r(height)).unwrap();
    let axis_trace = PcbTrace::new(
        NetId(u32::from(data[14] % 3)),
        TraceLayer(0),
        SweptLineSegment::new(
            LinePathSegment::new(
                p(signed(data[15]), signed(data[16])),
                p(signed(data[17]), signed(data[16])),
            ),
            r(2),
        )
        .unwrap(),
    );
    assert_eq!(
        check_trace_oriented_rect_pad_clearance(
            &axis_trace,
            &axis_pad,
            &clearance,
            PredicatePolicy::default(),
        )
        .status,
        check_trace_rect_pad_clearance(&axis_trace, &rect, &clearance, PredicatePolicy::default())
            .status,
    );
    assert_eq!(
        check_oriented_rect_pad_board_clearance(
            &axis_pad,
            &board,
            &clearance,
            PredicatePolicy::default(),
        )
        .status,
        check_rect_pad_board_clearance(&rect, &board, &clearance, PredicatePolicy::default())
            .status,
    );
});
