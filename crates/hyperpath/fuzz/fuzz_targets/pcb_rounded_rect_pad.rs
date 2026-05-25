#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    ClearanceStatus, LinePathSegment, NetId, PcbBoardOutline, PcbCircularPad, PcbRectPad,
    PcbRoundedRectPad, PcbTrace, SweptLineSegment, TraceLayer, check_rect_pad_board_clearance,
    check_rounded_rect_pad_board_clearance, check_trace_pad_clearance,
    check_trace_rect_pad_clearance, check_trace_rounded_rect_pad_clearance,
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

    let width = i64::from(data[0] % 64);
    let height = i64::from(data[1] % 64);
    let max_radius = width.min(height) / 2;
    let radius = if max_radius == 0 {
        0
    } else {
        i64::from(data[2]) % (max_radius + 1)
    };
    let pad_center = p(signed(data[3]), signed(data[4]));
    let pad = PcbRoundedRectPad::new(
        NetId(2),
        TraceLayer(u16::from(data[5] % 8)),
        pad_center.clone(),
        r(width),
        r(height),
        r(radius),
    )
    .unwrap();

    let trace_layer = if data[6] & 1 == 0 {
        pad.layer()
    } else {
        TraceLayer(u16::from((data[5] + 1) % 8))
    };
    let trace_start = p(signed(data[8]), signed(data[9]));
    let trace_end = if data[6] & 2 == 0 {
        p(signed(data[10]), signed(data[9]))
    } else {
        p(signed(data[8]), signed(data[11]))
    };
    let trace = PcbTrace::new(
        NetId(u32::from(data[7] % 4)),
        trace_layer,
        SweptLineSegment::new(
            LinePathSegment::new(trace_start, trace_end),
            r(i64::from(data[12] % 32)),
        )
        .unwrap(),
    );
    let clearance = r(i64::from(data[13] % 32));
    let rounded_report = check_trace_rounded_rect_pad_clearance(
        &trace,
        &pad,
        &clearance,
        PredicatePolicy::default(),
    );
    assert_ne!(rounded_report.status, ClearanceStatus::Unknown);

    let board = PcbBoardOutline::new(p(-160, -160), p(160, 160)).unwrap();
    let board_report = check_rounded_rect_pad_board_clearance(
        &pad,
        &board,
        &clearance,
        PredicatePolicy::default(),
    );
    assert_ne!(board_report.status, ClearanceStatus::Unknown);

    let zero_radius = PcbRoundedRectPad::new(
        NetId(2),
        pad.layer(),
        pad_center.clone(),
        r(width),
        r(height),
        r(0),
    )
    .unwrap();
    let rect = PcbRectPad::new(
        NetId(2),
        pad.layer(),
        pad_center.clone(),
        r(width),
        r(height),
    )
    .unwrap();
    let rounded_zero_report = check_trace_rounded_rect_pad_clearance(
        &trace,
        &zero_radius,
        &clearance,
        PredicatePolicy::default(),
    );
    let rect_report =
        check_trace_rect_pad_clearance(&trace, &rect, &clearance, PredicatePolicy::default());
    assert_eq!(rounded_zero_report.status, rect_report.status);
    assert_eq!(
        check_rounded_rect_pad_board_clearance(
            &zero_radius,
            &board,
            &clearance,
            PredicatePolicy::default()
        )
        .status,
        check_rect_pad_board_clearance(&rect, &board, &clearance, PredicatePolicy::default())
            .status,
    );

    let diameter = i64::from(data[14] % 64);
    let circle_center = p(signed(data[15]), signed(data[16]));
    let circular =
        PcbCircularPad::new(NetId(3), TraceLayer(0), circle_center.clone(), r(diameter)).unwrap();
    let rounded_circle = PcbRoundedRectPad::new(
        NetId(3),
        TraceLayer(0),
        circle_center,
        r(diameter),
        r(diameter),
        r(diameter / 2),
    )
    .unwrap();
    let circle_trace = PcbTrace::new(
        NetId(u32::from(data[17] % 3)),
        TraceLayer(0),
        SweptLineSegment::new(LinePathSegment::new(p(-200, 0), p(200, 0)), r(2)).unwrap(),
    );
    assert_eq!(
        check_trace_rounded_rect_pad_clearance(
            &circle_trace,
            &rounded_circle,
            &clearance,
            PredicatePolicy::default(),
        )
        .status,
        check_trace_pad_clearance(
            &circle_trace,
            &circular,
            &clearance,
            PredicatePolicy::default()
        )
        .status,
    );
});
