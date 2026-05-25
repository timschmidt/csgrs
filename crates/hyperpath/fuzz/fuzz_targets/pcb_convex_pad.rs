#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    BoardContourError, ClearanceStatus, LinePathSegment, NetId, PcbBoardOutline, PcbConvexPad,
    PcbRectPad, PcbTrace, SweptLineSegment, TraceLayer, check_convex_pad_board_clearance,
    check_trace_convex_pad_clearance, check_trace_rect_pad_clearance,
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

    let center_x = signed(data[0]);
    let center_y = signed(data[1]);
    let radius = i64::from(data[2] % 48) + 1;
    let diamond = PcbConvexPad::new(
        NetId(2),
        TraceLayer(u16::from(data[3] % 8)),
        vec![
            p(center_x, center_y + radius),
            p(center_x + radius, center_y),
            p(center_x, center_y - radius),
            p(center_x - radius, center_y),
        ],
    )
    .unwrap();
    assert_eq!(
        PcbConvexPad::new(NetId(2), TraceLayer(0), vec![p(0, 0), p(1, 0)]).unwrap_err(),
        BoardContourError::TooFewVertices
    );

    let trace_layer = if data[4] & 1 == 0 {
        diamond.layer()
    } else {
        TraceLayer(u16::from((data[3] + 1) % 8))
    };
    let trace = PcbTrace::new(
        NetId(u32::from(data[5] % 4)),
        trace_layer,
        SweptLineSegment::new(
            LinePathSegment::new(
                p(signed(data[6]), signed(data[7])),
                p(signed(data[8]), signed(data[9])),
            ),
            r(i64::from(data[10] % 48)),
        )
        .unwrap(),
    );
    let clearance = r(i64::from(data[11] % 48));
    let trace_report =
        check_trace_convex_pad_clearance(&trace, &diamond, &clearance, PredicatePolicy::default());
    assert_ne!(trace_report.status, ClearanceStatus::Unknown);

    let board = PcbBoardOutline::new(p(-180, -180), p(180, 180)).unwrap();
    let board_report =
        check_convex_pad_board_clearance(&diamond, &board, &clearance, PredicatePolicy::default());
    assert_ne!(board_report.status, ClearanceStatus::Unknown);

    let half_w = i64::from(data[12] % 32) + 1;
    let half_h = i64::from(data[13] % 32) + 1;
    let rect_center = p(signed(data[14]), signed(data[15]));
    let convex_rect = PcbConvexPad::new(
        NetId(3),
        TraceLayer(0),
        vec![
            p(signed(data[14]) - half_w, signed(data[15]) - half_h),
            p(signed(data[14]) + half_w, signed(data[15]) - half_h),
            p(signed(data[14]) + half_w, signed(data[15]) + half_h),
            p(signed(data[14]) - half_w, signed(data[15]) + half_h),
        ],
    )
    .unwrap();
    let rect = PcbRectPad::new(
        NetId(3),
        TraceLayer(0),
        rect_center,
        r(half_w * 2),
        r(half_h * 2),
    )
    .unwrap();
    let axis_trace = PcbTrace::new(
        NetId(u32::from(data[16] % 3)),
        TraceLayer(0),
        SweptLineSegment::new(
            LinePathSegment::new(p(-160, signed(data[17])), p(160, signed(data[17]))),
            r(2),
        )
        .unwrap(),
    );
    assert_eq!(
        check_trace_convex_pad_clearance(
            &axis_trace,
            &convex_rect,
            &clearance,
            PredicatePolicy::default(),
        )
        .status,
        check_trace_rect_pad_clearance(&axis_trace, &rect, &clearance, PredicatePolicy::default())
            .status,
    );
});
