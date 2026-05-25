#![no_main]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    BoardContourError, ClearanceStatus, LinePathSegment, NetId, PcbBoardOutline, PcbOrthogonalPad,
    PcbRectPad, PcbTrace, SweptLineSegment, TraceLayer, check_orthogonal_pad_board_clearance,
    check_trace_orthogonal_pad_clearance, check_trace_rect_pad_clearance,
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

    let layer = TraceLayer(u16::from(data[0] % 8));
    let x = signed(data[1]);
    let y = signed(data[2]);
    let width = i64::from(data[3] % 32) + 4;
    let height = i64::from(data[4] % 32) + 4;
    let notch_w = i64::from(data[5] % u8::try_from(width - 1).unwrap()) + 1;
    let notch_h = i64::from(data[6] % u8::try_from(height - 1).unwrap()) + 1;
    let pad = PcbOrthogonalPad::new(
        NetId(2),
        layer,
        vec![
            p(x, y),
            p(x + width, y),
            p(x + width, y + notch_h),
            p(x + notch_w, y + notch_h),
            p(x + notch_w, y + height),
            p(x, y + height),
        ],
    )
    .unwrap();
    assert!(pad.facts().exact.all_exact_rational);
    assert_eq!(
        PcbOrthogonalPad::new(
            NetId(2),
            TraceLayer(0),
            vec![p(0, 0), p(4, 2), p(4, 4), p(0, 4)]
        )
        .unwrap_err(),
        BoardContourError::NonOrthogonal
    );

    let trace = PcbTrace::new(
        NetId(u32::from(data[7] % 4)),
        if data[8] & 1 == 0 {
            layer
        } else {
            TraceLayer(9)
        },
        SweptLineSegment::new(
            LinePathSegment::new(
                p(signed(data[9]), signed(data[10])),
                p(signed(data[11]), signed(data[12])),
            ),
            r(i64::from(data[13] % 16)),
        )
        .unwrap(),
    );
    let clearance = r(i64::from(data[14] % 16));
    let report =
        check_trace_orthogonal_pad_clearance(&trace, &pad, &clearance, PredicatePolicy::default());
    assert_ne!(report.status, ClearanceStatus::Unknown);

    let board = PcbBoardOutline::new(p(-200, -200), p(200, 200)).unwrap();
    let board_report =
        check_orthogonal_pad_board_clearance(&pad, &board, &clearance, PredicatePolicy::default());
    assert_ne!(board_report.status, ClearanceStatus::Unknown);

    let rect_pad = PcbOrthogonalPad::new(
        NetId(3),
        TraceLayer(0),
        vec![p(0, 0), p(width, 0), p(width, height), p(0, height)],
    )
    .unwrap();
    let rect = PcbRectPad::new(
        NetId(3),
        TraceLayer(0),
        p(width / 2, height / 2),
        r(width),
        r(height),
    )
    .unwrap();
    let axis_trace = PcbTrace::new(
        NetId(u32::from(data[15] % 3)),
        TraceLayer(0),
        SweptLineSegment::new(
            LinePathSegment::new(p(-64, signed(data[10])), p(64, signed(data[10]))),
            r(2),
        )
        .unwrap(),
    );
    assert_eq!(
        check_trace_orthogonal_pad_clearance(
            &axis_trace,
            &rect_pad,
            &clearance,
            PredicatePolicy::default(),
        )
        .status,
        check_trace_rect_pad_clearance(&axis_trace, &rect, &clearance, PredicatePolicy::default())
            .status,
    );
});
