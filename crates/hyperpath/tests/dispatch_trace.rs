#![cfg(feature = "dispatch-trace")]

use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    ClearanceStatus, LinePathSegment, NetId, PcbTrace, SweptLineSegment, TraceLayer,
    check_trace_clearance,
};
use hyperreal::Real;

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn trace(net: u32, y: i32) -> PcbTrace {
    PcbTrace::new(
        NetId(net),
        TraceLayer(0),
        SweptLineSegment::new(
            LinePathSegment::new(point(0, y), point(10, y)),
            Real::from(2),
        )
        .unwrap(),
    )
}

#[test]
fn exact_clearance_fast_path_does_not_request_approximation() {
    hyperreal::dispatch_trace::reset();
    let _recording = hyperreal::dispatch_trace::recording_scope();

    let report = check_trace_clearance(&trace(1, 0), &trace(2, 5), &Real::from(3), PredicatePolicy);
    assert_eq!(report.status, ClearanceStatus::CertifiedClear);

    let correlation = hyperreal::dispatch_trace::snapshot_trace().correlation_summary();
    assert!(correlation.dispatch_events > 0);
    assert!(correlation.sign_or_zero_query_events > 0);
    assert_eq!(correlation.approximation_events, 0);
    assert_eq!(correlation.unknown_fact_events, 0);
}
