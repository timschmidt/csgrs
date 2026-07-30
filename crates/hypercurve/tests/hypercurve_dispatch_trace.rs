#![cfg(feature = "dispatch-trace")]

use hypercurve::{
    BulgeVertex2, Contour2, CurvePolicy, LineSeg2, Point2, Real, StraightSkeletonStage2,
};

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

#[test]
fn public_curve_query_emits_correlated_exact_path_trace() {
    let horizontal = LineSeg2::try_new(p(-4, 0), p(4, 0)).unwrap();
    let vertical = LineSeg2::try_new(p(0, -4), p(0, 4)).unwrap();

    hyperreal::dispatch_trace::reset();
    let relation = hyperreal::dispatch_trace::with_recording(|| {
        horizontal.intersect_line(&vertical, &CurvePolicy::STRICT)
    })
    .unwrap();
    assert!(!matches!(relation, hypercurve::LineLineIntersection::None));

    let snapshot = hyperreal::dispatch_trace::take_trace();
    let summary = snapshot.correlation_summary();
    assert!(summary.dispatch_events > 0);
    assert!(
        summary.predicate_events > 0
            || summary.sign_or_zero_query_events > 0
            || summary.exact_reducer_events > 0
    );
}

#[test]
fn public_finite_ring_import_emits_correlated_exact_path_trace() {
    hyperreal::dispatch_trace::reset();
    let import = hyperreal::dispatch_trace::with_recording(|| {
        Contour2::from_finite_ring(&[[0.0, 0.0], [4.0, 0.0], [4.0, 3.0], [0.0, 3.0]])
    })
    .unwrap();
    assert_eq!(import.len(), 4);

    let snapshot = hyperreal::dispatch_trace::take_trace();
    let summary = snapshot.correlation_summary();
    assert!(summary.dispatch_events > 0 || summary.rational_temporaries > 0);
}

#[test]
fn public_straight_skeleton_emits_correlated_exact_path_trace() {
    let contour = Contour2::from_bulge_vertices(&[
        BulgeVertex2::new(p(0, 0), Real::zero()),
        BulgeVertex2::new(p(30, 0), Real::zero()),
        BulgeVertex2::new(p(30, 24), Real::zero()),
        BulgeVertex2::new(p(20, 24), Real::zero()),
        BulgeVertex2::new(p(20, 7), Real::zero()),
        BulgeVertex2::new(p(17, 11), Real::zero()),
        BulgeVertex2::new(p(17, 24), Real::zero()),
        BulgeVertex2::new(p(0, 24), Real::zero()),
    ])
    .unwrap();

    hyperreal::dispatch_trace::reset();
    let evidence = hyperreal::dispatch_trace::with_recording(|| {
        contour.straight_skeleton(&CurvePolicy::STRICT)
    })
    .unwrap();
    assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);

    let summary = hyperreal::dispatch_trace::take_trace().correlation_summary();
    assert!(summary.dispatch_events > 0 || summary.rational_temporaries > 0);
    assert!(
        summary.predicate_events > 0
            || summary.sign_or_zero_query_events > 0
            || summary.exact_reducer_events > 0
    );
}
