#![cfg(feature = "dispatch-trace")]
mod support;

use hypercurve::{BulgeVertex2, Contour2, LineSeg2, Point2, Real, StraightSkeletonStage2};

fn p(x: i32, y: i32) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

#[test]
fn public_curve_query_emits_correlated_exact_path_trace() {
    let horizontal = LineSeg2::try_new(p(-4, 0), p(4, 0)).unwrap();
    let vertical = LineSeg2::try_new(p(0, -4), p(0, 4)).unwrap();

    hyperreal::dispatch_trace::reset();
    let relation =
        hyperreal::dispatch_trace::with_recording(|| horizontal.intersect_line(&vertical)).unwrap();
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
    let evidence =
        hyperreal::dispatch_trace::with_recording(|| contour.straight_skeleton()).unwrap();
    assert_eq!(evidence.stage(), StraightSkeletonStage2::Complete);

    let summary = hyperreal::dispatch_trace::take_trace().correlation_summary();
    assert!(summary.dispatch_events > 0 || summary.rational_temporaries > 0);
    assert!(
        summary.predicate_events > 0
            || summary.sign_or_zero_query_events > 0
            || summary.exact_reducer_events > 0
    );
}

/// A Boolean of a many-carrier region with a small one prepares only the
/// pair operands of carrier pairs that survive the broad phase.
#[test]
fn boolean_prepares_only_carriers_of_surviving_pairs() {
    use hypercurve::{BooleanOp, CurvePoint2, CurveRegion2, RegionPointLocation};

    // Each semicircular cap of a capsule is two rational quadratic Bezier
    // carriers; its straight sides are chord carriers.
    let capsule = |x: i32, y: i32| {
        CurveRegion2::try_from_native_material_contours(vec![
            Contour2::from_bulge_vertices(&[
                BulgeVertex2::new(p(x, y - 1), Real::zero()),
                BulgeVertex2::new(p(x + 2, y - 1), Real::one()),
                BulgeVertex2::new(p(x + 2, y + 1), Real::zero()),
                BulgeVertex2::new(p(x, y + 1), Real::one()),
            ])
            .unwrap(),
        ])
        .unwrap()
    };
    let mut many = CurveRegion2::empty();
    for k in 0..16 {
        many = many
            .boolean_region(&capsule(6 * k, 0), BooleanOp::Union)
            .unwrap();
    }
    // Its caps meet only the caps of the second capsule of `many`.
    let small = capsule(5, 1);
    // Both operands are regularized, so only cross-operand pairs are built.
    hyperreal::dispatch_trace::reset();
    let union =
        hyperreal::dispatch_trace::with_recording(|| many.boolean_region(&small, BooleanOp::Union))
            .unwrap();
    let prepared = hyperreal::dispatch_trace::take_trace().path_count(
        "hypercurve",
        "boolean-carrier-preparation",
        "prepared",
    );
    for (x, y, expected) in [
        (5, 1, RegionPointLocation::Inside),
        (1, 0, RegionPointLocation::Inside),
        (61, 0, RegionPointLocation::Inside),
        (4, -1, RegionPointLocation::Outside),
        (5, 5, RegionPointLocation::Outside),
    ] {
        assert_eq!(
            union.classify_point(&CurvePoint2::from(p(x, y))).unwrap(),
            expected
        );
    }
    // Only the four cap carriers of `small` and the four of the capsule it
    // meets are in surviving Bezier pairs; the other 60 Bezier carriers are
    // not.
    assert!(
        (1..=8).contains(&prepared),
        "prepared {prepared} carrier curves"
    );
}
