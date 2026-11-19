//! A terminal approximation cannot certify continuity in a retained path.
use hyperlimit::{Certainty, Point2, PredicateOutcome, PredicatePolicy, compare_reals};
use hyperpath::{
    FeedPathElement, LinePathSegment, LookaheadFeedPlanningLimits, RouteCertificationError,
    TangentAlignment, TangentJoinClass, TangentSpan, certify_corner_lookahead_limits,
    certify_lookahead_feed_schedule, classify_tangent_alignment, classify_tangent_chain,
    classify_tangent_join, plan_lookahead_feed_schedule,
};
use hyperreal::Real;
use std::cmp::Ordering;

fn point(x: i32, y: i32) -> Point2 {
    Point2::new(x.into(), y.into())
}

fn hidden_positive() -> Real {
    // The sine addition formula cancels the first three terms exactly.
    (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin()
        + Real::from(2).powi_i64(-2048).unwrap()
}

#[test]
fn endpoint_and_tangent_proofs_refuse_warm_approximate_zeroes() {
    let h = hidden_positive();
    let approximate = PredicatePolicy::APPROXIMATE_512;
    // Warm the policy-enabled comparison. A stronger future proof may resolve h.
    if let PredicateOutcome::Decided {
        value, certainty, ..
    } = compare_reals(&h, &Real::zero(), approximate)
        && certainty != Certainty::Approximate
    {
        assert_eq!(value, Ordering::Greater);
    }
    let origin = point(0, 0);
    let east = point(1, 0);
    let shifted = Point2::new(h.clone(), Real::zero());
    let tilted = Point2::new(Real::one(), h.clone());
    for policy in [approximate, PredicatePolicy::STRICT] {
        let gap = classify_tangent_join(&origin, &east, &shifted, &east, policy);
        match gap.class {
            TangentJoinClass::Unknown => assert_eq!(gap.endpoints_equal, None),
            TangentJoinClass::EndpointMismatch => assert_eq!(gap.endpoints_equal, Some(false)),
            other => panic!("nonzero endpoint gap classified as {other:?}"),
        }
        assert_eq!(gap.alignment, None);

        for (tangent, exact_alignment, exact_class) in [
            (
                &tilted,
                TangentAlignment::NotParallel,
                TangentJoinClass::Corner,
            ),
            (
                &shifted,
                TangentAlignment::SameDirection,
                TangentJoinClass::G1Continuous,
            ),
        ] {
            let alignment = classify_tangent_alignment(&east, tangent, policy);
            assert!(alignment == TangentAlignment::Unknown || alignment == exact_alignment);
            let join = classify_tangent_join(&origin, &east, &origin, tangent, policy);
            assert_eq!(join.endpoints_equal, Some(true));
            assert_eq!(join.alignment, Some(alignment));
            assert_eq!(
                join.class,
                if alignment == TangentAlignment::Unknown {
                    TangentJoinClass::Unknown
                } else {
                    exact_class
                }
            );
        }

        // A certified second coordinate or second zero vector can decide the result.
        let separated = Point2::new(h.clone(), Real::one());
        assert_eq!(
            classify_tangent_join(&origin, &east, &separated, &east, policy).class,
            TangentJoinClass::EndpointMismatch
        );
        assert_eq!(
            classify_tangent_alignment(&shifted, &origin, policy),
            TangentAlignment::Degenerate
        );
    }
}

#[test]
fn corner_planning_and_replay_preserve_unresolved_join_evidence() {
    let strict = PredicatePolicy::STRICT;
    let lines = [
        LinePathSegment::new(point(-2, 0), point(0, 0), strict).unwrap(),
        LinePathSegment::new(point(0, 0), point(2, 0), strict).unwrap(),
    ];
    let mut spans = lines.each_ref().map(TangentSpan::from_line_segment);
    let route = lines.map(FeedPathElement::Line);
    let limits = LookaheadFeedPlanningLimits {
        maximum_entry_feed: Real::zero(),
        maximum_corner_feeds: vec![Real::one()],
        corner_radii: vec![Real::one()],
        maximum_exit_feed: Real::zero(),
    };
    let schedule =
        plan_lookahead_feed_schedule(&route, &spans, &limits, Real::one(), Real::one(), strict)
            .unwrap()
            .schedule;
    assert_eq!(schedule.corner_feeds, [Real::one()]);
    spans[1].start.x = hidden_positive();
    for policy in [strict, PredicatePolicy::APPROXIMATE_512] {
        let chain = classify_tangent_chain(&spans, policy);
        assert_eq!(chain.first_non_g1_join(), Some(0));
        let expected_error = match chain.joins[0].class {
            TangentJoinClass::Unknown => RouteCertificationError::PredicateUnresolved,
            TangentJoinClass::EndpointMismatch => RouteCertificationError::UnsupportedRouteGeometry,
            other => panic!("nonzero endpoint gap classified as {other:?}"),
        };
        assert_eq!(
            certify_corner_lookahead_limits(
                &spans,
                Real::one(),
                Real::one(),
                Real::one(),
                Real::one(),
                policy
            )
            .unwrap_err(),
            expected_error
        );
        assert_eq!(
            plan_lookahead_feed_schedule(&route, &spans, &limits, Real::one(), Real::one(), policy)
                .unwrap_err(),
            expected_error
        );
        assert_eq!(
            certify_lookahead_feed_schedule(
                &route,
                &spans,
                &schedule,
                Real::one(),
                Real::one(),
                policy
            )
            .unwrap_err(),
            expected_error
        );
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn certified_rational_joins_preserve_predicate_counts() {
    use hyperreal::dispatch_trace;
    let origin = point(0, 0);
    let east = point(1, 0);
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (endpoint, tangent, class, comparisons) in [
            (point(0, 0), point(2, 0), TangentJoinClass::G1Continuous, 6),
            (
                point(0, 0),
                point(-1, 0),
                TangentJoinClass::ReversedTangent,
                6,
            ),
            (point(0, 0), point(0, 1), TangentJoinClass::Corner, 5),
            (
                point(0, 0),
                point(0, 0),
                TangentJoinClass::DegenerateTangent,
                2,
            ),
            (
                point(1, 0),
                point(1, 0),
                TangentJoinClass::EndpointMismatch,
                0,
            ),
        ] {
            dispatch_trace::reset();
            let report = dispatch_trace::with_recording(|| {
                classify_tangent_join(&origin, &east, &endpoint, &tangent, policy)
            });
            let trace = dispatch_trace::snapshot_trace();
            assert_eq!(report.class, class);
            assert_eq!(trace.operation_count("hyperlimit", "point2_equal"), 1);
            assert_eq!(
                trace.path_count("hyperlimit", "compare_reals", "exact-rational"),
                comparisons
            );
            assert_eq!(
                trace.operation_count("hyperlimit", "compare_reals"),
                comparisons
            );
            assert_eq!(trace.correlation_summary().approximation_events, 0);
        }
    }
}
