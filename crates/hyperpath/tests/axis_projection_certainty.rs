//! Exact axis projection must justify its input signs and its deterministic bottleneck.
use hyperlimit::{Certainty, PredicateOutcome, PredicatePolicy, Sign, classify_real_sign};
use hyperpath::{
    AffineSpanAxisProjection, AxisMotionLimits, RouteCertificationError,
    certify_axis_projected_motion_limits, plan_axis_projected_motion_limits,
};
use hyperreal::Real;

include!("../../hypersolve/tests/support/hidden_positive.rs");

fn limits() -> AxisMotionLimits {
    AxisMotionLimits {
        maximum_velocity: Real::one(),
        maximum_acceleration: Real::one(),
        maximum_jerk: Real::one(),
    }
}

fn span(components: Vec<Real>) -> [AffineSpanAxisProjection; 1] {
    [AffineSpanAxisProjection {
        absolute_axis_derivatives: components,
    }]
}

#[test]
fn an_unproved_zero_cannot_validate_an_absolute_axis_derivative() {
    // The signed addition formula leaves exactly -2^-2048. A second, moving
    // axis keeps the span nondegenerate even when the invalid axis is skipped.
    let negative = -hidden_positive();
    assert!(matches!(
        classify_real_sign(&negative, PredicatePolicy::APPROXIMATE_512),
        PredicateOutcome::Decided {
            value: Sign::Zero,
            certainty: Certainty::Approximate,
            ..
        }
    ));
    let spans = span(vec![Real::one(), negative]);
    let axes = [limits(), limits()];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let replay = certify_axis_projected_motion_limits(
            &spans,
            &axes,
            Real::one(),
            Real::one(),
            Real::one(),
            policy,
        );
        assert_eq!(
            replay.unwrap_err(),
            RouteCertificationError::PredicateUnresolved
        );
        let plan = plan_axis_projected_motion_limits(&spans, &axes, policy);
        assert_eq!(
            plan.unwrap_err(),
            RouteCertificationError::PredicateUnresolved
        );
    }
}

#[test]
fn an_unresolved_positive_limit_is_distinct_from_an_exact_zero() {
    let tiny = hidden_positive();
    let spans = span(vec![Real::one()]);
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        for (value, expected) in [
            (tiny.clone(), RouteCertificationError::PredicateUnresolved),
            (Real::zero(), RouteCertificationError::ZeroFeedRate),
        ] {
            let mut axis = limits();
            axis.maximum_velocity = value.clone();
            assert_eq!(
                plan_axis_projected_motion_limits(&spans, &[axis], policy).unwrap_err(),
                expected
            );
            assert_eq!(
                certify_axis_projected_motion_limits(
                    &spans,
                    &[limits()],
                    value,
                    Real::one(),
                    Real::one(),
                    policy
                )
                .unwrap_err(),
                expected
            );
        }
    }
}

#[test]
fn an_unresolved_order_cannot_establish_a_bottleneck_tie() {
    let mut axis = limits();
    axis.maximum_velocity = Real::one() + hidden_positive();
    let spans = span(vec![Real::one(), Real::one()]);
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let result = plan_axis_projected_motion_limits(&spans, &[axis.clone(), limits()], policy);
        assert_eq!(
            result.unwrap_err(),
            RouteCertificationError::PredicateUnresolved
        );
        let tie = plan_axis_projected_motion_limits(&spans, &[limits(), limits()], policy).unwrap();
        assert!(tie.all_satisfied());
        assert_eq!(
            (
                tie.feed_bottleneck.span_index,
                tie.feed_bottleneck.axis_index
            ),
            (0, 0)
        );
    }
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn rational_projection_preserves_exact_decision_counts() {
    use hyperreal::dispatch_trace;
    let axes = [limits(), limits(), limits()];
    let spans = span(vec![Real::zero(), Real::one(), Real::one()]);
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        dispatch_trace::reset();
        let plan = dispatch_trace::with_recording(|| {
            plan_axis_projected_motion_limits(&spans, &axes, policy)
        })
        .unwrap();
        assert_eq!(
            (
                plan.feed_bottleneck.span_index,
                plan.feed_bottleneck.axis_index
            ),
            (0, 1)
        );
        assert!(plan.all_satisfied());
        let trace = dispatch_trace::take_trace();
        let counts = trace
            .dispatch
            .iter()
            .filter(|row| row.layer == "hyperlimit")
            .map(|row| (row.operation, row.path, row.count))
            .collect::<Vec<_>>();
        assert_eq!(
            counts,
            [
                ("compare_reals", "exact-rational", 3),
                ("resolve_real_sign_direct", "structural-real-facts", 30),
            ]
        );
    }
}
