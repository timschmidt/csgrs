#![cfg(feature = "dispatch-trace")]

use hyperevolution::{
    FitnessDirection, FitnessInterval, FitnessIntervalComparison, GpRealExpr, Real,
};
use hyperreal::Rational;

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

#[test]
fn exact_gp_and_interval_replay_do_not_request_approximation() {
    let expression = GpRealExpr::Mul(
        Box::new(GpRealExpr::Add(
            Box::new(GpRealExpr::Input(0)),
            Box::new(GpRealExpr::Constant(Box::new(q(1, 3)))),
        )),
        Box::new(GpRealExpr::Input(1)),
    );
    let left = FitnessInterval::new(q(1, 3), q(2, 3));
    let right = FitnessInterval::new(q(1, 2), q(3, 2));

    hyperreal::dispatch_trace::reset();
    let _recording = hyperreal::dispatch_trace::recording_scope();
    assert_eq!(
        expression.eval(&[q(2, 3), Real::from(3)]).unwrap(),
        Real::from(3)
    );
    assert_eq!(
        left.compare(&right, FitnessDirection::Minimize),
        FitnessIntervalComparison::Overlapping
    );

    let correlation = hyperreal::dispatch_trace::snapshot_trace().correlation_summary();
    assert!(correlation.dispatch_events > 0);
    assert!(correlation.rational_reductions > 0);
    assert_eq!(correlation.approximation_events, 0);
    assert_eq!(correlation.unknown_fact_events, 0);
}
