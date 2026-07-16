#![cfg(feature = "dispatch-trace")]

use hyperphysics::{
    ForceAccumulator3, ForceContribution3, Real, StepReplayReport3, Triangle3, Vector3,
};

fn v(x: i32, y: i32, z: i32) -> Vector3 {
    Vector3::new([Real::from(x), Real::from(y), Real::from(z)])
}

#[test]
fn exact_query_and_step_paths_do_not_request_approximation() {
    hyperreal::dispatch_trace::reset();
    let _recording = hyperreal::dispatch_trace::recording_scope();

    let triangle = Triangle3::new([v(0, 0, 0), v(10, 0, 0), v(0, 10, 0)]);
    triangle.classify_point(&v(2, 3, 0)).unwrap();

    let mut forces = ForceAccumulator3::default();
    forces.push(ForceContribution3 {
        source: "trace-force".into(),
        force: v(6, 0, 0),
    });
    StepReplayReport3::symplectic_euler_replay(
        Real::from(3),
        Real::from(1),
        v(0, 0, 0),
        v(1, 0, 0),
        &forces,
    )
    .unwrap();

    let trace = hyperreal::dispatch_trace::snapshot_trace();
    let correlation = trace.correlation_summary();
    assert!(correlation.dispatch_events > 0);
    assert!(correlation.sign_or_zero_query_events > 0);
    assert_eq!(correlation.approximation_events, 0);
    assert_eq!(correlation.unknown_fact_events, 0);
}
