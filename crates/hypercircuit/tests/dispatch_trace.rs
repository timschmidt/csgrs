#![cfg(feature = "dispatch-trace")]

use hypercircuit::{ComponentId, LinearMnaSystem, LinearStamp, NetId, Real};

#[test]
fn exact_mna_replay_does_not_request_approximation() {
    hyperreal::dispatch_trace::reset();
    let _recording = hyperreal::dispatch_trace::recording_scope();

    let node = NetId::new("trace-node").unwrap();
    let system = LinearMnaSystem::from_stamps(
        vec![node.clone()],
        &[LinearStamp::Conductance {
            component: ComponentId::new("trace-conductance").unwrap(),
            part: None,
            pos: Some(node),
            neg: None,
            conductance: Real::from(3),
        }],
    )
    .unwrap();
    let replay = system.replay_candidate(&[Real::zero()]).unwrap();
    assert!(replay.accepted);

    let correlation = hyperreal::dispatch_trace::snapshot_trace().correlation_summary();
    assert!(correlation.dispatch_events > 0);
    assert!(correlation.sign_or_zero_query_events > 0);
    assert_eq!(correlation.approximation_events, 0);
    assert_eq!(correlation.unknown_fact_events, 0);
}
