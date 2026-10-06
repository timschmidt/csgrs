#![cfg(feature = "interchange")]

use hypercircuit::{
    CircuitEventAgenda, CircuitEventCause, CircuitEventKind, CircuitEventPhase,
    CircuitEventRequest, CircuitEventTarget, CircuitId, Real, StochasticStream,
};

#[test]
fn agenda_and_stochastic_provenance_round_trip_without_fingerprint_drift() {
    let mut agenda = CircuitEventAgenda::new(Real::zero());
    agenda
        .schedule(CircuitEventRequest {
            time: Real::one(),
            phase: CircuitEventPhase::Topology,
            source: Some(CircuitEventTarget::External("trace".into())),
            target: CircuitEventTarget::Circuit(CircuitId::new("root").unwrap()),
            kind: CircuitEventKind::ProtectionState { active: true },
            cause: CircuitEventCause::Authored {
                provenance: "round-trip-test".into(),
            },
        })
        .unwrap();
    let fingerprint = agenda.fingerprint();
    let encoded = serde_json::to_string(&agenda).unwrap();
    let decoded: CircuitEventAgenda = serde_json::from_str(&encoded).unwrap();
    assert_eq!(decoded, agenda);
    assert_eq!(decoded.fingerprint(), fingerprint);

    let mut stream = StochasticStream::new(17);
    stream.draw_unit("threshold-noise").unwrap();
    let encoded = serde_json::to_string(&stream).unwrap();
    let decoded: StochasticStream = serde_json::from_str(&encoded).unwrap();
    assert_eq!(decoded, stream);
    assert!(decoded.verify().is_ok());
}
