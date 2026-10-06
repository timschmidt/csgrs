use hypercircuit::{
    CircuitEventAgenda, CircuitEventAgendaError, CircuitEventCause, CircuitEventKind,
    CircuitEventLifecycleAction, CircuitEventPhase, CircuitEventRequest, CircuitEventTarget,
    CircuitId, ExactBreakpointProvider, ExactBreakpointSchedule, LogicValue, Real,
    StochasticStream,
};

fn request(time: i64, phase: CircuitEventPhase, label: &str) -> CircuitEventRequest {
    CircuitEventRequest {
        time: Real::from(time),
        phase,
        source: None,
        target: CircuitEventTarget::Circuit(CircuitId::new("root").unwrap()),
        kind: CircuitEventKind::Behavioral {
            kind: label.into(),
            fields: Default::default(),
        },
        cause: CircuitEventCause::Authored {
            provenance: "test".into(),
        },
    }
}

#[test]
fn delivered_phase_cannot_be_reentered_at_the_same_time() {
    let mut agenda = CircuitEventAgenda::new(Real::zero());
    agenda
        .schedule(request(0, CircuitEventPhase::PostSolve, "current"))
        .unwrap();
    agenda.deliver_next().unwrap();
    assert_eq!(
        agenda
            .schedule(request(0, CircuitEventPhase::Stimulus, "too-late"))
            .unwrap_err(),
        CircuitEventAgendaError::PhaseInPast
    );
    agenda
        .schedule(request(0, CircuitEventPhase::Observation, "allowed"))
        .unwrap();
}

#[test]
fn agenda_clock_advancement_cannot_skip_pending_events() {
    let mut agenda = CircuitEventAgenda::new(Real::zero());
    agenda
        .schedule(request(2, CircuitEventPhase::Stimulus, "future"))
        .unwrap();
    agenda.advance_time(Real::one()).unwrap();
    assert_eq!(agenda.time(), &Real::one());
    assert_eq!(
        agenda.advance_time(Real::from(3)).unwrap_err(),
        CircuitEventAgendaError::ClockWouldSkipEvent
    );
    assert_eq!(
        agenda
            .schedule(request(0, CircuitEventPhase::Stimulus, "past"))
            .unwrap_err(),
        CircuitEventAgendaError::EventInPast
    );
}

#[test]
fn named_breakpoint_schedules_support_trace_and_topology_adapters() {
    let schedule = ExactBreakpointSchedule::new(
        "switch:sw1",
        vec![Real::one(), Real::from(3), Real::from(5)],
    )
    .unwrap();
    assert_eq!(
        schedule.next_breakpoint_after(&Real::from(2)).unwrap(),
        Some(Real::from(3))
    );
    assert!(ExactBreakpointSchedule::new("bad", vec![Real::one(), Real::one()]).is_err());
}

#[test]
fn agenda_orders_by_exact_time_phase_and_stable_sequence() {
    let mut agenda = CircuitEventAgenda::new(Real::zero());
    let late = agenda
        .schedule(request(2, CircuitEventPhase::Stimulus, "late"))
        .unwrap();
    let observation = agenda
        .schedule(request(1, CircuitEventPhase::Observation, "observation"))
        .unwrap();
    let first = agenda
        .schedule(request(1, CircuitEventPhase::Stimulus, "first"))
        .unwrap();
    let second = agenda
        .schedule(request(1, CircuitEventPhase::Stimulus, "second"))
        .unwrap();

    let pending = agenda
        .pending()
        .iter()
        .map(|event| event.id)
        .collect::<Vec<_>>();
    assert_eq!(pending, vec![first, second, observation, late]);
    assert_eq!(agenda.counters().maximum_pending, 4);

    assert_eq!(agenda.deliver_next().unwrap().id, first);
    assert_eq!(agenda.time(), &Real::one());
    assert_eq!(agenda.counters().delivered, 1);
}

#[test]
fn cancellation_and_rescheduling_retain_lifecycle_evidence() {
    let mut agenda = CircuitEventAgenda::new(Real::zero());
    let obsolete = agenda
        .schedule(request(3, CircuitEventPhase::PostSolve, "obsolete"))
        .unwrap();
    let replacement = agenda
        .reschedule(
            obsolete,
            request(2, CircuitEventPhase::PostSolve, "replacement"),
        )
        .unwrap();
    let canceled = agenda
        .schedule(request(4, CircuitEventPhase::Observation, "canceled"))
        .unwrap();
    agenda.cancel(canceled).unwrap();

    assert_eq!(agenda.pending().len(), 1);
    assert_eq!(agenda.pending()[0].id, replacement);
    assert_eq!(agenda.counters().rescheduled, 1);
    assert_eq!(agenda.counters().canceled, 1);
    assert!(agenda.lifecycle().iter().any(|record| {
        record.event.id == obsolete
            && record.action == CircuitEventLifecycleAction::Rescheduled
            && record.related == Some(replacement)
    }));
    assert!(agenda.lifecycle().iter().any(|record| {
        record.event.id == canceled && record.action == CircuitEventLifecycleAction::Canceled
    }));
    assert!(agenda.verify_lifecycle().is_valid());
}

#[test]
fn invalid_preordered_trace_is_rejected_atomically() {
    let mut agenda = CircuitEventAgenda::new(Real::zero());
    agenda
        .schedule(request(1, CircuitEventPhase::Stimulus, "existing"))
        .unwrap();
    let before = agenda.clone();
    let error = agenda
        .schedule_preordered([
            request(3, CircuitEventPhase::Stimulus, "later"),
            request(2, CircuitEventPhase::Stimulus, "earlier"),
        ])
        .unwrap_err();

    assert_eq!(error, CircuitEventAgendaError::TraceOutOfOrder { index: 1 });
    assert_eq!(agenda, before);
}

#[test]
fn preordered_fast_path_merges_with_existing_events() {
    let mut agenda = CircuitEventAgenda::new(Real::zero());
    agenda
        .schedule(request(2, CircuitEventPhase::Observation, "existing"))
        .unwrap();
    let ids = agenda
        .schedule_preordered([
            request(1, CircuitEventPhase::Stimulus, "trace-first"),
            request(2, CircuitEventPhase::Topology, "trace-second"),
        ])
        .unwrap();
    assert_eq!(agenda.counters().preordered, 2);
    assert_eq!(agenda.counters().scheduled, 3);
    assert_eq!(agenda.pending()[0].id, ids[0]);
    assert_eq!(agenda.pending()[1].id, ids[1]);
    assert!(agenda.verify_lifecycle().is_valid());
}

#[test]
fn agenda_fingerprints_and_stochastic_draws_are_reproducible() {
    let mut left = CircuitEventAgenda::new(Real::zero());
    let mut right = CircuitEventAgenda::new(Real::zero());
    for agenda in [&mut left, &mut right] {
        agenda
            .schedule(CircuitEventRequest {
                time: Real::one(),
                phase: CircuitEventPhase::Stimulus,
                source: None,
                target: CircuitEventTarget::External("clock".into()),
                kind: CircuitEventKind::DigitalTransition {
                    value: LogicValue::High,
                },
                cause: CircuitEventCause::Authored {
                    provenance: "fixture".into(),
                },
            })
            .unwrap();
    }
    assert_eq!(left.fingerprint(), right.fingerprint());

    let mut first = StochasticStream::new(42);
    let mut second = StochasticStream::new(42);
    let a = first.draw_unit("resistor-tolerance").unwrap();
    let b = second.draw_unit("resistor-tolerance").unwrap();
    assert_eq!(a, b);
    assert_eq!(a.id.get(), 0);
    assert_eq!(first.samples(), &[a]);
    assert!(first.verify().is_ok());
}
