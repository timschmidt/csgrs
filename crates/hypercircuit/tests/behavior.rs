use hypercircuit::{
    BehaviorQueue, CircuitEventAgenda, CircuitEventCause, CircuitEventKind, CircuitEventPhase,
    CircuitEventRequest, CircuitEventTarget, CircuitId, Real, SelectiveEventMailbox,
};

fn event(agenda: &mut CircuitEventAgenda, time: i64, kind: &str) -> hypercircuit::CircuitEvent {
    agenda
        .schedule(CircuitEventRequest {
            time: Real::from(time),
            phase: CircuitEventPhase::PostSolve,
            source: None,
            target: CircuitEventTarget::Circuit(CircuitId::new("behavior").unwrap()),
            kind: CircuitEventKind::Behavioral {
                kind: kind.into(),
                fields: Default::default(),
            },
            cause: CircuitEventCause::Authored {
                provenance: "behavior-test".into(),
            },
        })
        .unwrap();
    agenda.deliver_next().unwrap()
}

#[test]
fn typed_queues_and_selective_receive_preserve_deterministic_order() {
    let mut queue = BehaviorQueue::new();
    queue.put(3_u32);
    queue.put(5_u32);
    assert_eq!(queue.take(), Some(3));
    assert_eq!(queue.take(), Some(5));

    let mut agenda = CircuitEventAgenda::new(Real::zero());
    let first = event(&mut agenda, 0, "first");
    let second = event(&mut agenda, 0, "second");
    let mut mailbox = SelectiveEventMailbox::default();
    mailbox.push(first);
    mailbox.push(second.clone());
    assert_eq!(
        mailbox
            .receive(|candidate| candidate.kind == second.kind)
            .unwrap()
            .id,
        second.id
    );
}

#[cfg(feature = "behavior-async")]
#[test]
fn cooperative_async_behavior_schedules_retained_timer_events() {
    use hypercircuit::{
        AsyncBehaviorRuntime, AsyncBehaviorStatus, AsyncCircuitBehavior, BehaviorContext,
        BehaviorError, CircuitEvent,
    };

    struct Sleeper;
    impl AsyncCircuitBehavior for Sleeper {
        fn resume(
            &mut self,
            _event: &CircuitEvent,
            context: &mut BehaviorContext<'_>,
        ) -> Result<AsyncBehaviorStatus, BehaviorError> {
            context.sleep_until(Real::one(), "wake")?;
            Ok(AsyncBehaviorStatus::Complete)
        }
    }

    let target = CircuitEventTarget::Circuit(CircuitId::new("behavior").unwrap());
    let mut agenda = CircuitEventAgenda::new(Real::zero());
    agenda
        .schedule(CircuitEventRequest {
            time: Real::zero(),
            phase: CircuitEventPhase::PostSolve,
            source: None,
            target: target.clone(),
            kind: CircuitEventKind::Timer {
                key: "start".into(),
            },
            cause: CircuitEventCause::Authored {
                provenance: "async-test".into(),
            },
        })
        .unwrap();
    let delivered = agenda.deliver_next().unwrap();
    let mut runtime = AsyncBehaviorRuntime::new();
    runtime.register(target, Sleeper);
    assert!(runtime.dispatch(&delivered, &mut agenda).unwrap());
    assert!(matches!(
        &agenda.pending()[0].kind,
        CircuitEventKind::Timer { key } if key == "wake"
    ));
}
