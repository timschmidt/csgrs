use hypercircuit::{
    AdapterKind, BehaviorContext, BehaviorError, BehaviorRuntime, Circuit, CircuitEvent,
    CircuitEventAgenda, CircuitEventCause, CircuitEventHandler, CircuitEventKind,
    CircuitEventPhase, CircuitEventRequest, CircuitEventTarget, CircuitId, CircuitInstance,
    CircuitInstanceId, CircuitParameter, ComponentId, DeviceModel, DeviceModelId, DeviceModelKind,
    DevicePin, LogicValue, Net, NetId, PinBinding, PinElectricalKind, PinRef, Real, SourceStimulus,
    SourceWaveform, TransientAdaptation, TransientPolicy, TransientRunPolicy, TransientSession,
};

fn pins() -> Vec<DevicePin> {
    ["+", "-"]
        .into_iter()
        .map(|name| DevicePin {
            pin: PinRef::new(name).unwrap(),
            kind: PinElectricalKind::Passive,
            optional: false,
        })
        .collect()
}

fn instance(id: &str, model: DeviceModelId, pos: NetId, neg: NetId) -> CircuitInstance {
    CircuitInstance {
        id: CircuitInstanceId::new(id).unwrap(),
        component: ComponentId::new(id).unwrap(),
        part: None,
        model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("+").unwrap(),
                net: pos,
            },
            PinBinding {
                pin: PinRef::new("-").unwrap(),
                net: neg,
            },
        ],
        parameters: Vec::new(),
    }
}

struct SourceObserver {
    output: NetId,
}

impl CircuitEventHandler for SourceObserver {
    fn on_event(
        &mut self,
        event: &CircuitEvent,
        context: &mut BehaviorContext<'_>,
    ) -> Result<(), BehaviorError> {
        context.emit_at(
            event.time.clone(),
            CircuitEventPhase::Observation,
            CircuitEventTarget::Net(self.output.clone()),
            CircuitEventKind::DigitalTransition {
                value: LogicValue::High,
            },
        )?;
        Ok(())
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ground = NetId::new("GND").unwrap();
    let output = NetId::new("OUT").unwrap();
    let source_model = DeviceModelId::new("current-source").unwrap();
    let resistor_model = DeviceModelId::new("resistor").unwrap();
    let source = ComponentId::new("I1").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("mixed-signal-session").unwrap(),
        TransientPolicy::Trapezoidal,
        AdapterKind::TransientDae,
    )
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    })
    .with_net(Net {
        id: output.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: source_model.clone(),
        kind: DeviceModelKind::CurrentSource,
        pins: pins(),
        parameters: Vec::new(),
    })
    .with_device_model(DeviceModel {
        id: resistor_model.clone(),
        kind: DeviceModelKind::Resistor,
        pins: pins(),
        parameters: vec![CircuitParameter {
            name: "resistance".into(),
            value: Real::one(),
            unit: "ohm".into(),
            source: "mixed-signal-example".into(),
        }],
    })
    .with_instance(instance("I1", source_model, ground.clone(), output.clone()))
    .with_instance(instance("R1", resistor_model, output.clone(), ground))
    .with_source_stimulus(SourceStimulus {
        component: source.clone(),
        waveform: SourceWaveform::Constant(Real::one()),
    });

    let mut agenda = CircuitEventAgenda::new(Real::zero());
    agenda.schedule(CircuitEventRequest {
        time: Real::one(),
        phase: CircuitEventPhase::Stimulus,
        source: Some(CircuitEventTarget::External("testbench".into())),
        target: CircuitEventTarget::Component(source.clone()),
        kind: CircuitEventKind::SourceOverride {
            value: Real::from(3),
        },
        cause: CircuitEventCause::Authored {
            provenance: "example stimulus".into(),
        },
    })?;

    let mut callbacks = BehaviorRuntime::new();
    callbacks.register(
        CircuitEventTarget::Component(source),
        SourceObserver {
            output: output.clone(),
        },
    );
    let mut session = TransientSession::new(
        &circuit,
        TransientRunPolicy {
            start_time: Real::zero(),
            stop_time: Real::from(2),
            initial_timestep: Real::one(),
            minimum_timestep: Real::one(),
            maximum_timestep: Real::one(),
            maximum_accepted_steps: 4,
            maximum_rejected_steps: 1,
            adaptation: TransientAdaptation::Fixed,
        },
        Default::default(),
        agenda,
    )?;
    while session.status() == hypercircuit::TransientSessionStatus::Running {
        session.step_with_runtime(&mut callbacks, 64)?;
    }

    let audit = session.audit_report();
    println!(
        "status={:?}, samples={}, events={}, fingerprint={}",
        audit.transient.status,
        audit.transient.samples.len(),
        audit.delivered_events.len(),
        audit.fingerprint.value,
    );
    Ok(())
}
