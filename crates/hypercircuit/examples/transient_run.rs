use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitParameter,
    ComponentId, DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, MnaUnknown, Net, NetId,
    PinBinding, PinElectricalKind, PinRef, Real, SourceStimulus, SourceWaveform,
    SourceWaveformPoint, TransientAdaptation, TransientPolicy, TransientRunPolicy,
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

fn parameter(name: &str, value: Real, unit: &str) -> CircuitParameter {
    CircuitParameter {
        name: name.into(),
        value,
        unit: unit.into(),
        source: "transient-run-example".into(),
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ground = NetId::new("GND").unwrap();
    let output = NetId::new("OUT").unwrap();
    let source_model = DeviceModelId::new("source").unwrap();
    let capacitor_model = DeviceModelId::new("capacitor").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("adaptive-capacitor").unwrap(),
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
        id: capacitor_model.clone(),
        kind: DeviceModelKind::Capacitor,
        pins: pins(),
        parameters: vec![parameter("capacitance", Real::one(), "F")],
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("I1").unwrap(),
        component: ComponentId::new("I1").unwrap(),
        part: None,
        model: source_model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("+").unwrap(),
                net: ground.clone(),
            },
            PinBinding {
                pin: PinRef::new("-").unwrap(),
                net: output.clone(),
            },
        ],
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("C1").unwrap(),
        component: ComponentId::new("C1").unwrap(),
        part: None,
        model: capacitor_model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("+").unwrap(),
                net: output.clone(),
            },
            PinBinding {
                pin: PinRef::new("-").unwrap(),
                net: ground,
            },
        ],
        parameters: Vec::new(),
    })
    .with_source_stimulus(SourceStimulus {
        component: ComponentId::new("I1").unwrap(),
        waveform: SourceWaveform::PiecewiseLinear {
            points: vec![
                SourceWaveformPoint {
                    time: Real::zero(),
                    value: Real::zero(),
                },
                SourceWaveformPoint {
                    time: Real::from(2),
                    value: Real::from(2),
                },
                SourceWaveformPoint {
                    time: Real::from(3),
                    value: Real::from(2),
                },
            ],
        },
    });

    let half = (Real::one() / Real::from(2)).unwrap();
    let tenth = (Real::one() / Real::from(10)).unwrap();
    let report = circuit.transient_run(
        &TransientRunPolicy {
            start_time: Real::zero(),
            stop_time: Real::from(3),
            initial_timestep: Real::one(),
            minimum_timestep: (Real::one() / Real::from(64)).unwrap(),
            maximum_timestep: Real::one(),
            maximum_accepted_steps: 1_000,
            maximum_rejected_steps: 100,
            adaptation: TransientAdaptation::StepDoubling {
                absolute_tolerance: tenth,
                relative_tolerance: Real::zero(),
                shrink_factor: half,
                growth_factor: Real::from(2),
            },
        },
        Default::default(),
    )?;
    let waveform = report
        .unknown_waveform(&MnaUnknown::NetVoltage(output))
        .expect("OUT exists in every accepted sample");
    let stimulus = report
        .source_waveform(&ComponentId::new("I1").unwrap())
        .expect("I1 is evaluated at every accepted sample");
    println!(
        "status={:?}, accepted={}, attempts={}",
        report.status,
        report.samples.len(),
        report.decisions.len()
    );
    for ((time, voltage), (_, current)) in waveform.into_iter().zip(stimulus) {
        println!("t={time}, I1={current}, OUT={voltage}");
    }
    Ok(())
}
