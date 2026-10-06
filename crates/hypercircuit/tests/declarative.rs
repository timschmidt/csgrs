use hypercircuit::{
    AdapterKind, Bus, BusId, BusSlice, BusSliceId, BusSliceOrder, Circuit, CircuitId,
    CircuitInstance, CircuitInstanceId, CircuitPort, CircuitValidationIssue, ComponentId,
    DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, Net, NetId, PinElectricalKind, PinRef,
    PortDirection, PortId, RailIntent, RailKind, Real, SourceStimulus, SourceWaveform,
    SourceWaveformPoint, TransientPolicy,
};

fn net(id: &str) -> NetId {
    NetId::new(id).unwrap()
}

#[test]
fn declarative_connectivity_retains_buses_ports_and_pin_nets() {
    let ground = net("GND");
    let signal = net("SDA");
    let model_id = DeviceModelId::new("connector").unwrap();
    let instance_id = CircuitInstanceId::new("J1").unwrap();
    let mut circuit = Circuit::new(
        CircuitId::new("sensor-interface").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    })
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    })
    .with_bus(Bus {
        id: BusId::new("I2C").unwrap(),
        nets: vec![signal.clone()],
    })
    .with_port(CircuitPort {
        id: PortId::new("sda").unwrap(),
        net: signal.clone(),
        direction: PortDirection::Bidirectional,
        optional: false,
    })
    .with_device_model(DeviceModel {
        id: model_id.clone(),
        kind: DeviceModelKind::Custom("connector".into()),
        pins: vec![
            DevicePin {
                pin: PinRef::new("1").unwrap(),
                kind: PinElectricalKind::Bidirectional,
                optional: false,
            },
            DevicePin {
                pin: PinRef::new("2").unwrap(),
                kind: PinElectricalKind::PowerInput,
                optional: false,
            },
        ],
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: instance_id.clone(),
        component: ComponentId::new("J1").unwrap(),
        part: None,
        model: model_id,
        pins: Vec::new(),
        parameters: Vec::new(),
    });

    circuit
        .connect_pin(&instance_id, PinRef::new("1").unwrap(), &signal)
        .unwrap();
    circuit
        .connect_pin(&instance_id, PinRef::new("2").unwrap(), &ground)
        .unwrap();

    assert!(circuit.validate().is_valid());
    assert_eq!(circuit.instances[0].pins.len(), 2);
}

#[test]
fn validation_reports_cross_reference_errors_before_lowering() {
    let missing = net("missing");
    let circuit = Circuit::new(
        CircuitId::new("invalid").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_port(CircuitPort {
        id: PortId::new("dangling").unwrap(),
        net: missing,
        direction: PortDirection::Passive,
        optional: false,
    });

    assert!(!circuit.validate().is_valid());
}

#[test]
fn bus_slices_and_rail_intent_are_retained_and_validated() {
    let ground = net("GND");
    let data = (0..4)
        .map(|index| net(&format!("D{index}")))
        .collect::<Vec<_>>();
    let bus_id = BusId::new("DATA").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("bus-and-rails").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    })
    .with_net(Net {
        id: data[0].clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: data[1].clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: data[2].clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: data[3].clone(),
        is_ground: false,
    })
    .with_bus(Bus {
        id: bus_id.clone(),
        nets: data.clone(),
    })
    .with_bus_slice(BusSlice {
        id: BusSliceId::new("upper-pair-reversed").unwrap(),
        bus: bus_id,
        offset: 2,
        width: 2,
        order: BusSliceOrder::Reverse,
    })
    .with_rail(RailIntent {
        net: ground,
        nominal_voltage: Some(Real::zero()),
        max_current: Some(Real::from(2)),
        kind: RailKind::Ground,
    });
    assert!(circuit.validate().is_valid());
    assert_eq!(
        circuit.bus_slices[0].members(&circuit.buses[0]).unwrap(),
        vec![data[3].clone(), data[2].clone()]
    );

    let mut invalid = circuit;
    invalid.bus_slices[0].width = 10;
    invalid.rails[0].max_current = Some(Real::zero());
    let report = invalid.validate();
    assert!(
        report
            .issues
            .iter()
            .any(|issue| matches!(issue, CircuitValidationIssue::InvalidBusSliceRange(_)))
    );
    assert!(
        report
            .issues
            .iter()
            .any(|issue| matches!(issue, CircuitValidationIssue::InvalidRailCurrent(_)))
    );
}

#[test]
fn source_stimuli_are_component_addressed_and_structurally_validated() {
    let component = ComponentId::new("U1").unwrap();
    let model = DeviceModelId::new("logic").unwrap();
    let stimulus = SourceStimulus {
        component: component.clone(),
        waveform: SourceWaveform::PiecewiseLinear { points: Vec::new() },
    };
    let circuit = Circuit::new(
        CircuitId::new("invalid-stimuli").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_device_model(DeviceModel {
        id: model.clone(),
        kind: DeviceModelKind::Custom("logic".into()),
        pins: Vec::new(),
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("U1").unwrap(),
        component,
        part: None,
        model,
        pins: Vec::new(),
        parameters: Vec::new(),
    })
    .with_source_stimulus(stimulus.clone())
    .with_source_stimulus(stimulus)
    .with_source_stimulus(SourceStimulus {
        component: ComponentId::new("missing").unwrap(),
        waveform: SourceWaveform::PiecewiseLinear {
            points: vec![
                SourceWaveformPoint {
                    time: Real::one(),
                    value: Real::zero(),
                },
                SourceWaveformPoint {
                    time: Real::zero(),
                    value: Real::one(),
                },
            ],
        },
    })
    .with_source_stimulus(SourceStimulus {
        component: ComponentId::new("invalid-pulse").unwrap(),
        waveform: SourceWaveform::Pulse {
            low_value: Real::zero(),
            high_value: Real::one(),
            delay: Real::zero(),
            rise_time: Real::from(2),
            high_time: Real::one(),
            fall_time: Real::one(),
            period: Real::from(3),
        },
    })
    .with_source_stimulus(SourceStimulus {
        component: ComponentId::new("invalid-sine").unwrap(),
        waveform: SourceWaveform::Sine {
            offset: Real::zero(),
            amplitude: Real::one(),
            frequency: Real::from(-1),
            delay: Real::zero(),
            damping: Real::zero(),
            phase_degrees: Real::zero(),
        },
    })
    .with_source_stimulus(SourceStimulus {
        component: ComponentId::new("invalid-exponential").unwrap(),
        waveform: SourceWaveform::Exponential {
            initial: Real::zero(),
            pulsed: Real::one(),
            rise_delay: Real::from(2),
            rise_time_constant: Real::zero(),
            fall_delay: Real::one(),
            fall_time_constant: Real::one(),
        },
    });
    let report = circuit.validate();
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        CircuitValidationIssue::DuplicateSourceStimulus(component)
            if component.as_str() == "U1"
    )));
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        CircuitValidationIssue::InvalidSourceStimulusTarget(component)
            if component.as_str() == "U1"
    )));
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        CircuitValidationIssue::EmptySourceWaveform(component)
            if component.as_str() == "U1"
    )));
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        CircuitValidationIssue::UnknownSourceStimulusComponent(component)
            if component.as_str() == "missing"
    )));
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        CircuitValidationIssue::NonIncreasingSourceWaveformTime(component)
            if component.as_str() == "missing"
    )));
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        CircuitValidationIssue::InvalidPulseSourceWaveform(component)
            if component.as_str() == "invalid-pulse"
    )));
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        CircuitValidationIssue::InvalidSineSourceWaveform(component)
            if component.as_str() == "invalid-sine"
    )));
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        CircuitValidationIssue::InvalidExponentialSourceWaveform(component)
            if component.as_str() == "invalid-exponential"
    )));
}
