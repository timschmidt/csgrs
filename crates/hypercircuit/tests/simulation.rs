use std::collections::BTreeMap;

use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitParameter,
    ComponentId, DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, DiodeNewtonPolicy,
    MnaUnknown, Net, NetId, PinBinding, PinElectricalKind, PinRef, Real, SourceStimulus,
    SourceWaveform, SourceWaveformPoint, TransientAdaptation, TransientPolicy, TransientRunError,
    TransientRunPolicy, TransientRunStatus, TransientStepDecisionKind,
};

fn parameter(name: &str, value: i64, unit: &str) -> CircuitParameter {
    CircuitParameter {
        name: name.into(),
        value: Real::from(value),
        unit: unit.into(),
        source: "simulation-test".into(),
    }
}

fn pins() -> Vec<DevicePin> {
    vec![
        DevicePin {
            pin: PinRef::new("+").unwrap(),
            kind: PinElectricalKind::Passive,
            optional: false,
        },
        DevicePin {
            pin: PinRef::new("-").unwrap(),
            kind: PinElectricalKind::Passive,
            optional: false,
        },
    ]
}

fn instance(
    id: &str,
    model: &str,
    output: &NetId,
    ground: &NetId,
    parameters: Vec<CircuitParameter>,
) -> CircuitInstance {
    CircuitInstance {
        id: CircuitInstanceId::new(id).unwrap(),
        component: ComponentId::new(id).unwrap(),
        part: None,
        model: DeviceModelId::new(model).unwrap(),
        pins: vec![
            PinBinding {
                pin: PinRef::new("+").unwrap(),
                net: output.clone(),
            },
            PinBinding {
                pin: PinRef::new("-").unwrap(),
                net: ground.clone(),
            },
        ],
        parameters,
    }
}

fn reactive_fixture(kind: DeviceModelKind, policy: TransientPolicy) -> Circuit {
    let output = NetId::new("OUT").unwrap();
    let ground = NetId::new("GND").unwrap();
    let source_kind = if matches!(kind, DeviceModelKind::Capacitor) {
        DeviceModelKind::CurrentSource
    } else {
        DeviceModelKind::VoltageSource
    };
    let source_parameter = if matches!(kind, DeviceModelKind::Capacitor) {
        "current"
    } else {
        "voltage"
    };
    let reactive_parameter = if matches!(kind, DeviceModelKind::Capacitor) {
        "capacitance"
    } else {
        "inductance"
    };
    let mut source = instance(
        "S1",
        "source",
        &output,
        &ground,
        vec![parameter(source_parameter, 1, "source-unit")],
    );
    if matches!(kind, DeviceModelKind::Capacitor) {
        source.pins[0].net = ground.clone();
        source.pins[1].net = output.clone();
    }
    Circuit::new(
        CircuitId::new("reactive-step").unwrap(),
        policy,
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
        id: DeviceModelId::new("source").unwrap(),
        kind: source_kind,
        pins: pins(),
        parameters: Vec::new(),
    })
    .with_device_model(DeviceModel {
        id: DeviceModelId::new("reactive").unwrap(),
        kind,
        pins: pins(),
        parameters: vec![parameter(reactive_parameter, 1, "reactive-unit")],
    })
    .with_instance(source)
    .with_instance(instance("X1", "reactive", &output, &ground, Vec::new()))
}

#[test]
fn retained_device_models_lower_solve_and_replay_without_manual_stamps() {
    let output = NetId::new("OUT").unwrap();
    let ground = NetId::new("GND").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("device-lowering").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
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
        id: DeviceModelId::new("resistor").unwrap(),
        kind: DeviceModelKind::Resistor,
        pins: pins(),
        parameters: vec![parameter("resistance", 10, "ohm")],
    })
    .with_device_model(DeviceModel {
        id: DeviceModelId::new("voltage-source").unwrap(),
        kind: DeviceModelKind::VoltageSource,
        pins: pins(),
        parameters: vec![parameter("voltage", 5, "V")],
    })
    .with_instance(instance(
        "R1",
        "resistor",
        &output,
        &ground,
        vec![parameter("resistance", 2, "ohm")],
    ))
    .with_instance(instance(
        "V1",
        "voltage-source",
        &output,
        &ground,
        Vec::new(),
    ));

    let lowered = circuit.lower_linear_devices();
    assert!(lowered.is_complete());
    assert_eq!(lowered.stamps.len(), 2);
    let solution = circuit
        .linear_mna_from_devices()
        .unwrap()
        .solve_exact()
        .unwrap();
    assert_eq!(solution.candidate[0], Real::from(5));
    assert_eq!(
        solution.candidate[1],
        (Real::from(-5) / Real::from(2)).unwrap()
    );
    assert!(solution.replay.accepted);
}

#[test]
fn backward_euler_capacitor_steps_replay_and_carry_exact_history() {
    let circuit = reactive_fixture(
        DeviceModelKind::Capacitor,
        TransientPolicy::GearBdf { order: 1 },
    );
    let first = circuit
        .transient_step(Real::one(), &Default::default())
        .unwrap();
    assert!(first.solution.replay.accepted);
    let first_state = &first.next_history.reactive[&ComponentId::new("X1").unwrap()];
    assert_eq!(first_state.voltage, Real::one());
    assert_eq!(first_state.current, Real::one());

    let second = circuit
        .transient_step(Real::one(), &first.next_history)
        .unwrap();
    let second_state = &second.next_history.reactive[&ComponentId::new("X1").unwrap()];
    assert_eq!(second_state.voltage, Real::from(2));
    assert_eq!(second_state.current, Real::one());
}

#[test]
fn trapezoidal_capacitor_uses_voltage_and_current_history() {
    let circuit = reactive_fixture(DeviceModelKind::Capacitor, TransientPolicy::Trapezoidal);
    let first = circuit
        .transient_step(Real::one(), &Default::default())
        .unwrap();
    let half = (Real::one() / Real::from(2)).unwrap();
    assert_eq!(
        first.next_history.reactive[&ComponentId::new("X1").unwrap()].voltage,
        half
    );
    let second = circuit
        .transient_step(Real::one(), &first.next_history)
        .unwrap();
    assert_eq!(
        second.next_history.reactive[&ComponentId::new("X1").unwrap()].voltage,
        (Real::from(3) / Real::from(2)).unwrap()
    );
}

#[test]
fn backward_euler_inductor_integrates_exact_current() {
    let circuit = reactive_fixture(
        DeviceModelKind::Inductor,
        TransientPolicy::GearBdf { order: 1 },
    );
    let first = circuit
        .transient_step(Real::one(), &Default::default())
        .unwrap();
    let state = &first.next_history.reactive[&ComponentId::new("X1").unwrap()];
    assert_eq!(state.voltage, Real::one());
    assert_eq!(state.current, Real::one());
    let second = circuit
        .transient_step(Real::one(), &first.next_history)
        .unwrap();
    assert_eq!(
        second.next_history.reactive[&ComponentId::new("X1").unwrap()].current,
        Real::from(2)
    );
}

#[test]
fn fixed_transient_run_emits_exact_time_ordered_waveforms() {
    let circuit = reactive_fixture(
        DeviceModelKind::Capacitor,
        TransientPolicy::GearBdf { order: 1 },
    );
    let report = circuit
        .transient_run(
            &TransientRunPolicy {
                start_time: Real::zero(),
                stop_time: Real::from(3),
                initial_timestep: Real::one(),
                minimum_timestep: Real::one(),
                maximum_timestep: Real::one(),
                maximum_accepted_steps: 3,
                maximum_rejected_steps: 1,
                adaptation: TransientAdaptation::Fixed,
            },
            Default::default(),
        )
        .unwrap();
    assert_eq!(report.status, TransientRunStatus::Complete);
    assert_eq!(report.samples.len(), 3);
    assert_eq!(
        report
            .samples
            .iter()
            .map(|sample| sample.time.clone())
            .collect::<Vec<_>>(),
        vec![Real::one(), Real::from(2), Real::from(3)]
    );
    assert!(report.decisions.iter().all(|decision| {
        decision.kind == TransientStepDecisionKind::Accepted
            && decision.maximum_error_ratio.is_none()
    }));
    let waveform = report
        .unknown_waveform(&MnaUnknown::NetVoltage(NetId::new("OUT").unwrap()))
        .unwrap();
    assert_eq!(
        waveform
            .iter()
            .map(|(_, value)| value.clone())
            .collect::<Vec<_>>(),
        vec![Real::one(), Real::from(2), Real::from(3)]
    );
    assert_eq!(
        report
            .reactive_voltage_waveform(&ComponentId::new("X1").unwrap())
            .unwrap(),
        waveform
    );
}

#[test]
fn adaptive_transient_run_rejects_then_refines_to_the_exact_stop_time() {
    let circuit = reactive_fixture(DeviceModelKind::Capacitor, TransientPolicy::Trapezoidal);
    let report = circuit
        .transient_run(
            &TransientRunPolicy {
                start_time: Real::zero(),
                stop_time: Real::one(),
                initial_timestep: Real::one(),
                minimum_timestep: (Real::one() / Real::from(16)).unwrap(),
                maximum_timestep: Real::one(),
                maximum_accepted_steps: 32,
                maximum_rejected_steps: 8,
                adaptation: TransientAdaptation::StepDoubling {
                    absolute_tolerance: (Real::one() / Real::from(10)).unwrap(),
                    relative_tolerance: Real::zero(),
                    shrink_factor: (Real::one() / Real::from(2)).unwrap(),
                    growth_factor: Real::from(2),
                },
            },
            Default::default(),
        )
        .unwrap();
    assert_eq!(report.status, TransientRunStatus::Complete);
    assert!(
        report
            .decisions
            .iter()
            .any(|decision| decision.kind == TransientStepDecisionKind::Rejected)
    );
    assert!(
        report
            .decisions
            .iter()
            .filter(|decision| decision.kind == TransientStepDecisionKind::Accepted)
            .all(|decision| {
                decision
                    .maximum_error_ratio
                    .as_ref()
                    .is_some_and(|ratio| ratio <= &Real::one())
            })
    );
    assert_eq!(report.samples.last().unwrap().time, Real::one());
    assert_eq!(
        report.final_history.reactive,
        report.samples.last().unwrap().reactive
    );
}

#[test]
fn transient_run_returns_audited_limits_instead_of_partial_success() {
    let circuit = reactive_fixture(
        DeviceModelKind::Inductor,
        TransientPolicy::GearBdf { order: 1 },
    );
    let limited = circuit
        .transient_run(
            &TransientRunPolicy {
                start_time: Real::zero(),
                stop_time: Real::from(10),
                initial_timestep: Real::one(),
                minimum_timestep: Real::one(),
                maximum_timestep: Real::one(),
                maximum_accepted_steps: 2,
                maximum_rejected_steps: 1,
                adaptation: TransientAdaptation::Fixed,
            },
            Default::default(),
        )
        .unwrap();
    assert_eq!(limited.status, TransientRunStatus::AcceptedStepLimit);
    assert_eq!(limited.samples.len(), 2);
    assert_eq!(limited.samples.last().unwrap().time, Real::from(2));

    let minimum = reactive_fixture(DeviceModelKind::Capacitor, TransientPolicy::Trapezoidal)
        .transient_run(
            &TransientRunPolicy {
                start_time: Real::zero(),
                stop_time: Real::one(),
                initial_timestep: Real::one(),
                minimum_timestep: (Real::one() / Real::from(2)).unwrap(),
                maximum_timestep: Real::one(),
                maximum_accepted_steps: 10,
                maximum_rejected_steps: 10,
                adaptation: TransientAdaptation::StepDoubling {
                    absolute_tolerance: (Real::one() / Real::from(1_000)).unwrap(),
                    relative_tolerance: Real::zero(),
                    shrink_factor: (Real::one() / Real::from(2)).unwrap(),
                    growth_factor: Real::from(2),
                },
            },
            Default::default(),
        )
        .unwrap();
    assert_eq!(minimum.status, TransientRunStatus::MinimumTimestep);
    assert!(minimum.samples.is_empty());
    assert_eq!(minimum.decisions.len(), 2);
}

#[test]
fn transient_run_rejects_invalid_time_policy_before_stepping() {
    let circuit = reactive_fixture(
        DeviceModelKind::Capacitor,
        TransientPolicy::GearBdf { order: 1 },
    );
    let result = circuit.transient_run(
        &TransientRunPolicy {
            stop_time: Real::zero(),
            ..TransientRunPolicy::default()
        },
        Default::default(),
    );
    assert_eq!(result, Err(TransientRunError::InvalidPolicy));
}

#[test]
fn source_waveforms_evaluate_steps_and_exact_linear_interpolation() {
    let step = SourceWaveform::Step {
        initial: Real::from(-1),
        final_value: Real::from(3),
        at: Real::from(2),
    };
    assert_eq!(step.value_at(&Real::one()).unwrap(), Real::from(-1));
    assert_eq!(step.value_at(&Real::from(2)).unwrap(), Real::from(3));

    let ramp = SourceWaveform::PiecewiseLinear {
        points: vec![
            SourceWaveformPoint {
                time: Real::zero(),
                value: Real::zero(),
            },
            SourceWaveformPoint {
                time: Real::from(2),
                value: Real::from(4),
            },
            SourceWaveformPoint {
                time: Real::from(3),
                value: Real::one(),
            },
        ],
    };
    assert_eq!(ramp.value_at(&Real::from(-1)).unwrap(), Real::zero());
    assert_eq!(ramp.value_at(&Real::one()).unwrap(), Real::from(2));
    assert_eq!(ramp.value_at(&Real::from(4)).unwrap(), Real::one());
}

#[test]
fn pulse_waveforms_repeat_with_exact_phase_values_and_breakpoints() {
    let pulse = SourceWaveform::Pulse {
        low_value: Real::zero(),
        high_value: Real::from(4),
        delay: Real::one(),
        rise_time: Real::from(2),
        high_time: Real::one(),
        fall_time: Real::from(2),
        period: Real::from(7),
    };
    for (time, expected) in [
        (0, Real::zero()),
        (1, Real::zero()),
        (2, Real::from(2)),
        (3, Real::from(4)),
        (4, Real::from(4)),
        (5, Real::from(2)),
        (6, Real::zero()),
        (8, Real::zero()),
        (9, Real::from(2)),
    ] {
        assert_eq!(pulse.value_at(&Real::from(time)).unwrap(), expected);
    }
    for (time, expected) in [(0, 1), (1, 3), (3, 4), (4, 6), (6, 8), (8, 10)] {
        assert_eq!(
            pulse.next_breakpoint_after(&Real::from(time)).unwrap(),
            Some(Real::from(expected))
        );
    }
}

#[test]
fn analytic_waveforms_evaluate_exact_sine_and_exponential_laws() {
    let sine = SourceWaveform::Sine {
        offset: Real::from(2),
        amplitude: Real::from(3),
        frequency: (Real::one() / Real::from(4)).unwrap(),
        delay: Real::one(),
        damping: Real::zero(),
        phase_degrees: Real::zero(),
    };
    assert_eq!(sine.value_at(&Real::zero()).unwrap(), Real::from(2));
    assert_eq!(sine.value_at(&Real::one()).unwrap(), Real::from(2));
    assert_eq!(sine.value_at(&Real::from(2)).unwrap(), Real::from(5));
    assert_eq!(sine.value_at(&Real::from(3)).unwrap(), Real::from(2));
    assert_eq!(sine.value_at(&Real::from(4)).unwrap(), Real::from(-1));
    assert_eq!(
        sine.next_breakpoint_after(&Real::zero()).unwrap(),
        Some(Real::one())
    );
    assert_eq!(sine.next_breakpoint_after(&Real::one()).unwrap(), None);

    let exponential = SourceWaveform::Exponential {
        initial: Real::one(),
        pulsed: Real::from(5),
        rise_delay: Real::one(),
        rise_time_constant: Real::one(),
        fall_delay: Real::from(3),
        fall_time_constant: Real::one(),
    };
    let transition = |elapsed: i64| Real::one() - (-Real::from(elapsed)).exp().unwrap();
    assert_eq!(
        exponential.value_at(&Real::from(2)).unwrap(),
        Real::one() + Real::from(4) * transition(1)
    );
    assert_eq!(
        exponential.value_at(&Real::from(3)).unwrap(),
        Real::one() + Real::from(4) * transition(2)
    );
    assert_eq!(
        exponential.value_at(&Real::from(4)).unwrap(),
        Real::one() + Real::from(4) * transition(3) - Real::from(4) * transition(1)
    );
    assert_eq!(
        exponential.next_breakpoint_after(&Real::zero()).unwrap(),
        Some(Real::one())
    );
    assert_eq!(
        exponential.next_breakpoint_after(&Real::one()).unwrap(),
        Some(Real::from(3))
    );
    assert_eq!(
        exponential.next_breakpoint_after(&Real::from(3)).unwrap(),
        None
    );
}

fn driven_capacitor() -> Circuit {
    let mut circuit = reactive_fixture(
        DeviceModelKind::Capacitor,
        TransientPolicy::GearBdf { order: 1 },
    );
    circuit
        .instances
        .iter_mut()
        .find(|instance| instance.component.as_str() == "S1")
        .unwrap()
        .parameters
        .clear();
    circuit.with_source_stimulus(SourceStimulus {
        component: ComponentId::new("S1").unwrap(),
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
    })
}

#[test]
fn time_aware_lowering_overrides_scalar_source_parameters_with_audited_values() {
    let source = ComponentId::new("S1").unwrap();
    let lowered = driven_capacitor().lower_linear_devices_at(&Real::one());
    assert!(lowered.is_complete());
    assert_eq!(lowered.source_values[&source], Real::one());
    assert!(lowered.stamps.iter().any(|stamp| matches!(
        stamp,
        hypercircuit::LinearStamp::CurrentSource {
            component,
            current,
            ..
        } if component == &source && current == &Real::one()
    )));
}

#[test]
fn driven_transient_run_retains_source_and_response_waveforms() {
    let circuit = driven_capacitor();
    let report = circuit
        .transient_run(
            &TransientRunPolicy {
                start_time: Real::zero(),
                stop_time: Real::from(3),
                initial_timestep: Real::one(),
                minimum_timestep: Real::one(),
                maximum_timestep: Real::one(),
                maximum_accepted_steps: 3,
                maximum_rejected_steps: 1,
                adaptation: TransientAdaptation::Fixed,
            },
            Default::default(),
        )
        .unwrap();
    assert_eq!(report.status, TransientRunStatus::Complete);
    let source = report
        .source_waveform(&ComponentId::new("S1").unwrap())
        .unwrap();
    assert_eq!(
        source
            .iter()
            .map(|(_, value)| value.clone())
            .collect::<Vec<_>>(),
        vec![Real::one(), Real::from(2), Real::from(2)]
    );
    assert_eq!(
        report
            .unknown_waveform(&MnaUnknown::NetVoltage(NetId::new("OUT").unwrap()))
            .unwrap()
            .iter()
            .map(|(_, value)| value.clone())
            .collect::<Vec<_>>(),
        vec![Real::one(), Real::from(3), Real::from(5)]
    );
}

#[test]
fn transient_run_lands_on_every_periodic_source_event() {
    let mut circuit = driven_capacitor();
    circuit.source_stimuli[0].waveform = SourceWaveform::Pulse {
        low_value: Real::zero(),
        high_value: Real::one(),
        delay: Real::one(),
        rise_time: Real::zero(),
        high_time: Real::one(),
        fall_time: Real::zero(),
        period: Real::from(4),
    };
    let report = circuit
        .transient_run(
            &TransientRunPolicy {
                start_time: Real::zero(),
                stop_time: Real::from(10),
                initial_timestep: Real::from(10),
                minimum_timestep: Real::one(),
                maximum_timestep: Real::from(10),
                maximum_accepted_steps: 10,
                maximum_rejected_steps: 1,
                adaptation: TransientAdaptation::Fixed,
            },
            Default::default(),
        )
        .unwrap();
    assert_eq!(report.status, TransientRunStatus::Complete);
    assert_eq!(
        report
            .source_waveform(&ComponentId::new("S1").unwrap())
            .unwrap(),
        vec![
            (Real::one(), Real::one()),
            (Real::from(2), Real::zero()),
            (Real::from(5), Real::one()),
            (Real::from(6), Real::zero()),
            (Real::from(9), Real::one()),
            (Real::from(10), Real::zero()),
        ]
    );
}

#[test]
fn analytic_source_delays_drive_the_exact_transient_endpoint_grid() {
    let mut circuit = driven_capacitor();
    let waveform = SourceWaveform::Exponential {
        initial: Real::zero(),
        pulsed: Real::one(),
        rise_delay: Real::one(),
        rise_time_constant: Real::one(),
        fall_delay: Real::from(3),
        fall_time_constant: Real::one(),
    };
    circuit.source_stimuli[0].waveform = waveform.clone();
    let report = circuit
        .transient_run(
            &TransientRunPolicy {
                start_time: Real::zero(),
                stop_time: Real::from(5),
                initial_timestep: Real::from(5),
                minimum_timestep: Real::one(),
                maximum_timestep: Real::from(5),
                maximum_accepted_steps: 3,
                maximum_rejected_steps: 1,
                adaptation: TransientAdaptation::Fixed,
            },
            Default::default(),
        )
        .unwrap();
    assert_eq!(report.status, TransientRunStatus::Complete);
    let source = report
        .source_waveform(&ComponentId::new("S1").unwrap())
        .unwrap();
    assert_eq!(
        source
            .iter()
            .map(|(time, _)| time.clone())
            .collect::<Vec<_>>(),
        vec![Real::one(), Real::from(3), Real::from(5)]
    );
    for (time, value) in source {
        assert_eq!(value, waveform.value_at(&time).unwrap());
    }
}

#[test]
fn adaptive_driven_run_evaluates_each_refined_endpoint() {
    let report = driven_capacitor()
        .transient_run(
            &TransientRunPolicy {
                start_time: Real::zero(),
                stop_time: Real::from(2),
                initial_timestep: Real::one(),
                minimum_timestep: (Real::one() / Real::from(16)).unwrap(),
                maximum_timestep: Real::one(),
                maximum_accepted_steps: 64,
                maximum_rejected_steps: 16,
                adaptation: TransientAdaptation::StepDoubling {
                    absolute_tolerance: (Real::one() / Real::from(10)).unwrap(),
                    relative_tolerance: Real::zero(),
                    shrink_factor: (Real::one() / Real::from(2)).unwrap(),
                    growth_factor: Real::from(2),
                },
            },
            Default::default(),
        )
        .unwrap();
    assert_eq!(report.status, TransientRunStatus::Complete);
    assert!(
        report
            .decisions
            .iter()
            .any(|decision| decision.kind == TransientStepDecisionKind::Rejected)
    );
    for (time, value) in report
        .source_waveform(&ComponentId::new("S1").unwrap())
        .unwrap()
    {
        assert_eq!(time, value);
    }
}

fn exact_parameter(name: &str, value: Real, unit: &str) -> CircuitParameter {
    CircuitParameter {
        name: name.into(),
        value,
        unit: unit.into(),
        source: "mixed-diode-transient".into(),
    }
}

fn mixed_diode_transient_fixture() -> Circuit {
    let ground = NetId::new("GND").unwrap();
    let supply = NetId::new("SUPPLY").unwrap();
    let output = NetId::new("OUT").unwrap();
    let voltage = DeviceModelId::new("voltage").unwrap();
    let resistor = DeviceModelId::new("resistor").unwrap();
    let capacitor = DeviceModelId::new("capacitor").unwrap();
    let diode = DeviceModelId::new("diode").unwrap();
    let bind = |id: &str, model: DeviceModelId, pos: NetId, neg: NetId| -> CircuitInstance {
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
    };
    Circuit::new(
        CircuitId::new("mixed-diode-transient").unwrap(),
        TransientPolicy::Trapezoidal,
        AdapterKind::TransientDae,
    )
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    })
    .with_net(Net {
        id: supply.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: output.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: voltage.clone(),
        kind: DeviceModelKind::VoltageSource,
        pins: pins(),
        parameters: Vec::new(),
    })
    .with_device_model(DeviceModel {
        id: resistor.clone(),
        kind: DeviceModelKind::Resistor,
        pins: pins(),
        parameters: vec![exact_parameter("resistance", Real::one(), "ohm")],
    })
    .with_device_model(DeviceModel {
        id: capacitor.clone(),
        kind: DeviceModelKind::Capacitor,
        pins: pins(),
        parameters: vec![exact_parameter("capacitance", Real::one(), "farad")],
    })
    .with_device_model(DeviceModel {
        id: diode.clone(),
        kind: DeviceModelKind::Diode,
        pins: pins(),
        parameters: vec![
            exact_parameter(
                "saturation_current",
                (Real::one() / Real::from(10)).unwrap(),
                "ampere",
            ),
            exact_parameter(
                "thermal_voltage",
                (Real::one() / Real::from(2)).unwrap(),
                "volt",
            ),
        ],
    })
    .with_instance(bind("V1", voltage, supply.clone(), ground.clone()))
    .with_instance(bind("R1", resistor, supply, output.clone()))
    .with_instance(bind("C1", capacitor, output.clone(), ground.clone()))
    .with_instance(bind("D1", diode, output, ground))
    .with_source_stimulus(SourceStimulus {
        component: ComponentId::new("V1").unwrap(),
        waveform: SourceWaveform::Step {
            initial: Real::zero(),
            final_value: Real::one(),
            at: Real::one(),
        },
    })
}

#[test]
fn mixed_linear_reactive_diode_run_replays_every_nonlinear_endpoint() {
    let tolerance = (Real::one() / Real::from(100_000_000)).unwrap();
    let report = mixed_diode_transient_fixture()
        .diode_transient_run(
            &TransientRunPolicy {
                start_time: Real::zero(),
                stop_time: Real::from(2),
                initial_timestep: (Real::one() / Real::from(2)).unwrap(),
                minimum_timestep: (Real::one() / Real::from(2)).unwrap(),
                maximum_timestep: (Real::one() / Real::from(2)).unwrap(),
                maximum_accepted_steps: 4,
                maximum_rejected_steps: 1,
                adaptation: TransientAdaptation::Fixed,
            },
            &DiodeNewtonPolicy {
                maximum_iterations: 32,
                voltage_tolerance: tolerance.clone(),
                current_tolerance: tolerance,
                damping: Real::one(),
            },
            Default::default(),
            BTreeMap::new(),
        )
        .unwrap();
    assert_eq!(report.status, TransientRunStatus::Complete);
    assert_eq!(report.samples.len(), 4);
    assert_eq!(report.nonlinear_steps.len(), 4);
    assert!(report.nonlinear_steps.iter().all(|step| step.iterations > 0
        && step.replay_accepted
        && step.maximum_kcl_residual >= Real::zero()
        && step.maximum_branch_residual >= Real::zero()));
    let waveform = report
        .unknown_waveform(&MnaUnknown::NetVoltage(NetId::new("OUT").unwrap()))
        .unwrap();
    assert_eq!(waveform[0].1, Real::zero());
    assert!(waveform[1].1 > Real::zero());
    assert!(waveform[3].1 > waveform[1].1);
    assert!(waveform[3].1 < Real::one());
}
