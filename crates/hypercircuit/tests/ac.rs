use std::collections::BTreeMap;

use hypercircuit::{
    AcAnalysisError, AcDeviceLoweringIssue, AcExcitation, AcNonlinearLinearization,
    AcOperatingPoint, AcOperatingPointDeviceKind, AcOperatingPointError,
    AcOperatingPointProvenance, AdapterKind, Circuit, CircuitId, CircuitInstance,
    CircuitInstanceId, CircuitParameter, ComponentId, DeviceModel, DeviceModelId, DeviceModelKind,
    DevicePin, DiodeNewtonPolicy, MnaUnknown, MosfetNewtonPolicy, MosfetNewtonStatus,
    MosfetPolarity, MosfetRegion, Net, NetId, Phasor, PinBinding, PinElectricalKind, PinRef, Real,
    TransientPolicy,
};

fn parameter(name: &str, value: Real, unit: &str) -> CircuitParameter {
    CircuitParameter {
        name: name.into(),
        value,
        unit: unit.into(),
        source: "ac-test".into(),
    }
}

fn pins(names: &[&str]) -> Vec<DevicePin> {
    names
        .iter()
        .map(|name| DevicePin {
            pin: PinRef::new(*name).unwrap(),
            kind: PinElectricalKind::Passive,
            optional: false,
        })
        .collect()
}

fn instance(id: &str, model: &str, bindings: &[(&str, &NetId)]) -> CircuitInstance {
    CircuitInstance {
        id: CircuitInstanceId::new(id).unwrap(),
        component: ComponentId::new(id).unwrap(),
        part: None,
        model: DeviceModelId::new(model).unwrap(),
        pins: bindings
            .iter()
            .map(|(pin, net)| PinBinding {
                pin: PinRef::new(*pin).unwrap(),
                net: (*net).clone(),
            })
            .collect(),
        parameters: Vec::new(),
    }
}

fn model(
    id: &str,
    kind: DeviceModelKind,
    pin_names: &[&str],
    parameters: Vec<CircuitParameter>,
) -> DeviceModel {
    DeviceModel {
        id: DeviceModelId::new(id).unwrap(),
        kind,
        pins: pins(pin_names),
        parameters,
    }
}

fn voltage<'a>(unknowns: &[MnaUnknown], candidate: &'a [Phasor], net: &NetId) -> &'a Phasor {
    let index = unknowns
        .iter()
        .position(|unknown| matches!(unknown, MnaUnknown::NetVoltage(id) if id == net))
        .unwrap();
    &candidate[index]
}

fn rc_low_pass() -> (Circuit, NetId) {
    let ground = NetId::new("GND").unwrap();
    let input = NetId::new("IN").unwrap();
    let output = NetId::new("OUT").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("rc-low-pass").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    })
    .with_net(Net {
        id: input.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: output.clone(),
        is_ground: false,
    })
    .with_device_model(model(
        "voltage-source",
        DeviceModelKind::VoltageSource,
        &["pos", "neg"],
        Vec::new(),
    ))
    .with_device_model(model(
        "resistor",
        DeviceModelKind::Resistor,
        &["1", "2"],
        vec![parameter("resistance", Real::one(), "ohm")],
    ))
    .with_device_model(model(
        "capacitor",
        DeviceModelKind::Capacitor,
        &["1", "2"],
        vec![parameter("capacitance", Real::one(), "F")],
    ))
    .with_instance(instance(
        "V1",
        "voltage-source",
        &[("pos", &input), ("neg", &ground)],
    ))
    .with_instance(instance("R1", "resistor", &[("1", &input), ("2", &output)]))
    .with_instance(instance(
        "C1",
        "capacitor",
        &[("1", &output), ("2", &ground)],
    ));
    (circuit, output)
}

#[test]
fn exact_rc_ac_sweep_solves_and_replays_in_authored_order() {
    let (circuit, output) = rc_low_pass();
    let excitation = AcExcitation::new(ComponentId::new("V1").unwrap(), Phasor::real(Real::one()));

    let sweep = circuit
        .ac_sweep([Real::one(), Real::from(2)], &[excitation])
        .unwrap();

    assert_eq!(sweep.points.len(), 2);
    let first = voltage(
        &sweep.points[0].unknowns,
        &sweep.points[0].solution.candidate,
        &output,
    );
    assert_eq!(
        first,
        &Phasor::new(
            (Real::one() / Real::from(2)).unwrap(),
            (-Real::one() / Real::from(2)).unwrap(),
        )
    );
    let second = voltage(
        &sweep.points[1].unknowns,
        &sweep.points[1].solution.candidate,
        &output,
    );
    assert_eq!(
        second,
        &Phasor::new(
            (Real::one() / Real::from(5)).unwrap(),
            (-Real::from(2) / Real::from(5)).unwrap(),
        )
    );
    assert!(
        sweep
            .points
            .iter()
            .all(|point| point.solution.replay.accepted)
    );
}

#[test]
fn inductor_and_vccs_share_the_exact_complex_mna_path() {
    let ground = NetId::new("GND").unwrap();
    let control = NetId::new("CTRL").unwrap();
    let output = NetId::new("OUT").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("inductor-vccs").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    })
    .with_net(Net {
        id: control.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: output.clone(),
        is_ground: false,
    })
    .with_device_model(model(
        "voltage-source",
        DeviceModelKind::VoltageSource,
        &["pos", "neg"],
        Vec::new(),
    ))
    .with_device_model(model(
        "inductor",
        DeviceModelKind::Inductor,
        &["1", "2"],
        vec![parameter("inductance", Real::one(), "H")],
    ))
    .with_device_model(model(
        "vccs",
        DeviceModelKind::ControlledSource,
        &["pos", "neg", "ctrl_pos", "ctrl_neg"],
        vec![parameter("transconductance", Real::from(2), "S")],
    ))
    .with_instance(instance(
        "V1",
        "voltage-source",
        &[("pos", &control), ("neg", &ground)],
    ))
    .with_instance(instance(
        "L1",
        "inductor",
        &[("1", &output), ("2", &ground)],
    ))
    .with_instance(instance(
        "G1",
        "vccs",
        &[
            ("pos", &output),
            ("neg", &ground),
            ("ctrl_pos", &control),
            ("ctrl_neg", &ground),
        ],
    ));
    let system = circuit
        .ac_mna_at(
            Real::from(2),
            &[AcExcitation::new(
                ComponentId::new("V1").unwrap(),
                Phasor::real(Real::one()),
            )],
        )
        .unwrap();
    let solution = system.solve_exact().unwrap();

    assert_eq!(
        voltage(&system.unknowns, &solution.candidate, &output),
        &Phasor::new(Real::zero(), -Real::from(4))
    );
    assert!(solution.replay.accepted);
}

#[test]
fn invalid_frequency_and_excitation_contracts_are_typed() {
    let (circuit, _) = rc_low_pass();
    let excitation = AcExcitation::new(ComponentId::new("V1").unwrap(), Phasor::real(Real::one()));
    assert_eq!(
        circuit.ac_mna_at(Real::zero(), std::slice::from_ref(&excitation)),
        Err(AcAnalysisError::InvalidAngularFrequency)
    );
    assert_eq!(
        circuit.ac_mna_at(Real::one(), &[excitation.clone(), excitation]),
        Err(AcAnalysisError::DuplicateExcitation(
            ComponentId::new("V1").unwrap()
        ))
    );
}

fn common_source() -> (Circuit, NetId) {
    let ground = NetId::new("GND").unwrap();
    let supply = NetId::new("VDD").unwrap();
    let gate = NetId::new("GATE").unwrap();
    let drain = NetId::new("DRAIN").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("ac-common-source").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
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
        id: gate.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: drain.clone(),
        is_ground: false,
    })
    .with_device_model(model(
        "dc-voltage",
        DeviceModelKind::VoltageSource,
        &["+", "-"],
        Vec::new(),
    ))
    .with_device_model(model(
        "load",
        DeviceModelKind::Resistor,
        &["1", "2"],
        vec![parameter("resistance", Real::one(), "ohm")],
    ))
    .with_device_model(model(
        "nmos",
        DeviceModelKind::Mosfet {
            polarity: MosfetPolarity::NChannel,
            drain: PinRef::new("D").unwrap(),
            gate: PinRef::new("G").unwrap(),
            source: PinRef::new("S").unwrap(),
        },
        &["D", "G", "S"],
        vec![
            parameter("threshold_voltage", Real::one(), "volt"),
            parameter("transconductance_parameter", Real::one(), "ampere/volt^2"),
            parameter("channel_length_modulation", Real::zero(), "1/volt"),
        ],
    ))
    .with_instance({
        let mut source = instance("VDD1", "dc-voltage", &[("+", &supply), ("-", &ground)]);
        source
            .parameters
            .push(parameter("voltage", Real::from(5), "volt"));
        source
    })
    .with_instance({
        let mut source = instance("VG1", "dc-voltage", &[("+", &gate), ("-", &ground)]);
        source
            .parameters
            .push(parameter("voltage", Real::from(2), "volt"));
        source
    })
    .with_instance(instance("R1", "load", &[("1", &supply), ("2", &drain)]))
    .with_instance(instance(
        "M1",
        "nmos",
        &[("D", &drain), ("G", &gate), ("S", &ground)],
    ));
    (circuit, drain)
}

#[test]
fn certified_mosfet_operating_point_drives_exact_small_signal_ac() {
    let (circuit, drain) = common_source();
    let dc = circuit
        .solve_mosfet_dc(&MosfetNewtonPolicy::default(), &BTreeMap::new())
        .unwrap();
    let operating_point = AcOperatingPoint::from_mosfet_newton(&dc).unwrap();
    let excitation = AcExcitation::new(ComponentId::new("VG1").unwrap(), Phasor::real(Real::one()));

    let legacy = circuit
        .lower_ac_devices(Real::one(), std::slice::from_ref(&excitation))
        .unwrap();
    assert_eq!(
        legacy.issues,
        vec![AcDeviceLoweringIssue::UnsupportedModel(
            ComponentId::new("M1").unwrap()
        )]
    );

    let sweep = circuit
        .small_signal_ac_sweep(
            [Real::one(), Real::from(10)],
            &[excitation],
            &operating_point,
        )
        .unwrap();
    assert_eq!(
        sweep.operating_point.provenance(),
        &AcOperatingPointProvenance::MosfetNewton {
            iterations: dc.iterations.len(),
            exact_zero_replay: true,
        }
    );
    assert_eq!(
        sweep
            .operating_point
            .certified_devices()
            .get(&ComponentId::new("M1").unwrap()),
        Some(&AcOperatingPointDeviceKind::SquareLawMosfet)
    );
    assert_eq!(sweep.nonlinear_linearizations.len(), 1);
    let AcNonlinearLinearization::Mosfet {
        operating_point: evidence,
    } = &sweep.nonlinear_linearizations[0]
    else {
        panic!("expected MOSFET evidence");
    };
    assert_eq!(evidence.region, MosfetRegion::Saturation);
    assert_eq!(evidence.transconductance, Real::one());
    assert_eq!(evidence.output_conductance, Real::zero());
    for point in &sweep.points {
        assert_eq!(
            voltage(&point.unknowns, &point.solution.candidate, &drain),
            &Phasor::real(-Real::one())
        );
        assert!(point.solution.replay.accepted);
    }

    let mut rejected = dc;
    rejected.status = MosfetNewtonStatus::IterationLimit;
    assert_eq!(
        AcOperatingPoint::from_mosfet_newton(&rejected),
        Err(AcOperatingPointError::NotConverged("MOSFET"))
    );

    let mut mismatched = circuit;
    mismatched
        .instances
        .iter_mut()
        .find(|instance| instance.component.as_str() == "M1")
        .unwrap()
        .component = ComponentId::new("M2").unwrap();
    assert_eq!(
        mismatched.small_signal_ac_mna_at(Real::one(), &[], &operating_point),
        Err(AcAnalysisError::Incomplete(vec![
            AcDeviceLoweringIssue::UncertifiedOperatingPoint {
                component: ComponentId::new("M2").unwrap(),
                required: AcOperatingPointDeviceKind::SquareLawMosfet,
            }
        ]))
    );
}

fn shockley_low_pass() -> (Circuit, NetId) {
    let ground = NetId::new("GND").unwrap();
    let supply = NetId::new("SUPPLY").unwrap();
    let output = NetId::new("OUT").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("ac-shockley").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
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
    .with_device_model(model(
        "dc-voltage",
        DeviceModelKind::VoltageSource,
        &["+", "-"],
        vec![parameter("voltage", Real::one(), "volt")],
    ))
    .with_device_model(model(
        "resistor",
        DeviceModelKind::Resistor,
        &["1", "2"],
        vec![parameter("resistance", Real::one(), "ohm")],
    ))
    .with_device_model(model(
        "diode",
        DeviceModelKind::Diode,
        &["A", "K"],
        vec![
            parameter(
                "saturation_current",
                (Real::one() / Real::from(10)).unwrap(),
                "ampere",
            ),
            parameter(
                "thermal_voltage",
                (Real::one() / Real::from(2)).unwrap(),
                "volt",
            ),
        ],
    ))
    .with_instance(instance(
        "V1",
        "dc-voltage",
        &[("+", &supply), ("-", &ground)],
    ))
    .with_instance(instance(
        "R1",
        "resistor",
        &[("1", &supply), ("2", &output)],
    ))
    .with_instance(instance("D1", "diode", &[("A", &output), ("K", &ground)]));
    (circuit, output)
}

#[test]
fn shockley_small_signal_retains_the_lossy_derivative_boundary() {
    let (circuit, output) = shockley_low_pass();
    let tolerance = (Real::one() / Real::from(100_000_000)).unwrap();
    let dc = circuit
        .solve_diode_dc(
            &DiodeNewtonPolicy {
                maximum_iterations: 32,
                voltage_tolerance: tolerance.clone(),
                current_tolerance: tolerance,
                damping: Real::one(),
            },
            &BTreeMap::new(),
        )
        .unwrap();
    let operating_point = AcOperatingPoint::from_diode_newton(&dc).unwrap();
    let sweep = circuit
        .small_signal_ac_sweep(
            [Real::one()],
            &[AcExcitation::new(
                ComponentId::new("V1").unwrap(),
                Phasor::real(Real::one()),
            )],
            &operating_point,
        )
        .unwrap();
    let AcNonlinearLinearization::Diode {
        conductance,
        used_lossy_linearization,
        ..
    } = &sweep.nonlinear_linearizations[0]
    else {
        panic!("expected diode evidence");
    };
    assert!(*used_lossy_linearization);
    assert_eq!(
        voltage(
            &sweep.points[0].unknowns,
            &sweep.points[0].solution.candidate,
            &output,
        ),
        &Phasor::real((Real::one() / (Real::one() + conductance.clone())).unwrap())
    );
    assert!(sweep.points[0].solution.replay.accepted);
}

#[cfg(feature = "layout")]
#[test]
fn fluent_declarative_rc_design_executes_the_same_ac_sweep() {
    use hypercircuit::{BoardOutline, Design, PcbStackup, parts};

    let mut design = Design::new(
        "fluent-ac",
        BoardOutline::rectangle(Real::from(20), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let input = design.signal("IN").unwrap();
    let output = design.signal("OUT").unwrap();
    let ground = design.ground("GND").unwrap();
    let source = design
        .add(parts::voltage_source("V1", Real::zero()))
        .unwrap();
    let resistor = design.add(parts::resistor("R1", Real::one())).unwrap();
    let capacitor = design.add(parts::capacitor("C1", Real::one())).unwrap();
    design
        .connect(
            &input,
            [source.pin("pos").unwrap(), resistor.pin("1").unwrap()],
        )
        .unwrap();
    design
        .connect(
            &output,
            [resistor.pin("2").unwrap(), capacitor.pin("1").unwrap()],
        )
        .unwrap();
    design
        .connect(
            &ground,
            [source.pin("neg").unwrap(), capacitor.pin("2").unwrap()],
        )
        .unwrap();
    let checked = design.finish().unwrap();

    let sweep = checked
        .circuit
        .ac_sweep(
            [Real::one()],
            &[AcExcitation::new(
                ComponentId::new(source.id().as_str()).unwrap(),
                Phasor::real(Real::one()),
            )],
        )
        .unwrap();
    let output_id = NetId::new("OUT").unwrap();
    assert_eq!(
        voltage(
            &sweep.points[0].unknowns,
            &sweep.points[0].solution.candidate,
            &output_id,
        ),
        &Phasor::new(
            (Real::one() / Real::from(2)).unwrap(),
            (-Real::one() / Real::from(2)).unwrap(),
        )
    );
}
