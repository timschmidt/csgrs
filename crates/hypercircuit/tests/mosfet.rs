use std::collections::BTreeMap;

use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitParameter,
    CircuitValidationIssue, ComponentId, DeviceModel, DeviceModelId, DeviceModelKind, DevicePin,
    MosfetNewtonPolicy, MosfetNewtonStatus, MosfetPolarity, MosfetRegion, Net, NetId, PinBinding,
    PinElectricalKind, PinRef, Real, SquareLawMosfet, TransientPolicy,
};

fn parameter(name: &str, value: Real, unit: &str) -> CircuitParameter {
    CircuitParameter {
        name: name.into(),
        value,
        unit: unit.into(),
        source: "mosfet-test".into(),
    }
}

fn pin(name: &str) -> DevicePin {
    DevicePin {
        pin: PinRef::new(name).unwrap(),
        kind: PinElectricalKind::Passive,
        optional: false,
    }
}

#[test]
fn square_law_selects_exact_nmos_and_pmos_regions() {
    let nmos = SquareLawMosfet {
        component: ComponentId::new("M1").unwrap(),
        polarity: MosfetPolarity::NChannel,
        drain: None,
        gate: None,
        source: None,
        threshold_voltage: Real::one(),
        transconductance_parameter: Real::from(2),
        channel_length_modulation: Real::zero(),
    };
    let cutoff = nmos
        .operating_point(&Real::one(), &Real::from(5), &Real::zero())
        .unwrap();
    assert_eq!(cutoff.region, MosfetRegion::Cutoff);
    assert_eq!(cutoff.drain_current, Real::zero());

    let triode = nmos
        .operating_point(&Real::from(3), &Real::one(), &Real::zero())
        .unwrap();
    assert_eq!(triode.region, MosfetRegion::Triode);
    assert_eq!(triode.drain_current, Real::from(3));
    assert_eq!(triode.transconductance, Real::from(2));
    assert_eq!(triode.output_conductance, Real::from(2));

    let saturation = nmos
        .operating_point(&Real::from(3), &Real::from(3), &Real::zero())
        .unwrap();
    assert_eq!(saturation.region, MosfetRegion::Saturation);
    assert_eq!(saturation.drain_current, Real::from(4));
    assert_eq!(saturation.transconductance, Real::from(4));
    assert_eq!(saturation.output_conductance, Real::zero());

    let pmos = SquareLawMosfet {
        polarity: MosfetPolarity::PChannel,
        component: ComponentId::new("M2").unwrap(),
        ..nmos
    };
    let mirrored = pmos
        .operating_point(&Real::from(2), &Real::from(2), &Real::from(5))
        .unwrap();
    assert_eq!(mirrored.region, MosfetRegion::Saturation);
    assert_eq!(mirrored.drain_current, Real::from(-4));
    assert_eq!(mirrored.transconductance, Real::from(4));
}

fn common_source_fixture() -> Circuit {
    let ground = NetId::new("GND").unwrap();
    let supply = NetId::new("VDD").unwrap();
    let gate = NetId::new("GATE").unwrap();
    let drain = NetId::new("DRAIN").unwrap();
    let voltage_model = DeviceModelId::new("voltage").unwrap();
    let resistor_model = DeviceModelId::new("resistor").unwrap();
    let mosfet_model = DeviceModelId::new("nmos").unwrap();
    let voltage_instance =
        |id: &str, net: NetId, value: i64, model: &DeviceModelId| CircuitInstance {
            id: CircuitInstanceId::new(id).unwrap(),
            component: ComponentId::new(id).unwrap(),
            part: None,
            model: model.clone(),
            pins: vec![
                PinBinding {
                    pin: PinRef::new("+").unwrap(),
                    net,
                },
                PinBinding {
                    pin: PinRef::new("-").unwrap(),
                    net: ground.clone(),
                },
            ],
            parameters: vec![parameter("voltage", Real::from(value), "volt")],
        };
    Circuit::new(
        CircuitId::new("common-source").unwrap(),
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
    .with_device_model(DeviceModel {
        id: voltage_model.clone(),
        kind: DeviceModelKind::VoltageSource,
        pins: vec![pin("+"), pin("-")],
        parameters: Vec::new(),
    })
    .with_device_model(DeviceModel {
        id: resistor_model.clone(),
        kind: DeviceModelKind::Resistor,
        pins: vec![pin("1"), pin("2")],
        parameters: vec![parameter("resistance", Real::one(), "ohm")],
    })
    .with_device_model(DeviceModel {
        id: mosfet_model.clone(),
        kind: DeviceModelKind::Mosfet {
            polarity: MosfetPolarity::NChannel,
            drain: PinRef::new("D").unwrap(),
            gate: PinRef::new("G").unwrap(),
            source: PinRef::new("S").unwrap(),
        },
        pins: vec![pin("D"), pin("G"), pin("S")],
        parameters: vec![
            parameter("threshold_voltage", Real::one(), "volt"),
            parameter("transconductance_parameter", Real::one(), "ampere/volt^2"),
            parameter("channel_length_modulation", Real::zero(), "1/volt"),
        ],
    })
    .with_instance(voltage_instance("VDD1", supply.clone(), 5, &voltage_model))
    .with_instance(voltage_instance("VG1", gate.clone(), 2, &voltage_model))
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("R1").unwrap(),
        component: ComponentId::new("R1").unwrap(),
        part: None,
        model: resistor_model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("1").unwrap(),
                net: supply,
            },
            PinBinding {
                pin: PinRef::new("2").unwrap(),
                net: drain.clone(),
            },
        ],
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("M1").unwrap(),
        component: ComponentId::new("M1").unwrap(),
        part: None,
        model: mosfet_model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("D").unwrap(),
                net: drain,
            },
            PinBinding {
                pin: PinRef::new("G").unwrap(),
                net: gate,
            },
            PinBinding {
                pin: PinRef::new("S").unwrap(),
                net: ground,
            },
        ],
        parameters: Vec::new(),
    })
}

#[test]
fn common_source_dc_solve_replays_the_exact_saturation_law() {
    let report = common_source_fixture()
        .solve_mosfet_dc(&MosfetNewtonPolicy::default(), &BTreeMap::new())
        .unwrap();
    assert_eq!(report.status, MosfetNewtonStatus::Converged);
    assert!(!report.used_lossy_linearization);
    assert!(report.replay.accepted);
    assert!(report.replay.exact_zero);
    assert_eq!(
        report.net_voltage(&NetId::new("DRAIN").unwrap()).unwrap(),
        &(Real::from(9) / Real::from(2)).unwrap()
    );
    assert_eq!(report.replay.operating_points.len(), 1);
    assert_eq!(
        report.replay.operating_points[0].region,
        MosfetRegion::Saturation
    );
    assert_eq!(
        report.replay.operating_points[0].drain_current,
        (Real::one() / Real::from(2)).unwrap()
    );
    assert!(
        report
            .iterations
            .iter()
            .all(|iteration| iteration.linear_proposal_replay_accepted
                && iteration.linearizations.len() == 1)
    );
}

#[test]
fn p_channel_common_source_mirrors_current_and_stamp_polarity() {
    let mut circuit = common_source_fixture();
    let ground = NetId::new("GND").unwrap();
    let supply = NetId::new("VDD").unwrap();

    let model = circuit
        .device_models
        .iter_mut()
        .find(|model| model.id.as_str() == "nmos")
        .unwrap();
    model.kind = DeviceModelKind::Mosfet {
        polarity: MosfetPolarity::PChannel,
        drain: PinRef::new("D").unwrap(),
        gate: PinRef::new("G").unwrap(),
        source: PinRef::new("S").unwrap(),
    };
    let gate_source = circuit
        .instances
        .iter_mut()
        .find(|instance| instance.id.as_str() == "VG1")
        .unwrap();
    gate_source.parameters[0].value = Real::from(3);
    let resistor = circuit
        .instances
        .iter_mut()
        .find(|instance| instance.id.as_str() == "R1")
        .unwrap();
    resistor.pins[0].net = ground;
    let mosfet = circuit
        .instances
        .iter_mut()
        .find(|instance| instance.id.as_str() == "M1")
        .unwrap();
    mosfet
        .pins
        .iter_mut()
        .find(|binding| binding.pin.as_str() == "S")
        .unwrap()
        .net = supply;

    let report = circuit
        .solve_mosfet_dc(&MosfetNewtonPolicy::default(), &BTreeMap::new())
        .unwrap();
    assert_eq!(report.status, MosfetNewtonStatus::Converged);
    assert!(report.replay.exact_zero);
    assert_eq!(
        report.net_voltage(&NetId::new("DRAIN").unwrap()).unwrap(),
        &(Real::one() / Real::from(2)).unwrap()
    );
    assert_eq!(
        report.replay.operating_points[0].region,
        MosfetRegion::Saturation
    );
    assert_eq!(
        report.replay.operating_points[0].drain_current,
        -(Real::one() / Real::from(2)).unwrap()
    );
}

#[test]
fn mosfet_terminal_roles_are_structurally_validated() {
    let mut circuit = common_source_fixture();
    let model = circuit
        .device_models
        .iter_mut()
        .find(|model| model.id.as_str() == "nmos")
        .unwrap();
    model.kind = DeviceModelKind::Mosfet {
        polarity: MosfetPolarity::NChannel,
        drain: PinRef::new("D").unwrap(),
        gate: PinRef::new("D").unwrap(),
        source: PinRef::new("S").unwrap(),
    };
    assert!(circuit.validate().issues.iter().any(|issue| matches!(
        issue,
        CircuitValidationIssue::InvalidMosfetTerminals(model)
            if model.as_str() == "nmos"
    )));
}

#[cfg(feature = "interchange")]
#[test]
fn retained_mosfet_model_round_trips_through_semantic_json() {
    let document = hypercircuit::SemanticDocument::new(common_source_fixture(), None).unwrap();
    let json = document.to_json_pretty().unwrap();
    assert!(json.contains("\"Mosfet\""));
    assert!(json.contains("\"NChannel\""));
    assert_eq!(
        hypercircuit::SemanticDocument::from_json(&json).unwrap(),
        document
    );
}
