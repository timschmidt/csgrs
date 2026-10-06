use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, ComponentId, DeviceModel,
    DeviceModelId, DeviceModelKind, DevicePin, ErcIssue, ErcRuleDeck, ErcRuleId, ErcSeverity, Net,
    NetId, PinBinding, PinElectricalKind, PinRef, TransientPolicy,
};

fn circuit() -> Circuit {
    Circuit::new(
        CircuitId::new("erc").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
}

fn model(id: &str, kind: PinElectricalKind) -> DeviceModel {
    DeviceModel {
        id: DeviceModelId::new(id).unwrap(),
        kind: DeviceModelKind::Custom(id.into()),
        pins: vec![DevicePin {
            pin: PinRef::new("1").unwrap(),
            kind,
            optional: false,
        }],
        parameters: Vec::new(),
    }
}

fn instance(id: &str, model: &str, net: &NetId) -> CircuitInstance {
    CircuitInstance {
        id: CircuitInstanceId::new(id).unwrap(),
        component: ComponentId::new(id).unwrap(),
        part: None,
        model: DeviceModelId::new(model).unwrap(),
        pins: vec![PinBinding {
            pin: PinRef::new("1").unwrap(),
            net: net.clone(),
        }],
        parameters: Vec::new(),
    }
}

#[test]
fn erc_reports_push_pull_driver_conflicts() {
    let signal = NetId::new("signal").unwrap();
    let circuit = circuit()
        .with_net(Net {
            id: signal.clone(),
            is_ground: false,
        })
        .with_device_model(model("output", PinElectricalKind::Output))
        .with_device_model(model("input", PinElectricalKind::Input))
        .with_instance(instance("U1", "output", &signal))
        .with_instance(instance("U2", "output", &signal))
        .with_instance(instance("U3", "input", &signal));

    assert!(circuit.validate().is_valid());
    let report = circuit.electrical_rule_check();
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        ErcIssue::MultiplePushPullDrivers { net, drivers }
            if net == &signal && drivers.len() == 2
    )));
}

#[test]
fn erc_distinguishes_unpowered_inputs_from_valid_power_sources() {
    let vcc = NetId::new("VCC").unwrap();
    let unpowered = circuit()
        .with_net(Net {
            id: vcc.clone(),
            is_ground: false,
        })
        .with_device_model(model("load", PinElectricalKind::PowerInput))
        .with_instance(instance("U1", "load", &vcc));
    assert!(
        unpowered
            .electrical_rule_check()
            .issues
            .iter()
            .any(|issue| matches!(issue, ErcIssue::UnpoweredInputs { .. }))
    );

    let powered = unpowered
        .with_device_model(model("supply", PinElectricalKind::PowerOutput))
        .with_instance(instance("VR1", "supply", &vcc));
    assert!(powered.electrical_rule_check().is_valid());
}

#[test]
fn erc_rule_deck_controls_release_severity_and_suppression() {
    let signal = NetId::new("signal").unwrap();
    let circuit = circuit()
        .with_net(Net {
            id: signal.clone(),
            is_ground: false,
        })
        .with_device_model(model("output", PinElectricalKind::Output))
        .with_instance(instance("U1", "output", &signal))
        .with_instance(instance("U2", "output", &signal));

    let strict = circuit.electrical_rule_check_with(&ErcRuleDeck::default());
    assert!(!strict.is_release_clean());
    assert_eq!(strict.findings[0].severity, ErcSeverity::Error);
    assert_eq!(strict.findings[0].rule, ErcRuleId::MultiplePushPullDrivers);

    let review_deck = ErcRuleDeck::default()
        .with_severity(ErcRuleId::MultiplePushPullDrivers, ErcSeverity::Warning);
    let review = circuit.electrical_rule_check_with(&review_deck);
    assert!(review.is_release_clean());
    assert_eq!(review.findings[0].severity, ErcSeverity::Warning);

    let ignored_deck = ErcRuleDeck::default()
        .with_severity(ErcRuleId::MultiplePushPullDrivers, ErcSeverity::Ignore);
    assert!(
        circuit
            .electrical_rule_check_with(&ignored_deck)
            .findings
            .is_empty()
    );
}
