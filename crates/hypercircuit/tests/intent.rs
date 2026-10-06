use std::collections::BTreeMap;

use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitLibrary,
    ComponentId, DesignIntent, DesignIntentIssue, DeviceModel, DeviceModelId, DeviceModelKind,
    DimensionedValue, FunctionalBinding, FunctionalBindingTarget, FunctionalRole,
    FunctionalRoleAssignment, FunctionalRoleTarget, Net, NetId, NetIntent, NetKind, NetScope,
    PartClass, PartRef, PartSelectionIntent, QuantityDimension, ResolvedPartEvidence,
    SemanticOrigin, SemanticTarget, SourcePosition, SourceSpan, TransientPolicy,
};

fn library() -> CircuitLibrary {
    let model = DeviceModelId::new("capacitor").unwrap();
    let instance = CircuitInstanceId::new("C1").unwrap();
    let net = NetId::new("VCC").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("board").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: net,
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: model.clone(),
        kind: DeviceModelKind::Capacitor,
        pins: Vec::new(),
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: instance.clone(),
        component: ComponentId::new("C1").unwrap(),
        part: Some(PartRef::new("CM::LCSC::C_100nF_0402").unwrap()),
        model,
        pins: Vec::new(),
        parameters: Vec::new(),
    });
    CircuitLibrary {
        root: circuit.id.clone(),
        circuits: vec![circuit],
    }
}

#[test]
fn coppertrace_intent_preserves_spans_roles_typed_values_and_resolution_evidence() {
    let circuit = CircuitId::new("board").unwrap();
    let net = NetId::new("VCC").unwrap();
    let instance = CircuitInstanceId::new("C1").unwrap();
    let voltage =
        DimensionedValue::new(hypercircuit::Real::from(5), QuantityDimension::Voltage, "V");
    let capacitance = DimensionedValue::new(
        (hypercircuit::Real::one() / hypercircuit::Real::from(10_000_000)).unwrap(),
        QuantityDimension::Capacitance,
        "F",
    );
    let intent = DesignIntent {
        origins: vec![SemanticOrigin {
            target: SemanticTarget::Instance {
                circuit: circuit.clone(),
                instance: instance.clone(),
            },
            span: SourceSpan::new(
                "board.trace",
                SourcePosition::new(12, 2, 5),
                SourcePosition::new(42, 2, 35),
            ),
            label: Some("component declaration".into()),
        }],
        nets: vec![NetIntent {
            circuit: circuit.clone(),
            net: net.clone(),
            kind: NetKind::PowerSupply,
            scope: NetScope::Global("VCC".into()),
            net_class: None,
            nominal_value: Some(voltage),
        }],
        roles: vec![FunctionalRoleAssignment {
            target: FunctionalRoleTarget::Instance {
                circuit: circuit.clone(),
                instance: instance.clone(),
            },
            role: FunctionalRole::DecouplingCapacitor,
            bindings: vec![FunctionalBinding {
                name: "supply".into(),
                target: FunctionalBindingTarget::Net {
                    circuit: circuit.clone(),
                    net,
                },
            }],
            parameters: BTreeMap::new(),
        }],
        resolved_parts: vec![ResolvedPartEvidence {
            circuit,
            instance,
            class: PartClass::Capacitor,
            selection: PartSelectionIntent::BareValue(capacitance),
            resolved: PartRef::new("CM::LCSC::C_100nF_0402").unwrap(),
            artifact_digest: Some("sha256:0123456789abcdef".into()),
            policy: BTreeMap::from([
                ("package".into(), "0402".into()),
                ("tolerance".into(), "10%".into()),
            ]),
            attributes: BTreeMap::new(),
        }],
    };

    assert!(intent.validate(&library()).is_valid());
}

#[test]
fn conflicting_global_declarations_and_stale_part_resolution_are_rejected() {
    let circuit = CircuitId::new("board").unwrap();
    let net = NetId::new("VCC").unwrap();
    let instance = CircuitInstanceId::new("C1").unwrap();
    let intent = DesignIntent {
        nets: vec![
            NetIntent {
                circuit: circuit.clone(),
                net: net.clone(),
                kind: NetKind::PowerSupply,
                scope: NetScope::Global("VCC".into()),
                net_class: None,
                nominal_value: Some(DimensionedValue::new(
                    hypercircuit::Real::from(5),
                    QuantityDimension::Voltage,
                    "V",
                )),
            },
            NetIntent {
                circuit: circuit.clone(),
                net,
                kind: NetKind::DigitalSignal,
                scope: NetScope::Global("VCC".into()),
                net_class: None,
                nominal_value: None,
            },
        ],
        resolved_parts: vec![ResolvedPartEvidence {
            circuit,
            instance,
            class: PartClass::Capacitor,
            selection: PartSelectionIntent::Alias("default-cap".into()),
            resolved: PartRef::new("CM::LCSC::different").unwrap(),
            artifact_digest: None,
            policy: BTreeMap::new(),
            attributes: BTreeMap::new(),
        }],
        ..DesignIntent::default()
    };
    let report = intent.validate(&library());

    assert!(report.issues.iter().any(
        |issue| matches!(issue, DesignIntentIssue::ConflictingGlobalNet(key) if key == "VCC")
    ));
    assert!(
        report
            .issues
            .iter()
            .any(|issue| matches!(issue, DesignIntentIssue::ResolvedPartMismatch { .. }))
    );
}
