#![cfg(feature = "interchange")]

use std::collections::BTreeMap;

use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitLibrary,
    CircuitPort, ComponentId, DesignIntent, DeviceModel, DeviceModelId, DeviceModelKind, DevicePin,
    DimensionedValue, FunctionalBinding, FunctionalBindingTarget, FunctionalRole,
    FunctionalRoleAssignment, FunctionalRoleTarget, Net, NetId, NetIntent, NetKind, NetScope,
    PartClass, PartRef, PartSelectionIntent, PinBinding, PinElectricalKind, PinRef, PortDirection,
    PortId, QuantityDimension, ResolvedPartEvidence, SemanticDocument, SemanticOrigin,
    SemanticTarget, SourcePosition, SourceSpan, SubcircuitInstance, SubcircuitInstanceId,
    TransientPolicy,
};

fn circuit(id: &str) -> Circuit {
    Circuit::new(
        CircuitId::new(id).unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
}

fn net(id: &str, ground: bool) -> Net {
    Net {
        id: NetId::new(id).unwrap(),
        is_ground: ground,
    }
}

fn net_intent(circuit: &str, net: &str, kind: NetKind, scope: NetScope) -> NetIntent {
    NetIntent {
        circuit: CircuitId::new(circuit).unwrap(),
        net: NetId::new(net).unwrap(),
        kind,
        scope,
        net_class: None,
        nominal_value: None,
    }
}

fn binding(circuit: &str, name: &str, net: &str) -> FunctionalBinding {
    FunctionalBinding {
        name: name.into(),
        target: FunctionalBindingTarget::Net {
            circuit: CircuitId::new(circuit).unwrap(),
            net: NetId::new(net).unwrap(),
        },
    }
}

fn origin(circuit: &str, label: &str, line: u32) -> SemanticOrigin {
    SemanticOrigin {
        target: SemanticTarget::Circuit(CircuitId::new(circuit).unwrap()),
        span: SourceSpan::new(
            format!("fixtures/{label}.copper"),
            SourcePosition::new(u64::from(line) * 10, line, 1),
            SourcePosition::new(u64::from(line) * 10 + 9, line, 10),
        ),
        label: Some(label.into()),
    }
}

fn assert_round_trip_and_erc(library: CircuitLibrary, intent: DesignIntent) {
    assert!(library.coppertrace_erc(&intent).is_valid());
    let document = SemanticDocument::new(circuit("interchange-placeholder"), None)
        .unwrap()
        .with_circuit_library(library)
        .unwrap()
        .with_design_intent(intent.clone())
        .unwrap();
    let json = document.to_json_pretty().unwrap();
    let decoded = SemanticDocument::from_json(&json).unwrap();
    assert_eq!(decoded.design_intent, intent);
    assert!(
        decoded
            .circuit_library()
            .coppertrace_erc(&intent)
            .is_valid()
    );
}

#[test]
fn spec_rc_lowpass_survives_semantic_interchange_and_role_erc() {
    let filter = circuit("RcLowpass")
        .with_net(net("sig_in", false))
        .with_net(net("sig_out", false))
        .with_net(net("gnd", true));
    let intent = DesignIntent {
        origins: vec![origin("RcLowpass", "rc-lowpass", 1)],
        nets: vec![
            net_intent(
                "RcLowpass",
                "sig_in",
                NetKind::AnalogSignal,
                NetScope::Local,
            ),
            net_intent(
                "RcLowpass",
                "sig_out",
                NetKind::AnalogSignal,
                NetScope::Local,
            ),
            net_intent("RcLowpass", "gnd", NetKind::Ground, NetScope::Local),
        ],
        roles: vec![FunctionalRoleAssignment {
            target: FunctionalRoleTarget::Circuit(CircuitId::new("RcLowpass").unwrap()),
            role: FunctionalRole::LowpassFilter,
            bindings: vec![
                binding("RcLowpass", "sig_in", "sig_in"),
                binding("RcLowpass", "sig_out", "sig_out"),
                binding("RcLowpass", "gnd", "gnd"),
            ],
            parameters: BTreeMap::from([(
                "fc".into(),
                DimensionedValue::new(1_000.into(), QuantityDimension::Frequency, "Hz"),
            )]),
        }],
        ..DesignIntent::default()
    };
    assert_round_trip_and_erc(
        CircuitLibrary {
            root: filter.id.clone(),
            circuits: vec![filter],
        },
        intent,
    );
}

#[test]
fn spec_lt3980_buck_preserves_supply_direction_and_computed_values() {
    let buck = circuit("Buck3v3")
        .with_net(net("vin", false))
        .with_net(net("out", false))
        .with_net(net("gnd", true))
        .with_port(CircuitPort {
            id: PortId::new("vin").unwrap(),
            net: NetId::new("vin").unwrap(),
            direction: PortDirection::PowerInput,
            optional: false,
        });
    let mut output = net_intent("Buck3v3", "out", NetKind::PowerSupply, NetScope::Local);
    output.nominal_value = Some(DimensionedValue::new(
        33.into(),
        QuantityDimension::Voltage,
        "0.1V",
    ));
    let intent = DesignIntent {
        origins: vec![origin("Buck3v3", "lt3980-buck", 1)],
        nets: vec![
            net_intent("Buck3v3", "vin", NetKind::PowerSupply, NetScope::Local),
            output,
            net_intent("Buck3v3", "gnd", NetKind::Ground, NetScope::Local),
        ],
        roles: vec![FunctionalRoleAssignment {
            target: FunctionalRoleTarget::Circuit(CircuitId::new("Buck3v3").unwrap()),
            role: FunctionalRole::BuckRegulator,
            bindings: vec![
                binding("Buck3v3", "vin", "vin"),
                binding("Buck3v3", "vout", "out"),
                binding("Buck3v3", "gnd", "gnd"),
            ],
            parameters: BTreeMap::from([
                (
                    "vout".into(),
                    DimensionedValue::new(33.into(), QuantityDimension::Voltage, "0.1V"),
                ),
                (
                    "feedback-reference".into(),
                    DimensionedValue::new(79.into(), QuantityDimension::Voltage, "0.01V"),
                ),
            ]),
        }],
        ..DesignIntent::default()
    };
    assert_round_trip_and_erc(
        CircuitLibrary {
            root: buck.id.clone(),
            circuits: vec![buck],
        },
        intent,
    );
}

#[test]
fn spec_rp2354a_board_joins_global_rails_and_retains_coppermine_evidence() {
    let rp_part = PartRef::new("CM::LCSC::RP2354A").unwrap();
    let cap_part = PartRef::new("CM::LCSC::C_100nF_0402").unwrap();
    let mcu_model = DeviceModelId::new("rp2354a").unwrap();
    let cap_model = DeviceModelId::new("capacitor").unwrap();
    let mut board = circuit("McuBoard")
        .with_net(net("p5v", false))
        .with_net(net("p3v3", false))
        .with_net(net("gnd", true))
        .with_port(CircuitPort {
            id: PortId::new("p5v-in").unwrap(),
            net: NetId::new("p5v").unwrap(),
            direction: PortDirection::PowerInput,
            optional: false,
        })
        .with_device_model(DeviceModel {
            id: mcu_model.clone(),
            kind: DeviceModelKind::Custom("ic".into()),
            pins: ["VDD", "IOVDD", "VSS"]
                .into_iter()
                .map(|pin| DevicePin {
                    pin: PinRef::new(pin).unwrap(),
                    kind: PinElectricalKind::Passive,
                    optional: false,
                })
                .collect(),
            parameters: Vec::new(),
        })
        .with_device_model(DeviceModel {
            id: cap_model.clone(),
            kind: DeviceModelKind::Capacitor,
            pins: ["1", "2"]
                .into_iter()
                .map(|pin| DevicePin {
                    pin: PinRef::new(pin).unwrap(),
                    kind: PinElectricalKind::Passive,
                    optional: false,
                })
                .collect(),
            parameters: Vec::new(),
        })
        .with_instance(CircuitInstance {
            id: CircuitInstanceId::new("u").unwrap(),
            component: ComponentId::new("u").unwrap(),
            part: Some(rp_part.clone()),
            model: mcu_model,
            pins: vec![
                PinBinding {
                    pin: PinRef::new("VDD").unwrap(),
                    net: NetId::new("p3v3").unwrap(),
                },
                PinBinding {
                    pin: PinRef::new("IOVDD").unwrap(),
                    net: NetId::new("p3v3").unwrap(),
                },
                PinBinding {
                    pin: PinRef::new("VSS").unwrap(),
                    net: NetId::new("gnd").unwrap(),
                },
            ],
            parameters: Vec::new(),
        })
        .with_subcircuit(SubcircuitInstance {
            id: SubcircuitInstanceId::new("reg").unwrap(),
            circuit: CircuitId::new("Buck3v3").unwrap(),
            ports: Vec::new(),
            parameter_overrides: Vec::new(),
        });
    for index in 0..4 {
        board = board.with_instance(CircuitInstance {
            id: CircuitInstanceId::new(format!("cd{index}")).unwrap(),
            component: ComponentId::new(format!("cd{index}")).unwrap(),
            part: Some(cap_part.clone()),
            model: cap_model.clone(),
            pins: vec![
                PinBinding {
                    pin: PinRef::new("1").unwrap(),
                    net: NetId::new("p3v3").unwrap(),
                },
                PinBinding {
                    pin: PinRef::new("2").unwrap(),
                    net: NetId::new("gnd").unwrap(),
                },
            ],
            parameters: Vec::new(),
        });
    }
    let buck = circuit("Buck3v3");
    let mut roles = vec![FunctionalRoleAssignment {
        target: FunctionalRoleTarget::Subcircuit {
            circuit: CircuitId::new("McuBoard").unwrap(),
            instance: SubcircuitInstanceId::new("reg").unwrap(),
        },
        role: FunctionalRole::BuckRegulator,
        bindings: vec![
            binding("McuBoard", "vin", "p5v"),
            binding("McuBoard", "vout", "p3v3"),
            binding("McuBoard", "gnd", "gnd"),
        ],
        parameters: BTreeMap::new(),
    }];
    let mut resolved_parts = vec![ResolvedPartEvidence {
        circuit: CircuitId::new("McuBoard").unwrap(),
        instance: CircuitInstanceId::new("u").unwrap(),
        class: PartClass::Ic,
        selection: PartSelectionIntent::ExactRegistryPath {
            registry: "CM".into(),
            catalog: "LCSC".into(),
            part_id: "RP2354A".into(),
        },
        resolved: rp_part,
        artifact_digest: Some("sha256:rp2354a-fixture".into()),
        policy: BTreeMap::from([("catalog-version".into(), "fixture-v1".into())]),
        attributes: BTreeMap::new(),
    }];
    for index in 0..4 {
        let id = CircuitInstanceId::new(format!("cd{index}")).unwrap();
        roles.push(FunctionalRoleAssignment {
            target: FunctionalRoleTarget::Instance {
                circuit: CircuitId::new("McuBoard").unwrap(),
                instance: id.clone(),
            },
            role: FunctionalRole::DecouplingCapacitor,
            bindings: vec![
                binding("McuBoard", "supply", "p3v3"),
                binding("McuBoard", "reference", "gnd"),
                FunctionalBinding {
                    name: "target".into(),
                    target: FunctionalBindingTarget::Pin {
                        circuit: CircuitId::new("McuBoard").unwrap(),
                        instance: CircuitInstanceId::new("u").unwrap(),
                        pin: PinRef::new(if index % 2 == 0 { "VDD" } else { "IOVDD" }).unwrap(),
                    },
                },
            ],
            parameters: BTreeMap::from([(
                "maximum-distance".into(),
                DimensionedValue::new(1.into(), QuantityDimension::Length, "mm"),
            )]),
        });
        resolved_parts.push(ResolvedPartEvidence {
            circuit: CircuitId::new("McuBoard").unwrap(),
            instance: id,
            class: PartClass::Capacitor,
            selection: PartSelectionIntent::BareValue(DimensionedValue::new(
                100.into(),
                QuantityDimension::Capacitance,
                "nF",
            )),
            resolved: cap_part.clone(),
            artifact_digest: Some("sha256:c-100nf-0402-fixture".into()),
            policy: BTreeMap::from([("package".into(), "0402".into())]),
            attributes: BTreeMap::new(),
        });
    }
    let intent = DesignIntent {
        origins: vec![origin("McuBoard", "rp2354a-board", 1)],
        nets: vec![
            net_intent(
                "McuBoard",
                "p5v",
                NetKind::PowerSupply,
                NetScope::Global("p5v".into()),
            ),
            net_intent(
                "McuBoard",
                "p3v3",
                NetKind::PowerSupply,
                NetScope::Global("p3v3".into()),
            ),
            net_intent(
                "McuBoard",
                "gnd",
                NetKind::Ground,
                NetScope::Global("gnd".into()),
            ),
        ],
        roles,
        resolved_parts,
    };
    assert_round_trip_and_erc(
        CircuitLibrary {
            root: board.id.clone(),
            circuits: vec![board, buck],
        },
        intent,
    );
}
