use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitLibrary,
    ComponentId, CopperTraceErcIssue, DesignIntent, DeviceModel, DeviceModelId, DeviceModelKind,
    DevicePin, FunctionalBinding, FunctionalBindingTarget, FunctionalRole,
    FunctionalRoleAssignment, FunctionalRoleTarget, Net, NetId, NetIntent, NetKind, NetScope,
    PinBinding, PinElectricalKind, PinRef, SubcircuitInstance, SubcircuitInstanceId,
    TransientPolicy,
};

fn empty(id: &str) -> Circuit {
    Circuit::new(
        CircuitId::new(id).unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
}

fn global_source_library(source_instances: usize) -> (CircuitLibrary, DesignIntent) {
    let source_net = NetId::new("OUT").unwrap();
    let source_model = DeviceModelId::new("regulator").unwrap();
    let source = empty("source")
        .with_net(Net {
            id: source_net.clone(),
            is_ground: false,
        })
        .with_device_model(DeviceModel {
            id: source_model.clone(),
            kind: DeviceModelKind::Custom("regulator".into()),
            pins: vec![DevicePin {
                pin: PinRef::new("OUT").unwrap(),
                kind: PinElectricalKind::PowerOutput,
                optional: false,
            }],
            parameters: Vec::new(),
        })
        .with_instance(CircuitInstance {
            id: CircuitInstanceId::new("U").unwrap(),
            component: ComponentId::new("U").unwrap(),
            part: None,
            model: source_model,
            pins: vec![PinBinding {
                pin: PinRef::new("OUT").unwrap(),
                net: source_net.clone(),
            }],
            parameters: Vec::new(),
        });
    let root_net = NetId::new("VCC").unwrap();
    let mut root = empty("root").with_net(Net {
        id: root_net.clone(),
        is_ground: false,
    });
    for index in 0..source_instances {
        root = root.with_subcircuit(SubcircuitInstance {
            id: SubcircuitInstanceId::new(format!("source-{index}")).unwrap(),
            circuit: source.id.clone(),
            ports: Vec::new(),
            parameter_overrides: Vec::new(),
        });
    }
    let library = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, source],
    };
    let intent = DesignIntent {
        nets: vec![
            NetIntent {
                circuit: CircuitId::new("root").unwrap(),
                net: root_net,
                kind: NetKind::PowerSupply,
                scope: NetScope::Global("VCC".into()),
                net_class: None,
                nominal_value: None,
            },
            NetIntent {
                circuit: CircuitId::new("source").unwrap(),
                net: source_net,
                kind: NetKind::PowerSupply,
                scope: NetScope::Global("VCC".into()),
                net_class: None,
                nominal_value: None,
            },
        ],
        ..DesignIntent::default()
    };
    (library, intent)
}

#[test]
fn global_power_supply_requires_exactly_one_elaborated_source() {
    let (one, one_intent) = global_source_library(1);
    assert!(one.coppertrace_erc(&one_intent).is_valid());

    let (zero, zero_intent) = global_source_library(0);
    assert!(
        zero.coppertrace_erc(&zero_intent)
            .issues
            .iter()
            .any(|issue| matches!(
                issue,
                CopperTraceErcIssue::PowerSupplySourceCount { sources, .. }
                    if sources.is_empty()
            ))
    );

    let (two, two_intent) = global_source_library(2);
    assert!(
        two.coppertrace_erc(&two_intent)
            .issues
            .iter()
            .any(|issue| matches!(
                issue,
                CopperTraceErcIssue::PowerSupplySourceCount { sources, .. }
                    if sources.len() == 2
            ))
    );
}

#[test]
fn seed_function_contracts_require_their_declared_interface() {
    let input = NetId::new("IN").unwrap();
    let output = NetId::new("OUT").unwrap();
    let circuit = empty("filter")
        .with_net(Net {
            id: input.clone(),
            is_ground: false,
        })
        .with_net(Net {
            id: output.clone(),
            is_ground: false,
        });
    let library = CircuitLibrary {
        root: circuit.id.clone(),
        circuits: vec![circuit],
    };
    let intent = DesignIntent {
        nets: vec![
            NetIntent {
                circuit: CircuitId::new("filter").unwrap(),
                net: input.clone(),
                kind: NetKind::AnalogSignal,
                scope: NetScope::Local,
                net_class: None,
                nominal_value: None,
            },
            NetIntent {
                circuit: CircuitId::new("filter").unwrap(),
                net: output.clone(),
                kind: NetKind::AnalogSignal,
                scope: NetScope::Local,
                net_class: None,
                nominal_value: None,
            },
        ],
        roles: vec![FunctionalRoleAssignment {
            target: FunctionalRoleTarget::Circuit(CircuitId::new("filter").unwrap()),
            role: FunctionalRole::LowpassFilter,
            bindings: vec![
                FunctionalBinding {
                    name: "sig_in".into(),
                    target: FunctionalBindingTarget::Net {
                        circuit: CircuitId::new("filter").unwrap(),
                        net: input,
                    },
                },
                FunctionalBinding {
                    name: "sig_out".into(),
                    target: FunctionalBindingTarget::Net {
                        circuit: CircuitId::new("filter").unwrap(),
                        net: output,
                    },
                },
            ],
            parameters: Default::default(),
        }],
        ..DesignIntent::default()
    };

    assert!(
        library
            .coppertrace_erc(&intent)
            .issues
            .iter()
            .any(|issue| matches!(
                issue,
                CopperTraceErcIssue::MissingRoleBinding { binding, .. }
                    if binding == "reference"
            ))
    );
}
