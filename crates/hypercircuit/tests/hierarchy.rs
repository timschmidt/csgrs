use hypercircuit::{
    AdapterKind, Bus, BusId, BusSlice, BusSliceId, BusSliceOrder, Circuit, CircuitId,
    CircuitInstance, CircuitInstanceId, CircuitLibrary, CircuitLibraryValidationIssue,
    CircuitModuleParameter, CircuitModuleParameterOverride, CircuitModuleParameterTarget,
    CircuitParameter, CircuitPort, CircuitValidationIssue, ComponentId, DeviceModel, DeviceModelId,
    DeviceModelKind, DevicePin, LinearStamp, Net, NetId, PinBinding, PinElectricalKind, PinRef,
    PortDirection, PortId, RailIntent, RailKind, Real, SourceStimulus, SourceWaveform,
    SubcircuitInstance, SubcircuitInstanceId, SubcircuitPortBinding, TransientPolicy,
};

fn empty(id: &str) -> Circuit {
    Circuit::new(
        CircuitId::new(id).unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
}

fn resistor_module() -> Circuit {
    let signal = NetId::new("signal").unwrap();
    let model = DeviceModelId::new("resistor").unwrap();
    empty("resistor-module")
        .with_net(Net {
            id: signal.clone(),
            is_ground: false,
        })
        .with_port(CircuitPort {
            id: PortId::new("signal").unwrap(),
            net: signal.clone(),
            direction: PortDirection::Passive,
            optional: false,
        })
        .with_bus(Bus {
            id: BusId::new("signal-bus").unwrap(),
            nets: vec![signal.clone()],
        })
        .with_bus_slice(BusSlice {
            id: BusSliceId::new("signal-slice").unwrap(),
            bus: BusId::new("signal-bus").unwrap(),
            offset: 0,
            width: 1,
            order: BusSliceOrder::Forward,
        })
        .with_rail(RailIntent {
            net: signal.clone(),
            nominal_voltage: Some(Real::from(3)),
            max_current: Some(Real::one()),
            kind: RailKind::Reference,
        })
        .with_device_model(DeviceModel {
            id: model.clone(),
            kind: DeviceModelKind::Resistor,
            pins: vec![DevicePin {
                pin: PinRef::new("1").unwrap(),
                kind: PinElectricalKind::Passive,
                optional: false,
            }],
            parameters: Vec::new(),
        })
        .with_instance(CircuitInstance {
            id: CircuitInstanceId::new("R").unwrap(),
            component: ComponentId::new("R").unwrap(),
            part: None,
            model,
            pins: vec![PinBinding {
                pin: PinRef::new("1").unwrap(),
                net: signal.clone(),
            }],
            parameters: Vec::new(),
        })
        .with_stamp(LinearStamp::Conductance {
            component: ComponentId::new("R").unwrap(),
            part: None,
            pos: Some(signal),
            neg: None,
            conductance: Real::from(2),
        })
}

fn child_instance(id: &str, net: &NetId) -> SubcircuitInstance {
    SubcircuitInstance {
        id: SubcircuitInstanceId::new(id).unwrap(),
        circuit: CircuitId::new("resistor-module").unwrap(),
        ports: vec![SubcircuitPortBinding {
            port: PortId::new("signal").unwrap(),
            net: net.clone(),
        }],
        parameter_overrides: Vec::new(),
    }
}

#[test]
fn reusable_modules_flatten_with_namespaced_ids_and_remapped_mna_stamps() {
    let output = NetId::new("OUT").unwrap();
    let root = empty("root")
        .with_net(Net {
            id: output.clone(),
            is_ground: false,
        })
        .with_subcircuit(child_instance("load-a", &output))
        .with_subcircuit(child_instance("load-b", &output));
    let library = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, resistor_module()],
    };

    assert!(library.validate().is_valid());
    let flattened = library.flatten().unwrap();
    assert!(flattened.subcircuits.is_empty());
    assert_eq!(flattened.instances.len(), 2);
    assert_eq!(flattened.instances[0].id.as_str(), "load-a/R");
    assert_eq!(flattened.instances[1].id.as_str(), "load-b/R");
    assert_eq!(flattened.stamps.len(), 2);
    assert_eq!(flattened.buses.len(), 2);
    assert_eq!(flattened.buses[0].id.as_str(), "load-a/signal-bus");
    assert_eq!(flattened.bus_slices.len(), 2);
    assert_eq!(flattened.bus_slices[1].id.as_str(), "load-b/signal-slice");
    assert_eq!(flattened.rails.len(), 1);
    assert_eq!(flattened.rails[0].net, output);

    let mna = flattened.linear_mna_system().unwrap();
    assert_eq!(mna.matrix, vec![vec![Real::from(4)]]);
}

#[test]
fn hierarchy_validation_rejects_missing_ports_and_recursive_definitions() {
    let output = NetId::new("OUT").unwrap();
    let root = empty("root")
        .with_net(Net {
            id: output,
            is_ground: false,
        })
        .with_subcircuit(SubcircuitInstance {
            id: SubcircuitInstanceId::new("missing-binding").unwrap(),
            circuit: CircuitId::new("resistor-module").unwrap(),
            ports: Vec::new(),
            parameter_overrides: Vec::new(),
        });
    let missing = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, resistor_module()],
    }
    .validate();
    assert!(missing.issues.iter().any(|issue| matches!(
        issue,
        CircuitLibraryValidationIssue::MissingChildPort { .. }
    )));

    let a = empty("a").with_subcircuit(SubcircuitInstance {
        id: SubcircuitInstanceId::new("b1").unwrap(),
        circuit: CircuitId::new("b").unwrap(),
        ports: Vec::new(),
        parameter_overrides: Vec::new(),
    });
    let b = empty("b").with_subcircuit(SubcircuitInstance {
        id: SubcircuitInstanceId::new("a1").unwrap(),
        circuit: CircuitId::new("a").unwrap(),
        ports: Vec::new(),
        parameter_overrides: Vec::new(),
    });
    let recursive = CircuitLibrary {
        root: a.id.clone(),
        circuits: vec![a, b],
    }
    .validate();
    assert!(
        recursive
            .issues
            .iter()
            .any(|issue| matches!(issue, CircuitLibraryValidationIssue::RecursiveCycle(_)))
    );
}

#[test]
fn flattening_exposes_scope_maps_and_namespaces_source_stimuli() {
    let output = NetId::new("OUT").unwrap();
    let reference = NetId::new("REF").unwrap();
    let model = DeviceModelId::new("current-source").unwrap();
    let source = empty("source-module")
        .with_net(Net {
            id: output.clone(),
            is_ground: false,
        })
        .with_net(Net {
            id: reference.clone(),
            is_ground: true,
        })
        .with_port(CircuitPort {
            id: PortId::new("out").unwrap(),
            net: output.clone(),
            direction: PortDirection::Output,
            optional: false,
        })
        .with_port(CircuitPort {
            id: PortId::new("ref").unwrap(),
            net: reference.clone(),
            direction: PortDirection::Ground,
            optional: false,
        })
        .with_device_model(DeviceModel {
            id: model.clone(),
            kind: DeviceModelKind::CurrentSource,
            pins: vec![
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
            ],
            parameters: Vec::new(),
        })
        .with_instance(CircuitInstance {
            id: CircuitInstanceId::new("I1").unwrap(),
            component: ComponentId::new("I1").unwrap(),
            part: None,
            model,
            pins: vec![
                PinBinding {
                    pin: PinRef::new("+").unwrap(),
                    net: output,
                },
                PinBinding {
                    pin: PinRef::new("-").unwrap(),
                    net: reference,
                },
            ],
            parameters: Vec::new(),
        })
        .with_source_stimulus(SourceStimulus {
            component: ComponentId::new("I1").unwrap(),
            waveform: SourceWaveform::Constant(Real::one()),
        });
    let root_output = NetId::new("ROOT_OUT").unwrap();
    let ground = NetId::new("GND").unwrap();
    let root = empty("source-root")
        .with_net(Net {
            id: root_output.clone(),
            is_ground: false,
        })
        .with_net(Net {
            id: ground.clone(),
            is_ground: true,
        })
        .with_subcircuit(SubcircuitInstance {
            id: SubcircuitInstanceId::new("source-a").unwrap(),
            circuit: source.id.clone(),
            ports: vec![
                SubcircuitPortBinding {
                    port: PortId::new("out").unwrap(),
                    net: root_output.clone(),
                },
                SubcircuitPortBinding {
                    port: PortId::new("ref").unwrap(),
                    net: ground,
                },
            ],
            parameter_overrides: Vec::new(),
        });
    let report = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, source],
    }
    .flatten_with_scopes()
    .unwrap();
    assert_eq!(report.scopes.len(), 1);
    assert_eq!(report.scopes[0].path[0].as_str(), "source-a");
    assert_eq!(
        report.scopes[0].instances[&CircuitInstanceId::new("I1").unwrap()].as_str(),
        "source-a/I1"
    );
    assert_eq!(
        report.scopes[0].nets[&NetId::new("OUT").unwrap()],
        root_output
    );
    assert_eq!(report.circuit.source_stimuli.len(), 1);
    assert_eq!(
        report.circuit.source_stimuli[0].component.as_str(),
        "source-a/I1"
    );
}

fn parameter(name: &str, value: i64, unit: &str, source: &str) -> CircuitParameter {
    CircuitParameter {
        name: name.into(),
        value: Real::from(value),
        unit: unit.into(),
        source: source.into(),
    }
}

fn resistor_pins() -> Vec<DevicePin> {
    ["+", "-"]
        .into_iter()
        .map(|name| DevicePin {
            pin: PinRef::new(name).unwrap(),
            kind: PinElectricalKind::Passive,
            optional: false,
        })
        .collect()
}

fn parameterized_resistor_module() -> Circuit {
    let output = NetId::new("OUT").unwrap();
    let reference = NetId::new("REF").unwrap();
    let instance_model = DeviceModelId::new("instance-resistor").unwrap();
    let shared_model = DeviceModelId::new("model-resistor").unwrap();
    let pins = |output: &NetId, reference: &NetId| {
        vec![
            PinBinding {
                pin: PinRef::new("+").unwrap(),
                net: output.clone(),
            },
            PinBinding {
                pin: PinRef::new("-").unwrap(),
                net: reference.clone(),
            },
        ]
    };
    empty("parameterized-resistors")
        .with_net(Net {
            id: output.clone(),
            is_ground: false,
        })
        .with_net(Net {
            id: reference.clone(),
            is_ground: true,
        })
        .with_port(CircuitPort {
            id: PortId::new("out").unwrap(),
            net: output.clone(),
            direction: PortDirection::Passive,
            optional: false,
        })
        .with_port(CircuitPort {
            id: PortId::new("ref").unwrap(),
            net: reference.clone(),
            direction: PortDirection::Ground,
            optional: false,
        })
        .with_device_model(DeviceModel {
            id: instance_model.clone(),
            kind: DeviceModelKind::Resistor,
            pins: resistor_pins(),
            parameters: vec![parameter("resistance", 100, "ohm", "model-fallback")],
        })
        .with_device_model(DeviceModel {
            id: shared_model.clone(),
            kind: DeviceModelKind::Resistor,
            pins: resistor_pins(),
            parameters: vec![parameter("resistance", 20, "ohm", "module-default")],
        })
        .with_instance(CircuitInstance {
            id: CircuitInstanceId::new("R_INSTANCE").unwrap(),
            component: ComponentId::new("R_INSTANCE").unwrap(),
            part: None,
            model: instance_model,
            pins: pins(&output, &reference),
            parameters: vec![parameter("resistance", 10, "ohm", "module-default")],
        })
        .with_instance(CircuitInstance {
            id: CircuitInstanceId::new("R_MODEL").unwrap(),
            component: ComponentId::new("R_MODEL").unwrap(),
            part: None,
            model: shared_model.clone(),
            pins: pins(&output, &reference),
            parameters: Vec::new(),
        })
        .with_module_parameter(CircuitModuleParameter {
            name: "series_resistance".into(),
            default: Real::from(10),
            unit: "ohm".into(),
            source: "module-default".into(),
            targets: vec![CircuitModuleParameterTarget::InstanceParameter {
                instance: CircuitInstanceId::new("R_INSTANCE").unwrap(),
                parameter: "resistance".into(),
            }],
        })
        .with_module_parameter(CircuitModuleParameter {
            name: "shunt_resistance".into(),
            default: Real::from(20),
            unit: "ohm".into(),
            source: "module-default".into(),
            targets: vec![CircuitModuleParameterTarget::ModelParameter {
                model: shared_model,
                parameter: "resistance".into(),
            }],
        })
}

fn parameterized_instance(
    id: &str,
    circuit: &str,
    output: &NetId,
    ground: &NetId,
    overrides: Vec<CircuitModuleParameterOverride>,
) -> SubcircuitInstance {
    SubcircuitInstance {
        id: SubcircuitInstanceId::new(id).unwrap(),
        circuit: CircuitId::new(circuit).unwrap(),
        ports: vec![
            SubcircuitPortBinding {
                port: PortId::new("out").unwrap(),
                net: output.clone(),
            },
            SubcircuitPortBinding {
                port: PortId::new("ref").unwrap(),
                net: ground.clone(),
            },
        ],
        parameter_overrides: overrides,
    }
}

#[test]
fn exact_module_overrides_apply_to_instance_and_model_parameters_before_mna() {
    let output = NetId::new("OUT").unwrap();
    let ground = NetId::new("GND").unwrap();
    let root = empty("parameter-root")
        .with_net(Net {
            id: output.clone(),
            is_ground: false,
        })
        .with_net(Net {
            id: ground.clone(),
            is_ground: true,
        })
        .with_subcircuit(parameterized_instance(
            "custom",
            "parameterized-resistors",
            &output,
            &ground,
            vec![
                CircuitModuleParameterOverride {
                    parameter: "series_resistance".into(),
                    value: Real::from(2),
                    source: "root.custom.series".into(),
                },
                CircuitModuleParameterOverride {
                    parameter: "shunt_resistance".into(),
                    value: Real::from(4),
                    source: "root.custom.shunt".into(),
                },
            ],
        ))
        .with_subcircuit(parameterized_instance(
            "defaulted",
            "parameterized-resistors",
            &output,
            &ground,
            Vec::new(),
        ));
    let flattened = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, parameterized_resistor_module()],
    }
    .flatten()
    .unwrap();
    let custom = flattened
        .instances
        .iter()
        .find(|instance| instance.id.as_str() == "custom/R_INSTANCE")
        .unwrap();
    assert_eq!(custom.parameters[0].value, Real::from(2));
    assert_eq!(custom.parameters[0].source, "root.custom.series");
    let defaulted = flattened
        .instances
        .iter()
        .find(|instance| instance.id.as_str() == "defaulted/R_INSTANCE")
        .unwrap();
    assert_eq!(defaulted.parameters[0].value, Real::from(10));
    assert_eq!(defaulted.parameters[0].source, "module-default");
    let custom_model = flattened
        .device_models
        .iter()
        .find(|model| model.id.as_str() == "custom/model-resistor")
        .unwrap();
    assert_eq!(custom_model.parameters[0].value, Real::from(4));
    assert_eq!(custom_model.parameters[0].source, "root.custom.shunt");

    let system = flattened.linear_mna_from_devices().unwrap();
    assert_eq!(
        system.matrix[0][0],
        (Real::from(9) / Real::from(10)).unwrap()
    );
    let solution = system.solve_exact().unwrap();
    assert_eq!(solution.candidate, vec![Real::zero()]);
    assert!(solution.replay.accepted);
}

#[test]
fn module_parameters_forward_through_nested_subcircuits() {
    let leaf = parameterized_resistor_module();
    let wrapper_output = NetId::new("OUT").unwrap();
    let wrapper_ground = NetId::new("REF").unwrap();
    let wrapper = empty("wrapper")
        .with_net(Net {
            id: wrapper_output.clone(),
            is_ground: false,
        })
        .with_net(Net {
            id: wrapper_ground.clone(),
            is_ground: true,
        })
        .with_port(CircuitPort {
            id: PortId::new("out").unwrap(),
            net: wrapper_output.clone(),
            direction: PortDirection::Passive,
            optional: false,
        })
        .with_port(CircuitPort {
            id: PortId::new("ref").unwrap(),
            net: wrapper_ground.clone(),
            direction: PortDirection::Ground,
            optional: false,
        })
        .with_subcircuit(parameterized_instance(
            "leaf",
            "parameterized-resistors",
            &wrapper_output,
            &wrapper_ground,
            Vec::new(),
        ))
        .with_module_parameter(CircuitModuleParameter {
            name: "forwarded_resistance".into(),
            default: Real::from(7),
            unit: "ohm".into(),
            source: "wrapper-default".into(),
            targets: vec![CircuitModuleParameterTarget::SubcircuitParameter {
                instance: SubcircuitInstanceId::new("leaf").unwrap(),
                parameter: "series_resistance".into(),
            }],
        });
    let output = NetId::new("OUT").unwrap();
    let ground = NetId::new("GND").unwrap();
    let root = empty("nested-root")
        .with_net(Net {
            id: output.clone(),
            is_ground: false,
        })
        .with_net(Net {
            id: ground.clone(),
            is_ground: true,
        })
        .with_subcircuit(parameterized_instance(
            "wrapper-a",
            "wrapper",
            &output,
            &ground,
            vec![CircuitModuleParameterOverride {
                parameter: "forwarded_resistance".into(),
                value: Real::from(3),
                source: "nested-root.override".into(),
            }],
        ));
    let flattened = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, wrapper, leaf],
    }
    .flatten()
    .unwrap();
    let resistor = flattened
        .instances
        .iter()
        .find(|instance| instance.id.as_str() == "wrapper-a/leaf/R_INSTANCE")
        .unwrap();
    assert_eq!(resistor.parameters[0].value, Real::from(3));
    assert_eq!(resistor.parameters[0].source, "nested-root.override");
}

#[test]
fn parameter_validation_rejects_unknown_units_targets_and_stale_stamps() {
    let mut invalid = parameterized_resistor_module();
    invalid.module_parameters[0].unit = "farad".into();
    invalid.stamps.push(LinearStamp::Conductance {
        component: ComponentId::new("manual").unwrap(),
        part: None,
        pos: Some(NetId::new("OUT").unwrap()),
        neg: Some(NetId::new("REF").unwrap()),
        conductance: Real::one(),
    });
    let report = invalid.validate();
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        CircuitValidationIssue::ModuleParameterUnitMismatch { .. }
    )));
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        CircuitValidationIssue::ModuleParametersWithManualStamps
    )));

    let output = NetId::new("OUT").unwrap();
    let ground = NetId::new("GND").unwrap();
    let root = empty("unknown-override")
        .with_net(Net {
            id: output.clone(),
            is_ground: false,
        })
        .with_net(Net {
            id: ground.clone(),
            is_ground: true,
        })
        .with_subcircuit(parameterized_instance(
            "child",
            "parameterized-resistors",
            &output,
            &ground,
            vec![CircuitModuleParameterOverride {
                parameter: "absent".into(),
                value: Real::one(),
                source: "test".into(),
            }],
        ));
    let report = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, parameterized_resistor_module()],
    }
    .validate();
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        CircuitLibraryValidationIssue::UnknownChildParameter { .. }
    )));
}

#[cfg(feature = "interchange")]
#[test]
fn module_parameter_declarations_and_overrides_round_trip_exactly() {
    let output = NetId::new("OUT").unwrap();
    let ground = NetId::new("GND").unwrap();
    let root = empty("serialized-parameters")
        .with_net(Net {
            id: output.clone(),
            is_ground: false,
        })
        .with_net(Net {
            id: ground.clone(),
            is_ground: true,
        })
        .with_subcircuit(parameterized_instance(
            "child",
            "parameterized-resistors",
            &output,
            &ground,
            vec![CircuitModuleParameterOverride {
                parameter: "series_resistance".into(),
                value: (Real::from(5) / Real::from(3)).unwrap(),
                source: "serialized-test".into(),
            }],
        ));
    let json = serde_json::to_string(&root).unwrap();
    assert_eq!(serde_json::from_str::<Circuit>(&json).unwrap(), root);
    let module = parameterized_resistor_module();
    let json = serde_json::to_string(&module).unwrap();
    assert_eq!(serde_json::from_str::<Circuit>(&json).unwrap(), module);
}
