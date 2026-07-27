#![cfg(feature = "interchange")]

use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, BundleMemberId, Circuit, CircuitEventCause,
    CircuitEventKind, CircuitEventPhase, CircuitEventRequest, CircuitEventTarget, CircuitId,
    CircuitInstance, CircuitInstanceId, CircuitLibrary, ComponentId, DesignIntent, DeviceModel,
    DeviceModelId, DeviceModelKind, DevicePin, DifferentialPair, DifferentialPairId,
    DifferentialPairNeckdown, DimensionedValue, FunctionalBinding, FunctionalBindingTarget,
    FunctionalRole, FunctionalRoleAssignment, FunctionalRoleTarget, LandPattern, LandPatternId,
    LandPatternPad, Net, NetId, NetIntent, NetKind, NetScope, PadId, PadShape, PcbDesignRules,
    PcbLayout, PcbStackup, PinBinding, PinElectricalKind, PinRef, Plating, QuantityDimension, Real,
    SEMANTIC_SCHEMA_VERSION, SchematicBlockPlacement, SchematicBlockSize, SchematicCanvasSettings,
    SchematicConnectionStyle, SchematicEndpoint, SchematicLayout, SchematicPinPlacement,
    SchematicPinSide, SchematicPoint, SchematicPresentation, SchematicSymbol,
    SchematicSymbolDefinition, SchematicSymbolDefinitionId, SchematicSymbolId, SchematicSymbolUnit,
    SchematicWire, SchematicWireId, SchematicWireMetadata, SchematicWireStyle, SemanticDocument,
    SemanticInterchangeError, SemanticMigrationStep, SemanticOrigin, SemanticTarget, SignalBundle,
    SignalBundleId, SignalBundleLibrary, SourcePosition, SourceSpan, SourceStimulus,
    SourceWaveform, StackupLayer, StackupLayerKind, SubcircuitInstance, SubcircuitInstanceId,
    TransientPolicy, TransientRunPolicy,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;

fn fixture() -> SemanticDocument {
    let net = NetId::new("signal").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("json-round-trip").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: net.clone(),
        is_ground: false,
    });
    let schematic = SchematicLayout {
        wires: vec![SchematicWire {
            id: SchematicWireId::new("w1").unwrap(),
            net,
            from: SchematicEndpoint::Junction(SchematicPoint::new(Real::from(1), Real::from(2))),
            waypoints: vec![SchematicPoint::new(Real::from(3), Real::from(5))],
            to: SchematicEndpoint::Junction(SchematicPoint::new(Real::from(8), Real::from(13))),
        }],
        ..SchematicLayout::default()
    };
    let point = |x, y| Point2::new(Real::from(x), Real::from(y));
    let pcb = PcbLayout {
        id: BoardId::new("main-board").unwrap(),
        outline: BoardOutline {
            exterior: vec![point(0, 0), point(40, 0), point(40, 30), point(0, 30)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![StackupLayer {
                name: "F.Cu".into(),
                kind: StackupLayerKind::Conductor(TraceLayer(0)),
                thickness: Real::from(1),
                material: Some("copper".into()),
            }],
        },
        land_patterns: vec![LandPattern {
            id: LandPatternId::new("rotated-pad").unwrap(),
            pads: vec![LandPatternPad {
                id: PadId::new("1").unwrap(),
                center: point(5, 5),
                rotation_degrees: Real::zero(),
                copper_layers: vec![TraceLayer(0)],
                shape: PadShape::Rectangle {
                    width: Real::from(2),
                    height: Real::one(),
                },
                drill: None,
                plating: Plating::Unspecified,
                solder_mask_margin: None,
                paste_margin: None,
            }],
            pin_map: Vec::new(),
            graphics: Vec::new(),
            body: None,
            models: Vec::new(),
        }],
        placements: Vec::new(),
        placement_constraints: Vec::new(),
        routes: Vec::new(),
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };
    SemanticDocument::new(circuit, Some(schematic))
        .unwrap()
        .with_pcb(pcb)
        .unwrap()
}

#[test]
fn exact_authored_event_trace_round_trips_and_validates() {
    let document = fixture()
        .with_event_trace(vec![CircuitEventRequest {
            time: Real::one(),
            phase: CircuitEventPhase::Stimulus,
            source: Some(CircuitEventTarget::External("testbench".into())),
            target: CircuitEventTarget::Circuit(CircuitId::new("json-round-trip").unwrap()),
            kind: CircuitEventKind::Behavioral {
                kind: "start".into(),
                fields: Default::default(),
            },
            cause: CircuitEventCause::Authored {
                provenance: "interchange-test".into(),
            },
        }])
        .unwrap();
    let restored = SemanticDocument::from_json(&document.to_json_pretty().unwrap()).unwrap();
    assert_eq!(restored.event_trace, document.event_trace);
    assert_eq!(restored.version, SEMANTIC_SCHEMA_VERSION);
}

#[test]
fn versioned_semantic_json_round_trips_exact_values() {
    let document = fixture();
    let json = document.to_json_pretty().unwrap();
    assert!(json.contains("org.hypercircuit.semantic"));
    let decoded = SemanticDocument::from_json(&json).unwrap();
    assert_eq!(decoded, document);
    assert_eq!(decoded.pcb.unwrap().id.as_str(), "main-board");
}

#[test]
fn authored_coppertrace_intent_round_trips_with_source_ranges_and_contracts() {
    let circuit = CircuitId::new("json-round-trip").unwrap();
    let net = NetId::new("signal").unwrap();
    let document = fixture()
        .with_design_intent(DesignIntent {
            origins: vec![SemanticOrigin {
                target: SemanticTarget::Net {
                    circuit: circuit.clone(),
                    net: net.clone(),
                },
                span: SourceSpan::new(
                    "filter.trace",
                    SourcePosition::new(4, 1, 5),
                    SourcePosition::new(21, 1, 22),
                ),
                label: Some("net declaration".into()),
            }],
            nets: vec![NetIntent {
                circuit: circuit.clone(),
                net: net.clone(),
                kind: NetKind::AnalogSignal,
                scope: NetScope::Local,
                net_class: None,
                nominal_value: None,
            }],
            roles: vec![FunctionalRoleAssignment {
                target: FunctionalRoleTarget::Circuit(circuit.clone()),
                role: FunctionalRole::LowpassFilter,
                bindings: vec![FunctionalBinding {
                    name: "output".into(),
                    target: FunctionalBindingTarget::Net { circuit, net },
                }],
                parameters: std::collections::BTreeMap::from([(
                    "cutoff".into(),
                    DimensionedValue::new(Real::from(1000), QuantityDimension::Frequency, "Hz"),
                )]),
            }],
            resolved_parts: Vec::new(),
        })
        .unwrap();

    let restored = SemanticDocument::from_json(&document.to_json_pretty().unwrap()).unwrap();
    assert_eq!(restored.design_intent, document.design_intent);
}

#[test]
fn version_twenty_eight_defaults_authored_design_intent() {
    let document = fixture();
    let mut value = serde_json::to_value(&document).unwrap();
    value["version"] = serde_json::Value::from(28);
    value.as_object_mut().unwrap().remove("design_intent");

    let (migrated, report) =
        SemanticDocument::from_json_migrating(&serde_json::to_string(&value).unwrap()).unwrap();
    assert_eq!(
        report.steps,
        vec![
            SemanticMigrationStep::AuthoredDesignIntent,
            SemanticMigrationStep::ManufacturingIntent,
        ]
    );
    assert_eq!(migrated, document);
}

#[test]
fn native_hierarchy_simulation_and_wire_presentation_round_trip_together() {
    let mut document = fixture();
    let child = Circuit::new(
        CircuitId::new("reusable-child").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    );
    document.circuit = document.circuit.with_subcircuit(SubcircuitInstance {
        id: SubcircuitInstanceId::new("child").unwrap(),
        circuit: child.id.clone(),
        ports: Vec::new(),
        parameter_overrides: Vec::new(),
    });
    let library = CircuitLibrary {
        root: document.circuit.id.clone(),
        circuits: vec![document.circuit.clone(), child],
    };
    let presentation = SchematicPresentation {
        canvas: SchematicCanvasSettings {
            connection_style: SchematicConnectionStyle::Orthogonal,
            grid_spacing: Some(Real::from(10)),
            snap_to_grid: true,
            show_grid: true,
            paper_size: Some("A3".into()),
            ..SchematicCanvasSettings::default()
        },
        wires: vec![SchematicWireMetadata {
            wire: SchematicWireId::new("w1").unwrap(),
            name: Some("command".into()),
            color: Some("#3366ff".into()),
            width: Some(Real::one()),
            style: SchematicWireStyle::Dashed,
            logging: true,
            manual_route: true,
            show_name: true,
        }],
        blocks: vec![SchematicBlockPlacement {
            instance: SubcircuitInstanceId::new("child").unwrap(),
            position: SchematicPoint::new(Real::from(20), Real::from(30)),
            size: Some(SchematicBlockSize {
                width: Real::from(12),
                height: Real::from(8),
            }),
            icon: Some("control".into()),
            color: Some("#445566".into()),
            mask: true,
            symbol: Some("controller".into()),
            modal: true,
            show_port_names: true,
        }],
    };
    let document = document
        .with_circuit_library(library)
        .unwrap()
        .with_event_trace(vec![CircuitEventRequest {
            time: Real::zero(),
            phase: CircuitEventPhase::Stimulus,
            source: None,
            target: CircuitEventTarget::Hierarchy(vec![
                SubcircuitInstanceId::new("child").unwrap(),
            ]),
            kind: CircuitEventKind::Behavioral {
                kind: "initialize".into(),
                fields: Default::default(),
            },
            cause: CircuitEventCause::Authored {
                provenance: "native-parity".into(),
            },
        }])
        .unwrap()
        .with_transient_run(TransientRunPolicy::default())
        .unwrap()
        .with_schematic_presentation(presentation)
        .unwrap();
    let restored = SemanticDocument::from_json(&document.to_json_pretty().unwrap()).unwrap();

    assert_eq!(restored, document);
    assert_eq!(restored.circuit_library().circuits.len(), 2);
    assert!(restored.transient_run.is_some());
    assert!(restored.schematic_presentation.wires[0].logging);
    assert!(restored.schematic_presentation.blocks[0].show_port_names);
}

#[test]
fn invalid_canvas_and_wire_presentation_is_rejected_at_interchange_boundary() {
    let presentation = SchematicPresentation {
        canvas: SchematicCanvasSettings {
            grid_spacing: Some(Real::zero()),
            ..SchematicCanvasSettings::default()
        },
        wires: vec![SchematicWireMetadata {
            wire: SchematicWireId::new("absent").unwrap(),
            name: Some(String::new()),
            color: None,
            width: Some(Real::zero()),
            style: SchematicWireStyle::Solid,
            logging: false,
            manual_route: false,
            show_name: false,
        }],
        blocks: vec![SchematicBlockPlacement {
            instance: SubcircuitInstanceId::new("absent").unwrap(),
            position: SchematicPoint::new(Real::zero(), Real::zero()),
            size: Some(SchematicBlockSize {
                width: Real::zero(),
                height: Real::one(),
            }),
            icon: Some(String::new()),
            color: None,
            mask: false,
            symbol: None,
            modal: false,
            show_port_names: false,
        }],
    };
    assert!(matches!(
        fixture().with_schematic_presentation(presentation),
        Err(SemanticInterchangeError::InvalidSchematicPresentation { issue_count: 7 })
    ));
}

#[test]
fn version_twenty_seven_promotes_legacy_bundle_member_names_to_real_types() {
    let document = fixture()
        .with_signal_bundles(SignalBundleLibrary::new().with_bundle(SignalBundle::new(
            SignalBundleId::new("Status").unwrap(),
            vec![BundleMemberId::new("ready").unwrap()],
        )))
        .unwrap();
    let mut value = serde_json::to_value(&document).unwrap();
    value["version"] = serde_json::Value::from(27);
    value["signal_bundles"]["bundles"][0]["members"] = serde_json::json!(["ready"]);
    value.as_object_mut().unwrap().remove("circuit_definitions");
    value.as_object_mut().unwrap().remove("transient_run");
    value
        .as_object_mut()
        .unwrap()
        .remove("schematic_presentation");
    value.as_object_mut().unwrap().remove("design_intent");

    let (migrated, report) =
        SemanticDocument::from_json_migrating(&serde_json::to_string(&value).unwrap()).unwrap();
    assert_eq!(
        report.steps,
        vec![
            SemanticMigrationStep::NativeInterfaceParity,
            SemanticMigrationStep::AuthoredDesignIntent,
            SemanticMigrationStep::ManufacturingIntent,
        ]
    );
    assert_eq!(
        migrated.signal_bundles.bundles[0].members[0].signal_type,
        hypercircuit::PortSignalType::Real
    );
}

#[test]
fn unknown_schema_versions_are_rejected() {
    let mut document = fixture();
    document.version += 1;
    assert!(matches!(
        document.to_json_pretty(),
        Err(SemanticInterchangeError::UnsupportedSchema { .. })
    ));
}

#[test]
fn version_eight_json_migrates_through_each_additive_schema_boundary() {
    let document = fixture();
    let mut value = serde_json::to_value(&document).unwrap();
    let legacy_exterior = value["pcb"]["outline"]["exterior"]
        .as_array()
        .unwrap()
        .iter()
        .map(|segment| segment["start"].clone())
        .collect();
    value["pcb"]["outline"]["exterior"] = serde_json::Value::Array(legacy_exterior);
    value["version"] = serde_json::Value::from(8);
    value["circuit"]
        .as_object_mut()
        .unwrap()
        .remove("module_parameters");
    value.as_object_mut().unwrap().remove("design_intent");
    let rules = value["pcb"]["rules"].as_object_mut().unwrap();
    rules.remove("route_constraint_regions");
    rules.remove("route_rule_regions");
    rules.remove("escape_policies");
    rules.remove("length_tuning_patterns");
    rules.remove("via_styles");
    value["pcb"]["land_patterns"][0]["pads"][0]
        .as_object_mut()
        .unwrap()
        .remove("rotation_degrees");
    let json = serde_json::to_string(&value).unwrap();

    let (migrated, report) = SemanticDocument::from_json_migrating(&json).unwrap();
    assert_eq!(report.from_version, 8);
    assert_eq!(report.to_version, SEMANTIC_SCHEMA_VERSION);
    assert_eq!(
        report.steps,
        vec![
            SemanticMigrationStep::ModuleParameters,
            SemanticMigrationStep::AdvancedRoutingIntent,
            SemanticMigrationStep::DiodeModels,
            SemanticMigrationStep::ViaStyles,
            SemanticMigrationStep::NetClassInheritance,
            SemanticMigrationStep::RegionalRouteRules,
            SemanticMigrationStep::MixedCurveBoardContours,
            SemanticMigrationStep::PeriodicSourceWaveforms,
            SemanticMigrationStep::AnalyticSourceWaveforms,
            SemanticMigrationStep::MosfetModels,
            SemanticMigrationStep::PreferredTraceWidths,
            SemanticMigrationStep::ReusableSchematicSymbols,
            SemanticMigrationStep::ExternalPcb3dModels,
            SemanticMigrationStep::IndependentPcb3dModels,
            SemanticMigrationStep::PadLocalRotations,
            SemanticMigrationStep::DifferentialPairImpedance,
            SemanticMigrationStep::PhaseTuningGroups,
            SemanticMigrationStep::DifferentialPairNeckdown,
            SemanticMigrationStep::MixedSignalWorkflow,
            SemanticMigrationStep::NativeInterfaceParity,
            SemanticMigrationStep::AuthoredDesignIntent,
            SemanticMigrationStep::ManufacturingIntent,
        ]
    );
    assert_eq!(migrated, document);
    assert_eq!(SemanticDocument::from_json(&json).unwrap(), document);
}

#[test]
fn versions_outside_the_migration_window_are_rejected() {
    for version in [7, SEMANTIC_SCHEMA_VERSION + 1] {
        let mut value = serde_json::to_value(fixture()).unwrap();
        value["version"] = serde_json::Value::from(version);
        let error =
            SemanticDocument::from_json(&serde_json::to_string(&value).unwrap()).unwrap_err();
        assert!(matches!(
            error,
            SemanticInterchangeError::UnsupportedSchema {
                version: rejected,
                ..
            } if rejected == version
        ));
    }
}

#[test]
fn version_twenty_three_defaults_new_differential_impedance_intent() {
    let mut document = fixture();
    let negative = NetId::new("signal-negative").unwrap();
    document.circuit.nets.push(Net {
        id: negative.clone(),
        is_ground: false,
    });
    document
        .pcb
        .as_mut()
        .unwrap()
        .rules
        .differential_pairs
        .push(DifferentialPair {
            id: DifferentialPairId::new("legacy-pair").unwrap(),
            positive: NetId::new("signal").unwrap(),
            negative,
            spacing: Real::one(),
            max_skew: None,
            target_impedance_ohms: None,
            impedance_tolerance_ohms: None,
            neckdown: None,
        });
    let mut value = serde_json::to_value(&document).unwrap();
    value["version"] = serde_json::Value::from(23);
    let pair = value["pcb"]["rules"]["differential_pairs"][0]
        .as_object_mut()
        .unwrap();
    pair.remove("target_impedance_ohms");
    pair.remove("impedance_tolerance_ohms");

    let (migrated, report) =
        SemanticDocument::from_json_migrating(&serde_json::to_string(&value).unwrap()).unwrap();
    assert_eq!(
        report.steps,
        vec![
            SemanticMigrationStep::DifferentialPairImpedance,
            SemanticMigrationStep::PhaseTuningGroups,
            SemanticMigrationStep::DifferentialPairNeckdown,
            SemanticMigrationStep::MixedSignalWorkflow,
            SemanticMigrationStep::NativeInterfaceParity,
            SemanticMigrationStep::AuthoredDesignIntent,
            SemanticMigrationStep::ManufacturingIntent,
        ]
    );
    let pair = &migrated.pcb.unwrap().rules.differential_pairs[0];
    assert_eq!(pair.target_impedance_ohms, None);
    assert_eq!(pair.impedance_tolerance_ohms, None);
}

#[test]
fn version_twenty_four_defaults_new_phase_tuning_groups() {
    let document = fixture();
    let mut value = serde_json::to_value(&document).unwrap();
    value["version"] = serde_json::Value::from(24);
    value["pcb"]["rules"]
        .as_object_mut()
        .unwrap()
        .remove("phase_tuning_groups");

    let (migrated, report) =
        SemanticDocument::from_json_migrating(&serde_json::to_string(&value).unwrap()).unwrap();
    assert_eq!(
        report.steps,
        vec![
            SemanticMigrationStep::PhaseTuningGroups,
            SemanticMigrationStep::DifferentialPairNeckdown,
            SemanticMigrationStep::MixedSignalWorkflow,
            SemanticMigrationStep::NativeInterfaceParity,
            SemanticMigrationStep::AuthoredDesignIntent,
            SemanticMigrationStep::ManufacturingIntent,
        ]
    );
    assert!(migrated.pcb.unwrap().rules.phase_tuning_groups.is_empty());
}

#[test]
fn version_twenty_five_defaults_new_differential_pair_neckdown() {
    let mut document = fixture();
    let negative = NetId::new("signal-negative").unwrap();
    document.circuit.nets.push(Net {
        id: negative.clone(),
        is_ground: false,
    });
    document
        .pcb
        .as_mut()
        .unwrap()
        .rules
        .differential_pairs
        .push(DifferentialPair {
            id: DifferentialPairId::new("neckdown-pair").unwrap(),
            positive: NetId::new("signal").unwrap(),
            negative,
            spacing: Real::from(2),
            max_skew: None,
            target_impedance_ohms: None,
            impedance_tolerance_ohms: None,
            neckdown: Some(DifferentialPairNeckdown {
                trace_width: (Real::one() / Real::from(2)).unwrap(),
                spacing: Real::one(),
                maximum_transition_length: Real::from(3),
            }),
        });
    assert_eq!(
        SemanticDocument::from_json(&document.to_json_pretty().unwrap()).unwrap(),
        document
    );
    let mut value = serde_json::to_value(&document).unwrap();
    value["version"] = serde_json::Value::from(25);
    value["pcb"]["rules"]["differential_pairs"][0]
        .as_object_mut()
        .unwrap()
        .remove("neckdown");
    let (migrated, report) =
        SemanticDocument::from_json_migrating(&serde_json::to_string(&value).unwrap()).unwrap();
    assert_eq!(
        report.steps,
        vec![
            SemanticMigrationStep::DifferentialPairNeckdown,
            SemanticMigrationStep::MixedSignalWorkflow,
            SemanticMigrationStep::NativeInterfaceParity,
            SemanticMigrationStep::AuthoredDesignIntent,
            SemanticMigrationStep::ManufacturingIntent,
        ]
    );
    assert!(
        migrated.pcb.unwrap().rules.differential_pairs[0]
            .neckdown
            .is_none()
    );
}

#[test]
fn version_nineteen_promotes_embedded_symbol_geometry_into_a_library() {
    let net = NetId::new("signal").unwrap();
    let model = DeviceModelId::new("buffer").unwrap();
    let pin = PinRef::new("A").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("legacy-symbol").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: net.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: model.clone(),
        kind: DeviceModelKind::Custom("buffer".into()),
        pins: vec![DevicePin {
            pin: pin.clone(),
            kind: PinElectricalKind::Input,
            optional: false,
        }],
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("U1").unwrap(),
        component: ComponentId::new("U1").unwrap(),
        part: None,
        model: model.clone(),
        pins: vec![PinBinding {
            pin: pin.clone(),
            net,
        }],
        parameters: Vec::new(),
    });
    let definition = SchematicSymbolDefinitionId::new("buffer-symbol").unwrap();
    let document = SemanticDocument::new(
        circuit,
        Some(SchematicLayout {
            symbol_definitions: vec![SchematicSymbolDefinition {
                id: definition.clone(),
                model,
                name: "Buffer".into(),
                units: vec![SchematicSymbolUnit {
                    unit: 1,
                    body_width: Real::from(10),
                    body_height: Real::from(8),
                    pins: vec![SchematicPinPlacement {
                        pin,
                        position: SchematicPoint::new(Real::from(-6), Real::zero()),
                        side: SchematicPinSide::Left,
                    }],
                    graphics: Vec::new(),
                }],
            }],
            symbols: vec![SchematicSymbol {
                id: SchematicSymbolId::new("U1:A").unwrap(),
                instance: CircuitInstanceId::new("U1").unwrap(),
                definition,
                unit: 1,
                position: SchematicPoint::new(Real::zero(), Real::zero()),
                quarter_turns: 0,
            }],
            ..SchematicLayout::default()
        }),
    )
    .unwrap();
    let mut value = serde_json::to_value(document).unwrap();
    let unit = value["schematic"]["symbol_definitions"][0]["units"][0].clone();
    let symbol = value["schematic"]["symbols"][0].as_object_mut().unwrap();
    symbol.remove("definition");
    symbol.insert("body_width".into(), unit["body_width"].clone());
    symbol.insert("body_height".into(), unit["body_height"].clone());
    symbol.insert("pins".into(), unit["pins"].clone());
    value["schematic"]
        .as_object_mut()
        .unwrap()
        .remove("symbol_definitions");
    value["version"] = serde_json::Value::from(19);

    let (migrated, report) =
        SemanticDocument::from_json_migrating(&serde_json::to_string(&value).unwrap()).unwrap();
    assert_eq!(
        report.steps,
        vec![
            SemanticMigrationStep::ReusableSchematicSymbols,
            SemanticMigrationStep::ExternalPcb3dModels,
            SemanticMigrationStep::IndependentPcb3dModels,
            SemanticMigrationStep::PadLocalRotations,
            SemanticMigrationStep::DifferentialPairImpedance,
            SemanticMigrationStep::PhaseTuningGroups,
            SemanticMigrationStep::DifferentialPairNeckdown,
            SemanticMigrationStep::MixedSignalWorkflow,
            SemanticMigrationStep::NativeInterfaceParity,
            SemanticMigrationStep::AuthoredDesignIntent,
            SemanticMigrationStep::ManufacturingIntent,
        ]
    );
    let schematic = migrated.schematic.unwrap();
    assert_eq!(schematic.symbol_definitions.len(), 1);
    assert_eq!(
        schematic.symbols[0].definition,
        schematic.symbol_definitions[0].id
    );
    assert_eq!(schematic.symbol_definitions[0].units[0].pins.len(), 1);
}

#[test]
fn decoded_schematic_must_agree_with_decoded_circuit() {
    let document = fixture();
    let mut value = serde_json::to_value(document).unwrap();
    value["schematic"]["wires"][0]["net"] = serde_json::Value::String("absent".into());
    let json = serde_json::to_string(&value).unwrap();
    assert!(matches!(
        SemanticDocument::from_json(&json),
        Err(SemanticInterchangeError::InvalidSchematic { .. })
    ));
}

#[test]
fn decoded_pcb_must_pass_structural_validation() {
    let document = fixture();
    let mut value = serde_json::to_value(document).unwrap();
    value["pcb"]["outline"]["exterior"] = serde_json::Value::Array(Vec::new());
    let json = serde_json::to_string(&value).unwrap();
    assert!(matches!(
        SemanticDocument::from_json(&json),
        Err(SemanticInterchangeError::InvalidPcb { .. })
    ));
}

#[test]
fn exact_source_stimuli_round_trip_through_semantic_json() {
    let ground = NetId::new("GND").unwrap();
    let output = NetId::new("OUT").unwrap();
    let model = DeviceModelId::new("current-source").unwrap();
    let component = ComponentId::new("I1").unwrap();
    let pins = ["+", "-"]
        .into_iter()
        .map(|name| DevicePin {
            pin: PinRef::new(name).unwrap(),
            kind: PinElectricalKind::Passive,
            optional: false,
        })
        .collect::<Vec<_>>();
    let circuit = Circuit::new(
        CircuitId::new("stimulus-round-trip").unwrap(),
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
        id: model.clone(),
        kind: DeviceModelKind::CurrentSource,
        pins,
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("I1").unwrap(),
        component: component.clone(),
        part: None,
        model,
        pins: vec![
            PinBinding {
                pin: PinRef::new("+").unwrap(),
                net: output,
            },
            PinBinding {
                pin: PinRef::new("-").unwrap(),
                net: ground,
            },
        ],
        parameters: Vec::new(),
    })
    .with_source_stimulus(SourceStimulus {
        component,
        waveform: SourceWaveform::Pulse {
            low_value: Real::zero(),
            high_value: (Real::from(2) / Real::from(7)).unwrap(),
            delay: (Real::one() / Real::from(3)).unwrap(),
            rise_time: (Real::one() / Real::from(11)).unwrap(),
            high_time: (Real::one() / Real::from(5)).unwrap(),
            fall_time: (Real::one() / Real::from(13)).unwrap(),
            period: Real::one(),
        },
    });
    let document = SemanticDocument::new(circuit, None).unwrap();
    let json = document.to_json_pretty().unwrap();
    assert!(json.contains("\"source_stimuli\""));
    assert_eq!(SemanticDocument::from_json(&json).unwrap(), document);

    for waveform in [
        SourceWaveform::Sine {
            offset: Real::one(),
            amplitude: (Real::from(2) / Real::from(7)).unwrap(),
            frequency: (Real::one() / Real::from(11)).unwrap(),
            delay: (Real::one() / Real::from(13)).unwrap(),
            damping: (Real::one() / Real::from(17)).unwrap(),
            phase_degrees: Real::from(30),
        },
        SourceWaveform::Exponential {
            initial: Real::zero(),
            pulsed: Real::one(),
            rise_delay: (Real::one() / Real::from(3)).unwrap(),
            rise_time_constant: (Real::one() / Real::from(5)).unwrap(),
            fall_delay: (Real::from(2) / Real::from(3)).unwrap(),
            fall_time_constant: (Real::one() / Real::from(7)).unwrap(),
        },
    ] {
        let json = serde_json::to_string(&waveform).unwrap();
        assert_eq!(
            serde_json::from_str::<SourceWaveform>(&json).unwrap(),
            waveform
        );
    }
}
