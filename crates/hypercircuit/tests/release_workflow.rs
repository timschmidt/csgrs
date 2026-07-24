#![cfg(all(feature = "drc", feature = "interchange"))]

use hypercircuit::{
    AdapterKind, AssemblyOutputs, AssemblyPartOverride, AssemblyVariant, AssemblyVariantId,
    BoardId, BoardOutline, BoardSide, Circuit, CircuitId, CircuitInstance, CircuitInstanceId,
    CircuitParameter, ComponentId, CopperZone, DeviceModel, DeviceModelId, DeviceModelKind,
    DevicePin, DifferentialPair, DrcReadinessPolicy, FabricationPackage, KeepoutId, KeepoutScope,
    KiCadExportOmission, KiCadExportOptions, KiCadImportOptions, KiCadImportReport, LandPattern,
    LandPatternBody, LandPatternGraphic, LandPatternGraphicId, LandPatternGraphicPrimitive,
    LandPatternId, LandPatternPad, LayerRole, MaterializationOptions, Net, NetClass, NetClassId,
    NetId, PadId, PadPinMap, PadShape, PartRef, PcbDesignRules, PcbKeepout, PcbLayout,
    PcbPlacement, PcbRoute, PcbStackup, PcbSvgOptions, PcbVia, PinBinding, PinElectricalKind,
    PinRef, PlacementConstraint, PlacementConstraintId, PlacementConstraintKind,
    PlacementResolutionIssue, Plating, Real, RouteId, RoutingProblemReport, RoutingSolution,
    SchematicEndpoint, SchematicLabel, SchematicLabelId, SchematicLayout, SchematicPinPlacement,
    SchematicPinSide, SchematicPoint, SchematicSvgOptions, SchematicSymbol,
    SchematicSymbolDefinition, SchematicSymbolDefinitionId, SchematicSymbolId, SchematicSymbolUnit,
    SchematicWire, SchematicWireId, SemanticDocument, StackupLayer, StackupLayerKind,
    TransientPolicy, ViaId, ZoneId,
};
use hyperlattice::Point2;
use hyperpath::{LinePathSegment, SpecctraRoute, TraceLayer};

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn sp(x: i64, y: i64) -> SchematicPoint {
    SchematicPoint::new(Real::from(x), Real::from(y))
}

fn parameter(name: &str, value: i64, unit: &str) -> CircuitParameter {
    CircuitParameter {
        name: name.into(),
        value: Real::from(value),
        unit: unit.into(),
        source: "release-fixture".into(),
    }
}

fn two_pins() -> Vec<DevicePin> {
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

fn instance(id: &str, model: &str, output: &NetId, ground: &NetId) -> CircuitInstance {
    CircuitInstance {
        id: CircuitInstanceId::new(id).unwrap(),
        component: ComponentId::new(id).unwrap(),
        part: Some(PartRef::new(format!("parts:{id}")).unwrap()),
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
        parameters: Vec::new(),
    }
}

fn release_fixture() -> (Circuit, SchematicLayout, PcbLayout) {
    let ground = NetId::new("GND").unwrap();
    let output = NetId::new("OUT").unwrap();
    let source_model = DeviceModelId::new("voltage-source").unwrap();
    let load_model = DeviceModelId::new("load-resistor").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("release-board").unwrap(),
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
        id: source_model,
        kind: DeviceModelKind::VoltageSource,
        pins: two_pins(),
        parameters: vec![parameter("voltage", 5, "V")],
    })
    .with_device_model(DeviceModel {
        id: load_model,
        kind: DeviceModelKind::Resistor,
        pins: two_pins(),
        parameters: vec![parameter("resistance", 10, "ohm")],
    })
    .with_instance(instance("V1", "voltage-source", &output, &ground))
    .with_instance(instance("R1", "load-resistor", &output, &ground));

    let source_symbol = SchematicSymbolId::new("V1:A").unwrap();
    let load_symbol = SchematicSymbolId::new("R1:A").unwrap();
    let source_definition = SchematicSymbolDefinitionId::new("voltage-source-symbol").unwrap();
    let load_definition = SchematicSymbolDefinitionId::new("load-resistor-symbol").unwrap();
    let pins = || {
        vec![
            SchematicPinPlacement {
                pin: PinRef::new("+").unwrap(),
                position: sp(-8, -3),
                side: SchematicPinSide::Left,
            },
            SchematicPinPlacement {
                pin: PinRef::new("-").unwrap(),
                position: sp(-8, 3),
                side: SchematicPinSide::Left,
            },
        ]
    };
    let definition = |id, model: &str| SchematicSymbolDefinition {
        id,
        model: DeviceModelId::new(model).unwrap(),
        name: model.into(),
        units: vec![SchematicSymbolUnit {
            unit: 1,
            body_width: Real::from(16),
            body_height: Real::from(14),
            pins: pins(),
            graphics: Vec::new(),
        }],
    };
    let symbol = |id: SchematicSymbolId,
                  definition: SchematicSymbolDefinitionId,
                  instance: &str,
                  x: i64| SchematicSymbol {
        id,
        instance: CircuitInstanceId::new(instance).unwrap(),
        definition,
        unit: 1,
        position: sp(x, 0),
        quarter_turns: 0,
    };
    let schematic = SchematicLayout {
        symbol_definitions: vec![
            definition(source_definition.clone(), "voltage-source"),
            definition(load_definition.clone(), "load-resistor"),
        ],
        symbols: vec![
            symbol(source_symbol.clone(), source_definition, "V1", 0),
            symbol(load_symbol.clone(), load_definition, "R1", 50),
        ],
        ports: Vec::new(),
        wires: vec![
            SchematicWire {
                id: SchematicWireId::new("out-wire").unwrap(),
                net: output.clone(),
                from: SchematicEndpoint::Pin {
                    symbol: source_symbol.clone(),
                    pin: PinRef::new("+").unwrap(),
                },
                waypoints: vec![sp(25, -3)],
                to: SchematicEndpoint::Pin {
                    symbol: load_symbol.clone(),
                    pin: PinRef::new("+").unwrap(),
                },
            },
            SchematicWire {
                id: SchematicWireId::new("ground-wire").unwrap(),
                net: ground.clone(),
                from: SchematicEndpoint::Pin {
                    symbol: source_symbol,
                    pin: PinRef::new("-").unwrap(),
                },
                waypoints: vec![sp(25, 3)],
                to: SchematicEndpoint::Pin {
                    symbol: load_symbol,
                    pin: PinRef::new("-").unwrap(),
                },
            },
        ],
        labels: vec![SchematicLabel {
            id: SchematicLabelId::new("out-label").unwrap(),
            net: output.clone(),
            position: sp(25, -6),
            text: "OUT".into(),
        }],
        sheets: Vec::new(),
        sheet_ports: Vec::new(),
        sheet_links: Vec::new(),
    };

    let front = TraceLayer(0);
    let back = TraceLayer(1);
    let pattern_id = LandPatternId::new("TWO_PIN").unwrap();
    let pattern = LandPattern {
        id: pattern_id.clone(),
        pads: vec![
            LandPatternPad {
                id: PadId::new("1").unwrap(),
                center: p(0, -2),
                rotation_degrees: Real::zero(),
                copper_layers: vec![front],
                shape: PadShape::Rectangle {
                    width: Real::from(2),
                    height: Real::from(2),
                },
                drill: None,
                plating: Plating::Unspecified,
                solder_mask_margin: None,
                paste_margin: None,
            },
            LandPatternPad {
                id: PadId::new("2").unwrap(),
                center: p(0, 2),
                rotation_degrees: Real::zero(),
                copper_layers: vec![front],
                shape: PadShape::Rectangle {
                    width: Real::from(2),
                    height: Real::from(2),
                },
                drill: None,
                plating: Plating::Unspecified,
                solder_mask_margin: None,
                paste_margin: None,
            },
        ],
        pin_map: vec![
            PadPinMap {
                pin: PinRef::new("+").unwrap(),
                pad: PadId::new("1").unwrap(),
            },
            PadPinMap {
                pin: PinRef::new("-").unwrap(),
                pad: PadId::new("2").unwrap(),
            },
        ],
        graphics: vec![LandPatternGraphic {
            id: LandPatternGraphicId::new("courtyard").unwrap(),
            layer: LayerRole::Courtyard,
            stroke_width: Some((Real::one() / Real::from(20)).unwrap()),
            primitive: LandPatternGraphicPrimitive::Polygon {
                vertices: vec![p(-4, -5), p(4, -5), p(4, 5), p(-4, 5)],
                filled: false,
            },
        }],
        body: Some(LandPatternBody {
            outline: vec![p(-3, -4), p(3, -4), p(3, 4), p(-3, 4)],
            height: Real::from(3),
            standoff: (Real::one() / Real::from(5)).unwrap(),
        }),
        models: vec![hypercircuit::Pcb3dModelReference {
            uri: "${KICAD9_3DMODEL_DIR}/Package.3dshapes/Two_Pin.step".into(),
            format: hypercircuit::Pcb3dModelFormat::Step,
            transform: hypercircuit::Pcb3dModelTransform {
                offset_z: (Real::one() / Real::from(5)).unwrap(),
                ..Default::default()
            },
        }],
    };
    let half = (Real::one() / Real::from(2)).unwrap();
    let mask = (Real::one() / Real::from(10)).unwrap();
    let layout = PcbLayout {
        id: BoardId::new("release-board").unwrap(),
        outline: BoardOutline {
            exterior: vec![p(0, 0), p(30, 0), p(30, 20), p(0, 20)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![
                StackupLayer {
                    name: "F.Mask".into(),
                    kind: StackupLayerKind::SolderMask,
                    thickness: mask.clone(),
                    material: Some("hyperphysics:solder-mask".into()),
                },
                StackupLayer {
                    name: "F.Cu".into(),
                    kind: StackupLayerKind::Conductor(front),
                    thickness: half.clone(),
                    material: Some("hyperphysics:copper".into()),
                },
                StackupLayer {
                    name: "Core".into(),
                    kind: StackupLayerKind::Dielectric,
                    thickness: Real::from(3),
                    material: Some("hyperphysics:FR4".into()),
                },
                StackupLayer {
                    name: "B.Cu".into(),
                    kind: StackupLayerKind::Conductor(back),
                    thickness: half.clone(),
                    material: Some("hyperphysics:copper".into()),
                },
                StackupLayer {
                    name: "B.Mask".into(),
                    kind: StackupLayerKind::SolderMask,
                    thickness: mask,
                    material: Some("hyperphysics:solder-mask".into()),
                },
            ],
        },
        land_patterns: vec![pattern],
        placements: vec![
            PcbPlacement {
                instance: CircuitInstanceId::new("V1").unwrap(),
                land_pattern: pattern_id.clone(),
                position: p(5, 5),
                rotation_degrees: Real::zero(),
                side: BoardSide::Front,
            },
            PcbPlacement {
                instance: CircuitInstanceId::new("R1").unwrap(),
                land_pattern: pattern_id,
                position: p(25, 5),
                rotation_degrees: Real::zero(),
                side: BoardSide::Front,
            },
        ],
        placement_constraints: vec![
            PlacementConstraint {
                id: PlacementConstraintId::new("source-fixed").unwrap(),
                kind: PlacementConstraintKind::Fixed {
                    instance: CircuitInstanceId::new("V1").unwrap(),
                    position: p(5, 5),
                },
            },
            PlacementConstraint {
                id: PlacementConstraintId::new("load-relative").unwrap(),
                kind: PlacementConstraintKind::Relative {
                    instance: CircuitInstanceId::new("R1").unwrap(),
                    anchor: CircuitInstanceId::new("V1").unwrap(),
                    offset: p(20, 0),
                },
            },
            PlacementConstraint {
                id: PlacementConstraintId::new("assembly-row").unwrap(),
                kind: PlacementConstraintKind::AlignY {
                    instances: vec![
                        CircuitInstanceId::new("V1").unwrap(),
                        CircuitInstanceId::new("R1").unwrap(),
                    ],
                },
            },
            PlacementConstraint {
                id: PlacementConstraintId::new("load-region").unwrap(),
                kind: PlacementConstraintKind::Within {
                    instance: CircuitInstanceId::new("R1").unwrap(),
                    min: p(20, 2),
                    max: p(28, 8),
                },
            },
        ],
        routes: vec![
            PcbRoute {
                id: RouteId::new("out-route").unwrap(),
                net: output.clone(),
                layer: front,
                width: Real::one(),
                segments: vec![LinePathSegment::new(p(5, 3), p(25, 3)).into()],
            },
            PcbRoute {
                id: RouteId::new("ground-route").unwrap(),
                net: ground.clone(),
                layer: front,
                width: Real::one(),
                segments: vec![
                    LinePathSegment::new(p(5, 7), p(5, 10)).into(),
                    LinePathSegment::new(p(5, 10), p(25, 10)).into(),
                    LinePathSegment::new(p(25, 10), p(25, 7)).into(),
                ],
            },
        ],
        vias: vec![PcbVia {
            id: ViaId::new("out-via").unwrap(),
            net: output.clone(),
            start_layer: front,
            end_layer: back,
            center: p(15, 3),
            land_diameter: Real::from(3),
            drill_diameter: Real::one(),
            plating: Plating::Plated,
            mask: hypercircuit::ViaMaskIntent::tented(),
        }],
        zones: vec![CopperZone {
            id: ZoneId::new("ground-plane").unwrap(),
            net: ground.clone(),
            layer: back,
            boundary: vec![p(2, 12), p(28, 12), p(28, 18), p(2, 18)],
            clearance: Real::zero(),
            fill: hypercircuit::CopperZoneFill::Solid,
            connection: hypercircuit::CopperZoneConnection::Solid,
            islands: hypercircuit::CopperZoneIslandPolicy::retain_all(),
            stitching: None,
            priority: 0,
        }],
        keepouts: vec![PcbKeepout {
            id: KeepoutId::new("assembly-reserve").unwrap(),
            boundary: vec![p(12, 13), p(18, 13), p(18, 17), p(12, 17)],
            scope: KeepoutScope::Components,
        }],
        rules: PcbDesignRules {
            net_classes: vec![NetClass {
                id: NetClassId::new("default").unwrap(),
                parent: None,
                nets: vec![ground, output],
                min_trace_width: Some(half.clone()),
                preferred_trace_width: None,
                min_clearance: Some(half),
                preferred_via_land_diameter: Some(Real::from(3)),
                preferred_via_drill_diameter: Some(Real::one()),
                preferred_via_style: None,
                max_length: Some(Real::from(100)),
                max_via_count: Some(2),
                target_impedance_ohms: None,
                impedance_tolerance_ohms: None,
                requires_reference_plane: false,
            }],
            differential_pairs: Vec::<DifferentialPair>::new(),
            ..PcbDesignRules::default()
        },
    };
    (circuit, schematic, layout)
}

#[test]
fn representative_board_spans_authoring_review_verification_and_release_outputs() {
    let (circuit, schematic, layout) = release_fixture();
    assert!(circuit.validate().is_valid());
    assert!(circuit.electrical_rule_check().is_valid());
    assert!(schematic.validate(&circuit).is_valid());
    assert!(layout.validate(&circuit).is_valid());
    let placement = layout.resolve_placement_constraints(&circuit);
    assert!(placement.is_satisfied());
    assert_eq!(placement.placements[1].position, p(25, 5));

    let dc = circuit
        .linear_mna_from_devices()
        .unwrap()
        .solve_exact()
        .unwrap();
    assert!(dc.replay.accepted);
    assert_eq!(dc.candidate[0], Real::from(5));

    let schematic_svg = schematic
        .to_svg(&circuit, SchematicSvgOptions::default())
        .unwrap();
    let pcb_svg = layout.to_svg(&circuit, PcbSvgOptions::default()).unwrap();
    assert!(schematic_svg.svg.contains("<polyline"));
    assert!(pcb_svg.svg.contains("out-route"));

    let routing = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    assert_eq!(routing.problem.terminals.len(), 4);
    let accepted = SpecctraRoute::with_vias(
        routing.problem.existing.traces().to_vec(),
        routing.problem.existing.vias().to_vec(),
    );
    let routed = RoutingSolution::from_hyperpath(&routing.problem, &accepted).unwrap();
    assert!(routed.replace_in(&layout).validate(&circuit).is_valid());

    let materialized = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    assert!(materialized.copper_features.iter().any(|feature| matches!(
        &feature.identity,
        hypercircuit::MaterializedCopperIdentity::Pad {
            instance,
            land_pattern,
            pad,
            pin: Some(pin),
        } if instance.as_str() == "V1"
            && land_pattern.as_str() == "TWO_PIN"
            && pad.as_str() == "1"
            && pin.as_str() == "+"
    )));
    assert_eq!(materialized.process_features.len(), 8);
    for role in [
        hypercircuit::ProcessLayerRole::FrontSolderMask,
        hypercircuit::ProcessLayerRole::FrontPaste,
    ] {
        assert!(
            materialized
                .process_layers
                .iter()
                .any(|image| image.role == role && image.image.is_some()),
            "missing decided {role:?}; available: {:?}",
            materialized
                .process_layers
                .iter()
                .map(|image| (image.role, image.image.is_some(), image.blocker.as_deref()))
                .collect::<Vec<_>>()
        );
    }
    let assembly_3d = materialized.stackup_3d(&layout);
    assert_eq!(
        assembly_3d.total_thickness,
        (Real::from(21) / Real::from(5)).unwrap()
    );
    assert_eq!(assembly_3d.component_bodies.len(), 2);
    assert_eq!(
        assembly_3d.component_bodies[0].metadata.z_start,
        (Real::from(22) / Real::from(5)).unwrap()
    );
    assert!(assembly_3d.omissions.iter().any(|omission| matches!(
        omission,
        hypercircuit::Pcb3dAssemblyOmission::ExternalPackageModelNotLoaded {
            uri,
            model_index: 0,
            ..
        } if uri.ends_with("Two_Pin.step")
    )));
    assert!(assembly_3d.subtractions.iter().any(|evidence| matches!(
        evidence.kind,
        hypercircuit::Pcb3dSubtractionKind::SolderMaskOpenings {
            role: hypercircuit::ProcessLayerRole::FrontSolderMask,
            source_feature_count: 4,
        }
    )));
    assert!(!assembly_3d.omissions.iter().any(|omission| matches!(
        omission,
        hypercircuit::Pcb3dAssemblyOmission::SolderMaskSideIndeterminate(_)
            | hypercircuit::Pcb3dAssemblyOmission::SolderMaskImageBlocked { .. }
            | hypercircuit::Pcb3dAssemblyOmission::SolderMaskSubtractionFailed { .. }
    )));
    let gltf = assembly_3d.to_gltf("release-board").unwrap();
    let gltf_json = serde_json::from_str::<serde_json::Value>(&gltf.gltf).unwrap();
    assert_eq!(gltf.objects.len(), 7);
    assert_eq!(gltf_json["meshes"].as_array().unwrap().len(), 7);
    assert_eq!(gltf_json["nodes"][0]["name"], "layer:0:F.Mask");
    let drc = hypercircuit::HyperDrcHandoff::from_materialization(&layout, &materialized);
    assert_eq!(drc.copper_layers.len(), 2);
    assert_eq!(drc.process_layers.len(), 2);
    assert_eq!(drc.authored_components.len(), 2);
    assert!(drc.authored_components.iter().all(|component| {
        component.kind == hyperdrc::authoring_intent::AuthoredComponentEnvelopeKind::Courtyard
    }));
    let readiness = drc.run_readiness(&DrcReadinessPolicy::default());
    assert!(readiness.is_release_clean());

    let fabrication = FabricationPackage::from_materialization(&layout, &materialized).unwrap();
    assert_eq!(fabrication.represented_process_features, 8);
    for function in ["Soldermask,Top", "Paste,Top", "Profile,NP"] {
        assert!(fabrication.files.iter().any(|file| {
            file.bytes
                .windows(function.len())
                .any(|window| window == function.as_bytes())
        }));
    }
    assert!(!fabrication.production_omissions.iter().any(|omission| {
        matches!(
            omission,
            hypercircuit::ProcessMaterializationOmission::ViaMaskIntentUnavailable { .. }
        )
    }));
    assert!(fabrication.verify_integrity().is_empty());
    let cam_round_trip = fabrication.audit_cam_round_trip();
    assert!(
        cam_round_trip.is_release_clean(),
        "CAM re-import issues: {:?}",
        cam_round_trip.issues
    );
    assert_eq!(fabrication.manifest.represented_test_points, 5);
    assert!(fabrication.manifest.connectivity_omissions.is_empty());
    assert_eq!(cam_round_trip.ipc356.as_ref().unwrap().points, 5);
    assert_eq!(
        cam_round_trip
            .ipc356
            .as_ref()
            .unwrap()
            .reference_pin_records,
        5
    );
    let assembly = AssemblyOutputs::from_design(&circuit, &layout).unwrap();
    assert_eq!(assembly.pick_and_place.len(), 2);
    assert!(assembly.unplaced_instances.is_empty());
    let assembly_round_trip = assembly.audit_csv_round_trip();
    assert!(
        assembly_round_trip.is_release_clean(),
        "assembly CSV issues: {:?}",
        assembly_round_trip.issues
    );
    let prototype = AssemblyVariant {
        id: AssemblyVariantId::new("prototype").unwrap(),
        dnp_instances: vec![CircuitInstanceId::new("R1").unwrap()],
        part_overrides: vec![AssemblyPartOverride {
            instance: CircuitInstanceId::new("V1").unwrap(),
            part: PartRef::new("parts:bench-source").unwrap(),
        }],
    };
    let prototype_outputs = AssemblyOutputs::from_variant(&circuit, &layout, &prototype).unwrap();
    assert_eq!(prototype_outputs.pick_and_place.len(), 1);
    assert_eq!(prototype_outputs.dnp_instances.len(), 1);
    assert_eq!(
        prototype_outputs.bom[0].part.as_ref().unwrap().as_str(),
        "parts:bench-source"
    );
    assert!(prototype_outputs.dnp_csv().contains("R1,prototype"));
    let prototype_round_trip = prototype_outputs.audit_csv_round_trip();
    assert!(
        prototype_round_trip.is_release_clean(),
        "variant assembly CSV issues: {:?}",
        prototype_round_trip.issues
    );

    let document = SemanticDocument::new(circuit.clone(), Some(schematic.clone()))
        .unwrap()
        .with_pcb(layout.clone())
        .unwrap();
    let restored = SemanticDocument::from_json(&document.to_json_pretty().unwrap()).unwrap();
    assert_eq!(restored, document);
    let mut legacy = serde_json::to_value(&document).unwrap();
    legacy["version"] = serde_json::Value::from(20);
    let pattern = legacy["pcb"]["land_patterns"][0].as_object_mut().unwrap();
    let model = pattern
        .remove("models")
        .unwrap()
        .as_array_mut()
        .unwrap()
        .remove(0);
    let body = pattern["body"].as_object_mut().unwrap();
    body.insert("model_handle".into(), model["uri"].clone());
    let (migrated, migration) =
        SemanticDocument::from_json_migrating(&serde_json::to_string(&legacy).unwrap()).unwrap();
    assert_eq!(
        migration.steps,
        vec![
            hypercircuit::SemanticMigrationStep::ExternalPcb3dModels,
            hypercircuit::SemanticMigrationStep::IndependentPcb3dModels,
            hypercircuit::SemanticMigrationStep::PadLocalRotations,
            hypercircuit::SemanticMigrationStep::DifferentialPairImpedance,
            hypercircuit::SemanticMigrationStep::PhaseTuningGroups,
            hypercircuit::SemanticMigrationStep::DifferentialPairNeckdown,
        ]
    );
    assert_eq!(
        migrated.pcb.as_ref().unwrap().land_patterns[0].models[0].format,
        hypercircuit::Pcb3dModelFormat::Step
    );

    let kicad = layout
        .export_kicad(&circuit, KiCadExportOptions::default())
        .unwrap();
    assert!(kicad.board.contains("(model "));
    assert!(
        kicad
            .omissions
            .iter()
            .any(|omission| matches!(omission, KiCadExportOmission::PlacementConstraints(4)))
    );
    assert!(kicad.omissions.iter().any(|omission| matches!(
        omission,
        KiCadExportOmission::ViaMaskIntent { via } if via == "out-via"
    )));
    let imported = KiCadImportReport::from_str(
        &kicad.board,
        KiCadImportOptions::new(
            CircuitId::new("release-import").unwrap(),
            BoardId::new("release-import").unwrap(),
            Real::one(),
        ),
    )
    .unwrap();
    assert!(imported.circuit.validate().is_valid());
    assert!(imported.layout.validate(&imported.circuit).is_valid());
    assert_eq!(imported.layout.placements.len(), 2);
    assert_eq!(imported.layout.vias.len(), 1);
    assert_eq!(imported.layout.zones.len(), 1);
    assert_eq!(imported.layout.land_patterns[0].models.len(), 1);
    assert_eq!(
        imported.layout.land_patterns[0].models[0],
        layout.land_patterns[0].models[0]
    );
}

#[test]
fn external_obj_vrml_and_gltf_package_models_resolve_with_digest_and_scene_identity() {
    let (circuit, _, mut layout) = release_fixture();
    let reference = &mut layout.land_patterns[0].models[0];
    reference.uri = "package://two-pin/body.obj".into();
    reference.format = hypercircuit::Pcb3dModelFormat::WavefrontObj;
    reference.transform = hypercircuit::Pcb3dModelTransform {
        offset_z: Real::one(),
        scale_x: Real::from(2),
        ..Default::default()
    };
    let mut detail = reference.clone();
    detail.uri = "package://two-pin/detail.gltf".into();
    detail.format = hypercircuit::Pcb3dModelFormat::Gltf;
    detail.transform.offset_x = Real::from(3);
    let mut outline = detail.clone();
    outline.uri = "package://two-pin/outline.wrl".into();
    outline.format = hypercircuit::Pcb3dModelFormat::Vrml;
    outline.transform.offset_y = Real::from(4);
    layout.land_patterns[0].models.push(detail);
    layout.land_patterns[0].models.push(outline);
    let materialized = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    let obj = b"v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n";
    let gltf = csgrs::mesh::Mesh::<()>::cube(Real::one(), ())
        .to_gltf("package-detail")
        .unwrap()
        .into_bytes();
    let vrml = br#"#VRML V2.0 utf8
Group { children [
  Shape { geometry IndexedLineSet {
    coord Coordinate { point [ 0 0 0, 1 1 0 ] }
    coordIndex [ 0 1 -1 ]
  } }
  Shape { geometry IndexedFaceSet {
    coord Coordinate { point [ 0 0 0, 1 0 0, 0 1 0, 2 0 0, 3 0 0 ] }
    coordIndex [ 0 1 2 -1, 0 0 0 -1, 0 1 3 4 -1 ]
  } }
] }
"#;
    let mut calls = 0;
    let mut resolver = |model: &hypercircuit::Pcb3dModelReference| {
        calls += 1;
        match model.uri.as_str() {
            "package://two-pin/body.obj" => Ok(obj.to_vec()),
            "package://two-pin/detail.gltf" => Ok(gltf.clone()),
            "package://two-pin/outline.wrl" => Ok(vrml.to_vec()),
            uri => panic!("unexpected package-model URI {uri}"),
        }
    };

    let assembly = materialized.stackup_3d_with_model_resolver(&layout, &mut resolver);
    assert_eq!(calls, 6);
    assert_eq!(assembly.component_models.len(), 6);
    assert_eq!(assembly.model_resolutions.len(), 6);
    assert!(assembly.model_resolutions.iter().all(|evidence| {
        evidence.model_index < 3
            && evidence.source_sha256.len() == 64
            && match evidence.model_index {
                0 => {
                    evidence.uri == "package://two-pin/body.obj"
                        && evidence.format == hypercircuit::Pcb3dModelFormat::WavefrontObj
                        && evidence.triangle_count == 4
                        && evidence.source_scene_index.is_none()
                        && evidence.source_mesh_node_count == 1
                        && evidence.source_primitive_count == 1
                        && evidence.ignored_non_mesh_geometry_count == 0
                        && evidence.ignored_degenerate_polygon_count == 0
                        && evidence.ignored_degenerate_triangle_count == 0
                }
                1 => {
                    evidence.uri == "package://two-pin/detail.gltf"
                        && evidence.format == hypercircuit::Pcb3dModelFormat::Gltf
                        && evidence.triangle_count == 12
                        && evidence.source_scene_index == Some(0)
                        && evidence.source_mesh_node_count == 1
                        && evidence.source_primitive_count == 1
                        && evidence.ignored_non_mesh_geometry_count == 0
                        && evidence.ignored_degenerate_polygon_count == 0
                        && evidence.ignored_degenerate_triangle_count == 0
                }
                2 => {
                    evidence.uri == "package://two-pin/outline.wrl"
                        && evidence.format == hypercircuit::Pcb3dModelFormat::Vrml
                        && evidence.triangle_count == 1
                        && evidence.source_scene_index.is_none()
                        && evidence.source_mesh_node_count == 2
                        && evidence.source_primitive_count == 1
                        && evidence.ignored_non_mesh_geometry_count == 1
                        && evidence.ignored_degenerate_polygon_count == 1
                        && evidence.ignored_degenerate_triangle_count == 1
                }
                _ => false,
            }
    }));
    assert!(!assembly.omissions.iter().any(|omission| matches!(
        omission,
        hypercircuit::Pcb3dAssemblyOmission::ExternalPackageModelNotLoaded { .. }
            | hypercircuit::Pcb3dAssemblyOmission::ExternalPackageModelResolutionFailed { .. }
            | hypercircuit::Pcb3dAssemblyOmission::ExternalPackageModelFormatUnsupported { .. }
            | hypercircuit::Pcb3dAssemblyOmission::ExternalPackageModelParseFailed { .. }
    )));
    let scene = assembly.to_gltf("resolved-package-models").unwrap();
    assert_eq!(
        scene
            .objects
            .iter()
            .filter(|object| matches!(
                object.kind,
                hypercircuit::Pcb3dSceneObjectKind::ComponentModel { .. }
            ))
            .count(),
        6
    );
    for name in [
        "component-model:V1:0",
        "component-model:V1:1",
        "component-model:V1:2",
        "component-model:R1:0",
        "component-model:R1:1",
        "component-model:R1:2",
    ] {
        assert!(scene.objects.iter().any(|object| object.name == name));
    }
    assert!(!scene.objects.iter().any(|object| matches!(
        object.kind,
        hypercircuit::Pcb3dSceneObjectKind::ComponentBody { .. }
    )));
}

#[test]
fn oversized_authored_mask_expansion_is_reported_by_native_hyperdrc() {
    let (circuit, _, layout) = release_fixture();
    let materialized = layout
        .materialize(
            &circuit,
            MaterializationOptions {
                default_solder_mask_margin: Real::one(),
                ..MaterializationOptions::default()
            },
        )
        .unwrap();
    let drc = hypercircuit::HyperDrcHandoff::from_materialization(&layout, &materialized);
    let readiness = drc.run_readiness(&DrcReadinessPolicy::default());

    assert!(readiness.violations.iter().any(|violation| {
        violation.check == "solder-mask-expansion"
            && violation.severity == hyperdrc::Severity::Warning
            && violation.layers.iter().any(|layer| layer == "F.Mask")
    }));
}

#[test]
fn cyclic_relative_placement_constraints_are_reported() {
    let (circuit, _, mut layout) = release_fixture();
    layout.placement_constraints = vec![
        PlacementConstraint {
            id: PlacementConstraintId::new("source-after-load").unwrap(),
            kind: PlacementConstraintKind::Relative {
                instance: CircuitInstanceId::new("V1").unwrap(),
                anchor: CircuitInstanceId::new("R1").unwrap(),
                offset: p(1, 0),
            },
        },
        PlacementConstraint {
            id: PlacementConstraintId::new("load-after-source").unwrap(),
            kind: PlacementConstraintKind::Relative {
                instance: CircuitInstanceId::new("R1").unwrap(),
                anchor: CircuitInstanceId::new("V1").unwrap(),
                offset: p(1, 0),
            },
        },
    ];

    assert!(layout.validate(&circuit).is_valid());
    let report = layout.resolve_placement_constraints(&circuit);
    assert!(!report.is_satisfied());
    assert!(
        report
            .issues
            .iter()
            .any(|issue| matches!(issue, PlacementResolutionIssue::RelativeCycle(_)))
    );
}

#[test]
fn overlapping_component_courtyards_are_release_blocking() {
    let (circuit, _, mut layout) = release_fixture();
    layout.placement_constraints.clear();
    layout.placements[1].position = layout.placements[0].position.clone();
    let materialized = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    let drc = hypercircuit::HyperDrcHandoff::from_materialization(&layout, &materialized);
    let readiness = drc.run_readiness(&DrcReadinessPolicy::default());

    assert!(!readiness.is_release_clean());
    assert!(readiness.violations.iter().any(|violation| {
        violation.check == "authored-component-readiness"
            && violation.severity == hyperdrc::Severity::Error
            && violation.layers.iter().any(|layer| layer == "component:V1")
            && violation.layers.iter().any(|layer| layer == "component:R1")
    }));
}

#[test]
fn component_keepouts_are_checked_against_placed_envelopes() {
    let (circuit, _, mut layout) = release_fixture();
    layout.keepouts[0].boundary = vec![p(3, 3), p(7, 3), p(7, 7), p(3, 7)];
    let materialized = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    let drc = hypercircuit::HyperDrcHandoff::from_materialization(&layout, &materialized);
    let readiness = drc.run_readiness(&DrcReadinessPolicy::default());

    assert!(!readiness.is_release_clean());
    assert!(readiness.violations.iter().any(|violation| {
        violation.check == "authored-component-readiness"
            && violation
                .layers
                .iter()
                .any(|layer| layer == "keepout:assembly-reserve")
            && violation.layers.iter().any(|layer| layer == "component:V1")
    }));
}

#[test]
fn package_body_is_an_explicit_fallback_when_no_courtyard_exists() {
    let (circuit, _, mut layout) = release_fixture();
    layout.land_patterns[0].graphics.clear();
    let materialized = layout
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    let drc = hypercircuit::HyperDrcHandoff::from_materialization(&layout, &materialized);

    assert_eq!(drc.authored_components.len(), 2);
    assert!(drc.authored_components.iter().all(|component| {
        component.kind == hyperdrc::authoring_intent::AuthoredComponentEnvelopeKind::Body
    }));
    assert!(
        drc.run_readiness(&DrcReadinessPolicy::default())
            .is_release_clean()
    );

    layout.land_patterns[0].body = None;
    let missing = hypercircuit::HyperDrcHandoff::from_materialization(&layout, &materialized);
    assert!(missing.authored_components.is_empty());
    assert!(missing.omissions.iter().any(|omission| matches!(
        omission,
        hypercircuit::DrcHandoffOmission::MissingComponentEnvelope(_)
    )));
    assert!(
        !missing
            .run_readiness(&DrcReadinessPolicy::default())
            .is_release_clean()
    );
}
