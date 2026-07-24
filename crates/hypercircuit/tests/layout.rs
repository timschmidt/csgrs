#![cfg(feature = "layout")]

use hypercircuit::{
    AdapterKind, AssemblyOutputs, BoardId, BoardOutline, BoardSide, Circuit, CircuitId,
    CircuitInstance, CircuitInstanceId, ComponentId, CopperZone, DeviceModel, DeviceModelId,
    DeviceModelKind, DevicePin, DrillShape, KeepoutId, KeepoutScope, KiCadExportOmission,
    KiCadExportOptions, KiCadImportOmission, KiCadImportOptions, KiCadImportReport, LandPattern,
    LandPatternGraphic, LandPatternGraphicId, LandPatternGraphicPrimitive, LandPatternId,
    LandPatternPad, LayerRole, LayoutValidationIssue, Net, NetClass, NetClassId, NetId, PadId,
    PadPinMap, PadShape, PcbDesignRules, PcbKeepout, PcbLayout, PcbPlacement, PcbRoute, PcbStackup,
    PcbVia, PinBinding, PinElectricalKind, PinRef, Plating, Real, RouteId, RoutingNetAliases,
    RoutingProblemReport, RoutingSolution, RoutingSolutionOmission, StackupLayer, StackupLayerKind,
    TransientPolicy, ViaId, ZoneId,
};
use hyperlattice::Point2;
use hyperpath::{LinePathSegment, PcbViaStack, SpecctraRoute, TraceLayer, ViaDrillIntent};

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

#[test]
fn placement_inverse_transform_exactly_replays_front_and_back_side_semantics() {
    let local = p(2, 3);
    let mut placement = PcbPlacement {
        instance: CircuitInstanceId::new("U1").unwrap(),
        land_pattern: LandPatternId::new("test").unwrap(),
        position: p(10, 10),
        rotation_degrees: Real::from(90),
        side: BoardSide::Front,
    };
    let front = placement.transform_point(&local);
    assert_eq!(front, p(7, 12));
    assert_eq!(placement.inverse_transform_point(&front), local);

    placement.side = BoardSide::Back;
    let back = placement.transform_point(&local);
    assert_eq!(back, p(7, 8));
    assert_eq!(placement.inverse_transform_point(&back), local);
}

fn fixture() -> (Circuit, PcbLayout) {
    let ground = NetId::new("GND").unwrap();
    let signal = NetId::new("SIGNAL").unwrap();
    let model = DeviceModelId::new("resistor").unwrap();
    let instance = CircuitInstanceId::new("R1").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("board").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: ground,
        is_ground: true,
    })
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
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
        id: instance.clone(),
        component: ComponentId::new("R1").unwrap(),
        part: None,
        model,
        pins: vec![PinBinding {
            pin: PinRef::new("1").unwrap(),
            net: signal.clone(),
        }],
        parameters: Vec::new(),
    });

    let front = TraceLayer(0);
    let back = TraceLayer(1);
    let footprint = LandPatternId::new("R_0603").unwrap();
    let layout = PcbLayout {
        id: BoardId::new("board").unwrap(),
        outline: BoardOutline {
            exterior: vec![p(0, 0), p(30, 0), p(30, 20), p(0, 20)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![
                StackupLayer {
                    name: "F.Cu".into(),
                    kind: StackupLayerKind::Conductor(front),
                    thickness: Real::from(1),
                    material: Some("hyperphysics:copper".into()),
                },
                StackupLayer {
                    name: "Core".into(),
                    kind: StackupLayerKind::Dielectric,
                    thickness: Real::from(2),
                    material: Some("hyperphysics:FR4".into()),
                },
                StackupLayer {
                    name: "B.Cu".into(),
                    kind: StackupLayerKind::Conductor(back),
                    thickness: Real::from(1),
                    material: Some("hyperphysics:copper".into()),
                },
            ],
        },
        land_patterns: vec![LandPattern {
            id: footprint.clone(),
            pads: vec![LandPatternPad {
                id: PadId::new("1").unwrap(),
                center: p(0, 0),
                rotation_degrees: Real::from(30),
                copper_layers: vec![front],
                shape: PadShape::Rectangle {
                    width: Real::from(2),
                    height: Real::from(1),
                },
                drill: None,
                plating: Plating::Unspecified,
                solder_mask_margin: None,
                paste_margin: None,
            }],
            pin_map: vec![PadPinMap {
                pin: PinRef::new("1").unwrap(),
                pad: PadId::new("1").unwrap(),
            }],
            graphics: vec![
                LandPatternGraphic {
                    id: LandPatternGraphicId::new("silk-reference-line").unwrap(),
                    layer: LayerRole::FrontSilkscreen,
                    stroke_width: Some((Real::one() / Real::from(5)).unwrap()),
                    primitive: LandPatternGraphicPrimitive::Line {
                        start: p(-2, -1),
                        end: p(2, -1),
                    },
                },
                LandPatternGraphic {
                    id: LandPatternGraphicId::new("courtyard").unwrap(),
                    layer: LayerRole::Courtyard,
                    stroke_width: Some((Real::one() / Real::from(20)).unwrap()),
                    primitive: LandPatternGraphicPrimitive::Polygon {
                        vertices: vec![p(-3, -2), p(3, -2), p(3, 2), p(-3, 2)],
                        filled: false,
                    },
                },
            ],
            body: None,
            models: Vec::new(),
        }],
        placements: vec![PcbPlacement {
            instance,
            land_pattern: footprint,
            position: p(5, 5),
            rotation_degrees: Real::zero(),
            side: BoardSide::Front,
        }],
        placement_constraints: Vec::new(),
        routes: vec![PcbRoute {
            id: RouteId::new("signal-1").unwrap(),
            net: signal.clone(),
            layer: front,
            width: Real::from(1),
            segments: vec![
                LinePathSegment::new(p(5, 5), p(10, 5)).into(),
                LinePathSegment::new(p(10, 5), p(15, 8)).into(),
            ],
        }],
        vias: vec![PcbVia {
            id: ViaId::new("signal-via").unwrap(),
            net: signal.clone(),
            start_layer: front,
            end_layer: back,
            center: p(15, 8),
            land_diameter: Real::from(2),
            drill_diameter: Real::from(1),
            plating: Plating::Plated,
            mask: hypercircuit::ViaMaskIntent::tented(),
        }],
        zones: vec![CopperZone {
            id: ZoneId::new("signal-zone").unwrap(),
            net: signal,
            layer: back,
            boundary: vec![p(14, 7), p(20, 7), p(20, 12), p(14, 12)],
            clearance: Real::zero(),
            fill: hypercircuit::CopperZoneFill::Solid,
            connection: hypercircuit::CopperZoneConnection::Solid,
            islands: hypercircuit::CopperZoneIslandPolicy::retain_all(),
            stitching: None,
            priority: 0,
        }],
        keepouts: vec![PcbKeepout {
            id: KeepoutId::new("mounting-hole").unwrap(),
            boundary: vec![p(24, 14), p(28, 14), p(28, 18), p(24, 18)],
            scope: KeepoutScope::Copper(vec![front, back]),
        }],
        rules: PcbDesignRules::default(),
    };
    (circuit, layout)
}

#[test]
fn representative_board_validates_and_lowers_routes_to_hyperpath() {
    let (circuit, layout) = fixture();
    assert!(circuit.validate().is_valid());
    assert!(layout.validate(&circuit).is_valid());

    let aliases = RoutingNetAliases::from_circuit(&circuit).unwrap();
    let route_alias = aliases.get(&layout.routes[0].net).unwrap();
    let lowered_route = layout.routes[0].to_hyperpath(route_alias).unwrap();
    assert_eq!(lowered_route.traces().len(), 2);
    assert!(lowered_route.arcs().is_empty());
    assert!(layout.vias[0].to_hyperpath(route_alias).is_ok());
}

#[test]
fn invalid_package_model_reports_its_independent_record() {
    let (circuit, mut layout) = fixture();
    layout.land_patterns[0]
        .models
        .push(hypercircuit::Pcb3dModelReference {
            uri: " ".into(),
            format: hypercircuit::Pcb3dModelFormat::WavefrontObj,
            transform: hypercircuit::Pcb3dModelTransform {
                scale_z: Real::zero(),
                ..Default::default()
            },
        });

    assert!(layout.validate(&circuit).issues.contains(
        &LayoutValidationIssue::InvalidPcb3dModelReference {
            land_pattern: layout.land_patterns[0].id.clone(),
            model_index: 0,
        }
    ));
}

#[test]
fn board_routing_problem_round_trips_accepted_hyperpath_candidates() {
    let (circuit, layout) = fixture();
    let handoff = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();

    assert!(handoff.omissions.is_empty());
    assert_eq!(handoff.problem.terminals.len(), 1);
    assert_eq!(handoff.problem.terminals[0].center, p(5, 5));
    assert_eq!(handoff.problem.existing.traces().len(), 2);
    assert_eq!(handoff.problem.existing.vias().len(), 1);
    assert_eq!(handoff.problem.keepouts.len(), 2);

    let accepted = SpecctraRoute::with_vias(
        handoff.problem.existing.traces().to_vec(),
        handoff.problem.existing.vias().to_vec(),
    );
    let solution = RoutingSolution::from_hyperpath(&handoff.problem, &accepted).unwrap();
    assert_eq!(solution.routes.len(), 2);
    assert_eq!(solution.routes[0].net, layout.routes[0].net);
    assert_eq!(solution.routes[0].width, layout.routes[0].width);
    assert_eq!(solution.vias.len(), 1);
    assert_eq!(solution.vias[0].id, layout.vias[0].id);
    assert_eq!(solution.vias[0].plating, Plating::Plated);
    assert_eq!(solution.vias[0].mask, layout.vias[0].mask);
    assert!(solution.omissions.is_empty());

    let rerouted = solution.replace_in(&layout);
    assert!(rerouted.validate(&circuit).is_valid());
}

#[test]
fn newly_routed_vias_report_missing_process_intent() {
    let (circuit, layout) = fixture();
    let handoff = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    let routing_net = handoff.problem.aliases.get(&layout.vias[0].net).unwrap();
    let proposed = PcbViaStack::with_drill_intent(
        routing_net,
        TraceLayer(0),
        TraceLayer(1),
        p(25, 5),
        Real::from(2),
        Real::one(),
        ViaDrillIntent::Plated,
    )
    .unwrap();
    let candidate = SpecctraRoute::with_vias(Vec::new(), vec![proposed]);
    let solution = RoutingSolution::from_hyperpath(&handoff.problem, &candidate).unwrap();

    assert_eq!(solution.vias.len(), 1);
    assert_eq!(
        solution.vias[0].mask,
        hypercircuit::ViaMaskIntent::default()
    );
    assert!(
        solution
            .omissions
            .contains(&RoutingSolutionOmission::ViaMaskIntentUnavailable(1))
    );
}

#[test]
fn via_mask_opening_must_retain_positive_geometry() {
    let (circuit, mut layout) = fixture();
    layout.vias[0].mask.front = hypercircuit::ViaMaskDisposition::Open {
        margin: -Real::from(2),
    };

    assert!(layout.validate(&circuit).issues.iter().any(|issue| {
        matches!(issue, LayoutValidationIssue::InvalidVia(id) if id == &layout.vias[0].id)
    }));
}

#[test]
fn assembly_views_derive_bom_and_pick_and_place_from_retained_identities() {
    let (circuit, layout) = fixture();
    let assembly = AssemblyOutputs::from_design(&circuit, &layout).unwrap();

    assert_eq!(assembly.bom.len(), 1);
    assert_eq!(assembly.bom[0].quantity, 1);
    assert_eq!(assembly.pick_and_place.len(), 1);
    assert!(assembly.unplaced_instances.is_empty());
    assert!(assembly.bom_csv().contains("R1"));
    assert!(assembly.pick_and_place_csv().contains("front"));
    let audit = assembly.audit_csv_round_trip();
    assert!(
        audit.is_release_clean(),
        "assembly CSV issues: {:?}",
        audit.issues
    );
    assert_eq!(audit.bom, assembly.bom);
    assert_eq!(audit.pick_and_place, assembly.pick_and_place);
    assert!(audit.dnp.is_empty());
}

#[test]
fn assembly_csv_reimport_reports_typed_tampering_and_handles_quoted_parts() {
    let (mut circuit, layout) = fixture();
    circuit.instances[0].part =
        Some(hypercircuit::PartRef::new("parts:resistor,\"precision\"").unwrap());
    let assembly = AssemblyOutputs::from_design(&circuit, &layout).unwrap();
    assert!(assembly.audit_csv_round_trip().is_release_clean());

    let altered_placement = assembly.pick_and_place_csv().replace(",front\n", ",back\n");
    let side_audit =
        assembly.audit_csv_documents(&assembly.bom_csv(), &altered_placement, &assembly.dnp_csv());
    assert!(side_audit.issues.iter().any(|issue| matches!(
        issue,
        hypercircuit::AssemblyRoundTripIssue::FieldMismatch {
            document: hypercircuit::AssemblyCsvDocument::PickAndPlace,
            reference: Some(reference),
            field: hypercircuit::AssemblyCsvField::Side,
            ..
        } if reference.as_str() == "R1"
    )));

    let altered_bom = assembly.bom_csv().replacen("1,R1,", "2,R1,", 1);
    let quantity_audit = assembly.audit_csv_documents(
        &altered_bom,
        &assembly.pick_and_place_csv(),
        &assembly.dnp_csv(),
    );
    assert!(quantity_audit.issues.iter().any(|issue| matches!(
        issue,
        hypercircuit::AssemblyRoundTripIssue::BomQuantity {
            declared: 2,
            references: 1,
            ..
        }
    )));

    let syntax_audit = assembly.audit_csv_documents(
        "quantity,references,part,model,land_pattern\n1,R1,\"unterminated\n",
        &assembly.pick_and_place_csv(),
        &assembly.dnp_csv(),
    );
    assert!(syntax_audit.issues.iter().any(|issue| matches!(
        issue,
        hypercircuit::AssemblyRoundTripIssue::CsvSyntax {
            document: hypercircuit::AssemblyCsvDocument::Bom,
            ..
        }
    )));

    let mut unplaced_layout = layout;
    unplaced_layout.placements.clear();
    let unplaced = AssemblyOutputs::from_design(&circuit, &unplaced_layout).unwrap();
    assert!(unplaced.audit_csv_round_trip().issues.iter().any(|issue| {
        matches!(
            issue,
            hypercircuit::AssemblyRoundTripIssue::UnplacedInstance(reference)
                if reference.as_str() == "R1"
        )
    }));
}

#[test]
fn placed_instances_require_complete_nonduplicated_pin_to_pad_mappings() {
    let (circuit, mut layout) = fixture();
    let duplicate_mapping = layout.land_patterns[0].pin_map[0].clone();
    layout.land_patterns[0].pin_map.push(duplicate_mapping);
    let duplicate = layout.validate(&circuit);
    assert!(
        duplicate
            .issues
            .iter()
            .any(|issue| matches!(issue, LayoutValidationIssue::DuplicatePinPadMapping { .. }))
    );

    layout.land_patterns[0].pin_map.clear();
    let missing = layout.validate(&circuit);
    assert!(
        missing
            .issues
            .iter()
            .any(|issue| matches!(issue, LayoutValidationIssue::MissingPlacedPinPad { .. }))
    );
}

#[test]
fn kicad_review_board_exports_all_retained_copper_feature_families() {
    let (circuit, layout) = fixture();
    let export = layout
        .export_kicad(&circuit, KiCadExportOptions::default())
        .unwrap();

    assert!(export.board.starts_with("(kicad_pcb"));
    assert!(export.board.contains("(footprint \"R_0603\""));
    assert!(export.board.contains("(segment "));
    assert!(export.board.contains("(via "));
    assert!(export.board.contains("(zone "));
    assert!(export.board.contains("(fp_line "));
    assert!(export.board.contains("(fp_poly "));
    assert!(!export.numeric_projections.is_empty());
    assert!(export.omissions.contains(&KiCadExportOmission::Keepouts(1)));
    assert!(export.omissions.iter().any(|omission| matches!(
        omission,
        KiCadExportOmission::ViaMaskIntent { via } if via == "signal-via"
    )));
}

#[test]
fn kicad_semantic_round_trip_can_be_edited_and_reexported() {
    let (circuit, mut layout) = fixture();
    layout.land_patterns[0].pads[0].shape = PadShape::Rectangle {
        width: Real::from(4),
        height: Real::from(2),
    };
    layout.land_patterns[0].pads[0].copper_layers = vec![TraceLayer(0), TraceLayer(1)];
    layout.land_patterns[0].pads[0].drill = Some(DrillShape::Slot {
        start: p(-1, 0),
        end: p(1, 0),
        width: Real::one(),
    });
    layout.land_patterns[0].pads[0].plating = Plating::Plated;
    let export = layout
        .export_kicad(&circuit, KiCadExportOptions::default())
        .unwrap();
    assert!(export.board.contains("(drill oval 3 1)"));
    let options = || {
        KiCadImportOptions::new(
            CircuitId::new("imported-board").unwrap(),
            BoardId::new("imported-board").unwrap(),
            Real::one(),
        )
    };
    let mut imported = KiCadImportReport::from_str(&export.board, options()).unwrap();
    assert!(imported.circuit.validate().is_valid());
    assert!(imported.layout.validate(&imported.circuit).is_valid());
    assert_eq!(imported.layout.land_patterns.len(), 1);
    assert_eq!(
        imported.layout.land_patterns[0].pads[0].rotation_degrees,
        Real::from(30)
    );
    assert_eq!(
        imported.layout.land_patterns[0].pads[0].drill,
        Some(DrillShape::Slot {
            start: p(-1, 0),
            end: p(1, 0),
            width: Real::one(),
        })
    );
    assert_eq!(imported.layout.placements.len(), 1);
    assert_eq!(imported.layout.routes.len(), 2);
    assert_eq!(imported.layout.vias.len(), 1);
    assert_eq!(imported.layout.zones.len(), 1);
    assert!(!imported.numeric_imports.is_empty());
    assert!(!imported.omissions.iter().any(|omission| matches!(
        omission,
        KiCadImportOmission::AssumedConductorThickness { .. }
            | KiCadImportOmission::DetailedStackup
    )));
    assert_eq!(imported.layout.stackup.layers.len(), 3);
    assert_eq!(imported.layout.stackup.layers[1].thickness, Real::from(2));
    assert_eq!(
        imported.layout.stackup.layers[1].material.as_deref(),
        Some("hyperphysics:FR4")
    );
    assert!(imported.omissions.iter().any(|omission| matches!(
        omission,
        KiCadImportOmission::ViaMaskIntentUnavailable { count: 1 }
    )));
    assert!(
        imported
            .omissions
            .iter()
            .any(|omission| matches!(omission, KiCadImportOmission::ZonePolicyDefaulted { .. }))
    );
    assert!(
        imported
            .omissions
            .iter()
            .any(|omission| matches!(omission, KiCadImportOmission::FootprintGraphics { .. }))
    );

    let edited_width = (Real::from(3) / Real::from(2)).unwrap();
    imported.layout.routes[0].width = edited_width.clone();
    let edited = imported
        .layout
        .export_kicad(&imported.circuit, KiCadExportOptions::default())
        .unwrap();
    assert!(edited.board.contains("(width 1.5)"));

    let reimported = KiCadImportReport::from_str(&edited.board, options()).unwrap();
    assert_eq!(
        reimported.layout.land_patterns[0].pads[0].rotation_degrees,
        Real::from(30)
    );
    assert_eq!(
        reimported.layout.land_patterns[0].pads[0].drill,
        Some(DrillShape::Slot {
            start: p(-1, 0),
            end: p(1, 0),
            width: Real::one(),
        })
    );
    assert!(
        reimported
            .layout
            .routes
            .iter()
            .any(|route| route.width == edited_width)
    );
}

#[test]
fn kicad_projects_non_native_routed_slots_to_valid_round_drills() {
    let (circuit, mut layout) = fixture();
    layout.land_patterns[0].pads[0].drill = Some(DrillShape::Slot {
        start: p(-1, -1),
        end: p(1, 1),
        width: Real::one(),
    });
    layout.land_patterns[0].pads[0].plating = Plating::Plated;

    let export = layout
        .export_kicad(&circuit, KiCadExportOptions::default())
        .unwrap();

    assert!(export.board.contains("(drill 1)"));
    assert!(export.omissions.iter().any(|omission| matches!(
        omission,
        KiCadExportOmission::RoutedSlotProjectedToRound { land_pattern, pad }
            if land_pattern == "R_0603" && pad == "1"
    )));
}

#[test]
fn kicad_stackup_and_custom_rule_companion_round_trip_exact_policy() {
    let (circuit, mut layout) = fixture();
    let signal = NetId::new("SIGNAL").unwrap();
    layout.rules.net_classes.push(NetClass {
        id: NetClassId::new("power").unwrap(),
        parent: None,
        nets: vec![signal],
        min_trace_width: Some((Real::from(3) / Real::from(5)).unwrap()),
        preferred_trace_width: Some((Real::from(4) / Real::from(5)).unwrap()),
        min_clearance: Some((Real::one() / Real::from(4)).unwrap()),
        preferred_via_land_diameter: Some((Real::from(6) / Real::from(5)).unwrap()),
        preferred_via_drill_diameter: Some((Real::from(3) / Real::from(5)).unwrap()),
        preferred_via_style: None,
        max_length: Some(Real::from(40)),
        max_via_count: Some(3),
        target_impedance_ohms: None,
        impedance_tolerance_ohms: None,
        requires_reference_plane: false,
    });
    let export = layout
        .export_kicad(&circuit, KiCadExportOptions::default())
        .unwrap();
    assert!(export.board.contains("(setup"));
    assert!(export.board.contains("(stackup"));
    assert!(
        export
            .board
            .contains("(layer \"F.Cu\" (type \"copper\") (thickness 1)")
    );
    assert!(export.board.contains(
        "(layer \"dielectric 1\" (type \"core\") (thickness 2) (material \"hyperphysics:FR4\"))"
    ));
    let design_rules = export.design_rules.as_deref().unwrap();
    assert!(design_rules.starts_with("(version 1)"));
    assert!(design_rules.contains("(rule \"hypercircuit.netclass.power\""));
    assert!(design_rules.contains("(condition \"A.NetName == 'SIGNAL'\")"));
    assert!(design_rules.contains("(constraint track_width (min 0.6mm))"));
    assert!(design_rules.contains("(constraint clearance (min 0.25mm))"));
    assert!(design_rules.contains("(constraint length (max 40mm))"));
    assert!(design_rules.contains("(constraint via_count (max 3))"));
    assert_eq!(export.design_rule_projections.len(), 1);
    let project = export.project.as_deref().unwrap();
    let project_json: serde_json::Value = serde_json::from_str(project).unwrap();
    assert_eq!(
        project_json["net_settings"]["classes"][0]["track_width"],
        serde_json::json!(0.8)
    );
    assert_eq!(
        project_json["net_settings"]["classes"][0]["clearance"],
        serde_json::json!(0.25)
    );
    assert_eq!(
        project_json["net_settings"]["classes"][0]["via_diameter"],
        serde_json::json!(1.2)
    );
    assert_eq!(
        project_json["net_settings"]["classes"][0]["via_drill"],
        serde_json::json!(0.6)
    );
    assert_eq!(
        project_json["net_settings"]["netclass_assignments"]["SIGNAL"],
        serde_json::json!(["power"])
    );
    assert_eq!(export.project_net_class_projections.len(), 1);
    assert!(
        export
            .omissions
            .iter()
            .all(|omission| !matches!(omission, KiCadExportOmission::AdvancedDesignRules { .. }))
    );

    let options = KiCadImportOptions::new(
        CircuitId::new("stackup-rule-import").unwrap(),
        BoardId::new("stackup-rule-import").unwrap(),
        Real::from(99),
    );
    let imported = KiCadImportReport::from_str_with_companions(
        &export.board,
        Some(project),
        Some(design_rules),
        options,
    )
    .unwrap();
    assert!(!imported.omissions.iter().any(|omission| matches!(
        omission,
        KiCadImportOmission::AssumedConductorThickness { .. }
            | KiCadImportOmission::DetailedStackup
            | KiCadImportOmission::BoardRules
            | KiCadImportOmission::ProjectNetClasses
            | KiCadImportOmission::UnsupportedDesignRule { .. }
    )));
    assert_eq!(imported.layout.stackup.layers.len(), 3);
    assert_eq!(imported.layout.stackup.layers[0].thickness, Real::one());
    assert_eq!(imported.layout.stackup.layers[1].thickness, Real::from(2));
    assert_eq!(imported.layout.stackup.layers[2].thickness, Real::one());
    assert_eq!(imported.layout.rules.net_classes, layout.rules.net_classes);

    let reexport = imported
        .layout
        .export_kicad(&imported.circuit, KiCadExportOptions::default())
        .unwrap();
    let reimported = KiCadImportReport::from_str_with_companions(
        &reexport.board,
        reexport.project.as_deref(),
        reexport.design_rules.as_deref(),
        KiCadImportOptions::new(
            CircuitId::new("stackup-rule-reimport").unwrap(),
            BoardId::new("stackup-rule-reimport").unwrap(),
            Real::from(101),
        ),
    )
    .unwrap();
    assert_eq!(
        reimported.layout.stackup.layers,
        imported.layout.stackup.layers
    );
    assert_eq!(
        reimported.layout.rules.net_classes,
        imported.layout.rules.net_classes
    );
}

#[test]
fn kicad_custom_rules_report_flattened_and_nonrepresentable_class_policy() {
    let (circuit, mut layout) = fixture();
    layout.rules.net_classes.extend([
        NetClass {
            id: NetClassId::new("base").unwrap(),
            parent: None,
            nets: Vec::new(),
            min_trace_width: Some((Real::one() / Real::from(2)).unwrap()),
            preferred_trace_width: None,
            min_clearance: None,
            preferred_via_land_diameter: None,
            preferred_via_drill_diameter: None,
            preferred_via_style: None,
            max_length: None,
            max_via_count: None,
            target_impedance_ohms: None,
            impedance_tolerance_ohms: None,
            requires_reference_plane: false,
        },
        NetClass {
            id: NetClassId::new("controlled").unwrap(),
            parent: Some(NetClassId::new("base").unwrap()),
            nets: vec![NetId::new("SIGNAL").unwrap()],
            min_trace_width: None,
            preferred_trace_width: Some((Real::from(3) / Real::from(4)).unwrap()),
            min_clearance: Some((Real::one() / Real::from(5)).unwrap()),
            preferred_via_land_diameter: Some(Real::one()),
            preferred_via_drill_diameter: Some((Real::one() / Real::from(2)).unwrap()),
            preferred_via_style: None,
            max_length: None,
            max_via_count: None,
            target_impedance_ohms: Some(Real::from(50)),
            impedance_tolerance_ohms: Some(Real::from(5)),
            requires_reference_plane: true,
        },
    ]);
    let export = layout
        .export_kicad(&circuit, KiCadExportOptions::default())
        .unwrap();
    let rules = export.design_rules.as_deref().unwrap();
    assert!(rules.contains("(constraint track_width (min 0.5mm))"));
    assert!(rules.contains("(constraint clearance (min 0.2mm))"));
    assert!(export.omissions.iter().any(|omission| matches!(
        omission,
        KiCadExportOmission::NetClassInheritanceLowered { net_class }
            if net_class == "controlled"
    )));
    let project: serde_json::Value =
        serde_json::from_str(export.project.as_deref().unwrap()).unwrap();
    assert_eq!(
        project["net_settings"]["classes"][1]["track_width"],
        serde_json::json!(0.75)
    );
    assert_eq!(
        project["net_settings"]["classes"][1]["via_diameter"],
        serde_json::json!(1)
    );
    assert_eq!(
        project["net_settings"]["classes"][1]["via_drill"],
        serde_json::json!(0.5)
    );
    assert!(export.omissions.iter().any(|omission| matches!(
        omission,
        KiCadExportOmission::NetClassSignalIntegrityPolicy { net_class }
            if net_class == "controlled"
    )));
    assert!(export.design_rule_projections[0].flattened_inheritance);
}

#[test]
fn kicad_project_import_keeps_native_defaults_and_reports_composite_patterns() {
    let (circuit, mut layout) = fixture();
    layout.rules.net_classes.push(NetClass {
        id: NetClassId::new("power").unwrap(),
        parent: None,
        nets: vec![NetId::new("SIGNAL").unwrap()],
        min_trace_width: Some((Real::from(3) / Real::from(5)).unwrap()),
        preferred_trace_width: Some((Real::from(4) / Real::from(5)).unwrap()),
        min_clearance: Some((Real::one() / Real::from(4)).unwrap()),
        preferred_via_land_diameter: Some((Real::from(6) / Real::from(5)).unwrap()),
        preferred_via_drill_diameter: Some((Real::from(3) / Real::from(5)).unwrap()),
        preferred_via_style: None,
        max_length: None,
        max_via_count: None,
        target_impedance_ohms: None,
        impedance_tolerance_ohms: None,
        requires_reference_plane: false,
    });
    let export = layout
        .export_kicad(&circuit, KiCadExportOptions::default())
        .unwrap();
    let mut project: serde_json::Value =
        serde_json::from_str(export.project.as_deref().unwrap()).unwrap();
    project["net_settings"]["classes"]
        .as_array_mut()
        .unwrap()
        .push(serde_json::json!({"name": "backup", "priority": 1}));
    project["net_settings"]["netclass_assignments"]["SIGNAL"] =
        serde_json::json!(["backup", "power"]);
    project["net_settings"]["netclass_patterns"] =
        serde_json::json!([{"pattern": "G*", "netclass": "backup"}]);
    let project = serde_json::to_string(&project).unwrap();
    let imported = KiCadImportReport::from_str_with_companions(
        &export.board,
        Some(&project),
        None,
        KiCadImportOptions::new(
            CircuitId::new("project-policy-import").unwrap(),
            BoardId::new("project-policy-import").unwrap(),
            Real::from(99),
        ),
    )
    .unwrap();
    let power = imported
        .layout
        .rules
        .net_classes
        .iter()
        .find(|class| class.id.as_str() == "power")
        .unwrap();
    assert_eq!(power.min_trace_width, None);
    assert_eq!(
        power.preferred_trace_width,
        Some((Real::from(4) / Real::from(5)).unwrap())
    );
    assert_eq!(
        power.preferred_via_land_diameter,
        Some((Real::from(6) / Real::from(5)).unwrap())
    );
    assert!(imported.omissions.iter().any(|omission| matches!(
        omission,
        KiCadImportOmission::CompositeProjectNetClassAssignment {
            net,
            net_classes
        } if net == "SIGNAL" && net_classes == &vec!["power".to_owned(), "backup".to_owned()]
    )));
    assert!(imported.omissions.iter().any(|omission| matches!(
        omission,
        KiCadImportOmission::UnsupportedProjectNetClassPattern {
            pattern,
            net_class
        } if pattern == "G*" && net_class == "backup"
    )));
}
