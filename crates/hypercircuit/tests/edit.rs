#![cfg(feature = "interchange")]

use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, BoardSide, BranchId, Bus, BusId, BusSlice, BusSliceId,
    BusSliceOrder, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitModuleParameter,
    CircuitModuleParameterOverride, CircuitModuleParameterTarget, CircuitParameter, CircuitPort,
    ComponentId, CopperZone, DESIGN_HISTORY_VERSION, DesignEdit, DesignEditBatch, DesignEditError,
    DesignEditId, DesignHistory, DesignHistoryAction, DesignHistoryError, DesignRevision,
    DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, EditAddress, EditTarget, KeepoutId,
    KeepoutScope, KiCadExportOptions, KiCadImportOptions, KiCadImportReport, LandPattern,
    LandPatternBody, LandPatternId, LinearStamp, Net, NetId, PcbDesignRules, PcbKeepout, PcbLayout,
    PcbPlacement, PcbRoute, PcbRouteSegment, PcbStackup, PcbVia, PinBinding, PinElectricalKind,
    PinRef, PlacementConstraint, PlacementConstraintId, PlacementConstraintKind, Plating,
    PortDirection, PortId, RailIntent, RailKind, Real, RouteId, SchematicEndpoint, SchematicLabel,
    SchematicLabelId, SchematicLayout, SchematicPoint, SchematicPortPlacement, SchematicSheet,
    SchematicSheetId, SchematicSheetLink, SchematicSheetLinkId, SchematicSheetPort,
    SchematicSheetPortId, SchematicSymbol, SchematicSymbolDefinition, SchematicSymbolDefinitionId,
    SchematicSymbolId, SchematicSymbolUnit, SchematicWire, SchematicWireId, SemanticDocument,
    SourceStimulus, SourceWaveform, StackupLayer, StackupLayerKind, SubcircuitInstance,
    SubcircuitInstanceId, SubcircuitPortBinding, TransientPolicy, ViaId, ViaMaskIntent, ZoneId,
};
use hyperlattice::Point2;
use hyperpath::{LinePathSegment, TraceLayer};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn schematic_definition() -> SchematicSymbolDefinition {
    SchematicSymbolDefinition {
        id: SchematicSymbolDefinitionId::new("editor-symbol").unwrap(),
        model: DeviceModelId::new("editor-part").unwrap(),
        name: "Editor symbol".into(),
        units: vec![SchematicSymbolUnit {
            unit: 1,
            body_width: Real::from(10),
            body_height: Real::from(8),
            pins: Vec::new(),
            graphics: Vec::new(),
        }],
    }
}

fn fixture() -> SemanticDocument {
    let signal = NetId::new("SIGNAL").unwrap();
    let model = DeviceModelId::new("editor-part").unwrap();
    let instance = CircuitInstanceId::new("U1").unwrap();
    let pattern = LandPatternId::new("EDITOR_PATTERN").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("editor-design").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: model.clone(),
        kind: DeviceModelKind::Custom("editor-fixture".into()),
        pins: Vec::new(),
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: instance.clone(),
        component: ComponentId::new("U1").unwrap(),
        part: None,
        model,
        pins: Vec::new(),
        parameters: Vec::new(),
    });
    let layout = PcbLayout {
        id: BoardId::new("editor-board").unwrap(),
        outline: BoardOutline {
            exterior: vec![point(0, 0), point(10, 0), point(10, 10), point(0, 10)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![StackupLayer {
                name: "F.Cu".into(),
                kind: StackupLayerKind::Conductor(TraceLayer(0)),
                thickness: Real::one(),
                material: Some("copper".into()),
            }],
        },
        land_patterns: vec![LandPattern {
            id: pattern.clone(),
            pads: Vec::new(),
            pin_map: Vec::new(),
            graphics: Vec::new(),
            body: None,
            models: Vec::new(),
        }],
        placements: vec![PcbPlacement {
            instance,
            land_pattern: pattern,
            position: point(1, 1),
            rotation_degrees: Real::zero(),
            side: BoardSide::Front,
        }],
        placement_constraints: vec![
            PlacementConstraint {
                id: PlacementConstraintId::new("inside").unwrap(),
                kind: PlacementConstraintKind::Within {
                    instance: CircuitInstanceId::new("U1").unwrap(),
                    min: point(0, 0),
                    max: point(10, 10),
                },
            },
            PlacementConstraint {
                id: PlacementConstraintId::new("rotations").unwrap(),
                kind: PlacementConstraintKind::AllowedRotations {
                    instance: CircuitInstanceId::new("U1").unwrap(),
                    rotations_degrees: vec![Real::zero(), Real::from(90)],
                },
            },
        ],
        routes: vec![PcbRoute {
            id: RouteId::new("route-1").unwrap(),
            net: signal.clone(),
            layer: TraceLayer(0),
            width: Real::one(),
            segments: vec![PcbRouteSegment::Line(LinePathSegment::new(
                point(1, 1),
                point(4, 1),
            ))],
        }],
        vias: vec![PcbVia {
            id: ViaId::new("via-1").unwrap(),
            net: signal.clone(),
            start_layer: TraceLayer(0),
            end_layer: TraceLayer(0),
            center: point(4, 1),
            land_diameter: Real::one(),
            drill_diameter: (Real::one() / Real::from(2)).unwrap(),
            plating: Plating::Plated,
            mask: ViaMaskIntent::tented(),
        }],
        zones: vec![CopperZone::solid(
            ZoneId::new("zone-1").unwrap(),
            signal,
            TraceLayer(0),
            vec![point(1, 1), point(9, 1), point(9, 9), point(1, 9)],
        )],
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };
    SemanticDocument::new(circuit, None)
        .unwrap()
        .with_pcb(layout)
        .unwrap()
}

fn batch(edits: Vec<DesignEdit>) -> DesignEditBatch {
    DesignEditBatch {
        id: DesignEditId::new("editor-batch-1").unwrap(),
        expected_revision: DesignRevision::default(),
        editor: "pcb-viewer".into(),
        edits,
    }
}

fn route(id: &str, start: Point2, end: Point2) -> PcbRoute {
    PcbRoute {
        id: RouteId::new(id).unwrap(),
        net: NetId::new("SIGNAL").unwrap(),
        layer: TraceLayer(0),
        width: Real::one(),
        segments: vec![PcbRouteSegment::Line(LinePathSegment::new(start, end))],
    }
}

fn via(id: &str, center: Point2) -> PcbVia {
    PcbVia {
        id: ViaId::new(id).unwrap(),
        net: NetId::new("SIGNAL").unwrap(),
        start_layer: TraceLayer(0),
        end_layer: TraceLayer(0),
        center,
        land_diameter: Real::one(),
        drill_diameter: (Real::one() / Real::from(2)).unwrap(),
        plating: Plating::Plated,
        mask: ViaMaskIntent::tented(),
    }
}

fn zone(id: &str, offset: i64) -> CopperZone {
    CopperZone::solid(
        ZoneId::new(id).unwrap(),
        NetId::new("SIGNAL").unwrap(),
        TraceLayer(0),
        vec![
            point(offset, offset),
            point(offset + 2, offset),
            point(offset + 2, offset + 2),
            point(offset, offset + 2),
        ],
    )
}

fn placement() -> PcbPlacement {
    fixture().pcb.unwrap().placements.remove(0)
}

fn placement_constraints() -> Vec<PlacementConstraint> {
    fixture().pcb.unwrap().placement_constraints
}

fn land_pattern(id: &str) -> LandPattern {
    LandPattern {
        id: LandPatternId::new(id).unwrap(),
        pads: Vec::new(),
        pin_map: Vec::new(),
        graphics: Vec::new(),
        body: None,
        models: Vec::new(),
    }
}

fn device_model(id: &str) -> DeviceModel {
    DeviceModel {
        id: DeviceModelId::new(id).unwrap(),
        kind: DeviceModelKind::Custom("editor-fixture".into()),
        pins: Vec::new(),
        parameters: Vec::new(),
    }
}

fn circuit_instance(id: &str, model: DeviceModelId) -> CircuitInstance {
    CircuitInstance {
        id: CircuitInstanceId::new(id).unwrap(),
        component: ComponentId::new(id).unwrap(),
        part: None,
        model,
        pins: Vec::new(),
        parameters: Vec::new(),
    }
}

#[test]
fn exact_editor_batch_commits_once_and_survives_json_and_kicad() {
    let mut document = fixture();
    let boundary = vec![point(2, 2), point(8, 2), point(8, 8), point(2, 8)];
    let centerline = vec![
        PcbRouteSegment::Line(LinePathSegment::new(point(1, 1), point(3, 2))),
        PcbRouteSegment::Line(LinePathSegment::new(point(3, 2), point(5, 2))),
    ];
    let edit = batch(vec![
        DesignEdit::SetPlacementTransform {
            instance: CircuitInstanceId::new("U1").unwrap(),
            position: point(2, 3),
            rotation_degrees: Real::from(90),
            side: BoardSide::Front,
        },
        DesignEdit::SetRouteWidth {
            route: RouteId::new("route-1").unwrap(),
            width: (Real::from(3) / Real::from(2)).unwrap(),
        },
        DesignEdit::SetRouteSegments {
            route: RouteId::new("route-1").unwrap(),
            segments: centerline.clone(),
        },
        DesignEdit::MoveVia {
            via: ViaId::new("via-1").unwrap(),
            center: point(5, 2),
        },
        DesignEdit::SetZoneBoundary {
            zone: ZoneId::new("zone-1").unwrap(),
            boundary: boundary.clone(),
        },
    ]);
    let encoded_edit = serde_json::to_string(&edit).unwrap();
    assert_eq!(
        serde_json::from_str::<DesignEditBatch>(&encoded_edit).unwrap(),
        edit
    );

    let report = document.apply_edit_batch(&edit).unwrap();
    assert_eq!(report.from_revision, DesignRevision::new(0));
    assert_eq!(report.to_revision, DesignRevision::new(1));
    assert_eq!(
        report.applied_targets,
        vec![
            EditTarget::Placement(CircuitInstanceId::new("U1").unwrap()),
            EditTarget::Route(RouteId::new("route-1").unwrap()),
            EditTarget::Route(RouteId::new("route-1").unwrap()),
            EditTarget::Via(ViaId::new("via-1").unwrap()),
            EditTarget::Zone(ZoneId::new("zone-1").unwrap()),
        ]
    );
    let pcb = document.pcb.as_ref().unwrap();
    assert_eq!(pcb.placements[0].position, point(2, 3));
    assert_eq!(
        pcb.routes[0].width,
        (Real::from(3) / Real::from(2)).unwrap()
    );
    assert_eq!(pcb.routes[0].segments, centerline);
    assert_eq!(pcb.vias[0].center, point(5, 2));
    assert_eq!(pcb.zones[0].boundary, boundary);

    let json = document.to_json_pretty().unwrap();
    let decoded = SemanticDocument::from_json(&json).unwrap();
    assert_eq!(decoded, document);
    assert_eq!(decoded.design_revision, DesignRevision::new(1));
    let kicad = decoded
        .pcb
        .as_ref()
        .unwrap()
        .export_kicad(&decoded.circuit, KiCadExportOptions::default())
        .unwrap();
    assert!(kicad.board.contains("(width 1.5)"));
    let reimported = KiCadImportReport::from_str(
        &kicad.board,
        KiCadImportOptions::new(
            CircuitId::new("edited-reimport").unwrap(),
            BoardId::new("edited-reimport").unwrap(),
            Real::one(),
        ),
    )
    .unwrap();
    assert!(
        reimported
            .layout
            .routes
            .iter()
            .any(|route| { route.width == (Real::from(3) / Real::from(2)).unwrap() })
    );
}

#[test]
fn stale_or_invalid_batches_leave_the_document_unchanged() {
    let original = fixture();

    let mut stale = original.clone();
    let error = stale
        .apply_edit_batch(&DesignEditBatch {
            expected_revision: DesignRevision::new(7),
            ..batch(vec![DesignEdit::SetRouteWidth {
                route: RouteId::new("route-1").unwrap(),
                width: Real::from(2),
            }])
        })
        .unwrap_err();
    assert!(matches!(error, DesignEditError::RevisionConflict { .. }));
    assert_eq!(stale, original);

    let mut invalid = original.clone();
    let error = invalid
        .apply_edit_batch(&batch(vec![DesignEdit::SetRouteWidth {
            route: RouteId::new("route-1").unwrap(),
            width: Real::zero(),
        }]))
        .unwrap_err();
    assert!(matches!(error, DesignEditError::InvalidResult(_)));
    assert_eq!(invalid, original);

    let mut disconnected = original.clone();
    let error = disconnected
        .apply_edit_batch(&batch(vec![DesignEdit::SetRouteSegments {
            route: RouteId::new("route-1").unwrap(),
            segments: vec![
                PcbRouteSegment::Line(LinePathSegment::new(point(1, 1), point(2, 1))),
                PcbRouteSegment::Line(LinePathSegment::new(point(3, 1), point(4, 1))),
            ],
        }]))
        .unwrap_err();
    assert!(matches!(error, DesignEditError::InvalidResult(_)));
    assert_eq!(disconnected, original);

    let mut missing = original.clone();
    let error = missing
        .apply_edit_batch(&batch(vec![DesignEdit::MoveVia {
            via: ViaId::new("absent").unwrap(),
            center: point(3, 3),
        }]))
        .unwrap_err();
    assert!(matches!(error, DesignEditError::MissingTarget(_)));
    assert_eq!(missing, original);
}

#[test]
fn reversible_history_round_trips_and_keeps_revisions_monotonic() {
    let original = fixture();
    let original_segments = original.pcb.as_ref().unwrap().routes[0].segments.clone();
    let edited_segments = vec![
        PcbRouteSegment::Line(LinePathSegment::new(point(1, 1), point(2, 2))),
        PcbRouteSegment::Line(LinePathSegment::new(point(2, 2), point(4, 1))),
    ];
    let mut history = DesignHistory::new(original).unwrap();
    let authored = DesignEditBatch {
        id: DesignEditId::new("route-shape").unwrap(),
        expected_revision: DesignRevision::new(0),
        editor: "route-tool".into(),
        edits: vec![
            DesignEdit::SetRouteSegments {
                route: RouteId::new("route-1").unwrap(),
                segments: edited_segments.clone(),
            },
            DesignEdit::SetRouteWidth {
                route: RouteId::new("route-1").unwrap(),
                width: Real::from(2),
            },
        ],
    };

    let committed = history.commit(authored).unwrap();
    assert_eq!(committed.action, DesignHistoryAction::Commit);
    assert_eq!(committed.replay.to_revision, DesignRevision::new(1));
    assert_eq!(history.undo_depth(), 1);
    assert_eq!(history.redo_depth(), 0);

    let undone = history.undo().unwrap();
    assert_eq!(undone.action, DesignHistoryAction::Undo);
    assert_eq!(undone.replay.to_revision, DesignRevision::new(2));
    assert_eq!(
        history.document().pcb.as_ref().unwrap().routes[0].segments,
        original_segments
    );
    assert_eq!(
        history.document().pcb.as_ref().unwrap().routes[0].width,
        Real::one()
    );
    assert_eq!((history.undo_depth(), history.redo_depth()), (0, 1));

    let json = history.to_json_pretty().unwrap();
    let mut restored = DesignHistory::from_json(&json).unwrap();
    let redone = restored.redo().unwrap();
    assert_eq!(redone.action, DesignHistoryAction::Redo);
    assert_eq!(redone.replay.to_revision, DesignRevision::new(3));
    assert_eq!(
        restored.document().pcb.as_ref().unwrap().routes[0].segments,
        edited_segments
    );
    assert_eq!(
        restored.document().pcb.as_ref().unwrap().routes[0].width,
        Real::from(2)
    );

    assert_eq!(
        restored.undo().unwrap().replay.to_revision,
        DesignRevision::new(4)
    );
    restored
        .commit(DesignEditBatch {
            id: DesignEditId::new("new-branch").unwrap(),
            expected_revision: DesignRevision::new(4),
            editor: "route-tool".into(),
            edits: vec![DesignEdit::SetRouteWidth {
                route: RouteId::new("route-1").unwrap(),
                width: Real::from(3),
            }],
        })
        .unwrap();
    assert_eq!(restored.document().design_revision, DesignRevision::new(5));
    assert_eq!(restored.redo_depth(), 0);
    assert_eq!(restored.redo(), Err(DesignHistoryError::NothingToRedo));
}

#[test]
fn structural_copper_edits_are_serializable_reversible_and_atomic() {
    let original = fixture();
    let inserted_route = route("route-2", point(2, 3), point(7, 3));
    let inserted_via = via("via-2", point(7, 3));
    let inserted_zone = zone("zone-2", 3);
    let authored = DesignEditBatch {
        id: DesignEditId::new("insert-copper").unwrap(),
        expected_revision: DesignRevision::new(0),
        editor: "interactive-router".into(),
        edits: vec![
            DesignEdit::insert_route(inserted_route.clone()),
            DesignEdit::insert_via(inserted_via.clone()),
            DesignEdit::insert_zone(inserted_zone.clone()),
        ],
    };
    let encoded = serde_json::to_string(&authored).unwrap();
    assert_eq!(
        serde_json::from_str::<DesignEditBatch>(&encoded).unwrap(),
        authored
    );

    let mut history = DesignHistory::new(original.clone()).unwrap();
    let committed = history.commit(authored).unwrap();
    assert_eq!(
        committed.replay.applied_addresses,
        vec![
            EditAddress::RoutePresence(inserted_route.id.clone()),
            EditAddress::ViaPresence(inserted_via.id.clone()),
            EditAddress::ZonePresence(inserted_zone.id.clone()),
        ]
    );
    let pcb = history.document().pcb.as_ref().unwrap();
    assert!(pcb.routes.contains(&inserted_route));
    assert!(pcb.vias.contains(&inserted_via));
    assert!(pcb.zones.contains(&inserted_zone));

    history.undo().unwrap();
    let pcb = history.document().pcb.as_ref().unwrap();
    assert_eq!(pcb.routes, original.pcb.as_ref().unwrap().routes);
    assert_eq!(pcb.vias, original.pcb.as_ref().unwrap().vias);
    assert_eq!(pcb.zones, original.pcb.as_ref().unwrap().zones);

    history.redo().unwrap();
    let pcb = history.document().pcb.as_ref().unwrap();
    assert!(pcb.routes.contains(&inserted_route));
    assert!(pcb.vias.contains(&inserted_via));
    assert!(pcb.zones.contains(&inserted_zone));

    let before = history.document().clone();
    let error = history
        .commit(DesignEditBatch {
            id: DesignEditId::new("duplicate-copper").unwrap(),
            expected_revision: DesignRevision::new(3),
            editor: "interactive-router".into(),
            edits: vec![
                DesignEdit::remove_route(inserted_route.id.clone()),
                DesignEdit::insert_via(inserted_via.clone()),
            ],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::Edit(DesignEditError::ExistingTarget(EditTarget::Via(
            inserted_via.id
        )))
    );
    assert_eq!(history.document(), &before);
}

#[test]
fn structural_placement_edits_are_serializable_reversible_and_atomic() {
    let original = fixture();
    let placed = placement();
    let constraints = placement_constraints();
    let remove = DesignEditBatch {
        id: DesignEditId::new("remove-placement").unwrap(),
        expected_revision: DesignRevision::new(0),
        editor: "placement-tool".into(),
        edits: vec![
            DesignEdit::remove_placement_constraint(constraints[0].id.clone()),
            DesignEdit::remove_placement_constraint(constraints[1].id.clone()),
            DesignEdit::remove_placement(placed.instance.clone()),
        ],
    };
    let encoded = serde_json::to_string(&remove).unwrap();
    assert_eq!(
        serde_json::from_str::<DesignEditBatch>(&encoded).unwrap(),
        remove
    );

    let mut history = DesignHistory::new(original.clone()).unwrap();
    let committed = history.commit(remove).unwrap();
    assert_eq!(
        committed.replay.applied_addresses,
        vec![
            EditAddress::PlacementConstraintPresence(constraints[0].id.clone()),
            EditAddress::PlacementConstraintPresence(constraints[1].id.clone()),
            EditAddress::PlacementPresence(placed.instance.clone()),
        ]
    );
    let pcb = history.document().pcb.as_ref().unwrap();
    assert!(pcb.placements.is_empty());
    assert!(pcb.placement_constraints.is_empty());

    history.undo().unwrap();
    let pcb = history.document().pcb.as_ref().unwrap();
    assert_eq!(pcb.placements, original.pcb.as_ref().unwrap().placements);
    assert_eq!(
        pcb.placement_constraints,
        original.pcb.as_ref().unwrap().placement_constraints
    );

    history.redo().unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("insert-placement").unwrap(),
            expected_revision: DesignRevision::new(3),
            editor: "placement-tool".into(),
            edits: vec![
                DesignEdit::insert_placement(placed.clone()),
                DesignEdit::insert_placement_constraint(constraints[0].clone()),
                DesignEdit::insert_placement_constraint(constraints[1].clone()),
            ],
        })
        .unwrap();
    let pcb = history.document().pcb.as_ref().unwrap();
    assert_eq!(pcb.placements, vec![placed]);
    assert_eq!(pcb.placement_constraints, constraints);

    history.undo().unwrap();
    assert!(
        history
            .document()
            .pcb
            .as_ref()
            .unwrap()
            .placements
            .is_empty()
    );
    history.redo().unwrap();
    assert_eq!(
        history.document().pcb.as_ref().unwrap().placements,
        original.pcb.as_ref().unwrap().placements
    );

    let mut atomic = original.clone();
    let error = atomic
        .apply_edit_batch(&batch(vec![DesignEdit::remove_placement(
            CircuitInstanceId::new("U1").unwrap(),
        )]))
        .unwrap_err();
    assert!(matches!(error, DesignEditError::InvalidResult(_)));
    assert_eq!(atomic, original);
}

#[test]
fn land_pattern_library_edits_are_dependency_aware_and_exactly_reversible() {
    let original = fixture();
    let alternate = land_pattern("ALTERNATE_PATTERN");
    let authored = DesignEditBatch {
        id: DesignEditId::new("swap-land-pattern").unwrap(),
        expected_revision: DesignRevision::new(0),
        editor: "library-tool".into(),
        edits: vec![
            DesignEdit::insert_land_pattern(alternate.clone()),
            DesignEdit::SetPlacementLandPattern {
                instance: CircuitInstanceId::new("U1").unwrap(),
                land_pattern: alternate.id.clone(),
            },
        ],
    };
    let encoded = serde_json::to_string(&authored).unwrap();
    assert_eq!(
        serde_json::from_str::<DesignEditBatch>(&encoded).unwrap(),
        authored
    );

    let mut history = DesignHistory::new(original.clone()).unwrap();
    let committed = history.commit(authored).unwrap();
    assert_eq!(
        committed.replay.applied_addresses,
        vec![
            EditAddress::LandPatternPresence(alternate.id.clone()),
            EditAddress::PlacementLandPattern(CircuitInstanceId::new("U1").unwrap()),
        ]
    );
    let pcb = history.document().pcb.as_ref().unwrap();
    assert_eq!(pcb.land_patterns.len(), 2);
    assert_eq!(pcb.placements[0].land_pattern, alternate.id);

    history.undo().unwrap();
    assert_eq!(history.document().pcb, original.pcb);
    history.redo().unwrap();
    let swapped = history.document().clone();

    let error = history
        .commit(DesignEditBatch {
            id: DesignEditId::new("dangling-pattern").unwrap(),
            expected_revision: DesignRevision::new(3),
            editor: "library-tool".into(),
            edits: vec![DesignEdit::remove_land_pattern(alternate.id.clone())],
        })
        .unwrap_err();
    assert!(matches!(
        error,
        DesignHistoryError::Edit(DesignEditError::InvalidResult(_))
    ));
    assert_eq!(history.document(), &swapped);

    let original_pattern = original.pcb.as_ref().unwrap().land_patterns[0].id.clone();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-and-reassign-pattern").unwrap(),
            expected_revision: DesignRevision::new(3),
            editor: "library-tool".into(),
            edits: vec![
                DesignEdit::remove_land_pattern(alternate.id.clone()),
                DesignEdit::SetPlacementLandPattern {
                    instance: CircuitInstanceId::new("U1").unwrap(),
                    land_pattern: original_pattern.clone(),
                },
            ],
        })
        .unwrap();
    assert_eq!(
        history.document().pcb.as_ref().unwrap().land_patterns,
        vec![land_pattern(original_pattern.as_str())]
    );
    assert_eq!(
        history.document().pcb.as_ref().unwrap().placements[0].land_pattern,
        original_pattern
    );
    history.undo().unwrap();
    assert_eq!(history.document().pcb, swapped.pcb);
}

#[test]
fn land_pattern_presence_conflicts_by_identity_while_assignment_rebases() {
    let alternate = land_pattern("ALTERNATE_PATTERN");
    let mut history = DesignHistory::new(fixture()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("add-pattern").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::insert_land_pattern(alternate.clone())],
        })
        .unwrap();

    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("duplicate-pattern").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::insert_land_pattern(alternate.clone())],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::LandPatternPresence(alternate.id.clone())],
        }
    );

    let rebased = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("use-pattern").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "carol".into(),
            edits: vec![DesignEdit::SetPlacementLandPattern {
                instance: CircuitInstanceId::new("U1").unwrap(),
                land_pattern: alternate.id.clone(),
            }],
        })
        .unwrap();
    assert_eq!(rebased.replay_revision, DesignRevision::new(1));
    assert_eq!(
        history.document().pcb.as_ref().unwrap().placements[0].land_pattern,
        alternate.id
    );
}

#[test]
fn board_configuration_keepouts_and_land_patterns_edit_reversibly() {
    let original = fixture();
    let board = BoardId::new("editor-board").unwrap();
    let mut stackup = original.pcb.as_ref().unwrap().stackup.clone();
    stackup.layers[0].material = Some("edited-copper".into());
    let rules = original.pcb.as_ref().unwrap().rules.clone();
    let mut land_pattern = original.pcb.as_ref().unwrap().land_patterns[0].clone();
    land_pattern.body = Some(LandPatternBody {
        outline: vec![point(0, 0), point(2, 0), point(2, 2), point(0, 2)],
        height: Real::from(2),
        standoff: Real::zero(),
    });
    land_pattern.models = vec![hypercircuit::Pcb3dModelReference {
        uri: "hyperparts:editor-body".into(),
        format: hypercircuit::Pcb3dModelFormat::Step,
        transform: hypercircuit::Pcb3dModelTransform::default(),
    }];
    let keepout = PcbKeepout {
        id: KeepoutId::new("mechanical-reserve").unwrap(),
        boundary: vec![point(6, 6), point(8, 6), point(7, 8)],
        scope: KeepoutScope::Components,
    };
    let changed_outline = BoardOutline {
        exterior: vec![point(-1, -1), point(11, -1), point(11, 11), point(-1, 11)].into(),
        cutouts: Vec::new(),
    };
    let mut history = DesignHistory::new(original.clone()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("board-configuration").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "board-editor".into(),
            edits: vec![
                DesignEdit::SetBoardOutline {
                    board: board.clone(),
                    outline: changed_outline.clone(),
                },
                DesignEdit::SetBoardStackup {
                    board: board.clone(),
                    stackup: stackup.clone(),
                },
                DesignEdit::SetBoardRules {
                    board: board.clone(),
                    rules,
                },
                DesignEdit::SetLandPatternDefinition {
                    land_pattern: Box::new(land_pattern.clone()),
                },
                DesignEdit::insert_keepout(keepout.clone()),
            ],
        })
        .unwrap();
    let edited = history.document().pcb.as_ref().unwrap();
    assert_eq!(edited.outline, changed_outline);
    assert_eq!(edited.stackup, stackup);
    assert_eq!(edited.land_patterns[0], land_pattern);
    assert_eq!(edited.keepouts, vec![keepout]);

    history.undo().unwrap();
    assert_eq!(history.document().pcb, original.pcb);

    let before_invalid = history.document().clone();
    let error = history
        .commit(DesignEditBatch {
            id: DesignEditId::new("empty-stackup").unwrap(),
            expected_revision: DesignRevision::new(2),
            editor: "board-editor".into(),
            edits: vec![DesignEdit::SetBoardStackup {
                board,
                stackup: PcbStackup::default(),
            }],
        })
        .unwrap_err();
    assert!(matches!(
        error,
        DesignHistoryError::Edit(DesignEditError::InvalidResult(_))
    ));
    assert_eq!(history.document(), &before_invalid);
}

#[test]
fn keepout_presence_conflicts_with_stale_definition_edits() {
    let keepout = PcbKeepout {
        id: KeepoutId::new("reserve").unwrap(),
        boundary: vec![point(6, 6), point(8, 6), point(7, 8)],
        scope: KeepoutScope::Components,
    };
    let mut history = DesignHistory::new(fixture()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("insert-keepout").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::insert_keepout(keepout.clone())],
        })
        .unwrap();
    let mut changed = keepout.clone();
    changed.scope = KeepoutScope::Vias;
    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-keepout").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::SetKeepoutDefinition { keepout: changed }],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::KeepoutDefinition(keepout.id)],
        }
    );
}

#[test]
fn complete_circuit_and_copper_definitions_edit_as_one_atomic_batch() {
    let original = fixture();
    let mut model = original.circuit.device_models[0].clone();
    model.kind = DeviceModelKind::Custom("edited-model".into());
    let mut instance = original.circuit.instances[0].clone();
    instance.parameters = vec![CircuitParameter {
        name: "gain".into(),
        value: Real::from(2),
        unit: "ratio".into(),
        source: "editor".into(),
    }];
    let pcb = original.pcb.as_ref().unwrap();
    let mut constraint = pcb.placement_constraints[1].clone();
    constraint.kind = PlacementConstraintKind::AllowedRotations {
        instance: CircuitInstanceId::new("U1").unwrap(),
        rotations_degrees: vec![Real::zero()],
    };
    let mut route = pcb.routes[0].clone();
    route.width = Real::from(2);
    let mut via = pcb.vias[0].clone();
    via.land_diameter = Real::from(2);
    via.drill_diameter = Real::one();
    let mut zone = pcb.zones[0].clone();
    zone.clearance = Real::one();
    zone.priority = 2;

    let authored = DesignEditBatch {
        id: DesignEditId::new("replace-complete-definitions").unwrap(),
        expected_revision: DesignRevision::new(0),
        editor: "semantic-editor".into(),
        edits: vec![
            DesignEdit::SetDeviceModelDefinition {
                model: Box::new(model.clone()),
            },
            DesignEdit::SetCircuitInstanceDefinition {
                instance: Box::new(instance.clone()),
            },
            DesignEdit::SetPlacementConstraintDefinition {
                constraint: Box::new(constraint.clone()),
            },
            DesignEdit::SetRouteDefinition {
                route: Box::new(route.clone()),
            },
            DesignEdit::SetViaDefinition {
                via: Box::new(via.clone()),
            },
            DesignEdit::SetZoneDefinition {
                zone: Box::new(zone.clone()),
            },
        ],
    };
    assert_eq!(
        serde_json::from_str::<DesignEditBatch>(&serde_json::to_string(&authored).unwrap())
            .unwrap(),
        authored
    );

    let mut history = DesignHistory::new(original.clone()).unwrap();
    history.commit(authored).unwrap();
    assert_eq!(history.document().circuit.device_models[0], model);
    assert_eq!(history.document().circuit.instances[0], instance);
    let edited = history.document().pcb.as_ref().unwrap();
    assert_eq!(edited.placement_constraints[1], constraint);
    assert_eq!(edited.routes[0], route);
    assert_eq!(edited.vias[0], via);
    assert_eq!(edited.zones[0], zone);

    history.undo().unwrap();
    assert_eq!(history.document().circuit, original.circuit);
    assert_eq!(history.document().pcb, original.pcb);

    let mut invalid_via = original.pcb.as_ref().unwrap().vias[0].clone();
    invalid_via.drill_diameter = Real::from(3);
    let before_invalid = history.document().clone();
    let error = history
        .commit(DesignEditBatch {
            id: DesignEditId::new("invalid-complete-via").unwrap(),
            expected_revision: DesignRevision::new(2),
            editor: "semantic-editor".into(),
            edits: vec![DesignEdit::SetViaDefinition {
                via: Box::new(invalid_via),
            }],
        })
        .unwrap_err();
    assert!(matches!(
        error,
        DesignHistoryError::Edit(DesignEditError::InvalidResult(_))
    ));
    assert_eq!(history.document(), &before_invalid);
}

#[test]
fn complete_route_definition_conflicts_with_stale_narrow_writes() {
    let original = fixture();
    let route_id = RouteId::new("route-1").unwrap();
    let mut replacement = original.pcb.as_ref().unwrap().routes[0].clone();
    replacement.width = Real::from(3);
    let mut history = DesignHistory::new(original).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("narrow-route-width").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::SetRouteWidth {
                route: route_id.clone(),
                width: Real::from(2),
            }],
        })
        .unwrap();
    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-complete-route").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::SetRouteDefinition {
                route: Box::new(replacement),
            }],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::RouteDefinition(route_id)],
        }
    );
}

#[test]
fn circuit_library_edits_work_without_a_pcb_and_round_trip_history() {
    let source = fixture();
    let mut document = SemanticDocument::new(source.circuit.clone(), None).unwrap();
    let alternate = device_model("alternate-model");
    let authored = DesignEditBatch {
        id: DesignEditId::new("circuit-only-model-swap").unwrap(),
        expected_revision: DesignRevision::new(0),
        editor: "circuit-editor".into(),
        edits: vec![
            DesignEdit::insert_device_model(alternate.clone()),
            DesignEdit::SetCircuitInstanceModel {
                instance: CircuitInstanceId::new("U1").unwrap(),
                model: alternate.id.clone(),
            },
        ],
    };
    let encoded = serde_json::to_string(&authored).unwrap();
    assert_eq!(
        serde_json::from_str::<DesignEditBatch>(&encoded).unwrap(),
        authored
    );

    let report = document.apply_edit_batch(&authored).unwrap();
    assert_eq!(
        report.applied_addresses,
        vec![
            EditAddress::DeviceModelPresence(alternate.id.clone()),
            EditAddress::CircuitInstanceModel(CircuitInstanceId::new("U1").unwrap()),
        ]
    );
    assert_eq!(document.circuit.device_models.len(), 2);
    assert_eq!(document.circuit.instances[0].model, alternate.id);
    assert!(document.pcb.is_none());
}

#[test]
fn pcb_container_attaches_to_circuit_only_documents_and_undoes_exactly() {
    let source = fixture();
    let original = SemanticDocument::new(source.circuit.clone(), None).unwrap();
    let pcb = source.pcb.clone().unwrap();
    let authored = DesignEditBatch {
        id: DesignEditId::new("attach-pcb").unwrap(),
        expected_revision: DesignRevision::new(0),
        editor: "board-editor".into(),
        edits: vec![DesignEdit::insert_pcb(pcb.clone())],
    };
    assert_eq!(
        serde_json::from_str::<DesignEditBatch>(&serde_json::to_string(&authored).unwrap())
            .unwrap(),
        authored
    );

    let mut history = DesignHistory::new(original.clone()).unwrap();
    history.commit(authored).unwrap();
    assert_eq!(history.document().pcb, Some(pcb.clone()));
    history.undo().unwrap();
    assert_eq!(history.document().pcb, None);
    assert_eq!(history.document().circuit, original.circuit);
    history.redo().unwrap();
    assert_eq!(history.document().pcb, Some(pcb.clone()));
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("detach-pcb").unwrap(),
            expected_revision: DesignRevision::new(3),
            editor: "board-editor".into(),
            edits: vec![DesignEdit::remove_pcb(pcb.id.clone())],
        })
        .unwrap();
    assert!(history.document().pcb.is_none());
    history.undo().unwrap();
    assert_eq!(history.document().pcb, Some(pcb));
}

#[test]
fn pcb_container_presence_conflicts_with_stale_nested_board_edits() {
    let board = BoardId::new("editor-board").unwrap();
    let route = RouteId::new("route-1").unwrap();
    let mut history = DesignHistory::new(fixture()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-pcb").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::remove_pcb(board)],
        })
        .unwrap();
    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-route-width").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::SetRouteWidth {
                route: route.clone(),
                width: Real::from(2),
            }],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::RouteWidth(route)],
        }
    );
}

#[test]
fn circuit_only_net_and_pin_binding_edits_are_reversible() {
    let mut circuit = fixture().circuit;
    circuit.device_models[0].pins.push(DevicePin {
        pin: PinRef::new("P1").unwrap(),
        kind: PinElectricalKind::Passive,
        optional: true,
    });
    let original = SemanticDocument::new(circuit, None).unwrap();
    let alternate = Net {
        id: NetId::new("ALTERNATE").unwrap(),
        is_ground: false,
    };
    let authored = DesignEditBatch {
        id: DesignEditId::new("connect-pin").unwrap(),
        expected_revision: DesignRevision::new(0),
        editor: "circuit-editor".into(),
        edits: vec![
            DesignEdit::insert_net(alternate.clone()),
            DesignEdit::SetNetGround {
                net: alternate.id.clone(),
                is_ground: true,
            },
            DesignEdit::SetCircuitInstancePins {
                instance: CircuitInstanceId::new("U1").unwrap(),
                pins: vec![PinBinding {
                    pin: PinRef::new("P1").unwrap(),
                    net: alternate.id.clone(),
                }],
            },
        ],
    };
    let encoded = serde_json::to_string(&authored).unwrap();
    assert_eq!(
        serde_json::from_str::<DesignEditBatch>(&encoded).unwrap(),
        authored
    );

    let mut history = DesignHistory::new(original.clone()).unwrap();
    let committed = history.commit(authored).unwrap();
    assert_eq!(
        committed.replay.applied_addresses,
        vec![
            EditAddress::NetPresence(alternate.id.clone()),
            EditAddress::NetGround(alternate.id.clone()),
            EditAddress::CircuitInstancePins(CircuitInstanceId::new("U1").unwrap()),
        ]
    );
    assert!(history.document().circuit.nets[1].is_ground);
    assert_eq!(
        history.document().circuit.instances[0].pins[0].net,
        alternate.id
    );
    history.undo().unwrap();
    assert_eq!(history.document().circuit, original.circuit);
    history.redo().unwrap();

    let before = history.document().clone();
    let error = history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-connected-net").unwrap(),
            expected_revision: DesignRevision::new(3),
            editor: "circuit-editor".into(),
            edits: vec![DesignEdit::remove_net(alternate.id)],
        })
        .unwrap_err();
    assert!(matches!(
        error,
        DesignHistoryError::Edit(DesignEditError::InvalidResult(_))
    ));
    assert_eq!(history.document(), &before);
}

#[test]
fn pcb_copper_net_reassignment_is_atomic_and_dependency_checked() {
    let original = fixture();
    let alternate = Net {
        id: NetId::new("ALTERNATE").unwrap(),
        is_ground: false,
    };
    let mut history = DesignHistory::new(original).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("reassign-copper-net").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "connectivity-tool".into(),
            edits: vec![
                DesignEdit::insert_net(alternate.clone()),
                DesignEdit::SetRouteNet {
                    route: RouteId::new("route-1").unwrap(),
                    net: alternate.id.clone(),
                },
                DesignEdit::SetViaNet {
                    via: ViaId::new("via-1").unwrap(),
                    net: alternate.id.clone(),
                },
                DesignEdit::SetZoneNet {
                    zone: ZoneId::new("zone-1").unwrap(),
                    net: alternate.id.clone(),
                },
            ],
        })
        .unwrap();
    let reassigned = history.document().clone();
    let pcb = reassigned.pcb.as_ref().unwrap();
    assert_eq!(pcb.routes[0].net, alternate.id);
    assert_eq!(pcb.vias[0].net, alternate.id);
    assert_eq!(pcb.zones[0].net, alternate.id);

    let error = history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-routed-net").unwrap(),
            expected_revision: DesignRevision::new(1),
            editor: "connectivity-tool".into(),
            edits: vec![DesignEdit::remove_net(alternate.id.clone())],
        })
        .unwrap_err();
    assert!(matches!(
        error,
        DesignHistoryError::Edit(DesignEditError::InvalidResult(_))
    ));
    assert_eq!(history.document(), &reassigned);

    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-and-reassign-net").unwrap(),
            expected_revision: DesignRevision::new(1),
            editor: "connectivity-tool".into(),
            edits: vec![
                DesignEdit::remove_net(alternate.id.clone()),
                DesignEdit::SetRouteNet {
                    route: RouteId::new("route-1").unwrap(),
                    net: NetId::new("SIGNAL").unwrap(),
                },
                DesignEdit::SetViaNet {
                    via: ViaId::new("via-1").unwrap(),
                    net: NetId::new("SIGNAL").unwrap(),
                },
                DesignEdit::SetZoneNet {
                    zone: ZoneId::new("zone-1").unwrap(),
                    net: NetId::new("SIGNAL").unwrap(),
                },
            ],
        })
        .unwrap();
    assert_eq!(history.document().circuit.nets.len(), 1);
    history.undo().unwrap();
    assert_eq!(history.document().circuit, reassigned.circuit);
    assert_eq!(history.document().pcb, reassigned.pcb);
}

#[test]
fn net_presence_conflicts_with_stale_net_property_writes() {
    let alternate = Net {
        id: NetId::new("ALTERNATE").unwrap(),
        is_ground: false,
    };
    let mut history = DesignHistory::new(fixture()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("insert-net").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::insert_net(alternate.clone())],
        })
        .unwrap();

    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-net-ground").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::SetNetGround {
                net: alternate.id.clone(),
                is_ground: true,
            }],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::NetGround(alternate.id)],
        }
    );
}

#[test]
fn bus_slice_and_port_edits_are_dependency_checked_and_reversible() {
    let source = fixture();
    let original = SemanticDocument::new(source.circuit, None).unwrap();
    let alternate = Net {
        id: NetId::new("ALTERNATE").unwrap(),
        is_ground: false,
    };
    let bus = Bus {
        id: BusId::new("DATA").unwrap(),
        nets: vec![NetId::new("SIGNAL").unwrap(), alternate.id.clone()],
    };
    let slice = BusSlice {
        id: BusSliceId::new("DATA_HIGH").unwrap(),
        bus: bus.id.clone(),
        offset: 1,
        width: 1,
        order: BusSliceOrder::Reverse,
    };
    let port = CircuitPort {
        id: PortId::new("data").unwrap(),
        net: alternate.id.clone(),
        direction: PortDirection::Input,
        optional: false,
    };
    let authored = DesignEditBatch {
        id: DesignEditId::new("add-interface").unwrap(),
        expected_revision: DesignRevision::new(0),
        editor: "interface-editor".into(),
        edits: vec![
            DesignEdit::insert_net(alternate.clone()),
            DesignEdit::insert_bus(bus.clone()),
            DesignEdit::insert_bus_slice(slice.clone()),
            DesignEdit::insert_circuit_port(port.clone()),
        ],
    };
    let encoded = serde_json::to_string(&authored).unwrap();
    assert_eq!(
        serde_json::from_str::<DesignEditBatch>(&encoded).unwrap(),
        authored
    );

    let mut history = DesignHistory::new(original.clone()).unwrap();
    let committed = history.commit(authored).unwrap();
    assert_eq!(
        committed.replay.applied_addresses,
        vec![
            EditAddress::NetPresence(alternate.id.clone()),
            EditAddress::BusPresence(bus.id.clone()),
            EditAddress::BusSlicePresence(slice.id.clone()),
            EditAddress::CircuitPortPresence(port.id.clone()),
        ]
    );
    history.undo().unwrap();
    assert_eq!(history.document().circuit, original.circuit);
    history.redo().unwrap();
    let inserted = history.document().clone();

    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("change-interface").unwrap(),
            expected_revision: DesignRevision::new(3),
            editor: "interface-editor".into(),
            edits: vec![
                DesignEdit::SetBusNets {
                    bus: bus.id.clone(),
                    nets: vec![alternate.id.clone(), NetId::new("SIGNAL").unwrap()],
                },
                DesignEdit::SetBusSliceDefinition {
                    slice: slice.id.clone(),
                    bus: bus.id.clone(),
                    offset: 0,
                    width: 2,
                    order: BusSliceOrder::Forward,
                },
                DesignEdit::SetCircuitPortDefinition {
                    port: port.id.clone(),
                    net: NetId::new("SIGNAL").unwrap(),
                    direction: PortDirection::Output,
                    optional: true,
                },
            ],
        })
        .unwrap();
    assert_eq!(
        history.document().circuit.buses[0].nets,
        vec![alternate.id.clone(), NetId::new("SIGNAL").unwrap()]
    );
    assert_eq!(history.document().circuit.bus_slices[0].width, 2);
    assert!(history.document().circuit.ports[0].optional);
    history.undo().unwrap();
    assert_eq!(history.document().circuit, inserted.circuit);

    let before = history.document().clone();
    let error = history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-referenced-bus").unwrap(),
            expected_revision: DesignRevision::new(5),
            editor: "interface-editor".into(),
            edits: vec![DesignEdit::remove_bus(bus.id.clone())],
        })
        .unwrap_err();
    assert!(matches!(
        error,
        DesignHistoryError::Edit(DesignEditError::InvalidResult(_))
    ));
    assert_eq!(history.document(), &before);

    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-bus-and-slice").unwrap(),
            expected_revision: DesignRevision::new(5),
            editor: "interface-editor".into(),
            edits: vec![
                DesignEdit::remove_bus(bus.id),
                DesignEdit::remove_bus_slice(slice.id),
            ],
        })
        .unwrap();
    assert!(history.document().circuit.buses.is_empty());
    assert!(history.document().circuit.bus_slices.is_empty());
    history.undo().unwrap();
    assert_eq!(history.document().circuit, before.circuit);
}

#[test]
fn bus_presence_conflicts_with_stale_membership_edits() {
    let bus = Bus {
        id: BusId::new("DATA").unwrap(),
        nets: vec![NetId::new("SIGNAL").unwrap()],
    };
    let mut history = DesignHistory::new(fixture()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("insert-bus").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::insert_bus(bus.clone())],
        })
        .unwrap();
    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-bus-members").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::SetBusNets {
                bus: bus.id.clone(),
                nets: vec![NetId::new("SIGNAL").unwrap()],
            }],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::BusNets(bus.id)],
        }
    );
}

#[test]
fn remaining_circuit_collections_edit_atomically_and_reversibly() {
    let signal = NetId::new("SIGNAL").unwrap();
    let model_id = DeviceModelId::new("source-model").unwrap();
    let component = ComponentId::new("SRC").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("editable-hierarchy").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: model_id.clone(),
        kind: DeviceModelKind::VoltageSource,
        pins: Vec::new(),
        parameters: vec![CircuitParameter {
            name: "level".into(),
            value: Real::one(),
            unit: "V".into(),
            source: "model-default".into(),
        }],
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("VSRC").unwrap(),
        component: component.clone(),
        part: None,
        model: model_id.clone(),
        pins: Vec::new(),
        parameters: Vec::new(),
    });
    let original = SemanticDocument::new(circuit, None).unwrap();
    let rail = RailIntent {
        net: signal.clone(),
        nominal_voltage: Some(Real::one()),
        max_current: Some(Real::one()),
        kind: RailKind::Power,
    };
    let parameter = CircuitModuleParameter {
        name: "output_level".into(),
        default: Real::one(),
        unit: "V".into(),
        source: "module-default".into(),
        targets: vec![CircuitModuleParameterTarget::ModelParameter {
            model: model_id.clone(),
            parameter: "level".into(),
        }],
    };
    let stimulus = SourceStimulus {
        component: component.clone(),
        waveform: SourceWaveform::Constant(Real::one()),
    };
    let subcircuit = SubcircuitInstance {
        id: SubcircuitInstanceId::new("FILTER").unwrap(),
        circuit: CircuitId::new("filter-v1").unwrap(),
        ports: vec![SubcircuitPortBinding {
            port: PortId::new("input").unwrap(),
            net: signal.clone(),
        }],
        parameter_overrides: Vec::new(),
    };

    let mut history = DesignHistory::new(original.clone()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("add-circuit-intent").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "circuit-editor".into(),
            edits: vec![
                DesignEdit::insert_rail(rail.clone()),
                DesignEdit::insert_module_parameter(parameter.clone()),
                DesignEdit::insert_source_stimulus(stimulus.clone()),
                DesignEdit::insert_subcircuit(subcircuit.clone()),
            ],
        })
        .unwrap();
    let inserted = history.document().clone();
    history.undo().unwrap();
    assert_eq!(history.document().circuit, original.circuit);
    history.redo().unwrap();
    assert_eq!(history.document().circuit, inserted.circuit);

    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("change-circuit-intent").unwrap(),
            expected_revision: DesignRevision::new(3),
            editor: "circuit-editor".into(),
            edits: vec![
                DesignEdit::SetRailDefinition {
                    net: signal.clone(),
                    nominal_voltage: Some(Real::from(2)),
                    max_current: Some(Real::from(2)),
                    kind: RailKind::Reference,
                },
                DesignEdit::SetModuleParameterDefinition {
                    parameter: parameter.name.clone(),
                    default: Real::from(2),
                    unit: "V".into(),
                    source: "editor".into(),
                    targets: parameter.targets.clone(),
                },
                DesignEdit::SetSourceStimulusWaveform {
                    component: component.clone(),
                    waveform: SourceWaveform::Step {
                        initial: Real::zero(),
                        final_value: Real::from(2),
                        at: Real::one(),
                    },
                },
                DesignEdit::SetSubcircuitDefinition {
                    subcircuit: subcircuit.id.clone(),
                    circuit: CircuitId::new("filter-v2").unwrap(),
                    ports: subcircuit.ports.clone(),
                    parameter_overrides: vec![CircuitModuleParameterOverride {
                        parameter: "cutoff".into(),
                        value: Real::from(2),
                        source: "editor".into(),
                    }],
                },
                DesignEdit::SetCircuitPolicy {
                    transient_policy: TransientPolicy::GearBdf { order: 2 },
                    adapter_policy: AdapterKind::TransientDae,
                },
            ],
        })
        .unwrap();
    assert_eq!(
        history.document().circuit.rails[0].kind,
        RailKind::Reference
    );
    assert_eq!(
        history.document().circuit.adapter_policy,
        AdapterKind::TransientDae
    );
    history.undo().unwrap();
    assert_eq!(history.document().circuit, inserted.circuit);

    let before_invalid = history.document().clone();
    let error = history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-referenced-net").unwrap(),
            expected_revision: DesignRevision::new(5),
            editor: "circuit-editor".into(),
            edits: vec![DesignEdit::remove_net(signal)],
        })
        .unwrap_err();
    assert!(matches!(
        error,
        DesignHistoryError::Edit(DesignEditError::InvalidResult(_))
    ));
    assert_eq!(history.document(), &before_invalid);
}

#[test]
fn identityless_manual_stamps_replace_as_one_exact_reversible_field() {
    let original = fixture();
    let stamps = vec![LinearStamp::VoltageSource {
        component: ComponentId::new("MANUAL_SOURCE").unwrap(),
        branch: BranchId::new("manual-branch").unwrap(),
        pos: Some(NetId::new("SIGNAL").unwrap()),
        neg: None,
        voltage: Real::from(5),
    }];
    let mut history = DesignHistory::new(original.clone()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("manual-stamps").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "simulation-editor".into(),
            edits: vec![DesignEdit::SetLinearStamps {
                stamps: stamps.clone(),
            }],
        })
        .unwrap();
    assert_eq!(history.document().circuit.stamps, stamps);
    history.undo().unwrap();
    assert_eq!(history.document().circuit.stamps, original.circuit.stamps);
    history.redo().unwrap();
    assert_eq!(history.document().circuit.stamps, stamps);
}

#[test]
fn rail_presence_conflicts_with_stale_definition_edits() {
    let signal = NetId::new("SIGNAL").unwrap();
    let mut history = DesignHistory::new(fixture()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("insert-rail").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::insert_rail(RailIntent {
                net: signal.clone(),
                nominal_voltage: Some(Real::one()),
                max_current: Some(Real::one()),
                kind: RailKind::Power,
            })],
        })
        .unwrap();
    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-rail-definition").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::SetRailDefinition {
                net: signal.clone(),
                nominal_voltage: Some(Real::from(2)),
                max_current: Some(Real::one()),
                kind: RailKind::Reference,
            }],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::RailDefinition(signal)],
        }
    );
}

#[test]
fn circuit_port_removal_observes_schematic_references() {
    let mut circuit = fixture().circuit;
    let port = CircuitPort {
        id: PortId::new("data").unwrap(),
        net: NetId::new("SIGNAL").unwrap(),
        direction: PortDirection::Bidirectional,
        optional: false,
    };
    circuit.ports.push(port.clone());
    let schematic = SchematicLayout {
        ports: vec![SchematicPortPlacement {
            port: port.id.clone(),
            position: SchematicPoint::new(Real::zero(), Real::zero()),
        }],
        ..SchematicLayout::default()
    };
    let mut document = SemanticDocument::new(circuit, Some(schematic)).unwrap();
    let original = document.clone();
    let error = document
        .apply_edit_batch(&batch(vec![DesignEdit::remove_circuit_port(port.id)]))
        .unwrap_err();
    assert!(matches!(error, DesignEditError::InvalidResult(_)));
    assert_eq!(document, original);
}

#[test]
fn schematic_structure_edits_build_hierarchy_and_undo_to_no_schematic() {
    let original = fixture();
    assert!(original.schematic.is_none());

    let signal = NetId::new("SIGNAL").unwrap();
    let circuit_port = CircuitPort {
        id: PortId::new("data").unwrap(),
        net: signal.clone(),
        direction: PortDirection::Bidirectional,
        optional: false,
    };
    let symbol = SchematicSymbol {
        id: SchematicSymbolId::new("U1:A").unwrap(),
        instance: CircuitInstanceId::new("U1").unwrap(),
        definition: SchematicSymbolDefinitionId::new("editor-symbol").unwrap(),
        unit: 1,
        position: SchematicPoint::new(Real::from(30), Real::from(10)),
        quarter_turns: 0,
    };
    let placement = SchematicPortPlacement {
        port: circuit_port.id.clone(),
        position: SchematicPoint::new(Real::zero(), Real::from(10)),
    };
    let root = SchematicSheetId::new("root").unwrap();
    let child = SchematicSheetId::new("child").unwrap();
    let root_port = SchematicSheetPort {
        id: SchematicSheetPortId::new("root-data").unwrap(),
        sheet: root.clone(),
        net: signal.clone(),
        name: "DATA".into(),
        position: SchematicPoint::new(Real::from(20), Real::from(10)),
    };
    let child_port = SchematicSheetPort {
        id: SchematicSheetPortId::new("child-data").unwrap(),
        sheet: child.clone(),
        net: signal.clone(),
        name: "DATA".into(),
        position: SchematicPoint::new(Real::zero(), Real::from(10)),
    };
    let root_wire = SchematicWire {
        id: SchematicWireId::new("root-data-wire").unwrap(),
        net: signal.clone(),
        from: SchematicEndpoint::Port(circuit_port.id.clone()),
        waypoints: Vec::new(),
        to: SchematicEndpoint::SheetPort(root_port.id.clone()),
    };
    let child_wire = SchematicWire {
        id: SchematicWireId::new("child-data-wire").unwrap(),
        net: signal.clone(),
        from: SchematicEndpoint::SheetPort(child_port.id.clone()),
        waypoints: Vec::new(),
        to: SchematicEndpoint::Junction(SchematicPoint::new(Real::from(20), Real::from(10))),
    };
    let label = SchematicLabel {
        id: SchematicLabelId::new("child-data-label").unwrap(),
        net: signal,
        position: SchematicPoint::new(Real::from(15), Real::from(10)),
        text: "DATA".into(),
    };
    let root_sheet = SchematicSheet {
        id: root.clone(),
        title: "Root".into(),
        parent: None,
        symbols: Vec::new(),
        ports: vec![circuit_port.id.clone()],
        wires: vec![root_wire.id.clone()],
        labels: Vec::new(),
    };
    let child_sheet = SchematicSheet {
        id: child.clone(),
        title: "Child".into(),
        parent: Some(root),
        symbols: vec![symbol.id.clone()],
        ports: Vec::new(),
        wires: vec![child_wire.id.clone()],
        labels: vec![label.id.clone()],
    };
    let link = SchematicSheetLink {
        id: SchematicSheetLinkId::new("root-to-child").unwrap(),
        parent_port: root_port.id.clone(),
        child_port: child_port.id.clone(),
    };

    let authored = DesignEditBatch {
        id: DesignEditId::new("build-schematic").unwrap(),
        expected_revision: DesignRevision::new(0),
        editor: "schematic-editor".into(),
        edits: vec![
            DesignEdit::insert_circuit_port(circuit_port),
            DesignEdit::insert_schematic(SchematicLayout::default()),
            DesignEdit::insert_schematic_symbol_library(schematic_definition()),
            DesignEdit::insert_schematic_symbol(symbol),
            DesignEdit::insert_schematic_port_placement(placement),
            DesignEdit::insert_schematic_wire(root_wire),
            DesignEdit::insert_schematic_wire(child_wire),
            DesignEdit::insert_schematic_label(label),
            DesignEdit::insert_schematic_sheet(root_sheet),
            DesignEdit::insert_schematic_sheet(child_sheet),
            DesignEdit::insert_schematic_sheet_port(root_port),
            DesignEdit::insert_schematic_sheet_port(child_port),
            DesignEdit::insert_schematic_sheet_link(link),
        ],
    };
    assert_eq!(
        serde_json::from_str::<DesignEditBatch>(&serde_json::to_string(&authored).unwrap())
            .unwrap(),
        authored
    );

    let mut history = DesignHistory::new(original.clone()).unwrap();
    history.commit(authored).unwrap();
    let built = history.document().clone();
    let schematic = built.schematic.as_ref().unwrap();
    assert_eq!(schematic.symbol_definitions.len(), 1);
    assert_eq!(schematic.symbols.len(), 1);
    assert_eq!(schematic.wires.len(), 2);
    assert_eq!(schematic.sheets.len(), 2);
    assert_eq!(schematic.sheet_ports.len(), 2);
    assert_eq!(schematic.sheet_links.len(), 1);

    history.undo().unwrap();
    assert_eq!(history.document().circuit, original.circuit);
    assert_eq!(history.document().schematic, original.schematic);
    assert_eq!(history.document().pcb, original.pcb);
    history.redo().unwrap();
    assert_eq!(history.document().schematic, built.schematic);

    let built_schematic = built.schematic.as_ref().unwrap();
    let mut changed_symbol = built_schematic.symbols[0].clone();
    changed_symbol.position = SchematicPoint::new(Real::from(31), Real::from(11));
    changed_symbol.quarter_turns = 1;
    let mut changed_library = built_schematic.symbol_definitions[0].clone();
    changed_library.name = "Edited editor symbol".into();
    let mut changed_port_placement = built_schematic.ports[0].clone();
    changed_port_placement.position = SchematicPoint::new(Real::one(), Real::from(11));
    let mut changed_wire = built_schematic.wires[0].clone();
    changed_wire.waypoints = vec![SchematicPoint::new(Real::from(10), Real::from(12))];
    let mut changed_label = built_schematic.labels[0].clone();
    changed_label.position = SchematicPoint::new(Real::from(16), Real::from(11));
    changed_label.text = "DATA_BUS".into();
    let mut changed_sheet = built_schematic.sheets[0].clone();
    changed_sheet.title = "Root Interface".into();
    let mut changed_sheet_port = built_schematic.sheet_ports[0].clone();
    changed_sheet_port.name = "DATA_IN".into();
    changed_sheet_port.position = SchematicPoint::new(Real::from(21), Real::from(11));
    let unchanged_link = built_schematic.sheet_links[0].clone();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("edit-schematic-definitions").unwrap(),
            expected_revision: DesignRevision::new(3),
            editor: "schematic-editor".into(),
            edits: vec![
                DesignEdit::SetSchematicSymbolDefinition {
                    symbol: Box::new(changed_symbol),
                },
                DesignEdit::SetSchematicSymbolLibraryDefinition {
                    definition: Box::new(changed_library),
                },
                DesignEdit::SetSchematicPortPlacementDefinition {
                    placement: changed_port_placement,
                },
                DesignEdit::SetSchematicWireDefinition {
                    wire: Box::new(changed_wire),
                },
                DesignEdit::SetSchematicLabelDefinition {
                    label: changed_label,
                },
                DesignEdit::SetSchematicSheetDefinition {
                    sheet: changed_sheet,
                },
                DesignEdit::SetSchematicSheetPortDefinition {
                    port: changed_sheet_port,
                },
                DesignEdit::SetSchematicSheetLinkDefinition {
                    link: unchanged_link,
                },
            ],
        })
        .unwrap();
    let edited_schematic = history.document().schematic.as_ref().unwrap();
    assert_eq!(edited_schematic.symbols[0].quarter_turns, 1);
    assert_eq!(edited_schematic.wires[0].waypoints.len(), 1);
    assert_eq!(edited_schematic.sheets[0].title, "Root Interface");
    history.undo().unwrap();
    assert_eq!(history.document().schematic, built.schematic);

    let before_invalid = history.document().clone();
    let error = history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-referenced-symbol").unwrap(),
            expected_revision: DesignRevision::new(5),
            editor: "schematic-editor".into(),
            edits: vec![DesignEdit::remove_schematic_symbol(
                SchematicSymbolId::new("U1:A").unwrap(),
            )],
        })
        .unwrap_err();
    assert!(matches!(
        error,
        DesignHistoryError::Edit(DesignEditError::InvalidResult(_))
    ));
    assert_eq!(history.document(), &before_invalid);

    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-schematic").unwrap(),
            expected_revision: DesignRevision::new(5),
            editor: "schematic-editor".into(),
            edits: vec![DesignEdit::remove_schematic()],
        })
        .unwrap();
    assert!(history.document().schematic.is_none());
    history.undo().unwrap();
    assert_eq!(history.document().schematic, built.schematic);
}

#[test]
fn schematic_container_presence_conflicts_with_stale_nested_edits() {
    let mut original = fixture();
    original.schematic = Some(SchematicLayout::default());
    let mut history = DesignHistory::new(original).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-schematic").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::remove_schematic()],
        })
        .unwrap();

    let symbol = SchematicSymbol {
        id: SchematicSymbolId::new("U1:A").unwrap(),
        instance: CircuitInstanceId::new("U1").unwrap(),
        definition: SchematicSymbolDefinitionId::new("editor-symbol").unwrap(),
        unit: 1,
        position: SchematicPoint::new(Real::zero(), Real::zero()),
        quarter_turns: 0,
    };
    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-symbol").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::insert_schematic_symbol(symbol)],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::SchematicSymbolPresence(
                SchematicSymbolId::new("U1:A").unwrap(),
            )],
        }
    );
}

#[test]
fn schematic_object_presence_conflicts_with_stale_definition_edits() {
    let symbol = SchematicSymbol {
        id: SchematicSymbolId::new("U1:A").unwrap(),
        instance: CircuitInstanceId::new("U1").unwrap(),
        definition: SchematicSymbolDefinitionId::new("editor-symbol").unwrap(),
        unit: 1,
        position: SchematicPoint::new(Real::zero(), Real::zero()),
        quarter_turns: 0,
    };
    let mut original = fixture();
    original.schematic = Some(SchematicLayout {
        symbol_definitions: vec![schematic_definition()],
        symbols: vec![symbol.clone()],
        ..SchematicLayout::default()
    });
    let mut history = DesignHistory::new(original).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-symbol").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::remove_schematic_symbol(symbol.id.clone())],
        })
        .unwrap();

    let mut moved = symbol.clone();
    moved.position = SchematicPoint::new(Real::one(), Real::one());
    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-symbol-move").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::SetSchematicSymbolDefinition {
                symbol: Box::new(moved),
            }],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::SchematicSymbolDefinition(symbol.id)],
        }
    );
}

#[test]
fn circuit_instances_and_models_edit_atomically_across_the_pcb_boundary() {
    let original = fixture();
    let original_model = original.circuit.device_models[0].id.clone();
    let alternate = device_model("alternate-model");
    let mut history = DesignHistory::new(original.clone()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("model-swap").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "circuit-editor".into(),
            edits: vec![
                DesignEdit::insert_device_model(alternate.clone()),
                DesignEdit::SetCircuitInstanceModel {
                    instance: CircuitInstanceId::new("U1").unwrap(),
                    model: alternate.id.clone(),
                },
            ],
        })
        .unwrap();
    history.undo().unwrap();
    assert_eq!(history.document().circuit, original.circuit);
    history.redo().unwrap();
    let swapped = history.document().clone();

    let error = history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-active-model").unwrap(),
            expected_revision: DesignRevision::new(3),
            editor: "circuit-editor".into(),
            edits: vec![DesignEdit::remove_device_model(alternate.id.clone())],
        })
        .unwrap_err();
    assert!(matches!(
        error,
        DesignHistoryError::Edit(DesignEditError::InvalidResult(_))
    ));
    assert_eq!(history.document(), &swapped);

    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-and-reassign-model").unwrap(),
            expected_revision: DesignRevision::new(3),
            editor: "circuit-editor".into(),
            edits: vec![
                DesignEdit::remove_device_model(alternate.id.clone()),
                DesignEdit::SetCircuitInstanceModel {
                    instance: CircuitInstanceId::new("U1").unwrap(),
                    model: original_model.clone(),
                },
            ],
        })
        .unwrap();
    assert_eq!(
        history.document().circuit.device_models,
        vec![device_model(original_model.as_str())]
    );
    history.undo().unwrap();
    assert_eq!(history.document().circuit, swapped.circuit);

    let u2 = circuit_instance("U2", original_model);
    let u2_placement = PcbPlacement {
        instance: u2.id.clone(),
        land_pattern: LandPatternId::new("EDITOR_PATTERN").unwrap(),
        position: point(2, 2),
        rotation_degrees: Real::zero(),
        side: BoardSide::Front,
    };
    let before_insert = history.document().clone();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("insert-instance-and-placement").unwrap(),
            expected_revision: DesignRevision::new(5),
            editor: "circuit-editor".into(),
            edits: vec![
                DesignEdit::insert_circuit_instance(u2.clone()),
                DesignEdit::insert_placement(u2_placement.clone()),
            ],
        })
        .unwrap();
    assert!(history.document().circuit.instances.contains(&u2));
    assert!(
        history
            .document()
            .pcb
            .as_ref()
            .unwrap()
            .placements
            .contains(&u2_placement)
    );
    history.undo().unwrap();
    assert_eq!(history.document().circuit, before_insert.circuit);
    assert_eq!(history.document().pcb, before_insert.pcb);
}

#[test]
fn circuit_presence_changes_reject_stale_identity_writes() {
    let mut history = DesignHistory::new(fixture()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("remove-circuit-instance").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![
                DesignEdit::remove_placement_constraint(
                    PlacementConstraintId::new("inside").unwrap(),
                ),
                DesignEdit::remove_placement_constraint(
                    PlacementConstraintId::new("rotations").unwrap(),
                ),
                DesignEdit::remove_placement(CircuitInstanceId::new("U1").unwrap()),
                DesignEdit::remove_circuit_instance(CircuitInstanceId::new("U1").unwrap()),
            ],
        })
        .unwrap();
    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-instance-model").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::SetCircuitInstanceModel {
                instance: CircuitInstanceId::new("U1").unwrap(),
                model: DeviceModelId::new("editor-part").unwrap(),
            }],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::CircuitInstanceModel(
                CircuitInstanceId::new("U1").unwrap()
            )],
        }
    );

    let alternate = device_model("alternate-model");
    let mut models = DesignHistory::new(fixture()).unwrap();
    models
        .commit(DesignEditBatch {
            id: DesignEditId::new("insert-model").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::insert_device_model(alternate.clone())],
        })
        .unwrap();
    let error = models
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-insert-model").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::insert_device_model(alternate.clone())],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::DeviceModelPresence(alternate.id)],
        }
    );
}

#[test]
fn presence_changes_conflict_with_stale_field_edits_but_not_other_objects() {
    let mut history = DesignHistory::new(fixture()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("delete-route").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::remove_route(RouteId::new("route-1").unwrap())],
        })
        .unwrap();

    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-route-width").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::SetRouteWidth {
                route: RouteId::new("route-1").unwrap(),
                width: Real::from(2),
            }],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::RouteWidth(RouteId::new("route-1").unwrap())],
        }
    );

    let inserted = route("route-2", point(2, 4), point(8, 4));
    let rebased = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("other-route").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "carol".into(),
            edits: vec![DesignEdit::insert_route(inserted.clone())],
        })
        .unwrap();
    assert_eq!(rebased.replay_revision, DesignRevision::new(1));
    assert!(
        history
            .document()
            .pcb
            .as_ref()
            .unwrap()
            .routes
            .contains(&inserted)
    );
}

#[test]
fn placement_presence_and_constraint_presence_reject_stale_writes() {
    let mut history = DesignHistory::new(fixture()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("delete-placement").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![
                DesignEdit::remove_placement_constraint(
                    PlacementConstraintId::new("inside").unwrap(),
                ),
                DesignEdit::remove_placement_constraint(
                    PlacementConstraintId::new("rotations").unwrap(),
                ),
                DesignEdit::remove_placement(CircuitInstanceId::new("U1").unwrap()),
            ],
        })
        .unwrap();

    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-placement").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::SetPlacementTransform {
                instance: CircuitInstanceId::new("U1").unwrap(),
                position: point(2, 2),
                rotation_degrees: Real::zero(),
                side: BoardSide::Front,
            }],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::PlacementTransform(
                CircuitInstanceId::new("U1").unwrap()
            )],
        }
    );

    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-placement-pattern").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "carol".into(),
            edits: vec![DesignEdit::SetPlacementLandPattern {
                instance: CircuitInstanceId::new("U1").unwrap(),
                land_pattern: LandPatternId::new("EDITOR_PATTERN").unwrap(),
            }],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::PlacementLandPattern(
                CircuitInstanceId::new("U1").unwrap()
            )],
        }
    );

    let mut constraints = DesignHistory::new(fixture()).unwrap();
    constraints
        .commit(DesignEditBatch {
            id: DesignEditId::new("delete-inside").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::remove_placement_constraint(
                PlacementConstraintId::new("inside").unwrap(),
            )],
        })
        .unwrap();
    let error = constraints
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("stale-delete-inside").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::remove_placement_constraint(
                PlacementConstraintId::new("inside").unwrap(),
            )],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(1),
            addresses: vec![EditAddress::PlacementConstraintPresence(
                PlacementConstraintId::new("inside").unwrap()
            )],
        }
    );
}

#[test]
fn concurrent_commits_rebase_disjoint_fields_and_reject_overlapping_writes() {
    let mut history = DesignHistory::new(fixture()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("alice-width").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::SetRouteWidth {
                route: RouteId::new("route-1").unwrap(),
                width: Real::from(2),
            }],
        })
        .unwrap();

    let routed = vec![
        PcbRouteSegment::Line(LinePathSegment::new(point(1, 1), point(2, 2))),
        PcbRouteSegment::Line(LinePathSegment::new(point(2, 2), point(4, 1))),
    ];
    let rebased = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("bob-centerline").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "bob".into(),
            edits: vec![DesignEdit::SetRouteSegments {
                route: RouteId::new("route-1").unwrap(),
                segments: routed.clone(),
            }],
        })
        .unwrap();
    assert_eq!(rebased.authored_revision, DesignRevision::new(0));
    assert_eq!(rebased.replay_revision, DesignRevision::new(1));
    assert_eq!(rebased.commit.replay.to_revision, DesignRevision::new(2));
    let route = &history.document().pcb.as_ref().unwrap().routes[0];
    assert_eq!(route.width, Real::from(2));
    assert_eq!(route.segments, routed);

    let before_conflict = history.document().clone();
    let error = history
        .commit_concurrent(DesignEditBatch {
            id: DesignEditId::new("carol-width").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "carol".into(),
            edits: vec![DesignEdit::SetRouteWidth {
                route: RouteId::new("route-1").unwrap(),
                width: Real::from(3),
            }],
        })
        .unwrap_err();
    assert_eq!(
        error,
        DesignHistoryError::MergeConflict {
            expected: DesignRevision::new(0),
            actual: DesignRevision::new(2),
            addresses: vec![EditAddress::RouteWidth(RouteId::new("route-1").unwrap())],
        }
    );
    assert_eq!(history.document(), &before_conflict);
    assert_eq!(history.replay_log().len(), 2);
}

#[test]
fn replay_log_compaction_and_v1_migration_refuse_unprovable_rebases() {
    let mut history = DesignHistory::new(fixture()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("move-via").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::MoveVia {
                via: ViaId::new("via-1").unwrap(),
                center: point(5, 1),
            }],
        })
        .unwrap();
    assert_eq!(history.compact_replay_log(), DesignRevision::new(1));
    assert!(history.replay_log().is_empty());
    let stale = DesignEditBatch {
        id: DesignEditId::new("stale-zone").unwrap(),
        expected_revision: DesignRevision::new(0),
        editor: "bob".into(),
        edits: vec![DesignEdit::SetZoneBoundary {
            zone: ZoneId::new("zone-1").unwrap(),
            boundary: vec![point(2, 2), point(8, 2), point(8, 8), point(2, 8)],
        }],
    };
    assert_eq!(
        history.commit_concurrent(stale.clone()),
        Err(DesignHistoryError::HistoryUnavailable {
            expected: DesignRevision::new(0),
            oldest: DesignRevision::new(1),
        })
    );

    let mut value = serde_json::to_value(&history).unwrap();
    value["version"] = serde_json::Value::from(1);
    let object = value.as_object_mut().unwrap();
    object.remove("replay_base_revision");
    object.remove("replay_log");
    let (mut migrated, report) =
        DesignHistory::from_json_migrating(&serde_json::to_string(&value).unwrap()).unwrap();
    assert_eq!(report.from_version, 1);
    assert_eq!(report.to_version, DESIGN_HISTORY_VERSION);
    assert_eq!(migrated.replay_base_revision(), DesignRevision::new(1));
    assert_eq!(
        migrated.commit_concurrent(stale),
        Err(DesignHistoryError::HistoryUnavailable {
            expected: DesignRevision::new(0),
            oldest: DesignRevision::new(1),
        })
    );
}

#[test]
fn serialized_history_rejects_noncontiguous_replay_evidence() {
    let mut history = DesignHistory::new(fixture()).unwrap();
    history
        .commit(DesignEditBatch {
            id: DesignEditId::new("width").unwrap(),
            expected_revision: DesignRevision::new(0),
            editor: "alice".into(),
            edits: vec![DesignEdit::SetRouteWidth {
                route: RouteId::new("route-1").unwrap(),
                width: Real::from(2),
            }],
        })
        .unwrap();
    let mut value = serde_json::to_value(history).unwrap();
    value["replay_log"][0]["to_revision"] = serde_json::Value::from(9);
    assert_eq!(
        DesignHistory::from_json(&serde_json::to_string(&value).unwrap()),
        Err(DesignHistoryError::InvalidReplayLog)
    );
}

#[test]
fn version_two_through_fourteen_histories_migrate_and_future_versions_are_rejected() {
    let history = DesignHistory::new(fixture()).unwrap();
    for version in [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14] {
        let mut value = serde_json::to_value(&history).unwrap();
        value["version"] = serde_json::Value::from(version);
        let (migrated, report) =
            DesignHistory::from_json_migrating(&serde_json::to_string(&value).unwrap()).unwrap();
        assert_eq!(report.from_version, version);
        assert_eq!(report.to_version, DESIGN_HISTORY_VERSION);
        assert_eq!(migrated.document(), history.document());
    }

    let mut value = serde_json::to_value(&history).unwrap();
    value["version"] = serde_json::Value::from(DESIGN_HISTORY_VERSION + 1);
    assert!(matches!(
        DesignHistory::from_json(&serde_json::to_string(&value).unwrap()),
        Err(DesignHistoryError::UnsupportedSchema { .. })
    ));
}

#[test]
fn touched_fixed_placement_constraints_are_not_silently_overridden() {
    let mut document = fixture();
    document
        .pcb
        .as_mut()
        .unwrap()
        .placement_constraints
        .push(PlacementConstraint {
            id: PlacementConstraintId::new("fixed").unwrap(),
            kind: PlacementConstraintKind::Fixed {
                instance: CircuitInstanceId::new("U1").unwrap(),
                position: point(1, 1),
            },
        });
    let original = document.clone();
    let error = document
        .apply_edit_batch(&batch(vec![DesignEdit::SetPlacementTransform {
            instance: CircuitInstanceId::new("U1").unwrap(),
            position: point(3, 3),
            rotation_degrees: Real::zero(),
            side: BoardSide::Front,
        }]))
        .unwrap_err();
    assert_eq!(
        error,
        DesignEditError::PlacementConstraintConflict(CircuitInstanceId::new("U1").unwrap())
    );
    assert_eq!(document, original);
}

#[test]
fn inserted_driving_constraint_cannot_silently_reposition_authored_placement() {
    let mut document = fixture();
    let original = document.clone();
    let error = document
        .apply_edit_batch(&batch(vec![DesignEdit::insert_placement_constraint(
            PlacementConstraint {
                id: PlacementConstraintId::new("fixed").unwrap(),
                kind: PlacementConstraintKind::Fixed {
                    instance: CircuitInstanceId::new("U1").unwrap(),
                    position: point(3, 3),
                },
            },
        )]))
        .unwrap_err();
    assert_eq!(
        error,
        DesignEditError::PlacementConstraintConflict(CircuitInstanceId::new("U1").unwrap())
    );
    assert_eq!(document, original);
}
