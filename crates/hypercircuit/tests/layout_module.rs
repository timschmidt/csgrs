#![cfg(feature = "layout")]

use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, BoardSide, Circuit, CircuitId, CircuitInstance,
    CircuitInstanceId, CircuitLibrary, CircuitPort, ComponentId, CopperZone, DeviceModel,
    DeviceModelId, DeviceModelKind, EscapePolicy, EscapePolicyId, KeepoutId, KeepoutScope,
    LandPattern, LandPatternId, LayoutAssembly, LayoutCompositionError, LayoutModule,
    LayoutModuleId, LayoutModuleInstance, LayoutModuleValidationIssue, LayoutTransform,
    LengthTuningPattern, LengthTuningPatternId, LengthTuningSide, Net, NetClass, NetClassId, NetId,
    PcbDesignRules, PcbKeepout, PcbLayout, PcbPlacement, PcbRoute, PcbRouteSegment, PcbStackup,
    PcbVia, PlacementConstraint, PlacementConstraintId, PlacementConstraintKind, PlacementGroup,
    PlacementGroupId, Plating, PortDirection, PortId, Real, RouteConstraintRegion,
    RouteConstraintRegionId, RouteDirection, RouteId, RouteRuleRegion, RouteRuleRegionId,
    StackupLayer, StackupLayerKind, SubcircuitInstance, SubcircuitInstanceId,
    SubcircuitPortBinding, TransientPolicy, ViaId, ViaMaskIntent, ViaStyle, ViaStyleId, ZoneId,
};
use hyperlattice::Point2;
use hyperpath::{ArcDirection, CubicBezier, ExplicitCircularArc, LinePathSegment, TraceLayer};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn child_circuit() -> Circuit {
    let input = NetId::new("IN").unwrap();
    let local = NetId::new("LOCAL").unwrap();
    let model = DeviceModelId::new("module-part").unwrap();
    Circuit::new(
        CircuitId::new("sensor-module").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: input.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: local,
        is_ground: false,
    })
    .with_port(CircuitPort {
        id: PortId::new("input").unwrap(),
        net: input,
        direction: PortDirection::Input,
        optional: false,
    })
    .with_device_model(DeviceModel {
        id: model.clone(),
        kind: DeviceModelKind::Custom("module-part".into()),
        pins: Vec::new(),
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("U1").unwrap(),
        component: ComponentId::new("U1").unwrap(),
        part: None,
        model: model.clone(),
        pins: Vec::new(),
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: CircuitInstanceId::new("U2").unwrap(),
        component: ComponentId::new("U2").unwrap(),
        part: None,
        model,
        pins: Vec::new(),
        parameters: Vec::new(),
    })
}

fn hierarchy() -> CircuitLibrary {
    let bus = NetId::new("BUS").unwrap();
    let child = |id: &str| SubcircuitInstance {
        id: SubcircuitInstanceId::new(id).unwrap(),
        circuit: CircuitId::new("sensor-module").unwrap(),
        ports: vec![SubcircuitPortBinding {
            port: PortId::new("input").unwrap(),
            net: bus.clone(),
        }],
        parameter_overrides: Vec::new(),
    };
    let root = Circuit::new(
        CircuitId::new("hierarchical-board").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: bus.clone(),
        is_ground: false,
    })
    .with_subcircuit(child("sensor-a"))
    .with_subcircuit(child("sensor-b"));
    CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, child_circuit()],
    }
}

fn board() -> PcbLayout {
    PcbLayout {
        id: BoardId::new("module-board").unwrap(),
        outline: BoardOutline {
            exterior: vec![point(0, -5), point(30, -5), point(30, 10), point(0, 10)].into(),
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
        land_patterns: Vec::new(),
        placements: Vec::new(),
        placement_constraints: Vec::new(),
        routes: Vec::new(),
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    }
}

fn module() -> LayoutModule {
    let local = NetId::new("LOCAL").unwrap();
    let pattern = LandPatternId::new("MODULE_PATTERN").unwrap();
    let arc = ExplicitCircularArc::new(
        point(2, 1),
        Real::one(),
        point(2, 0),
        point(3, 1),
        ArcDirection::Ccw,
    )
    .unwrap();
    LayoutModule {
        id: LayoutModuleId::new("sensor-layout").unwrap(),
        circuit: CircuitId::new("sensor-module").unwrap(),
        land_patterns: vec![LandPattern {
            id: pattern.clone(),
            pads: Vec::new(),
            pin_map: Vec::new(),
            graphics: Vec::new(),
            body: None,
            models: Vec::new(),
        }],
        placements: vec![
            PcbPlacement {
                instance: CircuitInstanceId::new("U1").unwrap(),
                land_pattern: pattern.clone(),
                position: point(1, 1),
                rotation_degrees: Real::zero(),
                side: BoardSide::Front,
            },
            PcbPlacement {
                instance: CircuitInstanceId::new("U2").unwrap(),
                land_pattern: pattern,
                position: point(2, 1),
                rotation_degrees: Real::zero(),
                side: BoardSide::Front,
            },
        ],
        placement_constraints: Vec::new(),
        placement_groups: vec![PlacementGroup {
            id: PlacementGroupId::new("analog-front-end").unwrap(),
            instances: vec![
                CircuitInstanceId::new("U1").unwrap(),
                CircuitInstanceId::new("U2").unwrap(),
            ],
            transform: LayoutTransform {
                position: point(2, 0),
                ..LayoutTransform::default()
            },
        }],
        routes: vec![PcbRoute {
            id: RouteId::new("local-route").unwrap(),
            net: local.clone(),
            layer: TraceLayer(0),
            width: Real::one(),
            segments: vec![
                PcbRouteSegment::Line(LinePathSegment::new(point(0, 0), point(2, 0))),
                PcbRouteSegment::CircularArc(arc),
                PcbRouteSegment::CubicBezier(CubicBezier::new(
                    point(3, 1),
                    point(3, 2),
                    point(4, 1),
                    point(4, 0),
                )),
            ],
        }],
        vias: vec![PcbVia {
            id: ViaId::new("local-via").unwrap(),
            net: local.clone(),
            start_layer: TraceLayer(0),
            end_layer: TraceLayer(0),
            center: point(4, 0),
            land_diameter: Real::one(),
            drill_diameter: (Real::one() / Real::from(2)).unwrap(),
            plating: Plating::Plated,
            mask: ViaMaskIntent::tented(),
        }],
        zones: vec![CopperZone::solid(
            ZoneId::new("local-zone").unwrap(),
            local.clone(),
            TraceLayer(0),
            vec![point(0, -1), point(5, -1), point(5, 3), point(0, 3)],
        )],
        keepouts: vec![PcbKeepout {
            id: KeepoutId::new("local-keepout").unwrap(),
            boundary: vec![point(1, 0), point(2, 0), point(2, 2), point(1, 2)],
            scope: KeepoutScope::Components,
        }],
        rules: PcbDesignRules {
            net_classes: vec![
                NetClass {
                    id: NetClassId::new("local-base").unwrap(),
                    parent: None,
                    nets: Vec::new(),
                    min_trace_width: Some(Real::one()),
                    preferred_trace_width: None,
                    min_clearance: Some(Real::one()),
                    preferred_via_land_diameter: Some(Real::one()),
                    preferred_via_drill_diameter: Some((Real::one() / Real::from(2)).unwrap()),
                    preferred_via_style: Some(ViaStyleId::new("local-via").unwrap()),
                    max_length: None,
                    max_via_count: Some(2),
                    target_impedance_ohms: None,
                    impedance_tolerance_ohms: None,
                    requires_reference_plane: false,
                },
                NetClass {
                    id: NetClassId::new("local-class").unwrap(),
                    parent: Some(NetClassId::new("local-base").unwrap()),
                    nets: vec![local],
                    min_trace_width: None,
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
            ],
            via_styles: vec![ViaStyle {
                id: ViaStyleId::new("local-via").unwrap(),
                land_diameter: Real::one(),
                drill_diameter: (Real::one() / Real::from(2)).unwrap(),
                plating: Plating::Plated,
                mask: ViaMaskIntent::tented(),
                allowed_spans: Vec::new(),
            }],
            differential_pairs: Vec::new(),
            route_constraint_regions: vec![RouteConstraintRegion {
                id: RouteConstraintRegionId::new("local-channel").unwrap(),
                boundary: vec![point(0, -1), point(5, -1), point(5, 1), point(0, 1)],
                nets: vec![NetId::new("LOCAL").unwrap()],
                allowed_layers: vec![TraceLayer(0)],
                allowed_directions: vec![
                    RouteDirection::Horizontal,
                    RouteDirection::DiagonalRising,
                    RouteDirection::Arbitrary,
                ],
                allow_vias: false,
            }],
            route_rule_regions: vec![RouteRuleRegion {
                id: RouteRuleRegionId::new("local-width").unwrap(),
                boundary: vec![point(0, -1), point(3, -1), point(3, 2), point(0, 2)],
                nets: vec![NetId::new("LOCAL").unwrap()],
                min_trace_width: Some(Real::from(2)),
                min_clearance: None,
            }],
            escape_policies: vec![EscapePolicy {
                id: EscapePolicyId::new("u1-fanout").unwrap(),
                instances: vec![CircuitInstanceId::new("U1").unwrap()],
                nets: vec![NetId::new("LOCAL").unwrap()],
                max_distance: Real::one(),
                allowed_layers: vec![TraceLayer(0)],
                allowed_directions: vec![
                    RouteDirection::Horizontal,
                    RouteDirection::DiagonalFalling,
                    RouteDirection::Arbitrary,
                ],
                allow_vias: false,
            }],
            length_tuning_patterns: vec![LengthTuningPattern {
                id: LengthTuningPatternId::new("local-tuning").unwrap(),
                net: NetId::new("LOCAL").unwrap(),
                route: Some(RouteId::new("local-route").unwrap()),
                region: vec![point(0, -1), point(5, -1), point(5, 3), point(0, 3)],
                target_length: Real::from(12),
                tolerance: Real::zero(),
                amplitude: Real::one(),
                pitch: Real::one(),
                maximum_cycles: 2,
                side: LengthTuningSide::Left,
            }],
            phase_tuning_groups: Vec::new(),
        },
    }
}

fn assembly() -> LayoutAssembly {
    LayoutAssembly {
        board: board(),
        modules: vec![module()],
        instances: vec![
            LayoutModuleInstance {
                hierarchy_path: vec![SubcircuitInstanceId::new("sensor-a").unwrap()],
                module: LayoutModuleId::new("sensor-layout").unwrap(),
                transform: LayoutTransform {
                    position: point(20, 0),
                    rotation_degrees: Real::zero(),
                    side: BoardSide::Back,
                },
            },
            LayoutModuleInstance {
                hierarchy_path: vec![SubcircuitInstanceId::new("sensor-b").unwrap()],
                module: LayoutModuleId::new("sensor-layout").unwrap(),
                transform: LayoutTransform::default(),
            },
        ],
    }
}

#[test]
fn reusable_layout_modules_compose_to_flat_hyperpath_ready_intent() {
    let report = assembly().compose(&hierarchy()).unwrap();
    assert_eq!(report.modules.len(), 2);
    assert_eq!(
        report.modules[0].placement_groups[0].as_str(),
        "sensor-a/analog-front-end"
    );
    assert_eq!(report.layout.land_patterns.len(), 2);
    assert_eq!(report.layout.placements.len(), 4);
    assert_eq!(report.layout.routes.len(), 2);
    assert_eq!(report.layout.vias.len(), 2);
    assert_eq!(report.layout.zones.len(), 2);
    assert_eq!(report.layout.keepouts.len(), 2);
    assert_eq!(report.layout.rules.net_classes.len(), 4);
    assert_eq!(report.layout.rules.via_styles.len(), 2);
    assert_eq!(report.layout.rules.route_constraint_regions.len(), 2);
    assert_eq!(report.layout.rules.route_rule_regions.len(), 2);
    assert_eq!(report.layout.rules.escape_policies.len(), 2);
    assert_eq!(report.layout.rules.length_tuning_patterns.len(), 2);
    assert!(report.layout.validate(&report.circuit).is_valid());
    assert_eq!(
        report.layout.rules.via_styles[0].id.as_str(),
        "sensor-a/local-via"
    );
    let class = report
        .layout
        .rules
        .net_classes
        .iter()
        .find(|class| class.id.as_str() == "sensor-a/local-class")
        .unwrap();
    assert_eq!(
        class.parent.as_ref().unwrap().as_str(),
        "sensor-a/local-base"
    );
    let resolved = report.layout.rules.resolve_net_classes().unwrap();
    let class = resolved
        .iter()
        .find(|class| class.id.as_str() == "sensor-a/local-class")
        .unwrap();
    assert_eq!(
        class.preferred_via_style.as_ref().unwrap().as_str(),
        "sensor-a/local-via"
    );
    assert_eq!(class.max_via_count, Some(2));

    let first = &report.layout.placements[0];
    assert_eq!(first.instance.as_str(), "sensor-a/U1");
    assert_eq!(first.land_pattern.as_str(), "sensor-a/MODULE_PATTERN");
    assert_eq!(first.position, point(17, 1));
    assert_eq!(first.side, BoardSide::Back);
    assert_eq!(report.layout.placements[1].position, point(16, 1));
    assert_eq!(report.layout.placements[2].position, point(3, 1));

    let route = &report.layout.routes[0];
    assert_eq!(route.id.as_str(), "sensor-a/local-route");
    assert_eq!(route.net.as_str(), "sensor-a/LOCAL");
    assert_eq!(route.segments[0].start(), &point(20, 0));
    assert_eq!(route.segments[0].end(), &point(18, 0));
    let PcbRouteSegment::CircularArc(arc) = &route.segments[1] else {
        panic!("second segment must remain a circular arc");
    };
    assert_eq!(arc.direction(), ArcDirection::Cw);
    assert_eq!(route.segments[2].end(), &point(16, 0));
    assert!(route.to_hyperpath(hyperpath::NetId(1)).is_ok());
    let region = &report.layout.rules.route_constraint_regions[0];
    assert_eq!(region.id.as_str(), "sensor-a/local-channel");
    assert_eq!(region.nets[0].as_str(), "sensor-a/LOCAL");
    assert_eq!(
        region.allowed_directions,
        vec![
            RouteDirection::Horizontal,
            RouteDirection::DiagonalFalling,
            RouteDirection::Arbitrary,
        ]
    );
    let rule_region = &report.layout.rules.route_rule_regions[0];
    assert_eq!(rule_region.id.as_str(), "sensor-a/local-width");
    assert_eq!(rule_region.nets[0].as_str(), "sensor-a/LOCAL");
    assert_eq!(rule_region.boundary[0], point(20, -1));
    let escape = &report.layout.rules.escape_policies[0];
    assert_eq!(escape.id.as_str(), "sensor-a/u1-fanout");
    assert_eq!(escape.instances[0].as_str(), "sensor-a/U1");
    assert_eq!(
        escape.allowed_directions,
        vec![
            RouteDirection::Horizontal,
            RouteDirection::DiagonalRising,
            RouteDirection::Arbitrary,
        ]
    );
    let tuning = &report.layout.rules.length_tuning_patterns[0];
    assert_eq!(tuning.id.as_str(), "sensor-a/local-tuning");
    assert_eq!(
        tuning.route.as_ref().unwrap().as_str(),
        "sensor-a/local-route"
    );
    assert_eq!(tuning.side, LengthTuningSide::Right);

    #[cfg(feature = "geometry")]
    {
        let mut materialization_fixture = report.layout.clone();
        for route in &mut materialization_fixture.routes {
            route.segments.truncate(1);
        }
        materialization_fixture.zones.clear();
        let materialized = materialization_fixture
            .materialize(
                &report.circuit,
                hypercircuit::MaterializationOptions::default(),
            )
            .unwrap();
        assert!(materialized.copper_features.iter().any(|feature| {
            feature.source == "route:sensor-a/local-route"
                && feature
                    .net
                    .as_ref()
                    .is_some_and(|net| net.as_str() == "sensor-a/LOCAL")
        }));
    }
}

#[test]
fn module_validation_and_scope_binding_fail_before_partial_composition() {
    let circuits = hierarchy();
    let mut invalid_group = assembly();
    invalid_group.modules[0].placement_groups[0]
        .instances
        .push(CircuitInstanceId::new("missing").unwrap());
    assert!(matches!(
        invalid_group.compose(&circuits),
        Err(LayoutCompositionError::InvalidModule { issues, .. })
            if issues.iter().any(|issue| matches!(
                issue,
                LayoutModuleValidationIssue::UnknownGroupedPlacement { .. }
            ))
    ));

    let mut unsatisfied_constraint = assembly();
    unsatisfied_constraint.modules[0]
        .placement_constraints
        .push(PlacementConstraint {
            id: PlacementConstraintId::new("back-only").unwrap(),
            kind: PlacementConstraintKind::AllowedSides {
                instance: CircuitInstanceId::new("U1").unwrap(),
                sides: vec![BoardSide::Back],
            },
        });
    assert!(matches!(
        unsatisfied_constraint.compose(&circuits),
        Err(LayoutCompositionError::InvalidModule { issues, .. })
            if issues.iter().any(|issue| matches!(
                issue,
                LayoutModuleValidationIssue::UnsatisfiedPlacementConstraints(_)
            ))
    ));

    let mut duplicate = assembly();
    duplicate.instances[1].hierarchy_path = vec![SubcircuitInstanceId::new("sensor-a").unwrap()];
    assert!(matches!(
        duplicate.compose(&circuits),
        Err(LayoutCompositionError::DuplicateHierarchyPath(_))
    ));

    let mut missing = assembly();
    missing.instances[0].hierarchy_path = vec![SubcircuitInstanceId::new("absent").unwrap()];
    assert!(matches!(
        missing.compose(&circuits),
        Err(LayoutCompositionError::UnknownHierarchyPath(_))
    ));
}

#[cfg(feature = "interchange")]
#[test]
fn reusable_layout_assembly_round_trips_before_composition() {
    let assembly = assembly();
    let json = serde_json::to_string_pretty(&assembly).unwrap();
    let decoded = serde_json::from_str::<LayoutAssembly>(&json).unwrap();
    assert_eq!(decoded, assembly);
    assert!(decoded.compose(&hierarchy()).is_ok());
}
