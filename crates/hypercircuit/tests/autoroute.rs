#![cfg(feature = "layout")]

use hypercircuit::{
    AdapterKind, BoardContour, BoardId, BoardOutline, BoardSide, Circuit, CircuitId,
    CircuitInstance, CircuitInstanceId, ComponentId, CopperZone, DeviceModel, DeviceModelId,
    DeviceModelKind, DevicePin, DifferentialPair, DifferentialPairId, DifferentialPairNeckdown,
    EscapePolicy, EscapePolicyId, KeepoutId, KeepoutScope, LandPattern, LandPatternId,
    LandPatternPad, LayoutValidationIssue, LengthTuningPattern, LengthTuningPatternId,
    LengthTuningSide, LengthTuningStatus, NegotiatedAdaptiveRoutePolicy,
    NegotiatedAdaptiveRouteStatus, NegotiatedGridMode, NegotiatedGridRefinementRegion,
    NegotiatedPlanarTopology, NegotiatedRouteConflictGeometry, NegotiatedRouteFailure,
    NegotiatedRouteHtmlOptions, NegotiatedRoutePolicy, NegotiatedRouteStatus,
    NegotiatedRouteSvgError, NegotiatedRouteSvgOptions, NegotiatedRouterError, Net, NetClass,
    NetClassId, NetId, PadId, PadPinMap, PadShape, PcbDesignRules, PcbKeepout, PcbLayout,
    PcbPlacement, PcbRoute, PcbRouteSegment, PcbStackup, PhaseTuningGroup, PhaseTuningGroupId,
    PhaseTuningIssue, PhaseTuningObstacle, PhaseTuningStatus, PhaseTuningSynthesisIssue,
    PhaseTuningSynthesisPolicy, PhaseTuningSynthesisStatus, PinBinding, PinElectricalKind, PinRef,
    Plating, Real, RouteConstraintRegion, RouteConstraintRegionId, RouteDirection, RouteId,
    RouteRuleRegion, RouteRuleRegionId, RoutingProblemReport, RoutingQualityIssue,
    RoutingQualityStatus, RoutingSolution, StackupLayer, StackupLayerKind, TransientPolicy,
    TscircuitRoutingError, TscircuitRoutingExportOptions, TscircuitRoutingImportOptions,
    TscircuitRoutingImportReport, TscircuitRoutingOmission, ViaMaskIntent, ViaStyle, ViaStyleId,
    ViaStyleSpan, ZoneId,
};
use hyperlattice::Point2;
use hyperpath::{ArcDirection, ExplicitCircularArc, LinePathSegment, TraceLayer};
use serde_json::json;

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn fixture(two_layers: bool) -> (Circuit, PcbLayout) {
    let net_a = NetId::new("A").unwrap();
    let net_b = NetId::new("B").unwrap();
    let model_id = DeviceModelId::new("terminal").unwrap();
    let pin = PinRef::new("1").unwrap();
    let mut circuit = Circuit::new(
        CircuitId::new("negotiated-router").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: net_a.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: net_b.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: model_id.clone(),
        kind: DeviceModelKind::Resistor,
        pins: vec![DevicePin {
            pin: pin.clone(),
            kind: PinElectricalKind::Passive,
            optional: false,
        }],
        parameters: Vec::new(),
    });
    let instances = [
        ("A0", net_a.clone(), p(2, 5)),
        ("A1", net_a, p(8, 5)),
        ("B0", net_b.clone(), p(5, 2)),
        ("B1", net_b, p(5, 8)),
    ];
    for (name, net, _) in &instances {
        circuit = circuit.with_instance(CircuitInstance {
            id: CircuitInstanceId::new(*name).unwrap(),
            component: ComponentId::new(*name).unwrap(),
            part: None,
            model: model_id.clone(),
            pins: vec![PinBinding {
                pin: pin.clone(),
                net: net.clone(),
            }],
            parameters: Vec::new(),
        });
    }
    let front = TraceLayer(0);
    let footprint = LandPatternId::new("terminal-pad").unwrap();
    let mut layers = vec![StackupLayer {
        name: "F.Cu".into(),
        kind: StackupLayerKind::Conductor(front),
        thickness: Real::one(),
        material: None,
    }];
    if two_layers {
        layers.push(StackupLayer {
            name: "B.Cu".into(),
            kind: StackupLayerKind::Conductor(TraceLayer(1)),
            thickness: Real::one(),
            material: None,
        });
    }
    let layout = PcbLayout {
        id: BoardId::new("negotiated-router").unwrap(),
        outline: BoardOutline {
            exterior: vec![p(0, 0), p(10, 0), p(10, 10), p(0, 10)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup { layers },
        land_patterns: vec![LandPattern {
            id: footprint.clone(),
            pads: vec![LandPatternPad {
                id: PadId::new("1").unwrap(),
                center: p(0, 0),
                rotation_degrees: Real::zero(),
                copper_layers: vec![front],
                shape: PadShape::Circle {
                    diameter: Real::one(),
                },
                drill: None,
                plating: Plating::Unspecified,
                solder_mask_margin: None,
                paste_margin: None,
            }],
            pin_map: vec![PadPinMap {
                pin,
                pad: PadId::new("1").unwrap(),
            }],
            graphics: Vec::new(),
            body: None,
            models: Vec::new(),
        }],
        placements: instances
            .iter()
            .map(|(name, _, position)| PcbPlacement {
                instance: CircuitInstanceId::new(*name).unwrap(),
                land_pattern: footprint.clone(),
                position: position.clone(),
                rotation_degrees: Real::zero(),
                side: BoardSide::Front,
            })
            .collect(),
        placement_constraints: Vec::new(),
        routes: Vec::new(),
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };
    (circuit, layout)
}

#[test]
fn tscircuit_simple_route_json_round_trips_external_wire_and_via_candidates() {
    let (circuit, layout) = fixture(true);
    let handoff = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    assert_eq!(
        handoff
            .export_tscircuit_simple_route_json(&layout, TscircuitRoutingExportOptions::default()),
        Err(TscircuitRoutingError::MissingMinimumTraceWidth)
    );
    let exported = handoff
        .export_tscircuit_simple_route_json(
            &layout,
            TscircuitRoutingExportOptions {
                fallback_min_trace_width: Some(Real::one()),
                ..TscircuitRoutingExportOptions::default()
            },
        )
        .unwrap();
    let mut protocol: serde_json::Value = serde_json::from_str(&exported.json).unwrap();
    assert_eq!(protocol["layerCount"], 2);
    assert_eq!(protocol["minTraceWidth"].as_f64(), Some(1.0));
    assert_eq!(protocol["connections"].as_array().unwrap().len(), 2);
    assert_eq!(protocol["obstacles"].as_array().unwrap().len(), 4);
    assert!(!exported.projections.is_empty());
    assert_eq!(
        exported
            .omissions
            .iter()
            .filter(|omission| matches!(omission, TscircuitRoutingOmission::PadShapeBoxed { .. }))
            .count(),
        4
    );

    protocol["traces"] = json!([
        {
            "type": "pcb_trace",
            "pcb_trace_id": "A",
            "route": [
                {"route_type": "wire", "x": 2, "y": 5, "width": 1, "layer": "F.Cu"},
                {"route_type": "wire", "x": 5, "y": 5, "width": 1, "layer": "F.Cu"},
                {"route_type": "via", "x": 5, "y": 5, "from_layer": "F.Cu", "to_layer": "B.Cu"},
                {"route_type": "wire", "x": 5, "y": 5, "width": 1, "layer": "B.Cu"},
                {"route_type": "wire", "x": 7, "y": 5, "width": 1, "layer": "B.Cu"},
                {"route_type": "via", "x": 7, "y": 5, "from_layer": "B.Cu", "to_layer": "F.Cu"},
                {"route_type": "wire", "x": 7, "y": 5, "width": 1, "layer": "F.Cu"},
                {"route_type": "wire", "x": 8, "y": 5, "width": 1, "layer": "F.Cu"}
            ]
        }
    ]);
    let imported = TscircuitRoutingImportReport::from_str(
        &handoff.problem,
        &serde_json::to_string(&protocol).unwrap(),
        TscircuitRoutingImportOptions {
            via_land_diameter: Real::one(),
            via_drill_diameter: (Real::one() / Real::from(2)).unwrap(),
            via_plating: Plating::Plated,
        },
    )
    .unwrap();

    assert_eq!(imported.solution.routes.len(), 3);
    assert_eq!(imported.solution.vias.len(), 2);
    assert!(
        imported
            .solution
            .routes
            .iter()
            .all(|route| route.net.as_str() == "A")
    );
    assert!(
        imported
            .solution
            .vias
            .iter()
            .all(|via| via.net.as_str() == "A" && via.plating == Plating::Plated)
    );
    assert!(!imported.numeric_imports.is_empty());
    let routed = imported.solution.append_to(&layout).unwrap();
    assert!(routed.validate(&circuit).is_valid());
}

#[test]
fn exact_route_quality_report_measures_stretch_and_typed_incompleteness() {
    let (circuit, layout) = fixture(false);
    let problem = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    let direct = RoutingSolution {
        routes: vec![
            PcbRoute {
                id: RouteId::new("quality-a").unwrap(),
                net: NetId::new("A").unwrap(),
                layer: TraceLayer(0),
                width: Real::one(),
                segments: vec![PcbRouteSegment::Line(LinePathSegment::new(
                    p(2, 5),
                    p(8, 5),
                ))],
            },
            PcbRoute {
                id: RouteId::new("quality-b").unwrap(),
                net: NetId::new("B").unwrap(),
                layer: TraceLayer(0),
                width: Real::one(),
                segments: vec![PcbRouteSegment::Line(LinePathSegment::new(
                    p(5, 2),
                    p(5, 8),
                ))],
            },
        ],
        vias: Vec::new(),
        omissions: Vec::new(),
    };
    let quality = direct.quality_report(&problem.problem);
    assert_eq!(quality.status, RoutingQualityStatus::Complete);
    assert_eq!(quality.nets.len(), 2);
    assert_eq!(quality.routed_length, Some(Real::from(12)));
    assert_eq!(quality.euclidean_mst_lower_bound, Some(Real::from(12)));
    assert_eq!(quality.excess_length, Some(Real::zero()));
    assert_eq!(quality.stretch, Some(Real::one()));
    assert_eq!(quality.vias, 0);
    assert!(quality.issues.is_empty());

    let incomplete = RoutingSolution {
        routes: vec![direct.routes[0].clone()],
        vias: Vec::new(),
        omissions: Vec::new(),
    }
    .quality_report(&problem.problem);
    assert_eq!(incomplete.status, RoutingQualityStatus::Incomplete);
    assert_eq!(
        incomplete.issues,
        vec![RoutingQualityIssue::MissingRoute(NetId::new("B").unwrap())]
    );

    let diagonal = RoutingSolution {
        routes: vec![
            direct.routes[0].clone(),
            PcbRoute {
                id: RouteId::new("quality-diagonal").unwrap(),
                net: NetId::new("B").unwrap(),
                layer: TraceLayer(0),
                width: Real::one(),
                segments: vec![PcbRouteSegment::Line(LinePathSegment::new(
                    p(5, 2),
                    p(6, 8),
                ))],
            },
        ],
        vias: Vec::new(),
        omissions: Vec::new(),
    }
    .quality_report(&problem.problem);
    assert_eq!(diagonal.status, RoutingQualityStatus::Complete);
    assert_eq!(
        diagonal.routed_length,
        Some(Real::from(6) + Real::from(37).sqrt().unwrap())
    );
    assert!(diagonal.issues.is_empty());
}

#[test]
fn negotiated_router_rips_up_conflicts_and_emits_hyperpath_geometry() {
    let (circuit, layout) = fixture(true);
    let policy = NegotiatedRoutePolicy {
        maximum_iterations: 24,
        present_congestion_penalty: 0,
        history_increment: 12,
        via_penalty: 4,
        via_land_diameter: Real::one(),
        via_drill_diameter: Real::one(),
        via_mask: ViaMaskIntent::tented(),
        ..NegotiatedRoutePolicy::default()
    };
    let report = layout
        .negotiated_autoroute(&circuit, policy.clone())
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete, "{report:?}");
    assert!(report.iterations.len() > 1);
    assert!(report.iterations[0].conflicted_resources > 0);
    assert_eq!(report.iterations.len(), report.iteration_states.len());
    assert_eq!(report.policy, policy);
    assert_eq!(
        report.work.grid_nodes,
        report.grid.x_coordinates * report.grid.y_coordinates * report.grid.conductor_layers
    );
    assert_eq!(report.work.iteration_budget, 24);
    assert_eq!(report.work.iterations_executed, report.iterations.len());
    assert_eq!(
        report.work.expanded_states_total,
        report
            .iterations
            .iter()
            .map(|iteration| iteration.expanded_states)
            .sum::<usize>()
    );
    assert_eq!(
        report.work.peak_expanded_states_per_iteration,
        report
            .iterations
            .iter()
            .map(|iteration| iteration.expanded_states)
            .max()
            .unwrap()
    );
    assert!(!report.work.iteration_budget_exhausted);
    assert_eq!(report.work.expansion_limit_failures, 0);
    assert_eq!(
        report.iteration_states[0].conflicts.len(),
        report.iterations[0].conflicted_resources
    );
    assert!(
        report.iteration_states[0]
            .conflicts
            .iter()
            .all(|conflict| conflict.nets.len() > 1)
    );
    let final_state = report.iteration_states.last().unwrap();
    assert!(final_state.conflicts.is_empty());
    assert!(final_state.failures.is_empty());
    assert_eq!(final_state.nets.len(), 2);
    let first_svg = report
        .iteration_svg(
            &layout,
            NegotiatedRouteSvgOptions {
                iteration: 0,
                ..NegotiatedRouteSvgOptions::default()
            },
        )
        .unwrap();
    assert_eq!(first_svg.iteration, 0);
    assert_eq!(first_svg.status, NegotiatedRouteStatus::Complete);
    assert!(first_svg.rendered_edges > 0);
    assert!(first_svg.rendered_conflicts > 0);
    assert_eq!(
        first_svg.rendered_conflicts,
        report.iteration_states[0].conflicts.len()
    );
    assert_eq!(first_svg.failures, 0);
    assert!(!first_svg.projections.is_empty());
    assert!(first_svg.svg.contains("data-iteration=\"0\""));
    assert!(first_svg.svg.contains("data-net=\"A\""));
    assert!(first_svg.svg.contains("data-conflict="));
    let final_svg = report
        .iteration_svg(
            &layout,
            NegotiatedRouteSvgOptions {
                iteration: final_state.iteration,
                layer: Some(TraceLayer(0)),
                ..NegotiatedRouteSvgOptions::default()
            },
        )
        .unwrap();
    assert!(final_svg.rendered_edges > 0);
    assert_eq!(final_svg.rendered_conflicts, 0);
    assert_eq!(final_svg.failures, 0);
    assert_eq!(
        report.iteration_svg(
            &layout,
            NegotiatedRouteSvgOptions {
                iteration: usize::MAX,
                ..NegotiatedRouteSvgOptions::default()
            }
        ),
        Err(NegotiatedRouteSvgError::UnknownIteration(usize::MAX))
    );
    let html = report
        .replay_html(
            &layout,
            NegotiatedRouteHtmlOptions {
                title: "A&B <routing> replay".into(),
                ..NegotiatedRouteHtmlOptions::default()
            },
        )
        .unwrap();
    assert_eq!(html.status, NegotiatedRouteStatus::Complete);
    assert_eq!(html.rendered_iterations, report.iteration_states.len());
    assert!(html.rendered_edges >= first_svg.rendered_edges);
    assert!(html.rendered_conflicts >= first_svg.rendered_conflicts);
    assert_eq!(html.failures, 0);
    assert!(html.projections.len() >= first_svg.projections.len());
    assert!(html.html.starts_with("<!doctype html>"));
    assert!(
        html.html
            .contains("<title>A&amp;B &lt;routing&gt; replay</title>")
    );
    assert!(html.html.contains("id=\"pass\""));
    assert!(html.html.contains("data-conflicts=\""));
    assert!(html.html.contains("ArrowRight"));
    let mut empty = report.clone();
    empty.iteration_states.clear();
    assert_eq!(
        empty.replay_html(&layout, NegotiatedRouteHtmlOptions::default()),
        Err(NegotiatedRouteSvgError::EmptyReport)
    );
    let replay = layout
        .negotiated_autoroute(&circuit, policy.clone())
        .unwrap();
    assert_eq!(replay.iteration_states, report.iteration_states);
    assert_eq!(replay.work, report.work);
    assert_eq!(
        replay
            .iteration_svg(&layout, NegotiatedRouteSvgOptions::default())
            .unwrap(),
        first_svg
    );
    assert_eq!(
        replay
            .replay_html(
                &layout,
                NegotiatedRouteHtmlOptions {
                    title: "A&B <routing> replay".into(),
                    ..NegotiatedRouteHtmlOptions::default()
                }
            )
            .unwrap(),
        html
    );
    let route = report.route.as_ref().unwrap();
    assert!(!route.traces().is_empty());
    assert!(!route.vias().is_empty());
    let problem = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    let quality = report
        .solution
        .as_ref()
        .unwrap()
        .quality_report(&problem.problem);
    assert_eq!(quality.status, RoutingQualityStatus::Complete);
    assert_eq!(quality.euclidean_mst_lower_bound, Some(Real::from(12)));
    assert!(
        quality.routed_length.as_ref().unwrap()
            >= quality.euclidean_mst_lower_bound.as_ref().unwrap()
    );
    assert_eq!(quality.vias, route.vias().len());
    let routed = report.apply_to(&layout).unwrap();
    assert!(routed.validate(&circuit).is_valid());
    assert!(routed.routes.iter().all(|route| {
        route.id.as_str().starts_with("negotiated-")
            && route
                .segments
                .windows(2)
                .all(|pair| pair[0].end() == pair[1].start())
    }));
    assert!(
        routed
            .vias
            .iter()
            .all(|via| via.mask == ViaMaskIntent::tented())
    );
}

#[test]
fn feature_aligned_grid_routes_exact_off_lattice_terminals_and_audits_injection() {
    let (circuit, mut layout) = fixture(true);
    layout.placements[0].position.x = (Real::from(3) / Real::from(2)).unwrap();
    layout.placements[1].position.x = (Real::from(17) / Real::from(2)).unwrap();
    let selected = vec![NetId::new("A").unwrap()];

    let uniform = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                nets: selected.clone(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(uniform.status, NegotiatedRouteStatus::IterationLimit);
    assert!(matches!(
        uniform.failures.as_slice(),
        [NegotiatedRouteFailure::OffGridTerminal { net, .. }] if net.as_str() == "A"
    ));

    let adaptive = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                grid_mode: NegotiatedGridMode::FeatureAligned {
                    coarse_pitch_multiplier: 4,
                    feature_halo_steps: 1,
                },
                nets: selected,
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(
        adaptive.status,
        NegotiatedRouteStatus::Complete,
        "{adaptive:?}"
    );
    assert_eq!(
        adaptive.grid.mode,
        NegotiatedGridMode::FeatureAligned {
            coarse_pitch_multiplier: 4,
            feature_halo_steps: 1,
        }
    );
    assert!(adaptive.grid.injected_x_coordinates > 0);
    assert!(adaptive.grid.injected_y_coordinates > 0);
    let problem = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    let quality = adaptive
        .solution
        .as_ref()
        .unwrap()
        .quality_report(&problem.problem);
    let net = quality
        .nets
        .iter()
        .find(|net| net.net.as_str() == "A")
        .unwrap();
    assert_eq!(net.euclidean_mst_lower_bound, Some(Real::from(7)));
    assert!(net.routed_length.as_ref().unwrap() >= net.euclidean_mst_lower_bound.as_ref().unwrap());
}

#[test]
fn negotiated_router_uses_exact_arc_board_clearance() {
    let (circuit, mut layout) = fixture(false);
    layout.outline.exterior = BoardContour::from_segments(vec![
        LinePathSegment::new(p(0, 0), p(10, 0)).into(),
        LinePathSegment::new(p(10, 0), p(10, 10)).into(),
        ExplicitCircularArc::new(
            p(5, 10),
            Real::from(5),
            p(10, 10),
            p(0, 10),
            ArcDirection::Ccw,
        )
        .unwrap()
        .into(),
        LinePathSegment::new(p(0, 10), p(0, 0)).into(),
    ]);
    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                nets: vec![NetId::new("A").unwrap()],
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete, "{report:?}");
    assert!(
        report
            .route
            .as_ref()
            .is_some_and(|route| !route.traces().is_empty())
    );
}

#[test]
fn locally_refined_grid_adds_dense_channel_capacity_without_a_global_fine_mesh() {
    let (circuit, mut layout) = fixture(false);
    layout.outline.exterior = vec![p(0, 0), p(12, 0), p(12, 12), p(0, 12)].into();
    for (instance, position) in [
        ("A0", p(2, 4)),
        ("A1", p(10, 4)),
        ("B0", p(2, 6)),
        ("B1", p(10, 6)),
    ] {
        layout
            .placements
            .iter_mut()
            .find(|placement| placement.instance.as_str() == instance)
            .unwrap()
            .position = position;
    }

    let uniform = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(uniform.status, NegotiatedRouteStatus::Complete);
    assert_eq!(uniform.grid.active_planar_nodes, 169);
    assert_eq!(uniform.grid.refined_planar_nodes, 0);

    let region = NegotiatedGridRefinementRegion {
        min: p(2, 3),
        max: p(10, 7),
    };
    let policy = NegotiatedRoutePolicy {
        grid_mode: NegotiatedGridMode::LocallyRefined {
            coarse_pitch_multiplier: 4,
            regions: vec![region.clone()],
        },
        via_land_diameter: Real::one(),
        via_drill_diameter: Real::one(),
        ..NegotiatedRoutePolicy::default()
    };
    let report = layout
        .negotiated_autoroute(&circuit, policy.clone())
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete);
    assert_eq!(report.grid.mode, policy.grid_mode);
    assert_eq!(report.grid.x_coordinates, 11);
    assert_eq!(report.grid.y_coordinates, 8);
    assert_eq!(report.grid.injected_x_coordinates, 7);
    assert_eq!(report.grid.injected_y_coordinates, 4);
    assert_eq!(report.grid.active_planar_nodes, 59);
    assert_eq!(report.grid.refined_planar_nodes, 43);
    assert_eq!(report.work.grid_nodes, 59);
    assert!(report.work.grid_nodes < uniform.work.grid_nodes);
    assert_eq!(report.solution.as_ref().unwrap().routes.len(), 2);
    let routed = report.apply_to(&layout).unwrap();
    assert!(routed.validate(&circuit).is_valid());
    #[cfg(feature = "geometry")]
    routed
        .materialize(&circuit, hypercircuit::MaterializationOptions::default())
        .unwrap();

    assert_eq!(
        layout.negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                grid_mode: NegotiatedGridMode::LocallyRefined {
                    coarse_pitch_multiplier: 4,
                    regions: Vec::new(),
                },
                ..NegotiatedRoutePolicy::default()
            },
        ),
        Err(NegotiatedRouterError::InvalidPolicy)
    );
    assert_eq!(
        layout.negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                grid_mode: NegotiatedGridMode::LocallyRefined {
                    coarse_pitch_multiplier: 4,
                    regions: vec![NegotiatedGridRefinementRegion {
                        min: p(1, 1),
                        max: p(3, 3),
                    }],
                },
                ..NegotiatedRoutePolicy::default()
            },
        ),
        Err(NegotiatedRouterError::InvalidPolicy)
    );
}

#[test]
fn adaptive_router_synthesizes_exact_capacity_after_a_coarse_channel_failure() {
    let (circuit, mut layout) = fixture(false);
    layout.outline.exterior = vec![p(0, 0), p(12, 0), p(12, 12), p(0, 12)].into();
    for (instance, position) in [
        ("A0", p(2, 6)),
        ("A1", p(10, 6)),
        ("B0", p(4, 6)),
        ("B1", p(8, 6)),
    ] {
        layout
            .placements
            .iter_mut()
            .find(|placement| placement.instance.as_str() == instance)
            .unwrap()
            .position = position;
    }
    let policy = NegotiatedAdaptiveRoutePolicy {
        route_policy: NegotiatedRoutePolicy {
            maximum_iterations: 2,
            present_congestion_penalty: 0,
            history_increment: 1,
            via_land_diameter: Real::one(),
            via_drill_diameter: Real::one(),
            ..NegotiatedRoutePolicy::default()
        },
        coarse_pitch_multiplier: 6,
        refinement_padding_steps: 2,
        maximum_refinement_rounds: 2,
        maximum_regions: 4,
    };
    let report = layout
        .adaptive_negotiated_autoroute(&circuit, policy.clone())
        .unwrap();
    assert_eq!(report.status, NegotiatedAdaptiveRouteStatus::Complete);
    assert_eq!(report.policy, policy);
    assert_eq!(report.rounds.len(), 2);
    assert_eq!(
        report.rounds[0].report.status,
        NegotiatedRouteStatus::IterationLimit
    );
    assert_eq!(
        report.rounds[0].report.grid.mode,
        NegotiatedGridMode::FeatureAligned {
            coarse_pitch_multiplier: 6,
            feature_halo_steps: 0,
        }
    );
    let synthesized = NegotiatedGridRefinementRegion {
        min: p(0, 4),
        max: p(12, 8),
    };
    assert_eq!(report.rounds[0].proposed_regions, vec![synthesized.clone()]);
    assert_eq!(report.refinement_regions, vec![synthesized.clone()]);
    assert_eq!(
        report.rounds[1].report.grid.mode,
        NegotiatedGridMode::LocallyRefined {
            coarse_pitch_multiplier: 6,
            regions: vec![synthesized],
        }
    );
    assert_eq!(
        report.final_report().status,
        NegotiatedRouteStatus::Complete
    );
    assert_eq!(report.final_report().grid.active_planar_nodes, 71);
    assert!(report.rounds[1].proposed_regions.is_empty());
    assert_eq!(
        report,
        layout
            .adaptive_negotiated_autoroute(&circuit, policy.clone())
            .unwrap()
    );
    let routed = report.apply_to(&layout).unwrap();
    assert!(routed.validate(&circuit).is_valid());
    #[cfg(feature = "geometry")]
    routed
        .materialize(&circuit, hypercircuit::MaterializationOptions::default())
        .unwrap();

    let limited = layout
        .adaptive_negotiated_autoroute(
            &circuit,
            NegotiatedAdaptiveRoutePolicy {
                maximum_refinement_rounds: 0,
                ..policy.clone()
            },
        )
        .unwrap();
    assert_eq!(
        limited.status,
        NegotiatedAdaptiveRouteStatus::RefinementLimit
    );
    assert_eq!(limited.rounds.len(), 1);
    assert!(limited.refinement_regions.is_empty());
    assert!(!limited.rounds[0].proposed_regions.is_empty());
    assert_eq!(
        layout.adaptive_negotiated_autoroute(
            &circuit,
            NegotiatedAdaptiveRoutePolicy {
                refinement_padding_steps: 0,
                ..policy
            },
        ),
        Err(NegotiatedRouterError::InvalidPolicy)
    );
}

#[test]
fn sparse_octilinear_crossings_share_one_generalized_diagonal_cell() {
    let (circuit, mut layout) = fixture(false);
    layout.outline.exterior = vec![p(0, 0), p(12, 0), p(12, 12), p(0, 12)].into();
    for (instance, position) in [
        ("A0", p(4, 4)),
        ("A1", p(8, 8)),
        ("B0", p(4, 8)),
        ("B1", p(8, 4)),
    ] {
        layout
            .placements
            .iter_mut()
            .find(|placement| placement.instance.as_str() == instance)
            .unwrap()
            .position = position;
    }
    layout.rules.route_constraint_regions = vec![
        RouteConstraintRegion {
            id: RouteConstraintRegionId::new("rising-only").unwrap(),
            nets: vec![NetId::new("A").unwrap()],
            boundary: vec![p(0, 0), p(12, 0), p(12, 12), p(0, 12)],
            allowed_layers: vec![TraceLayer(0)],
            allowed_directions: vec![RouteDirection::DiagonalRising],
            allow_vias: false,
        },
        RouteConstraintRegion {
            id: RouteConstraintRegionId::new("falling-only").unwrap(),
            nets: vec![NetId::new("B").unwrap()],
            boundary: vec![p(0, 0), p(12, 0), p(12, 12), p(0, 12)],
            allowed_layers: vec![TraceLayer(0)],
            allowed_directions: vec![RouteDirection::DiagonalFalling],
            allow_vias: false,
        },
    ];
    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                grid_pitch: Real::from(2),
                grid_mode: NegotiatedGridMode::LocallyRefined {
                    coarse_pitch_multiplier: 2,
                    regions: vec![NegotiatedGridRefinementRegion {
                        min: p(4, 8),
                        max: p(8, 12),
                    }],
                },
                planar_topology: NegotiatedPlanarTopology::Octilinear,
                maximum_iterations: 1,
                present_congestion_penalty: 0,
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::IterationLimit);
    assert_eq!(report.grid.active_planar_nodes, 21);
    assert_eq!(report.grid.refined_planar_nodes, 5);
    assert!(report.iteration_states[0].conflicts.iter().any(|conflict| {
        matches!(
            &conflict.geometry,
            NegotiatedRouteConflictGeometry::DiagonalCell {
                lower_left,
                upper_right,
                layer: TraceLayer(0),
            } if lower_left == &p(4, 4) && upper_right == &p(8, 8)
        )
    }));
}

#[test]
fn bounded_any_angle_visibility_routes_and_accounts_for_exact_crossing_clearance() {
    let (circuit, mut layout) = fixture(false);
    for (instance, position) in [
        ("A0", p(2, 2)),
        ("A1", p(8, 5)),
        ("B0", p(2, 5)),
        ("B1", p(8, 2)),
    ] {
        layout
            .placements
            .iter_mut()
            .find(|placement| placement.instance.as_str() == instance)
            .unwrap()
            .position = position;
    }
    layout.rules.route_constraint_regions = vec![
        RouteConstraintRegion {
            id: RouteConstraintRegionId::new("a-any-angle").unwrap(),
            nets: vec![NetId::new("A").unwrap()],
            boundary: vec![p(0, 0), p(10, 0), p(10, 10), p(0, 10)],
            allowed_layers: vec![TraceLayer(0)],
            allowed_directions: vec![RouteDirection::Arbitrary],
            allow_vias: false,
        },
        RouteConstraintRegion {
            id: RouteConstraintRegionId::new("b-any-angle").unwrap(),
            nets: vec![NetId::new("B").unwrap()],
            boundary: vec![p(0, 0), p(10, 0), p(10, 10), p(0, 10)],
            allowed_layers: vec![TraceLayer(0)],
            allowed_directions: vec![RouteDirection::Arbitrary],
            allow_vias: false,
        },
    ];
    let topology = NegotiatedPlanarTopology::AnyAngle {
        maximum_neighbors_per_node: 128,
    };
    let crossing = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                planar_topology: topology,
                maximum_iterations: 1,
                present_congestion_penalty: 0,
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(crossing.status, NegotiatedRouteStatus::IterationLimit);
    assert_eq!(crossing.iteration_states[0].conflicts.len(), 2);
    assert!(
        crossing.iteration_states[0]
            .conflicts
            .iter()
            .all(|conflict| {
                matches!(
                    conflict.geometry,
                    NegotiatedRouteConflictGeometry::Segment { .. }
                ) && conflict.nets == vec![NetId::new("A").unwrap(), NetId::new("B").unwrap()]
            })
    );

    let complete = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                planar_topology: topology,
                nets: vec![NetId::new("A").unwrap()],
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(complete.status, NegotiatedRouteStatus::Complete);
    let solution = complete.solution.as_ref().unwrap();
    assert_eq!(solution.routes.len(), 1);
    assert!(matches!(
        solution.routes[0].segments.as_slice(),
        [PcbRouteSegment::Line(line)]
            if line.start() == &p(2, 2) && line.end() == &p(8, 5)
    ));
    let mut problem = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    problem
        .problem
        .terminals
        .retain(|terminal| terminal.net.as_str() == "A");
    assert_eq!(
        solution.quality_report(&problem.problem).status,
        RoutingQualityStatus::Complete
    );
    let routed = complete.apply_to(&layout).unwrap();
    assert!(routed.validate(&circuit).is_valid());
    #[cfg(feature = "geometry")]
    routed
        .materialize(&circuit, hypercircuit::MaterializationOptions::default())
        .unwrap();

    assert_eq!(
        layout.negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                planar_topology: NegotiatedPlanarTopology::AnyAngle {
                    maximum_neighbors_per_node: 0,
                },
                ..NegotiatedRoutePolicy::default()
            },
        ),
        Err(NegotiatedRouterError::InvalidPolicy)
    );
}

#[test]
fn octilinear_router_accounts_for_crossing_diagonal_cells_and_reports_exact_quality() {
    let (circuit, mut layout) = fixture(true);
    for placement in &mut layout.placements {
        placement.position = match placement.instance.as_str() {
            "A0" => p(2, 2),
            "A1" => p(8, 8),
            "B0" => p(2, 8),
            "B1" => p(8, 2),
            _ => unreachable!("fixture has four terminal placements"),
        };
    }
    layout
        .rules
        .route_constraint_regions
        .push(RouteConstraintRegion {
            id: RouteConstraintRegionId::new("a-rising-diagonal").unwrap(),
            boundary: vec![p(1, 1), p(9, 1), p(9, 9), p(1, 9)],
            nets: vec![NetId::new("A").unwrap()],
            allowed_layers: vec![TraceLayer(0)],
            allowed_directions: vec![RouteDirection::DiagonalRising],
            allow_vias: false,
        });
    assert_eq!(
        layout.negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                nets: vec![NetId::new("A").unwrap()],
                planar_topology: NegotiatedPlanarTopology::Octilinear,
                ..NegotiatedRoutePolicy::default()
            }
        ),
        Err(NegotiatedRouterError::InvalidPolicy)
    );
    let crossing_policy = NegotiatedRoutePolicy {
        grid_pitch: Real::from(2),
        planar_topology: NegotiatedPlanarTopology::Octilinear,
        maximum_iterations: 1,
        present_congestion_penalty: 0,
        history_increment: 12,
        via_penalty: 4,
        ..NegotiatedRoutePolicy::default()
    };
    let crossing = layout
        .negotiated_autoroute(&circuit, crossing_policy.clone())
        .unwrap();
    assert_eq!(crossing.status, NegotiatedRouteStatus::IterationLimit);
    assert_eq!(crossing.policy, crossing_policy);
    assert!(
        crossing.iteration_states[0]
            .conflicts
            .iter()
            .any(|conflict| {
                matches!(
                    conflict.geometry,
                    NegotiatedRouteConflictGeometry::DiagonalCell { .. }
                )
            })
    );
    let crossing_svg = crossing
        .iteration_svg(
            &layout,
            NegotiatedRouteSvgOptions {
                layer: Some(TraceLayer(0)),
                ..NegotiatedRouteSvgOptions::default()
            },
        )
        .unwrap();
    assert!(crossing_svg.svg.contains("data-conflict=\"diagonal-cell\""));
    let policy = NegotiatedRoutePolicy {
        nets: vec![NetId::new("A").unwrap()],
        maximum_iterations: 8,
        ..crossing_policy
    };
    let report = layout
        .negotiated_autoroute(&circuit, policy.clone())
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete, "{report:?}");
    assert_eq!(report.policy, policy);
    assert_eq!(report.route_constraint_evidence.len(), 1);
    assert_eq!(
        report.route_constraint_evidence[0].constrained_planar_edges,
        3
    );
    let solution = report.solution.as_ref().unwrap();
    assert!(solution.routes.iter().any(|route| {
        route.segments.iter().any(|segment| {
            let PcbRouteSegment::Line(line) = segment else {
                return false;
            };
            let dx = line.end().x.clone() - line.start().x.clone();
            let dy = line.end().y.clone() - line.start().y.clone();
            dx.clone() * dx.clone() == dy.clone() * dy.clone() && dx != Real::zero()
        })
    }));
    let mut problem = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    problem
        .problem
        .terminals
        .retain(|terminal| terminal.net.as_str() == "A");
    let quality = solution.quality_report(&problem.problem);
    assert_eq!(quality.status, RoutingQualityStatus::Complete);
    assert!(quality.issues.is_empty());
    assert!(quality.euclidean_mst_lower_bound.is_some());
    assert!(
        quality.routed_length.as_ref().unwrap()
            >= quality.euclidean_mst_lower_bound.as_ref().unwrap()
    );
    let routed = report.apply_to(&layout).unwrap();
    assert!(routed.validate(&circuit).is_valid());
    #[cfg(feature = "geometry")]
    routed
        .materialize(&circuit, hypercircuit::MaterializationOptions::default())
        .unwrap();
}

#[test]
fn insufficient_single_layer_capacity_remains_an_audited_iteration_limit() {
    let (circuit, layout) = fixture(false);
    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                maximum_iterations: 2,
                present_congestion_penalty: 0,
                history_increment: 1,
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::IterationLimit);
    assert_eq!(report.iterations.len(), 2);
    assert_eq!(report.work.iteration_budget, 2);
    assert_eq!(report.work.iterations_executed, 2);
    assert!(report.work.iteration_budget_exhausted);
    assert!(report.solution.is_none());
}

#[test]
fn per_connection_expansion_budget_is_retained_and_audited() {
    let (circuit, mut layout) = fixture(true);
    layout.keepouts.push(PcbKeepout {
        id: KeepoutId::new("bounded-search-barrier").unwrap(),
        boundary: vec![p(4, 0), p(6, 0), p(6, 10), p(4, 10)],
        scope: KeepoutScope::Copper(vec![TraceLayer(0)]),
    });
    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                nets: vec![NetId::new("A").unwrap()],
                maximum_iterations: 1,
                maximum_expansions_per_connection: 16,
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::IterationLimit);
    assert!(matches!(
        report.failures.as_slice(),
        [NegotiatedRouteFailure::ExpansionLimit { net, .. }] if net.as_str() == "A"
    ));
    assert_eq!(report.work.expansion_budget_per_connection, 16);
    assert_eq!(report.work.expansion_limit_failures, 1);
    assert!(report.work.iteration_budget_exhausted);
    let svg = report
        .iteration_svg(&layout, NegotiatedRouteSvgOptions::default())
        .unwrap();
    assert_eq!(svg.failures, 1);
    assert_eq!(svg.rendered_conflicts, 0);
    assert!(svg.svg.contains("data-failure-index=\"0\""));
    assert!(svg.svg.contains("ExpansionLimit"));
    let html = report
        .replay_html(&layout, NegotiatedRouteHtmlOptions::default())
        .unwrap();
    assert_eq!(html.rendered_iterations, 1);
    assert_eq!(html.failures, 1);
    assert!(html.html.contains("data-failures=\"1\""));
}

#[test]
fn negotiated_router_fixture_corpus_has_reproducible_work_bounds() {
    let (direct_circuit, direct_layout) = fixture(false);
    let mut cases = vec![(
        "direct-single-layer",
        direct_circuit,
        direct_layout,
        NegotiatedRoutePolicy {
            nets: vec![NetId::new("A").unwrap()],
            ..NegotiatedRoutePolicy::default()
        },
        128_usize,
    )];
    let (adaptive_circuit, mut adaptive_layout) = fixture(false);
    adaptive_layout.placements[0].position.x = (Real::from(3) / Real::from(2)).unwrap();
    adaptive_layout.placements[1].position.x = (Real::from(17) / Real::from(2)).unwrap();
    cases.push((
        "feature-aligned-off-lattice",
        adaptive_circuit,
        adaptive_layout,
        NegotiatedRoutePolicy {
            grid_mode: NegotiatedGridMode::FeatureAligned {
                coarse_pitch_multiplier: 4,
                feature_halo_steps: 1,
            },
            nets: vec![NetId::new("A").unwrap()],
            ..NegotiatedRoutePolicy::default()
        },
        256,
    ));
    for (name, circuit, layout, policy, expansion_bound) in cases {
        let first = layout
            .negotiated_autoroute(&circuit, policy.clone())
            .unwrap();
        let replay = layout.negotiated_autoroute(&circuit, policy).unwrap();
        assert_eq!(first.status, NegotiatedRouteStatus::Complete, "{name}");
        assert!(
            first.work.expanded_states_total <= expansion_bound,
            "{name}: {:?}",
            first.work
        );
        assert_eq!(replay.work, first.work, "{name}");
        assert_eq!(replay.iteration_states, first.iteration_states, "{name}");
    }
}

#[test]
fn layer_scoped_copper_keepout_forces_a_back_layer_escape() {
    let (circuit, mut layout) = fixture(true);
    layout.keepouts.push(PcbKeepout {
        id: KeepoutId::new("front-barrier").unwrap(),
        boundary: vec![p(4, 0), p(6, 0), p(6, 10), p(4, 10)],
        scope: KeepoutScope::Copper(vec![TraceLayer(0)]),
    });
    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                nets: vec![NetId::new("A").unwrap()],
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete);
    let routed = report.apply_to(&layout).unwrap();
    assert!(routed.vias.iter().any(|via| via.net.as_str() == "A"));
    assert!(
        routed
            .routes
            .iter()
            .any(|route| route.net.as_str() == "A" && route.layer == TraceLayer(1))
    );
}

#[test]
fn regional_width_and_clearance_change_search_envelopes_and_split_output() {
    let (circuit, mut layout) = fixture(true);
    layout.placements[0].position = p(3, 6);
    layout.placements[1].position = p(9, 6);
    let region = RouteRuleRegionId::new("connector-neckdown").unwrap();
    layout.rules.route_rule_regions.push(RouteRuleRegion {
        id: region.clone(),
        boundary: vec![p(0, 0), p(5, 0), p(5, 10), p(0, 10)],
        nets: vec![NetId::new("A").unwrap()],
        min_trace_width: Some(Real::from(2)),
        min_clearance: Some(Real::one()),
    });
    assert!(layout.validate(&circuit).is_valid());
    let handoff = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    assert_eq!(handoff.problem.route_rule_regions[0].id, region);
    #[cfg(feature = "interchange")]
    {
        let document = hypercircuit::SemanticDocument::new(circuit.clone(), None)
            .unwrap()
            .with_pcb(layout.clone())
            .unwrap();
        assert_eq!(
            hypercircuit::SemanticDocument::from_json(&document.to_json_pretty().unwrap()).unwrap(),
            document
        );
    }

    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                nets: vec![NetId::new("A").unwrap()],
                grid_pitch: Real::from(3),
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete);
    assert_eq!(report.route_rule_region_evidence.len(), 1);
    let evidence = &report.route_rule_region_evidence[0];
    assert_eq!(evidence.region, region);
    assert!(evidence.constrained_planar_edges > 0);
    assert_eq!(
        evidence.width_override_edges,
        evidence.constrained_planar_edges
    );
    assert_eq!(
        evidence.clearance_override_edges,
        evidence.constrained_planar_edges + evidence.constrained_vias
    );
    let routed = report.apply_to(&layout).unwrap();
    let widths = routed
        .routes
        .iter()
        .filter(|route| route.net.as_str() == "A")
        .map(|route| route.width.clone())
        .collect::<Vec<_>>();
    assert!(widths.contains(&Real::from(2)));
    assert!(widths.contains(&Real::one()));
    assert!(routed.validate(&circuit).is_valid());
}

#[test]
fn invalid_regional_route_rules_fail_before_routing() {
    let (circuit, mut layout) = fixture(true);
    layout.rules.route_rule_regions.push(RouteRuleRegion {
        id: RouteRuleRegionId::new("invalid").unwrap(),
        boundary: vec![p(0, 0), p(1, 0)],
        nets: vec![NetId::new("absent").unwrap()],
        min_trace_width: Some(Real::zero()),
        min_clearance: None,
    });
    let validation = layout.validate(&circuit);
    assert!(validation.issues.iter().any(|issue| matches!(
        issue,
        LayoutValidationIssue::UnknownRouteRuleRegionNet { .. }
    )));
    assert!(
        validation
            .issues
            .iter()
            .any(|issue| matches!(issue, LayoutValidationIssue::InvalidRouteRuleRegion(_)))
    );
    assert!(matches!(
        layout.negotiated_autoroute(&circuit, NegotiatedRoutePolicy::default()),
        Err(NegotiatedRouterError::InvalidProblem(_))
    ));
}

#[test]
fn named_via_style_drives_search_emission_and_use_evidence() {
    let (circuit, mut layout) = fixture(true);
    let net = NetId::new("A").unwrap();
    let style = ViaStyleId::new("laser-via").unwrap();
    layout.keepouts.push(PcbKeepout {
        id: KeepoutId::new("front-barrier").unwrap(),
        boundary: vec![p(4, 0), p(6, 0), p(6, 10), p(4, 10)],
        scope: KeepoutScope::Copper(vec![TraceLayer(0)]),
    });
    layout.rules.via_styles.push(ViaStyle {
        id: style.clone(),
        land_diameter: Real::one(),
        drill_diameter: (Real::one() / Real::from(2)).unwrap(),
        plating: Plating::Plated,
        mask: ViaMaskIntent::open(Real::zero()),
        allowed_spans: vec![ViaStyleSpan {
            start_layer: TraceLayer(0),
            end_layer: TraceLayer(1),
        }],
    });
    let base_class = NetClassId::new("manufacturing-base").unwrap();
    layout.rules.net_classes.push(NetClass {
        id: base_class.clone(),
        parent: None,
        nets: Vec::new(),
        min_trace_width: Some(Real::one()),
        preferred_trace_width: None,
        min_clearance: None,
        preferred_via_land_diameter: None,
        preferred_via_drill_diameter: None,
        preferred_via_style: Some(style.clone()),
        max_length: Some(Real::from(20)),
        max_via_count: Some(4),
        target_impedance_ohms: None,
        impedance_tolerance_ohms: None,
        requires_reference_plane: true,
    });
    layout.rules.net_classes.push(NetClass {
        id: NetClassId::new("styled-a").unwrap(),
        parent: Some(base_class.clone()),
        nets: vec![net.clone()],
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
    });
    assert!(layout.validate(&circuit).is_valid());
    let handoff = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    assert_eq!(handoff.problem.via_styles[0].id, style);
    let resolved = handoff
        .problem
        .net_classes
        .iter()
        .find(|class| class.id.as_str() == "styled-a")
        .unwrap();
    assert_eq!(
        resolved.lineage,
        vec![base_class, NetClassId::new("styled-a").unwrap()]
    );
    assert_eq!(resolved.preferred_via_style.as_ref(), Some(&style));
    assert_eq!(resolved.min_trace_width, Some(Real::one()));
    assert_eq!(resolved.max_via_count, Some(4));
    assert!(resolved.requires_reference_plane);
    #[cfg(feature = "interchange")]
    {
        let document = hypercircuit::SemanticDocument::new(circuit.clone(), None)
            .unwrap()
            .with_pcb(layout.clone())
            .unwrap();
        let decoded =
            hypercircuit::SemanticDocument::from_json(&document.to_json_pretty().unwrap()).unwrap();
        assert_eq!(decoded, document);
    }

    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                nets: vec![net.clone()],
                via_land_diameter: Real::from(3),
                via_drill_diameter: Real::from(2),
                via_mask: ViaMaskIntent::tented(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete);
    assert_eq!(report.via_style_evidence.len(), 1);
    let evidence = &report.via_style_evidence[0];
    assert_eq!(evidence.net, net);
    assert_eq!(evidence.style.as_ref(), Some(&style));
    assert!(evidence.generated_vias > 0);
    assert_eq!(evidence.land_diameter, Real::one());
    assert_eq!(
        evidence.drill_diameter,
        (Real::one() / Real::from(2)).unwrap()
    );
    assert_eq!(evidence.mask, ViaMaskIntent::open(Real::zero()));
    let routed = report.apply_to(&layout).unwrap();
    assert!(routed.vias.iter().all(|via| {
        via.land_diameter == Real::one()
            && via.drill_diameter == (Real::one() / Real::from(2)).unwrap()
            && via.mask == ViaMaskIntent::open(Real::zero())
    }));
    #[cfg(feature = "geometry")]
    {
        let materialized = routed
            .materialize(&circuit, hypercircuit::MaterializationOptions::default())
            .unwrap();
        assert!(materialized.drills.iter().all(|drill| {
            drill.plating == Plating::Plated
                && drill.shape
                    == hypercircuit::DrillShape::Round {
                        diameter: (Real::one() / Real::from(2)).unwrap(),
                    }
        }));
        assert!(
            materialized.process_features.iter().any(|feature| {
                feature.kind == hypercircuit::ProcessFeatureKind::ViaMaskOpening
            })
        );
        #[cfg(feature = "drc")]
        {
            let drc = hypercircuit::HyperDrcHandoff::from_materialization(&routed, &materialized);
            let class = drc
                .net_classes
                .iter()
                .find(|class| class.name == "styled-a")
                .unwrap();
            assert_eq!(class.min_width, Some(Real::one()));
            assert_eq!(class.max_via_count, Some(4));
            assert_eq!(class.requires_reference_plane, Some(true));
        }
    }
}

#[test]
fn invalid_or_unknown_via_styles_fail_structural_validation() {
    let (circuit, mut layout) = fixture(true);
    layout.rules.via_styles.push(ViaStyle {
        id: ViaStyleId::new("invalid-span").unwrap(),
        land_diameter: Real::one(),
        drill_diameter: Real::one(),
        plating: Plating::Plated,
        mask: ViaMaskIntent::tented(),
        allowed_spans: vec![ViaStyleSpan {
            start_layer: TraceLayer(0),
            end_layer: TraceLayer(0),
        }],
    });
    layout.rules.net_classes.push(NetClass {
        id: NetClassId::new("unknown-style").unwrap(),
        parent: None,
        nets: vec![NetId::new("A").unwrap()],
        min_trace_width: None,
        preferred_trace_width: None,
        min_clearance: None,
        preferred_via_land_diameter: None,
        preferred_via_drill_diameter: None,
        preferred_via_style: Some(ViaStyleId::new("absent").unwrap()),
        max_length: None,
        max_via_count: None,
        target_impedance_ohms: None,
        impedance_tolerance_ohms: None,
        requires_reference_plane: false,
    });
    let report = layout.validate(&circuit);
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        LayoutValidationIssue::InvalidViaStyle(style) if style.as_str() == "invalid-span"
    )));
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        LayoutValidationIssue::UnknownNetClassViaStyle { style, .. }
            if style.as_str() == "absent"
    )));
}

#[test]
fn net_class_inheritance_rejects_missing_parents_cycles_and_ambiguous_nets() {
    let make_class = |id: &str, parent: Option<&str>, nets: Vec<NetId>| NetClass {
        id: NetClassId::new(id).unwrap(),
        parent: parent.map(|parent| NetClassId::new(parent).unwrap()),
        nets,
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
    };

    let (circuit, mut missing) = fixture(true);
    missing
        .rules
        .net_classes
        .push(make_class("derived", Some("absent"), Vec::new()));
    assert!(missing.validate(&circuit).issues.iter().any(|issue| {
        matches!(
            issue,
            LayoutValidationIssue::UnknownNetClassParent { class, parent }
                if class.as_str() == "derived" && parent.as_str() == "absent"
        )
    }));

    let (_, mut cyclic) = fixture(true);
    cyclic
        .rules
        .net_classes
        .push(make_class("first", Some("second"), Vec::new()));
    cyclic
        .rules
        .net_classes
        .push(make_class("second", Some("first"), Vec::new()));
    assert!(
        cyclic
            .validate(&circuit)
            .issues
            .iter()
            .any(|issue| { matches!(issue, LayoutValidationIssue::NetClassInheritanceCycle(_)) })
    );

    let (_, mut ambiguous) = fixture(true);
    ambiguous.rules.net_classes.extend([
        make_class("first", None, vec![NetId::new("A").unwrap()]),
        make_class("second", None, vec![NetId::new("A").unwrap()]),
    ]);
    assert!(ambiguous.validate(&circuit).issues.iter().any(|issue| {
        matches!(
            issue,
            LayoutValidationIssue::NetInMultipleClasses(net) if net.as_str() == "A"
        )
    }));
}

#[test]
fn route_regions_and_escape_policies_are_enforced_with_use_evidence() {
    let (circuit, mut layout) = fixture(true);
    layout
        .rules
        .route_constraint_regions
        .push(RouteConstraintRegion {
            id: RouteConstraintRegionId::new("back-channel").unwrap(),
            boundary: vec![p(5, 0), p(7, 0), p(7, 10), p(5, 10)],
            nets: vec![NetId::new("A").unwrap()],
            allowed_layers: vec![TraceLayer(1)],
            allowed_directions: vec![RouteDirection::Horizontal],
            allow_vias: false,
        });
    layout.rules.escape_policies.push(EscapePolicy {
        id: EscapePolicyId::new("a0-horizontal-fanout").unwrap(),
        instances: vec![CircuitInstanceId::new("A0").unwrap()],
        nets: vec![NetId::new("A").unwrap()],
        max_distance: Real::one(),
        allowed_layers: vec![TraceLayer(0)],
        allowed_directions: vec![RouteDirection::Horizontal],
        allow_vias: false,
    });
    assert!(layout.validate(&circuit).is_valid());
    let handoff = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    assert_eq!(handoff.problem.route_constraint_regions.len(), 1);
    assert_eq!(handoff.problem.escape_policies.len(), 1);

    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                nets: vec![NetId::new("A").unwrap()],
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete);
    assert_eq!(report.route_constraint_evidence.len(), 1);
    assert!(report.route_constraint_evidence[0].constrained_planar_edges > 0);
    assert_eq!(report.route_constraint_evidence[0].constrained_vias, 0);
    assert_eq!(report.escape_policy_evidence.len(), 1);
    assert!(report.escape_policy_evidence[0].constrained_planar_edges > 0);
    assert_eq!(report.escape_policy_evidence[0].constrained_vias, 0);

    let routed = report.apply_to(&layout).unwrap();
    assert!(routed.validate(&circuit).is_valid());
    assert!(
        routed
            .routes
            .iter()
            .any(|route| route.net.as_str() == "A" && route.layer == TraceLayer(1))
    );
    assert!(
        routed
            .vias
            .iter()
            .any(|via| via.net.as_str() == "A" && via.center.x == Real::from(4))
    );
}

#[test]
fn exact_length_tuning_inserts_a_bounded_serpentine_and_replays_idempotently() {
    let (circuit, mut layout) = fixture(true);
    let route_id = RouteId::new("a-authored").unwrap();
    layout.routes.push(PcbRoute {
        id: route_id.clone(),
        net: NetId::new("A").unwrap(),
        layer: TraceLayer(0),
        width: Real::one(),
        segments: vec![PcbRouteSegment::Line(LinePathSegment::new(
            p(2, 5),
            p(8, 5),
        ))],
    });
    let pattern = LengthTuningPatternId::new("a-serpentine").unwrap();
    layout
        .rules
        .length_tuning_patterns
        .push(LengthTuningPattern {
            id: pattern.clone(),
            net: NetId::new("A").unwrap(),
            route: Some(route_id.clone()),
            region: vec![p(1, 4), p(9, 4), p(9, 8), p(1, 8)],
            target_length: Real::from(10),
            tolerance: Real::zero(),
            amplitude: Real::one(),
            pitch: Real::one(),
            maximum_cycles: 3,
            side: LengthTuningSide::Left,
        });
    assert!(layout.validate(&circuit).is_valid());

    let report = layout.realize_length_tuning(&pattern);
    assert_eq!(report.status, LengthTuningStatus::Applied);
    assert_eq!(report.original_length, Some(Real::from(6)));
    assert_eq!(report.realized_length, Some(Real::from(10)));
    assert_eq!(report.cycles, 2);
    assert_eq!(report.route, Some(route_id));
    let tuned = report.apply_to(&layout).unwrap();
    assert!(tuned.validate(&circuit).is_valid());
    let tuned_route = tuned
        .routes
        .iter()
        .find(|route| route.id.as_str() == "a-authored")
        .unwrap();
    assert_eq!(tuned_route.segments.len(), 9);
    assert!(
        tuned_route
            .segments
            .iter()
            .any(|segment| segment.start().y == Real::from(6))
    );
    let replay = tuned.realize_length_tuning(&pattern);
    assert_eq!(replay.status, LengthTuningStatus::AlreadySatisfied);
    assert_eq!(replay.realized_length, Some(Real::from(10)));
}

fn install_differential_phase_tuning(layout: &mut PcbLayout) -> PhaseTuningGroupId {
    let positive_route = RouteId::new("a-phase-route").unwrap();
    let negative_route = RouteId::new("b-phase-route").unwrap();
    layout.routes.extend([
        PcbRoute {
            id: positive_route.clone(),
            net: NetId::new("A").unwrap(),
            layer: TraceLayer(0),
            width: Real::one(),
            segments: vec![PcbRouteSegment::Line(LinePathSegment::new(
                p(2, 4),
                p(8, 4),
            ))],
        },
        PcbRoute {
            id: negative_route.clone(),
            net: NetId::new("B").unwrap(),
            layer: TraceLayer(0),
            width: Real::one(),
            segments: vec![PcbRouteSegment::Line(LinePathSegment::new(
                p(2, 6),
                p(8, 6),
            ))],
        },
    ]);
    let positive_pattern = LengthTuningPatternId::new("a-phase-pattern").unwrap();
    let negative_pattern = LengthTuningPatternId::new("b-phase-pattern").unwrap();
    layout.rules.length_tuning_patterns.extend([
        LengthTuningPattern {
            id: positive_pattern.clone(),
            net: NetId::new("A").unwrap(),
            route: Some(positive_route),
            region: vec![p(1, 3), p(9, 3), p(9, 6), p(1, 6)],
            target_length: Real::from(10),
            tolerance: Real::zero(),
            amplitude: Real::one(),
            pitch: Real::one(),
            maximum_cycles: 2,
            side: LengthTuningSide::Left,
        },
        LengthTuningPattern {
            id: negative_pattern.clone(),
            net: NetId::new("B").unwrap(),
            route: Some(negative_route),
            region: vec![p(1, 5), p(9, 5), p(9, 8), p(1, 8)],
            target_length: Real::from(10),
            tolerance: Real::zero(),
            amplitude: Real::one(),
            pitch: Real::one(),
            maximum_cycles: 2,
            side: LengthTuningSide::Left,
        },
    ]);
    let group = PhaseTuningGroupId::new("usb-data-phase").unwrap();
    layout.rules.phase_tuning_groups.push(PhaseTuningGroup {
        id: group.clone(),
        patterns: vec![positive_pattern, negative_pattern],
        differential_pair: Some(DifferentialPairId::new("usb-data").unwrap()),
        maximum_skew: Real::zero(),
        minimum_clearance: Real::zero(),
    });
    group
}

#[test]
fn differential_phase_group_tunes_both_routes_atomically_and_replays() {
    let (circuit, mut layout) = differential_fixture();
    let group = install_differential_phase_tuning(&mut layout);
    assert!(layout.validate(&circuit).is_valid());
    let handoff = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    assert_eq!(
        handoff.problem.phase_tuning_groups,
        layout.rules.phase_tuning_groups
    );

    let report = layout.realize_phase_tuning(&circuit, &group);
    assert_eq!(report.status, PhaseTuningStatus::Applied);
    assert_eq!(report.members.len(), 2);
    assert_eq!(report.tuned_routes.len(), 2);
    assert_eq!(report.realized_skew, Some(Real::zero()));
    assert!(
        report
            .realized_lengths
            .iter()
            .all(|(_, length)| length == &Real::from(10))
    );

    let tuned = report.apply_to(&layout).unwrap();
    assert!(tuned.validate(&circuit).is_valid());
    let positive = tuned
        .routes
        .iter()
        .find(|route| route.id.as_str() == "a-phase-route")
        .unwrap();
    let negative = tuned
        .routes
        .iter()
        .find(|route| route.id.as_str() == "b-phase-route")
        .unwrap();
    assert_eq!(positive.segments.len(), negative.segments.len());
    assert!(
        positive
            .segments
            .iter()
            .zip(&negative.segments)
            .all(|(positive, negative)| {
                positive.start().x == negative.start().x
                    && positive.end().x == negative.end().x
                    && negative.start().y.clone() - positive.start().y.clone() == Real::from(2)
                    && negative.end().y.clone() - positive.end().y.clone() == Real::from(2)
            })
    );

    let replay = tuned.realize_phase_tuning(&circuit, &group);
    assert_eq!(replay.status, PhaseTuningStatus::AlreadySatisfied);
    assert!(replay.tuned_routes.is_empty());
    assert_eq!(replay.realized_skew, Some(Real::zero()));
}

#[test]
fn phase_tuning_synthesis_generates_and_certifies_exact_pair_intent() {
    let (circuit, mut layout) = differential_fixture();
    install_differential_phase_tuning(&mut layout);
    layout.rules.length_tuning_patterns.clear();
    layout.rules.phase_tuning_groups.clear();
    let policy = PhaseTuningSynthesisPolicy {
        target_length: Some(Real::from(10)),
        differential_pair: Some(DifferentialPairId::new("usb-data").unwrap()),
        ..PhaseTuningSynthesisPolicy::new(
            PhaseTuningGroupId::new("synthesized-phase").unwrap(),
            [NetId::new("A").unwrap(), NetId::new("B").unwrap()],
            Real::one(),
            Real::one(),
            2,
        )
    };

    let report = layout.synthesize_phase_tuning(&circuit, policy.clone());
    assert_eq!(report.status, PhaseTuningSynthesisStatus::Certified);
    assert_eq!(report.target_length, Some(Real::from(10)));
    assert_eq!(report.patterns.len(), 2);
    assert!(report.patterns.iter().all(|pattern| {
        pattern.region.len() == 4
            && pattern.route.is_some()
            && pattern.target_length == Real::from(10)
    }));
    assert!(report.group.is_some());
    assert!(report.candidate_assemblies_available >= 4);
    assert!(report.candidate_assemblies_considered > 0);
    assert!(report.issues.is_empty());
    let realization = report.realization.as_ref().unwrap();
    assert_eq!(realization.status, PhaseTuningStatus::Applied);
    assert_eq!(realization.realized_skew, Some(Real::zero()));
    assert_eq!(realization.tuned_routes.len(), 2);

    let intent = report.apply_intent_to(&circuit, &layout).unwrap();
    assert!(intent.validate(&circuit).is_valid());
    let replay = intent.realize_phase_tuning(
        &circuit,
        &PhaseTuningGroupId::new("synthesized-phase").unwrap(),
    );
    assert_eq!(replay, *realization);
    let tuned = report.apply_tuned_to(&layout).unwrap();
    assert!(tuned.validate(&circuit).is_valid());
    assert_eq!(
        layout.synthesize_phase_tuning(&circuit, policy.clone()),
        report
    );
    let satisfied = tuned.synthesize_phase_tuning(&circuit, policy.clone());
    assert_eq!(
        satisfied.status,
        PhaseTuningSynthesisStatus::AlreadySatisfied
    );
    let duplicate = intent.synthesize_phase_tuning(&circuit, policy);
    assert_eq!(duplicate.status, PhaseTuningSynthesisStatus::Rejected);
    assert_eq!(
        duplicate.issues,
        vec![PhaseTuningSynthesisIssue::DuplicateIdentity]
    );
}

#[test]
fn phase_tuning_synthesis_retains_its_candidate_certification_bound() {
    let (mut circuit, mut layout) = differential_fixture();
    install_differential_phase_tuning(&mut layout);
    layout.rules.length_tuning_patterns.clear();
    layout.rules.phase_tuning_groups.clear();
    circuit = circuit.with_net(Net {
        id: NetId::new("C").unwrap(),
        is_ground: false,
    });
    layout.routes.push(PcbRoute {
        id: RouteId::new("synthesis-obstacle").unwrap(),
        net: NetId::new("C").unwrap(),
        layer: TraceLayer(0),
        width: Real::one(),
        segments: vec![PcbRouteSegment::Line(LinePathSegment::new(
            p(3, 5),
            p(7, 5),
        ))],
    });
    let report = layout.synthesize_phase_tuning(
        &circuit,
        PhaseTuningSynthesisPolicy {
            target_length: Some(Real::from(10)),
            differential_pair: Some(DifferentialPairId::new("usb-data").unwrap()),
            maximum_candidate_assemblies: 1,
            ..PhaseTuningSynthesisPolicy::new(
                PhaseTuningGroupId::new("bounded-phase").unwrap(),
                [NetId::new("A").unwrap(), NetId::new("B").unwrap()],
                Real::one(),
                Real::one(),
                2,
            )
        },
    );
    assert_eq!(report.status, PhaseTuningSynthesisStatus::Rejected);
    assert_eq!(report.candidate_assemblies_considered, 1);
    assert!(report.candidate_assemblies_available > 1);
    assert_eq!(
        report.issues,
        vec![PhaseTuningSynthesisIssue::CandidateLimitReached]
    );
    assert!(report.patterns.is_empty());
    assert!(report.group.is_none());
    assert!(report.realization.is_none());
    assert!(report.apply_intent_to(&circuit, &layout).is_none());
    assert!(report.apply_tuned_to(&layout).is_none());
}

#[test]
fn phase_group_collision_rejects_every_member_without_partial_application() {
    let (mut circuit, mut layout) = differential_fixture();
    circuit = circuit.with_net(Net {
        id: NetId::new("C").unwrap(),
        is_ground: false,
    });
    let group = install_differential_phase_tuning(&mut layout);
    layout.routes.push(PcbRoute {
        id: RouteId::new("fixed-c-obstacle").unwrap(),
        net: NetId::new("C").unwrap(),
        layer: TraceLayer(0),
        width: Real::one(),
        segments: vec![PcbRouteSegment::Line(LinePathSegment::new(
            p(3, 5),
            p(4, 5),
        ))],
    });
    assert!(layout.validate(&circuit).is_valid());

    let report = layout.realize_phase_tuning(&circuit, &group);
    assert_eq!(report.status, PhaseTuningStatus::Rejected);
    assert!(report.tuned_routes.is_empty());
    assert!(report.apply_to(&layout).is_none());
    assert_eq!(
        report.issues,
        vec![PhaseTuningIssue::ClearanceViolation {
            route: RouteId::new("a-phase-route").unwrap(),
            obstacle: PhaseTuningObstacle::Route(RouteId::new("fixed-c-obstacle").unwrap()),
        }]
    );
    assert_eq!(
        layout
            .routes
            .iter()
            .find(|route| route.id.as_str() == "a-phase-route")
            .unwrap()
            .segments
            .len(),
        1
    );
}

#[test]
fn phase_group_rejects_a_foreign_placed_pad_envelope_atomically() {
    let (mut circuit, mut layout) = differential_fixture();
    let foreign = NetId::new("C").unwrap();
    circuit = circuit
        .with_net(Net {
            id: foreign.clone(),
            is_ground: false,
        })
        .with_instance(CircuitInstance {
            id: CircuitInstanceId::new("C0").unwrap(),
            component: ComponentId::new("C0").unwrap(),
            part: None,
            model: DeviceModelId::new("terminal").unwrap(),
            pins: vec![PinBinding {
                pin: PinRef::new("1").unwrap(),
                net: foreign,
            }],
            parameters: Vec::new(),
        });
    layout.placements.push(PcbPlacement {
        instance: CircuitInstanceId::new("C0").unwrap(),
        land_pattern: LandPatternId::new("terminal-pad").unwrap(),
        position: p(3, 5),
        rotation_degrees: Real::zero(),
        side: BoardSide::Front,
    });
    let group = install_differential_phase_tuning(&mut layout);
    assert!(layout.validate(&circuit).is_valid());

    let report = layout.realize_phase_tuning(&circuit, &group);
    assert_eq!(report.status, PhaseTuningStatus::Rejected);
    assert_eq!(
        report.zone_collision_mode,
        hypercircuit::PhaseTuningZoneCollisionMode::AuthoredBoundary
    );
    assert!(report.realized_zone_evidence.is_empty());
    assert!(report.apply_to(&layout).is_none());
    assert_eq!(
        report.issues,
        vec![PhaseTuningIssue::ClearanceViolation {
            route: RouteId::new("a-phase-route").unwrap(),
            obstacle: PhaseTuningObstacle::Pad {
                instance: CircuitInstanceId::new("C0").unwrap(),
                pad: PadId::new("1").unwrap(),
            },
        }]
    );
}

#[test]
fn phase_tuning_uses_exact_rotated_non_circular_pad_envelopes() {
    let (mut circuit, mut layout) = differential_fixture();
    let foreign = NetId::new("C").unwrap();
    circuit = circuit
        .with_net(Net {
            id: foreign.clone(),
            is_ground: false,
        })
        .with_instance(CircuitInstance {
            id: CircuitInstanceId::new("C0").unwrap(),
            component: ComponentId::new("C0").unwrap(),
            part: None,
            model: DeviceModelId::new("terminal").unwrap(),
            pins: vec![PinBinding {
                pin: PinRef::new("1").unwrap(),
                net: foreign,
            }],
            parameters: Vec::new(),
        });
    let mut foreign_pattern = layout.land_patterns[0].clone();
    foreign_pattern.id = LandPatternId::new("wide-terminal").unwrap();
    foreign_pattern.pads[0].shape = PadShape::Rectangle {
        width: Real::from(6),
        height: Real::one(),
    };
    layout.land_patterns.push(foreign_pattern);
    layout.placements.push(PcbPlacement {
        instance: CircuitInstanceId::new("C0").unwrap(),
        land_pattern: LandPatternId::new("wide-terminal").unwrap(),
        position: p(5, 1),
        rotation_degrees: Real::zero(),
        side: BoardSide::Front,
    });
    let group = install_differential_phase_tuning(&mut layout);
    assert!(layout.validate(&circuit).is_valid());

    let clear = layout.realize_phase_tuning(&circuit, &group);
    assert_eq!(clear.status, PhaseTuningStatus::Applied, "{clear:?}");

    layout
        .land_patterns
        .iter_mut()
        .find(|pattern| pattern.id.as_str() == "wide-terminal")
        .unwrap()
        .pads[0]
        .rotation_degrees = Real::from(90);
    let blocked = layout.realize_phase_tuning(&circuit, &group);
    assert_eq!(blocked.status, PhaseTuningStatus::Rejected);
    assert!(blocked.issues.iter().any(|issue| matches!(
        issue,
        PhaseTuningIssue::ClearanceViolation {
            obstacle: PhaseTuningObstacle::Pad { instance, .. },
            ..
        } if instance.as_str() == "C0"
    )));
}

#[test]
fn phase_group_rejects_a_foreign_copper_zone_atomically() {
    let (mut circuit, mut layout) = differential_fixture();
    let foreign = NetId::new("C").unwrap();
    circuit = circuit.with_net(Net {
        id: foreign.clone(),
        is_ground: false,
    });
    layout.zones.push(CopperZone::solid(
        ZoneId::new("foreign-zone").unwrap(),
        foreign,
        TraceLayer(0),
        vec![p(3, 5), p(4, 5), p(4, 6), p(3, 6)],
    ));
    let group = install_differential_phase_tuning(&mut layout);
    assert!(layout.validate(&circuit).is_valid());

    let report = layout.realize_phase_tuning(&circuit, &group);
    assert_eq!(report.status, PhaseTuningStatus::Rejected);
    assert!(report.apply_to(&layout).is_none());
    assert_eq!(
        report.issues,
        vec![PhaseTuningIssue::ClearanceViolation {
            route: RouteId::new("a-phase-route").unwrap(),
            obstacle: PhaseTuningObstacle::Zone(ZoneId::new("foreign-zone").unwrap()),
        }]
    );

    #[cfg(feature = "geometry")]
    {
        let repoured = layout.realize_phase_tuning_with_realized_zones(
            &circuit,
            &group,
            hypercircuit::MaterializationOptions::default(),
        );
        assert_eq!(repoured.status, PhaseTuningStatus::Applied, "{repoured:#?}");
        assert_eq!(
            repoured.zone_collision_mode,
            hypercircuit::PhaseTuningZoneCollisionMode::RealizedFill
        );
        assert_eq!(repoured.realized_zone_evidence.len(), 2);
        assert!(repoured.realized_zone_evidence.iter().all(|evidence| {
            evidence.zone.as_str() == "foreign-zone"
                && evidence.required_clearance == Real::zero()
                && evidence.status == hypercircuit::PhaseTuningRealizedZoneStatus::Clear
        }));
        let tuned = repoured.apply_to(&layout).unwrap();
        assert!(tuned.validate(&circuit).is_valid());
        tuned
            .materialize(&circuit, hypercircuit::MaterializationOptions::default())
            .unwrap();

        let mut stricter = layout.clone();
        let stricter_group = stricter
            .rules
            .phase_tuning_groups
            .iter_mut()
            .find(|candidate| candidate.id == group)
            .unwrap();
        stricter_group.minimum_clearance = Real::one();
        stricter_group.differential_pair = None;
        stricter
            .routes
            .iter_mut()
            .find(|route| route.id.as_str() == "b-phase-route")
            .unwrap()
            .layer = TraceLayer(1);
        for placement in stricter
            .placements
            .iter_mut()
            .filter(|placement| placement.instance.as_str().starts_with('B'))
        {
            placement.position.y = Real::from(9);
        }
        let blocked = stricter.realize_phase_tuning_with_realized_zones(
            &circuit,
            &group,
            hypercircuit::MaterializationOptions::default(),
        );
        assert_eq!(blocked.status, PhaseTuningStatus::Rejected);
        assert!(blocked.apply_to(&stricter).is_none());
        assert_eq!(
            blocked.issues,
            vec![PhaseTuningIssue::ClearanceViolation {
                route: RouteId::new("a-phase-route").unwrap(),
                obstacle: PhaseTuningObstacle::Zone(ZoneId::new("foreign-zone").unwrap()),
            }]
        );
        assert_eq!(
            blocked.realized_zone_evidence.last().unwrap().status,
            hypercircuit::PhaseTuningRealizedZoneStatus::Collision
        );
    }
}

#[test]
fn phase_group_rejects_a_copper_keepout_atomically() {
    let (circuit, mut layout) = differential_fixture();
    layout.keepouts.push(PcbKeepout {
        id: KeepoutId::new("phase-keepout").unwrap(),
        boundary: vec![p(3, 5), p(4, 5), p(4, 6), p(3, 6)],
        scope: KeepoutScope::Copper(vec![TraceLayer(0)]),
    });
    let group = install_differential_phase_tuning(&mut layout);
    assert!(layout.validate(&circuit).is_valid());

    let report = layout.realize_phase_tuning(&circuit, &group);
    assert_eq!(report.status, PhaseTuningStatus::Rejected);
    assert!(report.apply_to(&layout).is_none());
    assert_eq!(
        report.issues,
        vec![PhaseTuningIssue::ClearanceViolation {
            route: RouteId::new("a-phase-route").unwrap(),
            obstacle: PhaseTuningObstacle::Keepout(KeepoutId::new("phase-keepout").unwrap()),
        }]
    );
}

#[test]
fn advanced_routing_intent_rejects_unknown_targets_and_invalid_bounds() {
    let (circuit, mut layout) = fixture(true);
    layout
        .rules
        .route_constraint_regions
        .push(RouteConstraintRegion {
            id: RouteConstraintRegionId::new("invalid-region").unwrap(),
            boundary: vec![p(1, 1), p(2, 1)],
            nets: vec![NetId::new("absent").unwrap()],
            allowed_layers: Vec::new(),
            allowed_directions: Vec::new(),
            allow_vias: false,
        });
    layout.rules.escape_policies.push(EscapePolicy {
        id: EscapePolicyId::new("invalid-escape").unwrap(),
        instances: vec![CircuitInstanceId::new("absent").unwrap()],
        nets: vec![NetId::new("A").unwrap()],
        max_distance: Real::zero(),
        allowed_layers: vec![TraceLayer(0)],
        allowed_directions: vec![RouteDirection::Horizontal],
        allow_vias: false,
    });
    layout
        .rules
        .length_tuning_patterns
        .push(LengthTuningPattern {
            id: LengthTuningPatternId::new("invalid-tune").unwrap(),
            net: NetId::new("A").unwrap(),
            route: Some(RouteId::new("absent").unwrap()),
            region: vec![p(0, 0), p(1, 0), p(1, 1), p(0, 1)],
            target_length: Real::zero(),
            tolerance: -Real::one(),
            amplitude: Real::one(),
            pitch: Real::one(),
            maximum_cycles: 0,
            side: LengthTuningSide::Left,
        });
    layout.rules.phase_tuning_groups.push(PhaseTuningGroup {
        id: PhaseTuningGroupId::new("invalid-phase").unwrap(),
        patterns: vec![
            LengthTuningPatternId::new("invalid-tune").unwrap(),
            LengthTuningPatternId::new("invalid-tune").unwrap(),
        ],
        differential_pair: Some(DifferentialPairId::new("absent-pair").unwrap()),
        maximum_skew: -Real::one(),
        minimum_clearance: Real::zero(),
    });

    let issues = layout.validate(&circuit).issues;
    assert!(issues.iter().any(|issue| matches!(
        issue,
        LayoutValidationIssue::UnknownRouteConstraintRegionNet { .. }
    )));
    assert!(issues.iter().any(|issue| matches!(
        issue,
        LayoutValidationIssue::InvalidRouteConstraintRegion(_)
    )));
    assert!(issues.iter().any(|issue| matches!(
        issue,
        LayoutValidationIssue::UnknownEscapePolicyInstance { .. }
    )));
    assert!(
        issues
            .iter()
            .any(|issue| matches!(issue, LayoutValidationIssue::InvalidEscapePolicy(_)))
    );
    assert!(
        issues
            .iter()
            .any(|issue| matches!(issue, LayoutValidationIssue::InvalidLengthTuningTarget(_)))
    );
    assert!(
        issues
            .iter()
            .any(|issue| matches!(issue, LayoutValidationIssue::InvalidLengthTuningPattern(_)))
    );
    assert!(
        issues
            .iter()
            .any(|issue| matches!(issue, LayoutValidationIssue::InvalidPhaseTuningTarget(_)))
    );
    assert!(
        issues
            .iter()
            .any(|issue| matches!(issue, LayoutValidationIssue::InvalidPhaseTuningGroup(_)))
    );
}

#[cfg(feature = "interchange")]
#[test]
fn advanced_routing_intent_round_trips_exactly() {
    let (circuit, mut layout) = fixture(true);
    layout
        .rules
        .route_constraint_regions
        .push(RouteConstraintRegion {
            id: RouteConstraintRegionId::new("region").unwrap(),
            boundary: vec![p(3, 3), p(7, 3), p(7, 7), p(3, 7)],
            nets: vec![NetId::new("A").unwrap()],
            allowed_layers: vec![TraceLayer(0)],
            allowed_directions: vec![RouteDirection::Horizontal],
            allow_vias: false,
        });
    layout.rules.escape_policies.push(EscapePolicy {
        id: EscapePolicyId::new("escape").unwrap(),
        instances: vec![CircuitInstanceId::new("A0").unwrap()],
        nets: vec![NetId::new("A").unwrap()],
        max_distance: (Real::from(3) / Real::from(2)).unwrap(),
        allowed_layers: vec![TraceLayer(0)],
        allowed_directions: vec![RouteDirection::Horizontal],
        allow_vias: false,
    });
    layout
        .rules
        .length_tuning_patterns
        .push(LengthTuningPattern {
            id: LengthTuningPatternId::new("tune").unwrap(),
            net: NetId::new("A").unwrap(),
            route: None,
            region: vec![p(1, 4), p(9, 4), p(9, 8), p(1, 8)],
            target_length: Real::from(10),
            tolerance: (Real::one() / Real::from(3)).unwrap(),
            amplitude: Real::one(),
            pitch: Real::one(),
            maximum_cycles: 4,
            side: LengthTuningSide::Right,
        });
    let document = hypercircuit::SemanticDocument::new(circuit, None)
        .unwrap()
        .with_pcb(layout)
        .unwrap();
    let json = document.to_json_pretty().unwrap();
    let decoded = hypercircuit::SemanticDocument::from_json(&json).unwrap();
    assert_eq!(decoded, document);
}

#[cfg(feature = "interchange")]
#[test]
fn phase_tuning_group_round_trips_exactly() {
    let (circuit, mut layout) = differential_fixture();
    install_differential_phase_tuning(&mut layout);
    let document = hypercircuit::SemanticDocument::new(circuit, None)
        .unwrap()
        .with_pcb(layout)
        .unwrap();
    let json = document.to_json_pretty().unwrap();
    let decoded = hypercircuit::SemanticDocument::from_json(&json).unwrap();
    assert_eq!(decoded, document);
}

#[test]
fn nonselected_copper_is_preserved_and_treated_as_a_fixed_obstacle() {
    let (circuit, mut layout) = fixture(true);
    layout.routes.push(PcbRoute {
        id: RouteId::new("fixed-b-barrier").unwrap(),
        net: NetId::new("B").unwrap(),
        layer: TraceLayer(0),
        width: Real::one(),
        segments: vec![PcbRouteSegment::Line(LinePathSegment::new(
            p(5, 1),
            p(5, 9),
        ))],
    });
    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                nets: vec![NetId::new("A").unwrap()],
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete);
    let routed = report.apply_to(&layout).unwrap();
    assert!(
        routed
            .routes
            .iter()
            .any(|route| route.id.as_str() == "fixed-b-barrier")
    );
    assert!(routed.vias.iter().any(|via| via.net.as_str() == "A"));
    assert!(
        routed
            .routes
            .iter()
            .any(|route| route.net.as_str() == "A" && route.layer == TraceLayer(1))
    );
}

#[test]
fn exact_non_circular_pad_envelopes_open_real_channels_and_honor_pad_rotation() {
    let (circuit, mut layout) = fixture(false);
    let half = (Real::one() / Real::from(2)).unwrap();
    let shapes = vec![
        PadShape::Rectangle {
            width: Real::from(6),
            height: Real::one(),
        },
        PadShape::RoundedRectangle {
            width: Real::from(6),
            height: Real::one(),
            corner_radius: half.clone(),
        },
        PadShape::Obround {
            width: Real::from(6),
            height: Real::one(),
        },
        PadShape::Polygon {
            vertices: vec![
                Point2::new(Real::from(-3), -half.clone()),
                Point2::new(Real::from(3), -half.clone()),
                Point2::new(Real::from(3), half.clone()),
                Point2::new(Real::from(-3), half),
            ],
        },
    ];
    let policy = NegotiatedRoutePolicy {
        nets: vec![NetId::new("A").unwrap()],
        ..NegotiatedRoutePolicy::default()
    };
    for shape in shapes {
        layout.land_patterns[0].pads[0].shape = shape.clone();
        layout.land_patterns[0].pads[0].rotation_degrees = Real::zero();
        let report = layout
            .negotiated_autoroute(&circuit, policy.clone())
            .unwrap();
        assert_eq!(
            report.status,
            NegotiatedRouteStatus::Complete,
            "{shape:?}: {report:?}"
        );
        let problem = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
        let quality = report
            .solution
            .as_ref()
            .unwrap()
            .quality_report(&problem.problem);
        let net = quality
            .nets
            .iter()
            .find(|net| net.net.as_str() == "A")
            .unwrap();
        assert_eq!(net.routed_length, Some(Real::from(6)), "{shape:?}");
        assert_eq!(net.excess_length, Some(Real::zero()), "{shape:?}");

        layout.land_patterns[0].pads[0].rotation_degrees = Real::from(90);
        let blocked = layout
            .negotiated_autoroute(&circuit, policy.clone())
            .unwrap();
        assert_eq!(
            blocked.status,
            NegotiatedRouteStatus::IterationLimit,
            "{shape:?}: {blocked:?}"
        );
        assert!(blocked.failures.iter().any(
            |failure| matches!(failure, NegotiatedRouteFailure::NoPath { net, .. } if net.as_str() == "A")
        ));
    }
}

#[test]
fn electrically_unmapped_placed_pads_remain_physical_routing_obstacles() {
    let (circuit, mut layout) = fixture(false);
    let mut unconnected = layout.land_patterns[0].pads[0].clone();
    unconnected.id = PadId::new("NC").unwrap();
    unconnected.center = p(1, 0);
    layout.land_patterns[0].pads.push(unconnected);
    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                nets: vec![NetId::new("A").unwrap()],
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete, "{report:?}");
    let problem = RoutingProblemReport::from_layout(&circuit, &layout).unwrap();
    let quality = report
        .solution
        .as_ref()
        .unwrap()
        .quality_report(&problem.problem);
    let net = quality
        .nets
        .iter()
        .find(|net| net.net.as_str() == "A")
        .unwrap();
    assert_eq!(net.euclidean_mst_lower_bound, Some(Real::from(6)));
    assert!(
        net.routed_length.as_ref().unwrap() > net.euclidean_mst_lower_bound.as_ref().unwrap(),
        "{net:?}"
    );
}

#[test]
fn grid_pitch_must_cover_the_stricter_pairwise_net_clearance() {
    let (circuit, mut layout) = fixture(true);
    layout.rules.net_classes.push(NetClass {
        id: NetClassId::new("wide-clearance").unwrap(),
        parent: None,
        nets: vec![NetId::new("B").unwrap()],
        min_trace_width: Some(Real::one()),
        preferred_trace_width: None,
        min_clearance: Some(Real::one()),
        preferred_via_land_diameter: None,
        preferred_via_drill_diameter: None,
        preferred_via_style: None,
        max_length: None,
        max_via_count: None,
        target_impedance_ohms: None,
        impedance_tolerance_ohms: None,
        requires_reference_plane: false,
    });
    let result = layout.negotiated_autoroute(
        &circuit,
        NegotiatedRoutePolicy {
            via_land_diameter: Real::one(),
            via_drill_diameter: Real::one(),
            ..NegotiatedRoutePolicy::default()
        },
    );
    assert_eq!(result, Err(NegotiatedRouterError::InvalidPolicy));
}

fn differential_fixture() -> (Circuit, PcbLayout) {
    let (circuit, mut layout) = fixture(true);
    for (placement, position) in
        layout
            .placements
            .iter_mut()
            .zip([p(2, 4), p(8, 4), p(2, 6), p(8, 6)])
    {
        placement.position = position;
    }
    layout.rules.differential_pairs.push(DifferentialPair {
        id: DifferentialPairId::new("usb-data").unwrap(),
        positive: NetId::new("A").unwrap(),
        negative: NetId::new("B").unwrap(),
        spacing: Real::one(),
        max_skew: Some(Real::one()),
        target_impedance_ohms: None,
        impedance_tolerance_ohms: None,
        neckdown: None,
    });
    (circuit, layout)
}

#[test]
fn differential_pair_routes_atomically_with_translated_traces_and_vias() {
    let (circuit, mut layout) = differential_fixture();
    layout.keepouts.push(PcbKeepout {
        id: KeepoutId::new("front-pair-barrier").unwrap(),
        boundary: vec![p(4, 0), p(6, 0), p(6, 10), p(4, 10)],
        scope: KeepoutScope::Copper(vec![TraceLayer(0)]),
    });
    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete);
    assert_eq!(
        report.routed_differential_pairs,
        vec![DifferentialPairId::new("usb-data").unwrap()]
    );
    assert_eq!(report.differential_pair_evidence.len(), 1);
    let evidence = &report.differential_pair_evidence[0];
    assert_eq!(evidence.center_spacing, Real::from(2));
    assert_eq!(evidence.skew, Real::zero());
    assert!(evidence.paired_vias > 0);
    let routed = report.apply_to(&layout).unwrap();
    let positive_routes = routed
        .routes
        .iter()
        .filter(|route| route.net.as_str() == "A")
        .collect::<Vec<_>>();
    let negative_routes = routed
        .routes
        .iter()
        .filter(|route| route.net.as_str() == "B")
        .collect::<Vec<_>>();
    assert_eq!(positive_routes.len(), negative_routes.len());
    assert!(!positive_routes.is_empty());
    for (positive, negative) in positive_routes.iter().zip(&negative_routes) {
        assert_eq!(positive.layer, negative.layer);
        assert_eq!(positive.segments.len(), 1);
        assert_eq!(negative.segments.len(), 1);
        let positive = &positive.segments[0];
        let negative = &negative.segments[0];
        assert_eq!(positive.start().x, negative.start().x);
        assert_eq!(
            negative.start().y.clone() - positive.start().y.clone(),
            Real::from(2)
        );
        assert_eq!(positive.end().x, negative.end().x);
        assert_eq!(
            negative.end().y.clone() - positive.end().y.clone(),
            Real::from(2)
        );
    }
    let positive_vias = routed
        .vias
        .iter()
        .filter(|via| via.net.as_str() == "A")
        .collect::<Vec<_>>();
    let negative_vias = routed
        .vias
        .iter()
        .filter(|via| via.net.as_str() == "B")
        .collect::<Vec<_>>();
    assert_eq!(positive_vias.len(), negative_vias.len());
    assert!(!positive_vias.is_empty());
    for (positive, negative) in positive_vias.iter().zip(&negative_vias) {
        assert_eq!(positive.start_layer, negative.start_layer);
        assert_eq!(positive.end_layer, negative.end_layer);
        assert_eq!(positive.center.x, negative.center.x);
        assert_eq!(
            negative.center.y.clone() - positive.center.y.clone(),
            Real::from(2)
        );
    }

    #[cfg(feature = "geometry")]
    {
        let materialized = routed
            .materialize(&circuit, hypercircuit::MaterializationOptions::default())
            .unwrap();
        for net in ["A", "B"] {
            assert!(materialized.copper_features.iter().any(|feature| {
                feature.kind == hypercircuit::CopperFeatureKind::Route
                    && feature
                        .net
                        .as_ref()
                        .is_some_and(|value| value.as_str() == net)
            }));
            assert!(materialized.copper_features.iter().any(|feature| {
                feature.kind == hypercircuit::CopperFeatureKind::Via
                    && feature
                        .net
                        .as_ref()
                        .is_some_and(|value| value.as_str() == net)
            }));
        }

        #[cfg(feature = "drc")]
        {
            let handoff =
                hypercircuit::HyperDrcHandoff::from_materialization(&routed, &materialized);
            assert!(handoff.net_classes.iter().any(|class| {
                class.differential_pair.as_deref() == Some("usb-data")
                    && class.differential_role
                        == Some(hyperdrc::constraint_policy::DifferentialRole::Positive)
            }));
            assert!(handoff.net_classes.iter().any(|class| {
                class.differential_pair.as_deref() == Some("usb-data")
                    && class.differential_role
                        == Some(hyperdrc::constraint_policy::DifferentialRole::Negative)
            }));
        }
    }
}

#[test]
fn differential_pair_neckdown_fans_out_tight_terminals_before_joint_routing() {
    let (circuit, mut layout) = differential_fixture();
    let pair = &mut layout.rules.differential_pairs[0];
    pair.spacing = Real::from(3);
    pair.neckdown = Some(DifferentialPairNeckdown {
        trace_width: (Real::one() / Real::from(2)).unwrap(),
        spacing: (Real::from(3) / Real::from(2)).unwrap(),
        maximum_transition_length: Real::one(),
    });
    let exact_point = |x_numerator, x_denominator, y_numerator, y_denominator| {
        Point2::new(
            (Real::from(x_numerator) / Real::from(x_denominator)).unwrap(),
            (Real::from(y_numerator) / Real::from(y_denominator)).unwrap(),
        )
    };
    layout.keepouts.push(PcbKeepout {
        id: KeepoutId::new("neckdown-width-channel").unwrap(),
        boundary: vec![
            exact_point(23, 10, 71, 20),
            exact_point(24, 10, 71, 20),
            exact_point(24, 10, 15, 4),
            exact_point(23, 10, 15, 4),
        ],
        scope: KeepoutScope::Copper(vec![TraceLayer(0), TraceLayer(1)]),
    });
    let mut too_wide = layout.clone();
    let too_wide_neckdown = too_wide.rules.differential_pairs[0]
        .neckdown
        .as_mut()
        .unwrap();
    too_wide_neckdown.trace_width = Real::one();
    too_wide_neckdown.spacing = Real::one();
    let too_wide_report = too_wide
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                grid_mode: NegotiatedGridMode::FeatureAligned {
                    coarse_pitch_multiplier: 4,
                    feature_halo_steps: 0,
                },
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(
        too_wide_report.status,
        NegotiatedRouteStatus::IterationLimit
    );
    assert!(too_wide_report.failures.iter().any(|failure| {
        matches!(
            failure,
            NegotiatedRouteFailure::DifferentialPairNoPath(pair)
                if pair.as_str() == "usb-data"
        )
    }));
    let mut too_broad = layout.clone();
    let too_broad_neckdown = too_broad.rules.differential_pairs[0]
        .neckdown
        .as_mut()
        .unwrap();
    too_broad_neckdown.trace_width = Real::from(3);
    too_broad_neckdown.spacing = Real::from(2);
    assert_eq!(
        too_broad.negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                grid_mode: NegotiatedGridMode::FeatureAligned {
                    coarse_pitch_multiplier: 4,
                    feature_halo_steps: 0,
                },
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        ),
        Err(NegotiatedRouterError::UnsupportedDifferentialPair(
            DifferentialPairId::new("usb-data").unwrap()
        ))
    );
    let mut too_short = layout.clone();
    too_short.rules.differential_pairs[0]
        .neckdown
        .as_mut()
        .unwrap()
        .maximum_transition_length = (Real::one() / Real::from(2)).unwrap();
    assert_eq!(
        too_short.negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                grid_mode: NegotiatedGridMode::FeatureAligned {
                    coarse_pitch_multiplier: 4,
                    feature_halo_steps: 0,
                },
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        ),
        Err(NegotiatedRouterError::UnsupportedDifferentialPair(
            DifferentialPairId::new("usb-data").unwrap()
        ))
    );
    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                grid_mode: NegotiatedGridMode::FeatureAligned {
                    coarse_pitch_multiplier: 4,
                    feature_halo_steps: 0,
                },
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete, "{report:?}");
    let evidence = &report.differential_pair_evidence[0];
    assert_eq!(evidence.center_spacing, Real::from(4));
    assert_eq!(evidence.skew, Real::zero());
    let neckdown = evidence.neckdown.as_ref().unwrap();
    assert_eq!(neckdown.trace_width, (Real::one() / Real::from(2)).unwrap());
    assert_eq!(neckdown.spacing, (Real::from(3) / Real::from(2)).unwrap());
    assert_eq!(neckdown.transition_length, Real::one());
    assert_eq!(neckdown.source_planar_edges, 4);
    assert_eq!(neckdown.target_planar_edges, 4);

    let routed = report.apply_to(&layout).unwrap();
    for (net, terminal_y, expanded_y) in [("A", 4, 3), ("B", 6, 7)] {
        assert!(
            routed
                .routes
                .iter()
                .filter(|route| route.net.as_str() == net)
                .any(|route| route.segments.iter().any(|segment| {
                    route.width == (Real::one() / Real::from(2)).unwrap()
                        && segment.start().x == Real::from(2)
                        && segment.end().x == Real::from(2)
                        && [segment.start().y.clone(), segment.end().y.clone()]
                            .contains(&Real::from(terminal_y))
                        && [segment.start().y.clone(), segment.end().y.clone()]
                            .contains(&Real::from(expanded_y))
                }))
        );
    }
}

#[test]
fn feature_aligned_grid_preserves_exact_differential_pair_translation() {
    let (circuit, mut layout) = differential_fixture();
    let start = (Real::from(3) / Real::from(2)).unwrap();
    let end = (Real::from(17) / Real::from(2)).unwrap();
    layout.placements[0].position.x = start.clone();
    layout.placements[1].position.x = end.clone();
    layout.placements[2].position.x = start;
    layout.placements[3].position.x = end;
    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                grid_mode: NegotiatedGridMode::FeatureAligned {
                    coarse_pitch_multiplier: 4,
                    feature_halo_steps: 1,
                },
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    assert_eq!(report.status, NegotiatedRouteStatus::Complete, "{report:?}");
    assert_eq!(report.differential_pair_evidence.len(), 1);
    assert_eq!(
        report.differential_pair_evidence[0].center_spacing,
        Real::from(2)
    );
    assert_eq!(report.differential_pair_evidence[0].skew, Real::zero());
    let routed = report.apply_to(&layout).unwrap();
    let positive = routed
        .routes
        .iter()
        .filter(|route| route.net.as_str() == "A")
        .collect::<Vec<_>>();
    let negative = routed
        .routes
        .iter()
        .filter(|route| route.net.as_str() == "B")
        .collect::<Vec<_>>();
    assert_eq!(positive.len(), negative.len());
    for (positive, negative) in positive.iter().zip(negative) {
        assert_eq!(positive.segments.len(), negative.segments.len());
        for (positive, negative) in positive.segments.iter().zip(&negative.segments) {
            assert_eq!(positive.start().x, negative.start().x);
            assert_eq!(positive.end().x, negative.end().x);
            assert_eq!(
                negative.start().y.clone() - positive.start().y.clone(),
                Real::from(2)
            );
            assert_eq!(
                negative.end().y.clone() - positive.end().y.clone(),
                Real::from(2)
            );
        }
    }
}

#[test]
fn incompatible_differential_pair_terminal_translation_is_refused() {
    let (circuit, mut layout) = differential_fixture();
    layout.placements[3].position = p(8, 7);
    let result = layout.negotiated_autoroute(
        &circuit,
        NegotiatedRoutePolicy {
            via_land_diameter: Real::one(),
            via_drill_diameter: Real::one(),
            ..NegotiatedRoutePolicy::default()
        },
    );
    assert_eq!(
        result,
        Err(NegotiatedRouterError::UnsupportedDifferentialPair(
            DifferentialPairId::new("usb-data").unwrap()
        ))
    );
}

#[cfg(feature = "geometry")]
#[test]
fn accepted_autoroute_materializes_with_source_identity() {
    let (circuit, layout) = fixture(true);
    let report = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                maximum_iterations: 24,
                present_congestion_penalty: 0,
                history_increment: 12,
                via_penalty: 4,
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    let routed = report.apply_to(&layout).unwrap();
    let materialized = routed
        .materialize(&circuit, hypercircuit::MaterializationOptions::default())
        .unwrap();
    assert!(materialized.copper_features.iter().any(|feature| {
        feature.kind == hypercircuit::CopperFeatureKind::Route
            && feature.source.starts_with("route:negotiated-")
    }));
    assert!(materialized.copper_features.iter().any(|feature| {
        feature.kind == hypercircuit::CopperFeatureKind::Via
            && feature.source.starts_with("via:negotiated-")
    }));

    #[cfg(feature = "drc")]
    {
        let handoff = hypercircuit::HyperDrcHandoff::from_materialization(&routed, &materialized);
        assert!(handoff.board.copper.iter().any(|feature| {
            feature.kind == hyperdrc::kicad::CopperKind::Segment
                && feature
                    .sketch
                    .metadata()
                    .as_ref()
                    .is_some_and(|metadata| metadata.name.starts_with("route:negotiated-"))
        }));
        assert!(handoff.board.copper.iter().any(|feature| {
            feature.kind == hyperdrc::kicad::CopperKind::Via
                && feature
                    .sketch
                    .metadata()
                    .as_ref()
                    .is_some_and(|metadata| metadata.name.starts_with("via:negotiated-"))
        }));
    }
}
