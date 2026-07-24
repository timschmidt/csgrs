#![cfg(feature = "layout")]

use hypercircuit::{
    AdapterKind, BoardContour, BoardId, BoardOutline, BoardSide, Circuit, CircuitId,
    CircuitInstance, CircuitInstanceId, ComponentId, DeviceModel, DeviceModelId, DeviceModelKind,
    DevicePin, EscapePolicy, EscapePolicyId, KeepoutId, KeepoutScope, LandPattern, LandPatternBody,
    LandPatternId, LandPatternPad, NegotiatedRoutePolicy, NegotiatedRouteStatus, Net, NetId, PadId,
    PadPinMap, PadShape, PcbDesignRules, PcbKeepout, PcbLayout, PcbPlacement, PcbRoute,
    PcbRouteSegment, PcbStackup, PinBinding, PinElectricalKind, PinRef, PlacementConstraint,
    PlacementConstraintId, PlacementConstraintKind, PlacementPinAccessIssue,
    PlacementPinAccessPolicy, PlacementPinAccessStatus, PlacementSolveIssue, PlacementSolvePolicy,
    Plating, Real, RouteDirection, RouteId, StackupLayer, StackupLayerKind, TransientPolicy,
};
use hyperlattice::Point2;
use hyperpath::{ArcDirection, ExplicitCircularArc, LinePathSegment, TraceLayer};

fn p(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn fixture() -> (Circuit, PcbLayout) {
    let model = DeviceModelId::new("mechanical-package").unwrap();
    let first = CircuitInstanceId::new("U1").unwrap();
    let second = CircuitInstanceId::new("U2").unwrap();
    let third = CircuitInstanceId::new("U3").unwrap();
    let signal = NetId::new("SIGNAL").unwrap();
    let unrelated = NetId::new("UNRELATED").unwrap();
    let pin = PinRef::new("1").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("placement").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: unrelated.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: model.clone(),
        kind: DeviceModelKind::Custom("mechanical".into()),
        pins: vec![DevicePin {
            pin: pin.clone(),
            kind: PinElectricalKind::Passive,
            optional: false,
        }],
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: first.clone(),
        component: ComponentId::new("U1").unwrap(),
        part: None,
        model: model.clone(),
        pins: vec![PinBinding {
            pin: pin.clone(),
            net: unrelated,
        }],
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: second.clone(),
        component: ComponentId::new("U2").unwrap(),
        part: None,
        model: model.clone(),
        pins: vec![PinBinding {
            pin: pin.clone(),
            net: signal.clone(),
        }],
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: third.clone(),
        component: ComponentId::new("U3").unwrap(),
        part: None,
        model,
        pins: vec![PinBinding { pin, net: signal }],
        parameters: Vec::new(),
    });
    let pattern = LandPatternId::new("package").unwrap();
    let placement = |instance, position| PcbPlacement {
        instance,
        land_pattern: pattern.clone(),
        position,
        rotation_degrees: Real::zero(),
        side: BoardSide::Front,
    };
    let layout = PcbLayout {
        id: BoardId::new("placement").unwrap(),
        outline: BoardOutline {
            exterior: vec![p(0, 0), p(20, 0), p(20, 20), p(0, 20)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![StackupLayer {
                name: "F.Cu".into(),
                kind: StackupLayerKind::Conductor(TraceLayer(0)),
                thickness: Real::one(),
                material: None,
            }],
        },
        land_patterns: vec![LandPattern {
            id: pattern.clone(),
            pads: vec![LandPatternPad {
                id: PadId::new("1").unwrap(),
                center: p(0, 0),
                rotation_degrees: Real::zero(),
                copper_layers: vec![TraceLayer(0)],
                shape: PadShape::Circle {
                    diameter: Real::one(),
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
            graphics: Vec::new(),
            body: Some(LandPatternBody {
                outline: vec![p(-1, -1), p(1, -1), p(1, 1), p(-1, 1)],
                height: Real::one(),
                standoff: Real::zero(),
            }),
            models: Vec::new(),
        }],
        placements: vec![
            placement(first, p(5, 5)),
            placement(second, p(5, 5)),
            placement(third, p(15, 5)),
        ],
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
fn deterministic_search_moves_only_the_colliding_unconstrained_package() {
    let (circuit, layout) = fixture();
    let report = layout.solve_placement(&circuit, &PlacementSolvePolicy::default());

    assert!(report.is_solved(), "{:?}", report.issues);
    assert_eq!(report.moves.len(), 1, "{:#?}", report.moves);
    assert_eq!(report.moves[0].instance.as_str(), "U2");
    assert_eq!(report.placements[0].position, p(5, 5));
    assert_eq!(report.placements[1].position, p(13, 5));
    assert!(
        report.moves[0].accepted_score.connectivity_length
            < report.moves[0].authored_score.connectivity_length
    );

    let applied = report.apply_to(&layout);
    assert_eq!(applied.placements, report.placements);
}

#[test]
fn placement_search_uses_exact_arc_board_envelopes() {
    let (circuit, mut layout) = fixture();
    layout.outline.exterior = BoardContour::from_segments(vec![
        LinePathSegment::new(p(0, 0), p(20, 0)).into(),
        LinePathSegment::new(p(20, 0), p(20, 20)).into(),
        ExplicitCircularArc::new(
            p(10, 20),
            Real::from(10),
            p(20, 20),
            p(0, 20),
            ArcDirection::Ccw,
        )
        .unwrap()
        .into(),
        LinePathSegment::new(p(0, 20), p(0, 0)).into(),
    ]);
    let report = layout.solve_placement(&circuit, &PlacementSolvePolicy::default());
    assert!(report.is_solved(), "{:?}", report.issues);
    assert_eq!(report.moves.len(), 1);
}

#[test]
fn fixed_collisions_are_reported_without_breaking_constraints() {
    let (circuit, mut layout) = fixture();
    layout.placement_constraints = layout
        .placements
        .iter()
        .enumerate()
        .map(|(index, placement)| PlacementConstraint {
            id: PlacementConstraintId::new(format!("fixed-{index}")).unwrap(),
            kind: PlacementConstraintKind::Fixed {
                instance: placement.instance.clone(),
                position: p(5, 5),
            },
        })
        .collect();

    let report = layout.solve_placement(&circuit, &PlacementSolvePolicy::default());
    assert!(report.moves.is_empty());
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        PlacementSolveIssue::LockedCollision { first, second }
            if first.as_str() == "U1" && second.as_str() == "U2"
    )));
    assert_eq!(report.placements[0].position, p(5, 5));
    assert_eq!(report.placements[1].position, p(5, 5));
}

#[test]
fn regional_search_keeps_the_package_envelope_out_of_board_cutouts() {
    let (circuit, mut layout) = fixture();
    layout.outline.cutouts = vec![vec![p(10, 3), p(13, 3), p(13, 7), p(10, 7)].into()];
    layout.placement_constraints = vec![PlacementConstraint {
        id: PlacementConstraintId::new("u2-region").unwrap(),
        kind: PlacementConstraintKind::Within {
            instance: CircuitInstanceId::new("U2").unwrap(),
            min: p(9, 1),
            max: p(16, 9),
        },
    }];

    let report = layout.solve_placement(&circuit, &PlacementSolvePolicy::default());
    assert!(report.is_solved(), "{:?}", report.issues);
    let center = &report.placements[1].position;
    assert!(p(9, 1).x <= center.x && center.x <= p(16, 9).x);
    assert!(p(9, 1).y <= center.y && center.y <= p(16, 9).y);

    // The retained body spans one unit around its origin. It must be separated
    // from the cutout rectangle, including its boundary.
    assert!(
        center.x.clone() + Real::one() < Real::from(10)
            || center.x.clone() - Real::one() > Real::from(13)
            || center.y.clone() + Real::one() < Real::from(3)
            || center.y.clone() - Real::one() > Real::from(7),
        "accepted envelope overlaps the cutout at {center:?}"
    );
}

#[test]
fn optional_optimization_improves_a_legal_authored_connectivity_cost() {
    let (circuit, mut layout) = fixture();
    layout.placements[1].position = p(9, 5);
    let preserved = layout.solve_placement(&circuit, &PlacementSolvePolicy::default());
    assert!(preserved.is_solved());
    assert_eq!(preserved.placements[1].position, p(9, 5));

    let policy = PlacementSolvePolicy {
        optimize_legal_authored_positions: true,
        ..PlacementSolvePolicy::default()
    };
    let optimized = layout.solve_placement(&circuit, &policy);
    assert!(optimized.is_solved(), "{:?}", optimized.issues);
    let movement = optimized
        .moves
        .iter()
        .find(|movement| movement.instance.as_str() == "U2")
        .unwrap();
    assert_eq!(movement.to, p(13, 5));
    assert!(
        movement.accepted_score.connectivity_length < movement.authored_score.connectivity_length
    );
}

#[test]
fn routability_aware_optimization_reduces_exact_net_box_congestion() {
    let (mut circuit, mut layout) = fixture();
    let unrelated = NetId::new("UNRELATED").unwrap();
    circuit.instances[1].pins[0].net = unrelated;

    let mut fourth = circuit.instances[2].clone();
    fourth.id = CircuitInstanceId::new("U4").unwrap();
    fourth.component = ComponentId::new("U4").unwrap();
    circuit.instances.push(fourth);

    layout.placements[0].position = p(4, 4);
    layout.placements[1].position = p(12, 12);
    layout.placements[2].position = p(8, 8);
    let mut fourth_placement = layout.placements[2].clone();
    fourth_placement.instance = CircuitInstanceId::new("U4").unwrap();
    fourth_placement.position = p(16, 16);
    layout.placements.push(fourth_placement);
    layout.placement_constraints = [0_usize, 2, 3]
        .into_iter()
        .map(|index| PlacementConstraint {
            id: PlacementConstraintId::new(format!("locked-{index}")).unwrap(),
            kind: PlacementConstraintKind::Fixed {
                instance: layout.placements[index].instance.clone(),
                position: layout.placements[index].position.clone(),
            },
        })
        .collect();

    let policy = PlacementSolvePolicy {
        optimize_legal_authored_positions: true,
        minimize_routing_congestion: true,
        density_radius: Some(Real::from(20)),
        ..PlacementSolvePolicy::default()
    };
    let report = layout.solve_placement(&circuit, &policy);
    assert!(report.is_solved(), "{:?}", report.issues);
    let movement = report
        .moves
        .iter()
        .find(|movement| movement.instance.as_str() == "U2")
        .unwrap();
    assert!(
        movement.accepted_score.routing_congestion < movement.authored_score.routing_congestion,
        "{movement:#?}"
    );
    assert!(
        movement.accepted_score.density_pressure < movement.authored_score.density_pressure,
        "{movement:#?}"
    );
    let solved = report.apply_to(&layout);
    let routed = solved
        .negotiated_autoroute(&circuit, NegotiatedRoutePolicy::default())
        .unwrap();
    assert_eq!(routed.status, NegotiatedRouteStatus::Complete);
    assert!(routed.solution.is_some());

    let invalid = PlacementSolvePolicy {
        density_radius: Some(Real::zero()),
        ..PlacementSolvePolicy::default()
    };
    assert!(matches!(
        layout.solve_placement(&circuit, &invalid).issues.as_slice(),
        [PlacementSolveIssue::NonPositiveDensityRadius]
    ));
}

#[test]
fn exact_pin_access_feedback_moves_a_terminal_out_of_a_board_edge_trap() {
    let (circuit, mut layout) = fixture();
    layout.placements.truncate(1);
    layout.placements[0].position = p(2, 2);
    let policy = PlacementSolvePolicy {
        optimize_legal_authored_positions: true,
        pin_access: Some(PlacementPinAccessPolicy {
            probe_distance: Real::from(2),
            minimum_trace_width: Real::one(),
            minimum_clearance: Real::zero(),
        }),
        ..PlacementSolvePolicy::default()
    };

    let report = layout.solve_placement(&circuit, &policy);
    assert!(report.is_solved(), "{:?}", report.issues);
    let movement = report
        .moves
        .iter()
        .find(|movement| movement.instance.as_str() == "U1")
        .unwrap();
    assert_eq!(movement.from, p(2, 2));
    assert_eq!(movement.to, p(3, 3));
    assert_eq!(movement.authored_score.pin_access.blocked_probes, 2);
    assert_eq!(movement.accepted_score.pin_access.blocked_probes, 0);
    assert_eq!(movement.authored_score.pin_access.accessible_probes, 2);
    assert_eq!(movement.accepted_score.pin_access.accessible_probes, 4);

    let evidence = report.pin_access.as_ref().unwrap();
    assert!(evidence.issues.is_empty());
    assert_eq!(evidence.terminals.len(), 1);
    assert_eq!(evidence.terminals[0].instance.as_str(), "U1");
    assert_eq!(evidence.terminals[0].pin.as_str(), "1");
    assert_eq!(evidence.terminals[0].pad.as_str(), "1");
    assert!(
        evidence.terminals[0]
            .probes
            .iter()
            .all(|probe| probe.status == PlacementPinAccessStatus::Accessible)
    );
    assert_eq!(evidence.score(), movement.accepted_score.pin_access);

    let invalid = layout.placement_pin_access(
        &circuit,
        &PlacementPinAccessPolicy {
            probe_distance: Real::zero(),
            minimum_trace_width: Real::one(),
            minimum_clearance: Real::zero(),
        },
    );
    assert_eq!(invalid.issues, vec![PlacementPinAccessIssue::InvalidPolicy]);
    let invalid_solve = PlacementSolvePolicy {
        pin_access: Some(PlacementPinAccessPolicy {
            probe_distance: Real::one(),
            minimum_trace_width: Real::zero(),
            minimum_clearance: Real::zero(),
        }),
        ..PlacementSolvePolicy::default()
    };
    assert!(matches!(
        layout
            .solve_placement(&circuit, &invalid_solve)
            .issues
            .as_slice(),
        [PlacementSolveIssue::NonPositivePinAccessTraceWidth]
    ));
}

#[test]
fn pin_access_audit_observes_escape_policy_keepouts_and_foreign_pads() {
    let (circuit, mut layout) = fixture();
    layout.placements.truncate(2);
    layout.placements[0].position = p(5, 5);
    layout.placements[1].position = p(7, 5);
    layout.keepouts.push(PcbKeepout {
        id: KeepoutId::new("left-escape-blocker").unwrap(),
        boundary: vec![p(3, 4), p(4, 4), p(4, 6), p(3, 6)],
        scope: KeepoutScope::Copper(vec![TraceLayer(0)]),
    });
    layout.rules.escape_policies.push(EscapePolicy {
        id: EscapePolicyId::new("horizontal-only").unwrap(),
        instances: vec![CircuitInstanceId::new("U1").unwrap()],
        nets: Vec::new(),
        max_distance: Real::from(2),
        allowed_layers: vec![TraceLayer(0)],
        allowed_directions: vec![RouteDirection::Horizontal],
        allow_vias: false,
    });

    let report = layout.placement_pin_access(
        &circuit,
        &PlacementPinAccessPolicy {
            probe_distance: Real::from(2),
            minimum_trace_width: Real::one(),
            minimum_clearance: Real::zero(),
        },
    );
    assert!(report.issues.is_empty());
    let terminal = report
        .terminals
        .iter()
        .find(|terminal| terminal.instance.as_str() == "U1")
        .unwrap();
    assert_eq!(terminal.probes.len(), 4);
    assert!(
        terminal
            .probes
            .iter()
            .all(|probe| probe.status == PlacementPinAccessStatus::Blocked)
    );
    let score = report.score();
    assert!(score.evaluation_complete);
    assert_eq!(score.fully_blocked_terminals, 1);
    assert_eq!(score.blocked_probes, 5);
    assert_eq!(score.accessible_probes, 3);
    assert_eq!(score.indeterminate_probes, 0);

    let curve = RouteId::new("fixed-curve").unwrap();
    layout.routes.push(PcbRoute {
        id: curve.clone(),
        net: NetId::new("UNRELATED").unwrap(),
        layer: TraceLayer(0),
        width: Real::one(),
        segments: vec![PcbRouteSegment::CircularArc(
            ExplicitCircularArc::new(
                p(10, 10),
                Real::from(2),
                p(12, 10),
                p(10, 12),
                ArcDirection::Ccw,
            )
            .unwrap(),
        )],
    });
    let access_policy = PlacementPinAccessPolicy {
        probe_distance: Real::from(2),
        minimum_trace_width: Real::one(),
        minimum_clearance: Real::zero(),
    };
    let incomplete = layout.placement_pin_access(&circuit, &access_policy);
    assert_eq!(
        incomplete.issues,
        vec![PlacementPinAccessIssue::UnsupportedFixedRoute(
            curve.clone()
        )]
    );
    let solve = layout.solve_placement(
        &circuit,
        &PlacementSolvePolicy {
            pin_access: Some(access_policy),
            ..PlacementSolvePolicy::default()
        },
    );
    assert!(!solve.is_solved());
    assert!(solve.issues.contains(&PlacementSolveIssue::PinAccess(
        PlacementPinAccessIssue::UnsupportedFixedRoute(curve)
    )));
}

#[test]
fn retained_rotation_and_side_choices_are_searched_and_audited() {
    let (circuit, mut layout) = fixture();
    layout.placements.truncate(1);
    layout.outline.exterior = vec![p(0, 0), p(4, 0), p(4, 10), p(0, 10)].into();
    layout.placements[0].position = p(2, 5);
    layout.land_patterns[0].body.as_mut().unwrap().outline =
        vec![p(-2, -1), p(2, -1), p(2, 1), p(-2, 1)];
    let instance = layout.placements[0].instance.clone();
    layout.placement_constraints = vec![
        PlacementConstraint {
            id: PlacementConstraintId::new("u1-rotation").unwrap(),
            kind: PlacementConstraintKind::AllowedRotations {
                instance: instance.clone(),
                rotations_degrees: vec![Real::zero(), Real::from(90)],
            },
        },
        PlacementConstraint {
            id: PlacementConstraintId::new("u1-side").unwrap(),
            kind: PlacementConstraintKind::AllowedSides {
                instance: instance.clone(),
                sides: vec![BoardSide::Back],
            },
        },
        PlacementConstraint {
            id: PlacementConstraintId::new("u1-fixed").unwrap(),
            kind: PlacementConstraintKind::Fixed {
                instance,
                position: p(2, 5),
            },
        },
    ];

    let report = layout.solve_placement(&circuit, &PlacementSolvePolicy::default());
    assert!(report.is_solved(), "{:?}", report.issues);
    assert_eq!(report.placements[0].position, p(2, 5));
    assert_eq!(report.placements[0].rotation_degrees, Real::from(90));
    assert_eq!(report.placements[0].side, BoardSide::Back);
    assert_eq!(report.moves.len(), 1);
    assert_eq!(report.moves[0].from_rotation_degrees, Real::zero());
    assert_eq!(report.moves[0].to_rotation_degrees, Real::from(90));
    assert_eq!(report.moves[0].from_side, BoardSide::Front);
    assert_eq!(report.moves[0].to_side, BoardSide::Back);
    assert_eq!(report.moves[0].accepted_score.orientation_changes, 2);

    #[cfg(feature = "interchange")]
    {
        let json = serde_json::to_string(&layout.placement_constraints).unwrap();
        assert!(json.contains("AllowedRotations"));
        assert!(json.contains("AllowedSides"));
        let decoded: Vec<PlacementConstraint> = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, layout.placement_constraints);
    }
}

#[cfg(feature = "drc")]
#[test]
fn solved_placement_is_clean_under_hyperdrc_component_readiness() {
    use hypercircuit::{DrcReadinessPolicy, HyperDrcHandoff, MaterializationOptions};

    let (circuit, layout) = fixture();
    let report = layout.solve_placement(&circuit, &PlacementSolvePolicy::default());
    let solved = report.apply_to(&layout);
    let materialized = solved
        .materialize(&circuit, MaterializationOptions::default())
        .unwrap();
    let readiness = HyperDrcHandoff::from_materialization(&solved, &materialized)
        .run_readiness(&DrcReadinessPolicy::default());

    assert!(
        readiness
            .violations
            .iter()
            .all(|violation| violation.check != "authored-component-overlap"),
        "{:?}",
        readiness.violations
    );
}
