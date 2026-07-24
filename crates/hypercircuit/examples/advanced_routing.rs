use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, BoardSide, Circuit, CircuitId, CircuitInstance,
    CircuitInstanceId, ComponentId, DeviceModel, DeviceModelId, DeviceModelKind, DevicePin,
    EscapePolicy, EscapePolicyId, LandPattern, LandPatternId, LandPatternPad, LengthTuningPattern,
    LengthTuningPatternId, LengthTuningSide, LengthTuningStatus, NegotiatedRouteHtmlOptions,
    NegotiatedRoutePolicy, NegotiatedRouteSvgOptions, Net, NetId, PadId, PadPinMap, PadShape,
    PcbDesignRules, PcbLayout, PcbPlacement, PcbStackup, PinBinding, PinElectricalKind, PinRef,
    Plating, Real, RouteConstraintRegion, RouteConstraintRegionId, RouteDirection, StackupLayer,
    StackupLayerKind, TransientPolicy,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;
use std::path::Path;

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn main() {
    let net = NetId::new("DATA").unwrap();
    let model = DeviceModelId::new("terminal").unwrap();
    let pin = PinRef::new("1").unwrap();
    let mut circuit = Circuit::new(
        CircuitId::new("advanced-routing").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: net.clone(),
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: model.clone(),
        kind: DeviceModelKind::Resistor,
        pins: vec![DevicePin {
            pin: pin.clone(),
            kind: PinElectricalKind::Passive,
            optional: false,
        }],
        parameters: Vec::new(),
    });
    for id in ["source", "sink"] {
        circuit = circuit.with_instance(CircuitInstance {
            id: CircuitInstanceId::new(id).unwrap(),
            component: ComponentId::new(id).unwrap(),
            part: None,
            model: model.clone(),
            pins: vec![PinBinding {
                pin: pin.clone(),
                net: net.clone(),
            }],
            parameters: Vec::new(),
        });
    }

    let front = TraceLayer(0);
    let back = TraceLayer(1);
    let footprint = LandPatternId::new("terminal-pad").unwrap();
    let tuning = LengthTuningPatternId::new("data-length").unwrap();
    let layout = PcbLayout {
        id: BoardId::new("advanced-routing").unwrap(),
        outline: BoardOutline {
            exterior: vec![point(0, 0), point(10, 0), point(10, 10), point(0, 10)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![
                StackupLayer {
                    name: "F.Cu".into(),
                    kind: StackupLayerKind::Conductor(front),
                    thickness: Real::one(),
                    material: Some("copper".into()),
                },
                StackupLayer {
                    name: "B.Cu".into(),
                    kind: StackupLayerKind::Conductor(back),
                    thickness: Real::one(),
                    material: Some("copper".into()),
                },
            ],
        },
        land_patterns: vec![LandPattern {
            id: footprint.clone(),
            pads: vec![LandPatternPad {
                id: PadId::new("1").unwrap(),
                center: point(0, 0),
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
        placements: vec![
            PcbPlacement {
                instance: CircuitInstanceId::new("source").unwrap(),
                land_pattern: footprint.clone(),
                position: point(2, 5),
                rotation_degrees: Real::zero(),
                side: BoardSide::Front,
            },
            PcbPlacement {
                instance: CircuitInstanceId::new("sink").unwrap(),
                land_pattern: footprint,
                position: point(8, 5),
                rotation_degrees: Real::zero(),
                side: BoardSide::Front,
            },
        ],
        placement_constraints: Vec::new(),
        routes: Vec::new(),
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules {
            route_constraint_regions: vec![RouteConstraintRegion {
                id: RouteConstraintRegionId::new("back-only-channel").unwrap(),
                boundary: vec![point(5, 0), point(7, 0), point(7, 10), point(5, 10)],
                nets: vec![net.clone()],
                allowed_layers: vec![back],
                allowed_directions: vec![RouteDirection::Horizontal],
                allow_vias: false,
            }],
            escape_policies: vec![EscapePolicy {
                id: EscapePolicyId::new("source-fanout").unwrap(),
                instances: vec![CircuitInstanceId::new("source").unwrap()],
                nets: vec![net.clone()],
                max_distance: Real::one(),
                allowed_layers: vec![front],
                allowed_directions: vec![RouteDirection::Horizontal],
                allow_vias: false,
            }],
            length_tuning_patterns: vec![LengthTuningPattern {
                id: tuning.clone(),
                net: net.clone(),
                route: None,
                region: vec![point(3, 4), point(9, 4), point(9, 8), point(3, 8)],
                target_length: Real::from(10),
                tolerance: Real::zero(),
                amplitude: Real::one(),
                pitch: Real::one(),
                maximum_cycles: 2,
                side: LengthTuningSide::Left,
            }],
            ..PcbDesignRules::default()
        },
    };
    assert!(layout.validate(&circuit).is_valid());

    let routed = layout
        .negotiated_autoroute(
            &circuit,
            NegotiatedRoutePolicy {
                nets: vec![net],
                via_land_diameter: Real::one(),
                via_drill_diameter: Real::one(),
                ..NegotiatedRoutePolicy::default()
            },
        )
        .unwrap();
    let constrained_edges = routed.route_constraint_evidence[0].constrained_planar_edges;
    let escaped_edges = routed.escape_policy_evidence[0].constrained_planar_edges;
    let replay_directory = Path::new("target/advanced-routing-replay");
    std::fs::create_dir_all(replay_directory).unwrap();
    for state in &routed.iteration_states {
        let view = routed
            .iteration_svg(
                &layout,
                NegotiatedRouteSvgOptions {
                    iteration: state.iteration,
                    ..NegotiatedRouteSvgOptions::default()
                },
            )
            .unwrap();
        std::fs::write(
            replay_directory.join(format!("iteration-{}.svg", state.iteration)),
            view.svg,
        )
        .unwrap();
    }
    let replay = routed
        .replay_html(
            &layout,
            NegotiatedRouteHtmlOptions {
                title: "Advanced routing replay".into(),
                ..NegotiatedRouteHtmlOptions::default()
            },
        )
        .unwrap();
    std::fs::write(replay_directory.join("index.html"), replay.html).unwrap();
    let replay_passes = routed.iteration_states.len();
    let routed = routed.apply_to(&layout).unwrap();
    let tuned = routed.realize_length_tuning(&tuning);
    assert_eq!(tuned.status, LengthTuningStatus::Applied);
    let final_layout = tuned.apply_to(&routed).unwrap();
    assert!(final_layout.validate(&circuit).is_valid());
    println!(
        "region_edges={constrained_edges}, escape_edges={escaped_edges}, vias={}, length={} -> {}, cycles={}, replay_passes={replay_passes}, replay_dir={}",
        routed.vias.len(),
        tuned.original_length.as_ref().unwrap(),
        tuned.realized_length.as_ref().unwrap(),
        tuned.cycles,
        replay_directory.display(),
    );
}
