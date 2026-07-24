use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, BoardSide, Circuit, CircuitId, CircuitInstance,
    CircuitInstanceId, ComponentId, DeviceModel, DeviceModelId, DeviceModelKind, DevicePin,
    LandPattern, LandPatternId, LandPatternPad, NegotiatedGridMode, NegotiatedPlanarTopology,
    NegotiatedRoutePolicy, Net, NetId, PadId, PadPinMap, PadShape, PcbDesignRules, PcbLayout,
    PcbPlacement, PcbStackup, PinBinding, PinElectricalKind, PinRef, Plating, Real, StackupLayer,
    StackupLayerKind, TransientPolicy,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;

pub const TSCIRCUIT_DATASET_SOURCE: &str = "https://github.com/tscircuit/autorouting";
pub const TSCIRCUIT_ROUTER_SOURCE: &str = "https://github.com/tscircuit/tscircuit-autorouter";

pub struct RoutingCorpusCase {
    pub name: &'static str,
    pub category: &'static str,
    pub circuit: Circuit,
    pub layout: PcbLayout,
    pub policy: NegotiatedRoutePolicy,
    pub expected_nets: usize,
    pub maximum_expanded_states: usize,
}

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn terminal_case(
    name: &'static str,
    category: &'static str,
    board_max: Point2,
    pairs: &[(Point2, Point2)],
    conductor_layers: usize,
    policy: NegotiatedRoutePolicy,
    maximum_expanded_states: usize,
) -> RoutingCorpusCase {
    let model_id = DeviceModelId::new(format!("{name}-terminal")).unwrap();
    let pin = PinRef::new("1").unwrap();
    let footprint = LandPatternId::new(format!("{name}-pad")).unwrap();
    let front = TraceLayer(0);
    let mut circuit = Circuit::new(
        CircuitId::new(name).unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_device_model(DeviceModel {
        id: model_id.clone(),
        kind: DeviceModelKind::Custom("routing-corpus-terminal".into()),
        pins: vec![DevicePin {
            pin: pin.clone(),
            kind: PinElectricalKind::Passive,
            optional: false,
        }],
        parameters: Vec::new(),
    });
    let mut placements = Vec::with_capacity(pairs.len() * 2);
    for (index, (source, sink)) in pairs.iter().enumerate() {
        let net = NetId::new(format!("N{index:02}")).unwrap();
        circuit = circuit.with_net(Net {
            id: net.clone(),
            is_ground: false,
        });
        for (suffix, position) in [("A", source), ("B", sink)] {
            let instance = CircuitInstanceId::new(format!("N{index:02}{suffix}")).unwrap();
            circuit = circuit.with_instance(CircuitInstance {
                id: instance.clone(),
                component: ComponentId::new(format!("N{index:02}{suffix}")).unwrap(),
                part: None,
                model: model_id.clone(),
                pins: vec![PinBinding {
                    pin: pin.clone(),
                    net: net.clone(),
                }],
                parameters: Vec::new(),
            });
            placements.push(PcbPlacement {
                instance,
                land_pattern: footprint.clone(),
                position: position.clone(),
                rotation_degrees: Real::zero(),
                side: BoardSide::Front,
            });
        }
    }
    let layers = (0..conductor_layers)
        .map(|index| StackupLayer {
            name: if index == 0 {
                "F.Cu".into()
            } else {
                format!("B{index}.Cu")
            },
            kind: StackupLayerKind::Conductor(TraceLayer(
                u16::try_from(index).expect("routing corpus layer count fits u16"),
            )),
            thickness: Real::one(),
            material: Some("copper".into()),
        })
        .collect();
    let layout = PcbLayout {
        id: BoardId::new(name).unwrap(),
        outline: BoardOutline {
            exterior: vec![
                point(0, 0),
                Point2::new(board_max.x.clone(), Real::zero()),
                board_max.clone(),
                Point2::new(Real::zero(), board_max.y.clone()),
            ]
            .into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup { layers },
        land_patterns: vec![LandPattern {
            id: footprint,
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
        placements,
        placement_constraints: Vec::new(),
        routes: Vec::new(),
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };
    RoutingCorpusCase {
        name,
        category,
        circuit,
        layout,
        policy,
        expected_nets: pairs.len(),
        maximum_expanded_states,
    }
}

pub fn cases() -> Vec<RoutingCorpusCase> {
    let parallel_pairs = (0..12)
        .map(|index| {
            let y = 3 + index * 2;
            (point(3, y), point(45, y))
        })
        .collect::<Vec<_>>();
    let mut crossing_pairs = Vec::new();
    for cluster in 0..2 {
        let x = 1 + cluster * 10;
        crossing_pairs.push((point(x + 1, 5), point(x + 7, 5)));
        crossing_pairs.push((point(x + 4, 2), point(x + 4, 8)));
    }
    let any_angle_pairs = (0..3)
        .map(|index| {
            let source_y = 3 + index * 3;
            (point(3, source_y), point(17, source_y + 1))
        })
        .collect::<Vec<_>>();
    vec![
        terminal_case(
            "parallel-bus-12",
            "single-layer multi-trace",
            point(48, 28),
            &parallel_pairs,
            1,
            NegotiatedRoutePolicy {
                maximum_iterations: 8,
                maximum_expansions_per_connection: 20_000,
                ..NegotiatedRoutePolicy::default()
            },
            4_000,
        ),
        terminal_case(
            "two-layer-crossings-4",
            "multilayer multi-trace",
            point(22, 10),
            &crossing_pairs,
            2,
            NegotiatedRoutePolicy {
                maximum_iterations: 16,
                maximum_expansions_per_connection: 20_000,
                ..NegotiatedRoutePolicy::default()
            },
            20_000,
        ),
        terminal_case(
            "any-angle-fanout-3",
            "bounded any-angle multi-trace",
            point(20, 12),
            &any_angle_pairs,
            1,
            NegotiatedRoutePolicy {
                planar_topology: NegotiatedPlanarTopology::AnyAngle {
                    maximum_neighbors_per_node: 128,
                },
                grid_mode: NegotiatedGridMode::FeatureAligned {
                    coarse_pitch_multiplier: 4,
                    feature_halo_steps: 0,
                },
                maximum_iterations: 8,
                maximum_expansions_per_connection: 20_000,
                ..NegotiatedRoutePolicy::default()
            },
            5_000,
        ),
    ]
}
