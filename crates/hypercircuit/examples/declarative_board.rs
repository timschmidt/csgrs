use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, Circuit, CircuitId, Net, NetId, PcbDesignRules, PcbLayout,
    PcbRoute, PcbStackup, Real, RouteId, RoutingNetAliases, StackupLayer, StackupLayerKind,
    TransientPolicy,
};
use hyperlattice::Point2;
use hyperpath::{LinePathSegment, TraceLayer};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let signal = NetId::new("SIGNAL")?;
    let circuit = Circuit::new(
        CircuitId::new("declarative-board")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    });

    let front = TraceLayer(0);
    let layout = PcbLayout {
        id: BoardId::new("main")?,
        outline: BoardOutline {
            exterior: vec![point(0, 0), point(40, 0), point(40, 25), point(0, 25)].into(),
            cutouts: Vec::new(),
        },
        stackup: PcbStackup {
            layers: vec![StackupLayer {
                name: "F.Cu".into(),
                kind: StackupLayerKind::Conductor(front),
                thickness: Real::from(1),
                material: Some("hyperphysics:copper".into()),
            }],
        },
        land_patterns: Vec::new(),
        placements: Vec::new(),
        placement_constraints: Vec::new(),
        routes: vec![PcbRoute {
            id: RouteId::new("signal")?,
            net: signal.clone(),
            layer: front,
            width: Real::from(1),
            segments: vec![LinePathSegment::new(point(5, 5), point(30, 20)).into()],
        }],
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };

    assert!(circuit.validate().is_valid());
    assert!(layout.validate(&circuit).is_valid());
    let aliases = RoutingNetAliases::from_circuit(&circuit)?;
    let route_net = aliases.get(&signal).ok_or("route net has no alias")?;
    let certified_carriers = layout.routes[0].to_hyperpath(route_net)?;
    println!(
        "{} retained route segment(s)",
        certified_carriers.traces().len() + certified_carriers.arcs().len()
    );
    Ok(())
}
