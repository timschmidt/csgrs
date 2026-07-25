use hypercircuit::{
    AdapterKind, BundleEndpointId, BundleMemberId, BundlePortBinding, Circuit, CircuitId,
    CircuitLibrary, CircuitPort, Modport, ModportId, ModportMember, Net, NetId, PortDirection,
    PortId, SignalBundle, SignalBundleEndpoint, SignalBundleId, SignalBundleLibrary,
    SubcircuitInstance, SubcircuitInstanceId, TransientPolicy,
};

fn module(id: &str, direction: PortDirection) -> Circuit {
    let signal = NetId::new("signal").unwrap();
    Circuit::new(
        CircuitId::new(id).unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    })
    .with_port(CircuitPort {
        id: PortId::new("signal").unwrap(),
        net: signal,
        direction,
        optional: false,
    })
}

fn endpoint(circuit: &str, endpoint: &str, modport: &str) -> SignalBundleEndpoint {
    SignalBundleEndpoint::new(
        BundleEndpointId::new(endpoint).unwrap(),
        CircuitId::new(circuit).unwrap(),
        SignalBundleId::new("Stream").unwrap(),
        ModportId::new(modport).unwrap(),
        vec![BundlePortBinding {
            member: BundleMemberId::new("signal").unwrap(),
            port: PortId::new("signal").unwrap(),
        }],
    )
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let producer = Modport::new(
        ModportId::new("producer").unwrap(),
        vec![ModportMember {
            member: BundleMemberId::new("signal").unwrap(),
            direction: PortDirection::Output,
        }],
    );
    let stream = SignalBundle::new(
        SignalBundleId::new("Stream").unwrap(),
        vec![BundleMemberId::new("signal").unwrap()],
    )
    .with_modport(producer.dual(ModportId::new("consumer").unwrap()))
    .with_modport(producer);

    let child = module("producer-module", PortDirection::Output);
    let root = module("board", PortDirection::Input).with_subcircuit(SubcircuitInstance {
        id: SubcircuitInstanceId::new("producer").unwrap(),
        circuit: child.id.clone(),
        ports: Vec::new(),
        parameter_overrides: Vec::new(),
    });
    let mut circuits = CircuitLibrary {
        root: root.id.clone(),
        circuits: vec![root, child],
    };
    let interfaces = SignalBundleLibrary::new()
        .with_bundle(stream)
        .with_endpoint(endpoint("board", "stream-in", "consumer"))
        .with_endpoint(endpoint("producer-module", "stream-out", "producer"));

    interfaces.bind_subcircuit(
        &mut circuits,
        &CircuitId::new("board").unwrap(),
        &SubcircuitInstanceId::new("producer").unwrap(),
        &BundleEndpointId::new("stream-in").unwrap(),
        &BundleEndpointId::new("stream-out").unwrap(),
    )?;

    assert!(circuits.validate().is_valid());
    let flattened = circuits.flatten()?;
    println!("flattened {} net(s)", flattened.nets.len());
    Ok(())
}
