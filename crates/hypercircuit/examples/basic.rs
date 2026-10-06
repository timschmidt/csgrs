use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitResult, ComponentId, LinearStamp, Net, NetId, Real,
    TransientPolicy,
};

fn main() -> CircuitResult<()> {
    let output = NetId::new("out")?;
    let circuit = Circuit::new(
        CircuitId::new("conductance")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: output.clone(),
        is_ground: false,
    })
    .with_stamp(LinearStamp::Conductance {
        component: ComponentId::new("g1")?,
        part: None,
        pos: Some(output),
        neg: None,
        conductance: Real::from(2),
    });

    let system = circuit.linear_mna_system()?;
    let solution = system.solve_exact()?;
    assert!(solution.replay.accepted);
    assert_eq!(solution.candidate, vec![Real::zero()]);
    Ok(())
}
