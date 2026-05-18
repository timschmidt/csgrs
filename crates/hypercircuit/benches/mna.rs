use std::hint::black_box;
use std::time::Instant;

use hypercircuit::{
    AdapterKind, BranchId, Circuit, CircuitId, ComponentId, LinearMnaSystem, LinearStamp, Net,
    NetId, Real, TransientPolicy,
};

fn main() {
    let node = NetId::new("vout").unwrap();
    let stamps = vec![
        LinearStamp::VoltageSource {
            component: ComponentId::new("v1").unwrap(),
            branch: BranchId::new("iv1").unwrap(),
            pos: Some(node.clone()),
            neg: None,
            voltage: Real::from(5),
        },
        LinearStamp::Conductance {
            component: ComponentId::new("r1").unwrap(),
            part: None,
            pos: Some(node.clone()),
            neg: None,
            conductance: Real::from(2),
        },
        LinearStamp::Companion {
            component: ComponentId::new("c1-companion").unwrap(),
            pos: Some(node.clone()),
            neg: None,
            conductance: Real::from(4),
            history_current: Real::from(6),
        },
    ];
    let iterations = 100_000_u32;
    let started = Instant::now();
    let mut accepted = 0_usize;

    for _ in 0..iterations {
        let system = LinearMnaSystem::from_stamps(vec![node.clone()], black_box(&stamps)).unwrap();
        let replay = system
            .replay_candidate(&[Real::from(5), Real::from(-36)])
            .unwrap();
        if replay.accepted {
            accepted += 1;
        }
    }

    let elapsed = started.elapsed();
    println!(
        "mna_stamp_and_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), accepted={accepted}",
        elapsed / iterations
    );

    let circuit = Circuit::new(
        CircuitId::new("bench").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: node.clone(),
        is_ground: false,
    })
    .with_stamp(stamps[0].clone())
    .with_stamp(stamps[1].clone());
    let started = Instant::now();
    let mut carrier_checksum = 0_usize;
    for _ in 0..iterations {
        let system = black_box(&circuit).linear_mna_system().unwrap();
        carrier_checksum ^= system.unknowns.len();
        carrier_checksum ^= system.matrix.len();
    }
    let elapsed = started.elapsed();
    println!(
        "circuit_carrier_lowering: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={carrier_checksum}",
        elapsed / iterations
    );

    let started = Instant::now();
    let mut coupling_checksum = 0_usize;
    for _ in 0..iterations {
        let report = hypercircuit::ElectrothermalRcReport::replay(
            ComponentId::new("r1").unwrap(),
            Real::from(10),
            Real::from(2),
            Real::from(305),
            Real::from(300),
            Real::from(3),
            "hyperphysics:thermal/r1",
        )
        .unwrap();
        coupling_checksum ^= format!("{:?}", report.joule_heating).len();
        coupling_checksum ^= report.residual_block.residuals.len();
    }
    let elapsed = started.elapsed();
    println!(
        "electrothermal_rc_replay: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={coupling_checksum}",
        elapsed / iterations
    );

    let started = Instant::now();
    let mut nonlinear_checksum = 0_usize;
    for _ in 0..iterations {
        let report = hypercircuit::NonlinearDeviceReport::switch(
            ComponentId::new("sw1").unwrap(),
            hypercircuit::EventPolicy::ExactReplayRequired,
            hypercircuit::SwitchState::Proposed,
        );
        nonlinear_checksum ^= format!("{:?}", report.kind).len();
        nonlinear_checksum ^= report.event_policy.is_some() as usize;
    }
    let elapsed = started.elapsed();
    println!(
        "nonlinear_event_report: {iterations} iterations in {elapsed:?} ({:?}/iter), checksum={nonlinear_checksum}",
        elapsed / iterations
    );
}
