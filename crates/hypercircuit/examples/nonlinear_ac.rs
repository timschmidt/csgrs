#![cfg(feature = "layout")]

use std::collections::BTreeMap;

use hypercircuit::{
    AcExcitation, AcNonlinearLinearization, AcOperatingPoint, BoardOutline, ComponentId, Design,
    MnaUnknown, MosfetNewtonPolicy, NetId, PcbStackup, Phasor, Real, parts,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut design = Design::new(
        "common-source-small-signal",
        BoardOutline::rectangle(Real::from(20), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )?;
    let supply = design.signal("VDD")?;
    let gate = design.signal("GATE")?;
    let drain = design.signal("DRAIN")?;
    let ground = design.ground("GND")?;
    let supply_source = design.add(parts::voltage_source("VDD1", Real::from(5)))?;
    let gate_source = design.add(parts::voltage_source("VG1", Real::from(2)))?;
    let load = design.add(parts::resistor("R1", Real::one()))?;
    let transistor = design.add(parts::nmos("M1", Real::one(), Real::one()))?;

    design.connect(&supply, [supply_source.pin("pos")?, load.pin("1")?])?;
    design.connect(&gate, [gate_source.pin("pos")?, transistor.pin("G")?])?;
    design.connect(&drain, [load.pin("2")?, transistor.pin("D")?])?;
    design.connect(
        &ground,
        [
            supply_source.pin("neg")?,
            gate_source.pin("neg")?,
            transistor.pin("S")?,
        ],
    )?;

    let checked = design.finish()?;
    let dc = checked
        .circuit
        .solve_mosfet_dc(&MosfetNewtonPolicy::default(), &BTreeMap::new())?;
    let operating_point = AcOperatingPoint::from_mosfet_newton(&dc)?;
    let sweep = checked.circuit.small_signal_ac_sweep(
        [Real::one(), Real::from(1_000)],
        &[AcExcitation::new(
            ComponentId::new(gate_source.id().as_str())?,
            Phasor::real(Real::one()),
        )],
        &operating_point,
    )?;

    let drain = NetId::new("DRAIN")?;
    for point in &sweep.points {
        let value = point
            .value(&MnaUnknown::NetVoltage(drain.clone()))
            .expect("checked non-ground net has an AC voltage");
        println!(
            "omega={} rad/s: DRAIN=({}, {}j), replay={}",
            point.angular_frequency, value.real, value.imaginary, point.solution.replay.accepted
        );
    }
    if let AcNonlinearLinearization::Mosfet {
        operating_point: evidence,
    } = &sweep.nonlinear_linearizations[0]
    {
        println!(
            "region={:?}, gm={}, gds={}, DC exact replay={}",
            evidence.region,
            evidence.transconductance,
            evidence.output_conductance,
            dc.replay.exact_zero
        );
    }
    Ok(())
}
