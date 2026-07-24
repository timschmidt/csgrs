#![cfg(feature = "layout")]

use hypercircuit::{
    AcExcitation, BoardOutline, ComponentId, Design, NetId, PcbStackup, Phasor, Real, parts,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut design = Design::new(
        "rc-low-pass",
        BoardOutline::rectangle(Real::from(20), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )?;
    let input = design.signal("IN")?;
    let output = design.signal("OUT")?;
    let ground = design.ground("GND")?;
    let source = design.add(parts::voltage_source("V1", Real::zero()))?;
    let resistor = design.add(parts::resistor("R1", Real::from(1_000)))?;
    let capacitor = design.add(parts::capacitor(
        "C1",
        (Real::one() / Real::from(1_000_000)).expect("constant denominator is nonzero"),
    ))?;

    design.connect(&input, [source.pin("pos")?, resistor.pin("1")?])?;
    design.connect(&output, [resistor.pin("2")?, capacitor.pin("1")?])?;
    design.connect(&ground, [source.pin("neg")?, capacitor.pin("2")?])?;

    let checked = design.finish()?;
    let excitation = AcExcitation::new(
        ComponentId::new(source.id().as_str())?,
        Phasor::real(Real::one()),
    );
    let sweep = checked.circuit.ac_sweep(
        [Real::from(100), Real::from(1_000), Real::from(10_000)],
        &[excitation],
    )?;
    let output = NetId::new("OUT")?;
    for point in sweep.points {
        let voltage = point
            .net_voltage(&output)
            .expect("validated non-ground output net has an MNA unknown");
        println!(
            "omega={} rad/s: OUT=({}, {}j), |OUT|^2={}",
            point.angular_frequency,
            voltage.real,
            voltage.imaginary,
            voltage.magnitude_squared()
        );
    }
    Ok(())
}
