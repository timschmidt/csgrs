use hypercircuit::{BoardOutline, Design, PcbStackup, Real, parts};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut design = Design::new(
        "source-diagnostics",
        BoardOutline::rectangle(Real::from(20), Real::from(12)),
        PcbStackup::single_layer(Real::one(), None),
    )?;
    let signal = design.signal("SIGNAL")?;
    let resistor = design.add(parts::resistor("R1", Real::from(1_000)))?;
    design.connect(&signal, [resistor.pin("1")?])?;

    let report = design.check();
    assert!(!report.is_valid(), "R1 pin 2 is intentionally unconnected");
    for diagnostic in report.diagnostics {
        println!("{:?}", diagnostic.issue);
        for source in diagnostic.sources {
            println!("  at {source}");
        }
    }
    Ok(())
}
