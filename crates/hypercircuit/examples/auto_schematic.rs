#![cfg(feature = "layout")]

use hypercircuit::{
    BoardOutline, Design, PcbStackup, Real, SchematicAutoLayoutPolicy, SchematicSvgOptions, parts,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut design = Design::new(
        "auto-schematic-divider",
        BoardOutline::rectangle(Real::from(30), Real::from(20)),
        PcbStackup::single_layer(Real::one(), None),
    )?;
    let supply = design.signal("VCC")?;
    let output = design.signal("OUT")?;
    let ground = design.ground("GND")?;
    let source = design.add(parts::voltage_source("V1", Real::from(5)))?;
    let upper = design.add(parts::resistor("R1", Real::from(1_000)))?;
    let lower = design.add(parts::resistor("R2", Real::from(1_000)))?;
    design.connect(&supply, [source.pin("pos")?, upper.pin("1")?])?;
    design.connect(&output, [upper.pin("2")?, lower.pin("1")?])?;
    design.connect(&ground, [source.pin("neg")?, lower.pin("2")?])?;

    let mut checked = design.finish()?;
    let report = checked.replace_schematic_with_auto_layout(SchematicAutoLayoutPolicy::default())?;
    let svg = checked
        .schematic
        .to_svg(&checked.circuit, SchematicSvgOptions::default())?;
    eprintln!(
        "generated {} symbols, {} wires, {} labels from {} connectivity edges",
        report.placements.len(),
        report.generated_wires,
        report.generated_labels,
        report.connectivity_edges
    );
    print!("{}", svg.svg);
    Ok(())
}
