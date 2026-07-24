//! Define one reusable component library entry and instantiate it twice.

use hypercircuit::{
    BoardOutline, Design, DeviceModelKind, Footprint, PartDefinition, PartInstance, PartSymbolUnit,
    PcbStackup, Real, SchematicPinSide, SchematicPoint, SymbolPin, SymbolUnitPlacement, pin,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut design = Design::new(
        "reusable-parts",
        BoardOutline::rectangle(Real::from(24), Real::from(12)),
        PcbStackup::single_layer(Real::one(), None),
    )?;
    let common = design.ground("GND")?;
    let resistor = design.define_part(
        PartDefinition::new("resistor-0603", "0603 resistor")
            .model_kind(DeviceModelKind::Resistor)
            .part_ref("hyperparts:resistor-0603")
            .pin(pin("1").pad("1"))
            .pin(pin("2").pad("2"))
            .symbol_name("R")
            .symbol_unit(
                PartSymbolUnit::new(1, Real::from(6), Real::from(3))
                    .pin(SymbolPin::new(
                        "1",
                        SchematicPoint::new(Real::from(-4), Real::zero()),
                        SchematicPinSide::Left,
                    ))
                    .pin(SymbolPin::new(
                        "2",
                        SchematicPoint::new(Real::from(4), Real::zero()),
                        SchematicPinSide::Right,
                    ))
                    .rectangular_body(Real::one())?,
            )
            .footprint(Footprint::two_pad_smd(
                Real::one(),
                Real::one(),
                Real::from(2),
                vec![TraceLayer(0)],
            )),
    )?;
    let r1 = design.instantiate(
        &resistor,
        PartInstance::new("R1")
            .parameter("resistance", Real::from(1_000), "ohm")
            .symbol(SymbolUnitPlacement::new(
                1,
                SchematicPoint::new(Real::from(5), Real::from(3)),
            ))
            .at(Point2::new(Real::from(6), Real::from(6))),
    )?;
    let r2 = design.instantiate(
        &resistor,
        PartInstance::new("R2")
            .parameter("resistance", Real::from(2_200), "ohm")
            .symbol(SymbolUnitPlacement::new(
                1,
                SchematicPoint::new(Real::from(15), Real::from(3)),
            ))
            .at(Point2::new(Real::from(16), Real::from(6))),
    )?;
    design.connect(
        &common,
        [r1.pin("1")?, r1.pin("2")?, r2.pin("1")?, r2.pin("2")?],
    )?;

    let checked = design.finish()?;
    assert_eq!(checked.circuit.device_models.len(), 1);
    assert_eq!(checked.circuit.instances.len(), 2);
    assert_eq!(checked.schematic.symbol_definitions.len(), 1);
    assert_eq!(checked.layout.land_patterns.len(), 1);
    Ok(())
}
