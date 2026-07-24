//! Author a checked, simulatable PCB without manually assembling retained containers.

use hypercircuit::{
    BoardOutline, Design, Footprint, Keepout, KeepoutScope, NetClassRule, PcbStackup,
    PlacementRule, RailKind, Real, Route, SchematicPinSide, SchematicPoint, SchematicSvgOptions,
    Symbol, SymbolPin, Via, ViaMaskIntent, ViaStyleRule, Zone, parts,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut design = Design::new(
        "fluent-demo",
        BoardOutline::rectangle(Real::from(30), Real::from(18)),
        PcbStackup::two_layer(
            (Real::from(35) / Real::from(1_000))?,
            (Real::from(153) / Real::from(100))?,
            Some("hyperphysics:copper".into()),
            Some("hyperphysics:FR4".into()),
        ),
    )?;
    let supply = design.rail(
        "VCC",
        Some(Real::from(5)),
        Some(Real::one()),
        RailKind::Power,
    )?;
    let ground = design.ground("GND")?;
    let through = design.via_style(
        ViaStyleRule::new("through", Real::from(2), Real::one())
            .mask(ViaMaskIntent::tented())
            .span(TraceLayer(0), TraceLayer(1)),
    )?;
    design.net_class(
        NetClassRule::new("power", [&supply, &ground])
            .min_trace_width(Real::one())
            .preferred_trace_width(Real::one())
            .min_clearance(Real::one())
            .preferred_via_style(&through)
            .preferred_via_geometry(Real::from(2), Real::one())
            .max_via_count(2)
            .reference_plane(),
    )?;
    let source = design.add(
        parts::voltage_source("V1", Real::from(5)).symbol(
            Symbol::new(
                SchematicPoint::new(Real::from(4), Real::from(4)),
                Real::from(4),
                Real::from(3),
            )
            .pin(SymbolPin::new(
                "pos",
                SchematicPoint::new(Real::from(2), Real::zero()),
                SchematicPinSide::Right,
            ))
            .pin(SymbolPin::new(
                "neg",
                SchematicPoint::new(Real::from(-2), Real::zero()),
                SchematicPinSide::Left,
            )),
        ),
    )?;
    let resistor = design.add(
        parts::resistor("R1", Real::from(1_000))
            .symbol(Symbol::two_pin_horizontal(
                SchematicPoint::new(Real::from(12), Real::from(4)),
                Real::from(2),
                Real::from(3),
                Real::from(2),
            ))
            .footprint(Footprint::two_pad_smd(
                Real::one(),
                Real::one(),
                Real::from(2),
                vec![TraceLayer(0)],
            ))
            .at(Point2::new(Real::from(15), Real::from(9))),
    )?;
    design.connect(&supply, [source.pin("pos")?, resistor.pin("1")?])?;
    design.connect(&ground, [source.pin("neg")?, resistor.pin("2")?])?;
    design.constrain(PlacementRule::fixed(
        "r1-origin",
        &resistor,
        Point2::new(Real::from(15), Real::from(9)),
    ))?;
    design.route(
        &supply,
        Route::new("vcc-route", TraceLayer(0), Real::one()).line(
            Point2::new(Real::from(2), Real::from(9)),
            Point2::new(Real::from(14), Real::from(9)),
        ),
    )?;
    design.via(
        &supply,
        Via::new(
            "vcc-via",
            TraceLayer(0),
            TraceLayer(1),
            Point2::new(Real::from(2), Real::from(9)),
            Real::from(2),
            Real::one(),
        )
        .mask(ViaMaskIntent::tented()),
    )?;
    design.zone(
        &ground,
        Zone::solid(
            "ground-plane",
            TraceLayer(1),
            vec![
                Point2::new(Real::one(), Real::one()),
                Point2::new(Real::from(29), Real::one()),
                Point2::new(Real::from(29), Real::from(17)),
                Point2::new(Real::one(), Real::from(17)),
            ],
        )
        .clearance(Real::one()),
    )?;
    design.keepout(Keepout::new(
        "edge-connector",
        vec![
            Point2::new(Real::from(24), Real::from(6)),
            Point2::new(Real::from(29), Real::from(6)),
            Point2::new(Real::from(29), Real::from(12)),
            Point2::new(Real::from(24), Real::from(12)),
        ],
        KeepoutScope::Components,
    ))?;

    let checked = design.finish()?;
    let lowering = checked.circuit.lower_linear_devices();
    let schematic = checked
        .schematic
        .to_svg(&checked.circuit, SchematicSvgOptions::default())?;
    println!(
        "{} nets, {} parts, {} schematic wires, {} placed footprint, {} route, {} via, {} zone, {} keepout, {} simulation stamps, {} SVG bytes",
        checked.circuit.nets.len(),
        checked.circuit.instances.len(),
        checked.schematic.wires.len(),
        checked.layout.placements.len(),
        checked.layout.routes.len(),
        checked.layout.vias.len(),
        checked.layout.zones.len(),
        checked.layout.keepouts.len(),
        lowering.stamps.len(),
        schematic.svg.len(),
    );
    Ok(())
}
