use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, Circuit, CircuitId, LcedaProExportOptions,
    LcedaProExportReport, LcedaProImportReport, LcedaSchematicImportReport, LcedaSourceLengthUnit,
    Net, NetId, PcbDesignRules, PcbLayout, PcbRoute, PcbStackup, Real, RouteId, SchematicEndpoint,
    SchematicLabel, SchematicLabelId, SchematicLayout, SchematicPoint, SchematicWire,
    SchematicWireId, StackupLayer, StackupLayerKind, TransientPolicy,
};
use hyperlattice::Point2;
use hyperpath::{LinePathSegment, TraceLayer};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let signal = NetId::new("SIGNAL")?;
    let circuit = Circuit::new(
        CircuitId::new("lceda-demo")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    });
    let schematic = SchematicLayout {
        wires: vec![SchematicWire {
            id: SchematicWireId::new("signal-wire")?,
            net: signal.clone(),
            from: SchematicEndpoint::Junction(SchematicPoint::new(Real::from(5), Real::from(5))),
            waypoints: vec![SchematicPoint::new(Real::from(20), Real::from(5))],
            to: SchematicEndpoint::Junction(SchematicPoint::new(Real::from(30), Real::from(15))),
        }],
        labels: vec![SchematicLabel {
            id: SchematicLabelId::new("signal-label")?,
            net: signal.clone(),
            position: SchematicPoint::new(Real::from(20), Real::from(5)),
            text: "SIGNAL".into(),
        }],
        ..SchematicLayout::default()
    };
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
            net: signal,
            layer: front,
            width: Real::from(1),
            segments: vec![LinePathSegment::new(point(5, 5), point(30, 20)).into()],
        }],
        vias: Vec::new(),
        zones: Vec::new(),
        keepouts: Vec::new(),
        rules: PcbDesignRules::default(),
    };

    let report = LcedaProExportReport::from_design(
        &circuit,
        Some(&schematic),
        Some(&layout),
        LcedaProExportOptions::millimeters(),
    )?;
    std::fs::write("lceda-demo.epro2", &report.archive)?;
    let imported = LcedaProImportReport::from_archive(
        &circuit,
        &layout,
        &report.archive,
        LcedaSourceLengthUnit::Millimeter,
    )?;
    let imported_schematic =
        LcedaSchematicImportReport::from_archive(&circuit, &schematic, &report.archive)?;
    let replay = LcedaProExportReport::from_design(
        &circuit,
        Some(&imported_schematic.schematic),
        Some(&imported.layout),
        LcedaProExportOptions::millimeters(),
    )?;
    std::fs::write("lceda-demo-roundtrip.epro2", &replay.archive)?;
    println!(
        "wrote {} bytes with {} numeric projections and {} disclosed omissions; restored {} route segments and {} schematic wires with {} PCB plus {} schematic exact numeric imports",
        report.archive.len(),
        report.numeric_projections.len(),
        report.omissions.len(),
        imported.route_segments,
        imported_schematic.wires,
        imported.numeric_imports.len(),
        imported_schematic.numeric_imports.len(),
    );
    Ok(())
}
