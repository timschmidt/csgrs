use std::collections::BTreeMap;

use hypercircuit::{
    AdapterKind, Circuit, CircuitId, KiCadSchematicBookExportOptions,
    KiCadSchematicBookImportReport, Net, NetId, Real, SchematicEndpoint, SchematicLayout,
    SchematicPoint, SchematicSheet, SchematicSheetId, SchematicSheetLink, SchematicSheetLinkId,
    SchematicSheetPort, SchematicSheetPortId, SchematicSvgOptions, SchematicWire, SchematicWireId,
    TransientPolicy,
};

fn point(x: i64, y: i64) -> SchematicPoint {
    SchematicPoint::new(Real::from(x), Real::from(y))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let signal = NetId::new("SIGNAL")?;
    let circuit = Circuit::new(
        CircuitId::new("hierarchical-example")?,
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    });
    let root = SchematicSheetId::new("root")?;
    let child = SchematicSheetId::new("sensor")?;
    let root_port = SchematicSheetPortId::new("root-signal")?;
    let child_port = SchematicSheetPortId::new("sensor-signal")?;
    let root_wire = SchematicWireId::new("root-wire")?;
    let child_wire = SchematicWireId::new("sensor-wire")?;
    let layout = SchematicLayout {
        wires: vec![
            SchematicWire {
                id: root_wire.clone(),
                net: signal.clone(),
                from: SchematicEndpoint::Junction(point(0, 0)),
                waypoints: Vec::new(),
                to: SchematicEndpoint::SheetPort(root_port.clone()),
            },
            SchematicWire {
                id: child_wire.clone(),
                net: signal.clone(),
                from: SchematicEndpoint::SheetPort(child_port.clone()),
                waypoints: Vec::new(),
                to: SchematicEndpoint::Junction(point(30, 0)),
            },
        ],
        sheets: vec![
            SchematicSheet {
                id: root.clone(),
                title: "Controller".into(),
                parent: None,
                symbols: Vec::new(),
                ports: Vec::new(),
                wires: vec![root_wire],
                labels: Vec::new(),
            },
            SchematicSheet {
                id: child.clone(),
                title: "Sensor".into(),
                parent: Some(root.clone()),
                symbols: Vec::new(),
                ports: Vec::new(),
                wires: vec![child_wire],
                labels: Vec::new(),
            },
        ],
        sheet_ports: vec![
            SchematicSheetPort {
                id: root_port.clone(),
                sheet: root,
                net: signal.clone(),
                name: "SIGNAL".into(),
                position: point(20, 0),
            },
            SchematicSheetPort {
                id: child_port.clone(),
                sheet: child,
                net: signal,
                name: "SIGNAL".into(),
                position: point(10, 0),
            },
        ],
        sheet_links: vec![SchematicSheetLink {
            id: SchematicSheetLinkId::new("controller-to-sensor")?,
            parent_port: root_port,
            child_port,
        }],
        ..SchematicLayout::default()
    };

    let pages = layout.to_svg_book(&circuit, SchematicSvgOptions::default())?;
    for page in pages.pages {
        println!("{}: {} SVG bytes", page.title, page.report.svg.len());
    }
    if let Some(directory) = std::env::args_os().nth(1) {
        let directory = std::path::PathBuf::from(directory);
        std::fs::create_dir_all(&directory)?;
        let book = layout.export_kicad_schematic_book(
            &circuit,
            KiCadSchematicBookExportOptions {
                root_filename: "hierarchical-example.kicad_sch".into(),
                ..KiCadSchematicBookExportOptions::default()
            },
        )?;
        let mut files = BTreeMap::new();
        for file in &book.files {
            std::fs::write(directory.join(&file.projection.filename), &file.schematic)?;
            files.insert(file.projection.filename.clone(), file.schematic.clone());
        }
        let imported =
            KiCadSchematicBookImportReport::from_files(&book.root_filename, &files, &circuit)?;
        println!(
            "wrote and re-imported {} KiCad sheets at {}",
            imported.files.len(),
            directory.display()
        );
    }
    Ok(())
}
