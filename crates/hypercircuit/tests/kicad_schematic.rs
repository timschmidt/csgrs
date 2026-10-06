use std::collections::BTreeMap;
use std::process::Command;

use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitPort, ComponentId,
    DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, KiCadSchematicBookExportOptions,
    KiCadSchematicBookImportReport, KiCadSchematicExportError, KiCadSchematicExportOmission,
    KiCadSchematicExportOptions, KiCadSchematicImportOmission, KiCadSchematicImportReport, Net,
    NetId, PinBinding, PinElectricalKind, PinRef, PortDirection, PortId, Real,
    SchematicAutoLayoutPolicy, SchematicEndpoint, SchematicGraphic, SchematicGraphicFill,
    SchematicLayout, SchematicPoint, SchematicSheet, SchematicSheetId, SchematicSheetLink,
    SchematicSheetLinkId, SchematicSheetPort, SchematicSheetPortId, SchematicSymbolId,
    SchematicWire, SchematicWireId, TransientPolicy,
};

fn pin(name: &str, kind: PinElectricalKind) -> DevicePin {
    DevicePin {
        pin: PinRef::new(name).unwrap(),
        kind,
        optional: false,
    }
}

fn instance(id: &str, model: &str, bindings: &[(&str, &NetId)]) -> CircuitInstance {
    CircuitInstance {
        id: CircuitInstanceId::new(id).unwrap(),
        component: ComponentId::new(id).unwrap(),
        part: None,
        model: DeviceModelId::new(model).unwrap(),
        pins: bindings
            .iter()
            .map(|(pin, net)| PinBinding {
                pin: PinRef::new(*pin).unwrap(),
                net: (*net).clone(),
            })
            .collect(),
        parameters: Vec::new(),
    }
}

fn divider() -> Circuit {
    let ground = NetId::new("GND").unwrap();
    let supply = NetId::new("VCC").unwrap();
    let output = NetId::new("OUT").unwrap();
    Circuit::new(
        CircuitId::new("kicad-schematic-divider").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    })
    .with_net(Net {
        id: supply.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: output.clone(),
        is_ground: false,
    })
    .with_port(CircuitPort {
        id: PortId::new("VIN").unwrap(),
        net: supply.clone(),
        direction: PortDirection::Input,
        optional: false,
    })
    .with_port(CircuitPort {
        id: PortId::new("VOUT").unwrap(),
        net: output.clone(),
        direction: PortDirection::Output,
        optional: false,
    })
    .with_device_model(DeviceModel {
        id: DeviceModelId::new("voltage-source").unwrap(),
        kind: DeviceModelKind::VoltageSource,
        pins: vec![
            pin("pos", PinElectricalKind::PowerOutput),
            pin("neg", PinElectricalKind::PowerOutput),
        ],
        parameters: Vec::new(),
    })
    .with_device_model(DeviceModel {
        id: DeviceModelId::new("resistor").unwrap(),
        kind: DeviceModelKind::Resistor,
        pins: vec![
            pin("1", PinElectricalKind::Passive),
            pin("2", PinElectricalKind::Passive),
        ],
        parameters: Vec::new(),
    })
    .with_instance(instance(
        "V1",
        "voltage-source",
        &[("pos", &supply), ("neg", &ground)],
    ))
    .with_instance(instance(
        "R1",
        "resistor",
        &[("1", &supply), ("2", &output)],
    ))
    .with_instance(instance(
        "R2",
        "resistor",
        &[("1", &output), ("2", &ground)],
    ))
}

fn hierarchical_divider() -> (Circuit, SchematicLayout) {
    let circuit = divider();
    let generated = circuit
        .auto_schematic(SchematicAutoLayoutPolicy::default())
        .unwrap()
        .layout;
    let root_symbol = generated
        .symbols
        .iter()
        .find(|symbol| symbol.instance.as_str() == "R1")
        .unwrap()
        .clone();
    let child_symbol = generated
        .symbols
        .iter()
        .find(|symbol| symbol.instance.as_str() == "R2")
        .unwrap()
        .clone();
    let output = NetId::new("OUT").unwrap();
    let root = SchematicSheetId::new("root").unwrap();
    let child = SchematicSheetId::new("load").unwrap();
    let root_port = SchematicSheetPortId::new("root-out").unwrap();
    let child_port = SchematicSheetPortId::new("load-in").unwrap();
    let root_wire = SchematicWireId::new("root-out-wire").unwrap();
    let child_wire = SchematicWireId::new("load-in-wire").unwrap();
    let layout = SchematicLayout {
        symbol_definitions: generated.symbol_definitions.clone(),
        symbols: vec![root_symbol.clone(), child_symbol.clone()],
        wires: vec![
            SchematicWire {
                id: root_wire.clone(),
                net: output.clone(),
                from: SchematicEndpoint::Pin {
                    symbol: root_symbol.id.clone(),
                    pin: PinRef::new("2").unwrap(),
                },
                waypoints: Vec::new(),
                to: SchematicEndpoint::SheetPort(root_port.clone()),
            },
            SchematicWire {
                id: child_wire.clone(),
                net: output.clone(),
                from: SchematicEndpoint::SheetPort(child_port.clone()),
                waypoints: Vec::new(),
                to: SchematicEndpoint::Pin {
                    symbol: child_symbol.id.clone(),
                    pin: PinRef::new("1").unwrap(),
                },
            },
        ],
        sheets: vec![
            SchematicSheet {
                id: root.clone(),
                title: "Divider".into(),
                parent: None,
                symbols: vec![root_symbol.id],
                ports: Vec::new(),
                wires: vec![root_wire],
                labels: Vec::new(),
            },
            SchematicSheet {
                id: child.clone(),
                title: "Load".into(),
                parent: Some(root.clone()),
                symbols: vec![child_symbol.id],
                ports: Vec::new(),
                wires: vec![child_wire],
                labels: Vec::new(),
            },
        ],
        sheet_ports: vec![
            SchematicSheetPort {
                id: root_port.clone(),
                sheet: root,
                net: output.clone(),
                name: "OUT".into(),
                position: SchematicPoint::new(Real::from(90), Real::from(20)),
            },
            SchematicSheetPort {
                id: child_port.clone(),
                sheet: child,
                net: output,
                name: "OUT".into(),
                position: SchematicPoint::new(Real::from(70), Real::from(20)),
            },
        ],
        sheet_links: vec![SchematicSheetLink {
            id: SchematicSheetLinkId::new("divider-to-load").unwrap(),
            parent_port: root_port,
            child_port,
        }],
        ..SchematicLayout::default()
    };
    assert!(layout.validate(&circuit).is_valid());
    (circuit, layout)
}

#[test]
fn reusable_multipart_library_and_exact_graphics_round_trip_once() {
    let circuit = divider();
    let generated = circuit
        .auto_schematic(SchematicAutoLayoutPolicy::default())
        .unwrap()
        .layout;
    let mut first = generated
        .symbols
        .iter()
        .find(|symbol| symbol.instance.as_str() == "R1")
        .unwrap()
        .clone();
    first.id = SchematicSymbolId::new("R1:A").unwrap();
    first.position = SchematicPoint::new(Real::from(20), Real::from(20));
    let mut second = first.clone();
    second.id = SchematicSymbolId::new("R1:B").unwrap();
    second.unit = 2;
    second.position = SchematicPoint::new(Real::from(50), Real::from(20));

    let mut definition = generated
        .symbol_definition(&first.definition)
        .unwrap()
        .clone();
    let mut unit_a = definition.units[0].clone();
    let mut unit_b = unit_a.clone();
    unit_a.pins.truncate(1);
    unit_b.unit = 2;
    unit_b.pins.remove(0);
    unit_a.graphics = vec![
        SchematicGraphic::Rectangle {
            start: SchematicPoint::new(Real::from(-5), Real::from(-4)),
            end: SchematicPoint::new(Real::from(5), Real::from(4)),
            stroke_width: Real::one(),
            fill: SchematicGraphicFill::Background,
        },
        SchematicGraphic::Line {
            from: SchematicPoint::new(Real::from(-3), Real::zero()),
            to: SchematicPoint::new(Real::from(3), Real::zero()),
            stroke_width: Real::one(),
        },
        SchematicGraphic::Circle {
            center: SchematicPoint::new(Real::zero(), Real::zero()),
            radius: Real::from(2),
            stroke_width: Real::one(),
            fill: SchematicGraphicFill::None,
        },
    ];
    unit_b.graphics = vec![
        SchematicGraphic::Arc {
            start: SchematicPoint::new(Real::from(-2), Real::zero()),
            mid: SchematicPoint::new(Real::zero(), Real::from(-2)),
            end: SchematicPoint::new(Real::from(2), Real::zero()),
            stroke_width: Real::one(),
        },
        SchematicGraphic::Polyline {
            points: vec![
                SchematicPoint::new(Real::from(-3), Real::from(2)),
                SchematicPoint::new(Real::zero(), Real::from(-2)),
                SchematicPoint::new(Real::from(3), Real::from(2)),
            ],
            closed: true,
            stroke_width: Real::one(),
            fill: SchematicGraphicFill::Foreground,
        },
        SchematicGraphic::Text {
            position: SchematicPoint::new(Real::zero(), Real::from(3)),
            text: "B".into(),
            size: Real::from(2),
            quarter_turns: 0,
        },
    ];
    definition.units = vec![unit_a, unit_b];
    let layout = SchematicLayout {
        symbol_definitions: vec![definition],
        symbols: vec![first, second],
        ..SchematicLayout::default()
    };
    assert!(layout.validate(&circuit).is_valid());

    let exported = layout
        .export_kicad_schematic(&circuit, KiCadSchematicExportOptions::default())
        .unwrap();
    assert_eq!(exported.symbols.len(), 2);
    assert_eq!(
        exported.symbols[0].library_id,
        exported.symbols[1].library_id
    );
    let imported = KiCadSchematicImportReport::from_str(&exported.schematic, &circuit).unwrap();
    assert_eq!(
        imported.layout.symbol_definitions,
        layout.symbol_definitions
    );
    assert_eq!(imported.layout.symbols, layout.symbols);

    let svg = layout
        .to_svg(&circuit, hypercircuit::SchematicSvgOptions::default())
        .unwrap()
        .svg;
    assert!(svg.contains("<polygon"));
    assert!(svg.contains("<path"));
    assert!(svg.contains(">B</text>"));

    if Command::new("kicad-cli").arg("version").output().is_ok() {
        let directory = std::env::temp_dir().join(format!(
            "hypercircuit-kicad-multipart-{}",
            std::process::id()
        ));
        let output = directory.join("svg");
        std::fs::create_dir_all(&output).unwrap();
        let schematic = directory.join("multipart.kicad_sch");
        std::fs::write(&schematic, exported.schematic).unwrap();
        let result = Command::new("kicad-cli")
            .env("XDG_CONFIG_HOME", directory.join("config"))
            .args(["sch", "export", "svg", "--exclude-drawing-sheet"])
            .arg("--output")
            .arg(&output)
            .arg(&schematic)
            .output()
            .unwrap();
        assert!(
            result.status.success(),
            "KiCad CLI rejected reusable multipart graphics:\n{}",
            String::from_utf8_lossy(&result.stderr)
        );
    }
}

#[test]
fn native_schematic_round_trip_reconstructs_presentation_against_circuit_truth() {
    let circuit = divider();
    let generated = circuit
        .auto_schematic(SchematicAutoLayoutPolicy::default())
        .unwrap();
    let exported = generated
        .layout
        .export_kicad_schematic(&circuit, KiCadSchematicExportOptions::default())
        .unwrap();

    assert!(exported.schematic.starts_with("(kicad_sch"));
    assert!(exported.schematic.contains("(generator \"hypercircuit\")"));
    assert!(exported.schematic.contains("\"HyperCircuit Symbol\""));
    assert_eq!(exported.symbols.len(), generated.layout.symbols.len());
    assert!(!exported.numeric_projections.is_empty());
    let imported = KiCadSchematicImportReport::from_str(&exported.schematic, &circuit).unwrap();
    assert!(imported.layout.validate(&circuit).is_valid());
    assert_eq!(imported.layout.symbols, generated.layout.symbols);
    assert_eq!(imported.layout.ports, generated.layout.ports);
    assert_eq!(imported.layout.wires.len(), generated.layout.wires.len());
    assert_eq!(imported.layout.labels.len(), generated.layout.labels.len());
    assert!(
        imported
            .omissions
            .contains(&KiCadSchematicImportOmission::GeneratedWireIdentities {
                count: generated.layout.wires.len(),
            })
    );
}

#[test]
fn bent_wires_split_into_native_segments_and_rejoin_as_valid_connectivity() {
    let circuit = divider();
    let mut layout = circuit
        .auto_schematic(SchematicAutoLayoutPolicy::default())
        .unwrap()
        .layout;
    let ground_wire = layout
        .wires
        .iter_mut()
        .find(|wire| wire.net.as_str() == "GND")
        .unwrap();
    ground_wire.waypoints = vec![
        SchematicPoint::new(Real::from(20), Real::from(40)),
        SchematicPoint::new(Real::from(100), Real::from(40)),
    ];
    let ground_wire_id = ground_wire.id.clone();
    assert!(layout.validate(&circuit).is_valid());

    let exported = layout
        .export_kicad_schematic(&circuit, KiCadSchematicExportOptions::default())
        .unwrap();
    let projection = exported
        .wires
        .iter()
        .find(|projection| projection.wire == ground_wire_id)
        .unwrap();
    assert_eq!(projection.native_uuids.len(), 3);
    assert_eq!(exported.generated_connectivity_labels, 2);

    let imported = KiCadSchematicImportReport::from_str(&exported.schematic, &circuit).unwrap();
    assert!(imported.layout.validate(&circuit).is_valid());
    assert_eq!(imported.layout.labels.len(), layout.labels.len());
    assert_eq!(imported.layout.wires.len(), layout.wires.len());
    assert_eq!(
        imported
            .layout
            .wires
            .iter()
            .filter(|wire| wire.net.as_str() == "GND")
            .count(),
        1
    );
}

#[test]
fn native_symbol_position_edits_reimport_without_changing_circuit_topology() {
    let circuit = divider();
    let generated = circuit
        .auto_schematic(SchematicAutoLayoutPolicy::default())
        .unwrap();
    let symbol = generated.layout.symbols[0].clone();
    let layout = SchematicLayout {
        symbol_definitions: generated
            .layout
            .symbol_definitions
            .iter()
            .filter(|definition| definition.id == symbol.definition)
            .cloned()
            .collect(),
        symbols: vec![symbol.clone()],
        ..SchematicLayout::default()
    };
    let exported = layout
        .export_kicad_schematic(&circuit, KiCadSchematicExportOptions::default())
        .unwrap();
    let library_id = exported.symbols[0].library_id.clone();
    let original = format!(
        "(lib_id \"{}\")\n    (at {:.6} {:.6} 0)",
        library_id,
        symbol.position.x.to_f64_lossy().unwrap(),
        symbol.position.y.to_f64_lossy().unwrap()
    );
    let edited = exported.schematic.replacen(
        &original,
        &format!(
            "(lib_id \"{}\")\n    (at 42.500000 19.250000 0)",
            library_id
        ),
        1,
    );
    assert_ne!(edited, exported.schematic);

    let imported = KiCadSchematicImportReport::from_str(&edited, &circuit).unwrap();
    assert_eq!(
        imported.layout.symbols[0].position,
        SchematicPoint::new(
            (Real::from(85) / Real::from(2)).unwrap(),
            (Real::from(77) / Real::from(4)).unwrap(),
        )
    );
    assert_eq!(circuit.instances.len(), 3);
    assert_eq!(imported.layout.symbols[0].instance, symbol.instance);
}

#[test]
fn label_projection_and_hierarchy_boundary_are_typed() {
    let circuit = divider();
    let mut layout = circuit
        .auto_schematic(SchematicAutoLayoutPolicy::default())
        .unwrap()
        .layout;
    layout.labels[0].text = "friendly supply".into();
    let exported = layout
        .export_kicad_schematic(&circuit, KiCadSchematicExportOptions::default())
        .unwrap();
    assert!(exported.omissions.iter().any(|omission| matches!(
        omission,
        KiCadSchematicExportOmission::LabelTextProjected { authored, .. }
            if authored == "friendly supply"
    )));

    layout.sheets.push(SchematicSheet {
        id: SchematicSheetId::new("root").unwrap(),
        title: "Root".into(),
        parent: None,
        symbols: layout
            .symbols
            .iter()
            .map(|symbol| symbol.id.clone())
            .collect(),
        ports: layout
            .ports
            .iter()
            .map(|placement| placement.port.clone())
            .collect(),
        wires: layout.wires.iter().map(|wire| wire.id.clone()).collect(),
        labels: layout.labels.iter().map(|label| label.id.clone()).collect(),
    });
    assert_eq!(
        layout.export_kicad_schematic(&circuit, KiCadSchematicExportOptions::default()),
        Err(KiCadSchematicExportError::HierarchyRequiresMultiFileExport)
    );
}

#[test]
fn installed_kicad_cli_can_render_the_generated_native_file() {
    let available = Command::new("kicad-cli").arg("version").output();
    if available.is_err() {
        return;
    }
    let circuit = divider();
    let layout = circuit
        .auto_schematic(SchematicAutoLayoutPolicy::default())
        .unwrap()
        .layout;
    let directory =
        std::env::temp_dir().join(format!("hypercircuit-kicad-sch-{}", std::process::id()));
    let output = directory.join("svg");
    std::fs::create_dir_all(&output).unwrap();
    let stages = [
        ("empty", SchematicLayout::default()),
        (
            "symbols",
            SchematicLayout {
                symbol_definitions: layout.symbol_definitions.clone(),
                symbols: layout.symbols.clone(),
                ..SchematicLayout::default()
            },
        ),
        (
            "ports",
            SchematicLayout {
                symbol_definitions: layout.symbol_definitions.clone(),
                symbols: layout.symbols.clone(),
                ports: layout.ports.clone(),
                ..SchematicLayout::default()
            },
        ),
        (
            "labels",
            SchematicLayout {
                symbol_definitions: layout.symbol_definitions.clone(),
                symbols: layout.symbols.clone(),
                ports: layout.ports.clone(),
                labels: layout.labels.clone(),
                ..SchematicLayout::default()
            },
        ),
        ("complete", layout),
    ];
    for (stage, layout) in stages {
        let exported = layout
            .export_kicad_schematic(&circuit, KiCadSchematicExportOptions::default())
            .unwrap();
        let schematic = directory.join(format!("{stage}.kicad_sch"));
        std::fs::write(&schematic, exported.schematic).unwrap();
        let result = Command::new("kicad-cli")
            .env("XDG_CONFIG_HOME", directory.join("config"))
            .args(["sch", "export", "svg", "--exclude-drawing-sheet"])
            .arg("--output")
            .arg(&output)
            .arg(&schematic)
            .output()
            .unwrap();
        assert!(
            result.status.success(),
            "KiCad CLI rejected {stage} generated schematic:\nstdout:\n{}\nstderr:\n{}",
            String::from_utf8_lossy(&result.stdout),
            String::from_utf8_lossy(&result.stderr)
        );
    }
    assert!(std::fs::read_dir(&output).unwrap().any(|entry| {
        entry
            .unwrap()
            .path()
            .extension()
            .is_some_and(|ext| ext == "svg")
    }));
    std::fs::remove_dir_all(directory).unwrap();
}

#[test]
fn installed_kicad_cli_can_follow_the_generated_native_hierarchy() {
    let available = Command::new("kicad-cli").arg("version").output();
    if available.is_err() {
        return;
    }
    let (circuit, layout) = hierarchical_divider();
    let exported = layout
        .export_kicad_schematic_book(
            &circuit,
            KiCadSchematicBookExportOptions {
                root_filename: "divider.kicad_sch".into(),
                ..KiCadSchematicBookExportOptions::default()
            },
        )
        .unwrap();
    assert_eq!(exported.files.len(), 2);
    assert!(exported.files[0].schematic.contains("(sheet"));
    assert!(
        exported.files[0]
            .schematic
            .contains("\"HyperCircuit Link 1\"")
    );
    assert!(
        exported.files[1]
            .schematic
            .contains("(hierarchical_label \"OUT\"")
    );
    let package = exported
        .files
        .iter()
        .map(|file| (file.projection.filename.clone(), file.schematic.clone()))
        .collect::<BTreeMap<_, _>>();
    let imported =
        KiCadSchematicBookImportReport::from_files(&exported.root_filename, &package, &circuit)
            .unwrap();
    assert!(imported.layout.validate(&circuit).is_valid());
    assert_eq!(imported.files.len(), 2);
    assert_eq!(imported.layout.sheet_ports, layout.sheet_ports);
    assert_eq!(imported.layout.sheet_links, layout.sheet_links);
    assert_eq!(imported.layout.symbols, layout.symbols);
    assert_eq!(
        imported
            .layout
            .sheets
            .iter()
            .map(|sheet| {
                (
                    sheet.id.as_str().to_owned(),
                    sheet
                        .parent
                        .as_ref()
                        .map(|parent| parent.as_str().to_owned()),
                    sheet.symbols.len(),
                    sheet.wires.len(),
                )
            })
            .collect::<Vec<_>>(),
        vec![
            ("root".into(), None, 1, 1),
            ("load".into(), Some("root".into()), 1, 1),
        ]
    );

    let directory =
        std::env::temp_dir().join(format!("hypercircuit-kicad-book-{}", std::process::id()));
    let output = directory.join("svg");
    std::fs::create_dir_all(&output).unwrap();
    for file in &exported.files {
        std::fs::write(directory.join(&file.projection.filename), &file.schematic).unwrap();
    }
    let result = Command::new("kicad-cli")
        .env("XDG_CONFIG_HOME", directory.join("config"))
        .args(["sch", "export", "svg", "--exclude-drawing-sheet"])
        .arg("--output")
        .arg(&output)
        .arg(directory.join(&exported.root_filename))
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "KiCad CLI rejected generated hierarchy:\nstdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&result.stdout),
        String::from_utf8_lossy(&result.stderr)
    );
    assert_eq!(
        std::fs::read_dir(&output)
            .unwrap()
            .filter_map(Result::ok)
            .filter(|entry| entry.path().extension().is_some_and(|ext| ext == "svg"))
            .count(),
        2
    );
    std::fs::remove_dir_all(directory).unwrap();
}

#[test]
fn nested_book_round_trip_restores_link_identity_and_reports_missing_files() {
    let (circuit, mut layout) = hierarchical_divider();
    let load = SchematicSheetId::new("load").unwrap();
    let leaf = SchematicSheetId::new("sensor").unwrap();
    let parent_port = SchematicSheetPortId::new("load-out").unwrap();
    let child_port = SchematicSheetPortId::new("sensor-in").unwrap();
    layout.sheets.push(SchematicSheet {
        id: leaf.clone(),
        title: "Sensor".into(),
        parent: Some(load.clone()),
        symbols: Vec::new(),
        ports: Vec::new(),
        wires: Vec::new(),
        labels: Vec::new(),
    });
    layout.sheet_ports.push(SchematicSheetPort {
        id: parent_port.clone(),
        sheet: load,
        net: NetId::new("OUT").unwrap(),
        name: "LOAD_OUT".into(),
        position: SchematicPoint::new(Real::from(120), Real::from(30)),
    });
    layout.sheet_ports.push(SchematicSheetPort {
        id: child_port.clone(),
        sheet: leaf,
        net: NetId::new("OUT").unwrap(),
        name: "OUT".into(),
        position: SchematicPoint::new(Real::from(20), Real::from(30)),
    });
    layout.sheet_links.push(SchematicSheetLink {
        id: SchematicSheetLinkId::new("load-to-sensor").unwrap(),
        parent_port,
        child_port,
    });
    assert!(layout.validate(&circuit).is_valid());

    let exported = layout
        .export_kicad_schematic_book(
            &circuit,
            KiCadSchematicBookExportOptions {
                root_filename: "nested.kicad_sch".into(),
                ..KiCadSchematicBookExportOptions::default()
            },
        )
        .unwrap();
    assert_eq!(exported.files.len(), 3);
    assert_eq!(
        exported.files[2]
            .projection
            .instance_path
            .matches('/')
            .count(),
        3
    );
    assert!(exported.files[1].omissions.iter().any(|omission| matches!(
        omission,
        KiCadSchematicExportOmission::SheetPortNameProjected {
            parent_name,
            emitted,
            ..
        } if parent_name == "LOAD_OUT" && emitted == "OUT"
    )));
    let mut package = exported
        .files
        .iter()
        .map(|file| (file.projection.filename.clone(), file.schematic.clone()))
        .collect::<BTreeMap<_, _>>();
    let imported =
        KiCadSchematicBookImportReport::from_files(&exported.root_filename, &package, &circuit)
            .unwrap();
    assert!(imported.layout.validate(&circuit).is_valid());
    assert_eq!(imported.layout.sheets.len(), 3);
    assert_eq!(imported.layout.sheet_ports, layout.sheet_ports);
    assert_eq!(imported.layout.sheet_links, layout.sheet_links);

    let missing = exported.files[2].projection.filename.clone();
    package.remove(&missing);
    assert_eq!(
        KiCadSchematicBookImportReport::from_files(&exported.root_filename, &package, &circuit,),
        Err(hypercircuit::KiCadSchematicImportError::MissingBookFile(
            missing
        ))
    );
}

#[test]
fn non_schematic_roots_are_rejected_without_mutating_circuit_truth() {
    let circuit = divider();
    let original = circuit.clone();
    assert!(KiCadSchematicImportReport::from_str("(kicad_pcb)", &circuit).is_err());
    assert_eq!(circuit, original);

    let empty = SchematicLayout::default();
    assert_eq!(
        empty.export_kicad_schematic(
            &circuit,
            KiCadSchematicExportOptions {
                decimal_places: 13,
                project_name: "invalid".into(),
            }
        ),
        Err(KiCadSchematicExportError::InvalidOptions)
    );
}
