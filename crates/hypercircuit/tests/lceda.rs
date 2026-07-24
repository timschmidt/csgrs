#![cfg(feature = "lceda")]

use std::collections::BTreeMap;
use std::str::FromStr;

use hypercircuit::{
    AdapterKind, BoardId, BoardOutline, BoardSide, Circuit, CircuitId, CircuitInstance,
    CircuitInstanceId, ComponentId, CopperZone, DeviceModel, DeviceModelId, DeviceModelKind,
    DevicePin, DrillShape, KeepoutId, KeepoutScope, LCEDA_PRO_EXPORT_VERSION, LandPattern,
    LandPatternId, LandPatternPad, LcedaExportError, LcedaExportOmission, LcedaImportError,
    LcedaImportOmission, LcedaProExportOptions, LcedaProExportReport, LcedaProImportReport,
    LcedaSchematicImportReport, LcedaSourceLengthUnit, Net, NetId, PadId, PadPinMap, PadShape,
    PcbDesignRules, PcbKeepout, PcbLayout, PcbPlacement, PcbRoute, PcbStackup, PcbVia, PinBinding,
    PinElectricalKind, PinRef, Plating, Real, RouteId, SchematicEndpoint, SchematicLabel,
    SchematicLabelId, SchematicLayout, SchematicPinPlacement, SchematicPinSide, SchematicPoint,
    SchematicSymbol, SchematicSymbolDefinition, SchematicSymbolDefinitionId, SchematicSymbolId,
    SchematicSymbolUnit, SchematicWire, SchematicWireId, StackupLayer, StackupLayerKind,
    TransientPolicy, ViaId, ZoneId,
};
use hyperlattice::Point2;
use hyperpath::{LinePathSegment, TraceLayer};

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn fixture() -> (Circuit, SchematicLayout, PcbLayout) {
    let signal = NetId::new("SIGNAL").unwrap();
    let other = NetId::new("OTHER_").unwrap();
    let model = DeviceModelId::new("resistor").unwrap();
    let instance = CircuitInstanceId::new("R1").unwrap();
    let pin = PinRef::new("1").unwrap();
    let circuit = Circuit::new(
        CircuitId::new("LCEDA demo").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: signal.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: other,
        is_ground: false,
    })
    .with_device_model(DeviceModel {
        id: model.clone(),
        kind: DeviceModelKind::Resistor,
        pins: vec![DevicePin {
            pin: pin.clone(),
            kind: PinElectricalKind::Passive,
            optional: false,
        }],
        parameters: Vec::new(),
    })
    .with_instance(CircuitInstance {
        id: instance.clone(),
        component: ComponentId::new("R1").unwrap(),
        part: None,
        model,
        pins: vec![PinBinding {
            pin: pin.clone(),
            net: signal.clone(),
        }],
        parameters: Vec::new(),
    });
    let symbol_definition = SchematicSymbolDefinitionId::new("resistor-symbol").unwrap();
    let symbol = SchematicSymbolId::new("R1:A").unwrap();
    let schematic = SchematicLayout {
        symbol_definitions: vec![SchematicSymbolDefinition {
            id: symbol_definition.clone(),
            model: DeviceModelId::new("resistor").unwrap(),
            name: "resistor".into(),
            units: vec![SchematicSymbolUnit {
                unit: 1,
                body_width: Real::from(20),
                body_height: Real::from(12),
                pins: vec![SchematicPinPlacement {
                    pin: pin.clone(),
                    position: SchematicPoint::new(Real::from(-3), Real::from(-4)),
                    side: SchematicPinSide::Left,
                }],
                graphics: Vec::new(),
            }],
        }],
        symbols: vec![SchematicSymbol {
            id: symbol.clone(),
            instance: instance.clone(),
            definition: symbol_definition,
            unit: 1,
            position: SchematicPoint::new(Real::from(3), Real::from(4)),
            quarter_turns: 1,
        }],
        wires: vec![SchematicWire {
            id: SchematicWireId::new("signal-wire").unwrap(),
            net: signal.clone(),
            from: SchematicEndpoint::Pin {
                symbol,
                pin: pin.clone(),
            },
            waypoints: vec![SchematicPoint::new(Real::from(10), Real::zero())],
            to: SchematicEndpoint::Junction(SchematicPoint::new(Real::from(10), Real::from(10))),
        }],
        labels: vec![SchematicLabel {
            id: SchematicLabelId::new("signal-label").unwrap(),
            net: signal.clone(),
            position: SchematicPoint::new(Real::from(6), Real::from(7)),
            text: "SIGNAL".into(),
        }],
        ..SchematicLayout::default()
    };
    let front = TraceLayer(0);
    let back = TraceLayer(1);
    let pattern = LandPatternId::new("R_0603").unwrap();
    let layout = PcbLayout {
        id: BoardId::new("main-board").unwrap(),
        outline: BoardOutline {
            exterior: vec![point(0, 0), point(30, 0), point(30, 20), point(0, 20)].into(),
            cutouts: vec![vec![point(20, 10), point(22, 10), point(22, 12), point(20, 12)].into()],
        },
        stackup: PcbStackup {
            layers: vec![
                StackupLayer {
                    name: "F.Cu".into(),
                    kind: StackupLayerKind::Conductor(front),
                    thickness: Real::from(1),
                    material: Some("copper".into()),
                },
                StackupLayer {
                    name: "Core".into(),
                    kind: StackupLayerKind::Dielectric,
                    thickness: Real::from(2),
                    material: Some("FR4".into()),
                },
                StackupLayer {
                    name: "B.Cu".into(),
                    kind: StackupLayerKind::Conductor(back),
                    thickness: Real::from(1),
                    material: Some("copper".into()),
                },
            ],
        },
        land_patterns: vec![LandPattern {
            id: pattern.clone(),
            pads: vec![LandPatternPad {
                id: PadId::new("1").unwrap(),
                center: point(0, 0),
                rotation_degrees: Real::from(30),
                copper_layers: vec![front, back],
                shape: PadShape::Obround {
                    width: Real::from(2),
                    height: Real::from(3),
                },
                drill: Some(DrillShape::Slot {
                    start: point(0, -1),
                    end: point(0, 1),
                    width: Real::from(1),
                }),
                plating: Plating::Plated,
                solder_mask_margin: Some(Real::zero()),
                paste_margin: None,
            }],
            pin_map: vec![PadPinMap {
                pin,
                pad: PadId::new("1").unwrap(),
            }],
            graphics: Vec::new(),
            body: None,
            models: Vec::new(),
        }],
        placements: vec![PcbPlacement {
            instance,
            land_pattern: pattern,
            position: point(5, 5),
            rotation_degrees: Real::zero(),
            side: BoardSide::Front,
        }],
        placement_constraints: Vec::new(),
        routes: vec![PcbRoute {
            id: RouteId::new("signal-route").unwrap(),
            net: signal.clone(),
            layer: front,
            width: Real::from(1),
            segments: vec![LinePathSegment::new(point(5, 5), point(15, 8)).into()],
        }],
        vias: vec![PcbVia {
            id: ViaId::new("signal-via").unwrap(),
            net: signal.clone(),
            start_layer: front,
            end_layer: back,
            center: point(15, 8),
            land_diameter: Real::from(2),
            drill_diameter: Real::from(1),
            plating: Plating::Plated,
            mask: hypercircuit::ViaMaskIntent::default(),
        }],
        zones: vec![CopperZone {
            id: ZoneId::new("signal-zone").unwrap(),
            net: signal,
            layer: back,
            boundary: vec![point(14, 7), point(19, 7), point(19, 12), point(14, 12)],
            clearance: Real::zero(),
            fill: hypercircuit::CopperZoneFill::Solid,
            connection: hypercircuit::CopperZoneConnection::Solid,
            islands: hypercircuit::CopperZoneIslandPolicy::retain_all(),
            stitching: None,
            priority: 0,
        }],
        keepouts: vec![PcbKeepout {
            id: KeepoutId::new("mounting").unwrap(),
            boundary: vec![point(24, 14), point(28, 14), point(28, 18), point(24, 18)],
            scope: KeepoutScope::Copper(vec![front, back]),
        }],
        rules: PcbDesignRules::default(),
    };
    (circuit, schematic, layout)
}

#[test]
fn exports_deterministic_parseable_pro_archive_with_authored_pcb_records() {
    let (circuit, schematic, layout) = fixture();
    assert!(circuit.validate().is_valid());
    assert!(schematic.validate(&circuit).is_valid());
    assert!(layout.validate(&circuit).is_valid());

    let first = LcedaProExportReport::from_design(
        &circuit,
        Some(&schematic),
        Some(&layout),
        LcedaProExportOptions::millimeters(),
    )
    .unwrap();
    let second = LcedaProExportReport::from_design(
        &circuit,
        Some(&schematic),
        Some(&layout),
        LcedaProExportOptions::millimeters(),
    )
    .unwrap();

    assert_eq!(first.archive, second.archive);
    assert_eq!(first.version, LCEDA_PRO_EXPORT_VERSION);
    assert!(first.archive.starts_with(b"PK\x03\x04"));
    let files = stored_zip_files(&first.archive);
    assert_eq!(files["project2.json"], first.project_json.as_bytes());
    assert_eq!(files["LCEDA_demo.epru"], first.record_stream.as_bytes());
    assert!(files.contains_key("IMAGE/"));
    serde_json::from_str::<serde_json::Value>(&first.project_json).unwrap();
    for line in first.record_stream.lines() {
        let (header, body) = line.split_once("||").unwrap();
        serde_json::from_str::<serde_json::Value>(header).unwrap();
        serde_json::from_str::<serde_json::Value>(body.strip_suffix('|').unwrap()).unwrap();
    }
    for doc_type in [
        "FOOTPRINT",
        "SYMBOL",
        "DEVICE",
        "SCH",
        "SCH_PAGE",
        "PCB",
        "CONFIG",
        "FONT",
    ] {
        assert!(
            first
                .record_stream
                .contains(&format!("\"docType\":\"{doc_type}\""))
        );
    }
    for record_type in ["PAD_NET", "TRACK", "VIA", "COPPERAREA"] {
        assert!(
            first
                .record_stream
                .contains(&format!("\"type\":\"{record_type}\""))
        );
    }
    assert!(first.record_stream.contains("\"hypercircuitFill\""));
    assert!(first.record_stream.contains("\"hypercircuitConnection\""));
    assert!(first.record_stream.contains("\"hypercircuitIslands\""));
    assert!(first.record_stream.contains("\"hypercircuitVia\""));
    assert!(first.record_stream.contains("\"hypercircuitZone\""));
    assert!(first.record_stream.contains("\"hypercircuitLabel\""));
    assert!(first.record_stream.contains("\"relativeAngle\":30"));
    assert!(first.numeric_projections.iter().any(|projection| {
        projection.emitted_unit == "mil" && projection.emitted == "39.370079"
    }));
    assert!(
        first
            .omissions
            .contains(&LcedaExportOmission::DetailedStackup)
    );
    assert!(
        first
            .omissions
            .contains(&LcedaExportOmission::EditorConformanceReview)
    );
    assert!(first.omissions.contains(&LcedaExportOmission::Keepouts(1)));
    assert!(first.omissions.iter().any(|omission| matches!(
        omission,
        LcedaExportOmission::ZonePolicyExtension { zone } if zone == "signal-zone"
    )));
    assert!(first.omissions.iter().any(|omission| matches!(
        omission,
        LcedaExportOmission::RoutedSlotApproximation { land_pattern, pad }
            if land_pattern == "R_0603" && pad == "1"
    )));
}

#[test]
fn rejects_zero_precision() {
    let (circuit, _, _) = fixture();
    let error = LcedaProExportReport::from_design(
        &circuit,
        None,
        None,
        LcedaProExportOptions {
            decimal_places: 0,
            ..LcedaProExportOptions::millimeters()
        },
    )
    .unwrap_err();
    assert_eq!(error, LcedaExportError::InvalidOptions);
}

#[test]
fn supported_pcb_subset_imports_editor_changes_and_reexports_stably() {
    let (circuit, schematic, layout) = fixture();
    let exported = LcedaProExportReport::from_design(
        &circuit,
        Some(&schematic),
        Some(&layout),
        LcedaProExportOptions::millimeters(),
    )
    .unwrap();
    let imported = LcedaProImportReport::from_archive(
        &circuit,
        &layout,
        &exported.archive,
        LcedaSourceLengthUnit::Millimeter,
    )
    .unwrap();
    assert!(imported.layout.validate(&circuit).is_valid());
    assert_eq!(imported.placements, 1);
    assert_eq!(imported.route_segments, 1);
    assert_eq!(imported.vias, 1);
    assert_eq!(imported.zones, 1);
    assert_eq!(imported.outline_segments, 8);
    assert!(!imported.numeric_imports.is_empty());
    assert!(
        imported
            .omissions
            .contains(&LcedaImportOmission::BaselineCircuitTruth)
    );
    assert!(
        imported
            .omissions
            .contains(&LcedaImportOmission::BaselinePhysicalDefinitions)
    );
    let connectivity_edit = replace_archive_token(
        exported.archive.clone(),
        b"\"locked\":false,\"netName\":\"SIGNAL\",\"startX\"",
        b"\"locked\":false,\"netName\":\"OTHER_\",\"startX\"",
    );
    assert!(matches!(
        LcedaProImportReport::from_archive(
            &circuit,
            &layout,
            &connectivity_edit,
            LcedaSourceLengthUnit::Millimeter,
        ),
        Err(LcedaImportError::InvalidField(field)) if field == "route.signal-route.net"
    ));
    let missing_pcb = replace_archive_token(
        exported.archive.clone(),
        b"\"docType\":\"PCB\"",
        b"\"docType\":\"PCX\"",
    );
    assert_eq!(
        LcedaProImportReport::from_archive(
            &circuit,
            &layout,
            &missing_pcb,
            LcedaSourceLengthUnit::Millimeter,
        ),
        Err(LcedaImportError::InvalidResult)
    );
    let duplicate_outline = replace_archive_token(
        exported.archive.clone(),
        b"\"hypercircuitContour\":\"board.exterior\",\"hypercircuitSegment\":1",
        b"\"hypercircuitContour\":\"board.exterior\",\"hypercircuitSegment\":0",
    );
    assert!(matches!(
        LcedaProImportReport::from_archive(
            &circuit,
            &layout,
            &duplicate_outline,
            LcedaSourceLengthUnit::Millimeter,
        ),
        Err(LcedaImportError::InvalidField(field)) if field == "outline.board.exterior[0]"
    ));

    let edited_archive = replace_archive_token(
        exported.archive.clone(),
        b"\"x\":196.850394",
        b"\"x\":236.220472",
    );
    let edited = LcedaProImportReport::from_archive(
        &circuit,
        &layout,
        &edited_archive,
        LcedaSourceLengthUnit::Millimeter,
    )
    .unwrap();
    assert_eq!(
        edited.layout.placements[0].position.x,
        (Real::from_str("236.220472").unwrap() * Real::from(127) / Real::from(5000)).unwrap()
    );
    assert_ne!(
        edited.layout.placements[0].position.x,
        layout.placements[0].position.x
    );

    let reexported = LcedaProExportReport::from_design(
        &circuit,
        Some(&schematic),
        Some(&edited.layout),
        LcedaProExportOptions::millimeters(),
    )
    .unwrap();
    assert!(reexported.record_stream.contains("\"x\":236.220472"));
    let replay = LcedaProImportReport::from_archive(
        &circuit,
        &layout,
        &reexported.archive,
        LcedaSourceLengthUnit::Millimeter,
    )
    .unwrap();
    assert_eq!(replay.layout, edited.layout);
    assert_eq!(
        LcedaProImportReport::from_archive(
            &circuit,
            &layout,
            b"not a zip",
            LcedaSourceLengthUnit::Millimeter,
        ),
        Err(LcedaImportError::InvalidArchive)
    );
}

#[test]
fn supported_schematic_presentation_imports_editor_changes_without_connectivity_changes() {
    let (circuit, schematic, layout) = fixture();
    let exported = LcedaProExportReport::from_design(
        &circuit,
        Some(&schematic),
        Some(&layout),
        LcedaProExportOptions::millimeters(),
    )
    .unwrap();
    let imported =
        LcedaSchematicImportReport::from_archive(&circuit, &schematic, &exported.archive).unwrap();
    assert_eq!(imported.schematic, schematic);
    assert_eq!(imported.symbols, 1);
    assert_eq!(imported.wires, 1);
    assert_eq!(imported.labels, 1);
    assert!(!imported.numeric_imports.is_empty());
    assert!(
        imported
            .omissions
            .contains(&LcedaImportOmission::BaselineSchematicTopology)
    );
    assert!(imported.omissions.iter().any(|omission| matches!(
        omission,
        LcedaImportOmission::AnchoredWireEndpointPreserved { wire, endpoint }
            if wire == "signal-wire" && *endpoint == "from"
    )));
    let connectivity_edit = replace_archive_token(
        exported.archive.clone(),
        b"\"net\":\"SIGNAL\",\"text\":\"SIGNAL\"",
        b"\"net\":\"OTHER_\",\"text\":\"SIGNAL\"",
    );
    assert!(matches!(
        LcedaSchematicImportReport::from_archive(&circuit, &schematic, &connectivity_edit),
        Err(LcedaImportError::InvalidField(field)) if field == "label.signal-label.net"
    ));
    let missing_page = replace_archive_token(
        exported.archive.clone(),
        b"\"docType\":\"SCH_PAGE\"",
        b"\"docType\":\"SCH_PAGX\"",
    );
    assert_eq!(
        LcedaSchematicImportReport::from_archive(&circuit, &schematic, &missing_page),
        Err(LcedaImportError::InvalidResult)
    );

    let edited_archive = replace_archive_token(exported.archive.clone(), b"\"x\":3", b"\"x\":4");
    let edited_archive =
        replace_archive_token(edited_archive, b"\"rotation\":90", b"\"rotation\": 0");
    let edited_archive = replace_archive_token(
        edited_archive,
        b"\"text\":\"SIGNAL\"",
        b"\"text\":\"EDITED\"",
    );
    let edited_archive = replace_archive_token(
        edited_archive,
        b"\"endX\":10,\"endY\":0",
        b"\"endX\":11,\"endY\":0",
    );
    let edited_archive = replace_archive_token(
        edited_archive,
        b"\"startX\":10,\"startY\":0",
        b"\"startX\":11,\"startY\":0",
    );
    let edited =
        LcedaSchematicImportReport::from_archive(&circuit, &schematic, &edited_archive).unwrap();
    assert_eq!(edited.schematic.symbols[0].position.x, Real::from(4));
    assert_eq!(edited.schematic.symbols[0].quarter_turns, 0);
    assert_eq!(edited.schematic.labels[0].text, "EDITED");
    assert_eq!(
        edited.schematic.wires[0].waypoints,
        vec![SchematicPoint::new(Real::from(11), Real::zero())]
    );
    assert_eq!(edited.schematic.wires[0].net, schematic.wires[0].net);
    assert_eq!(edited.schematic.wires[0].from, schematic.wires[0].from);
    assert!(edited.schematic.validate(&circuit).is_valid());

    let reexported = LcedaProExportReport::from_design(
        &circuit,
        Some(&edited.schematic),
        Some(&layout),
        LcedaProExportOptions::millimeters(),
    )
    .unwrap();
    let replay =
        LcedaSchematicImportReport::from_archive(&circuit, &schematic, &reexported.archive)
            .unwrap();
    assert_eq!(replay.schematic, edited.schematic);
}

fn stored_zip_files(bytes: &[u8]) -> BTreeMap<String, Vec<u8>> {
    let mut files = BTreeMap::new();
    let mut offset = 0;
    while bytes.get(offset..offset + 4) == Some(&0x0403_4b50_u32.to_le_bytes()) {
        let size = u32::from_le_bytes(bytes[offset + 18..offset + 22].try_into().unwrap()) as usize;
        let name_len =
            u16::from_le_bytes(bytes[offset + 26..offset + 28].try_into().unwrap()) as usize;
        let extra_len =
            u16::from_le_bytes(bytes[offset + 28..offset + 30].try_into().unwrap()) as usize;
        let name_start = offset + 30;
        let content_start = name_start + name_len + extra_len;
        let name = String::from_utf8(bytes[name_start..name_start + name_len].to_vec()).unwrap();
        files.insert(name, bytes[content_start..content_start + size].to_vec());
        offset = content_start + size;
    }
    files
}

fn replace_archive_token(mut bytes: Vec<u8>, source: &[u8], replacement: &[u8]) -> Vec<u8> {
    assert_eq!(source.len(), replacement.len());
    let matches = bytes
        .windows(source.len())
        .enumerate()
        .filter_map(|(index, value)| (value == source).then_some(index))
        .collect::<Vec<_>>();
    assert_eq!(matches.len(), 1);
    bytes[matches[0]..matches[0] + source.len()].copy_from_slice(replacement);
    let edited = matches[0];
    let mut offset = 0;
    while bytes.get(offset..offset + 4) == Some(&0x0403_4b50_u32.to_le_bytes()) {
        let size = u32::from_le_bytes(bytes[offset + 18..offset + 22].try_into().unwrap()) as usize;
        let name_len =
            u16::from_le_bytes(bytes[offset + 26..offset + 28].try_into().unwrap()) as usize;
        let extra_len =
            u16::from_le_bytes(bytes[offset + 28..offset + 30].try_into().unwrap()) as usize;
        let content_start = offset + 30 + name_len + extra_len;
        let content_end = content_start + size;
        if edited >= content_start && edited < content_end {
            let crc = test_crc32(&bytes[content_start..content_end]);
            bytes[offset + 14..offset + 18].copy_from_slice(&crc.to_le_bytes());
            return bytes;
        }
        offset = content_end;
    }
    panic!("edited token was not inside a stored ZIP member");
}

fn test_crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xffff_ffff_u32;
    for byte in bytes {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320_u32 & (0_u32.wrapping_sub(crc & 1)));
        }
    }
    !crc
}
