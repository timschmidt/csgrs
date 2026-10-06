use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, ComponentId, DeviceModel,
    DeviceModelId, DeviceModelKind, DevicePin, Net, NetId, PinBinding, PinElectricalKind, PinRef,
    Real, SchematicEndpoint, SchematicGraphic, SchematicGraphicFill, SchematicLabel,
    SchematicLabelId, SchematicLayout, SchematicPinPlacement, SchematicPinSide, SchematicPoint,
    SchematicSheet, SchematicSheetId, SchematicSheetLink, SchematicSheetLinkId, SchematicSheetPort,
    SchematicSheetPortId, SchematicSvgOptions, SchematicSymbol, SchematicSymbolDefinition,
    SchematicSymbolDefinitionId, SchematicSymbolId, SchematicSymbolUnit, SchematicValidationIssue,
    SchematicWire, SchematicWireId, TransientPolicy,
};

fn point(x: i64, y: i64) -> SchematicPoint {
    SchematicPoint::new(Real::from(x), Real::from(y))
}

fn fixture() -> (Circuit, SchematicLayout, NetId) {
    let signal = NetId::new("SIGNAL").unwrap();
    let other = NetId::new("OTHER").unwrap();
    let output_model = DeviceModelId::new("driver").unwrap();
    let input_model = DeviceModelId::new("receiver").unwrap();
    let pin = PinRef::new("1").unwrap();
    let model = |id: DeviceModelId, kind| DeviceModel {
        id,
        kind: DeviceModelKind::Custom("logic".into()),
        pins: vec![DevicePin {
            pin: pin.clone(),
            kind,
            optional: false,
        }],
        parameters: Vec::new(),
    };
    let instance = |id: &str, model: DeviceModelId| CircuitInstance {
        id: CircuitInstanceId::new(id).unwrap(),
        component: ComponentId::new(id).unwrap(),
        part: None,
        model,
        pins: vec![PinBinding {
            pin: pin.clone(),
            net: signal.clone(),
        }],
        parameters: Vec::new(),
    };
    let circuit = Circuit::new(
        CircuitId::new("schematic").unwrap(),
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
    .with_device_model(model(output_model.clone(), PinElectricalKind::Output))
    .with_device_model(model(input_model.clone(), PinElectricalKind::Input))
    .with_instance(instance("U1", output_model))
    .with_instance(instance("U2", input_model));

    let pin_placement = |side| SchematicPinPlacement {
        pin: pin.clone(),
        position: match side {
            SchematicPinSide::Right => point(10, 0),
            _ => point(-10, 0),
        },
        side,
    };
    let left = SchematicSymbolId::new("U1:A").unwrap();
    let right = SchematicSymbolId::new("U2:A").unwrap();
    let left_definition = SchematicSymbolDefinitionId::new("driver-symbol").unwrap();
    let right_definition = SchematicSymbolDefinitionId::new("receiver-symbol").unwrap();
    let definition = |id, model, side| SchematicSymbolDefinition {
        id,
        model,
        name: "logic".into(),
        units: vec![SchematicSymbolUnit {
            unit: 1,
            body_width: Real::from(20),
            body_height: Real::from(14),
            pins: vec![pin_placement(side)],
            graphics: vec![SchematicGraphic::Rectangle {
                start: point(-10, -7),
                end: point(10, 7),
                stroke_width: Real::one(),
                fill: SchematicGraphicFill::Background,
            }],
        }],
    };
    let layout = SchematicLayout {
        symbol_definitions: vec![
            definition(
                left_definition.clone(),
                DeviceModelId::new("driver").unwrap(),
                SchematicPinSide::Right,
            ),
            definition(
                right_definition.clone(),
                DeviceModelId::new("receiver").unwrap(),
                SchematicPinSide::Left,
            ),
        ],
        symbols: vec![
            SchematicSymbol {
                id: left.clone(),
                instance: CircuitInstanceId::new("U1").unwrap(),
                definition: left_definition,
                unit: 1,
                position: point(0, 0),
                quarter_turns: 0,
            },
            SchematicSymbol {
                id: right.clone(),
                instance: CircuitInstanceId::new("U2").unwrap(),
                definition: right_definition,
                unit: 1,
                position: point(50, 0),
                quarter_turns: 0,
            },
        ],
        ports: Vec::new(),
        wires: vec![SchematicWire {
            id: SchematicWireId::new("signal-wire").unwrap(),
            net: signal.clone(),
            from: SchematicEndpoint::Pin {
                symbol: left,
                pin: pin.clone(),
            },
            waypoints: vec![point(25, 0)],
            to: SchematicEndpoint::Pin { symbol: right, pin },
        }],
        labels: vec![SchematicLabel {
            id: SchematicLabelId::new("signal-label").unwrap(),
            net: signal.clone(),
            position: point(25, -3),
            text: "SIGNAL".into(),
        }],
        sheets: Vec::new(),
        sheet_ports: Vec::new(),
        sheet_links: Vec::new(),
    };
    (circuit, layout, signal)
}

fn hierarchical_fixture() -> (Circuit, SchematicLayout) {
    let (circuit, mut layout, signal) = fixture();
    let root = SchematicSheetId::new("root").unwrap();
    let child = SchematicSheetId::new("receiver-sheet").unwrap();
    let root_port = SchematicSheetPortId::new("root-signal").unwrap();
    let child_port = SchematicSheetPortId::new("receiver-signal").unwrap();
    let original = layout.wires.remove(0);
    let root_wire = SchematicWireId::new("root-signal-wire").unwrap();
    let child_wire = SchematicWireId::new("receiver-signal-wire").unwrap();
    layout.wires = vec![
        SchematicWire {
            id: root_wire.clone(),
            net: signal.clone(),
            from: original.from,
            waypoints: Vec::new(),
            to: SchematicEndpoint::SheetPort(root_port.clone()),
        },
        SchematicWire {
            id: child_wire.clone(),
            net: signal.clone(),
            from: SchematicEndpoint::SheetPort(child_port.clone()),
            waypoints: Vec::new(),
            to: original.to,
        },
    ];
    layout.sheets = vec![
        SchematicSheet {
            id: root.clone(),
            title: "Driver".into(),
            parent: None,
            symbols: vec![SchematicSymbolId::new("U1:A").unwrap()],
            ports: Vec::new(),
            wires: vec![root_wire],
            labels: vec![SchematicLabelId::new("signal-label").unwrap()],
        },
        SchematicSheet {
            id: child.clone(),
            title: "Receiver".into(),
            parent: Some(root.clone()),
            symbols: vec![SchematicSymbolId::new("U2:A").unwrap()],
            ports: Vec::new(),
            wires: vec![child_wire],
            labels: Vec::new(),
        },
    ];
    layout.sheet_ports = vec![
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
            position: point(30, 0),
        },
    ];
    layout.sheet_links = vec![SchematicSheetLink {
        id: SchematicSheetLinkId::new("driver-to-receiver").unwrap(),
        parent_port: root_port,
        child_port,
    }];
    (circuit, layout)
}

#[test]
fn schematic_validates_typed_endpoints_and_renders_audited_svg() {
    let (circuit, layout, _) = fixture();
    assert!(layout.validate(&circuit).is_valid());

    let svg = layout
        .to_svg(&circuit, SchematicSvgOptions::default())
        .unwrap();
    assert!(svg.svg.starts_with("<svg"));
    assert!(svg.svg.contains("<polyline"));
    assert!(svg.svg.contains("U1"));
    assert!(!svg.projections.is_empty());
}

#[test]
fn schematic_rejects_wire_net_that_disagrees_with_pin_bindings() {
    let (circuit, mut layout, _) = fixture();
    layout.wires[0].net = NetId::new("OTHER").unwrap();
    let report = layout.validate(&circuit);
    assert!(report.issues.iter().any(|issue| matches!(
        issue,
        SchematicValidationIssue::WireEndpointNetMismatch { .. }
    )));
}

#[test]
fn hierarchical_schematic_validates_renders_pages_and_round_trips() {
    let (circuit, layout) = hierarchical_fixture();
    assert!(layout.validate(&circuit).is_valid());

    let book = layout
        .to_svg_book(&circuit, SchematicSvgOptions::default())
        .unwrap();
    assert_eq!(book.pages.len(), 2);
    assert_eq!(book.pages[0].title, "Driver");
    assert_eq!(book.pages[1].title, "Receiver");
    assert!(book.pages.iter().all(|page| {
        page.report.svg.contains("data-sheet-port") && !page.report.projections.is_empty()
    }));

    #[cfg(feature = "interchange")]
    {
        let document = hypercircuit::SemanticDocument::new(circuit, Some(layout)).unwrap();
        let restored =
            hypercircuit::SemanticDocument::from_json(&document.to_json_pretty().unwrap()).unwrap();
        assert_eq!(restored, document);
    }
}

#[test]
fn hierarchical_schematic_rejects_cycles_and_boundary_net_disagreement() {
    let (circuit, mut layout) = hierarchical_fixture();
    layout.sheet_ports[1].net = NetId::new("OTHER").unwrap();
    layout.sheets[0].parent = Some(SchematicSheetId::new("receiver-sheet").unwrap());
    let report = layout.validate(&circuit);

    assert!(
        report
            .issues
            .iter()
            .any(|issue| matches!(issue, SchematicValidationIssue::SheetHierarchyCycle(_)))
    );
    assert!(
        report
            .issues
            .iter()
            .any(|issue| matches!(issue, SchematicValidationIssue::SheetLinkNetMismatch(_)))
    );
}
