use hypercircuit::{
    AdapterKind, Circuit, CircuitId, CircuitInstance, CircuitInstanceId, CircuitPort, ComponentId,
    DeviceModel, DeviceModelId, DeviceModelKind, DevicePin, Net, NetId, PinBinding,
    PinElectricalKind, PinRef, PortDirection, PortId, Real, SchematicAutoLayoutError,
    SchematicAutoLayoutPolicy, SchematicEndpoint, SchematicPinSide, TransientPolicy,
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

fn chain_circuit() -> Circuit {
    let ground = NetId::new("GND").unwrap();
    let input = NetId::new("INPUT").unwrap();
    let output = NetId::new("OUTPUT").unwrap();
    Circuit::new(
        CircuitId::new("auto-schematic").unwrap(),
        TransientPolicy::Static,
        AdapterKind::Dc,
    )
    .with_net(Net {
        id: ground.clone(),
        is_ground: true,
    })
    .with_net(Net {
        id: input.clone(),
        is_ground: false,
    })
    .with_net(Net {
        id: output.clone(),
        is_ground: false,
    })
    .with_port(CircuitPort {
        id: PortId::new("input-port").unwrap(),
        net: input.clone(),
        direction: PortDirection::Input,
        optional: false,
    })
    .with_port(CircuitPort {
        id: PortId::new("output-port").unwrap(),
        net: output.clone(),
        direction: PortDirection::Output,
        optional: false,
    })
    .with_device_model(DeviceModel {
        id: DeviceModelId::new("source").unwrap(),
        kind: DeviceModelKind::VoltageSource,
        pins: vec![
            pin("pos", PinElectricalKind::PowerOutput),
            pin("neg", PinElectricalKind::PowerOutput),
        ],
        parameters: Vec::new(),
    })
    .with_device_model(DeviceModel {
        id: DeviceModelId::new("passive").unwrap(),
        kind: DeviceModelKind::Resistor,
        pins: vec![
            pin("1", PinElectricalKind::Passive),
            pin("2", PinElectricalKind::Passive),
        ],
        parameters: Vec::new(),
    })
    .with_device_model(DeviceModel {
        id: DeviceModelId::new("load").unwrap(),
        kind: DeviceModelKind::Custom("load".into()),
        pins: vec![
            pin("in", PinElectricalKind::Input),
            pin("gnd", PinElectricalKind::PowerInput),
        ],
        parameters: Vec::new(),
    })
    .with_instance(instance(
        "V1",
        "source",
        &[("pos", &input), ("neg", &ground)],
    ))
    .with_instance(instance("R1", "passive", &[("1", &input), ("2", &output)]))
    .with_instance(instance("U1", "load", &[("in", &output), ("gnd", &ground)]))
}

#[test]
fn connectivity_depth_places_a_chain_without_ground_collapsing_columns() {
    let circuit = chain_circuit();
    let report = circuit
        .auto_schematic(SchematicAutoLayoutPolicy::default())
        .unwrap();

    assert!(report.layout.validate(&circuit).is_valid());
    assert_eq!(report.connectivity_edges, 2);
    assert_eq!(
        report
            .placements
            .iter()
            .map(|placement| (placement.instance.as_str(), placement.column, placement.row,))
            .collect::<Vec<_>>(),
        vec![("V1", 0, 0), ("R1", 1, 0), ("U1", 2, 0)]
    );
    assert_eq!(report.layout.symbols.len(), 3);
    assert_eq!(report.layout.ports.len(), 2);
    assert_eq!(report.generated_wires, 5);
    assert_eq!(report.generated_labels, 3);
    assert!(
        report
            .layout
            .labels
            .iter()
            .all(|label| !label.text.is_empty())
    );

    let source = report
        .layout
        .symbols
        .iter()
        .find(|symbol| symbol.instance.as_str() == "V1")
        .unwrap();
    assert_eq!(
        report
            .layout
            .symbol_unit(source)
            .unwrap()
            .pins
            .iter()
            .find(|pin| pin.pin.as_str() == "neg")
            .unwrap()
            .side,
        SchematicPinSide::Bottom
    );
    assert!(report.layout.wires.iter().all(|wire| {
        matches!(
            wire.from,
            SchematicEndpoint::Pin { .. } | SchematicEndpoint::Port(_)
        ) && matches!(
            wire.to,
            SchematicEndpoint::Pin { .. } | SchematicEndpoint::Port(_)
        )
    }));
}

#[test]
fn automatic_layout_policy_and_work_bounds_fail_typed() {
    let circuit = chain_circuit();
    let invalid = SchematicAutoLayoutPolicy {
        column_spacing: Real::zero(),
        ..SchematicAutoLayoutPolicy::default()
    };
    assert_eq!(
        circuit.auto_schematic(invalid),
        Err(SchematicAutoLayoutError::InvalidPolicy)
    );

    let bounded = SchematicAutoLayoutPolicy {
        max_instances: 2,
        ..SchematicAutoLayoutPolicy::default()
    };
    assert_eq!(
        circuit.auto_schematic(bounded),
        Err(SchematicAutoLayoutError::InstanceLimit {
            instances: 3,
            limit: 2,
        })
    );
}

#[cfg(feature = "layout")]
#[test]
fn checked_fluent_design_can_replace_an_empty_drawing_and_render_it() {
    use hypercircuit::{BoardOutline, Design, PcbStackup, SchematicSvgOptions, parts};

    let mut design = Design::new(
        "fluent-auto-schematic",
        BoardOutline::rectangle(Real::from(30), Real::from(20)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let input = design.signal("INPUT").unwrap();
    let output = design.signal("OUTPUT").unwrap();
    let ground = design.ground("GND").unwrap();
    let source = design
        .add(parts::voltage_source("V1", Real::from(5)))
        .unwrap();
    let resistor = design
        .add(parts::resistor("R1", Real::from(1_000)))
        .unwrap();
    let load = design
        .add(parts::resistor("R2", Real::from(1_000)))
        .unwrap();
    design
        .connect(
            &input,
            [source.pin("pos").unwrap(), resistor.pin("1").unwrap()],
        )
        .unwrap();
    design
        .connect(
            &output,
            [resistor.pin("2").unwrap(), load.pin("1").unwrap()],
        )
        .unwrap();
    design
        .connect(
            &ground,
            [source.pin("neg").unwrap(), load.pin("2").unwrap()],
        )
        .unwrap();
    let mut checked = design.finish().unwrap();
    assert!(checked.schematic.symbols.is_empty());
    let original_circuit = checked.circuit.clone();
    let original_layout = checked.layout.clone();
    let original_sources = checked.source_map.clone();

    let report = checked
        .replace_schematic_with_auto_layout(SchematicAutoLayoutPolicy::default())
        .unwrap();
    let svg = checked
        .schematic
        .to_svg(&checked.circuit, SchematicSvgOptions::default())
        .unwrap();

    assert_eq!(checked.circuit, original_circuit);
    assert_eq!(checked.layout, original_layout);
    assert_eq!(checked.source_map, original_sources);
    assert_eq!(checked.schematic, report.layout);
    assert!(svg.svg.contains(">V1</text>"));
    assert!(svg.svg.contains(">OUTPUT</text>"));
}
